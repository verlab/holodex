#include "motor_controller.hpp"
#include <cmath>
#include <algorithm>
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

static const char* TAG = "StepDirMotorController";

namespace holodex {

StepDirMotorController::StepDirMotorController(float steps_per_rev, float max_speed_rad_s, float max_accel_rad_s2)
    : steps_per_rev_(steps_per_rev),
      max_speed_rad_s_(max_speed_rad_s),
      max_accel_rad_s2_(max_accel_rad_s2),
      is_enabled_(false),
      all_stopped_since_us_(0),
      pcnt_initialized_(false)
{
    ledc_timers_[0] = LEDC_TIMER_0;
    ledc_timers_[1] = LEDC_TIMER_1;
    ledc_timers_[2] = LEDC_TIMER_2;
    ledc_timers_[3] = LEDC_TIMER_3;

    ledc_channels_[0] = LEDC_CHANNEL_0;
    ledc_channels_[1] = LEDC_CHANNEL_1;
    ledc_channels_[2] = LEDC_CHANNEL_2;
    ledc_channels_[3] = LEDC_CHANNEL_3;

    for (int i = 0; i < 4; ++i) {
        pcnt_units_[i] = nullptr;
        pcnt_chans_[i] = nullptr;
        last_pcnt_count_[i] = 0;
        current_position_steps_[i] = 0;
        last_dir_[i] = true;
        last_freq_hz_[i] = 0;
        pulsing_[i] = false;
        no_pulse_cycles_[i] = 0;
    }
}

StepDirMotorController::~StepDirMotorController()
{
    emergencyStop();
    for (int i = 0; i < 4; ++i) {
        if (pcnt_units_[i] != nullptr) {
            pcnt_unit_stop(pcnt_units_[i]);
            pcnt_unit_disable(pcnt_units_[i]);
            if (pcnt_chans_[i] != nullptr) {
                pcnt_del_channel(pcnt_chans_[i]);
            }
            pcnt_del_unit(pcnt_units_[i]);
        }
    }
}

bool StepDirMotorController::init()
{
    ESP_LOGI(TAG, "Configurando hardware de motores: LEDC (pulsos) + PCNT (contador) + GPIO (Dir/Ena)...");

    const gpio_num_t step_pins[4] = { PIN_STEP_FL, PIN_STEP_FR, PIN_STEP_RL, PIN_STEP_RR };
    const gpio_num_t dir_pins[4]  = { PIN_DIR_FL, PIN_DIR_FR, PIN_DIR_RL, PIN_DIR_RR };
    const gpio_num_t ena_pins[4]  = { PIN_ENA_FL, PIN_ENA_FR, PIN_ENA_RL, PIN_ENA_RR };

    // 1. Configura pinos DIR e ENA como saída digital
    uint64_t out_mask = 0;
    for (int i = 0; i < 4; ++i) {
        out_mask |= (1ULL << dir_pins[i]);
        out_mask |= (1ULL << ena_pins[i]);
    }
    gpio_config_t io_conf = {
        .pin_bit_mask = out_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar GPIOs de Dir/Ena: %s", esp_err_to_name(err));
        return false;
    }

    // 2. Configura LEDC para os 4 pinos de STEP
    for (int i = 0; i < 4; ++i) {
        ledc_timer_config_t timer_cfg = {};
        timer_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
        timer_cfg.duty_resolution = LEDC_TIMER_10_BIT;
        timer_cfg.timer_num = ledc_timers_[i];
        timer_cfg.freq_hz = 1000;
        timer_cfg.clk_cfg = LEDC_AUTO_CLK;
        err = ledc_timer_config(&timer_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Falha timer LEDC canal %d: %s", i, esp_err_to_name(err));
            return false;
        }

        ledc_channel_config_t chan_cfg = {};
        chan_cfg.gpio_num = step_pins[i];
        chan_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
        chan_cfg.channel = ledc_channels_[i];
        chan_cfg.intr_type = LEDC_INTR_DISABLE;
        chan_cfg.timer_sel = ledc_timers_[i];
        chan_cfg.duty = 0; // Inicia parado
        chan_cfg.hpoint = 0;
        err = ledc_channel_config(&chan_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Falha canal LEDC %d: %s", i, esp_err_to_name(err));
            return false;
        }

        // Habilita entrada nos pinos STEP e DIR para permitir loopback interno no PCNT
        gpio_set_direction(step_pins[i], GPIO_MODE_INPUT_OUTPUT);
        gpio_set_direction(dir_pins[i], GPIO_MODE_INPUT_OUTPUT);
    }

    // 3. Configura PCNT para monitorar pulsos de hardware nos pinos STEP/DIR
    for (int i = 0; i < 4; ++i) {
        pcnt_unit_config_t unit_cfg = {};
        unit_cfg.low_limit = -32768;
        unit_cfg.high_limit = 32767;
        err = pcnt_new_unit(&unit_cfg, &pcnt_units_[i]);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "PCNT unit %d falhou (%s), contagem por software ativa", i, esp_err_to_name(err));
            pcnt_units_[i] = nullptr;
            continue;
        }

        pcnt_chan_config_t chan_cfg = {};
        chan_cfg.edge_gpio_num = step_pins[i];
        chan_cfg.level_gpio_num = dir_pins[i];
        err = pcnt_new_channel(pcnt_units_[i], &chan_cfg, &pcnt_chans_[i]);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "PCNT chan %d falhou: %s", i, esp_err_to_name(err));
            pcnt_del_unit(pcnt_units_[i]);
            pcnt_units_[i] = nullptr;
            continue;
        }

        pcnt_channel_set_edge_action(pcnt_chans_[i], PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
        pcnt_channel_set_level_action(pcnt_chans_[i], PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

        pcnt_unit_enable(pcnt_units_[i]);
        pcnt_unit_clear_count(pcnt_units_[i]);
        pcnt_unit_start(pcnt_units_[i]);
    }
    pcnt_initialized_ = true;

    // Inicializa saídas desativadas (ENA=1)
    setEnabled(false);
    emergencyStop();

    ESP_LOGI(TAG, "Controlador Step/Dir pronto: 4 canais LEDC com pulsos reais e contadores PCNT.");
    return true;
}

void StepDirMotorController::applyChannelHardware(int channel, float speed_rad_s)
{
    gpio_num_t dir_pin = GPIO_NUM_NC;
    gpio_num_t ena_pin = GPIO_NUM_NC;
    ledc_timer_t timer = ledc_timers_[channel];
    ledc_channel_t chan = ledc_channels_[channel];

    switch (channel) {
        case 0: dir_pin = PIN_DIR_FL; ena_pin = PIN_ENA_FL; break;
        case 1: dir_pin = PIN_DIR_FR; ena_pin = PIN_ENA_FR; break;
        case 2: dir_pin = PIN_DIR_RL; ena_pin = PIN_ENA_RL; break;
        case 3: dir_pin = PIN_DIR_RR; ena_pin = PIN_ENA_RR; break;
        default: return;
    }

    if (!is_enabled_) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, chan, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, chan);
        pulsing_[channel] = false;
        gpio_set_level(ena_pin, 1); // Desabilitado
        return;
    }

    const float rad_to_steps = steps_per_rev_ / (2.0f * (float)M_PI);
    float speed_steps = speed_rad_s * rad_to_steps;
    uint32_t freq_hz = (uint32_t)std::abs(speed_steps);

    if (freq_hz < 10) {
        // Velocidade nula: para o PWM (linha em nível baixo estável)
        if (pulsing_[channel]) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, chan, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, chan);
            pulsing_[channel] = false;
        }
    } else {
        // Habilita driver (Active-LOW: 0 = ligado)
        gpio_set_level(ena_pin, 0);

        // Sentido de rotação
        bool forward = (speed_rad_s >= 0.0f);
        if (forward != last_dir_[channel]) {
            gpio_set_level(dir_pin, forward ? 1 : 0);
            ets_delay_us(30); // Setup time do driver (TB6600 / A4988)
            last_dir_[channel] = forward;
        }

        // Limita frequência máxima para prevenir perda de passo
        if (freq_hz > 30000) {
            freq_hz = 30000;
        }

        // Só reconfigura o timer quando a frequência muda (evita glitch nos pulsos a cada ciclo)
        if (freq_hz != last_freq_hz_[channel]) {
            esp_err_t ferr = ledc_set_freq(LEDC_LOW_SPEED_MODE, timer, freq_hz);
            last_freq_hz_[channel] = freq_hz;
            if (ferr != ESP_OK) {
                // Frequência fora da faixa do divisor do LEDC (10 bits): não deixa o motor
                // rodando na frequência anterior.
                ESP_LOGW(TAG, "Canal %d: ledc_set_freq(%u Hz) falhou (%s). Pulsos parados.",
                         channel, (unsigned)freq_hz, esp_err_to_name(ferr));
                ledc_set_duty(LEDC_LOW_SPEED_MODE, chan, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, chan);
                pulsing_[channel] = false;
                return;
            }
        }

        if (!pulsing_[channel]) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, chan, 512); // 50% de ciclo ativo (onda quadrada)
            ledc_update_duty(LEDC_LOW_SPEED_MODE, chan);
            pulsing_[channel] = true;
        }
    }
}

void StepDirMotorController::updateStepCounters() const
{
    const int32_t commanded[4] = {
        status_.fl.current_speed_steps_s, status_.fr.current_speed_steps_s,
        status_.rl.current_speed_steps_s, status_.rr.current_speed_steps_s
    };

    for (int i = 0; i < 4; ++i) {
        int delta = 0;
        if (pcnt_initialized_ && pcnt_units_[i] != nullptr) {
            int count = 0;
            pcnt_unit_get_count(pcnt_units_[i], &count);
            delta = count - last_pcnt_count_[i];
            last_pcnt_count_[i] = count;

            // Se o contador de 16 bits estiver perto dos limites, limpa sem perder contagem
            if (count > 25000 || count < -25000) {
                pcnt_unit_clear_count(pcnt_units_[i]);
                last_pcnt_count_[i] = 0;
            }
        }

        // Os passos vêm SOMENTE do PCNT. Não estima a partir do comando: se o contador não
        // enxerga os pulsos, o problema precisa aparecer em vez de ser mascarado.
        if (commanded[i] != 0 && delta == 0) {
            if (++no_pulse_cycles_[i] == 50) { // ~1 s com comando ativo e nenhum pulso contado
                ESP_LOGW(TAG, "Canal %d: comando ativo mas PCNT nao contou pulsos (pino STEP sem entrada "
                              "ou LEDC parado?). Posicao em passos NAO e confiavel.", i);
            }
        } else {
            no_pulse_cycles_[i] = 0;
        }

        current_position_steps_[i] += delta;
    }

    status_.fl.current_position_steps = current_position_steps_[0];
    status_.fr.current_position_steps = current_position_steps_[1];
    status_.rl.current_position_steps = current_position_steps_[2];
    status_.rr.current_position_steps = current_position_steps_[3];
}

void StepDirMotorController::setWheelVelocities(const WheelVelocities& velocities)
{
    updateStepCounters();

    target_velocities_.fl = std::clamp(velocities.fl, -max_speed_rad_s_, max_speed_rad_s_);
    target_velocities_.fr = std::clamp(velocities.fr, -max_speed_rad_s_, max_speed_rad_s_);
    target_velocities_.rl = std::clamp(velocities.rl, -max_speed_rad_s_, max_speed_rad_s_);
    target_velocities_.rr = std::clamp(velocities.rr, -max_speed_rad_s_, max_speed_rad_s_);

    const float rad_to_steps = steps_per_rev_ / (2.0f * (float)M_PI);

    status_.fl.current_speed_steps_s = (int32_t)(target_velocities_.fl * rad_to_steps);
    status_.fr.current_speed_steps_s = (int32_t)(target_velocities_.fr * rad_to_steps);
    status_.rl.current_speed_steps_s = (int32_t)(target_velocities_.rl * rad_to_steps);
    status_.rr.current_speed_steps_s = (int32_t)(target_velocities_.rr * rad_to_steps);

    bool any_moving = (status_.fl.current_speed_steps_s != 0 ||
                       status_.fr.current_speed_steps_s != 0 ||
                       status_.rl.current_speed_steps_s != 0 ||
                       status_.rr.current_speed_steps_s != 0);

    status_.fl.is_running = (status_.fl.current_speed_steps_s != 0);
    status_.fr.is_running = (status_.fr.current_speed_steps_s != 0);
    status_.rl.is_running = (status_.rl.current_speed_steps_s != 0);
    status_.rr.is_running = (status_.rr.current_speed_steps_s != 0);

    int64_t now_us = esp_timer_get_time();

    if (any_moving) {
        all_stopped_since_us_ = 0;
        if (!is_enabled_) {
            setEnabled(true);
        }
    } else {
        if (all_stopped_since_us_ == 0) {
            all_stopped_since_us_ = now_us;
        } else if (now_us - all_stopped_since_us_ > 3000000ULL) { // 3 segundos parado
            // Desabilita saídas para economizar energia e não aquecer os motores
            if (is_enabled_) {
                setEnabled(false);
            }
        }
    }

    applyChannelHardware(0, target_velocities_.fl);
    applyChannelHardware(1, target_velocities_.fr);
    applyChannelHardware(2, target_velocities_.rl);
    applyChannelHardware(3, target_velocities_.rr);
}

void StepDirMotorController::getGroupStatus(StepperGroupStatus& out_status) const
{
    updateStepCounters();
    out_status = status_;
}

WheelVelocities StepDirMotorController::getEstimatedWheelVelocities() const
{
    const float steps_to_rad = (2.0f * (float)M_PI) / steps_per_rev_;
    return WheelVelocities(
        (float)status_.fl.current_speed_steps_s * steps_to_rad,
        (float)status_.fr.current_speed_steps_s * steps_to_rad,
        (float)status_.rl.current_speed_steps_s * steps_to_rad,
        (float)status_.rr.current_speed_steps_s * steps_to_rad
    );
}

void StepDirMotorController::setEnabled(bool enabled)
{
    is_enabled_ = enabled;
    status_.fl.is_enabled = enabled;
    status_.fr.is_enabled = enabled;
    status_.rl.is_enabled = enabled;
    status_.rr.is_enabled = enabled;

    gpio_set_level(PIN_ENA_FL, enabled ? 0 : 1);
    gpio_set_level(PIN_ENA_FR, enabled ? 0 : 1);
    gpio_set_level(PIN_ENA_RL, enabled ? 0 : 1);
    gpio_set_level(PIN_ENA_RR, enabled ? 0 : 1);

    if (!enabled) {
        for (int i = 0; i < 4; ++i) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, ledc_channels_[i], 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, ledc_channels_[i]);
            pulsing_[i] = false;
        }
    }
}

void StepDirMotorController::emergencyStop()
{
    target_velocities_ = WheelVelocities(0.0f, 0.0f, 0.0f, 0.0f);
    status_.fl.current_speed_steps_s = 0;
    status_.fr.current_speed_steps_s = 0;
    status_.rl.current_speed_steps_s = 0;
    status_.rr.current_speed_steps_s = 0;
    status_.fl.is_running = false;
    status_.fr.is_running = false;
    status_.rl.is_running = false;
    status_.rr.is_running = false;

    for (int i = 0; i < 4; ++i) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ledc_channels_[i], 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ledc_channels_[i]);
        pulsing_[i] = false;
    }
    updateStepCounters();
}

} // namespace holodex

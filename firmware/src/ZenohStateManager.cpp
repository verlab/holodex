#include "ZenohStateManager.hpp"






void ZenohStateManager::addListener(IZenohListener* listener) {
    z_view_keyexpr_t sub_ke;
    z_view_keyexpr_from_str_unchecked(&sub_ke, listener->getTopic().c_str());

    z_owned_closure_sample_t callback_closure;
    z_closure_sample(&callback_closure, ZenohStateManager::staticMessageHandler, NULL, listener);

    if (z_declare_subscriber(
        z_session_loan(&session), 
        &sub, 
        z_view_keyexpr_loan(&sub_ke), 
        z_closure_sample_move(&callback_closure), 
        NULL
    ) < 0) {
        Serial.println("Erro ao declarar o subscriber!");
        return;
    }
    
    
}

void ZenohStateManager::staticMessageRouter(z_loaned_sample_t *sample, void *arg)
{
    auto* instance = static_cast<IZenohListener*>(arg);
    if (instance) {
        instance->handle(sample);
    }
}

void ZenohStateManager::staticMessageHandler(z_loaned_sample_t *sample, void *arg)
{
    auto* instance = static_cast<ZenohStateManager*>(arg);
    if (instance) {
        instance->messageHandler(sample);
    }
}

// O método que realmente trata o dado dentro do contexto do objeto
void ZenohStateManager::messageHandler(const z_loaned_sample_t* sample) {
    const z_loaned_bytes_t* payload = z_sample_payload(sample);
    size_t payload_size = z_bytes_len(payload);

    // Logic for MotorDTO
    if (payload_size == sizeof(MotorDTO)) {
        MotorDTO data;
        z_bytes_reader_t reader = z_bytes_get_reader(payload);
        z_bytes_reader_read(&reader, (uint8_t*)&data, sizeof(MotorDTO));
        Serial.print("\n[Zenoh Class] Struct Recebido!");
        Serial.print(" V1: "); Serial.print(data.motorA);
        Serial.print(" | V2: "); Serial.print(data.motorB);
        Serial.print(" | V3: "); Serial.print(data.motorC);
        Serial.print(" | V4: "); Serial.println(data.motorD);
        lastPacket = data;
    } else {
        Serial.print("\n[Zenoh Class] Erro: Tamanho de payload inválido. Esperado: ");
        Serial.print(sizeof(MotorDTO));
        Serial.print(" bytes, Recebido: ");
        Serial.println(payload_size);
    }
}

ZenohStateManager::ZenohStateManager(const char* zenoh_router_endpoint)
{
    lastPacket.motorA = 0;
    lastPacket.motorB = 0;
    lastPacket.motorC = 0;
    lastPacket.motorD = 0;

    // 1. Zenoh initial setup
    z_owned_config_t config;
    z_config_default(&config);

    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, "client");
    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, zenoh_router_endpoint);

    // 2. Opening zenoh session
    Serial.print("Abrindo sessao Zenoh...");
    if (z_open(&session, z_config_move(&config), NULL) < 0) {
        Serial.println(" Erro ao abrir a sessao!");
        return;
    }
    Serial.println(" Sucesso!");

    // 3. Start processes
    if (zp_start_read_task(z_session_loan_mut(&session), NULL) < 0 || 
        zp_start_lease_task(z_session_loan_mut(&session), NULL) < 0) {
        Serial.println("Erro ao iniciar as tarefas de background!");
        return;
    }

    // 4. Declaring pub topic
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str_unchecked(&ke, "demo/esp32");
    
    // 5. Publisher declaration
    if (z_declare_publisher(z_session_loan(&session), &pub, z_view_keyexpr_loan(&ke), NULL) < 0) {
        Serial.println("Erro ao declarar o publisher!");
        return;
    }

    // 6. Internal subscriber config
    z_view_keyexpr_t sub_ke;
    z_view_keyexpr_from_str_unchecked(&sub_ke, "demo/pc_to_esp");

    z_owned_closure_sample_t callback_closure;
    z_closure_sample(&callback_closure, ZenohStateManager::staticMessageHandler, NULL, this);

    if (z_declare_subscriber(
        z_session_loan(&session), 
        &sub, 
        z_view_keyexpr_loan(&sub_ke), 
        z_closure_sample_move(&callback_closure), 
        NULL
    ) < 0) {
        Serial.println("Erro ao declarar o subscriber!");
        return;
    }
    
    zenoh_ready = true;
    Serial.println("Zenoh inicializado com sucesso!");
}

void ZenohStateManager::sendMessage(const char* message)
{
    if (zenoh_ready) {
        z_owned_bytes_t data;
        z_bytes_copy_from_str(&data, message);

        if (z_publisher_put(z_publisher_loan(&pub), z_bytes_move(&data), NULL) < 0) {
            Serial.println("Falha ao enviar dados pelo Zenoh.");
        } else {
            Serial.println("Enviado com sucesso!");
        }
    }
}

bool ZenohStateManager::isReady()
{
    return zenoh_ready;
}

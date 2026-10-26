#include <iostream>
#include <cassert>
#include <cmath>
#include <cstdint>

// Teste de host (sem ESP-IDF) do código de PRODUÇÃO: cinemática Mecanum e unwrapping do AS5600.
// Build:  g++ -std=c++17 -Imain -o /tmp/khost test/test_kinematics_host.cpp main/mecanum_kinematics.cpp
#include "mecanum_kinematics.hpp"
#include "encoder_math.hpp"

using holodex::MecanumKinematics;
using holodex::TwistCommand;
using holodex::WheelVelocities;

static int32_t unwrap_encoder_delta(uint16_t last_raw, uint16_t prev_raw) {
    return holodex::unwrapEncoderDelta12(last_raw, prev_raw);
}

int main() {
    std::cout << "=== Executando Testes Unitarios de Host (Holodex Base) ===" << std::endl;

    MecanumKinematics kin(0.050f, 0.390f, 0.380f); // R=50mm, Track=390mm, Base=380mm

    // Teste 1: Movimento linear puro para frente (+X)
    {
        TwistCommand cmd(0.20f, 0.0f, 0.0f);
        WheelVelocities w = kin.inverseKinematics(cmd);
        float expected_w = 0.20f / 0.050f; // 4.0 rad/s
        assert(std::fabs(w.fl - expected_w) < 1e-4);
        assert(std::fabs(w.fr - expected_w) < 1e-4);
        assert(std::fabs(w.rl - expected_w) < 1e-4);
        assert(std::fabs(w.rr - expected_w) < 1e-4);

        TwistCommand recovered = kin.forwardKinematics(w);
        assert(std::fabs(recovered.linear_x - cmd.linear_x) < 1e-4);
        assert(std::fabs(recovered.linear_y) < 1e-4);
        assert(std::fabs(recovered.angular_z) < 1e-4);
        std::cout << "[PASS] Teste 1: Cinemática para frente (+X)" << std::endl;
    }

    // Teste 2: Deslocamento lateral puro (+Y, strafe)
    {
        TwistCommand cmd(0.0f, 0.15f, 0.0f);
        WheelVelocities w = kin.inverseKinematics(cmd);
        // FL = -w, FR = +w, RL = +w, RR = -w
        float expected = 0.15f / 0.050f; // 3.0 rad/s
        assert(std::fabs(w.fl - (-expected)) < 1e-4);
        assert(std::fabs(w.fr - expected) < 1e-4);
        assert(std::fabs(w.rl - expected) < 1e-4);
        assert(std::fabs(w.rr - (-expected)) < 1e-4);

        TwistCommand recovered = kin.forwardKinematics(w);
        assert(std::fabs(recovered.linear_x) < 1e-4);
        assert(std::fabs(recovered.linear_y - cmd.linear_y) < 1e-4);
        assert(std::fabs(recovered.angular_z) < 1e-4);
        std::cout << "[PASS] Teste 2: Cinemática lateral (+Y strafe)" << std::endl;
    }

    // Teste 3: Rotação pura no próprio eixo (+Yaw, anti-horário)
    {
        TwistCommand cmd(0.0f, 0.0f, 1.0f);
        WheelVelocities w = kin.inverseKinematics(cmd);
        // Rodas da esquerda giram para trás (-K*w/R), direita para frente (+K*w/R)
        assert(w.fl < 0);
        assert(w.rl < 0);
        assert(w.fr > 0);
        assert(w.rr > 0);

        TwistCommand recovered = kin.forwardKinematics(w);
        assert(std::fabs(recovered.linear_x) < 1e-4);
        assert(std::fabs(recovered.linear_y) < 1e-4);
        assert(std::fabs(recovered.angular_z - cmd.angular_z) < 1e-4);
        std::cout << "[PASS] Teste 3: Cinemática de rotação (+Yaw)" << std::endl;
    }

    // Teste 4: Round-trip composto (vx + vy + wz)
    {
        TwistCommand cmd(0.35f, -0.20f, 0.85f);
        WheelVelocities w = kin.inverseKinematics(cmd);
        TwistCommand recovered = kin.forwardKinematics(w);
        assert(std::fabs(recovered.linear_x - cmd.linear_x) < 1e-4);
        assert(std::fabs(recovered.linear_y - cmd.linear_y) < 1e-4);
        assert(std::fabs(recovered.angular_z - cmd.angular_z) < 1e-4);
        std::cout << "[PASS] Teste 4: Round-trip completo (vx, vy, wz)" << std::endl;
    }

    // Teste 5: Unwrapping de encoder AS5600 através da fronteira 4095 <-> 0
    {
        // Giro para frente: 4090 -> 5 (deve ser +11)
        assert(unwrap_encoder_delta(5, 4090) == 11);

        // Giro para trás: 5 -> 4090 (deve ser -11)
        assert(unwrap_encoder_delta(4090, 5) == -11);

        // Sem rotação: 2000 -> 2000 (deve ser 0)
        assert(unwrap_encoder_delta(2000, 2000) == 0);

        // Pequeno incremento normal: 100 -> 115 (deve ser +15)
        assert(unwrap_encoder_delta(115, 100) == 15);

        // Pequeno decremento normal: 115 -> 100 (deve ser -15)
        assert(unwrap_encoder_delta(100, 115) == -15);

        std::cout << "[PASS] Teste 5: Unwrapping de encoder 12 bits com salto de fronteira" << std::endl;
    }

    std::cout << ">>> TODOS OS TESTES PASSARAM COM SUCESSO! <<<" << std::endl;
    return 0;
}

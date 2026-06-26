#include "ZenohStateManager.hpp"






// O "trampolim" estático que o Zenoh chama
void ZenohStateManager::staticMessageHandler(z_loaned_sample_t* sample, void* arg) {
    auto* instance = static_cast<ZenohStateManager*>(arg);
    if (instance) {
        instance->messageHandler(sample);
    }
}

// O método que realmente trata o dado dentro do contexto do objeto
void ZenohStateManager::messageHandler(const z_loaned_sample_t* sample) {

    // Recupera a estrutura gerenciadora de bytes do Zenoh
    const z_loaned_bytes_t* payload = z_sample_payload(sample);

    // 1. API oficial para recuperar o tamanho (não acesse _slices._len diretamente)
    size_t payload_size = z_bytes_len(payload);

    // 2. Validação de segurança
    if (payload_size == sizeof(MotorDTO)) {
        MotorDTO data;
        
        // 3. Extrai os bytes REAIS usando a API de Reader oficial do Zenoh
        z_bytes_reader_t reader = z_bytes_get_reader(payload);
        z_bytes_reader_read(&reader, (uint8_t*)&data, sizeof(MotorDTO));

        // 4. Agora você pode usar as variáveis!
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

    // 2. Inicialização da Configuração
    z_owned_config_t config;
    z_config_default(&config);

    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, "client");
    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, zenoh_router_endpoint);

    // 4. Abrindo a sessão Zenoh
    Serial.print("Abrindo sessao Zenoh...");
    if (z_open(&session, z_config_move(&config), NULL) < 0) {
        Serial.println(" Erro ao abrir a sessao!");
        return;
    }
    Serial.println(" Sucesso!");

    // 5. Iniciar as tarefas de rede em background
    if (zp_start_read_task(z_session_loan_mut(&session), NULL) < 0 || 
        zp_start_lease_task(z_session_loan_mut(&session), NULL) < 0) {
        Serial.println("Erro ao iniciar as tarefas de background!");
        return;
    }

    // 6. Declarando a Key Expression (Tópico) do Publisher
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str_unchecked(&ke, "demo/esp32");
    
    // 7. Declarando o Publisher
    if (z_declare_publisher(z_session_loan(&session), &pub, z_view_keyexpr_loan(&ke), NULL) < 0) {
        Serial.println("Erro ao declarar o publisher!");
        return;
    }

    // 8. Configurando o Subscriber
    z_view_keyexpr_t sub_ke;
    z_view_keyexpr_from_str_unchecked(&sub_ke, "demo/pc_to_esp");

    // CORREÇÃO DA CLOSURE: Instanciamos e configuramos o container da closure explicitamente
    z_owned_closure_sample_t callback_closure;
    z_closure_sample(&callback_closure, ZenohStateManager::staticMessageHandler, NULL, this);

    // Registramos passando a closure movida via 'z_closure_sample_move'
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
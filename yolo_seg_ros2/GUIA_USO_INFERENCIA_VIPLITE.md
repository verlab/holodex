# Guia de Inferência em C++ com Runtime VIPLite na Orange Pi (Allwinner A733)

Este guia ensina como integrar e executar o modelo de segmentação compilado [`yolov8n_seg_final.nb`](file:///workspace/yolov8n_seg_final.nb) na placa **Orange Pi (Allwinner A733)** usando a biblioteca **VIPLite** (`vip_lite.h`, `libVIPhal.so`) e **OpenCV** em C++.

---

## 1. Arquitetura da Solução

Para maximizar o FPS e evitar gargalos:
* **NPU (VIP9000)**: Executa com aceleração de hardware todas as convoluções em INT8 puro, gerando os 10 tensores de features.
* **CPU (ARM Cortex)**: Executa o pré-processamento (letterbox e quantização de entrada) e o pós-processamento vetorial (desquantização, DFL, sigmoid, NMS, multiplicação matricial Proto x coeficientes e corte da máscara).

```
   [ Imagem da Câmera / Arquivo ]
                 │
       (Pré-proc CPU / Letterbox)
                 │
                 ▼
       [ Buffer INT8 416x416x3 ]
                 │
           (NPU VIP9000)
                 │
                 ▼
     [ 10 Tensores de Saída INT8 ]
     ┌───────────┬──────────────────────────────────────────┐
     │  Output 0 │ Protótipos de Máscara (104x104x32)       │
     │  Outputs  │ P3, P4, P5: Caixas (64), Classes (80),   │
     │   1 a 9   │             Coeficientes de Máscara (32) │
     └───────────┴──────────────────────────────────────────┘
                 │
      (Pós-processamento C++ na CPU)
       ├── 1. Desquantização (scale, zero_point)
       ├── 2. Decodificação DFL das Bounding Boxes
       ├── 3. Sigmoid de Classes e Threshold de Confiança
       ├── 4. Supressão de Não-Máximos (NMS)
       └── 5. Matmul: Coeficientes x Protótipos -> Máscara
                 │
                 ▼
     [ Detecções com Bounding Boxes e Máscaras de Instância ]
```

---

## 2. Metadados e Tabela de Quantização do Modelo

O binário [`yolov8n_seg_final.nb`](file:///workspace/yolov8n_seg_final.nb) possui a seguinte assinatura de tensores:

### Entrada (Input 0)
* **Dimensões**: `[416, 416, 3, 1]` (Largura, Altura, Canais, Batch)
* **Tipo**: `VIP_BUFFER_FORMAT_INT8`
* **Scale**: `0.003922` ($1/255.0$)
* **ZeroPoint**: `-128`
* **Fórmula de Quantização**: $q = \text{clamp}\left(\text{round}\left(\frac{\text{pixel}}{255.0 \times \text{scale}}\right) + \text{zeropoint}, -128, 127\right) = \text{pixel} - 128$

### Saídas (10 Tensores)

| Índice | Nome | Dimensões ($W, H, C$) | Stride | Quant Scale | ZeroPoint | Descrição |
| :---: | :--- | :---: | :---: | :---: | :---: | :--- |
| **0** | `uid_30004_out_0` | `[104, 104, 32]` | /4 | `0.026333` | `-117` | **Protótipos de Máscara (Proto)** |
| **1** | `uid_18_out_0t_0` | `[52, 52, 64]` | /8 (P3) | `0.132677` | `-66` | P3 — Bounding Boxes (4 x 16 bins DFL) |
| **2** | `uid_17_out_0t_0` | `[52, 52, 80]` | /8 (P3) | `0.125060` | `111` | P3 — Classes (80 classes COCO) |
| **3** | `uid_16_out_0t_0` | `[52, 52, 32]` | /8 (P3) | `0.021611` | `24` | P3 — Coeficientes de Máscara (32 coefs) |
| **4** | `uid_15_out_0t_0` | `[26, 26, 64]` | /16 (P4) | `0.094824` | `-40` | P4 — Bounding Boxes (4 x 16 bins DFL) |
| **5** | `uid_14_out_0t_0` | `[26, 26, 80]` | /16 (P4) | `0.183596` | `114` | P4 — Classes |
| **6** | `uid_13_out_0t_0` | `[26, 26, 32]` | /16 (P4) | `0.019247` | `32` | P4 — Coeficientes de Máscara |
| **7** | `uid_12_out_0t_0` | `[13, 13, 64]` | /32 (P5) | `0.085507` | `-49` | P5 — Bounding Boxes (4 x 16 bins DFL) |
| **8** | `uid_11_out_0t_0` | `[13, 13, 80]` | /32 (P5) | `0.193360` | `110` | P5 — Classes |
| **9** | `uid_10_out_0t_0` | `[13, 13, 32]` | /32 (P5) | `0.017907` | `37` | P5 — Coeficientes de Máscara |

---

## 3. Código C++ Completo de Inferência (`yolo_seg_viplite.cpp`)

Salve o arquivo a seguir na sua aplicação:

```cpp
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "vip_lite.h"

// -------------------------------------------------------------
// Constantes e Estruturas
// -------------------------------------------------------------
constexpr int INPUT_W = 416;
constexpr int INPUT_H = 416;
constexpr int NUM_CLASSES = 80;
constexpr int NUM_MASKS = 32;
constexpr int PROTO_W = 104;
constexpr int PROTO_H = 104;
constexpr float CONF_THRESHOLD = 0.25f;
constexpr float NMS_THRESHOLD = 0.45f;
constexpr float MASK_THRESHOLD = 0.5f;

struct Detection {
    cv::Rect2f box;
    int class_id;
    float score;
    std::vector<float> mask_coef; // 32 coeficientes
    cv::Mat mask;                 // Máscara binária resultante
};

// Parâmetros de quantização dos 10 tensores de saída
struct QuantParam {
    float scale;
    int32_t zero_point;
};

const QuantParam OUT_QUANT[10] = {
    {0.026333f, -117}, // Output 0: Proto (104x104x32)
    {0.132677f,  -66}, // Output 1: P3 Box (52x52x64)
    {0.125060f,  111}, // Output 2: P3 Class (52x52x80)
    {0.021611f,   24}, // Output 3: P3 Mask (52x52x32)
    {0.094824f,  -40}, // Output 4: P4 Box (26x26x64)
    {0.183596f,  114}, // Output 5: P4 Class (26x26x80)
    {0.019247f,   32}, // Output 6: P4 Mask (26x26x32)
    {0.085507f,  -49}, // Output 7: P5 Box (13x13x64)
    {0.193360f,  110}, // Output 8: P5 Class (13x13x80)
    {0.017907f,   37}  // Output 9: P5 Mask (13x13x32)
};

// -------------------------------------------------------------
// Funções Auxiliares
// -------------------------------------------------------------
inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

inline float dequantize(int8_t val, const QuantParam& q) {
    return (static_cast<float>(val) - q.zero_point) * q.scale;
}

// Letterbox com padding simétrico mantendo proporção da imagem
cv::Mat preprocess_letterbox(const cv::Mat& src, float& scale, int& pad_x, int& pad_y) {
    float r = std::min(static_cast<float>(INPUT_W) / src.cols,
                       static_cast<float>(INPUT_H) / src.rows);
    int new_w = static_cast<int>(std::round(src.cols * r));
    int new_h = static_cast<int>(std::round(src.rows * r));

    pad_x = (INPUT_W - new_w) / 2;
    pad_y = (INPUT_H - new_h) / 2;

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

    cv::Mat dst(INPUT_H, INPUT_W, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(dst(cv::Rect(pad_x, pad_y, new_w, new_h)));

    scale = r;
    return dst;
}

// Decodificação DFL (Distribution Focal Loss) de 16 bins para 1 coordenada
float decode_single_dfl(const float* raw_16_bins) {
    float sum_exp = 0.0f;
    float softmax[16];
    for (int i = 0; i < 16; ++i) {
        softmax[i] = std::exp(raw_16_bins[i]);
        sum_exp += softmax[i];
    }
    float coord = 0.0f;
    for (int i = 0; i < 16; ++i) {
        coord += (softmax[i] / sum_exp) * i;
    }
    return coord;
}

// -------------------------------------------------------------
// Pós-Processamento das Cabeças (P3, P4, P5)
// -------------------------------------------------------------
void decode_head_scale(
    int grid_size, int stride,
    const int8_t* box_buf,   const QuantParam& q_box,
    const int8_t* cls_buf,   const QuantParam& q_cls,
    const int8_t* mask_buf,  const QuantParam& q_mask,
    std::vector<Detection>& candidates)
{
    // Layout no buffer Vivante: [W, H, C]
    for (int h = 0; h < grid_size; ++h) {
        for (int w = 0; w < grid_size; ++w) {
            // Localiza a classe de maior confiança
            int max_cls_id = -1;
            float max_score = -1.0f;

            for (int c = 0; c < NUM_CLASSES; ++c) {
                int idx = (c * grid_size * grid_size) + (h * grid_size) + w;
                float raw_score = dequantize(cls_buf[idx], q_cls);
                float prob = sigmoid(raw_score);
                if (prob > max_score) {
                    max_score = prob;
                    max_cls_id = c;
                }
            }

            if (max_score < CONF_THRESHOLD) continue;

            // Decodifica as 4 distâncias (Left, Top, Right, Bottom) via DFL
            float dfl_dist[4];
            for (int d = 0; d < 4; ++d) {
                float raw_16[16];
                for (int b = 0; b < 16; ++b) {
                    int c_idx = (d * 16) + b;
                    int idx = (c_idx * grid_size * grid_size) + (h * grid_size) + w;
                    raw_16[b] = dequantize(box_buf[idx], q_box);
                }
                dfl_dist[d] = decode_single_dfl(raw_16);
            }

            // Converte distâncias em coordenadas [x1, y1, x2, y2]
            float cx = (w + 0.5f);
            float cy = (h + 0.5f);
            float x1 = (cx - dfl_dist[0]) * stride;
            float y1 = (cy - dfl_dist[1]) * stride;
            float x2 = (cx + dfl_dist[2]) * stride;
            float y2 = (cy + dfl_dist[3]) * stride;

            Detection det;
            det.box = cv::Rect2f(x1, y1, x2 - x1, y2 - y1);
            det.class_id = max_cls_id;
            det.score = max_score;

            // Extrai os 32 coeficientes de máscara
            det.mask_coef.resize(NUM_MASKS);
            for (int m = 0; m < NUM_MASKS; ++m) {
                int idx = (m * grid_size * grid_size) + (h * grid_size) + w;
                det.mask_coef[m] = dequantize(mask_buf[idx], q_mask);
            }

            candidates.push_back(det);
        }
    }
}

// -------------------------------------------------------------
// Geração das Máscaras de Instância (Matmul com Proto)
// -------------------------------------------------------------
void generate_instance_masks(
    std::vector<Detection>& detections,
    const int8_t* proto_raw,
    const QuantParam& q_proto,
    const cv::Size& orig_size,
    float scale, int pad_x, int pad_y)
{
    // Desquantiza o tensor Proto (104 x 104 x 32)
    // Layout: [32, 104, 104]
    std::vector<float> proto_float(NUM_MASKS * PROTO_H * PROTO_W);
    for (size_t i = 0; i < proto_float.size(); ++i) {
        proto_float[i] = dequantize(proto_raw[i], q_proto);
    }

    for (auto& det : detections) {
        cv::Mat mask_104(PROTO_H, PROTO_W, CV_32FC1, cv::Scalar(0));

        // Multiplicação matricial: sum(coef[m] * proto[m, h, w])
        for (int m = 0; m < NUM_MASKS; ++m) {
            float coef = det.mask_coef[m];
            const float* proto_plane = &proto_float[m * PROTO_H * PROTO_W];

            for (int h = 0; h < PROTO_H; ++h) {
                float* mask_row = mask_104.ptr<float>(h);
                const float* p_row = &proto_plane[h * PROTO_W];
                for (int w = 0; w < PROTO_W; ++w) {
                    mask_row[w] += coef * p_row[w];
                }
            }
        }

        // Aplica Sigmoid e threshold
        cv::Mat mask_bin(PROTO_H, PROTO_W, CV_8UC1);
        for (int h = 0; h < PROTO_H; ++h) {
            float* m_row = mask_104.ptr<float>(h);
            uint8_t* b_row = mask_bin.ptr<uint8_t>(h);
            for (int w = 0; w < PROTO_W; ++w) {
                b_row[w] = (sigmoid(m_row[w]) > MASK_THRESHOLD) ? 255 : 0;
            }
        }

        // Redimensiona do espaço 104x104 para 416x416
        cv::Mat mask_416;
        cv::resize(mask_bin, mask_416, cv::Size(INPUT_W, INPUT_H), 0, 0, cv::INTER_LINEAR);

        // Recorta apenas a região interna da bounding box
        cv::Rect roi = det.box;
        roi = roi & cv::Rect(0, 0, INPUT_W, INPUT_H);
        cv::Mat cropped_mask = cv::Mat::zeros(INPUT_H, INPUT_W, CV_8UC1);
        mask_416(roi).copyTo(cropped_mask(roi));

        // Mapeia de volta para a imagem original (desfaz letterbox)
        cv::Rect orig_roi(
            static_cast<int>((det.box.x - pad_x) / scale),
            static_cast<int>((det.box.y - pad_y) / scale),
            static_cast<int>(det.box.width / scale),
            static_cast<int>(det.box.height / scale)
        );
        orig_roi = orig_roi & cv::Rect(0, 0, orig_size.width, orig_size.height);
        det.box = cv::Rect2f(orig_roi);

        cv::Mat unpad_mask = cropped_mask(cv::Rect(pad_x, pad_y, INPUT_W - 2 * pad_x, INPUT_H - 2 * pad_y));
        cv::resize(unpad_mask, det.mask, orig_size, 0, 0, cv::INTER_NEAREST);
    }
}

// -------------------------------------------------------------
// Função Principal com Ciclo de Vida VIPLite
// -------------------------------------------------------------
int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Uso: " << argv[0] << " <modelo.nb> <imagem.jpg>\n";
        return -1;
    }

    std::string nb_path = argv[1];
    std::string img_path = argv[2];

    // 1. Carrega o arquivo NBG compilado na memória
    std::ifstream nb_file(nb_path, std::ios::binary | std::ios::ate);
    if (!nb_file.is_open()) {
        std::cerr << "Falha ao abrir modelo: " << nb_path << "\n";
        return -1;
    }
    size_t nb_size = nb_file.tellg();
    nb_file.seekg(0, std::ios::beg);
    std::vector<char> nb_buffer(nb_size);
    nb_file.read(nb_buffer.data(), nb_size);
    nb_file.close();

    // 2. Inicializa o hardware NPU via VIPLite
    vip_status_e status = vip_init();
    if (status != VIP_SUCCESS) {
        std::cerr << "vip_init falhou: " << status << "\n";
        return -1;
    }

    vip_network network = nullptr;
    status = vip_create_network(nb_buffer.data(), nb_size, VIP_CREATE_NETWORK_FROM_MEMORY, &network);
    if (status != VIP_SUCCESS) {
        std::cerr << "vip_create_network falhou: " << status << "\n";
        vip_destroy();
        return -1;
    }

    // 3. Prepara os buffers de entrada e saída
    vip_buffer input_buf = nullptr;
    vip_buffer_create_params_t in_param = {};
    in_param.data_format = VIP_BUFFER_FORMAT_INT8;
    in_param.data_type = VIP_BUFFER_TYPE_TENSOR;
    in_param.num_of_dims = 4;
    in_param.sizes[0] = INPUT_W;
    in_param.sizes[1] = INPUT_H;
    in_param.sizes[2] = 3;
    in_param.sizes[3] = 1;
    vip_create_buffer(&in_param, sizeof(in_param), &input_buf);
    vip_set_input(network, 0, input_buf);

    vip_buffer out_bufs[10];
    vip_uint32_t out_count = 0;
    vip_query_network(network, VIP_NETWORK_PROP_OUTPUT_COUNT, &out_count);

    for (uint32_t i = 0; i < out_count; ++i) {
        vip_buffer_create_params_t out_param = {};
        vip_query_output(network, i, VIP_BUFFER_PROP_DATA_FORMAT, &out_param.data_format);
        vip_query_output(network, i, VIP_BUFFER_PROP_DATA_TYPE, &out_param.data_type);
        vip_query_output(network, i, VIP_BUFFER_PROP_NUM_OF_DIMS, &out_param.num_of_dims);
        vip_query_output(network, i, VIP_BUFFER_PROP_SIZES, out_param.sizes);

        vip_create_buffer(&out_param, sizeof(out_param), &out_bufs[i]);
        vip_set_output(network, i, out_bufs[i]);
    }

    // 4. Carrega e pré-processa a imagem
    cv::Mat orig_img = cv::imread(img_path);
    if (orig_img.empty()) {
        std::cerr << "Falha ao abrir imagem: " << img_path << "\n";
        return -1;
    }

    float scale = 1.0f;
    int pad_x = 0, pad_y = 0;
    cv::Mat letterbox_img = preprocess_letterbox(orig_img, scale, pad_x, pad_y);

    // Mapeia pixels BGR -> RGB e quantiza: q = pixel - 128
    int8_t* in_ptr = static_cast<int8_t*>(vip_map_buffer(input_buf));
    for (int c = 0; c < 3; ++c) { // RGB Planar (NCHW)
        int src_c = 2 - c; // Inverte BGR -> RGB
        for (int h = 0; h < INPUT_H; ++h) {
            for (int w = 0; w < INPUT_W; ++w) {
                uint8_t pix = letterbox_img.at<cv::Vec3b>(h, w)[src_c];
                *in_ptr++ = static_cast<int8_t>(pix - 128);
            }
        }
    }
    vip_unmap_buffer(input_buf);

    // 5. Executa a inferência na NPU
    std::cout << "[NPU] Executando inferencia...\n";
    vip_run_network(network);
    std::cout << "[NPU] Inferencia concluida!\n";

    // 6. Mapeia as saídas da NPU
    const int8_t* out_ptrs[10];
    for (int i = 0; i < 10; ++i) {
        out_ptrs[i] = static_cast<const int8_t*>(vip_map_buffer(out_bufs[i]));
    }

    // 7. Pós-processamento na CPU
    std::vector<Detection> candidates;

    // Escala P3: 52x52 (stride 8)
    decode_head_scale(52, 8,
        out_ptrs[1], OUT_QUANT[1],
        out_ptrs[2], OUT_QUANT[2],
        out_ptrs[3], OUT_QUANT[3],
        candidates);

    // Escala P4: 26x26 (stride 16)
    decode_head_scale(26, 16,
        out_ptrs[4], OUT_QUANT[4],
        out_ptrs[5], OUT_QUANT[5],
        out_ptrs[6], OUT_QUANT[6],
        candidates);

    // Escala P5: 13x13 (stride 32)
    decode_head_scale(13, 32,
        out_ptrs[7], OUT_QUANT[7],
        out_ptrs[8], OUT_QUANT[8],
        out_ptrs[9], OUT_QUANT[9],
        candidates);

    // 8. Aplica NMS
    std::vector<cv::Rect> nms_boxes;
    std::vector<float> nms_scores;
    for (const auto& c : candidates) {
        nms_boxes.push_back(c.box);
        nms_scores.push_back(c.score);
    }

    std::vector<int> keep_indices;
    cv::dnn::NMSBoxes(nms_boxes, nms_scores, CONF_THRESHOLD, NMS_THRESHOLD, keep_indices);

    std::vector<Detection> final_detections;
    for (int idx : keep_indices) {
        final_detections.push_back(candidates[idx]);
    }

    // 9. Gera as máscaras das detecções filtradas
    generate_instance_masks(final_detections, out_ptrs[0], OUT_QUANT[0], orig_img.size(), scale, pad_x, pad_y);

    std::cout << "Deteccoes encontradas: " << final_detections.size() << "\n";

    // 10. Desenha caixas e sobrepõe máscaras coloridas
    cv::Mat overlay = orig_img.clone();
    for (const auto& det : final_detections) {
        cv::rectangle(orig_img, det.box, cv::Scalar(0, 255, 0), 2);
        std::string label = "Cls " + std::to_string(det.class_id) + ": " + cv::format("%.2f", det.score);
        cv::putText(orig_img, label, cv::Point(det.box.x, det.box.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

        // Aplica cor na máscara
        cv::Scalar mask_color(rand() % 200 + 55, rand() % 200 + 55, rand() % 200 + 55);
        overlay.setTo(mask_color, det.mask);
    }
    cv::addWeighted(orig_img, 0.6, overlay, 0.4, 0, orig_img);

    cv::imwrite("resultado_segmentacao.jpg", orig_img);
    std::cout << "Resultado salvo em 'resultado_segmentacao.jpg'!\n";

    // 11. Libera recursos do VIPLite
    for (int i = 0; i < 10; ++i) {
        vip_unmap_buffer(out_bufs[i]);
        vip_destroy_buffer(out_bufs[i]);
    }
    vip_destroy_buffer(input_buf);
    vip_destroy_network(network);
    vip_destroy();

    return 0;
}
```

---

## 4. Compilando na Placa Orange Pi (Makefile)

Crie o arquivo `Makefile` no diretório da sua aplicação na placa:

```makefile
CXX = g++
CXXFLAGS = -O3 -std=c++14 `pkg-config --cflags opencv4`
LDFLAGS = `pkg-config --libs opencv4` -L/usr/lib -lVIPhal -lpthread -ldl

TARGET = yolo_seg_viplite
SRCS = yolo_seg_viplite.cpp

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET) resultado_segmentacao.jpg
```

Para compilar e executar diretamente na Orange Pi:
```bash
make
./yolo_seg_viplite yolov8n_seg_final.nb teste.jpg
```

---

## 5. Dicas de Otimização e Performance no Target

1. **Zero-Copy com Buffers de Vídeo (DMA / V4L2)**:
   Em vez de alocar buffers comuns via CPU, utilize `VIP_BUFFER_MEMORY_TYPE_DMA_BUF` em `vip_create_buffer` para conectar diretamente o buffer da câmera V4L2/ISP à entrada da NPU sem cópia de memória (`memcpy`).

2. **Loop de Inferência Contínuo**:
   A rede deve ser criada **apenas uma vez** (`vip_create_network`) na inicialização da aplicação e os buffers de entrada/saída reutilizados em loop chamando apenas `vip_run_network()`.

3. **Cálculo Paralelo do Matmul das Máscaras**:
   O pós-processamento de máscara em CPU (`generate_instance_masks`) pode ser acelerado usando OpenMP (`#pragma omp parallel for`) para calcular simultaneamente os pixels das máscaras em múltiplos núcleos ARM.

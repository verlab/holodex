#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "vip_lite.h"

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>

// -------------------------------------------------------------
// Constantes e Estruturas YOLOv8-seg (416x416)
// -------------------------------------------------------------
constexpr int INPUT_W = 416;
constexpr int INPUT_H = 416;
constexpr int NUM_CLASSES = 80;
constexpr int NUM_MASKS = 32;
constexpr int PROTO_W = 104;
constexpr int PROTO_H = 104;

const std::vector<std::string> COCO_CLASSES = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"
};

struct Detection {
    cv::Rect2f box;
    int class_id;
    float score;
    std::vector<float> mask_coef;
    cv::Mat mask;
};

struct QuantParam {
    float scale;
    int32_t zero_point;
};

struct LetterboxInfo {
    float scale;
    int pad_x;
    int pad_y;
    int new_w;
    int new_h;
};

// -------------------------------------------------------------
// Funções Matemáticas Auxiliares
// -------------------------------------------------------------
inline bool is_dynamic_class(int class_id) {
    // Classes COCO com movimento independente (humanos, veiculos e animais)
    switch (class_id) {
        case 0:  // person
        case 1:  // bicycle
        case 2:  // car
        case 3:  // motorcycle
        case 5:  // bus
        case 7:  // truck
        case 14: // bird
        case 15: // cat
        case 16: // dog
        case 17: // horse
        case 18: // sheep
        case 19: // cow
            return true;
        default:
            return false;
    }
}

inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

inline float dequantize(int8_t val, const QuantParam& q) {
    return (static_cast<float>(val) - q.zero_point) * q.scale;
}

// Decodificação DFL (Distribution Focal Loss) de 16 bins com estabilidade numérica
float decode_single_dfl(const float* raw_16_bins) {
    float max_val = raw_16_bins[0];
    for (int i = 1; i < 16; ++i) {
        if (raw_16_bins[i] > max_val) max_val = raw_16_bins[i];
    }
    float sum_exp = 0.0f;
    float softmax[16];
    for (int i = 0; i < 16; ++i) {
        softmax[i] = std::exp(raw_16_bins[i] - max_val);
        sum_exp += softmax[i];
    }
    float coord = 0.0f;
    for (int i = 0; i < 16; ++i) {
        coord += (softmax[i] / sum_exp) * i;
    }
    return coord;
}

// -------------------------------------------------------------
// Classe Engine VIPLite para YOLOv8-seg na NPU
// -------------------------------------------------------------
class VipLiteYoloSegEngine {
public:
    VipLiteYoloSegEngine() = default;
    ~VipLiteYoloSegEngine() {
        cleanup();
    }

    bool init(const std::string& model_path) {
        std::cout << "[NPU] Carregando modelo compilado: " << model_path << "\n";
        std::ifstream file(model_path, std::ios::binary | std::ios::ate);
        if (!file.is_open()) {
            std::cerr << "[NPU ERROR] Falha ao abrir o arquivo do modelo: " << model_path << "\n";
            return false;
        }

        size_t size = file.tellg();
        file.seekg(0, std::ios::beg);
        m_nb_buffer.resize(size);
        file.read(m_nb_buffer.data(), size);
        file.close();

        vip_status_e status = vip_init();
        if (status != VIP_SUCCESS) {
            std::cerr << "[NPU ERROR] vip_init falhou com status: " << status << "\n";
            return false;
        }

        status = vip_create_network(m_nb_buffer.data(), size, VIP_CREATE_NETWORK_FROM_MEMORY, &m_network);
        if (status != VIP_SUCCESS) {
            std::cerr << "[NPU ERROR] vip_create_network falhou: " << status << "\n";
            return false;
        }

        status = vip_prepare_network(m_network);
        if (status != VIP_SUCCESS) {
            std::cerr << "[NPU ERROR] vip_prepare_network falhou: " << status << "\n";
            return false;
        }

        // Configurar Buffer de Entrada
        vip_buffer_create_params_t in_param;
        std::memset(&in_param, 0, sizeof(in_param));
        vip_query_input(m_network, 0, VIP_BUFFER_PROP_DATA_FORMAT, &in_param.data_format);
        vip_query_input(m_network, 0, VIP_BUFFER_PROP_NUM_OF_DIMENSION, &in_param.num_of_dims);
        vip_query_input(m_network, 0, VIP_BUFFER_PROP_SIZES_OF_DIMENSION, in_param.sizes);
        vip_query_input(m_network, 0, VIP_BUFFER_PROP_QUANT_FORMAT, &in_param.quant_format);
        vip_query_input(m_network, 0, VIP_BUFFER_PROP_TF_SCALE, &in_param.quant_data.affine.scale);
        vip_query_input(m_network, 0, VIP_BUFFER_PROP_TF_ZERO_POINT, &in_param.quant_data.affine.zeroPoint);

        status = vip_create_buffer(&in_param, sizeof(in_param), &m_input_buf);
        if (status != VIP_SUCCESS) {
            std::cerr << "[NPU ERROR] vip_create_buffer para entrada falhou: " << status << "\n";
            return false;
        }
        vip_set_input(m_network, 0, m_input_buf);

        // Configurar Buffers de Saída (10 tensores)
        vip_query_network(m_network, VIP_NETWORK_PROP_OUTPUT_COUNT, &m_out_count);
        if (m_out_count != 10) {
            std::cerr << "[NPU WARN] Quantidade de saidas esperadas: 10, obtido: " << m_out_count << "\n";
        }

        for (uint32_t i = 0; i < m_out_count && i < 10; ++i) {
            vip_buffer_create_params_t out_param;
            std::memset(&out_param, 0, sizeof(out_param));
            vip_query_output(m_network, i, VIP_BUFFER_PROP_DATA_FORMAT, &out_param.data_format);
            vip_query_output(m_network, i, VIP_BUFFER_PROP_NUM_OF_DIMENSION, &out_param.num_of_dims);
            vip_query_output(m_network, i, VIP_BUFFER_PROP_SIZES_OF_DIMENSION, out_param.sizes);
            vip_query_output(m_network, i, VIP_BUFFER_PROP_QUANT_FORMAT, &out_param.quant_format);
            vip_query_output(m_network, i, VIP_BUFFER_PROP_TF_SCALE, &m_out_quant[i].scale);
            vip_query_output(m_network, i, VIP_BUFFER_PROP_TF_ZERO_POINT, &m_out_quant[i].zero_point);

            out_param.quant_data.affine.scale = m_out_quant[i].scale;
            out_param.quant_data.affine.zeroPoint = m_out_quant[i].zero_point;

            status = vip_create_buffer(&out_param, sizeof(out_param), &m_out_bufs[i]);
            if (status != VIP_SUCCESS) {
                std::cerr << "[NPU ERROR] vip_create_buffer para saida " << i << " falhou: " << status << "\n";
                return false;
            }
            vip_set_output(m_network, i, m_out_bufs[i]);
        }

        std::cout << "[NPU] Inicializacao concluida com sucesso na VIP9000 (3 TOPS)!\n";
        m_initialized = true;
        return true;
    }

    void preprocess(const cv::Mat& src_bgr, LetterboxInfo& info) {
        float r = std::min(static_cast<float>(INPUT_W) / src_bgr.cols,
                           static_cast<float>(INPUT_H) / src_bgr.rows);
        int new_w = static_cast<int>(std::round(src_bgr.cols * r));
        int new_h = static_cast<int>(std::round(src_bgr.rows * r));

        info.scale = r;
        info.new_w = new_w;
        info.new_h = new_h;
        info.pad_x = (INPUT_W - new_w) / 2;
        info.pad_y = (INPUT_H - new_h) / 2;

        cv::Mat resized;
        cv::resize(src_bgr, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

        cv::Mat letterbox(INPUT_H, INPUT_W, CV_8UC3, cv::Scalar(114, 114, 114));
        resized.copyTo(letterbox(cv::Rect(info.pad_x, info.pad_y, new_w, new_h)));

        // Mapear buffer de entrada da NPU: layout Planar RGB [3, 416, 416], quantizado: pixel - 128
        int8_t* in_ptr = static_cast<int8_t*>(vip_map_buffer(m_input_buf));
        for (int c = 0; c < 3; ++c) {
            int src_c = 2 - c; // BGR -> RGB
            for (int h = 0; h < INPUT_H; ++h) {
                for (int w = 0; w < INPUT_W; ++w) {
                    uint8_t pix = letterbox.at<cv::Vec3b>(h, w)[src_c];
                    *in_ptr++ = static_cast<int8_t>(pix - 128);
                }
            }
        }
        vip_unmap_buffer(m_input_buf);
    }

    bool infer() {
        if (!m_initialized) return false;
        vip_status_e status = vip_run_network(m_network);
        return (status == VIP_SUCCESS);
    }

    void postprocess(const cv::Mat& orig_bgr, const LetterboxInfo& info,
                     float conf_threshold, float nms_threshold, float mask_threshold,
                     std::vector<Detection>& final_detections, cv::Mat& annotated_img,
                     std::string& debug_top_candidate) {
        // Mapear ponteiros dos 10 tensores de saída da NPU
        const int8_t* out_ptrs[10];
        for (uint32_t i = 0; i < m_out_count; ++i) {
            out_ptrs[i] = static_cast<const int8_t*>(vip_map_buffer(m_out_bufs[i]));
        }

        std::vector<Detection> candidates;
        float global_max_score = -1.0f;
        int global_max_cls = -1;

        // Escala P3: 52x52, Stride 8 (Tensores 1=Box, 2=Cls, 3=Mask)
        decode_head_scale(52, 8,
                          out_ptrs[1], m_out_quant[1],
                          out_ptrs[2], m_out_quant[2],
                          out_ptrs[3], m_out_quant[3],
                          conf_threshold, candidates,
                          global_max_score, global_max_cls);

        // Escala P4: 26x26, Stride 16 (Tensores 4=Box, 5=Cls, 6=Mask)
        decode_head_scale(26, 16,
                          out_ptrs[4], m_out_quant[4],
                          out_ptrs[5], m_out_quant[5],
                          out_ptrs[6], m_out_quant[6],
                          conf_threshold, candidates,
                          global_max_score, global_max_cls);

        // Escala P5: 13x13, Stride 32 (Tensores 7=Box, 8=Cls, 9=Mask)
        decode_head_scale(13, 32,
                          out_ptrs[7], m_out_quant[7],
                          out_ptrs[8], m_out_quant[8],
                          out_ptrs[9], m_out_quant[9],
                          conf_threshold, candidates,
                          global_max_score, global_max_cls);

        if (global_max_cls >= 0 && global_max_cls < static_cast<int>(COCO_CLASSES.size())) {
            debug_top_candidate = COCO_CLASSES[global_max_cls] + " (" + cv::format("%.1f%%", global_max_score * 100.0f) + ")";
        }

        // Aplicação de NMS
        std::vector<cv::Rect> nms_boxes;
        std::vector<float> nms_scores;
        for (const auto& c : candidates) {
            nms_boxes.push_back(c.box);
            nms_scores.push_back(c.score);
        }

        std::vector<int> keep_indices;
        cv::dnn::NMSBoxes(nms_boxes, nms_scores, conf_threshold, nms_threshold, keep_indices);

        final_detections.clear();
        for (int idx : keep_indices) {
            final_detections.push_back(candidates[idx]);
        }

        // Geração das máscaras de segmentação de instância via matmul com Proto (Tensor 0)
        generate_instance_masks(final_detections, out_ptrs[0], m_out_quant[0],
                                orig_bgr.size(), info, mask_threshold);

        // Desmapear saídas
        for (uint32_t i = 0; i < m_out_count; ++i) {
            vip_unmap_buffer(m_out_bufs[i]);
        }

        // Renderizar bounding boxes e sobreposição de máscaras
        render_results(orig_bgr, final_detections, annotated_img);
    }

private:
    void decode_head_scale(
        int grid_size, int stride,
        const int8_t* box_buf,   const QuantParam& q_box,
        const int8_t* cls_buf,   const QuantParam& q_cls,
        const int8_t* mask_buf,  const QuantParam& q_mask,
        float conf_threshold,
        std::vector<Detection>& candidates,
        float& global_max_score, int& global_max_cls)
    {
        for (int h = 0; h < grid_size; ++h) {
            for (int w = 0; w < grid_size; ++w) {
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

                if (max_score > global_max_score) {
                    global_max_score = max_score;
                    global_max_cls = max_cls_id;
                }

                if (max_score < conf_threshold) continue;

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

                float cx = (w + 0.5f);
                float cy = (h + 0.5f);
                float x1 = (cx - dfl_dist[0]) * stride;
                float y1 = (cy - dfl_dist[1]) * stride;
                float x2 = (cx + dfl_dist[2]) * stride;
                float y2 = (cy + dfl_dist[3]) * stride;

                Detection det;
                det.box = cv::Rect2f(x1, y1, std::max(0.0f, x2 - x1), std::max(0.0f, y2 - y1));
                det.class_id = max_cls_id;
                det.score = max_score;

                det.mask_coef.resize(NUM_MASKS);
                for (int m = 0; m < NUM_MASKS; ++m) {
                    int idx = (m * grid_size * grid_size) + (h * grid_size) + w;
                    det.mask_coef[m] = dequantize(mask_buf[idx], q_mask);
                }

                candidates.push_back(det);
            }
        }
    }

    void generate_instance_masks(
        std::vector<Detection>& detections,
        const int8_t* proto_raw,
        const QuantParam& q_proto,
        const cv::Size& orig_size,
        const LetterboxInfo& info,
        float mask_threshold)
    {
        if (detections.empty()) return;

        std::vector<float> proto_float(NUM_MASKS * PROTO_H * PROTO_W);
        for (size_t i = 0; i < proto_float.size(); ++i) {
            proto_float[i] = dequantize(proto_raw[i], q_proto);
        }

        for (auto& det : detections) {
            cv::Mat mask_104 = cv::Mat::zeros(PROTO_H, PROTO_W, CV_32FC1);

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

            cv::Mat mask_bin(PROTO_H, PROTO_W, CV_8UC1);
            for (int h = 0; h < PROTO_H; ++h) {
                const float* m_row = mask_104.ptr<float>(h);
                uint8_t* b_row = mask_bin.ptr<uint8_t>(h);
                for (int w = 0; w < PROTO_W; ++w) {
                    b_row[w] = (sigmoid(m_row[w]) > mask_threshold) ? 255 : 0;
                }
            }

            cv::Mat mask_416;
            cv::resize(mask_bin, mask_416, cv::Size(INPUT_W, INPUT_H), 0, 0, cv::INTER_LINEAR);

            cv::Rect roi = det.box;
            roi = roi & cv::Rect(0, 0, INPUT_W, INPUT_H);
            cv::Mat cropped_mask = cv::Mat::zeros(INPUT_H, INPUT_W, CV_8UC1);
            if (roi.width > 0 && roi.height > 0) {
                mask_416(roi).copyTo(cropped_mask(roi));
            }

            int orig_x = std::clamp(static_cast<int>((det.box.x - info.pad_x) / info.scale), 0, orig_size.width);
            int orig_y = std::clamp(static_cast<int>((det.box.y - info.pad_y) / info.scale), 0, orig_size.height);
            int orig_w = std::clamp(static_cast<int>(det.box.width / info.scale), 0, orig_size.width - orig_x);
            int orig_h = std::clamp(static_cast<int>(det.box.height / info.scale), 0, orig_size.height - orig_y);
            det.box = cv::Rect2f(orig_x, orig_y, orig_w, orig_h);

            int valid_w = std::min(info.new_w, INPUT_W - info.pad_x);
            int valid_h = std::min(info.new_h, INPUT_H - info.pad_y);
            cv::Rect valid_rect(info.pad_x, info.pad_y, valid_w, valid_h);
            cv::Mat unpad_mask = cropped_mask(valid_rect);
            cv::resize(unpad_mask, det.mask, orig_size, 0, 0, cv::INTER_NEAREST);
        }
    }

    void render_results(const cv::Mat& src, const std::vector<Detection>& detections, cv::Mat& dst) {
        dst = src.clone();
        cv::Mat overlay = src.clone();

        for (const auto& det : detections) {
            cv::Scalar color(
                (det.class_id * 67 + 43) % 256,
                (det.class_id * 131 + 89) % 256,
                (det.class_id * 197 + 137) % 256
            );

            if (!det.mask.empty() && det.mask.size() == dst.size()) {
                overlay.setTo(color, det.mask);
            }

            cv::rectangle(dst, det.box, color, 2);

            std::string cls_name = (det.class_id >= 0 && det.class_id < static_cast<int>(COCO_CLASSES.size()))
                ? COCO_CLASSES[det.class_id] : ("Class_" + std::to_string(det.class_id));
            std::string label = cls_name + " " + cv::format("%.2f", det.score);

            int baseline = 0;
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            int text_y = std::max(static_cast<int>(det.box.y), text_size.height + 4);
            cv::rectangle(dst, cv::Point(det.box.x, text_y - text_size.height - 4),
                          cv::Point(det.box.x + text_size.width + 4, text_y + 2), color, cv::FILLED);
            cv::putText(dst, label, cv::Point(det.box.x + 2, text_y - 2),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }
        cv::addWeighted(dst, 0.65, overlay, 0.35, 0, dst);
    }

    void cleanup() {
        if (!m_initialized) return;
        for (uint32_t i = 0; i < m_out_count; ++i) {
            if (m_out_bufs[i]) {
                vip_destroy_buffer(m_out_bufs[i]);
                m_out_bufs[i] = nullptr;
            }
        }
        if (m_input_buf) {
            vip_destroy_buffer(m_input_buf);
            m_input_buf = nullptr;
        }
        if (m_network) {
            vip_finish_network(m_network);
            vip_destroy_network(m_network);
            m_network = nullptr;
        }
        vip_destroy();
        m_initialized = false;
    }

    bool m_initialized = false;
    std::vector<char> m_nb_buffer;
    vip_network m_network = nullptr;
    vip_buffer m_input_buf = nullptr;
    uint32_t m_out_count = 0;
    vip_buffer m_out_bufs[10] = {nullptr};
    QuantParam m_out_quant[10] = {};
};

// -------------------------------------------------------------
// Nó ROS 2 para Subscrição e Inferência
// -------------------------------------------------------------
class YoloSegRosNode : public rclcpp::Node {
public:
    YoloSegRosNode(std::shared_ptr<VipLiteYoloSegEngine> engine,
                   const std::string& topic_name,
                   const std::string& mask_topic,
                   const std::string& output_path,
                   float conf_th, float nms_th,
                   int warmup_frames,
                   bool single_frame)
        : Node("yolo_seg_node"),
          m_engine(engine),
          m_mask_topic(mask_topic),
          m_output_path(output_path),
          m_conf_th(conf_th),
          m_nms_th(nms_th),
          m_warmup_frames(warmup_frames),
          m_single_frame(single_frame),
          m_frame_counter(0),
          m_processed_count(0)
    {
        RCLCPP_INFO(this->get_logger(), "Aguardando frames do topico ROS 2: %s (warmup: %d frames)",
                    topic_name.c_str(), warmup_frames);
        RCLCPP_INFO(this->get_logger(), "Publicador de mascara dinamica ativado em: %s",
                    mask_topic.c_str());

        m_pub_mask = this->create_publisher<sensor_msgs::msg::Image>(
            mask_topic, rclcpp::SensorDataQoS()
        );

        m_sub = this->create_subscription<sensor_msgs::msg::Image>(
            topic_name,
            rclcpp::SensorDataQoS(),
            std::bind(&YoloSegRosNode::imageCallback, this, std::placeholders::_1)
        );
    }

    bool hasProcessed() const {
        return m_processed_count > 0;
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (m_single_frame && m_processed_count > 0) return;

        m_frame_counter++;
        if (m_frame_counter <= m_warmup_frames) {
            RCLCPP_INFO(this->get_logger(), "[Warmup] Descartando frame %d/%d para estabilizacao de exposicao...",
                        m_frame_counter, m_warmup_frames);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "-> Processando Frame %d: %ux%u (encoding=%s, step=%u)",
                    m_frame_counter, msg->width, msg->height, msg->encoding.c_str(), msg->step);

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erro cv_bridge: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Frame vazio recebido!");
            return;
        }

        auto t0 = std::chrono::high_resolution_clock::now();

        // 1. Pre-processamento
        LetterboxInfo info;
        m_engine->preprocess(frame, info);

        auto t1 = std::chrono::high_resolution_clock::now();

        // 2. Inferencia na NPU
        if (!m_engine->infer()) {
            RCLCPP_ERROR(this->get_logger(), "Falha durante inferencia na NPU!");
            return;
        }

        auto t2 = std::chrono::high_resolution_clock::now();

        // 3. Pos-processamento
        std::vector<Detection> detections;
        cv::Mat annotated;
        std::string debug_top_candidate = "nenhum";
        m_engine->postprocess(frame, info, m_conf_th, m_nms_th, 0.5f, detections, annotated, debug_top_candidate);

        auto t3 = std::chrono::high_resolution_clock::now();

        double t_pre = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double t_npu = std::chrono::duration<double, std::milli>(t2 - t1).count();
        double t_post = std::chrono::duration<double, std::milli>(t3 - t2).count();
        double t_total = std::chrono::duration<double, std::milli>(t3 - t0).count();

        // 4. Construção da máscara binária para o filtro de odometria 3D
        cv::Mat dynamic_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        int dynamic_count = 0;
        for (const auto& det : detections) {
            if (is_dynamic_class(det.class_id) && !det.mask.empty()) {
                if (det.mask.size() == dynamic_mask.size()) {
                    cv::bitwise_or(dynamic_mask, det.mask, dynamic_mask);
                    dynamic_count++;
                }
            }
        }

        // Garante dimensão 640x480 da Astra Pro
        if (dynamic_mask.cols != 640 || dynamic_mask.rows != 480) {
            cv::resize(dynamic_mask, dynamic_mask, cv::Size(640, 480), 0, 0, cv::INTER_NEAREST);
        }

        // Publica máscara para o PointcloudFilterComponent
        if (m_pub_mask) {
            sensor_msgs::msg::Image::SharedPtr mask_msg =
                cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, dynamic_mask).toImageMsg();
            m_pub_mask->publish(*mask_msg);
        }

        if (m_single_frame) {
            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), "INFERENCIA CONCLUIDA COM SUCESSO!");
            RCLCPP_INFO(this->get_logger(), "  Pre-proc (Letterbox + Quant): %.2f ms", t_pre);
            RCLCPP_INFO(this->get_logger(), "  NPU Inferencia (VIP9000):     %.2f ms", t_npu);
            RCLCPP_INFO(this->get_logger(), "  Pos-proc (DFL, NMS, Mascaras): %.2f ms", t_post);
            RCLCPP_INFO(this->get_logger(), "  Tempo Total:                  %.2f ms (%.1f FPS)", t_total, 1000.0 / t_total);
            RCLCPP_INFO(this->get_logger(), "  Deteccoes encontradas:         %zu (conf >= %.2f)", detections.size(), m_conf_th);

            if (detections.empty()) {
                RCLCPP_INFO(this->get_logger(), "  (Maior confianca no frame: %s)", debug_top_candidate.c_str());
            }

            for (size_t i = 0; i < detections.size(); ++i) {
                const auto& det = detections[i];
                std::string name = (det.class_id >= 0 && det.class_id < static_cast<int>(COCO_CLASSES.size()))
                    ? COCO_CLASSES[det.class_id] : ("Class_" + std::to_string(det.class_id));
                RCLCPP_INFO(this->get_logger(), "    [%zu] %s (score: %.1f%%) em [%.0f, %.0f, %.0f, %.0f]",
                            i + 1, name.c_str(), det.score * 100.0f,
                            det.box.x, det.box.y, det.box.width, det.box.height);
            }

            if (cv::imwrite(m_output_path, annotated)) {
                RCLCPP_INFO(this->get_logger(), "Imagem anotada salva em: %s", m_output_path.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Falha ao gravar imagem em: %s", m_output_path.c_str());
            }
            RCLCPP_INFO(this->get_logger(), "========================================");
        } else {
            // Em modo contínuo: log throttled a cada ~1s (30 frames) sem escrita em disco para economizar SWaP
            if (m_frame_counter % 30 == 0) {
                RCLCPP_INFO(this->get_logger(),
                    "[YOLO VIP9000] Frame %d | NPU=%.1f ms | Total=%.1f ms (%.1f FPS) | Deteccoes=%zu | Dinamicos=%d",
                    m_frame_counter, t_npu, t_total, 1000.0 / t_total, detections.size(), dynamic_count);
            }
        }

        m_processed_count++;

        if (m_single_frame) {
            RCLCPP_INFO(this->get_logger(), "Modo single-frame: finalizando execucao do no.");
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<VipLiteYoloSegEngine> m_engine;
    std::string m_mask_topic;
    std::string m_output_path;
    float m_conf_th;
    float m_nms_th;
    int m_warmup_frames;
    bool m_single_frame;
    int m_frame_counter;
    int m_processed_count;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_mask;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_sub;
};

// -------------------------------------------------------------
// Ponto de Entrada (main)
// -------------------------------------------------------------
int main(int argc, char** argv) {
    std::string model_path = "/home/orangepi/yolov8n_seg_final.nb";
    std::string topic_name = "/camera/color/image_raw";
    std::string mask_topic = "/yolo/dynamic_mask";
    std::string output_path = "resultado_yolov8_seg.jpg";
    std::string static_img_path = "";
    float conf_th = 0.25f;
    float nms_th = 0.45f;
    int warmup_frames = 5;
    bool single_frame = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--model" && i + 1 < argc) model_path = argv[++i];
        else if (arg == "--topic" && i + 1 < argc) topic_name = argv[++i];
        else if (arg == "--mask-topic" && i + 1 < argc) mask_topic = argv[++i];
        else if (arg == "--output" && i + 1 < argc) output_path = argv[++i];
        else if (arg == "--image" && i + 1 < argc) static_img_path = argv[++i];
        else if (arg == "--conf" && i + 1 < argc) conf_th = std::stof(argv[++i]);
        else if (arg == "--nms" && i + 1 < argc) nms_th = std::stof(argv[++i]);
        else if (arg == "--warmup" && i + 1 < argc) warmup_frames = std::stoi(argv[++i]);
        else if (arg == "--continuous") single_frame = false;
        else if (arg == "--help" || arg == "-h") {
            std::cout << "Uso: " << argv[0] << " [opcoes]\n"
                      << "  --model <path>      Caminho para o arquivo .nb (padrao: " << model_path << ")\n"
                      << "  --topic <name>      Topico ROS 2 de imagem RGB (padrao: " << topic_name << ")\n"
                      << "  --mask-topic <name> Topico de mascara dinamica (padrao: " << mask_topic << ")\n"
                      << "  --output <path>     Arquivo de saida para imagem anotada (padrao: " << output_path << ")\n"
                      << "  --image <path>      Executa inferencia em arquivo de imagem estatica (sem camera ROS)\n"
                      << "  --conf <float>      Threshold de confianca (padrao: 0.25)\n"
                      << "  --nms <float>       Threshold de NMS (padrao: 0.45)\n"
                      << "  --warmup <int>      Frames iniciais descartados para auto-exposicao (padrao: 5)\n"
                      << "  --continuous        Modo continuo (processa e publica em tempo real sem salvar em disco)\n";
            return 0;
        }
    }

    auto engine = std::make_shared<VipLiteYoloSegEngine>();
    if (!engine->init(model_path)) {
        std::cerr << "Falha ao inicializar motor VIPLite com modelo: " << model_path << "\n";
        return 1;
    }

    // Modo Imagem Estatica (opcional para testes sem camera)
    if (!static_img_path.empty()) {
        std::cout << "[INFO] Processando imagem estatica: " << static_img_path << "\n";
        cv::Mat img = cv::imread(static_img_path);
        if (img.empty()) {
            std::cerr << "[ERROR] Nao foi possivel carregar a imagem: " << static_img_path << "\n";
            return 1;
        }

        LetterboxInfo info;
        engine->preprocess(img, info);
        engine->infer();
        std::vector<Detection> detections;
        cv::Mat annotated;
        std::string top_cand;
        engine->postprocess(img, info, conf_th, nms_th, 0.5f, detections, annotated, top_cand);

        std::cout << "Deteccoes encontradas: " << detections.size() << "\n";
        for (size_t i = 0; i < detections.size(); ++i) {
            std::string name = (detections[i].class_id >= 0 && detections[i].class_id < static_cast<int>(COCO_CLASSES.size()))
                ? COCO_CLASSES[detections[i].class_id] : ("Class_" + std::to_string(detections[i].class_id));
            std::cout << "  [" << i + 1 << "] " << name << " (" << detections[i].score * 100.0f << "%) em ["
                      << detections[i].box.x << ", " << detections[i].box.y << ", "
                      << detections[i].box.width << ", " << detections[i].box.height << "]\n";
        }
        cv::imwrite(output_path, annotated);
        std::cout << "Resultado gravado em: " << output_path << "\n";
        return 0;
    }

    // Modo ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YoloSegRosNode>(engine, topic_name, mask_topic, output_path, conf_th, nms_th, warmup_frames, single_frame);
    rclcpp::spin(node);

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}

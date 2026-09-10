#include "pointcloud_odometry/pointcloud_filter_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cstring>

namespace pointcloud_odometry {

PointcloudFilterComponent::PointcloudFilterComponent(const rclcpp::NodeOptions& options)
    : Node("pointcloud_filter_component", options) {
    // Declare and get parameters (Default stride changed to 8 as requested)
    input_cloud_topic_ = this->declare_parameter<std::string>("input_cloud_topic", "/camera/depth/points");
    output_cloud_topic_ = this->declare_parameter<std::string>("output_cloud_topic", "/camera/filtered_points");
    mask_topic_ = this->declare_parameter<std::string>("mask_topic", "/yolo/dynamic_mask");
    stride_u_ = this->declare_parameter<int>("stride_u", 8);
    stride_v_ = this->declare_parameter<int>("stride_v", 8);
    min_range_ = this->declare_parameter<double>("min_range", 0.4);
    max_range_ = this->declare_parameter<double>("max_range", 3.8);
    enable_dynamic_filter_ = this->declare_parameter<bool>("enable_dynamic_filter", false);
    mask_timeout_sec_ = this->declare_parameter<double>("mask_timeout_sec", 0.5);

    point_buffer_.reserve(5000 * 3); // Pre-allocate for ~3k-5k points (x, y, z)

    // Publishers and Subscribers
    rclcpp::SensorDataQoS sensor_qos;
    pub_filtered_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_cloud_topic_, sensor_qos);

    rclcpp::QoS camera_sub_qos(5);
    camera_sub_qos.reliable();
    camera_sub_qos.durability_volatile();

    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, camera_sub_qos,
        std::bind(&PointcloudFilterComponent::pointCloudCallback, this, std::placeholders::_1));

    if (enable_dynamic_filter_) {
        sub_mask_ = this->create_subscription<sensor_msgs::msg::Image>(
            mask_topic_, sensor_qos,
            std::bind(&PointcloudFilterComponent::maskCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Filtro dinamico ATIVADO. Aguardando mascaras em: %s", mask_topic_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Filtro dinamico DESATIVADO por padrao (apenas decimação e range pass-through).");
    }

    RCLCPP_INFO(this->get_logger(),
        "PointcloudFilterComponent inicializado: in=%s, out=%s, stride=(%d, %d), range=[%.2f, %.2f] m",
        input_cloud_topic_.c_str(), output_cloud_topic_.c_str(), stride_u_, stride_v_, min_range_, max_range_);
}

void PointcloudFilterComponent::maskCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(mask_mutex_);
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "mono8");
        latest_mask_ = cv_ptr->image.clone();
        latest_mask_stamp_ = msg->header.stamp;
        has_mask_ = true;
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Erro ao converter mascara via cv_bridge: %s", e.what());
    }
}

bool PointcloudFilterComponent::isDynamicPixel(uint32_t v, uint32_t u, const builtin_interfaces::msg::Time& stamp) {
    if (!enable_dynamic_filter_) return false;

    std::lock_guard<std::mutex> lock(mask_mutex_);
    if (!has_mask_ || latest_mask_.empty()) return false;

    double t_cloud = stamp.sec + stamp.nanosec * 1e-9;
    double t_mask = latest_mask_stamp_.sec + latest_mask_stamp_.nanosec * 1e-9;
    if (std::abs(t_cloud - t_mask) > mask_timeout_sec_) {
        return false;
    }

    if (v < static_cast<uint32_t>(latest_mask_.rows) && u < static_cast<uint32_t>(latest_mask_.cols)) {
        return latest_mask_.at<uint8_t>(v, u) > 0;
    }
    return false;
}

void PointcloudFilterComponent::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    auto t_start = std::chrono::high_resolution_clock::now();

    if (msg->data.empty() || msg->point_step == 0) {
        return;
    }

    // Localizar rigorosamente os campos x, y, z e seus datatypes
    int offset_x = -1, offset_y = -1, offset_z = -1;
    uint8_t datatype_x = 0, datatype_y = 0, datatype_z = 0;
    for (const auto& field : msg->fields) {
        std::string fname = field.name;
        std::transform(fname.begin(), fname.end(), fname.begin(), ::tolower);
        if (fname == "x") { offset_x = field.offset; datatype_x = field.datatype; }
        else if (fname == "y") { offset_y = field.offset; datatype_y = field.datatype; }
        else if (fname == "z") { offset_z = field.offset; datatype_z = field.datatype; }
    }

    if (offset_x < 0 || offset_y < 0 || offset_z < 0) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Campos x, y ou z nao encontrados no PointCloud2 recebido!");
        return;
    }

    // Helper lambda para extrair float independente do tipo subjacente (FLOAT32, FLOAT64, INT16, UINT16)
    auto read_coord = [](const uint8_t* ptr, uint8_t datatype) -> float {
        if (datatype == sensor_msgs::msg::PointField::FLOAT32) {
            return *reinterpret_cast<const float*>(ptr);
        } else if (datatype == sensor_msgs::msg::PointField::FLOAT64) {
            return static_cast<float>(*reinterpret_cast<const double*>(ptr));
        } else if (datatype == sensor_msgs::msg::PointField::UINT16) {
            return static_cast<float>(*reinterpret_cast<const uint16_t*>(ptr));
        } else if (datatype == sensor_msgs::msg::PointField::INT16) {
            return static_cast<float>(*reinterpret_cast<const int16_t*>(ptr));
        }
        return *reinterpret_cast<const float*>(ptr);
    };

    point_buffer_.clear();

    const uint32_t width = msg->width;
    const uint32_t height = msg->height;
    const uint32_t point_step = msg->point_step;
    const uint32_t row_step = msg->row_step;
    const uint8_t* raw_data = msg->data.data();
    const size_t total_data_size = msg->data.size();

    float min_x = 1e9f, max_x = -1e9f;
    float min_y = 1e9f, max_y = -1e9f;
    float min_z = 1e9f, max_z = -1e9f;


    if (height > 1) {
        // Nuvem organizada (640x480 da Astra Pro): Decimação direta 2D por pixel striding
        for (uint32_t v = 0; v < height; v += stride_v_) {
            size_t row_offset = static_cast<size_t>(v) * row_step;
            if (row_offset >= total_data_size) break;

            const uint8_t* row_ptr = raw_data + row_offset;
            for (uint32_t u = 0; u < width; u += stride_u_) {
                size_t pt_offset = static_cast<size_t>(u) * point_step;
                if (row_offset + pt_offset + 12 > total_data_size) continue;

                const uint8_t* pt_ptr = row_ptr + pt_offset;
                float x = read_coord(pt_ptr + offset_x, datatype_x);
                float y = read_coord(pt_ptr + offset_y, datatype_y);
                float z = read_coord(pt_ptr + offset_z, datatype_z);

                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

                // Verificação automática de escala (se vier em milímetros, ex: z > 20.0m em câmera de alcance 4m)
                if (z > 20.0f) {
                    x *= 0.001f;
                    y *= 0.001f;
                    z *= 0.001f;
                }

                // Range Pass-Through em metros
                if (z < min_range_ || z > max_range_) continue;

                if (enable_dynamic_filter_ && isDynamicPixel(v, u, msg->header.stamp)) {
                    continue; // Ponto dinâmico rejeitado pela máscara
                }

                point_buffer_.push_back(x);
                point_buffer_.push_back(y);
                point_buffer_.push_back(z);

                min_x = std::min(min_x, x); max_x = std::max(max_x, x);
                min_y = std::min(min_y, y); max_y = std::max(max_y, y);
                min_z = std::min(min_z, z); max_z = std::max(max_z, z);
            }
        }
    } else {
        // Nuvem desorganizada (fallback): decimar linearmente
        int step_factor = std::max(1, (stride_u_ * stride_v_) / 2);
        size_t total_points = width * height;
        for (size_t i = 0; i < total_points; i += step_factor) {
            size_t pt_offset = i * point_step;
            if (pt_offset + 12 > total_data_size) break;

            const uint8_t* pt_ptr = raw_data + pt_offset;
            float x = read_coord(pt_ptr + offset_x, datatype_x);
            float y = read_coord(pt_ptr + offset_y, datatype_y);
            float z = read_coord(pt_ptr + offset_z, datatype_z);

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            if (z > 20.0f) { x *= 0.001f; y *= 0.001f; z *= 0.001f; }
            if (z < min_range_ || z > max_range_) continue;

            point_buffer_.push_back(x);
            point_buffer_.push_back(y);
            point_buffer_.push_back(z);

            min_x = std::min(min_x, x); max_x = std::max(max_x, x);
            min_y = std::min(min_y, y); max_y = std::max(max_y, y);
            min_z = std::min(min_z, z); max_z = std::max(max_z, z);
        }
    }

    size_t num_filtered_points = point_buffer_.size() / 3;

    // Criar mensagem de saída PointCloud2 de alta performance (cópia direta de memória)
    auto out_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out_msg->header = msg->header;
    out_msg->height = 1;
    out_msg->width = static_cast<uint32_t>(num_filtered_points);
    out_msg->is_dense = true;
    out_msg->is_bigendian = false;

    out_msg->fields.resize(3);
    out_msg->fields[0].name = "x";
    out_msg->fields[0].offset = 0;
    out_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out_msg->fields[0].count = 1;

    out_msg->fields[1].name = "y";
    out_msg->fields[1].offset = 4;
    out_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out_msg->fields[1].count = 1;

    out_msg->fields[2].name = "z";
    out_msg->fields[2].offset = 8;
    out_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out_msg->fields[2].count = 1;

    out_msg->point_step = 12; // 3 floats * 4 bytes
    out_msg->row_step = out_msg->point_step * out_msg->width;
    out_msg->data.resize(out_msg->row_step);

    if (num_filtered_points > 0) {
        std::memcpy(out_msg->data.data(), point_buffer_.data(), num_filtered_points * 3 * sizeof(float));
    }

    pub_filtered_cloud_->publish(std::move(out_msg));

    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    // Log de debug temporário com Bounding Box para diagnosticar escala e colapso de voxels
    if (num_filtered_points > 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[Filter BBox] Pts=%zu (raw %ux%u) | X=[%.3f, %.3f] Y=[%.3f, %.3f] Z=[%.3f, %.3f] m | Tempo=%.2f ms",
            num_filtered_points, width, height, min_x, max_x, min_y, max_y, min_z, max_z, elapsed_ms);
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[Filter WARN] 0 pontos validos apos filtragem! Verifique conexao e exposicao da camera.");
    }
}

} // namespace pointcloud_odometry

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_odometry::PointcloudFilterComponent)

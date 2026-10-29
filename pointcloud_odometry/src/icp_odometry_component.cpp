#include "pointcloud_odometry/icp_odometry_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <small_gicp/ros/ros2.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/util/fast_floor.hpp>
#include <small_gicp/util/downsampling_omp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <fstream>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <cmath>

namespace pointcloud_odometry {

IcpOdometryComponent::IcpOdometryComponent(const rclcpp::NodeOptions& options)
    : Node("icp_odometry_component", options) {
    // Parameters declaration
    input_cloud_topic_ = this->declare_parameter<std::string>("input_cloud_topic", "/camera/filtered_points");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/icp/odom");
    initial_guess_topic_ = this->declare_parameter<std::string>("initial_guess_topic", "/odometry/filtered");
    odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_link");
    publish_tf_ = this->declare_parameter<bool>("publish_tf", false); // RIGIDLY FALSE by default to avoid collision with robot_localization
    use_tf_extrinsic_ = this->declare_parameter<bool>("use_tf_extrinsic", false);
    voxel_resolution_ = this->declare_parameter<double>("voxel_resolution", 0.15); // 15 cm otimizado para ruído de profundidade RGB-D indoor
    keyframe_delta_trans_ = this->declare_parameter<double>("keyframe_delta_trans", 0.15); // 15 cm
    keyframe_delta_rot_rad_ = this->declare_parameter<double>("keyframe_delta_rot_deg", 8.0) * M_PI / 180.0; // 8 degrees
    num_threads_ = this->declare_parameter<int>("num_threads", 4);
    max_iterations_ = this->declare_parameter<int>("max_iterations", 30);
    max_corr_dist_ = this->declare_parameter<double>("max_corr_dist", 0.30);
    default_map_path_ = this->declare_parameter<std::string>("default_map_path", "/home/orangepi/submap_cloud.ply");
    save_on_shutdown_ = this->declare_parameter<bool>("save_on_shutdown", false);
    publish_submap_cloud_ = this->declare_parameter<bool>("publish_submap_cloud", true);
    submap_cloud_topic_ = this->declare_parameter<std::string>("submap_cloud_topic", "/icp/submap_cloud");
    submap_publish_interval_sec_ = this->declare_parameter<double>("submap_publish_interval_sec", 1.0);

    // Communication setup
    rclcpp::SensorDataQoS sensor_qos;
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    pub_telemetry_ = this->create_publisher<std_msgs::msg::String>("/icp/telemetry", 10);
    if (publish_submap_cloud_) {
        pub_submap_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(submap_cloud_topic_, 10);
    }

    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, sensor_qos,
        std::bind(&IcpOdometryComponent::cloudCallback, this, std::placeholders::_1));

    sub_initial_guess_ = this->create_subscription<nav_msgs::msg::Odometry>(
        initial_guess_topic_, 10,
        std::bind(&IcpOdometryComponent::initialGuessCallback, this, std::placeholders::_1));

    // Services to export pointcloud map for scientific paper figures
    srv_save_map_ = this->create_service<std_srvs::srv::Trigger>(
        "/icp/save_map",
        std::bind(&IcpOdometryComponent::handleSaveMap, this, std::placeholders::_1, std::placeholders::_2));

    srv_save_submap_ = this->create_service<std_srvs::srv::Trigger>(
        "/icp/save_submap",
        std::bind(&IcpOdometryComponent::handleSaveMap, this, std::placeholders::_1, std::placeholders::_2));

    srv_save_map_alias_ = this->create_service<std_srvs::srv::Trigger>(
        "/save_pointcloud_map",
        std::bind(&IcpOdometryComponent::handleSaveMap, this, std::placeholders::_1, std::placeholders::_2));

    // TF Broadcaster & Listener
    if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        RCLCPP_WARN(this->get_logger(), "ATENCAO: publish_tf está ATIVADO. Certifique-se de que nao haja conflito com o robot_localization!");
    } else {
        RCLCPP_INFO(this->get_logger(), "TF Publishing DESATIVADO (Seguro: evita colisão destrutiva com EKF das rodas Mecanum).");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(),
        "IcpOdometryComponent inicializado: in=%s, out=%s, guess=%s, submap_res=%.2fm, kf=(%.2fm, %.1f deg)",
        input_cloud_topic_.c_str(), odom_topic_.c_str(), initial_guess_topic_.c_str(),
        voxel_resolution_, keyframe_delta_trans_, keyframe_delta_rot_rad_ * 180.0 / M_PI);
}

IcpOdometryComponent::~IcpOdometryComponent() {
    if (save_on_shutdown_ && !accumulated_map_points_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Shutdown acionado: salvando submapa consolidado...");
        saveMapToFile(default_map_path_);
    }
}

void IcpOdometryComponent::initialGuessCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    pose.linear() = Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z).toRotationMatrix();

    initial_guess_buffer_.push_back({t, pose});
    if (initial_guess_buffer_.size() > 500) {
        initial_guess_buffer_.pop_front();
    }
}

bool IcpOdometryComponent::computeInitialGuess(double current_stamp, Eigen::Isometry3d& guess_pose) {
    std::lock_guard<std::mutex> lock(odom_mutex_);

    if (initial_guess_buffer_.size() >= 2 && last_cloud_stamp_ > 0.0) {
        auto find_closest = [this](double target_t) -> const OdomSample* {
            const OdomSample* best = nullptr;
            double min_diff = std::numeric_limits<double>::max();
            for (const auto& sample : initial_guess_buffer_) {
                double diff = std::abs(sample.timestamp - target_t);
                if (diff < min_diff) {
                    min_diff = diff;
                    best = &sample;
                }
            }
            // Defasagem máxima de 100 ms (0.10s) em relação ao carimbo da câmera
            if (min_diff <= 0.10) return best;
            return nullptr;
        };

        const OdomSample* s_prev = find_closest(last_cloud_stamp_);
        const OdomSample* s_curr = find_closest(current_stamp);

        if (s_prev && s_curr) {
            Eigen::Isometry3d delta_wheel = s_prev->pose.inverse() * s_curr->pose;
            double step_wheel_trans = delta_wheel.translation().norm();
            // Validar consistência do incremento das rodas (< 0.10m em 33ms)
            if (step_wheel_trans < 0.10 && delta_wheel.matrix().allFinite()) {
                guess_pose = current_pose_ * delta_wheel;
                return true;
            }
        }
    }

    // Se a odometria de rodas estiver ausente ou defasada em mais de 100 ms,
    // usar explicitamente a identidade como incremento relativo (guess = current_pose_),
    // evitando que buffers vazios ou extrapolações cinemáticas injetem saltos na otimização.
    guess_pose = current_pose_;
    return false;
}

void IcpOdometryComponent::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    auto t0 = std::chrono::high_resolution_clock::now();
    double current_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    // 1. Converter PointCloud2 para small_gicp::PointCloud
    small_gicp::PointCloud::Ptr source_cloud = small_gicp::from_ros_msg(*msg);
    if (!source_cloud || source_cloud->empty() || source_cloud->size() < 40) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "[ICP] Nuvem de pontos recebida possui dados insuficientes (%zu pts). Ignorando.",
            source_cloud ? source_cloud->size() : 0);
        return;
    }

    // Calcular bounding box do source_cloud para diagnóstico
    float min_x = 1e9f, max_x = -1e9f;
    float min_y = 1e9f, max_y = -1e9f;
    float min_z = 1e9f, max_z = -1e9f;
    for (size_t i = 0; i < source_cloud->size(); ++i) {
        const auto& pt = source_cloud->points[i];
        min_x = std::min(min_x, static_cast<float>(pt.x()));
        max_x = std::max(max_x, static_cast<float>(pt.x()));
        min_y = std::min(min_y, static_cast<float>(pt.y()));
        max_y = std::max(max_y, static_cast<float>(pt.y()));
        min_z = std::min(min_z, static_cast<float>(pt.z()));
        max_z = std::max(max_z, static_cast<float>(pt.z()));
    }

    // 2. Transformar nuvem para base_frame_id se extrínseca estiver ativa
    if (use_tf_extrinsic_) {
        try {
            geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
                base_frame_id_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.05));
            Eigen::Isometry3d T_base_cam = tf2::transformToEigen(tf_stamped.transform);
            for (size_t i = 0; i < source_cloud->size(); ++i) {
                source_cloud->points[i] = T_base_cam * source_cloud->points[i];
            }
        } catch (const tf2::TransformException& ex) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[ICP] TF lookup %s -> %s falhou: %s. Utilizando frame nativo.",
                base_frame_id_.c_str(), msg->header.frame_id.c_str(), ex.what());
        }
    }

    // 3. Estimar normais e covariâncias (OpenMP multithreaded)
    int k_neighbors = std::min(20, static_cast<int>(source_cloud->size()) - 1);
    small_gicp::estimate_normals_covariances_omp(*source_cloud, k_neighbors, num_threads_);

    // 4. Inicialização do Primeiro Frame
    if (is_first_frame_) {
        submap_voxelmap_ = std::make_shared<small_gicp::GaussianVoxelMap>(voxel_resolution_);
        current_pose_ = Eigen::Isometry3d::Identity();
        last_pose_ = current_pose_;
        last_keyframe_pose_ = current_pose_;

        submap_voxelmap_->insert(*source_cloud, current_pose_);
        keyframe_count_++;

        {
            std::lock_guard<std::mutex> lock(map_export_mutex_);
            for (size_t i = 0; i < source_cloud->size(); ++i) {
                Eigen::Vector4d pt = source_cloud->points[i];
                accumulated_map_points_.emplace_back(pt.x(), pt.y(), pt.z());
            }
        }

        is_first_frame_ = false;
        last_cloud_stamp_ = current_stamp;

        RCLCPP_INFO(this->get_logger(),
            "[ICP First Frame] Inseridos %zu pontos -> Submapa consolidou %zu voxels (resolucao=%.2fm). BBox: X=[%.3f, %.3f] Y=[%.3f, %.3f] Z=[%.3f, %.3f] m",
            source_cloud->size(), submap_voxelmap_->size(), voxel_resolution_, min_x, max_x, min_y, max_y, min_z, max_z);

        publishOdometry(msg->header.stamp, current_pose_, Eigen::Matrix<double, 6, 6>::Identity() * 100.0,
                        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

        if (publish_submap_cloud_) {
            publishSubmapCloud(msg->header.stamp);
        }
        return;
    }

    // 5. Calcular Chute Inicial (Initial Guess) via Sensor Fusion
    Eigen::Isometry3d guess_pose = current_pose_;
    bool used_wheel_guess = computeInitialGuess(current_stamp, guess_pose);

    // 6. Alinhamento Frame-to-Submap (VGICP via small_gicp)
    double align_max_corr_dist = max_corr_dist_; // 0.30 m por padrão
    if (tracking_state_ == TrackingState::LOST && relocalize_attempt_counter_ < 5) {
        align_max_corr_dist = 0.50; // Raio expandido para relocalização no submapa
        relocalize_attempt_counter_++;
    }


    small_gicp::RegistrationSetting setting;
    setting.type = small_gicp::RegistrationSetting::VGICP;
    setting.voxel_resolution = voxel_resolution_;
    setting.max_correspondence_distance = align_max_corr_dist;
    setting.num_threads = num_threads_;
    setting.max_iterations = max_iterations_;

    size_t submap_voxels_before_align = submap_voxelmap_->size();
    if (submap_voxels_before_align <= 5) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
            "[ICP ALERTA] Submapa com poucos voxels (%zu voxels)! Verifique se a resolucao (%.2fm) nao esta muito larga para o campo de visao.",
            submap_voxels_before_align, voxel_resolution_);
    }

    small_gicp::RegistrationResult result = small_gicp::align(*submap_voxelmap_, *source_cloud, guess_pose, setting);

    double dt = (last_cloud_stamp_ > 0.0) ? (current_stamp - last_cloud_stamp_) : 0.033;
    if (dt <= 0.0) dt = 0.033;

    Eigen::Vector3d linear_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();

    // 1. Validar convergência numérica básica do solver
    bool solver_ok = result.converged && result.T_target_source.matrix().allFinite();

    // 2. Validação Cinemática ANTES de atualizar pose ou avaliar Keyframe:
    double step_trans = (result.T_target_source.translation() - current_pose_.translation()).norm();
    Eigen::AngleAxisd step_aa(current_pose_.linear().transpose() * result.T_target_source.linear());
    double step_rot = std::abs(step_aa.angle());

    // Limites cinemáticos físicos para o robô a ~30Hz (dt ~ 33ms):
    // Durante relocalização (LOST), permite raio maior (até 0.30m e 25 deg) para reconectar ao submapa
    double max_allowed_trans = (tracking_state_ == TrackingState::LOST) ? 0.30 : (used_wheel_guess ? 0.20 : 0.12);
    double max_allowed_rot = (tracking_state_ == TrackingState::LOST) ? (25.0 * M_PI / 180.0) :
                             (used_wheel_guess ? (20.0 * M_PI / 180.0) : (15.0 * M_PI / 180.0));

    bool kinematic_sane = (step_trans <= max_allowed_trans) && (step_rot <= max_allowed_rot);
    double inlier_ratio = (source_cloud->size() > 0) ? (static_cast<double>(result.num_inliers) / source_cloud->size()) : 0.0;

    // Condição de frame aceitável para tracking
    bool is_good_frame = solver_ok && kinematic_sane && (inlier_ratio >= 0.25);

    // Gerenciamento de estado de Tracking e Relocalização
    if (is_good_frame) {
        lost_frames_counter_ = 0;
        if (tracking_state_ == TrackingState::LOST) {
            RCLCPP_INFO(this->get_logger(),
                "============================================================");
            RCLCPP_INFO(this->get_logger(),
                "[ICP RECOVERY] Relocalizado com sucesso contra o submapa! Inliers=%.0f%% | Submap Voxels=%zu",
                inlier_ratio * 100.0, submap_voxelmap_->size());
            RCLCPP_INFO(this->get_logger(),
                "============================================================");
            tracking_state_ = TrackingState::TRACKING;
            relocalize_attempt_counter_ = 0;
        }
    } else {
        lost_frames_counter_++;
        if (lost_frames_counter_ > 10) {
            if (tracking_state_ != TrackingState::LOST) {
                RCLCPP_WARN(this->get_logger(),
                    "[ICP TRACKING_LOST] Perda de rastreio detectada (>10 frames consecutivos com inliers < 25%%)! Ativando busca com raio expandido (0.50m)...");
                tracking_state_ = TrackingState::LOST;
                relocalize_attempt_counter_ = 0;
            }
        }
    }

    bool valid_convergence = is_good_frame;

    if (valid_convergence) {
        last_pose_ = current_pose_;
        current_pose_ = result.T_target_source;

        linear_vel = (current_pose_.translation() - last_pose_.translation()) / dt;
        Eigen::AngleAxisd delta_rot(last_pose_.linear().transpose() * current_pose_.linear());
        angular_vel = delta_rot.axis() * (delta_rot.angle() / dt);
    } else {
        if (!kinematic_sane && solver_ok) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[ICP WARN] Salto cinematico anomalo descartado! step_d=%.3fm (> %.2fm), step_rot=%.1f deg. Mantendo pose anterior.",
                step_trans, max_allowed_trans, step_rot * 180.0 / M_PI);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                "[ICP WARN] small_gicp nao convergiu confiavelmente! Inliers: %zu (%.0f%%), Submap voxels: %zu%s. Mantendo pose anterior.",
                result.num_inliers, inlier_ratio * 100.0, submap_voxelmap_->size(),
                (tracking_state_ == TrackingState::LOST ? " [STATE: LOST]" : ""));
        }
        // Pose mantida intacta; nenhum ponto espúrio será inserido no submapa
    }

    // 7. Avaliar Critério de Keyframe para Submapa em RAM
    // Condição para adicionar um novo Keyframe ao GaussianVoxelMap:
    // 1) O ICP convergiu e passou na validação cinemática (valid_convergence == true);
    // 2) Inliers mínimos razoáveis para aceitar o frame: inlier_ratio > 0.35 (35%);
    // 3) Deslocamento atingido: delta_d > 0.15m OU delta_theta > 8.0 deg em relação ao último keyframe.
    bool can_be_keyframe = valid_convergence && (inlier_ratio > 0.35);

    if (can_be_keyframe) {
        double delta_dist = (current_pose_.translation() - last_keyframe_pose_.translation()).norm();
        Eigen::AngleAxisd aa(last_keyframe_pose_.linear().transpose() * current_pose_.linear());
        double delta_rot = std::abs(aa.angle());

        if (delta_dist > keyframe_delta_trans_ || delta_rot > keyframe_delta_rot_rad_) {
            submap_voxelmap_->insert(*source_cloud, current_pose_);
            last_keyframe_pose_ = current_pose_;
            keyframe_count_++;

            std::lock_guard<std::mutex> lock(map_export_mutex_);
            for (size_t i = 0; i < source_cloud->size(); ++i) {
                Eigen::Vector4d world_pt = current_pose_ * source_cloud->points[i];
                accumulated_map_points_.emplace_back(world_pt.x(), world_pt.y(), world_pt.z());
            }

            // Otimização SWaP: Mantém a memória de pontos acumulados limitada (< 5 MB RAM)
            // aplicando voxelgrid downsampling in-place a cada 80.000 pontos
            if (accumulated_map_points_.size() > 80000) {
                small_gicp::PointCloud raw_cloud;
                raw_cloud.resize(accumulated_map_points_.size());
                for (size_t i = 0; i < accumulated_map_points_.size(); ++i) {
                    raw_cloud.points[i] = Eigen::Vector4d(accumulated_map_points_[i].x(), accumulated_map_points_[i].y(), accumulated_map_points_[i].z(), 1.0);
                }
                auto ds_cloud = small_gicp::voxelgrid_sampling_omp(raw_cloud, 0.05, num_threads_);
                accumulated_map_points_.clear();
                accumulated_map_points_.reserve(ds_cloud->size());
                for (size_t i = 0; i < ds_cloud->size(); ++i) {
                    accumulated_map_points_.emplace_back(ds_cloud->points[i].x(), ds_cloud->points[i].y(), ds_cloud->points[i].z());
                }
            }

            RCLCPP_INFO(this->get_logger(),
                "[Keyframe #%zu] delta_d=%.3fm, delta_theta=%.1f deg, inliers=%.0f%% -> Submap Voxels=%zu",
                keyframe_count_, delta_dist, delta_rot * 180.0 / M_PI, inlier_ratio * 100.0, submap_voxelmap_->size());

            if (publish_submap_cloud_) {
                rclcpp::Time current_time = msg->header.stamp;
                if ((current_time - last_submap_pub_time_).seconds() >= submap_publish_interval_sec_) {
                    publishSubmapCloud(current_time);
                }
            }
        }
    }

    // 8. Publicar Odometria ROS 2
    publishOdometry(msg->header.stamp, current_pose_, result.H, linear_vel, angular_vel);

    last_cloud_stamp_ = current_stamp;

    auto t1 = std::chrono::high_resolution_clock::now();
    double time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    double inlier_pct = (source_cloud->size() > 0) ? (100.0 * result.num_inliers / source_cloud->size()) : 0.0;

    // Monitoramento de Throughput Real (mensagens reais processadas por segundo)
    throughput_frame_count_++;
    auto now_tp = std::chrono::steady_clock::now();
    double tp_dt = std::chrono::duration<double>(now_tp - last_throughput_time_).count();
    if (tp_dt >= 1.0) {
        real_throughput_hz_ = throughput_frame_count_ / tp_dt;
        throughput_frame_count_ = 0;
        last_throughput_time_ = now_tp;
    }

    // Calcular Roll, Pitch, Yaw em graus a partir da matriz de rotação
    Eigen::Matrix3d R = current_pose_.linear();
    double roll_deg = std::atan2(R(2, 1), R(2, 2)) * 180.0 / M_PI;
    double pitch_deg = std::asin(std::clamp(-R(2, 0), -1.0, 1.0)) * 180.0 / M_PI;
    double yaw_deg = std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;

    std::string lost_tag = (tracking_state_ == TrackingState::LOST) ? " | [TRACKING_LOST]" : "";

    // Telemetria Completa no Log (exibe Throughput real medido em msg/s e tempo de cálculo ICP)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
        "[ICP Odometria] Throughput=%.1f msg/s | Pts=%zu | Inliers=%zu (%.0f%%) | ICP=%.1f ms | Pose=[%.2f, %.2f, %.2f] RPY=[%.1f, %.1f, %.1f] deg | Submap Voxels=%zu%s",
        real_throughput_hz_, source_cloud->size(), result.num_inliers, inlier_pct, time_ms,
        current_pose_.translation().x(), current_pose_.translation().y(), current_pose_.translation().z(),
        roll_deg, pitch_deg, yaw_deg, submap_voxelmap_->size(), lost_tag.c_str());

    if (pub_telemetry_) {
        auto t_msg = std_msgs::msg::String();
        t_msg.data = "state:" + std::string(tracking_state_ == TrackingState::LOST ? "TRACKING_LOST" : "TRACKING_OK") +
                     ";inliers:" + std::to_string(result.num_inliers) +
                     ";inlier_pct:" + std::to_string(inlier_pct) +
                     ";points:" + std::to_string(source_cloud->size()) +
                     ";submap_voxels:" + std::to_string(submap_voxelmap_->size()) +
                     ";keyframes:" + std::to_string(keyframe_count_) +
                     ";icp_ms:" + std::to_string(time_ms) +
                     ";throughput_hz:" + std::to_string(real_throughput_hz_);
        pub_telemetry_->publish(t_msg);
    }
}

void IcpOdometryComponent::publishOdometry(const rclcpp::Time& stamp,
                                          const Eigen::Isometry3d& pose,
                                          const Eigen::Matrix<double, 6, 6>& H,
                                          const Eigen::Vector3d& linear_vel,
                                          const Eigen::Vector3d& angular_vel) {
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = stamp;
    odom_msg->header.frame_id = odom_frame_id_;
    odom_msg->child_frame_id = base_frame_id_;

    odom_msg->pose.pose.position.x = pose.translation().x();
    odom_msg->pose.pose.position.y = pose.translation().y();
    odom_msg->pose.pose.position.z = pose.translation().z();

    Eigen::Quaterniond q(pose.linear());
    odom_msg->pose.pose.orientation.x = q.x();
    odom_msg->pose.pose.orientation.y = q.y();
    odom_msg->pose.pose.orientation.z = q.z();
    odom_msg->pose.pose.orientation.w = q.w();

    odom_msg->twist.twist.linear.x = linear_vel.x();
    odom_msg->twist.twist.linear.y = linear_vel.y();
    odom_msg->twist.twist.linear.z = linear_vel.z();
    odom_msg->twist.twist.angular.x = angular_vel.x();
    odom_msg->twist.twist.angular.y = angular_vel.y();
    odom_msg->twist.twist.angular.z = angular_vel.z();

    odom_msg->pose.covariance.fill(0.0);
    odom_msg->twist.covariance.fill(0.0);

    bool cov_valid = false;
    if (H.allFinite() && std::abs(H.determinant()) > 1e-6) {
        Eigen::Matrix<double, 6, 6> cov_gicp = H.inverse();
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                odom_msg->pose.covariance[r * 6 + c] = std::clamp(cov_gicp(3 + r, 3 + c), 1e-6, 1.0);
            }
        }
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                odom_msg->pose.covariance[(3 + r) * 6 + (3 + c)] = std::clamp(cov_gicp(r, c), 1e-6, 1.0);
            }
        }
        cov_valid = true;
    }

    if (!cov_valid) {
        odom_msg->pose.covariance[0] = 1e-3;   // x
        odom_msg->pose.covariance[7] = 1e-3;   // y
        odom_msg->pose.covariance[14] = 1e-3;  // z
        odom_msg->pose.covariance[21] = 1e-2;  // roll
        odom_msg->pose.covariance[28] = 1e-2;  // pitch
        odom_msg->pose.covariance[35] = 1e-2;  // yaw
    }

    for (int i = 0; i < 6; ++i) {
        odom_msg->twist.covariance[i * 6 + i] = odom_msg->pose.covariance[i * 6 + i] * 5.0;
    }

    pub_odom_->publish(std::move(odom_msg));

    if (publish_tf_ && tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = odom_frame_id_;
        tf_msg.child_frame_id = base_frame_id_;
        tf_msg.transform.translation.x = pose.translation().x();
        tf_msg.transform.translation.y = pose.translation().y();
        tf_msg.transform.translation.z = pose.translation().z();
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(tf_msg);
    }
}

void IcpOdometryComponent::publishSubmapCloud(const rclcpp::Time& stamp) {
    if (!submap_voxelmap_ || submap_voxelmap_->size() == 0 || !pub_submap_cloud_) {
        return;
    }

    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    msg->header.stamp = stamp;
    msg->header.frame_id = odom_frame_id_;
    msg->height = 1;
    msg->is_bigendian = false;
    msg->is_dense = true;

    msg->fields.resize(3);
    msg->fields[0].name = "x";
    msg->fields[0].offset = 0;
    msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg->fields[0].count = 1;

    msg->fields[1].name = "y";
    msg->fields[1].offset = 4;
    msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg->fields[1].count = 1;

    msg->fields[2].name = "z";
    msg->fields[2].offset = 8;
    msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg->fields[2].count = 1;

    msg->point_step = 12;

    size_t num_voxels = submap_voxelmap_->flat_voxels.size();
    msg->width = static_cast<uint32_t>(num_voxels);
    msg->row_step = msg->point_step * msg->width;
    msg->data.resize(msg->row_step);

    float* data_ptr = reinterpret_cast<float*>(msg->data.data());
    for (size_t i = 0; i < num_voxels; ++i) {
        const Eigen::Vector4d& mean = submap_voxelmap_->flat_voxels[i]->second.mean;
        data_ptr[i * 3 + 0] = static_cast<float>(mean.x());
        data_ptr[i * 3 + 1] = static_cast<float>(mean.y());
        data_ptr[i * 3 + 2] = static_cast<float>(mean.z());
    }

    last_submap_pub_time_ = stamp;
    pub_submap_cloud_->publish(std::move(msg));
}

void IcpOdometryComponent::handleSaveMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    std::string filename = default_map_path_;
    size_t saved_points = 0;
    if (saveMapToFile(filename, &saved_points)) {
        res->success = true;
        res->message = "Submapa de nuvem de pontos salvo com sucesso em: " + filename +
                       " (" + std::to_string(saved_points) + " pontos consolidados e filtrados via SOR)";
    } else {
        res->success = false;
        res->message = "Falha ao gravar arquivo do submapa em: " + filename;
    }
}

std::vector<Eigen::Vector3f> IcpOdometryComponent::applySORFilter(
    const std::vector<Eigen::Vector3f>& input_points,
    int k_neighbors,
    double std_ratio,
    int num_threads) {
    if (input_points.size() <= static_cast<size_t>(k_neighbors + 1)) {
        return input_points;
    }

    small_gicp::PointCloud kdtree_cloud;
    kdtree_cloud.resize(input_points.size());
    for (size_t i = 0; i < input_points.size(); ++i) {
        kdtree_cloud.points[i] = Eigen::Vector4d(input_points[i].x(), input_points[i].y(), input_points[i].z(), 1.0);
    }

    small_gicp::UnsafeKdTree<small_gicp::PointCloud> tree(kdtree_cloud);

    std::vector<double> mean_dists(input_points.size(), 0.0);

    #pragma omp parallel for num_threads(num_threads)
    for (size_t i = 0; i < input_points.size(); ++i) {
        std::vector<size_t> k_indices(k_neighbors + 1);
        std::vector<double> k_sq_dists(k_neighbors + 1);
        size_t found = tree.knn_search(kdtree_cloud.points[i], k_neighbors + 1, k_indices.data(), k_sq_dists.data());

        double dist_sum = 0.0;
        int count = 0;
        for (size_t j = 1; j < found; ++j) {
            dist_sum += std::sqrt(k_sq_dists[j]);
            count++;
        }
        mean_dists[i] = (count > 0) ? (dist_sum / count) : 1e6;
    }

    double total_mean = 0.0;
    for (double d : mean_dists) total_mean += d;
    total_mean /= mean_dists.size();

    double total_var = 0.0;
    for (double d : mean_dists) total_var += (d - total_mean) * (d - total_mean);
    double std_dev = std::sqrt(total_var / mean_dists.size());

    double max_distance_threshold = total_mean + std_ratio * std_dev;

    std::vector<Eigen::Vector3f> filtered;
    filtered.reserve(input_points.size());
    for (size_t i = 0; i < input_points.size(); ++i) {
        if (mean_dists[i] <= max_distance_threshold) {
            filtered.push_back(input_points[i]);
        }
    }
    return filtered;
}

bool IcpOdometryComponent::saveMapToFile(const std::string& filepath, size_t* out_saved_points) {
    std::lock_guard<std::mutex> lock(map_export_mutex_);
    if (accumulated_map_points_.empty() && (!submap_voxelmap_ || submap_voxelmap_->size() == 0)) {
        RCLCPP_WARN(this->get_logger(), "Nenhum ponto de submapa acumulado para salvar!");
        return false;
    }

    std::vector<Eigen::Vector3f> candidate_points;
    candidate_points.reserve(accumulated_map_points_.size());

    // 1. Filtragem por Persistência Espacial (N_hits >= 2)
    // Voxels que só foram observados 1 vez ao longo dos keyframes são descartados como transitórios
    if (submap_voxelmap_ && keyframe_count_ >= 3) {
        double inv_leaf = submap_voxelmap_->inv_leaf_size;
        for (const auto& pt : accumulated_map_points_) {
            Eigen::Vector4d p4(pt.x(), pt.y(), pt.z(), 1.0);
            Eigen::Vector3i coord = small_gicp::fast_floor(p4 * inv_leaf).head<3>();
            auto found = submap_voxelmap_->voxels.find(coord);
            if (found != submap_voxelmap_->voxels.end()) {
                size_t v_idx = found->second;
                if (submap_voxelmap_->flat_voxels[v_idx]->second.num_observations >= 2) {
                    candidate_points.push_back(pt);
                }
            }
        }
    } else {
        candidate_points = accumulated_map_points_;
    }

    if (candidate_points.empty()) {
        candidate_points = accumulated_map_points_;
    }

    RCLCPP_INFO(this->get_logger(),
        "[Salvar Mapa] Pontos acumulados brutos: %zu | Apos persistencia (N_hits >= 2): %zu",
        accumulated_map_points_.size(), candidate_points.size());

    // 2. Voxelgrid Downsampling (4 cm) para eliminar camadas duplicadas de múltiplos keyframes
    std::vector<Eigen::Vector3f> downsampled_pts;
    if (candidate_points.size() > 100) {
        small_gicp::PointCloud raw_cloud;
        raw_cloud.resize(candidate_points.size());
        for (size_t i = 0; i < candidate_points.size(); ++i) {
            raw_cloud.points[i] = Eigen::Vector4d(candidate_points[i].x(), candidate_points[i].y(), candidate_points[i].z(), 1.0);
        }
        auto ds_cloud = small_gicp::voxelgrid_sampling_omp(raw_cloud, 0.04, num_threads_);
        downsampled_pts.reserve(ds_cloud->size());
        for (size_t i = 0; i < ds_cloud->size(); ++i) {
            downsampled_pts.emplace_back(ds_cloud->points[i].x(), ds_cloud->points[i].y(), ds_cloud->points[i].z());
        }
    } else {
        downsampled_pts = candidate_points;
    }

    // 3. Aplicar Filtro Statistical Outlier Removal (SOR)
    RCLCPP_INFO(this->get_logger(), "[Salvar Mapa] Aplicando filtro Statistical Outlier Removal (SOR)...");
    std::vector<Eigen::Vector3f> final_map_points = applySORFilter(downsampled_pts, 16, 1.2, num_threads_);

    RCLCPP_INFO(this->get_logger(),
        "[Salvar Mapa] Pontos finais apos SOR: %zu (removidos %zu outliers residuais)",
        final_map_points.size(), downsampled_pts.size() - final_map_points.size());

    // 3. Gravar arquivo .ply estritamente no padrão de nuvem de pontos com orientação vertical corrigida (+Y para cima)
    float min_up_y = 1e9f, max_up_y = -1e9f;
    for (const auto& pt : final_map_points) {
        float up_y = -pt.y(); // Inverte o eixo óptico Y para que UP seja positivo
        min_up_y = std::min(min_up_y, up_y);
        max_up_y = std::max(max_up_y, up_y);
    }
    float y_span = std::max(0.01f, max_up_y - min_up_y);

    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Nao foi possivel criar o arquivo de saida: %s", filepath.c_str());
        return false;
    }

    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "comment Consolidated 3D point cloud exported by pointcloud_odometry (small_gicp)\n";
    ofs << "comment Orientation: X=Right, Y=Up (corrected for 3D viewers), Z=Forward\n";
    ofs << "element vertex " << final_map_points.size() << "\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "property uchar red\n";
    ofs << "property uchar green\n";
    ofs << "property uchar blue\n";
    ofs << "element face 0\n";
    ofs << "property list uchar int vertex_indices\n";
    ofs << "end_header\n";

    ofs << std::fixed << std::setprecision(4);
    for (const auto& pt : final_map_points) {
        float out_x = pt.x();
        float out_y = -pt.y(); // Invertido: teto no topo (+Y), chão na base (-Y)
        float out_z = pt.z();

        float t = (out_y - min_up_y) / y_span; // 0.0 (chão) até 1.0 (teto)
        int r = static_cast<int>(std::clamp(255.0f * (1.5f - 4.0f * std::abs(t - 0.75f)), 0.0f, 255.0f));
        int g = static_cast<int>(std::clamp(255.0f * (1.5f - 4.0f * std::abs(t - 0.50f)), 0.0f, 255.0f));
        int b = static_cast<int>(std::clamp(255.0f * (1.5f - 4.0f * std::abs(t - 0.25f)), 0.0f, 255.0f));

        ofs << out_x << " " << out_y << " " << out_z << " " << r << " " << g << " " << b << "\n";
    }
    ofs.close();

    // Também exportar formato universal .xyz (sem qualquer suporte a polígonos/faces)
    std::string xyz_filepath = filepath;
    size_t ext_pos = xyz_filepath.find_last_of('.');
    if (ext_pos != std::string::npos) {
        xyz_filepath = xyz_filepath.substr(0, ext_pos) + ".xyz";
    } else {
        xyz_filepath += ".xyz";
    }

    std::ofstream ofs_xyz(xyz_filepath);
    if (ofs_xyz.is_open()) {
        ofs_xyz << std::fixed << std::setprecision(4);
        for (const auto& pt : final_map_points) {
            float out_x = pt.x();
            float out_y = -pt.y();
            float out_z = pt.z();
            float t = (out_y - min_up_y) / y_span;
            int r = static_cast<int>(std::clamp(255.0f * (1.5f - 4.0f * std::abs(t - 0.75f)), 0.0f, 255.0f));
            int g = static_cast<int>(std::clamp(255.0f * (1.5f - 4.0f * std::abs(t - 0.50f)), 0.0f, 255.0f));
            int b = static_cast<int>(std::clamp(255.0f * (1.5f - 4.0f * std::abs(t - 0.25f)), 0.0f, 255.0f));
            ofs_xyz << out_x << " " << out_y << " " << out_z << " " << r << " " << g << " " << b << "\n";
        }
        ofs_xyz.close();
    }

    RCLCPP_INFO(this->get_logger(),
        "============================================================");
    RCLCPP_INFO(this->get_logger(),
        "MAPA EXPORTADO COM SUCESSO PARA O ARTIGO!");
    RCLCPP_INFO(this->get_logger(),
        "  Arquivo: %s", filepath.c_str());
    RCLCPP_INFO(this->get_logger(),
        "  Total de pontos filtrados: %zu", final_map_points.size());
    RCLCPP_INFO(this->get_logger(),
        "  Keyframes acumulados: %zu", keyframe_count_);
    RCLCPP_INFO(this->get_logger(),
        "============================================================");
    if (out_saved_points) {
        *out_saved_points = final_map_points.size();
    }
    return true;
}

} // namespace pointcloud_odometry

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_odometry::IcpOdometryComponent)

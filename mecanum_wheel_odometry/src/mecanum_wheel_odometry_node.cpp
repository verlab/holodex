#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace holodex {

class MecanumWheelOdometryNode : public rclcpp::Node {
public:
    explicit MecanumWheelOdometryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("mecanum_wheel_odometry_node", options),
          has_last_ticks_(false),
          odom_x_(0.0),
          odom_y_(0.0),
          odom_theta_(0.0),
          last_time_(0, 0, RCL_ROS_TIME)
    {
        // 1. Declaração de parâmetros
        declare_parameter<double>("wheel_radius", 0.050);
        declare_parameter<double>("track_width", 0.390);
        declare_parameter<double>("wheel_base", 0.380);
        declare_parameter<double>("ticks_per_rev", 4096.0);
        declare_parameter<bool>("publish_tf", false);
        declare_parameter<std::string>("odom_frame", "odom");
        declare_parameter<std::string>("base_frame", "base_link");
        declare_parameter<std::string>("ticks_topic", "/wheel/ticks");
        // Sinal por roda (FL, FR, RL, RR): use -1.0 se o encoder daquele lado conta ao contrário
        declare_parameter<std::vector<double>>("wheel_signs", {1.0, 1.0, 1.0, 1.0});
        declare_parameter<std::string>("odom_topic", "/odom/wheel");

        declare_parameter<std::vector<double>>("pose_covariance_diagonal", {
            0.01, 0.01, 1000000.0, 1000000.0, 1000000.0, 0.02
        });
        declare_parameter<std::vector<double>>("twist_covariance_diagonal", {
            0.01, 0.01, 1000000.0, 1000000.0, 1000000.0, 0.02
        });

        // 2. Leitura de parâmetros
        wheel_radius_ = get_parameter("wheel_radius").as_double();
        track_width_ = get_parameter("track_width").as_double();
        wheel_base_ = get_parameter("wheel_base").as_double();
        ticks_per_rev_ = get_parameter("ticks_per_rev").as_double();
        publish_tf_ = get_parameter("publish_tf").as_bool();
        odom_frame_ = get_parameter("odom_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        auto signs = get_parameter("wheel_signs").as_double_array();
        for (size_t i = 0; i < 4; ++i) {
            wheel_signs_[i] = (i < signs.size()) ? signs[i] : 1.0;
        }
        std::string ticks_topic = get_parameter("ticks_topic").as_string();
        std::string odom_topic = get_parameter("odom_topic").as_string();

        k_geom_ = (track_width_ + wheel_base_) / 2.0; // Lever arm L = lx + ly (0.385m)
        rad_per_tick_ = (2.0 * M_PI) / ticks_per_rev_;

        // Configuração de covariâncias
        setupCovariances();

        // 3. TF Broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 4. Publicador de odometria de rodas
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(10));

        // 5. Assinatura do fluxo de ticks crus a 50 Hz (Best Effort para micro-ROS)
        auto qos = rclcpp::QoS(10).best_effort().durability_volatile();
        ticks_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            ticks_topic,
            qos,
            std::bind(&MecanumWheelOdometryNode::onWheelTicks, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(),
            "MecanumWheelOdometryNode inicializado [R=%.3fm, L=%.3fm, ticks=%.0f, publish_tf=%s]",
            wheel_radius_, k_geom_, ticks_per_rev_, publish_tf_ ? "true" : "false");
    }

private:
    void setupCovariances()
    {
        auto pose_diag = get_parameter("pose_covariance_diagonal").as_double_array();
        auto twist_diag = get_parameter("twist_covariance_diagonal").as_double_array();

        pose_covariance_.fill(0.0);
        twist_covariance_.fill(0.0);

        for (size_t i = 0; i < 6 && i < pose_diag.size(); ++i) {
            pose_covariance_[i * 6 + i] = pose_diag[i];
        }
        for (size_t i = 0; i < 6 && i < twist_diag.size(); ++i) {
            twist_covariance_[i * 6 + i] = twist_diag[i];
        }
    }

    void onWheelTicks(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 4) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "/wheel/ticks com tamanho invalido (%zu < 4)", msg->data.size());
            return;
        }

        // Tempo da AMOSTRA (esp_timer do ESP32, 32 bits em us, wrap-safe). Sem isso, cai para o
        // tempo de chegada, que carrega o jitter do link serial.
        const bool has_sample_time = msg->data.size() >= 5;
        const uint32_t sample_us = has_sample_time ? static_cast<uint32_t>(msg->data[4]) : 0u;
        rclcpp::Time now = this->now();

        if (!has_last_ticks_) {
            for (int i = 0; i < 4; ++i) {
                last_ticks_[i] = msg->data[i];
            }
            last_time_ = now;
            last_sample_us_ = sample_us;
            has_last_ticks_ = true;
            return;
        }

        double dt;
        if (has_sample_time) {
            const int32_t dt_signed_us = static_cast<int32_t>(sample_us - last_sample_us_);
            if (dt_signed_us <= 0) {
                return; // amostra repetida ou pacote fora de ordem descartado com seguranca
            }
            dt = static_cast<double>(dt_signed_us) * 1e-6;
        } else {
            dt = (now - last_time_).seconds();
        }

        if (dt <= 0.0) {
            return; // amostra repetida
        }
        if (dt > 0.5) {
            // Salto temporal ou reinicialização da fonte (ex.: ESP32 reiniciou e os ticks zeraram)
            RCLCPP_WARN(get_logger(), "Delta t elevado (%.3fs). Reiniciando base de tempo.", dt);
            for (int i = 0; i < 4; ++i) {
                last_ticks_[i] = msg->data[i];
            }
            last_time_ = now;
            last_sample_us_ = sample_us;
            return;
        }
        last_time_ = now;
        last_sample_us_ = sample_us;

        // 1. Variação de ticks por roda (FL, FR, RL, RR)
        int32_t d_ticks[4];
        for (int i = 0; i < 4; ++i) {
            d_ticks[i] = msg->data[i] - last_ticks_[i];
            last_ticks_[i] = msg->data[i];
        }

        // 2. Velocidade angular de cada roda (rad/s)
        double wheel_omega[4];
        for (int i = 0; i < 4; ++i) {
            double delta_rad = wheel_signs_[i] * static_cast<double>(d_ticks[i]) * rad_per_tick_;
            wheel_omega[i] = delta_rad / dt;
        }

        // 3. Cinemática direta Mecanum 4WD
        // vx = (R / 4) * (w_fl + w_fr + w_rl + w_rr)
        // vy = (R / 4) * (-w_fl + w_fr + w_rl - w_rr)
        // wz = (R / (4 * L)) * (-w_fl + w_fr - w_rl + w_rr)
        const double r_over_4 = wheel_radius_ * 0.25;
        double vx = r_over_4 * (wheel_omega[0] + wheel_omega[1] + wheel_omega[2] + wheel_omega[3]);
        double vy = r_over_4 * (-wheel_omega[0] + wheel_omega[1] + wheel_omega[2] - wheel_omega[3]);
        double wz = (r_over_4 / k_geom_) * (-wheel_omega[0] + wheel_omega[1] - wheel_omega[2] + wheel_omega[3]);

        // 4. Integração de pose com rotação pelo ângulo médio (Mid-Point Runge-Kutta 2)
        double delta_theta = wz * dt;
        double mid_theta = odom_theta_ + (delta_theta * 0.5);

        double cos_theta = std::cos(mid_theta);
        double sin_theta = std::sin(mid_theta);

        double delta_x = (vx * cos_theta - vy * sin_theta) * dt;
        double delta_y = (vx * sin_theta + vy * cos_theta) * dt;

        odom_x_ += delta_x;
        odom_y_ += delta_y;
        odom_theta_ += delta_theta;

        // Normaliza theta para [-pi, pi]
        while (odom_theta_ > M_PI)  odom_theta_ -= 2.0 * M_PI;
        while (odom_theta_ < -M_PI) odom_theta_ += 2.0 * M_PI;

        // 5. Quatérnion de orientação (rotação pura em Z)
        double half_yaw = odom_theta_ * 0.5;
        double qz = std::sin(half_yaw);
        double qw = std::cos(half_yaw);

        // 6. Mensagem de Odometria
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;

        odom_msg.pose.pose.position.x = odom_x_;
        odom_msg.pose.pose.position.y = odom_y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = qz;
        odom_msg.pose.pose.orientation.w = qw;
        odom_msg.pose.covariance = pose_covariance_;

        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = wz;
        odom_msg.twist.covariance = twist_covariance_;

        odom_pub_->publish(odom_msg);

        // 7. TF Opcional (Desabilitado por padrão para EKF)
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = now;
            tf_msg.header.frame_id = odom_frame_;
            tf_msg.child_frame_id = base_frame_;

            tf_msg.transform.translation.x = odom_x_;
            tf_msg.transform.translation.y = odom_y_;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation.x = 0.0;
            tf_msg.transform.rotation.y = 0.0;
            tf_msg.transform.rotation.z = qz;
            tf_msg.transform.rotation.w = qw;

            tf_broadcaster_->sendTransform(tf_msg);
        }
    }

    double wheel_radius_;
    double track_width_;
    double wheel_base_;
    double k_geom_;
    double ticks_per_rev_;
    double rad_per_tick_;
    bool publish_tf_;
    std::string odom_frame_;
    std::string base_frame_;

    std::array<double, 36> pose_covariance_;
    std::array<double, 36> twist_covariance_;

    bool has_last_ticks_;
    int32_t last_ticks_[4];
    uint32_t last_sample_us_ = 0;
    double wheel_signs_[4] = {1.0, 1.0, 1.0, 1.0};
    double odom_x_;
    double odom_y_;
    double odom_theta_;
    rclcpp::Time last_time_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr ticks_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

} // namespace holodex

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<holodex::MecanumWheelOdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace pointcloud_odometry {

// Ancora de dead-reckoning: par de poses (ICP, referencia) no ultimo frame BOM.
// A referencia e uma odometria independente do ICP (rodas ou EKF), em qualquer referencial odom proprio.
struct DeadReckoningAnchor {
    bool active{false};
    Eigen::Isometry3d icp_pose{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d ref_pose{Eigen::Isometry3d::Identity()};
};

// Propaga a pose do ICP desde a ancora usando o incremento (no corpo) da referencia:
//   T_icp_agora = T_icp_ancora * (T_ref_ancora^-1 * T_ref_agora)
inline Eigen::Isometry3d propagateFromAnchor(const DeadReckoningAnchor& a, const Eigen::Isometry3d& ref_now) {
    return a.icp_pose * (a.ref_pose.inverse() * ref_now);
}

inline double rotationAngle(const Eigen::Isometry3d& from, const Eigen::Isometry3d& to) {
    return std::abs(Eigen::AngleAxisd(from.linear().transpose() * to.linear()).angle());
}

// Porta adaptativa: aceita o resultado do ICP se o desvio em relacao a predicao for compativel com a
// incerteza da predicao, que cresce com o deslocamento previsto (deslizamento das rodas Mecanum).
//   max_desvio = base + slip_ratio * deslocamento_previsto
struct GateParams {
    double base_trans{0.08};          // m
    double base_rot_rad{5.0 * M_PI / 180.0};
    double lost_trans{0.30};          // m   (base usada durante dead-reckoning / LOST)
    double lost_rot_rad{25.0 * M_PI / 180.0};
    double slip_ratio{0.5};           // fracao do deslocamento previsto tolerada como erro
};

inline bool withinGate(const Eigen::Isometry3d& prediction, const Eigen::Isometry3d& result,
                       double predicted_trans, double predicted_rot, bool dead_reckoning,
                       const GateParams& g) {
    const double dev_t = (result.translation() - prediction.translation()).norm();
    const double dev_r = rotationAngle(prediction, result);
    const double max_t = (dead_reckoning ? g.lost_trans : g.base_trans) + g.slip_ratio * predicted_trans;
    const double max_r = (dead_reckoning ? g.lost_rot_rad : g.base_rot_rad) + g.slip_ratio * predicted_rot;
    return dev_t <= max_t && dev_r <= max_r;
}

} // namespace pointcloud_odometry

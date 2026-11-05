// Teste standalone (sem gtest): g++ -std=c++17 -I include -I /usr/include/eigen3 test/test_recovery_utils.cpp
#include <cstdio>
#include "pointcloud_odometry/recovery_utils.hpp"

using namespace pointcloud_odometry;
static int fails = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FALHOU: %s (linha %d)\n", #c, __LINE__); ++fails; } } while (0)

static Eigen::Isometry3d pose(double x, double y, double yaw) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translate(Eigen::Vector3d(x, y, 0));
    T.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    return T;
}

int main() {
    // Referencia (rodas/EKF) esta num referencial diferente do ICP (origem e yaw distintos).
    DeadReckoningAnchor a;
    a.active = true;
    a.icp_pose = pose(2.0, 1.0, 0.5);
    a.ref_pose = pose(10.0, -3.0, -1.0);

    // Robo anda 1 m para frente (corpo) e gira 0.3 rad na referencia.
    Eigen::Isometry3d ref_now = a.ref_pose * pose(1.0, 0.0, 0.0) * pose(0.0, 0.0, 0.3);
    Eigen::Isometry3d g = propagateFromAnchor(a, ref_now);
    Eigen::Isometry3d expected = a.icp_pose * pose(1.0, 0.0, 0.0) * pose(0.0, 0.0, 0.3);
    CHECK((g.matrix() - expected.matrix()).norm() < 1e-9);

    // Sem movimento: a propagacao devolve a pose ancorada.
    CHECK((propagateFromAnchor(a, a.ref_pose).matrix() - a.icp_pose.matrix()).norm() < 1e-9);

    // Strafe lateral puro (holonomico) e preservado no corpo.
    Eigen::Isometry3d g2 = propagateFromAnchor(a, a.ref_pose * pose(0.0, 0.5, 0.0));
    CHECK(((a.icp_pose.inverse() * g2).translation() - Eigen::Vector3d(0, 0.5, 0)).norm() < 1e-9);

    GateParams gp;
    // Normal, quase parado: desvio de 3 cm aceito, 20 cm rejeitado.
    CHECK(withinGate(pose(0, 0, 0), pose(0.03, 0, 0), 0.0, 0.0, false, gp));
    CHECK(!withinGate(pose(0, 0, 0), pose(0.20, 0, 0), 0.0, 0.0, false, gp));
    // Mesmo desvio de 20 cm e aceito se o robo previu andar 0.3 m (tolerancia cresce com o deslocamento).
    CHECK(withinGate(pose(0.3, 0, 0), pose(0.5, 0, 0), 0.3, 0.0, false, gp));
    // Rotacao: 10 graus rejeitado no normal, aceito em dead-reckoning.
    CHECK(!withinGate(pose(0, 0, 0), pose(0, 0, 10 * M_PI / 180), 0.0, 0.0, false, gp));
    CHECK(withinGate(pose(0, 0, 0), pose(0, 0, 10 * M_PI / 180), 0.0, 0.0, true, gp));
    // Dead-reckoning nao aceita saltos absurdos (1 m).
    CHECK(!withinGate(pose(0, 0, 0), pose(1.0, 0, 0), 0.0, 0.0, true, gp));

    std::printf(fails ? "%d falha(s)\n" : "OK: todos os testes passaram\n", fails);
    return fails ? 1 : 0;
}

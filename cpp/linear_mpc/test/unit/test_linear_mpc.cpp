
#include "linear_mpc/linear_mpc.h"

#include <Eigen/Dense>
#include <iostream>
#include <math.h>

#include <gtest/gtest.h>

const double INF = 1000000;

TEST(TestLinearMPC, constructor) {
  // Linear Drone example+
  // Configurable parameters
  const int Nu = 5;   // Number of control inputs (appended gravity term)
  const int Nx = 6;   // Number of states
  const int N = 20;   // Time horizons to consider
  const int dt = 0.1; // Time horizon
  const int m = 1;    // Mass of drone

  // Weights on state deviation and control input
  Eigen::MatrixXd Qx(Nx, Nx);
  Qx.diagonal() << 100, 100, 1000, 1, 1, 1;
  Eigen::MatrixXd Qn = 5 * Qx;
  Eigen::MatrixXd Ru(Nx, Nu);
  Ru.diagonal() << 1, 1, 1, 1, 1;

  // Bounds on states and controls
  Eigen::MatrixXd xbounds(Nx, 2);
  xbounds.col(0) << -INF, -INF, -INF, -INF, -INF, -INF;
  xbounds.col(1) << INF, INF, INF, INF, INF, INF;

  Eigen::MatrixXd ubounds(Nu, 2);
  ubounds.col(0) << -M_PI / 2, -M_PI / 2, -M_PI / 2, -20, 1;
  ubounds.col(1) << M_PI / 2, M_PI / 2, M_PI / 2, 20, 1;

  // Linearized dynamics
  Eigen::MatrixXd Ad(Nx, Nx);
  Ad.row(0) << 1, 0, 0, dt, 0, 0;
  Ad.row(1) << 0, 1, 0, 0, dt, 0;
  Ad.row(2) << 0, 0, 1, 0, 0, dt;
  Ad.row(3) << 0, 0, 0, 1, 0, 0;
  Ad.row(4) << 0, 0, 0, 0, 1, 0;
  Ad.row(5) << 0, 0, 0, 0, 0, 1;

  Eigen::MatrixXd Bd(Nx, Nu);
  Bd.row(0) << 0, 0, 0, 0, 0;
  Bd.row(1) << 0, 0, 0, 0, 0;
  Bd.row(2) << 0, 0, 0, 0, 0;
  Bd.row(3) << -9.8 * dt, 0, 0, 0, 0;
  Bd.row(4) << 0, 9.8 * dt, 0, 0, 0;
  Bd.row(5) << 0, 0, 0, dt / m, -9.8 * dt;

  // Reference trajectory
  Eigen::MatrixXd ref_traj(Nx, N + 1);
  ref_traj.setZero();
  /*ref_traj.row(0) << Eigen::VectorXd::Zero(N + 1); // xref
  ref_traj.row(1) << Eigen::VectorXd::Zero(N + 1); // yref
  ref_traj.row(2) << Eigen::VectorXd::Zero(N + 1); // zref

  ref_traj.row(3) << Eigen::VectorXd::Zero(N + 1); // dxref
  ref_traj.row(4) << Eigen::VectorXd::Zero(N + 1); // dyref
  ref_traj.row(5) << Eigen::VectorXd::Zero(N + 1); // dzref*/
  control::mpc::LinearMPC mpc(Ad, Bd, Qx, Qn, Ru, xbounds, ubounds, N);

  // Test Cost Function
  Eigen::MatrixXd H;
  Eigen::MatrixXd f;
  mpc.get_cost_function(ref_traj, H, f);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
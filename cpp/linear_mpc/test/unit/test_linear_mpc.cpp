
#include "linear_mpc/linear_mpc.h"

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <math.h>

#include <gtest/gtest.h>

const double INF = 1000000;

//========================================================================================
void serialize_signals(std::ofstream &ofs, Eigen::VectorXd var) {

  for (int i = 0; i < var.size(); i++) {
    ofs << var[i] << ",";
  }
  ofs << "\n";
}

TEST(TestLinearMPC, constructor) {
  // Linear Drone example+
  // Configurable parameters
  const int Nu = 5; // Appended gravity term
  const int Nx = 6; // Number of states
  const int N = 10;   // Time horizons to consider
  const double dt = 0.1;             // Time horizon
  const int m = 1;                   // Mass of drone

  // Weights on state deviation and control input
  Eigen::MatrixXd Qx(Nx, Nx);
  Qx.setZero();
  Qx.diagonal() << 100, 100, 1000, 1, 1, 1;
  Eigen::MatrixXd Qn = 5 * Qx;
  Eigen::MatrixXd Ru(Nu, Nu);
  Ru.setZero();
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
  for (int i = 0; i < N + 1; i++) {
    ref_traj(0, i) = cos(0.5 * i); // xref
    ref_traj(1, i) = sin(0.5 * i); // yref
    ref_traj(2, i) = 0.1 * i;      // zref
  }

  // Initial state
  Eigen::VectorXd x0 = ref_traj.col(0);

  control::mpc::LinearMPC mpc(Ad, Bd, Qx, Qn, Ru, xbounds, ubounds,N);

  // Test Cost Function
  Eigen::MatrixXd H;
  Eigen::VectorXd f;
  mpc.get_cost_function(ref_traj, H, f);
  // TODO eval
  // std::cout << f << std::endl;

  // Test Dynamic Constraint function
  Eigen::MatrixXd Aeq;
  Eigen::MatrixXd beq;
  mpc.get_dynamics_constraint(Aeq, beq);

  // Test state and control bounds
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  mpc.get_state_control_bounds(x0, lb, ub);

  // Test solving
  Eigen::MatrixXd x_out;
  double f_val;
  mpc.solve(x0, ref_traj, x_out, f_val);

  // Test resolving
  mpc.solve(x0, ref_traj, x_out, f_val);

  // Test extracting solution
  Eigen::MatrixXd first_control;
  Eigen::MatrixXd opt_traj;
<<<<<<< HEAD
  mpc.get_output(x_out, first_control, opt_traj);

  // Expected output
  Eigen::Matrix<double, Nx, N + 1> opt_traj_expect;
  opt_traj_expect.row(0) << 1, 1, 0.846614, 0.539702, 0.0791166, -0.272055,
      -0.469205, -0.512353, -0.401526, -0.136747, 0.120345;
  opt_traj_expect.row(1) << -1.04253e-07, -2.30047e-07, 0.153928, 0.373809,
      0.439772, 0.351829, 0.109976, -0.238516, -0.568924, -0.809883, -0.941784;
  opt_traj_expect.row(2) << -3.9392e-07, -1.09946e-06, 0.0758745, 0.194776,
      0.333952, 0.48039, 0.628918, 0.765976, 0.876618, 0.942629, 0.940891;
  opt_traj_expect.row(3) << 6.78385e-06, -1.5339, -3.06913, -4.60586, -3.51173,
      -1.9715, -0.431493, 1.10827, 2.64779, 2.57092, 1.03125;
  opt_traj_expect.row(4) << -2.15308e-07, 1.53928, 2.19881, 0.659634, -0.879432,
      -2.41853, -3.48492, -3.30409, -2.40959, -1.31901, -0.234775;
  opt_traj_expect.row(5) << -3.09856e-06, 0.75876, 1.18902, 1.39176, 1.46439,
      1.48527, 1.37058, 1.10641, 0.660112, -0.0173789, -0.957193;

  EXPECT_TRUE(opt_traj_expect.isApprox(opt_traj, 1e-5));
=======
  mpc.get_output(x_out,first_control,opt_traj);
>>>>>>> upstream/devel/cpp
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
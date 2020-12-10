#include "linear_mpc/linear_mpc.h"
#include "linear_mpc/matrix_algebra.h"

#include <iostream>
namespace control {
namespace mpc {
using namespace Eigen;
using namespace control::math;
//========================================================================================
LinearMPC::LinearMPC(const Eigen::MatrixXd &Ad, const Eigen::MatrixXd &Bd,
                     const Eigen::MatrixXd &Q, const Eigen::MatrixXd &Qn,
                     const Eigen::MatrixXd &R,
                     const Eigen::MatrixXd &state_bounds,
                     const Eigen::MatrixXd &control_bounds)
    : m_Ad(Ad), m_Bd(Bd), m_Q(Q), m_Qn(Qn), m_R(R),
      m_state_bounds(state_bounds), m_control_bounds(control_bounds) {

  solver_.initSolver();
  // solver_.settings()->setVerbosity(false);
  solver_.settings()->setWarmStart(true);
}

//========================================================================================
void LinearMPC::get_cost_function(const MatrixXd &ref_traj, MatrixXd &H,
                                  MatrixXd &f) {

  m_Hq = control::math::kron(MatrixXd::Identity(m_N, m_N), m_Q);
  m_Hqn = m_Qn;
  m_Hu = control::math::kron(MatrixXd::Identity(m_N, m_N), m_R);

  H = control::math::block_diag(m_Hq, m_Hqn, m_Hu);

  MatrixXd y = reshape(ref_traj, ref_traj.cols() * ref_traj.rows(), 1);
  Ref<Matrix<double, 1, m_num_state_vars>> y_fixed(y);

  MatrixXd H_tmp = block_diag(kron(MatrixXd::Identity(m_N, m_N), m_Q), m_Qn);

  MatrixXd fx = y_fixed * H_tmp.block<m_num_state_vars, m_num_state_vars>(0, 0);
  MatrixXd fu = MatrixXd::Zero(m_N * m_Nu, 1);

  MatrixXd f_tmp(m_Nx * (m_N + 1) + m_Nu * m_N, 1);
  f_tmp << -fx.transpose(), -fu;

  f = f_tmp;
}

//========================================================================================
void LinearMPC::get_dynamics_constraint(Eigen::MatrixXd &A_eq,
                                        Eigen::MatrixXd &b_eq) {

  Matrix<double, m_N * m_Nx, m_num_state_vars> A_padded_eye;
  A_padded_eye.setZero();

  A_padded_eye.block<m_N * m_Nx, m_N * m_Nx>(0, m_Nx) =
      -MatrixXd::Identity(m_N * m_Nx, m_N * m_Nx);

  Matrix<double, m_N * m_Nx, m_num_state_vars> A_padded_ad;
  A_padded_ad.setZero();
  for (int i = 0; i < m_N; i++) {
    A_padded_ad.block<m_Nx, m_Nx>(i * m_Nx, i * m_Nx) = m_Ad;
  }

  Matrix<double, m_N * m_Nx, m_num_decision_vars> A_eq_;
  A_eq_.setZero();
  A_eq_.block<m_N * m_Nx, m_num_state_vars>(0, 0) = A_padded_eye + A_padded_ad;
  for (int i = 0; i < m_Nx; i++) {
    A_eq_.block<m_Nx, m_Nu>(i * m_Nx, m_num_state_vars + i * m_Nu) = m_Bd;
  }

  Matrix<double, m_num_state_vars, 1> b_eq_;
  b_eq_.setZero();

  // return matrices
  A_eq = A_eq_;
  b_eq = b_eq_;
}

//========================================================================================
void LinearMPC::get_state_control_bounds(Eigen::VectorXd initial_state,
                                         Eigen::MatrixXd &lb,
                                         Eigen::MatrixXd &ub) {

  Eigen::VectorXd x_min = m_state_bounds.col(0);
  Eigen::VectorXd x_max = m_state_bounds.col(1);

  Eigen::VectorXd u_min = m_control_bounds.col(0);
  Eigen::VectorXd u_max = m_control_bounds.col(1);

  Matrix<double, m_num_decision_vars, 1> lb_, ub_;
  lb_ << initial_state, x_min.replicate(m_N, 1), u_min.replicate(m_N, 1);
  ub_ << initial_state, x_max.replicate(m_N, 1), u_max.replicate(m_N, 1);

  // Return bound vectors
  lb = lb_;
  ub = ub_;
}

//========================================================================================
void LinearMPC::solve(const Eigen::VectorXd &initial_state,
                      const Eigen::MatrixXd &ref_traj, Eigen::MatrixXd &x_out,
                      double &f_val) {
  // Collect MPC Matrices
  Eigen::MatrixXd H_dense, f_dynamic;
  this->get_cost_function(ref_traj, H_dense, f_dynamic);

  Eigen::MatrixXd A_dyn_dense, b_dyn;
  this->get_dynamics_constraint(A_dyn_dense, b_dyn);

  Eigen::MatrixXd lb_dynamic, ub_dynamic; // Should be row vectors
  this->get_state_control_bounds(initial_state, lb_dynamic, ub_dynamic);

  // Cast to OSQP style QP
  Eigen::MatrixXd A_dense(A_dyn_dense.rows(),
                          A_dyn_dense.cols() + m_num_decision_vars);
  A_dense << A_dyn_dense,
      Eigen::MatrixXd::Identity(m_num_decision_vars, m_num_decision_vars);

  Eigen::SparseMatrix<double> H = H_dense.sparseView();
  Eigen::SparseMatrix<double> A = A_dense.sparseView();
  Eigen::Matrix<double, m_num_decision_vars, 1> f = f_dynamic;

  Eigen::MatrixXd lb(m_num_constraints, 1);
  lb << b_dyn, lb_dynamic;
  Eigen::Matrix<double, m_num_constraints, 1> l = lb;

  Eigen::MatrixXd ub(m_num_constraints, 1);
  ub << b_dyn, ub_dynamic;
  Eigen::Matrix<double, m_num_constraints, 1> u = ub;

  solver_.data()->setNumberOfVariables(2);
  solver_.data()->setNumberOfConstraints(2);
  solver_.data()->setHessianMatrix(H);
  solver_.data()->setGradient(f);
  solver_.data()->setLinearConstraintsMatrix(A);
  solver_.data()->setLowerBound(l);
  solver_.data()->setUpperBound(u);

  // Call solver
  solver_.solve();
  Eigen::MatrixXd qp_solution = solver_.getSolution();
}

} // namespace mpc
} // namespace control

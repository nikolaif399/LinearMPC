#include "linear_mpc/linear_mpc.h"
#include "linear_mpc/matrix_algebra.h"

#include <iostream>
namespace control {
namespace mpc {
using namespace Eigen;
using namespace control::math;
//========================================================================================
LinearMPC::LinearMPC(const Eigen::Matrix<double, m_Nx, m_Nx> &Ad,
                     const Eigen::MatrixXd &Bd, const Eigen::MatrixXd &Q,
                     const Eigen::MatrixXd &Qn, const Eigen::MatrixXd &R,
                     const Eigen::MatrixXd &state_bounds,
                     const Eigen::MatrixXd &control_bounds)
    : m_Ad(Ad), m_Bd(Bd), m_Q(Q), m_Qn(Qn), m_R(R),
      m_state_bounds(state_bounds), m_control_bounds(control_bounds) {}

//========================================================================================
void LinearMPC::get_cost_function(const MatrixXd &ref_traj, MatrixXd &H,
                                  MatrixXd &f) {

  m_Hq = control::math::kron(MatrixXd::Identity(m_N, m_N), m_Q);
  m_Hqn = m_Qn;
  m_Hu = control::math::kron(MatrixXd::Identity(m_N, m_N), m_R);

  H = control::math::block_diag(m_Hq, m_Hqn, m_Hu);
  Matrix<double, 1, m_num_state_vars> y;
  y.block(0, 0, 1, m_num_state_vars) =
      reshape(ref_traj, 1, ref_traj.cols() * ref_traj.rows());
  MatrixXd H_tmp = block_diag(kron(MatrixXd::Identity(m_N, m_N), m_Q), m_Qn);

  MatrixXd fx = y * H_tmp.block(0, 0, m_num_state_vars, m_num_state_vars);
  MatrixXd fu = MatrixXd::Zero(m_N * m_Nu, 1);

  MatrixXd f_(1, m_Nq);
  f_ << -fx, -fu.transpose();

  f = f_;

  std::cout << "[get_cost_function]" << std::endl;
  std::cout << " Hq: " << m_Hq.rows() << "," << m_Hq.cols() << std::endl;
  std::cout << " Hqn: " << m_Hqn.rows() << "," << m_Hqn.cols() << std::endl;
  std::cout << " Hu: " << m_Hu.rows() << "," << m_Hu.cols() << std::endl;
  std::cout << " H: " << H.rows() << "," << H.cols() << std::endl;
  std::cout << " y: " << y.rows() << "," << y.cols() << std::endl;
  std::cout << " fx: " << fx.rows() << "," << fx.cols() << std::endl;
  std::cout << " fu: " << fu.rows() << "," << fu.cols() << std::endl;
  std::cout << " f: " << f.rows() << "," << f.cols() << std::endl;
  std::cout << std::endl;
}

//========================================================================================
void LinearMPC::get_dynamics_constraint(Eigen::MatrixXd &A_eq,
                                        Eigen::MatrixXd &b_eq) {

  Matrix<double, m_Nx_vars, m_Nx_decision> A_padded_eye;
  A_padded_eye.setZero();

  A_padded_eye.block<m_Nx_vars, m_Nx_vars>(0, m_Nx) =
      -MatrixXd::Identity(m_Nx_vars, m_Nx_vars);

  Matrix<double, m_Nx_vars, m_Nx_decision> A_padded_ad;
  A_padded_ad.setZero();
  for (int i = 0; i < m_N; i++) {
    A_padded_ad.block<m_Nx, m_Nx>(i * m_Nx, i * m_Nx) = m_Ad;
  }
  // std::cout << "A_padded_ad: \n" << A_padded_ad << std::endl;

  Matrix<double, m_Nx_vars, m_Nq> A_eq_;
  A_eq_.setZero();
  A_eq_.block<m_Nx_vars, m_Nx_decision>(0, 0) = A_padded_eye + A_padded_ad;
  for (int i = 0; i < m_N; i++) {
    A_eq_.block<m_Nx, m_Nu>(i * m_Nx, m_Nx_decision + i * m_Nu) = m_Bd;
  }
  // std::cout << "A_eq: \n" << A_eq_ << std::endl;

  Matrix<double, m_Nx_vars, 1> b_eq_;
  b_eq_.setZero();

  // return matrices
  A_eq = A_eq_;
  b_eq = b_eq_;

  std::cout << "[get_dynamics_constraint]" << std::endl;
  std::cout << " A_padded_eye: " << A_padded_eye.rows() << ","
            << A_padded_eye.cols() << std::endl;
  std::cout << " A_padded_ad: " << A_padded_ad.rows() << ","
            << A_padded_ad.cols() << std::endl;
  std::cout << " Aeq: " << A_eq.rows() << "," << A_eq.cols() << std::endl;
  std::cout << " beq: " << b_eq.rows() << "," << b_eq.cols() << std::endl;
  std::cout << std::endl;
}

//========================================================================================
void LinearMPC::get_state_control_bounds(const Eigen::VectorXd &initial_state,
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
  // std::cout << "lb: \n" << lb << std::endl;
  // std::cout << "ub: \n" << ub << std::endl;

  std::cout << "[get_state_control_bounds]" << std::endl;
  std::cout << "lb: " << lb.rows() << "," << lb.cols() << std::endl;
  std::cout << "ub: " << ub.rows() << "," << ub.cols() << std::endl;
  std::cout << std::endl;
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
  Eigen::MatrixXd A_dense(A_dyn_dense.rows() + m_num_decision_vars,
                          A_dyn_dense.cols());
  A_dense.block(0, 0, A_dyn_dense.rows(), A_dyn_dense.cols()) = A_dyn_dense;
  A_dense.block(A_dyn_dense.rows(), 0, m_num_decision_vars,
                m_num_decision_vars) =
      Eigen::MatrixXd::Identity(m_num_decision_vars, m_num_decision_vars);

  /*A_dense << A_dyn_dense,
      Eigen::MatrixXd::Identity(m_num_decision_vars, m_num_decision_vars);*/

  Eigen::SparseMatrix<double> H = H_dense.sparseView();
  Eigen::SparseMatrix<double> A = A_dense.sparseView();
  Eigen::Matrix<double, m_num_decision_vars, 1> f = f_dynamic;

  Eigen::Matrix<double, 1, m_num_constraints> l;
  std::cout << "b_dyn: " << b_dyn.cols() << "," << b_dyn.rows() << std::endl;
  std::cout << "lb_dynamic: " << lb_dynamic.cols() << "," << lb_dynamic.rows()
            << std::endl;
  std::cout << "l: " << l.cols() << "," << l.rows() << std::endl;
  l << b_dyn, lb_dynamic;

  /**Eigen::Matrix<double, m_num_constraints, 1> u; // = ub;
  u << b_dyn, ub_dynamic;

  /*std::cout << "N: " << m_N << std::endl;
  std::cout << "Nx: " << m_Nx << std::endl;
  std::cout << "Nu: " << m_Nu << std::endl;
  std::cout << "Num constraints: " << m_num_constraints << std::endl;
  std::cout << "Num decision vars: " << m_num_decision_vars << std::endl;

  std::cout << "A_dense: " << A_dense.rows() << ", " << A_dense.cols() <<
  std::endl;
  std::cout << "H_dense: " << H_dense.rows() << ", " << H_dense.cols() <<
  std::endl;
  //std::cout << "A: " << A.rows() << ", " << A.cols() << std::endl;
  //std::cout << "H: " << H.rows() << ", " << H.cols() << std::endl;
  * /
  solver_.data()->setNumberOfVariables(m_num_decision_vars);
  solver_.data()->setNumberOfConstraints(m_num_constraints);
  solver_.data()->setHessianMatrix(H);
  solver_.data()->setGradient(f);
  solver_.data()->setLinearConstraintsMatrix(A);
  solver_.data()->setLowerBound(l);
  solver_.data()->setUpperBound(u);

  // Init solver if not already initialized
  if (!solver_.isInitialized()) {
    std::cout << solver_.initSolver() << std::endl;
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
  }
  // Call solver
  solver_.solve();
  Eigen::MatrixXd qp_solution = solver_.getSolution();*/
}

} // namespace mpc
} // namespace control

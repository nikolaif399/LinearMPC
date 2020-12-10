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

  // Construct H matrix
  m_Hq = control::math::kron(MatrixXd::Identity(m_N, m_N), m_Q);
  m_Hqn = m_Qn;
  m_Hu = control::math::kron(MatrixXd::Identity(m_N, m_N), m_R);
  H = control::math::block_diag(m_Hq, m_Hqn, m_Hu);

  // Construct fx vector
  Matrix<double, 1, m_Nx_decision> y;
  y.block(0, 0, 1, m_Nx_decision) =
      reshape(ref_traj, 1, ref_traj.cols() * ref_traj.rows());
  MatrixXd H_tmp = block_diag(kron(MatrixXd::Identity(m_N, m_N), m_Q), m_Qn);
  MatrixXd fx = y * H_tmp.block(0, 0, m_num_state_vars, m_num_state_vars);

  // Construct fu vector
  MatrixXd fu = MatrixXd::Zero(m_N * m_Nu, 1);

  // Construct f vector
  Matrix<double, m_Nq, 1> f_;
  f_ << -fx.transpose(), -fu;
  f = f_;

  /*std::cout << "H_tmp: \n" << H_tmp << std::endl;
  std::cout << "y: \n" << y << std::endl;
  std::cout << "fx: \n" << fx << std::endl;
  std::cout << "f:\n" << f << std::endl;*/

  /*std::cout << "[get_cost_function]" << std::endl;
  std::cout << " Hq: " << m_Hq.rows() << "," << m_Hq.cols() << std::endl;
  std::cout << " Hqn: " << m_Hqn.rows() << "," << m_Hqn.cols() << std::endl;
  std::cout << " Hu: " << m_Hu.rows() << "," << m_Hu.cols() << std::endl;
  std::cout << " H: " << H.rows() << "," << H.cols() << std::endl;
  std::cout << " y: " << y.rows() << "," << y.cols() << std::endl;
  std::cout << " fx: " << fx.rows() << "," << fx.cols() << std::endl;
  std::cout << " fu: " << fu.rows() << "," << fu.cols() << std::endl;
  std::cout << " f: " << f.rows() << "," << f.cols() << std::endl;
  std::cout << std::endl;*/
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

  /*std::cout << "[get_dynamics_constraint]" << std::endl;
  std::cout << " A_padded_eye: " << A_padded_eye.rows() << ","
            << A_padded_eye.cols() << std::endl;
  std::cout << " A_padded_ad: " << A_padded_ad.rows() << ","
            << A_padded_ad.cols() << std::endl;
  std::cout << " Aeq: " << A_eq.rows() << "," << A_eq.cols() << std::endl;
  std::cout << " beq: " << b_eq.rows() << "," << b_eq.cols() << std::endl;
  std::cout << std::endl;*/
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
  /*std::cout << "[get_state_control_bounds]" << std::endl;
  std::cout << "lb: " << lb.rows() << "," << lb.cols() << std::endl;
  std::cout << "ub: " << ub.rows() << "," << ub.cols() << std::endl;
  std::cout << std::endl;*/
}

//========================================================================================
void LinearMPC::get_output(const Eigen::MatrixXd &x_out,
                      Eigen::MatrixXd &first_control,
                      Eigen::MatrixXd &opt_traj) {

  // Resize and wipe output containers
  first_control.resize(m_Nu,1);
  first_control.setZero();
  opt_traj.resize(m_Nx,m_N+1);
  opt_traj.setZero();

  // Collect optimized control
  first_control = x_out.block<m_Nu,1>((m_N+1)*m_Nx,0);

  // std::cout << x_out.rows() << ", " << x_out.cols() << std::endl; // 28 x 1
  //std::cout << x_out << std::endl;

  // Collect optimized trajectory
  for (size_t i = 0; i < m_N + 1; ++i)
  {
    for (size_t j = 0; j < m_Nx; ++j)
    {
      opt_traj(j,i) = x_out(i*m_Nx+j,0);
    }
  }
  //std::cout << std::endl << opt_traj << std::endl;
  //std::cout << std::endl << first_control << std::endl;
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
  Eigen::MatrixXd A_dense(A_dyn_dense.rows() + m_Nq, A_dyn_dense.cols());
  A_dense.block(0, 0, A_dyn_dense.rows(), A_dyn_dense.cols()) = A_dyn_dense;
  A_dense.block(A_dyn_dense.rows(), 0, m_Nq, m_Nq) =
      Eigen::MatrixXd::Identity(m_Nq, m_Nq);

  Eigen::SparseMatrix<double> H = H_dense.sparseView();
  Eigen::SparseMatrix<double> A = A_dense.sparseView();
  Eigen::Matrix<double, m_Nq, 1> f = f_dynamic;

  Matrix<double, m_Nx_vars + m_Nq, 1> l;
  l.block(0, 0, b_dyn.rows(), 1) = b_dyn;
  l.block(b_dyn.rows(), 0, lb_dynamic.rows(), 1) = lb_dynamic;

  Matrix<double, m_Nx_vars + m_Nq, 1> u;
  u.block(0, 0, b_dyn.rows(), 1) = b_dyn;
  u.block(b_dyn.rows(), 0, ub_dynamic.rows(), 1) = ub_dynamic;

  // Set OSPQ options
  solver_.data()->setNumberOfVariables(m_Nq);
  solver_.data()->setNumberOfConstraints(m_Nconst);
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
  x_out = solver_.getSolution();
}

} // namespace mpc
} // namespace control

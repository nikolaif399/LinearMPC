#include "linear_mpc/linear_mpc.h"
#include "linear_mpc/matrix_algebra.h"

#include <iostream>
namespace control {
namespace mpc {
using namespace Eigen;

//========================================================================================
LinearMPC::LinearMPC(const Eigen::MatrixXd &Ad, const Eigen::MatrixXd &Bd,
                     const Eigen::MatrixXd &Q, const Eigen::MatrixXd &Qn,
                     const Eigen::MatrixXd &R,
                     const Eigen::MatrixXd &state_bounds,
                     const Eigen::MatrixXd &control_bounds)
    : m_Ad(Ad), m_Bd(Bd), m_Q(Q), m_Qn(Qn), m_R(R),
      m_state_bounds(state_bounds), m_control_bounds(control_bounds) {

  solver_.initSolver();
  //solver_.settings()->setVerbosity(false);
  solver_.settings()->setWarmStart(true);
}

//========================================================================================
void LinearMPC::get_cost_function(const MatrixXd &ref_traj, MatrixXd &H,
                                  MatrixXd &f) {

  m_Hq = control::math::kron(MatrixXd::Identity(m_N, m_N), m_Q);
  m_Hqn = m_Qn;
  m_Hu = control::math::kron(MatrixXd::Identity(m_N, m_N), m_R);

  H = control::math::block_diag(m_Hq, m_Hqn, m_Hu);

  MatrixXd y =
      control::math::reshape(ref_traj, ref_traj.cols() * ref_traj.rows(), 1);

  std::cout << "H: " << H.cols() << "," << H.rows() << std::endl;
  std::cout << "y: " << y.cols() << "," << y.rows() << std::endl;

  // MatrixXd fx =  * H;
  MatrixXd fu = MatrixXd::Zero(m_N * m_N, 1);
  MatrixXd f_tmp(m_Nx * (m_N + 1) + m_Nu * m_N, 1);
  // f << -fx.transpose(), -fu;
}

void LinearMPC::get_dynamics_constraint(Eigen::MatrixXd &Aeq,
                                        Eigen::MatrixXd &beq) {

}

void LinearMPC::get_state_control_bounds(const Eigen::VectorXd &initial_state,
                                Eigen::MatrixXd &lb, Eigen::MatrixXd &ub) {

}

void LinearMPC::solve(const Eigen::VectorXd &initial_state,
                      const Eigen::MatrixXd &ref_traj,
                      Eigen::MatrixXd &x_out, double &f_val) {
  // Collect MPC Matrices
  Eigen::MatrixXd H_dense,f_dynamic;
  this->get_cost_function(ref_traj,H_dense,f_dynamic);

  Eigen::MatrixXd A_dyn_dense, b_dyn;
  this->get_dynamics_constraint(A_dyn_dense,b_dyn);

  Eigen::MatrixXd lb_dynamic, ub_dynamic; // Should be row vectors
  this->get_state_control_bounds(initial_state,lb_dynamic,ub_dynamic);

  const int num_decision_vars = lb_dynamic.cols();

  // Cast to OSQP style QP
  Eigen::MatrixXd A_dense(A_dyn_dense.rows(),A_dyn_dense.cols() + num_decision_vars);
  A_dense << A_dyn_dense, Eigen::MatrixXd::Identity(num_decision_vars,
                                                    num_decision_vars);

  Eigen::SparseMatrix<double> H = H_dense.sparseView();
  Eigen::SparseMatrix<double> A = A_dense.sparseView();

  // Getting the size on these right is going to be tough
  Eigen::Matrix<double,12,1> f = f_dynamic;
  Eigen::Matrix<double,12,1> lb = lb_dynamic;
  Eigen::Matrix<double,12,1> ub = ub_dynamic;

  solver_.data()->setNumberOfVariables(2);
  solver_.data()->setNumberOfConstraints(2);
  solver_.data()->setHessianMatrix(H);
  solver_.data()->setGradient(f);
  solver_.data()->setLinearConstraintsMatrix(A);
  solver_.data()->setLowerBound(lb);
  solver_.data()->setUpperBound(ub);

  // Call solver
  solver_.solve();
  Eigen::MatrixXd qp_solution = solver_.getSolution();
}


} // namespace mpc
} // namespace control

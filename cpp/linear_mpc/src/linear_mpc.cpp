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
                     const Eigen::MatrixXd &control_bounds, const int &N)
    : m_Ad(Ad), m_Bd(Bd), m_Q(Q), m_Qn(Qn), m_R(R),
      m_state_bounds(state_bounds), m_control_bounds(control_bounds), m_N(N),
      m_Nx(m_Bd.rows()), m_Nu(m_Bd.cols()) {

  update_horizon_length(m_N);
}

//========================================================================================
void LinearMPC::update_horizon_length(const int &N) {
  m_Nq = (N + 1) * m_Nx + m_N * m_Nu;
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

  auto tmp = kron(MatrixXd::Identity(m_N, m_N), m_Qn);
  // auto Hd = control::math::block_diag()
  // MatrixXd fx =  * H;
  MatrixXd fu = MatrixXd::Zero(m_N * m_N, 1);
  MatrixXd f_tmp(m_Nx * (m_N + 1) + m_Nu * m_N, 1);
  // f << -fx.transpose(), -fu;
}
} // namespace mpc
} // namespace control

#pragma once

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Core> // Ref
#include <Eigen/Dense>
#include <Eigen/Dense>

namespace control {
namespace mpc {

/*
constexpr int m_N = 10;
constexpr int m_Nx = 6;
constexpr int m_Nu = 5;
constexpr int m_Nq = (m_N + 1) * m_Nx + m_N * m_Nu;
constexpr int m_Nx_vars = m_N * m_Nx;
constexpr int m_Nx_decision = (m_N + 1) * m_Nx;
constexpr int m_Nconst = m_Nq + m_N * m_Nx;
//  const int totNx = N_x*N_t, totNu = N_u*N_t, fullNx = N_x*(N_t+1);

constexpr int m_num_control_vars = m_N * m_Nu;
constexpr int m_num_state_vars = (m_N + 1) * m_Nx;
constexpr int m_num_decision_vars = (m_N + 1) * m_Nx + m_N * m_Nu;
constexpr int m_num_constraints = m_num_decision_vars + m_N * m_Nx;
*/
class LinearMPC {

public:
  LinearMPC(const Eigen::MatrixXd &Ad,
            const Eigen::MatrixXd &Bd, const Eigen::MatrixXd &Q,
            const Eigen::MatrixXd &Qn, const Eigen::MatrixXd &R,
            const Eigen::MatrixXd &state_bounds,
            const Eigen::MatrixXd &control_bounds,
            const int N);
  ~LinearMPC() = default;

  /**
   * @brief Constructs the quadratic cost function of the form
   * 
   * @param[in] ref_traj Reference trajectory
   * @param[out] H Quadratic component of cost
   * @param[out] f Linear component of cost
   */
  void get_cost_function(const Eigen::MatrixXd &ref_traj, Eigen::MatrixXd &H,
                         Eigen::VectorXd &f);

  /**
  * @brief Construct the linear equality constraint matrix Aeq and vector beq
  * that enforce the dynamics constraints. These constraints
  * are in the form of
  *
  * Aeq * x = beq
  *
  * @param[out] Aeq Linear equality constraint matrix
  * @param[out] beq Linear equality constraint vector
  */
  void get_dynamics_constraint(Eigen::MatrixXd &Aeq, Eigen::MatrixXd &beq);

  /**
   * @brief Collect stacked bounds
   * @param[in] initial_state Vector with initial state
   * @param[out] lb Lower bound
   * @param[out] ub Upper bound
   */
  void get_state_control_bounds(const Eigen::VectorXd &initial_state,
                                Eigen::VectorXd &lb, Eigen::VectorXd &ub);

  /**
   * @brief Collect first control value and all states and return them.
   * @param[in] x_out optimized decision variable
   * @param[out] first_control First control input to apply (Nu x 1)
   * @param[out] opt_traj Optimized state trajector (Nx x (N+1))
   */
  void get_output(const Eigen::MatrixXd &x_out,
                        Eigen::MatrixXd &first_control,
                        Eigen::MatrixXd &opt_traj);

  /**
   * @brief Collect matrices into specific type for solver and solve
   * @param[in] initial_state Vector with initial state
   * @param[in] ref_traj Matrix holding desired reference trajectory
   * @param[out] x_out Optimized output
   * @param[out] f_val Cost function value
   */
  void solve(const Eigen::VectorXd &initial_state,
             const Eigen::MatrixXd &ref_traj, Eigen::MatrixXd &x_out,
             double &f_val);

private:

  int m_N;
  int m_Nx;
  int m_Nu;

  int m_Nq;
  int m_Nx_vars;
  int m_Nx_decision;
  int m_Nconst;
  int m_num_control_vars;
  int m_num_state_vars;
  int m_num_decision_vars;
  int m_num_constraints;

  // A matrix in discrete state space form
  Eigen::MatrixXd m_Ad;

  // B matrix in discrete state space form
  Eigen::MatrixXd m_Bd;

  // Wieght matrix on state deviation from state 0 -> N-1
  Eigen::MatrixXd m_Q;

  // Weight matrix on state deviation at state N
  Eigen::MatrixXd m_Qn;

  // Weight matrix on control inputs
  Eigen::MatrixXd m_R;

  // Nx x 2 matrix of lower and upper bounds on state variables
  Eigen::MatrixXd m_state_bounds;

  // Nu x 2 matrix of lower and upper bounds on control variables
  Eigen::MatrixXd m_control_bounds;

  // MPC variables
  Eigen::MatrixXd m_Hq;
  Eigen::MatrixXd m_Hqn;
  Eigen::MatrixXd m_Hu;

  // OSQP solver
  OsqpEigen::Solver solver_;
};
} // namespace mpc
} // namespace control
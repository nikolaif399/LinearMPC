#pragma once

#include <Eigen/Dense>
#include <Eigen/Core> // Ref
#include "OsqpEigen/OsqpEigen.h"

namespace control {
namespace mpc {

constexpr int m_N = 10;
constexpr int m_Nx = 6;
constexpr int m_Nu = 4;

constexpr int m_num_decision_vars = (m_N+1) * m_Nx + m_N * m_Nu;
constexpr int m_num_constraints = m_num_decision_vars + m_N * m_Nx;

class LinearMPC {

public:
  LinearMPC(const Eigen::MatrixXd &Ad, const Eigen::MatrixXd &Bd,
            const Eigen::MatrixXd &Q, const Eigen::MatrixXd &Qn,
            const Eigen::MatrixXd &R, const Eigen::MatrixXd &state_bounds,
            const Eigen::MatrixXd &control_bounds);
  ~LinearMPC() = default;

  /**
   * @brief Constructs the quadratic cost function of the form
   *
   * @param[in] ref_traj
   * @param[out] H
   * @param[out] f
   */
  void get_cost_function(const Eigen::MatrixXd &ref_traj, Eigen::MatrixXd &H,
                         Eigen::MatrixXd &f);

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
                                Eigen::MatrixXd &lb, Eigen::MatrixXd &ub);

  /**
   * @brief Collect first control value and all states and return them.
   */
  void get_output(const Eigen::MatrixXd &x_out);

  /**
   * @brief Collect matrices into specific type for solver and solve
   * @param[in] initial_state Vector with initial state
   * @param[in] ref_traj Matrix holding desired reference trajectory
   * @param[out] x_out Optimized output
   * @param[out] f_val Cost function value
   */
  void solve(const Eigen::VectorXd &initial_state,
             const Eigen::MatrixXd &ref_traj,
             Eigen::MatrixXd &x_out, double &f_val);


private:
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
  Eigen::MatrixXd m_H;


  // OSQP solver
  OsqpEigen::Solver solver_;
};
} // namespace mpc
} // namespace control
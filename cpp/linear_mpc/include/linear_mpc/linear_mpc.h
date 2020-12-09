#pragma once

#include <Eigen/Dense>
#include "OsqpEigen/OsqpEigen.h"

namespace control {
namespace mpc {
class LinearMPC {

public:
  LinearMPC(const Eigen::MatrixXd &Ad, const Eigen::MatrixXd &Bd,
            const Eigen::MatrixXd &Q, const Eigen::MatrixXd &Qn,
            const Eigen::MatrixXd &R, const Eigen::MatrixXd &state_bounds,
            const Eigen::MatrixXd &control_bounds, const int &N);
  ~LinearMPC() = default;

  void update_horizon_length(const int &N);

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
   * @brief
   */
  void get_dynamics_constraints();

  /**
    * @brief Construct the linear equality constraint matrix Aeq and vector beq
    * that enforce the initial state and dynamics constraints. This constraints
   * are in the form of
    *
    * Aeq * x <= beq
    *
    * @param[in] initial_state Vector with initial state
    * @param[out] Aeq Linear equality constraint matrix
    * @param[out] beq Linear equality constraint vector
    */
  void get_equality_constraints(const Eigen::MatrixXd &initial_state,
                                Eigen::MatrixXd &Aeq, Eigen::VectorXd &beq);

  /**
   * @brief
   */
  void get_state_control_bounds(const Eigen::VectorXd &initial_state);

  /**
   * @brief Collect first control value and all states and return them.
   */
  void get_output(const Eigen::MatrixXd &x_out);

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

  // Number of control intervals
  const int m_N;

  // Number of states
  const int m_Nx;

  // Number of controls
  const int m_Nu;

  // Total number of decision variables
  int m_Nq;

  // MPC variables
  Eigen::MatrixXd m_Hq;
  Eigen::MatrixXd m_Hqn;
  Eigen::MatrixXd m_Hu;
  Eigen::MatrixXd m_H;
};
} // namespace mpc
} // namespace control
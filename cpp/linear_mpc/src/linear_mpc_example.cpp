#include "linear_mpc/linear_mpc.h"
#include "linear_mpc/matrix_algebra.h"
#include <iostream>

int main(int argc,char** argv)
{

  Eigen::MatrixXd Ad,Bd,Q,Qn,R,state_bounds,control_bounds;

  control::mpc::LinearMPC mpc(Ad,Bd,Q,Qn,R,state_bounds,control_bounds);

  return 0;
}
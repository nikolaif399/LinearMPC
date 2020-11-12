# Linear Model Predictive Control
A generic Linear Model Predictive Control library in MATLAB. 

Compatible backend solvers are quadprog (OSQP and qpOASES under development)

![Linear MPC Tracking](images/LinearMPCTracking.png)


## Setup Instructions
The default solver in this library is MATLAB Optimization Toolbox's built in quadprog function. To use this, just clone the repo locally and run test_solvers.m.

This library also supports qpOASES as a backend solver. To use, follow the instructions [here](https://github.com/coin-or/qpOASES) (under doc/manual) to setup the MATLAB bindings of qpOASES on your machine. Make sure that the qpOASES mex file is added to the matlab path before you attempt to use it.


close all; clc;
addpath('../..')
addpath('../../qpOASES/interfaces/matlab')
addpath('../../osqp-matlab')
addpath('../../../../ballbot_ctrl_sim/models')

N = 10;
dt = 0.01;
dt_attitude = 0.01; % Attitude controller update rate

% System parameters

% Weights on state deviation and control input
Qx = diag([100 1 1 100]);
Qn = 10*Qx;
Ru = diag([1 1 1 1]);

% Bounds on states and controls
xmin = [-inf;-inf;-inf;-inf];
xmax = [inf; inf; inf;inf];
umin = [-20; -20; -20; -20];
umax = [20; 20; 20; 20];

stateBounds = [xmin xmax];
controlBounds = [umin umax];

% Linearized dynamics
Ad = [0, 0, 1, 0;...
      0, 0, 0, 1;...
      0, -171.8039, 0, 0;...
      0, 24.3626, 0,0];
  
Bd = [0;
      0;
      5.0686;
      -0.4913];
  
Klqr = [-1.0000 -173.1954   -2.0268  -48.6683];

Acl = Ad - Bd*Klqr;
Bcl = Bd*Klqr;

% load ballbot params
[params] = get_ballbot2D_model_params();
%[params] = get_shmoo_model_params(2);
load model_params.mat;

% Setup MPC object
mpc = LinearMPC(Acl,Bcl,Qx,Qn,Ru,stateBounds,controlBounds,N,'Solver','osqp');


% Reference Trajectory Generation
refTraj = generateReference('sinusoidal',dt);
N_traj = size(refTraj,2);

qCur = refTraj(1:4,1);

qCache = [];
optCache = [];
uCache = [];
tCache = 0;

% Simulate
step = 1;
while(step < N_traj)
    tic

    % Get ref trajectory for next N steps
    if (step + N < N_traj)
        mpcRef = refTraj(1:4,step:step+N);
    else % If we reach the end of the trajectory, hover at final state
        mpcRef = refTraj(1:4,step:end);
        
        lastState = mpcRef(:,end);
        lastState(4:end) = 0; % No velocity, no orientation
        
        mpcRef = [mpcRef, repmat(lastState,1,N+1-size(mpcRef,2))];
    end
    
    % Collect MPC Control (roll,pitch,thrust commands, all in world frame)
    tic
    [Qout,fval] = mpc.solve(qCur,mpcRef);
    toc
    [u,optTraj] = mpc.getOutput(Qout); % Collect first control, optimzied state traj 
    
    % Simulate with ode45
    t0 = (step-1)*dt;
    tf = t0+dt;
    [~,qNext] = ode45(@(t,q) ballbotDynamics(t,q,u,params),t0:dt_attitude:tf,qCur);
    qCur = qNext(end,:)';
    
    % Store outputs and update step
    qCache =[qCache, qCur];
    optCache = [optCache, optTraj];
    uCache  = [uCache, u];
    tCache = [tCache,tf];
    step = step + 1;
    
end

%plotTrajectory(qCache,optCache,uCache,refTraj,dt,false)
plotResults(qCache,optCache,uCache,refTraj,dt,false)



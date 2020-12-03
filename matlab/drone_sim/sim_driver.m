addpath('..')
addpath('../qpOASES/interfaces/matlab')
addpath('../osqp-matlab')

N = 15;
dt = 0.04;
dt_attitude = 0.002; % Attitude controller update rate

% System parameters
params.g = 9.81;
params.m = 5;
k_cmd = 1;
tau = 0.01;

% Weights on state deviation and control input
Qx = diag([1000 1000 1000 1 1 1 10 10]);
Qn = 10*Qx;
Ru = diag([0.1 0.1 0.01 0]);

% Bounds on states and controls
xmin = [-inf;-inf;-inf;-inf;-inf;-inf; -pi/3;-pi/3];
xmax = [inf; inf; inf;inf;inf;inf; pi/3; pi/3];
umin = [-pi/2;-pi/2; 0.5*params.m*params.g;1];
umax = [pi/2; pi/2; 3*params.m*params.g; 1];

stateBounds = [xmin xmax];
controlBounds = [umin umax];

% Linearized dynamics
Ad = [1 0 0 dt 0  0  0  dt^2*params.g/2;
      0 1 0 0  dt 0 -dt^2*params.g/2  0;
      0 0 1 0  0  dt 0  0;
      0 0 0 1  0  0  0  dt*params.g;
      0 0 0 0  1  0  -dt*params.g 0;
      0 0 0 0  0  1  0  0;
      0 0 0 0  0  0  1-dt/tau  0;
      0 0 0 0  0  0  0  1-dt/tau];
  
Bd = [0 0 0 0;
      0 0 0 0;
      0 0 dt^2/(2*params.m) -dt^2*params.g/2;
      0 0 0 0;
      0 0 0 0;
      0 0 dt/params.m -params.g*dt;
      k_cmd*dt/tau 0  0  0;
      0 k_cmd*dt/tau  0  0];

% Setup MPC object
mpc = LinearMPC(Ad,Bd,Qx,Qn,Ru,stateBounds,controlBounds,N,'Solver','osqp');

% Reference Trajectory Generation
refTraj = generateReference('straight',dt);
N_traj = size(refTraj,2);

qCur = refTraj(:,1);

qCache = {};
optCache = {};

% Simulate
step = 1;
while(step < N_traj)
    tic

    % Get ref trajectory for next N steps
    if (step + N < N_traj)
        mpcRef = refTraj(:,step:step+N);
    else % If we reach the end of the trajectory, hover at final state
        mpcRef = refTraj(:,step:end);
        
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
    [~,qNext] = ode45(@(t,q) droneDynamics(t,q,u,params),t0:dt_attitude:tf,qCur);
    qCur = qNext(end,:)';
    
    % Store outputs and update step
    qCache{step} = qCur;
    optCache{step} = optTraj;
    step = step + 1;
    
end

plotTrajectory(qCache,optCache,refTraj,dt,false)



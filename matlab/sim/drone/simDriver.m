addpath('../..')
addpath('../../qpOASES/interfaces/matlab')
addpath('../../osqp-matlab')

N = 20;
dt = 0.04;
dt_attitude = 0.0025; % Attitude controller update rate

% System parameters
params.g = 9.81;
params.m = 5;
k_cmd = 1;
tau = 1/10;

% Weights on state deviation and control input
Qx = diag([20 20 100 5 5 10 1 1]); % {x, y, z, xdot, ydot, zdot, roll, pitch}
Qn = 10*Qx; 
Ru = diag([10 10 0.2 0]); % {roll, pitch, thrust, 0}

% Bounds on states and controls
angle_lim = 20*pi/180;
xmin = [-inf;-inf;-inf;-inf;-inf;-inf; -angle_lim;-angle_lim];
xmax = [inf; inf; inf;inf;inf;inf; angle_lim; angle_lim];
umin = [-angle_lim;-angle_lim; 0.2*params.g;1];
umax = [angle_lim; angle_lim; 1.5*params.g; 1];

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
      0 0 dt^2/2 -dt^2*params.g/2;
      0 0 0 0;
      0 0 0 0;
      0 0 dt -params.g*dt;
      k_cmd*dt/tau 0  0  0;
      0 k_cmd*dt/tau  0  0];

% Setup MPC object
mpc = LinearMPC(Ad,Bd,Qx,Qn,Ru,stateBounds,controlBounds,N,'Solver','osqp');

% Reference Trajectory Generation
refTraj = generateReference('sawtooth_vertical',dt);% {straight, rising_spiral}
N_traj = size(refTraj,2);

qCur = refTraj(:,1);

qCache = {};
optCache = {};

% Simulate
u_arr = [];
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
    [u,optTraj] = mpc.getOutput(Qout); % Collect first control, optimized state traj 
    u_arr = [u_arr, u];
    
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

% plot MPC outputs
figure(2)
subplot(1,3,1)
plot( u_arr(1,:) )
ylabel('Roll')
subplot(1,3,2)
plot(u_arr(2,:))
ylabel('Pitch')
subplot(1,3,3)
plot(u_arr(3,:))
ylabel('Thrust')

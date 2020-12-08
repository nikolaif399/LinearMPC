addpath('../..')
addpath('../../qpOASES/interfaces/matlab')
addpath('../../osqp-matlab')

N = 20;
dt_control = 0.05;
dt_inner = 0.01;
g = 9.81;
mcart = 0.493;
mpend = 0.312;
Ipend = 0.00024;
l = 0.04;
f = 0.01;

% Weights on state deviation and control input
Qx = diag([100 100 10 10]);
Qn = 10*Qx;
Ru = 1;

% Bounds on states and controls
xmin = [-inf;-inf;-pi/4;-inf];
xmax = [inf; inf; pi/4;inf];
umin = -10;
umax = 10;

stateBounds = [xmin xmax];
controlBounds = [umin umax];

% Linearized dynamics
denom = Ipend*(mcart*mpend) + mcart*mpend*l^2;
A = [0 1 0 0;
      0 -(Ipend+mpend*l^2)*f/denom mpend^2*g*l^2/denom 0;
      0 0 0 1;
      0 -mpend*l*f/denom mpend*g*l*(mcart*mpend)/denom 0];
  
B = [0; Ipend+mpend*l^2/denom;0;mpend*l/denom];

Ad = expm(A*dt_control);
Bd = expm(A*dt_control - eye(size(A)))*pinv(A)*B;

params.A = A;
params.B = B;

% Setup MPC object
mpc = LinearMPC(Ad,Bd,Qx,Qn,Ru,stateBounds,controlBounds,N,'Solver','osqp');

% Reference Trajectory Generation
ts = 1:200;
refTraj = [0.3*sin(0.1*ts);zeros(3,length(ts))];

N_traj = size(refTraj,2);

qCur = refTraj(:,1);
%qCur = [0.1;0;0;0];

qCache = {};
optCache = {};

% Simulate
step = 1;
while(step < N_traj)

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
    t0 = (step-1)*dt_control;
    tf = t0+dt_control;
    [~,qNext] = ode45(@(t,q) tumbllerDynamics(t,q,u,params),t0:dt_inner:tf,qCur);
    qCur = qNext(end,:)';
    
    % Store outputs and update step
    qCache{step} = qCur;
    optCache{step} = optTraj;
    step = step + 1;
    
end

plotTrajectory(qCache,optCache,refTraj,dt_control,false)



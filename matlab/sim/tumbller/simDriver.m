addpath('../..')
addpath('../../qpOASES/interfaces/matlab')
addpath('../../osqp-matlab')

N = 20;
dt_control = 0.01;
dt_inner = 0.001;
g = 9.81;
mcart = 0.493;
mpend = 0.312;
Ipend = 0.00024;
l = 0.04;
f = 0.01;

% Weights on state deviation and control input
Q = diag([1000 0.1 3 0.1]);
Qn = 10*Q;
Ru = 0.1;

% Bounds on states and controls
xmin = [-inf;-1;-pi/2;-inf];
xmax = [inf; 1; pi/2;inf];
umin = -1;
umax = 1;

stateBounds = [xmin xmax];
controlBounds = [umin umax];

% Linearized dynamics
denom = Ipend*(mcart+mpend) + mcart*mpend*l^2;
A = [0 1 0 0;
      0 -(Ipend+mpend*l^2)*f/denom mpend^2*g*l^2/denom 0;
      0 0 0 1;
      0 -mpend*l*f/denom mpend*g*l*(mcart+mpend)/denom 0];
  
B = [0; Ipend+mpend*l^2/denom;0;mpend*l/denom];

sys = ss(A,B,eye(size(A)),zeros(4,1));
sysd = c2d(sys,dt_control,'tustin');

Ad = sysd.A;%expm(A*dt_control);
Bd = sysd.B;%expm(A*dt_control - eye(size(A)))*pinv(A)*B;

params.A = A;
params.B = B;

% Setup MPC object
mpc = LinearMPC(Ad,Bd,Q,Qn,Ru,stateBounds,controlBounds,N,'Solver','osqp');

% Reference Trajectory Generation
ts = 0:dt_control:8;
refTraj = zeros(4,length(ts));
refTraj(1,:) = 0.3*sin(0.8*ts);

N_traj = size(refTraj,2);

qCur = [0 0 0 0]';

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
    
    % Collect MPC Control
    tic
    [Qout,fval] = mpc.solve(qCur,mpcRef);
    toc
    [u,optTraj,uTraj] = mpc.getOutput(Qout); % Collect first control, optimzied state traj 
    
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

plotTrajectory(qCache,optCache,refTraj,dt_control,true)



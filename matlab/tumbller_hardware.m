addpath('osqp-matlab')

N = 30;
dt_control = 0.01;

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
xmin = [-inf;-0.5;-pi/4;-inf]; % x,dx,theta,dtheta
xmax = [inf; 0.5; pi/4;inf];
umin = -1; % Torque min
umax = 1; % Torque max

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

% Setup MPC object
mpc = LinearMPC(Ad,Bd,Qx,Qn,Ru,stateBounds,controlBounds,N,'Solver','osqp');

while(true)
    tic
    
    %%%%%%% GET CURRENT STATE FROM BLUETOOTH %%%%
    qCur = zeros(4,1); % Current state, receive from Tumbller
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    mpcRef = zeros(4,N+1);
    mpcRef(1,:) = qCur(1); % Set desired x traj to current x position
    
    [Qout,~] = mpc.solve(qCur,mpcRef);
    [u,optTraj] = mpc.getOutput(Qout); % Collect first control, optimized state traj
    optTraj(:,1) = []; % Crop off current state
    
    %%%%%%% SEND OPTIMIZED TRAJECTORY OVER BLUETOOTH %%%%%%%
    % Send optTraj, which is 4 x N and contains the optimized reference
    % trajectory (x,dx,theta,dtheta) for the next N timesteps
    
    % bluetooth send function here
    
    %%%%%%%%%%%%%%%%%%%%%%%%
    loop_time = toc;
end



%% Prepare workspace
clear all; clc; close all;

%% Load Model
modelName = 'ballbot2D';
load(strcat('syms_model_',modelName,'.mat'));

% Model Parameters values
[params, unpacked_params] = get_ballbot2D_model_params();
load(unpacked_params);

% Ballbot 2D Model
% Define symbolic variables
syms phi theta dphi dtheta ddphi ddtheta tau real % State variables

% State vector
q = [theta; phi];
dq = [dtheta; dphi];
X = [q;dq];
u = tau;

% Linearize System
Alin = jacobian(dX,X);
Blin = jacobian(dX,u);

% subs parameters
theta = 0; phi = 0;
dtheta = 0; dphi = 0;

Anum = double(subs(Alin));
Bnum = double(subs(Blin));

%%
Nu = size(Bnum,2); % Number of control inputs (appended gravity term)
Nx = size(Bnum,1); % Number of states
N = 50; % Time horizons to consider
Nq = (N+1)*Nx + N*Nu; % Toal number of decision variables
dt = 0.1; % Time step
m = 5; % Mass of drone
g=9.81;
k_cmd=1;
tau = 0.01;



% Weights on state deviation and control input
Qx = diag([100 100 1 100]);
Qn = 10*Qx;
Ru = diag([1]);

% Bounds on states and controls
xmin = [-inf;-inf;-inf;-inf];
xmax = [inf; inf; inf;inf];
umin = [-20];
umax = [20];

stateBounds = [xmin xmax];
controlBounds = [umin umax];

% Reference trajectory
theta_ref = sin(0.5*(0:N));
phi_ref = zeros(1,N+1);

dtheta_ref = zeros(1,N+1);
dphi_ref = zeros(1,N+1);
x0 = [theta_ref(1);phi_ref(1);dtheta_ref(1);dphi_ref(1)];
refTraj = [theta_ref; phi_ref; dtheta_ref; dphi_ref];

% Setup MPC object
mpc = LinearMPC(Anum,Bnum,Qx,Qn,Ru,stateBounds,controlBounds,N);
mpc.updateHorizonLength(N);
mpc.setupCostFunction(refTraj);
[H,f,A,b,Aeq,beq,lb,ub] = mpc.getQuadprogMatrices(x0,refTraj);

[Qout,fval] = quadprog(H,f,A,b,Aeq,beq,lb,ub);

% Extract results
xend = Nx*(N+1);
theta_out = Qout(1:Nx:xend);
phi_out = Qout(2:Nx:xend);
dtheta_out = Qout(3:Nx:xend);
dphi_out = Qout(4:Nx:xend);

X_out = [theta_out, phi_out,dtheta_out, dphi_out];
t_out = 0:dt:dt*50;
ustart = xend+1;
u1out = Qout(ustart+0:Nu:end);

%% Plot Results 
figure
plot(theta_ref,'b*-','DisplayName','Reference')
hold on
plot(theta_out, 'r*-','DisplayName','MPC')
grid on
ylabel('Theta [rad]')
xlabel('Steps')
title('Ballbot ball trajectory')

figure
plot(phi_ref,'b*-','DisplayName','Reference')
hold on
plot(phi_out, 'r*-','DisplayName','MPC')
grid on
ylabel('Phi [rad]')
xlabel('Steps')
title('Ballbot body lean trajectory')

%% Run Animation
Anim.speed = 1;
Anim.plotFunc = @draw_bb;
animate(t_out,X_out,Anim);

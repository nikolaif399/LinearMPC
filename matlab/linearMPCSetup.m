Nu = 5; % Number of control inputs (appended gravity term)
Nx = 6; % Number of states
N = 40; % Time horizons to consider
Nq = N*(Nx+1) + N*(Nu); % Toal number of decision variables
dt = 0.1; % Time horizon
m = 1; % Mass of drone

% Weights on state deviation and control input
Qx = diag([100 100 1000 1 1 1]);
Qn = 10*Qx;
Ru = diag([1 1 1 0.01 0]);

% Linearized dynamics
Ad = [1 0 0 dt 0  0;
      0 1 0 0  dt 0;
      0 0 1 0  0  dt;
      0 0 0 1  0  0 ;
      0 0 0 0  1  0;
      0 0 0 0  0  1];
  
Bd = [0 0 0 0 0;
      0 0 0 0 0;
      0 0 0 0 0;
      -9.8 0 0 0 0;
      0 0 9.8 0 0;
      0 0 0 dt/m -9.8*dt];

% Quadratic cost function
Hq = kron(eye(N),Qx);
Hqn = Qn;
Hu = kron(eye(N),Ru);

H = blkdiag(Hq,Hqn,Hu);

xref = cos(0.5*(0:N));
yref = sin(0.5*(0:N));
zref = 0.1*(0:N);

dxref = zeros(1,N+1);
dyref = zeros(1,N+1);
dzref = zeros(1,N+1);

y = [xref;yref;zref;dxref;dyref;dzref];
y = y(:);
fx = y'*blkdiag(kron(eye(N),Qx),Qn);
fu = zeros(N*Nu,1);
f = -[fx';fu];

% Linearized dynamics equality constraints
A_padded_eye = padarray(kron(eye(N), -eye(Nx,Nx)), [0 Nx],0,'pre');
A_padded_ad = padarray(kron(eye(N),Ad),[0,Nx],0,'post');
Aeq = [A_padded_eye + A_padded_ad, kron(eye(N),Bd)];
beq = zeros(N*Nx,1);

% Bounds on states and controls
xmin = [-inf;-inf;-inf;-inf;-inf;-inf];
xmax = [inf; inf; inf;inf;inf;inf];
umin = [-pi/2;-pi/2;-pi/2;-20;1];
umax = [pi/2;pi/2;pi/2;20;1];

x0 = [xref(1);yref(1);zref(1);dxref(1);dyref(1);dzref(1)];
Lb = [x0;repmat(xmin,N,1);repmat(umin,N,1)];
Ub = [x0;repmat(xmax,N,1);repmat(umax,N,1)];

% No inequality constraints (control constraints specified as bounds)
A = [];
b = [];

% Initial condition
q_initial = []; % Include as last arg in quadprog if desired

tic
[Qout,fval] = quadprog(H,f,A,b,Aeq,beq,Lb,Ub);
fval
toc

xend = Nx*(N+1);
xout = Qout(1:Nx:xend);
yout = Qout(2:Nx:xend);
zout = Qout(3:Nx:xend);
dxout = Qout(4:Nx:xend);
dyout = Qout(5:Nx:xend);
dzout = Qout(6:Nx:xend);

ustart = xend+1;
u1out = Qout(ustart+0:Nu:end);
u2out = Qout(ustart+1:Nu:end);
u3out = Qout(ustart+2:Nu:end);
u4out = Qout(ustart+3:Nu:end);
u5out = Qout(ustart+4:Nu:end);

figure
plot3(xref,yref,zref, 'b*-')
hold on
plot3(xout,yout,zout, 'r*-')
grid on
xlabel('x')
ylabel('y')
zlabel('z')
title('Drone Trajectory')
legend('Reference', 'MPC')
xlim([-5 5])
ylim([-5 5])
zlim([0 5])

figure
subplot(1,2,1)
plot(u4out)
title('Thrust')
ylim([umin(4), umax(4)])

subplot(1,2,2)
plot(zref)
hold on
plot(zout)
title('Z tracking')
legend('Reference', 'MPC')


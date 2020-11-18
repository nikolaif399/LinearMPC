% Linear Drone MPC example

% Configurable parameters
Nu = 5; % Number of control inputs (appended gravity term)
Nx = 6; % Number of states
N = 20; % Time horizons to consider
dt = 0.1; % Time horizon
m = 1; % Mass of drone

% Weights on state deviation and control input
Qx = diag([100 100 1000 1 1 1]);
Qn = 5*Qx;
Ru = diag([1 1 1 1 1]);

% Bounds on states and controls
xmin = [-inf;-inf;-inf;-inf;-inf;-inf];
xmax = [inf; inf; inf;inf;inf;inf];
umin = [-pi/2;-pi/2;-pi/2;-20;1];
umax = [pi/2;pi/2;pi/2;20;1];

stateBounds = [xmin xmax];
controlBounds = [umin umax];

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
      -9.8*dt 0 0 0 0;
      0 9.8*dt 0 0 0;
      0 0 0 dt/m -9.8*dt];

% Reference trajectory
xref = cos(0.5*(0:N));
yref = sin(0.5*(0:N));
zref = 0.1*(0:N);

dxref = zeros(1,N+1);
dyref = zeros(1,N+1);
dzref = zeros(1,N+1);  
x0 = [xref(1);yref(1);zref(1);dxref(1);dyref(1);dzref(1)];
refTraj = [xref;yref;zref;dxref;dyref;dzref];

% Setup MPC object
addpath('qpOASES/interfaces/matlab')
mpc = LinearMPC(Ad,Bd,Qx,Qn,Ru,stateBounds,controlBounds,N,'Solver','qpoases');

[Qout,fval] = mpc.solve(x0,refTraj);


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


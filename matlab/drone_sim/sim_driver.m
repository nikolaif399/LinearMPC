addpath('..')
addpath('../qpOASES/interfaces/matlab')

vis_factor = 3; % Slows down animation by this factor, should be 1 if MPC solves in real time

N_traj = 200;
N = 20;
dt = 0.05;
dt_attitude = 0.01; % Attitude controller update rate

% System parameters
params.g = 9.81;
params.m = 5;
k_cmd = 1;
tau = 0.01;

% Weights on state deviation and control input
Qx = diag([1000 1000 1000 1 1 1 10 10]);
Qn = 3*Qx;
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
mpc = LinearMPC(Ad,Bd,Qx,Qn,Ru,stateBounds,controlBounds,N,'Solver','qpoases');

% Reference Trajectory Generation
xref = cos(0.02*(0:N_traj-1));
yref = sin(0.02*(0:N_traj-1));
zref = 0.02*(0:N_traj-1);
rollref = zeros(size(zref));
pitchref = zeros(size(zref));

dxref = [0, diff(xref)/dt];
dyref = [0, diff(yref)/dt];
dzref = [0, diff(zref)/dt]; 

refTraj = [xref;yref;zref;dxref;dyref;dzref;rollref;pitchref];
qCur = refTraj(:,1);
nx = length(qCur);

% Simulate
step = 1;
v = VideoWriter('animation.mp4','MPEG-4');
v.FrameRate = 1/dt;
open(v)
figure
while(step + N < N_traj)
    tic
    %step
    % Get ref trajectory for next N steps
    mpcRef = refTraj(:,step:step+N);
    
    % Collect MPC Control (roll,pitch,thrust commands, all in world frame)
    [Qout,fval] = mpc.solve(qCur,mpcRef);
    Nx = 8;
    Nu = 4;
    xend = Nx*(N+1);
    xout = Qout(1:Nx:xend);
    yout = Qout(2:Nx:xend);
    zout = Qout(3:Nx:xend);
    dxout = Qout(4:Nx:xend);
    dyout = Qout(5:Nx:xend);
    dzout = Qout(6:Nx:xend);
    rollout = Qout(7:Nx:xend);
    pitchout = Qout(8:Nx:xend);

    ustart = xend+1;
    u1out = Qout(ustart+0:Nu:end);
    u2out = Qout(ustart+1:Nu:end);
    u3out = Qout(ustart+2:Nu:end);
    u4out = Qout(ustart+3:Nu:end);
    
    u = [u1out(1);u2out(1);u3out(1)];
    
    % Simulate nonlinear drone dynamic model
    t0 = (step-1)*dt;
    tf = t0 + dt;
    [~,qNext] = ode45(@(t,q) droneDynamics(t,q,u,params),t0:dt_attitude:tf,qCur);
    qCur = qNext(end,:)';
    
    % Update plots and sleep for real time animation
    plot3(mpcRef(1,:),mpcRef(2,:),mpcRef(3,:),'b')
    hold on
    plot3(xout,yout,zout, 'g-')
    plot3(qCur(1),qCur(2),qCur(3),'r*')
    hold off
    title('Drone Trajectory')
    
    xlim([min(xref)-0.2 max(xref)+0.2])
    ylim([min(yref)-0.2 max(yref)+0.2])
    zlim([min(zref)-0.2 max(zref)+0.2])
    grid on
    
    writeVideo(v,getframe(gcf));
    step = step + 1;
    tsleep = dt*vis_factor - toc;
    if (tsleep > 0)
        pause(tsleep)
    end
    
end

close(v)





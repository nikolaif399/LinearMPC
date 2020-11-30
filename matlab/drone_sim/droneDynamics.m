function dqdt = droneDynamics(t,q,u,params)
%droneDynamics Drone dynamics function, includes low level attitude
%controller

% q = x, y, z, dx, dy, dz, roll, pitch
x=q(1);y=q(2);z=q(3);
dx=q(4);dy=q(5);dz=q(6);
roll=q(7);pitch=q(8);

% u = roll, pitch, thrust  (command, angles in world frame);
roll_command=u(1);pitch_command=u(2);thrust_command=u(3);

% Attitude controller here, update dpitch and droll

R = rotz(0)*roty(pitch_command)*rotx(roll_command); % roll and pitch in world frame so yaw irrelevant
acc = [0;0;-params.g] + R*[0;0;thrust_command/params.m];

dqdt = [dx;dy;dz;acc;0;0];

end


function [] = plotQuad(ax,q)
%PLOTQUAD Summary of this function goes here
%   q should be x, y, z, dx, dy, dz

quadDiam = 0.4; % Diameter of quadcopter

bodyPos = reshape(q(1:3),[3 1]);
motorPos1Body = [quadDiam/2;0;0];
motorPos2Body = [0;quadDiam/2;0];
motorPos3Body = [-quadDiam/2;0;0];
motorPos4Body = [0;-quadDiam/2;0];

% Rotation matrix from body to world
roll = q(7);
pitch = q(8);
R = rotz(0)*roty(rad2deg(pitch))*rotx(rad2deg(roll));

motorPos1 = R*motorPos1Body + bodyPos;
motorPos2 = R*motorPos2Body + bodyPos;
motorPos3 = R*motorPos3Body + bodyPos;
motorPos4 = R*motorPos4Body + bodyPos;

plot3(ax, bodyPos(1),bodyPos(2),bodyPos(3),'r*')
plot3(ax,[motorPos1(1),motorPos3(1)],[motorPos1(2),motorPos3(2)],[motorPos1(3),motorPos3(3)],'ko-')
plot3(ax,[motorPos2(1),motorPos4(1)],[motorPos2(2),motorPos4(2)],[motorPos2(3),motorPos4(3)],'ko-')

end


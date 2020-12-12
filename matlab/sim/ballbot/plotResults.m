function [] = plotResults(qCache,optCache,uCache,refTraj,dt,record)
%PLOTTRAJECTORY 
% qCache Cell array of states
% optCache cell array of optimized trajectory segments

N = size(qCache,2);

% Control command history
figure(1)
subplot(4,1,1)
plot(uCache(1,:))

subplot(4,1,2)
plot(rad2deg(uCache(2,:)))

subplot(4,1,3)
plot(uCache(3,:))

subplot(4,1,4)
plot(rad2deg(uCache(3,:)))

% Actual trajectory
figure(2)
subplot(4,1,1)
plot(qCache(1,:))

subplot(4,1,3)
plot(rad2deg(qCache(2,:)))

subplot(4,1,2)
plot(qCache(3,:))

subplot(4,1,4)
plot(rad2deg(qCache(3,:)))

% Reference trajectory
figure(3)
subplot(2,2,1)
plot(refTraj(1,:))
ylabel('Theta [rad]');

subplot(2,2,3)
plot(rad2deg(refTraj(2,:)))
ylabel('Phi [deg]');

subplot(2,2,2)
plot(refTraj(3,:))
ylabel('dTheta [rad/s]');

subplot(2,2,4)
plot(rad2deg(refTraj(4,:)))
ylabel('dPhi [deg/s]');


end
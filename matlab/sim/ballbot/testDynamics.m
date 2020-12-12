
t0 = 0.0;
dt_attitude = 0.01;
tf = 10.0;
[params] = get_ballbot2D_model_params(2);
%[params] = get_shmoo_model_params(2);
load(params);
u = zeros(4,1);
qCur = [0;1*pi/180;0;0;];
[tHist, xHist] = ode45(@(t,q) ballbotDynamics(t,q,u,params),t0:dt_attitude:tf,qCur);

figure(3)
subplot(2,2,1)
plot(xHist(1,:))
ylabel('Theta [rad]');

subplot(2,2,3)
plot(rad2deg(xHist(2,:)))
ylabel('Phi [deg]');

subplot(2,2,2)
plot(xHist(3,:))
ylabel('dTheta [rad/s]');

subplot(2,2,4)
plot(rad2deg(xHist(4,:)))
ylabel('dPhi [deg/s]');
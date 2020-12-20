clear all;
close all;

% Set plotting parameters
set(0, 'DefaultAxesFontSize', 24);
set(0, 'DefaultLineMarkerSize', 10);
set(0,'defaultfigurecolor',[1 1 1]);
set(0,'DefaultAxesXGrid','off','DefaultAxesYGrid','off')
set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

rollTruek1 = 0.0;
rollTruek = 0.0;
rollVelTrue = 0.0;
gyroDriftTrue = 1.0;
gyroCalBiasTrue = 0.01;
gyroSigmaNoise = 0.002;
accelSigmaNoise = sqrt(0.05);

accelMeask1 = 0.0;
accelMeask = 0.0;
gyroMeask1 = 0.0;
gyroMeask = 0.0;

dt = 0.004;
time = 0.0;
dataStore = [];

angleRollk1 = 0.0;
angleRollk = 0.0;

% Complementary filter
k1 = 0.99;
k2 = 0.01;

% Kalman filter parameters
xk = zeros(1,2);
pk = [0.5 0 0 0.01];
K = zeros(1,2);
phi = [1 dt 0 1];
psi = [dt 0];
R = 0.03;
Q = [0.002^2 0 0 0];
H = [1 0];

% Create a simulation at 250 Hz
endPoint = 250000;
for i = 1:endPoint
    
    rollVelTrue = 0.0;
    
    if(i >= 500) && (i < 750)
        rollVelTrue = 5.0;
    end
    if(i >= 1250) && (i < 1500)
        rollVelTrue = -10.0;
    end
    if(i >= 1750) && (i < 2000)
        rollVelTrue = 5.0;
    end


% Gyro and accelerometer measurements at each timestep
rollTruek1 = rollTruek + rollVelTrue * dt;

accelMeask1 = rollTruek1 + randn(1) * accelSigmaNoise;

gyroReadk1 = rollVelTrue - gyroDriftTrue + randn(1) * gyroSigmaNoise + gyroCalBiasTrue;

gyroMeask1 = gyroReadk1 - gyroCalBiasTrue;

% Complementary filter 
angleRollk1 = k1 * (angleRollk+gyroMeask1*dt) + k2*accelMeask1;

% Kalman filter implementation 
uk = gyroMeask1;
zk = accelMeask1;

% xkMinus = phi * xk + psi * uk in equation form for arduino implementation
xk1Minus(1) = phi(1) * xk(1) + phi(2) * xk(2) + psi(1) * uk;
xk1Minus(2) = phi(3) * xk(1) + phi(4) * xk(2) + psi(2) * uk;

% pk1Minus = phi * pk * phi' + Q
pk1Minus(1) = (phi(1)*pk(1)+phi(2)*pk(3))*phi(1)+ ...
    (phi(1)*pk(2)+phi(2)*pk(4))*phi(2)+Q(1);

pk1Minus(2) = (phi(1)*pk(1)+phi(2)*pk(3))*phi(3)+ ...
    (phi(1)*pk(2)+phi(2)*pk(4))*phi(4)+Q(2);

pk1Minus(3) = (phi(3)*pk(1)+phi(4)*pk(3))*phi(1)+ ...
    (phi(3)*pk(2)+phi(4)*pk(4))*phi(2)+Q(3);

pk1Minus(4) = (phi(3)*pk(1)+phi(4)*pk(3))*phi(3)+ ...
    (phi(3)*pk(2)+phi(4)*pk(4))*phi(4)+Q(4);

% S = H * pk1Minus * H' + R
S = (H(1) * pk1Minus(1) + H(2) * pk1Minus(3)) * H(1) + ...
    (H(1) * pk1Minus(2) + H(2) * pk1Minus(4)) * H(1) + R;

% K = pk1Minus * H' * inv(S)
K(1) = (pk1Minus(1) * H(1) + pk1Minus(2) * H(2)) / S;
K(2) = (pk1Minus(3) * H(1) + pk1Minus(4) * H(2)) / S;

% xk1 = xk1Minus + K * (zk - H * xk1Minus)
xk1(1) = xk1Minus(1) + K(1) * (zk - (H(1) * xk1Minus(1) + ...
    H(2) * xk1Minus(2)));

xk1(2) = xk1Minus(2) + K(2) * (zk - (H(1) * xk1Minus(1) + ...
    H(2) * xk1Minus(2)));

% pk1 = (eye(2,2) - K * H) * pk1Minus
pk1(1,1) = (1 - K(1) * H(1)) * pk1Minus(1) + ...
    (0 - K(1) * H(2)) * pk1Minus(3);

pk1(1,2) = (1 - K(1) * H(1)) * pk1Minus(2) + ...
    (0 - K(1) * H(2)) * pk1Minus(4);

pk1(2,1) = (1 - K(2) * H(1)) * pk1Minus(1) + ...
    (0 - K(2) * H(2)) * pk1Minus(3);

pk1(2,2) = (1 - K(2) * H(1)) * pk1Minus(2) + ...
    (0 - K(2) * H(2)) * pk1Minus(4);

% Cache data
dataStore(i,:) = [time rollTruek1 accelMeask1 gyroMeask1 angleRollk1 xk1(1) xk1(2)];

% Reset values for next iteration 
angleRollk = angleRollk1;
rollTruek = rollTruek1;
time = time + dt;

xk = xk1;
pk = pk1;

end

subplot(2,1,1)
plot(dataStore(1:endPoint,1), dataStore(1:endPoint,2), '--r', "LineWidth", 2);
hold on;
plot(dataStore(1:endPoint,1),dataStore(1:endPoint,5), 'b', "LineWidth", 1);
hold on;
plot(dataStore(1:endPoint,1),dataStore(1:endPoint,6), 'k', "LineWidth", 1);
title("Complementary Filter vs Kalman Filter Comparison")
grid on
axis([0 10 -10 15]);
xlabel("Time [Sec]");
ylabel("Angle (deg)");
legend('True', 'Comp Filter', 'KF')

subplot(2,1,2)
plot(dataStore(1:endPoint,1), abs(dataStore(1:endPoint,2)-dataStore(1:endPoint,5)), 'b', "LineWidth", 1);
hold on 
plot(dataStore(1:endPoint,1), abs(dataStore(1:endPoint,2)-dataStore(1:endPoint,6)), 'k', "LineWidth", 1);

xlabel("Time [Sec]");
ylabel("Error [deg]");


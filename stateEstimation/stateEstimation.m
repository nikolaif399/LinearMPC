clear all;
close all;

rollTruek1 = 0.0;
rollTruek = 0.0;
rollVelTrue = 0.0;
gyroDriftTrue = 1.0;
gyroCalBiasTrue = 0.01;
gyroSigmaNoise = 0.002;
accelSigmaNoise = sqrt(0.03);

accelMeask1 = 0.0;
accelMeask = 0.0;
gyroMeask1 = 0.0;
gyroMeask = 0.0;

dt = 0.004;
time = 0.0;
dataStore = zeros(5010, 5);

angleRollk1 = 0.0;
angleRollk = 0.0;

k1 = 0.99;
k2 = 0.001;

% Create a simulation at 250 Hz
endPoint = 2500;
for i = 1:endPoint
    if(i >= 500) && (i < 750)
        rollVelTrue = 30.0;
    end
    if(i >= 1250) && (i < 1500)
        rollVelTrue = -40.0;
    end
    if(i >= 1750) && (i < 2000)
        rollVelTrue = 10.0;  
    end


% Gyro and accelerometer measurements at each timestep
rollTruek1 = rollTruek + rollVelTrue * dt;

accelMeask1 = rollTruek1 + randn(1) * accelSigmaNoise;

gyroReadk1 = rollVelTrue - gyroDriftTrue + randn(1) * gyroSigmaNoise + gyroCalBiasTrue;

gyroMeask1 = gyroReadk1 - gyroCalBiasTrue;

% Complementary filter 
angleRollk1 = k1 * (angleRollk+gyroMeask1*dt) + k2*accelMeask1;

% Cache data
dataStore(i,:) = [time rollTruek1 accelMeask1 gyroMeask1 angleRollk1];

% Reset values for next iteration 
time = time + dt;
angleRollk = angleRollk1;
rollTruek = rollTruek1;

end

figure(1)
plot(dataStore(1:endPoint,1), dataStore(1:endPoint,2));
% hold on;
% plot(dataStore(:,1), dataStore(:,5));


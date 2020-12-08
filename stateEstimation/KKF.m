% Kinematic Kalman Filter for position and acceleration measurements

meas = load("meas.mat");
time = load("time.mat");

x_pos = meas.xx(1,:);
y_pos = meas.xx(2,:);

sp = 0.1; % Sigma for position
p_x = 0;
p_y = 0;
m = length(x_pos); % Number of measurements
time = time.t;

% position measurement
mpx = x_pos + sp .* randn(1,m);
mpy = y_pos + sp .* randn(1,m);

% Acceleration measurement
sa = 0.1; % Sigma for acceleration
ax = 0.0; % in X
ay = 0.0; % in Y

max = ax + sa .* randn(1,m);
may = ay + sa .* randn(1, m);

% Stack position and acceleration measurements
% meas = [mpx; mpy; max; may];
meas = [mpx; mpy];

%% Setup matrices

dt = 0.1;
% State propagation matrix
A = [1.0, 0.0, dt, 0.0, 1/2.0*dt^2, 0.0;
     0.0, 1.0, 0.0, dt, 0.0, 1/2.0*dt^2;
     0.0, 0.0, 1.0, 0.0, dt, 0.0;
     0.0, 0.0, 0.0, 1.0, 0.0, dt;
     0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
     0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
 
% Measurement function (Position and acceleration) 
% H = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
%     0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
%     0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
%     0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
H = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0];


ra = 10.0^2;   % Noise of Acceleration Measurement
rp = 0.1^2;  % Noise of Position Measurement
% R = [rp, 0.0, 0.0, 0.0;
%     0.0, rp, 0.0, 0.0;
%     0.0, 0.0, ra, 0.0;
%     0.0, 0.0, 0.0, ra];
R = [rp, 0.0;
     0.0, rp];

sa = 0.001;
G = [1/2.0*dt^2;
    1/2.0*dt^2;
    dt;
    dt;
    1.0;
    1.0];
Q = G * G' * sa^2;

I = eye(6);

P = diag([100.0, 100.0, 10.0, 10.0, 1.0, 1.0]);

% Cache variables 
xEst = [];

x = zeros(6,1);

% Main Kalman filter loop
for filterstep = 1:m
    
    % Time Update (Prediction)
    % Project the state ahead
    
    x = A * x;
    
    % Project the error covariance ahead
    P = A * P * A' + Q;    
    
    
    % Measurement Update (Correction)
    % Compute the Kalman Gain
    S = H * P * H' + R;
    K = P * H' * inv(S);
    
        
    % Update the estimate via z
    Z = meas(:,filterstep);
    y = Z - (H * x); % Innovation or Residual
    x = x + (K * y);
        
    % Update the error covariance
    P = (I - (K * H)) * P;
    
    % Cache data
    xEst = [xEst, x];
end

%% Visualization

figure(1)
subplot(221)
plot(time, xEst(1,1:m-1),'k','linewidth',1.5); hold on
plot(t(N_MHE+1:end),X_estimate(:,1),'r','linewidth',1.5); hold on
plot(time, x_pos(1:m-1), 'g', 'linewidth', 1.5);hold on
xlabel("Time[Sec]")
ylabel("X[m]")
legend("KF", "MHE", "GT")

subplot(222)
plot(time, xEst(2,1:m-1), 'k', 'linewidth',1.5); hold on
plot(t(N_MHE+1:end),X_estimate(:,2),'r','linewidth',1.5); hold on
plot(time, y_pos(1:m-1), 'g', 'linewidth', 1.5); hold on
xlabel("Time[Sec]")
ylabel("Y[m]")

Ex_kf = abs(xEst(1,:) - x_pos);
Ex_mhe = abs(X_estimate(:,1)' - x_pos(N_MHE+1:end-1));
subplot(223)
plot(time, Ex_kf(1:m-1), 'k', 'linewidth', 1.5); hold on;
plot(time(N_MHE+1:end), Ex_mhe, 'r', 'linewidth', 1.5); hold on;

xlabel("Time[Sec]")
ylabel("Ex")

Ey_kf = abs(xEst(2,:) - y_pos);
Ey_mhe = abs(X_estimate(:,2)' - y_pos(N_MHE+1:end-1));
subplot(224)
plot(time, Ey_kf(1:m-1), 'k', 'linewidth', 1.5); hold on;
plot(time(N_MHE+1:end), Ey_mhe, 'r', 'linewidth', 1.5); hold on;

xlabel("Time[Sec]")
ylabel("Ey")

suptitle("MHE-KF Estimation Comparison")


% Kinematic Kalman Filter for position and acceleration measurements

meas = load("meas.mat");

x_pos = meas.xx(1,:);
y_pos = meas.xx(2,:);

sp = 1.0; % Sigma for position
m = length(x_pos); % Number of measurements

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
meas = [mpx; mpy; max; may];

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
H = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0];

ra = 10.0^2;   % Noise of Acceleration Measurement
rp = 100.0^2;  % Noise of Position Measurement
R = [rp, 0.0, 0.0, 0.0;
    0.0, rp, 0.0, 0.0;
    0.0, 0.0, ra, 0.0;
    0.0, 0.0, 0.0, ra];

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

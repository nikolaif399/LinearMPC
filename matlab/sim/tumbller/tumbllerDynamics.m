function dqdt = tumbllerDynamics(t,q,u,params)
%tumbllerDynamics Tumbller dynamics function

% Simple linear model
% u is applied force 
dqdt = params.A * q + params.B * u;

end


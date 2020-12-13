function dqdt = ballbotDynamics(t,q,u,params)
%ballbotDynamics Ballbot dynamics function, includes low level balancing
%controller

% q = x, y, z, dx, dy, dz, roll, pitch

% u = roll, pitch, thrust  (command, angles in world frame);


% Linearized dynamics
Ad = [0, 0, 1, 0;...
      0, 0, 0, 1;...
      0, -171.8039, 0, 0;...
      0, 24.3626, 0,0];
  
Bd = [0;
      0;
      5.0686;
      -0.4913];
  
Klqr = [-1.0000 -173.1954   -2.0268  -48.6683];

error = u - q;
u = Klqr*error;

dqdt = Ad*q + Bd*u;

end


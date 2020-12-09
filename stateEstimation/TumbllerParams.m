% Set plotting parameters
set(0, 'DefaultAxesFontSize', 24);
set(0, 'DefaultLineMarkerSize', 10);
set(0,'defaultfigurecolor',[1 1 1]);
set(0,'DefaultAxesXGrid','off','DefaultAxesYGrid','off')
set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

mcart = 0.493;
mpend = 0.312;
Ipend = 0.00024;
l = 0.04;
f = 0.01;
kt = 0.11;
R = 10;
r = 0.0335;
g = 9.81; 

A = [0, 1, 0, 0; 0, (-(Ipend + mpend*l^2)*f)/(Ipend*(mcart + mpend) + mcart*mpend*l^2), ...
    (mpend^2*g*l^2)/(Ipend*(mcart + mpend) + mcart*mpend*l^2), 0; 0, 0, 0, 1; ...
    0, (-mpend*f*l)/(Ipend*(mcart+mpend)+mcart*mpend*l^2), ...
    (mpend*g*l*(mcart+mpend))/(Ipend*(mcart+mpend)+mcart*mpend*l^2) ,0];

B = [0; (Ipend + mpend*l^2)/(Ipend*(mcart+mpend) + mcart*mpend*l^2); 0; ...
    (mpend * l)/(Ipend*(mcart + mpend)+mcart*mpend*l^2)] .* ((R*r)/(2*kt));

C = eye(4);

D = zeros(1,1);

sys_cont = ss(A,B,C,D);
sys_disc = c2d(sys_cont, 0.005);

Ad = sys_disc.A;
Bd = sys_disc.B;
Cd = sys_disc.C;

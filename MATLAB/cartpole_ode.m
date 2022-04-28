function dy = cartpole_ode(y,m,M,L,g,b,u)

Sy = sin(y(3));
Cy = cos(y(3));
% NOTE: could not solve in reasonable time with these denominators
%  i think they found a common denominator "D" by cross multiplying
%  please check.
G = (M+m) - m*Cy;
F = (M+m)*L + m*L*Cy^2;

% [x xdot th thdot]
dy(1,1) = y(2);
dy(2,1) = (u - b*y(2) - m*g*Sy + m*L*y(4)^2*Sy) / G;
dy(3,1) = y(4);
dy(4,1) = ((u - b*y(2))*Cy - (M+m)*g*Sy + m*L*y(4)^2*Sy*Cy) / F;

% function dy = cartpole_ode(y,m,M,L,g,d,u)
% 
% Sy = sin(y(3));
% Cy = cos(y(3));
% D = m*L*L*(M+m*(1-Cy^2));
% 
% dy(1,1) = y(2);
% dy(2,1) = (1/D)*(-m^2*L^2*g*Cy*Sy + m*L^2*(m*L*y(4)^2*Sy - d*y(2))) + m*L*L*(1/D)*u;
% dy(3,1) = y(4);
% dy(4,1) = (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*y(4)^2*Sy - d*y(2))) - m*L*Cy*(1/D)*u +.01*randn;
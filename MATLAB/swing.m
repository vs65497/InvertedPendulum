function [x, xdot, xddot, theta, thetadot, thetaddot] = swing(x0, x0dot, theta0, theta0dot, dt, Ke, Kp, Kd, m, g, L)

E = (1/2)*m*L^2*theta0dot^2 - m*g*L*cos(theta0) - m*g*L;

xddot = Ke*m*L*theta0dot*cos(theta0)*E - Kp*x0 - Kd*x0dot;
xdot = xddot*dt + x0dot;
x = (xdot-x0dot)*dt + x0;
%x = mod(x,2*pi);

thetaddot = (1/L)*(g*sin(theta0) - xddot*cos(theta0));
thetadot = thetaddot*dt + theta0dot;
theta = (thetadot-theta0dot)*dt + theta0;
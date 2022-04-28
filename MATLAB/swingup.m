g = 9.8; % m/s^2, down
m = 123 / 1000; % kg
L = 18.6 / 100; % m

Ke = 0.15; % energy gain
Kp = 0.08; % cart position gain
Kd = 0.01; % cart velocity gain

x = 0;
xdot = 0;
theta = 0.1;
thetadot = 0;

hold on
counter = 0;
for dt = 0.00:0.00135:1.93
    if(abs(theta) > pi+0.01 || abs(theta) < pi-0.01)
        counter = dt;
        [x, xdot, xddot, theta, thetadot, thetaddot] = swing(x, xdot, theta, thetadot, dt, Ke, Kp, Kd, m, g, L);
        plot(theta, thetadot)
    end
end
hold off


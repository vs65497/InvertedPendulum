g = 1; % m/s^2, down
m = 1; % kg
M = 3; % kg
L = 1; % m
b = 0.3; % N/m/s (coeff. friction)

A = [0 1 0 0; 
   0 -b/M g*m/M 0; 
   0 0 0 1; 
   0 -b/(M*L) g*(M+m)/(M*L) 0];

B = [0; 1/M; 0; 1/(M*L)];

C = [1 0 0 0];

D = 0;

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'};

sys = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs)

% LQR
Q = [12 0 0 0;
    0 1 0 0;
    0 0 102 0;
    0 0 0 10];
R = .045;
K = lqr(sys,Q,R)

tspan = 0:.001:10;
y0 = [-3; 0; pi+.1; 0];

   % LQR works! 
    [t,y] = ode45(@(t,y)((A-B*K)*(y-[0; 0; pi; 0])),tspan,y0);
 
   % LQG worked!
   %[t,y] = ode45(@(t,y)((Akf-B*Klqg)*(y-[0; 0; pi; 0])),tspan,y0);
 
   % currently cannot solve for in time. look at file for notes
   %[t,y] = ode45(@(t,y)cartpole_ode(y,m,M,L,g,b,-K*(y-[1; 0; pi; 0])),tspan,y0);
    
v = VideoWriter('cartpole.avi');
open(v);

for k=1:100:length(t)
    drawcartpole(y(k,:),m,M,L);
    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);
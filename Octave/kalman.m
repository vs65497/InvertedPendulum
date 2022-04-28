pkg load symbolic
pkg load control

M = 0.308;
m = 0.123;
L = 0.186;
g = 9.81;
b = 0.3;

A = [0 1 0 0; 0 -b/M g*m/M 0; 0 0 0 1; 0 -b/M/L g*(M+m)/M/L 0];
B = [0; 1/M; 0; 1/M/L];
C = [1 0 1 0];
D = [ 0 ];

sys = ss(A, B);

rank(ctrb(A,B)); % returns 4 -> controllable
rank(obsv(A,C)); % returns 4 -> observable

Vd = [0.5 0 0 0; 0 0.5 0 0; 0 0 5 0; 0 0 0 50];
Vn = 0.1;

Kf = (lqr(A',C',Vd,Vn))';
akfc = A - Kf*C;

sysKF = ss(A-Kf*C,[B Kf]);

printf("akfc.rows = 4;\n");
printf("akfc.cols = 4;\n\n");

c = 0;
for j = 1:4
        printf("akfc.elements[%d][0] = %f;\n", c, akfc(j,1));
        printf("akfc.elements[%d][1] = %f;\n", c, akfc(j,2));
        printf("akfc.elements[%d][2] = %f;\n", c, akfc(j,3));
        printf("akfc.elements[%d][3] = %f;\n\n", c, akfc(j,4));
        c += 1;
endfor

printf("kf.rows = 4;\n");
printf("kf.cols = 1;\n\n");

c = 0;
for j = 1:4
        printf("kf.elements[%d][0] = %f;\n", c, Kf(j));
        c += 1;
endfor
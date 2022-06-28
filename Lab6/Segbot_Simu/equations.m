g = 9.81;
m = 0.0615;
R = 0.067;
J_w = (m*R^2)/2;
M = 0.5764;
L = 0.049;
J_psi = (M*L^2)/3;
J_m = 2.285e-4;
R_m = 1.875;
K_b = 0.30646;
K_t = 0.30646;
n = 1;
f_m = 0.0022;
alpha = (n*K_t)/R_m;
beta = ((n*K_t*K_b)/R_m) + f_m;

D = [(M*L^2 + J_psi + 2*n^2*J_m) (M*L*R - 2*n^2*J_m);
    (M*L*R - 2*n^2*J_m) ((2*m+M)*R^2 + 2*J_w + 2*n^2*J_m)];
b = [M*g*L -2*beta 0 2*beta;
    0 2*beta 0 -2*beta];
h = [-2*alpha;
    2*alpha];
Dib = inv(D)*b;
Dih = inv(D)*h;

A4 = [0 1 0 0;
    Dib(1,:);
    0 0 0 1;
    Dib(2,:)];
B4 = [0;
    Dih(1);
    0;
    Dih(2)];
A = [A4(1,1) A4(1,2) A4(1,4);
    A4(2,1) A4(2,2) A4(2,4);
    A4(4,1) A4(4,2) A4(4,4)]
B = [B4(1);
    B4(2);
    B4(4)]
K = [-30,-2.8,-1]*6/10  % 6/10 because Simulation max is 6V and real system max is 10 for pwm output

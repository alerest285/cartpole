%% design_gains.m  -  gains for the course's own Simulink templates
% Same model as the Python stack (cartpole/dynamics.py, the PDF's equations L1/L2).
% Uses cp_config (shared Python export). Requires Control System Toolbox.
% Paste the printed K into state_feedback_controller.slx
% (u = -K*[x; alpha; x_dot; alpha_dot]) and into swingup_*.slx as the catcher.
%
% Signs: alpha = 0 upright, counter-clockwise positive; +V moves the cart to +x.
% The position entry of K comes out NEGATIVE: positive feedback on x, the cart
% has to move away before it can come back. Same for the PID position loop.

addpath(fileparts(mfilename('fullpath')));
cp_cfg = cp_config(); p = cp_cfg.plant;
m = p.m_p; l = p.l; J = p.J;
a = p.k_amp*p.a; M = p.M; g = p.g;
c_v = p.b+p.B_f; B_p = p.B_p;
D0 = M*J - (m*l)^2;

A = [0 0 1 0;
     0 0 0 1;
     0 m^2*l^2*g/D0     -J*c_v/D0      -m*l*B_p/D0;
     0 M*m*g*l/D0       -m*l*c_v/D0    -M*B_p/D0];
B = [0; 0; J*a/D0; m*l*a/D0];
C = [1 0 0 0; 0 1 0 0]; D = [0; 0];

fprintf('open-loop poles: %s\n', mat2str(eig(A)', 4));
fprintf('rank ctrb = %d, rank obsv = %d\n', rank(ctrb(A,B)), rank(obsv(A,C)));

% --- pole placement (handout hint: real, close together, in [-10, 0))
p_place = [-5 -6 -7 -8];
K_place = place(A, B, p_place);
fprintf('K_place = %s\n', mat2str(K_place, 4));

% --- LQR (the Python default: Q = diag(10, 40, 0, 0), R = 0.1)
Q = diag([10 40 0 0]); R = 0.1;
K_lqr = lqr(A, B, Q, R);
fprintf('K_lqr   = %s   closed loop: %s\n', mat2str(K_lqr, 4), mat2str(eig(A - B*K_lqr)', 3));

% --- discrete version (what actually runs at 500 Hz)
sysd = c2d(ss(A, B, C, D), 0.002);
K_dlqr = dlqr(sysd.A, sysd.B, Q*0.002, R*0.002);
fprintf('K_dlqr  = %s\n', mat2str(K_dlqr, 4));

% --- PID equivalent of the LQR. u = -K s = -K1 x - K2 a - K3 x_dot - K4 a_dot,
%     and with e = r - y (r = 0): e_x = -x, e_a = -a, de/dt = -velocity. So
%     Kp_x = K(1), Kd_x = K(3) (both negative), Kp_a = K(2), Kd_a = K(4).
%     These are the Python PID defaults (cartpole/controllers/classic.py).
fprintf('PID from LQR: angle Kp %.1f Kd %.2f | position Kp %.1f Kd %.1f (both position gains negative)\n', ...
        K_lqr(2), K_lqr(4), K_lqr(1), K_lqr(3));

%% --- Section 0: Local System Parameters ---
% Do not change anything in this section!

% Safety Watchdog Parameters
X_MAX = 0.2;
X_MIN = -X_MAX;
X_LIM_ENABLE = 1;
V_MAX = 10;

% Actuator Saturation Parameters
VMAX_AMP = 10;
VMAX_DAC = 10;

% Motor Gear Parameters
K_g = 3.71;
K_m = 0.00767;
K_t = K_m;
J_m = 3.9e-7;
r_mp = 6.35e-3;
R_m = 2.6;

% Grav. acceleration
g = 9.81;
% Pendulum mass
M_p = 0.127;
% Distance from pivot to center of mass
r_p = 0.1778;
% Cart mass
M_c = 0.57;

%% Following is only relevant for the regression task
% Measure each 'decimation' time step of the simulation
decimation = 1;
% Sampling rate in milliseconds
h = 0.002 * decimation;
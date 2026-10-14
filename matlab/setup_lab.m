% Workspace for course QUARC models; does not load, build, start or arm them.
% Course model state order MUST be [x; alpha; x_dot; alpha_dot], u=-K*s.
addpath(fileparts(mfilename('fullpath')));
cp_cfg=cp_config();
K_dlqr=cp_cfg.gains.lqr;
K_lqr=cp_cfg.gains.continuous;
K_place_discrete=cp_cfg.gains.place;
K_catch_energy=cp_cfg.gains.energy;
K_catch_sine=cp_cfg.gains.sine;
Ts=cp_cfg.h;
A=cp_cfg.A; B=cp_cfg.B; C=[1 0 0 0;0 1 0 0]; D=zeros(2,1);
fprintf('Imported %s, policy %s. Ts = %.3f s.\n',cp_cfg.generated_utc,cp_cfg.policy.version,Ts);
fprintf('Use u=-K*s. K_dlqr = %s\n',mat2str(K_dlqr,8));
fprintf('These gains were tested with the Kalman estimator; course derivative filters require revalidation.\n');
fprintf('No hardware opened. Keep the course watchdog and saturation blocks intact.\n');

function report = validate_cartpole()
% Compare MATLAB against exported Python physics, safety and controller vectors.
here=fileparts(mfilename('fullpath')); c=cp_config();
data=readmatrix(fullfile(here,'generated','model_vectors.csv'));
errors=zeros(size(data,1),3);
for k=1:size(data,1)
    s=data(k,1:4)'; v=data(k,5);
    errors(k,1)=max(abs(cp_deriv(s,v,c.plant)-data(k,6:9)'));
    errors(k,2)=max(abs(cp_rk4(s,v,c.plant,c.h)-data(k,10:13)'));
    errors(k,3)=abs(cp_safe(s,v,c)-data(k,14));
end
assert(max(errors(:))<1e-9,'cartpole:parity','Physics/safety parity failed');
report=struct('max_model_error',max(errors(:)));
data=readmatrix(fullfile(here,'generated','estimator_vectors.csv')); z=[];
est_errors=zeros(size(data,1),1);
for k=1:size(data,1)
    [hat,z]=cp_estimate(data(k,1:4)',c,z);
    est_errors(k)=max(abs(hat-data(k,5:8)'));
end
report.max_estimator_error=max(est_errors);
assert(max(est_errors)<1e-8,'cartpole:estimator','Estimator parity failed');
modes={'lqr','place','pid','energy','sine','ppo'};
for j=1:numel(modes)
    mode=modes{j}; z=[];
    d=readmatrix(fullfile(here,'generated',['controller_' mode '.csv']));
    e=zeros(size(d,1),1);
    for k=1:size(d,1)
        [v,z]=cp_control(d(k,2:5)',d(k,1),mode,c,z); e(k)=abs(v-d(k,6));
    end
    report.(mode)=max(e);
    assert(max(e)<1e-8,'cartpole:parity','Controller parity failed: %s',mode);
end
disp(report); fprintf('Python/MATLAB reference parity passed.\n');
end

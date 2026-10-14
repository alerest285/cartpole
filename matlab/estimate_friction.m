function report = estimate_friction(source, make_plot)
% Step 2: scalar B_f least squares, then compare a Coulomb extension.
% estimate_friction reads regression.slx workspace signals.
% estimate_friction('recording.csv') accepts t,x,alpha,v in SI units.
% Holds out the final 35% with a filter guard. Never changes the controller.
if nargin<1, d=read_course_data(); else, d=read_course_data(source); end
if nargin<2, make_plot=true; end
c=cp_config(); p=c.plant; t=d.t(:); h=median(diff(t)); n=numel(t);
assert(h>0&&h<=0.01&&max(abs(diff(t)-h))<0.02*h, ...
    'cartpole:time','Need uniform timestamps at >=100 Hz');
sigma=max(1,0.012/h); half=floor(4*sigma); k=(-half:half)';
kernel=exp(-0.5*(k/sigma).^2); kernel=kernel/sum(kernel);
x=conv(d.x,kernel,'same'); a=conv(unwrap(d.alpha),kernel,'same');
xd=gradient(x,h); xdd=gradient(xd,h); ad=gradient(a,h); add=gradient(ad,h);
if ismember('v',d.Properties.VariableNames)
    voltage=interp1((1:n)',d.v,max(1,(1:n)'-0.5),'linear');
    force=p.a*p.k_amp*conv(voltage,kernel,'same')-p.b*xd;
else
    assert(ismember('force',d.Properties.VariableNames),'cartpole:force','Need v or force column');
    force=conv(d.force,kernel,'same');
end
y=force-p.M*xdd+p.ml*(cos(a).*add-sin(a).*ad.^2);
idx=(1:n)'; guard=ceil(4*sigma)+2;
ok=idx>guard & idx<n-guard & abs(d.x)<0.17 & abs(xd)>0.015;
cut=floor(0.65*n); train=ok & idx<cut-guard; valid=ok & idx>cut+guard;
assert(nnz(train)>=300&&nnz(valid)>=150,'cartpole:excitation', ...
    'Record 20-30 seconds with varied speed in both directions');
assert(nnz(xd(train)>0.015)>40&&nnz(xd(train)<-0.015)>40&& ...
    nnz(xd(valid)>0.015)>40&&nnz(xd(valid)<-0.015)>40, ...
    'cartpole:directions','Both estimation and validation need motion in both directions');
viscous=fit_model(xd,y,train,valid); both=fit_model([xd tanh(xd/0.01)],y,train,valid);
selected='viscous';
if both.accepted && (~viscous.accepted || both.rmse_N<0.85*viscous.rmse_N), selected='viscous_coulomb'; end
report=struct('exercise_dimension',1,'viscous',viscous,'viscous_coulomb',both, ...
    'selected',selected,'n_train',nnz(train),'n_validation',nnz(valid));
fprintf('d = 1 for handout regression. B_f = %.4f N s/m; held-out R2 = %.3f\n',viscous.B_f,viscous.r2);
fprintf('Coulomb extension: B_f %.4f N s/m, F_coul %.4f N; held-out R2 %.3f\n',both.B_f,both.F_coul,both.r2);
fprintf('Selected: %s. Accepted: %d. No model parameters changed.\n',selected,report.(selected).accepted);
out=fullfile(fileparts(mfilename('fullpath')),'results'); if ~isfolder(out), mkdir(out); end
stamp=char(datetime('now','Format','yyyyMMdd-HHmmss-SSS'));
save(fullfile(out,['friction-' stamp '.mat']),'report');
writetable(d,fullfile(out,['friction-' stamp '.csv']));
if make_plot
    figure('Name','Step 2: friction identification','Color','w');
    subplot(2,1,1); plot(xd(train),y(train),'.',xd(train),viscous.B_f*xd(train),'.');
    xlabel('Cart velocity [m/s]'); ylabel('Friction residual [N]'); grid on;
    legend('Estimation samples','B_f x dot');
    subplot(2,1,2); plot(t(valid),[y(valid),viscous.B_f*xd(valid), ...
        both.B_f*xd(valid)+both.F_coul*tanh(xd(valid)/0.01)]);
    xlabel('Time [s]'); ylabel('Held-out force [N]'); grid on;
    legend('Measured residual','Viscous model','Viscous + Coulomb');
end
end

function r=fit_model(X,y,train,valid)
theta=X(train,:)\y(train); residual=y(valid)-X(valid,:)*theta;
r2=1-sum(residual.^2)/max(1e-12,sum((y(valid)-mean(y(valid))).^2));
condition=cond(X(train,:)./sqrt(mean(X(train,:).^2,1)));
fc=0; if numel(theta)>1, fc=theta(2); end
r=struct('B_f',theta(1),'F_coul',fc,'rmse_N',sqrt(mean(residual.^2)), ...
    'r2',r2,'scaled_condition',condition,'accepted', ...
    all(isfinite(theta))&&theta(1)>=0&&theta(1)<=30&&fc>=0&&fc<=3&&r2>=0.6&&condition<100);
end

function result = course_modeling(recording)
% Handout Steps 1 and 4: 5 V/10 Hz model, linearization, poles and Q sweep.
% Optionally overlay a normalized actual recording. No hardware is opened.
c=cp_config(); p=c.plant; h=c.h; duration=5; n=round(duration/h);
states=zeros(4,2); states(2,:)=-pi; rows=zeros(n,7);
ideal=p; ideal.K_m=0; ideal.rotor_mass=false; ideal.B_p=0; ideal.F_coul=0;
ideal=cp_derive(ideal); % ideal voltage-to-force gain, no back-EMF or rotor inertia
for k=1:n
    t=(k-1)*h; voltage=5*sin(2*pi*10*t);
    rows(k,:)=[t states(1,1) states(2,1) states(1,2) states(2,2) voltage 0];
    states(:,1)=cp_rk4(cp_rk4(states(:,1),voltage,p,h/2),voltage,p,h/2);
    states(:,2)=cp_rk4(cp_rk4(states(:,2),voltage,ideal,h/2),voltage,ideal,h/2);
    if any(abs(states(1,:))>0.19), rows=rows(1:k,:); break; end
end
A=c.A; B=c.B; C=[1 0 0 0;0 1 0 0]; D=zeros(2,1);
Ctrb=[B A*B A^2*B A^3*B]; Obsv=[C;C*A;C*A^2;C*A^3];
result=struct('data',rows,'A',A,'B',B,'C',C,'D',D, ...
    'controllability_rank',rank(Ctrb),'observability_rank',rank(Obsv), ...
    'open_loop_poles',eig(A),'K_lqr',c.gains.continuous,'K_place',c.gains.place);
assert(rank(Ctrb)==4&&rank(Obsv)==4,'cartpole:rank','Model must be controllable and observable');
figure('Name','Step 1: 5 V, 10 Hz comparison','Color','w');
subplot(2,1,1); plot(rows(:,1),100*rows(:,[2 4])); ylabel('Cart [cm]'); grid on;
legend('Full motor model','Ideal motor force');
subplot(2,1,2); plot(rows(:,1),180/pi*rows(:,[3 5])); ylabel('Angle [deg]'); xlabel('Time [s]'); grid on;
if nargin>0
    d=read_course_data(recording);
    subplot(2,1,1); hold on; plot(d.t-d.t(1),100*d.x,'k'); legend('Full motor','Ideal motor','Recorded rig');
    subplot(2,1,2); hold on; plot(d.t-d.t(1),unwrap(d.alpha)*180/pi,'k');
end
% Independent continuous LQR and Q-weight experiment when toolbox is available.
if exist('lqr','file')==2
    weights=[10 40 0 0;40 40 0 0;10 160 0 0;10 40 1 1];
    result.q_sweep=cell(size(weights,1),1);
    for j=1:size(weights,1)
        K=lqr(A,B,diag(weights(j,:)),0.1);
        result.q_sweep{j}=struct('Q',weights(j,:),'R',0.1,'K',K,'poles',eig(A-B*K));
    end
    result.K_place_continuous=place(A,B,[-5 -6 -7 -8]);
end
fprintf('Controllability/observability ranks: %d/%d. Physical comparison pending until a recording is supplied.\n',rank(Ctrb),rank(Obsv));
out=fullfile(fileparts(mfilename('fullpath')),'results'); if ~isfolder(out), mkdir(out); end
save(fullfile(out,'course_modeling.mat'),'result');
end

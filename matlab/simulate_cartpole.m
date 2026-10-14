function result = simulate_cartpole(mode,duration,c,s0)
% SIMULATE_CARTPOLE('energy',12) runs a quantised 500 Hz nonlinear simulation.
% Methods: lqr, place, pid, energy, sine, ppo. No toolbox or hardware required.
if nargin<1, mode='energy'; end
if nargin<2, duration=12; end
if nargin<3, c=cp_config(); end
if nargin<4
    s0=[0;-pi;0;0];
    if any(strcmp(mode,{'lqr','place','pid'})), s0(2)=0.05; end
end
n=round(duration/c.h); rows=zeros(n,11); s=s0(:); z=[]; fault='';
for k=1:n
    t=(k-1)*c.h;
    [next,z,v,wanted,hat,fault]=cp_tick(s,t,mode,c,z);
    rows(k,:)=[t s' hat' wanted v];
    if ~isempty(fault), rows=rows(1:k,:); break; end
    s=next;
end
angle=pi-mod(pi-rows(:,3),2*pi);
tail=rows(:,1)>=rows(end,1)-2;
result=struct('mode',mode,'data',rows,'fault',fault,'config',c, ...
    'columns',{{'t','x','alpha','xd','ad','x_hat','alpha_hat','xd_hat','ad_hat','v_des','v'}}, ...
    'balanced',isempty(fault)&&all(abs(angle(tail))<5*pi/180), ...
    'max_abs_x_cm',100*max(abs(rows(:,2))));
end

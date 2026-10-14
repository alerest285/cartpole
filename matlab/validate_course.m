function validate_course()
% Offline checks to run on the licensed lab PC before opening any hardware model.
validate_cartpole;
c=cp_config(); p=c.plant; t=(0:.002:19.998)';
x=.04*sin(2.4*t)+.015*sin(6.8*t); xd=.096*cos(2.4*t)+.102*cos(6.8*t);
xdd=-.2304*sin(2.4*t)-.6936*sin(6.8*t);
alpha=.3*sin(3.3*t); ad=.99*cos(3.3*t); add=-3.267*sin(3.3*t);
force=p.M*xdd-p.ml*cos(alpha).*add+p.ml*sin(alpha).*ad.^2+3.3*xd+.25*tanh(xd/.01);
report=estimate_friction(table(t,x,alpha,force),false);
assert(report.viscous_coulomb.accepted);
assert(abs(report.viscous_coulomb.B_f-3.3)<.15&&abs(report.viscous_coulomb.F_coul-.25)<.02);
% Exercise the actual fixed-size function used inside generated QUARC blocks.
physical=[p.M p.J p.ml p.g p.a p.b p.B_f p.B_p p.F_coul p.k_amp];
settings=[35 45 5 5 2 2 3 2 0 0 1 1.05 -40 0 0];
for mode=[1 2 3 6 8]
    clear cp_lab_controller;
    K=c.gains.lqr; state=[0;.05;0;0];
    if mode==2, K=c.gains.place; end
    if mode==3, K=[-10 42 -16 5.5]; end
    if mode==6, K=c.gains.energy; state(2)=-pi; end
    if mode==8, K=c.gains.sine; state(2)=-pi; settings(11)=8; end
    tail=zeros(1000,1);
    for k=1:6000
        y=[round(state(1)/abs(c.hardware.cart_m_per_count))*abs(c.hardware.cart_m_per_count); ...
            pi-mod(pi-(round((state(2)+pi)/c.hardware.pole_rad_per_count)*c.hardware.pole_rad_per_count-pi),2*pi)];
        [v,~,phase]=cp_lab_controller(y,0,1,0,10,mode,physical,K,settings);
        assert(abs(v)<=10 && phase>=0,'cartpole:labcore','Lab controller faulted in simulation');
        state=cp_rk4(cp_rk4(state,v,p,c.h/2),v,p,c.h/2);
        assert(abs(state(1))<.19);
        if k>5000, tail(k-5000)=abs(pi-mod(pi-state(2),2*pi)); end
    end
    assert(all(tail<5*pi/180),'cartpole:labcore','Mode %d failed final balance',mode);
end
clear cp_lab_controller;
[v,~,~]=cp_lab_controller([0;0],0,0,0,4,1,physical,c.gains.lqr,settings); assert(v==0);
for x=[-.09 0 .09]
    [v,info]=course_barrier([x;0],10,1,4);
    assert(abs(v)<=4&&info.feasible);
end
fprintf('Course friction, controller, Arm=0 and barrier checks passed. No hardware opened.\n');
end

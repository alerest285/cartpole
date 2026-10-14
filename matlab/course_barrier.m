function [u,info] = course_barrier(state,desired,b,cap)
% Step 6: nearest feasible voltage for xdd=b*u, |x|<=0.10 m.
% Relative degree TWO: impose hdd+2*w*hd+w^2*h >= 0 on both sides.
% Forward invariance also needs initially compatible velocity and feasibility.
if nargin<3, b=1; end
if nargin<4, cap=4; end
assert(b>0&&cap>0&&all(isfinite([state(:);desired;b;cap])));
x=state(1); xd=state(2); w=12;
lo=max(-cap,(-w*w*(0.10+x)-2*w*xd)/b);
hi=min(cap,( w*w*(0.10-x)-2*w*xd)/b);
feasible=lo<=hi;
if feasible && exist('quadprog','file')==2
    u=quadprog(1,-desired,[],[],[],[],lo,hi,[],optimoptions('quadprog','Display','off'));
elseif feasible
    u=max(lo,min(hi,desired)); % exact scalar-QP solution, no toolbox
else
    u=max(-cap,min(cap,(lo+hi)/2));
end
info=struct('lower',lo,'upper',hi,'feasible',feasible, ...
    'initial_velocity_compatible',-w*(0.10+x)<=xd&&xd<=w*(0.10-x));
end

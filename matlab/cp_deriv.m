function ds = cp_deriv(s,v,p)
% State [x; alpha; x_dot; alpha_dot], alpha=0 upright, CCW positive.
s = s(:); a=s(2); xd=s(3); ad=s(4);
F=p.k_amp*p.a*v-(p.b+p.B_f)*xd-p.F_coul*tanh(xd/0.01);
r1=F-p.ml*sin(a)*ad^2; r2=p.ml*p.g*sin(a)-p.B_p*ad;
D=p.M*p.J-(p.ml*cos(a))^2;
ds=[xd;ad;(p.J*r1+p.ml*cos(a)*r2)/D; ...
    (p.ml*cos(a)*r1+p.M*r2)/D];
end

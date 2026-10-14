function s = cp_rk4(s,v,p,h)
s=s(:);
k1=cp_deriv(s,v,p); k2=cp_deriv(s+h*k1/2,v,p);
k3=cp_deriv(s+h*k2/2,v,p); k4=cp_deriv(s+h*k3,v,p);
s=s+h*(k1+2*k2+2*k3+k4)/6;
end

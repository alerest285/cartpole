function [v,fault] = cp_safe(s,wanted,c)
fault='';
if any(~isfinite(s))
    fault='non-finite state';
elseif abs(s(1))>c.x_hard
    fault='cart hard limit';
elseif abs(s(4))>60
    fault='pole speed limit';
end
if ~isempty(fault), v=0; return; end
a0=cp_deriv(s,0,c.plant); a1=cp_deriv(s,1,c.plant);
beta=a1(3)-a0(3);
lo=max(-c.v_cap,(-c.wn^2*(c.x_soft+s(1))-2*c.wn*s(3)-a0(3))/beta);
hi=min(c.v_cap,(c.wn^2*(c.x_soft-s(1))-2*c.wn*s(3)-a0(3))/beta);
if lo>hi, lo=max(-c.v_cap,min(c.v_cap,(lo+hi)/2)); hi=lo; end
v=min(max(wanted,lo),hi);
if ~isfinite(wanted), v=0; end
end

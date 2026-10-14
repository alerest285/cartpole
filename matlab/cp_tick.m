function [next,z,v,wanted,estimate,fault] = cp_tick(s,t,mode,c,z)
if isempty(z), z=struct('est',[],'ctrl',[],'delay',zeros(c.truth.delay_steps,1),'fault',''); end
[estimate,z.est]=cp_estimate(s,c,z.est);
[wanted,z.ctrl]=cp_control(estimate,t,mode,c,z.ctrl);
[v,fault]=cp_safe(estimate,wanted,c);
if ~isempty(fault), z.fault=fault; end
if ~isempty(z.fault), v=0; fault=z.fault; end
applied=v;
if ~isempty(z.delay), applied=z.delay(1); z.delay=[z.delay(2:end);v]; end
applied=max(-c.hardware.v_limit,min(c.hardware.v_limit,applied));
next=cp_rk4(s,applied,c.truth,c.h/2);
next=cp_rk4(next,applied,c.truth,c.h/2);
end

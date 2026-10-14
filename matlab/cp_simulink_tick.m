function y = cp_simulink_tick(u)
% Normal-mode simulation adapter, called exactly once per 2 ms sample.
% Not a QUARC hardware block. The reusable controller is cp_control.m.
persistent z last_t cached c mode
t=u(5);
if isempty(last_t) || t<last_t
    c=evalin('base','cp_cfg'); mode=evalin('base','cp_mode'); z=[]; last_t=-inf;
end
if t~=last_t
    [next,z,v,wanted]=cp_tick(u(1:4),t,mode,c,z);
    cached=[next;v;wanted]; last_t=t;
end
y=cached;
end

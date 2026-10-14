function d = read_course_data(source, sample_period)
% Normalize the course regression workspace, SimulationOutput, MAT or CSV.
% d is a table with t [s], x [m], alpha [rad, upright=0], v [logical V].
% With no argument, read x, alpha, F_c, x_dot from the base workspace.
if nargin<2, sample_period=0.002; end
if nargin<1
    source=struct();
    names={'x','alpha','F_c','x_dot','t','tout','lab_state','lab_voltage'};
    for k=1:numel(names)
        if evalin('base',sprintf('exist(''%s'',''var'')',names{k}))
            source.(names{k})=evalin('base',names{k});
        end
    end
end
if ischar(source)||isstring(source)
    [~,~,ext]=fileparts(source);
    if strcmpi(ext,'.csv'), d=readtable(source); return; end
    source=load(source);
    if isfield(source,'out'), source=source.out; end
end
if istable(source), d=source; return; end
if isa(source,'Simulink.SimulationOutput')
    names=source.who; unpacked=struct();
    for k=1:numel(names), unpacked.(names{k})=source.get(names{k}); end
    source=unpacked;
end
if isfield(source,'lab_state')
    [state,t]=signal(source.lab_state,sample_period);
    assert(size(state,2)==4,'cartpole:columns','lab_state must contain [x alpha xd ad]');
    assert(isfield(source,'lab_voltage'),'cartpole:voltage', ...
        'Need lab_voltage AFTER the safety gate; lab_command is not applied voltage.');
    [v,tv]=signal(source.lab_voltage,sample_period); same_time(t,tv);
    d=table(t,state(:,1),state(:,2),v,'VariableNames',{'t','x','alpha','v'});
else
    assert(all(isfield(source,{'x','alpha','F_c','x_dot'})), ...
        'cartpole:recording','Need x, alpha, F_c and x_dot from regression.slx');
    [x,t]=signal(source.x,sample_period); [alpha,ta]=signal(source.alpha,sample_period);
    [force,tf]=signal(source.F_c,sample_period); [xd,td]=signal(source.x_dot,sample_period);
    same_time(t,ta); same_time(t,tf); same_time(t,td);
    if isfield(source,'t'), t=source.t(:);
    elseif isfield(source,'tout') && isnumeric(source.tout), t=source.tout(:); end
    % The original F_c block computes a*V - b*x_dot. Recover V before
    % applying one consistent offline derivative/filter to every signal.
    p=cp_config(); p=p.plant;
    v=(force+p.b*xd)/p.a;
    d=table(t,x,alpha,v,force,'VariableNames',{'t','x','alpha','v','force'});
end
assert(height(d)>=1000 && all(isfinite(d{:,:}),'all'), ...
    'cartpole:data','Need at least 1,000 finite aligned samples');
end

function [y,t]=signal(s,h)
if isa(s,'timeseries')
    y=squeeze(s.Data); t=s.Time(:);
elseif isstruct(s)&&isfield(s,'signals')
    y=squeeze(s.signals.values); t=s.time(:);
elseif isnumeric(s)
    y=s; if isvector(y), y=y(:); end
    t=(0:size(y,1)-1)'*h;
else
    error('cartpole:signal','Expected a timeseries, Structure With Time or numeric signal');
end
if size(y,1)~=numel(t)&&size(y,2)==numel(t), y=y'; end
assert(size(y,1)==numel(t),'cartpole:alignment','Signal length differs from time');
end

function same_time(a,b)
assert(numel(a)==numel(b)&&max(abs(a-b))<1e-6,'cartpole:alignment', ...
    'Signals have different timestamps. Export synchronized 500 Hz signals.');
end

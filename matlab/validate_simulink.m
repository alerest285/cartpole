function report = validate_simulink()
% Test the built Simulink model against the MATLAB loop, including timing.
model=build_cartpole_simulink('energy');
simout=sim(model,'StopTime','11.998');
reference=simulate_cartpole('energy',12);
truth=squeeze(simout.state_log.Data);
if size(truth,2)~=4, truth=truth'; end
v=squeeze(simout.voltage_log.Data);
assert(size(truth,1)==size(reference.data,1),'cartpole:length','Simulink sample count differs');
report.state_error=max(abs(truth-reference.data(:,2:5)),[],'all');
report.voltage_error=max(abs(v(:)-reference.data(:,11)));
assert(max([report.state_error report.voltage_error])<1e-7,'cartpole:simulink','Simulink parity failed');
disp(report);
end

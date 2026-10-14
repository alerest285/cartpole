function model = build_cartpole_simulink(mode,cfg)
% Build an ordinary Simulink model without any QUARC or hardware dependency.
% Run build_cartpole_simulink('energy'), then press Run. Requires Simulink.
if nargin<1, mode='energy'; end
here=fileparts(mfilename('fullpath')); addpath(here);
assert(license('test','Simulink'),'cartpole:license','Simulink license required');
if nargin<2, cfg=cp_config(); end
s0=[0;-pi;0;0];
if any(strcmp(mode,{'lqr','place','pid'})), s0(2)=0.05; end
assignin('base','cp_cfg',cfg); assignin('base','cp_mode',mode); assignin('base','cp_s0',s0);
model='cartpole_sim';
out=fullfile(here,'generated'); file=fullfile(out,[model '.slx']);
if bdIsLoaded(model), close_system(model,0); end
new_system(model); load_system('simulink');
set_param(model,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','0.002','StopTime','12','ReturnWorkspaceOutputs','on', ...
    'InitFcn','clear cp_simulink_tick');
add_block('simulink/Discrete/Unit Delay',[model '/Plant state'], ...
    'InitialCondition','cp_s0','SampleTime','0.002','Position',[80 100 135 140]);
add_block('simulink/Sources/Digital Clock',[model '/Sample time'], ...
    'SampleTime','0.002','Position',[70 200 135 230]);
add_block('simulink/Signal Routing/Mux',[model '/State and time'], ...
    'Inputs','2','Position',[190 110 195 210]);
add_block('simulink/User-Defined Functions/Interpreted MATLAB Fcn',[model '/Sampled control and plant'], ...
    'MATLABFcn','cp_simulink_tick(u)','OutputDimensions','6', ...
    'Position',[255 125 450 185]);
add_block('simulink/Signal Routing/Demux',[model '/Separate outputs'], ...
    'Outputs','[4 1 1]','Position',[500 110 505 210]);
add_block('simulink/Sinks/Scope',[model '/State scope'], ...
    'Position',[245 20 290 60]);
add_block('simulink/Sinks/Scope',[model '/Voltage scope'], ...
    'Position',[590 135 635 175]);
add_block('simulink/Sinks/Terminator',[model '/Requested voltage'], ...
    'Position',[595 195 615 215]);
add_block('simulink/Sinks/To Workspace',[model '/State log'], ...
    'VariableName','state_log','SaveFormat','Timeseries','Position',[360 20 450 60]);
add_block('simulink/Sinks/To Workspace',[model '/Voltage log'], ...
    'VariableName','voltage_log','SaveFormat','Timeseries','Position',[580 255 680 290]);
add_line(model,'Plant state/1','State and time/1','autorouting','on');
add_line(model,'Sample time/1','State and time/2','autorouting','on');
add_line(model,'State and time/1','Sampled control and plant/1','autorouting','on');
add_line(model,'Sampled control and plant/1','Separate outputs/1','autorouting','on');
add_line(model,'Separate outputs/1','Plant state/1','autorouting','on');
add_line(model,'Plant state/1','State scope/1','autorouting','on');
add_line(model,'Plant state/1','State log/1','autorouting','on');
add_line(model,'Separate outputs/2','Voltage scope/1','autorouting','on');
add_line(model,'Separate outputs/2','Voltage log/1','autorouting','on');
add_line(model,'Separate outputs/3','Requested voltage/1','autorouting','on');
annotation=Simulink.Annotation(model, ...
    'SIMULATION ONLY | 500 Hz | encoder -> Kalman -> controller -> limits -> RK4 plant');
annotation.Position=[60 320];
save_system(model,file); open_system(model);
fprintf('Built %s. Press Run; no hardware is opened.\n',file);
end

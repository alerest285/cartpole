function model = build_course_lqr(course_folder)
% Fill the course's state-feedback template on a lab PC with MATLAB + QUARC.
% Saves a COPY with Arm=0 and a 4 V cap. Never builds, downloads or runs it.
% This uses the course's derivative filters, not the Python Kalman estimator.
here=fileparts(mfilename('fullpath')); addpath(here);
if nargin<1, course_folder=fullfile(here,'..','course_original'); end
source=fullfile(course_folder,'state_feedback_controller.slx');
assert(isfile(source),'cartpole:template','Course template missing: %s',source);
assert(exist('quarc_library','file')~=0,'cartpole:quarc','Run this builder on the QUARC lab PC');
c=cp_config(); model='cartpole_lab_lqr';
output=fullfile(here,'generated',[model '.slx']);
if bdIsLoaded(model), close_system(model,0); end
copyfile(source,output); load_system(output);
% Rewire only the top-level template. Protected I/O and safety stay intact.
lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
for j=1:numel(lines)
    if ishandle(lines(j)), delete_line(lines(j)); end
end
add_block('simulink/Signal Routing/Mux',[model '/Measured state'], ...
    'Inputs','4','Position',[300 330 305 430]);
add_block('simulink/Signal Routing/Mux',[model '/Reference state'], ...
    'Inputs','[1 1 2]','Position',[380 160 385 270]);
add_block('simulink/Sources/Constant',[model '/Velocity reference'], ...
    'Value','[0;0]','Position',[300 300 335 325]);
add_block('simulink/Math Operations/Sum',[model '/State error'], ...
    'Inputs','+-','Position',[430 330 460 370]);
add_block('simulink/Math Operations/Gain',[model '/LQR voltage'], ...
    'Gain',mat2str(-c.gains.lqr,16),'Multiplication','Matrix(K*u)', ...
    'Position',[500 330 625 370]);
add_block('simulink/Discontinuities/Saturation',[model '/First session 4 V cap'], ...
    'UpperLimit','4','LowerLimit','-4','Position',[665 330 705 370]);
add_block('simulink/Sources/Constant',[model '/Arm'], ...
    'Value','0','Position',[660 425 700 455]);
add_block('simulink/Math Operations/Product',[model '/Armed command'], ...
    'Inputs','**','Position',[750 340 785 380]);
add_block('simulink/Sinks/To Workspace',[model '/Measured state log'], ...
    'VariableName','lab_state','SaveFormat','Timeseries','Position',[430 430 550 460]);
add_block('simulink/Sinks/To Workspace',[model '/Command log'], ...
    'VariableName','lab_command','SaveFormat','Timeseries','Position',[810 430 930 460]);
for j=1:4
    add_line(model,sprintf('Subsystem1/%d',j),sprintf('Measured state/%d',j),'autorouting','on');
end
add_line(model,'Subsystem1/1','Subsystem/2','autorouting','on');
add_line(model,'Subsystem1/2','Subsystem/3','autorouting','on');
add_line(model,'Position Reference [m]/1','Reference state/1','autorouting','on');
add_line(model,'Angle Reference [rad]/1','Reference state/2','autorouting','on');
add_line(model,'Velocity reference/1','Reference state/3','autorouting','on');
add_line(model,'Measured state/1','State error/1','autorouting','on');
add_line(model,'Reference state/1','State error/2','autorouting','on');
add_line(model,'State error/1','LQR voltage/1','autorouting','on');
add_line(model,'LQR voltage/1','First session 4 V cap/1','autorouting','on');
add_line(model,'First session 4 V cap/1','Armed command/1','autorouting','on');
add_line(model,'Arm/1','Armed command/2','autorouting','on');
add_line(model,'Armed command/1','Subsystem/1','autorouting','on');
add_line(model,'Measured state/1','Measured state log/1','autorouting','on');
add_line(model,'Armed command/1','Command log/1','autorouting','on');
set_param(model,'FixedStep','0.002');
save_system(model,output); open_system(model);
fprintf('Prepared %s, Arm=0, cap=4 V. NOT built or run.\n',output);
fprintf('lab_command is BEFORE the course angle gate, not a measurement of motor voltage.\n');
fprintf('Verify encoder/voltage signs and timing on the rig before setting Arm to 1.\n');
end

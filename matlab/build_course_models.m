function models = build_course_models(course_folder)
% Fill course copies for every classical control exercise. Does NOT run hardware.
% Requires Simulink/QUARC. Defaults: Arm=0, cap=4 V, excitation/sine amplitude=1 V.
here=fileparts(mfilename('fullpath')); addpath(here);
if nargin<1, course_folder=fullfile(here,'..','course_original'); end
assert(exist('quarc_library','file')~=0,'cartpole:quarc','Run on the QUARC lab PC');
c=cp_config(); p=c.plant;
physical=[p.M p.J p.ml p.g p.a p.b p.B_f p.B_p p.F_coul p.k_amp];
methods={'lqr','place','pid','mouse','sine','energy','identify','locked_sine'};
models=cell(size(methods));
folder=fullfile(here,'generated'); if ~isfolder(folder), mkdir(folder); end
for mode=1:numel(methods)
    method=methods{mode}; balanced=mode<=3;
    if mode==3
        template='PID.slx';
    elseif balanced
        template='state_feedback_controller.slx';
    else
        template='swingup_mouse.slx';
    end
    model=['cartpole_course_' method]; models{mode}=model;
    if bdIsLoaded(model), close_system(model,0); end
    output=fullfile(folder,[model '.slx']); copyfile(fullfile(course_folder,template),output);
    load_system(output);
    % Load the original protected constants into this model's own workspace.
    workspace=get_param(model,'ModelWorkspace');
    evalin(workspace,fileread(fullfile(course_folder,'system_setup.m')));
    lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
    for j=1:numel(lines), if ishandle(lines(j)), delete_line(lines(j)); end; end
    if balanced
        sensor='Subsystem1'; xpos=1; angle=2; io='Subsystem';
    else
        sensor=sprintf('SIP + IP02:\nActual Plant1'); xpos=1; angle=4; io=sensor;
        delete_block([model '/Abs']); delete_block([model '/Switch']);
        if mode~=4
            delete_block([model '/Host Mouse']); delete_block([model '/Host Initialize']);
            delete_block([model '/Normalizer']);
        end
    end
    add_block('simulink/Signal Routing/Mux',[model '/Encoders'], ...
        'Inputs','2','Position',[300 340 305 400]);
    add_line(model,sprintf('%s/%d',sensor,xpos),'Encoders/1','autorouting','on');
    add_line(model,sprintf('%s/%d',sensor,angle),'Encoders/2','autorouting','on');
    add_block('simulink/Discrete/Zero-Order Hold',[model '/Sample 500 Hz'], ...
        'SampleTime','0.002','Position',[340 350 400 390]);
    add_line(model,'Encoders/1','Sample 500 Hz/1','autorouting','on');
    add_block('simulink/User-Defined Functions/MATLAB Function',[model '/Controller'], ...
        'Position',[700 260 900 470]);
    root=sfroot; chart=root.find('-isa','Stateflow.EMChart','Path',[model '/Controller']);
    assert(isscalar(chart),'cartpole:chart','Could not locate MATLAB Function block');
    chart.Script=fileread(fullfile(here,'cp_lab_controller.m'));
    add_line(model,'Sample 500 Hz/1','Controller/1','autorouting','on');
    if mode==4
        add_line(model,'Host Mouse/4','Normalizer/1','autorouting','on');
        add_line(model,'Normalizer/1','Controller/2','autorouting','on');
    else
        constant(model,'Manual voltage','0',2,80);
    end
    constant(model,'Arm','0',3,130);
    if balanced
        add_line(model,'Position Reference [m]/1','Controller/4','autorouting','on');
    else
        constant(model,'Position reference','0',4,180);
    end
    constant(model,'Voltage cap','4',5,230);
    constant(model,'Method',num2str(mode),6,280);
    constant(model,'Physical model',mat2str(physical,16),7,330);
    K=c.gains.lqr;
    if mode==2
        K=c.gains.place;
    elseif mode==3
        K=[-10 42 -16 5.5];
    elseif mode==5||mode==8
        K=c.gains.sine;
    elseif mode>=4
        K=c.gains.energy;
    end
    constant(model,'Feedback gain',mat2str(K,16),8,380);
    settings=[35 45 5 5 2 2 3 2 0 0 1 1.05 -40 0 0];
    constant(model,'Tuning',mat2str(settings,16),9,430);
    add_line(model,'Controller/1',[io '/1'],'autorouting','on');
    if balanced
        add_line(model,[sensor '/1'],[io '/2'],'autorouting','on');
        add_line(model,[sensor '/2'],[io '/3'],'autorouting','on');
    end
    add_block('simulink/Sinks/To Workspace',[model '/State log'], ...
        'VariableName','lab_state','SaveFormat','Timeseries','Position',[960 300 1080 330]);
    add_line(model,'Controller/2','State log/1','autorouting','on');
    add_block('simulink/Sinks/To Workspace',[model '/Phase log'], ...
        'VariableName','lab_phase','SaveFormat','Timeseries','Position',[960 370 1080 400]);
    add_line(model,'Controller/3','Phase log/1','autorouting','on');
    % Only add a logging branch: keep every original I/O/safety connection intact.
    io_path=[model '/' io]; tap=find_system(io_path,'SearchDepth',1,'BlockType','Saturate');
    assert(isscalar(tap),'cartpole:log','Expected one final amplifier limiter');
    add_block('simulink/Sinks/To Workspace',[io_path '/Applied voltage log'], ...
        'VariableName','lab_voltage','SaveFormat','Timeseries','Position',[850 350 970 380]);
    add_line(io_path,[get_param(tap{1},'Name') '/1'],'Applied voltage log/1','autorouting','on');
    set_param(model,'FixedStep','0.002','SolverType','Fixed-step');
    save_system(model,output); close_system(model,0);
    fprintf('Prepared %s: Arm=0, cap=4 V; not compiled or run.\n',output);
end
fprintf('Tuning vector: catch/drop deg, ke, amax, kx, kv, eps, kick, Eover, adapt, amp, Hz, lead deg, Ki angle, Ki x.\n');
fprintf('Stop Python before QUARC connects. Centre cart and hang pole before model startup.\n');
end

function constant(model,name,value,port,y)
add_block('simulink/Sources/Constant',[model '/' name], ...
    'Value',value,'Position',[480 y 610 y+30]);
add_line(model,[name '/1'],sprintf('Controller/%d',port),'autorouting','on');
end

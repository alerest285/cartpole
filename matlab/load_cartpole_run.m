function run = load_cartpole_run(csvfile)
% Load CSV + JSON from the Python runtime's runs/ folder for analysis.
run=struct('samples',readtable(csvfile),'metadata',struct());
[folder,name]=fileparts(csvfile); metafile=fullfile(folder,[name '.json']);
if isfile(metafile), run.metadata=jsondecode(fileread(metafile)); end
d=run.samples; t=d.t-d.t(1);
figure('Name',name,'Color','w');
subplot(3,1,1); plot(t,100*d.x); ylabel('Cart [cm]'); grid on;
subplot(3,1,2); plot(t,180/pi*d.alpha); ylabel('Angle [deg]'); grid on;
subplot(3,1,3); plot(t,[d.v_des d.v]); ylabel('Voltage [V]'); xlabel('Time [s]');
legend('Requested','Applied command'); grid on;
end

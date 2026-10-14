% Run this script in MATLAB. Six controller simulations, plots and animation.
addpath(fileparts(mfilename('fullpath')));
validate_cartpole;
modes={'lqr','place','pid','energy','sine','ppo'};
results=cell(size(modes));
figure('Name','Cart-pole: nonlinear 500 Hz simulation','Color','w');
for j=1:numel(modes)
    results{j}=simulate_cartpole(modes{j},12);
    r=results{j}; d=r.data;
    subplot(3,2,j); plot(d(:,1),(180/pi)*(pi-mod(pi-d(:,3),2*pi)));
    title(sprintf('%s | balanced %d | max x %.1f cm',modes{j},r.balanced,r.max_abs_x_cm));
    xlabel('Time [s]'); ylabel('Angle from upright [deg]'); grid on;
    assert(r.balanced,'cartpole:simulation','%s failed: %s',modes{j},r.fault);
end
out=fullfile(fileparts(mfilename('fullpath')),'results');
if ~isfolder(out), mkdir(out); end
save(fullfile(out,'simulation_results.mat'),'results');
animate_cartpole(results{4});

function c = cp_config()
% Shared Python/MATLAB parameters, gains and trained policy; base MATLAB only.
here = fileparts(mfilename('fullpath'));
c = jsondecode(fileread(fullfile(here,'generated','contract.json')));
c.plant = cp_derive(c.plant);
c.truth = c.plant;
c.v_cap = 10; c.x_soft = 0.17; c.x_hard = 0.19; c.wn = 12;
c.x_ref = 0;
names = fieldnames(c.gains);
for k=1:numel(names)
    gain=c.gains.(names{k}); c.gains.(names{k})=gain(:)';
end
end

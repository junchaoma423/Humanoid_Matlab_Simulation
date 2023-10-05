clear, clc
% Run initialization
Humanoid_Parameters;
% Load the simulink model
handle = load_system('Humanoid_stepping_R2021b');

% Run the simulation
out = sim('Humanoid_stepping_R2021b', 1)

hws = get_param(handle, 'modelworkspace');
hws.DataSource = 'MAT-File';
hws.FileName = 'params';
hws.assignin('thigh_mass', 2);
hws.saveToSource;
hws.reload;

total_torque = sum(out.tau_MPC(:))

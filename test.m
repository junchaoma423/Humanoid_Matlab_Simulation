clear, clc

Humanoid_Parameters;

% Define discrete set of thigh_mass values
mass_values = [1.563,1.731,1.667, 2.202];
com_x_value = [0, 0, 0, 0];
com_y_value = [0.018, 0.0238, 0.021, 0.0356];
com_z_value = [-0.0531, -0.048, -0.05, -0.0377];
ixx_value = [0.0073, 0.00832, 0.0078, 0.01088];
iyy_value = [0.0064, 0.00701, 0.00674, 0.00891];
izz_value = [0.00283, 0.00343, 0.00311, 0.00512];

% Initialize variables to store optimal results
min_torque = inf;  % Set initial minimum torque to infinity
optimal_mass = mass_values(1);  % Set initial optimal mass to first element
optimal_com_x = com_x_value(1);
optimal_com_y = com_y_value(1);
optimal_com_z = com_z_value(1);
optimal_ixx = ixx_value(1);
optimal_iyy = iyy_value(1);
optimal_izz = izz_value(1);

% Iterate through all possible mass values
for i = 2:length(mass_values)
    thigh_mass = mass_values(i);
    com_x = com_x_value(i);
    com_y = com_y_value(i);
    com_z = com_z_value(i);
    ixx = ixx_value(i);
    iyy = iyy_value(i);
    izz = izz_value(i);
    
    % Compute the objective function for the current mass value
    current_torque = objectiveFunction(thigh_mass, com_x, com_y, com_z, ixx, iyy, izz);
    
    % Update optimal results if current result is better
    if current_torque < min_torque
        min_torque = current_torque;
        optimal_mass = thigh_mass;
        optimal_com_x = com_x;
        optimal_com_y = com_y;
        optimal_com_z = com_z;
        optimal_ixx = ixx;
        optimal_iyy = iyy;
        optimal_izz = izz;
    end
end

% Display the optimal results
fprintf('Optimal thigh mass: %.2f\n', optimal_mass);
fprintf('Optimal com_y: %.2f\n', optimal_com_y)
fprintf('Minimum total torque: %.2f\n', min_torque);

% Nested function for optimization objective
function total_torque = objectiveFunction(thigh_mass, x_com, y_com, z_com, ixx, iyy, izz)
    % Run initialization
%     Humanoid_Parameters;

    % Load the Simulink model
    handle = load_system('Humanoid_stepping_R2021b');

    % Modify thigh_mass in the model workspace
    hws = get_param(handle, 'modelworkspace');
    hws.DataSource = 'MAT-File';
    hws.FileName = 'params';
    hws.assignin('thigh_mass', thigh_mass);
    hws.assignin('thigh_com_x', x_com);
    hws.assignin('thigh_com_y', y_com);
    hws.assignin('thigh_com_z', z_com);
    hws.assignin('thigh_ixx', ixx);
    hws.assignin('thigh_iyy', iyy);
    hws.assignin('thigh_izz', izz);
    hws.saveToSource;
    hws.reload;

    % Run the simulation
    out = sim('Humanoid_stepping_R2021b', 1);
    
    % Compute the objective to minimize
    total_torque = sum(out.tau_MPC(:));
    
    % Optionally, close the model to avoid excessive RAM usage
    save_system('Humanoid_stepping_R2021b');
    close_system('Humanoid_stepping_R2021b');
end

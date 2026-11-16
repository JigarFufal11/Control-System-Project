%% build_model.m — Programmatically build the Simulink thermal control model
%  Creates 'thermal_control' model with:
%    - Energy-balance thermal plant (u·P, R_th input, C, integrator, delay)
%    - ON-OFF relay controller (±0.5°C hysteresis)
%    - PI controller with anti-windup and saturation [0,1]
%    - Manual switch to select controller
%    - Reference signal (30→40→35°C)
%    - Disturbance on R_th
%    - Scopes and logging to workspace
%
%  Prerequisites: Run init_params.m first to load workspace variables.
%
%  Author : Control Systems Project Team
%  -----------------------------------------------------------------------

%% ---- Check prerequisites ----------------------------------------------
if ~exist('P', 'var') || ~exist('C_th', 'var')
    fprintf('Running init_params.m first...\n');
    run('init_params.m');
end

%% ---- Model Setup -------------------------------------------------------
modelName = 'thermal_control';

% Close if already open
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

% Create new model
new_system(modelName);

fprintf('Building model: %s\n', modelName);

%% ---- Helper function for block positioning -----------------------------
% Grid layout: each block is 30x30, spaced 100 apart
pos = @(col, row, w, h) [(col-1)*120+50, (row-1)*80+30, ...
                          (col-1)*120+50+w, (row-1)*80+30+h];

%% ========================================================================
%  PLANT SUBSYSTEM
%  ========================================================================
% Create plant subsystem
plantSys = [modelName '/Plant'];
add_block('simulink/Ports & Subsystems/Subsystem', plantSys);
delete_line(plantSys, 'In1/1', 'Out1/1');
delete_block([plantSys '/In1']);
delete_block([plantSys '/Out1']);

% ---- Plant Inputs ----
add_block('simulink/Sources/In1', [plantSys '/u'], ...
    'Position', pos(1,2,30,14));
add_block('simulink/Sources/In1', [plantSys '/R_th_in'], ...
    'Position', pos(1,4,30,14), 'Port', '2');

% ---- Heater Power: u * P ----
add_block('simulink/Math Operations/Gain', [plantSys '/Gain_P'], ...
    'Gain', 'P', 'Position', pos(2,2,40,30));

% ---- Heat loss: (T - T_amb) / R_th ----
add_block('simulink/Math Operations/Sum', [plantSys '/Sum_Tamb'], ...
    'Inputs', '+-', 'Position', pos(5,3,20,20));
add_block('simulink/Sources/Constant', [plantSys '/T_amb_const'], ...
    'Value', 'T_amb', 'Position', pos(4,4,40,20));
add_block('simulink/Math Operations/Product', [plantSys '/Divide_Rth'], ...
    'Inputs', '*/', 'Position', pos(6,3,30,30));

% ---- Energy balance sum: Q_in - Q_loss ----
add_block('simulink/Math Operations/Sum', [plantSys '/Sum_Energy'], ...
    'Inputs', '+-', 'Position', pos(3,2,20,20));

% ---- 1/C gain ----
add_block('simulink/Math Operations/Gain', [plantSys '/Gain_1_C'], ...
    'Gain', '1/C_th', 'Position', pos(4,2,40,30));

% ---- Integrator (IC = T_amb) ----
add_block('simulink/Continuous/Integrator', [plantSys '/Integrator'], ...
    'InitialCondition', 'T_amb', 'Position', pos(5,2,40,30));

% ---- Transport Delay ----
add_block('simulink/Continuous/Transport Delay', [plantSys '/Delay'], ...
    'DelayTime', 'L', 'Position', pos(6,2,40,30));

% ---- Output ----
add_block('simulink/Sinks/Out1', [plantSys '/T_out'], ...
    'Position', pos(7,2,30,14));

% ---- Wire the plant ----
add_line(plantSys, 'u/1',          'Gain_P/1');
add_line(plantSys, 'Gain_P/1',     'Sum_Energy/1');
add_line(plantSys, 'Sum_Energy/1', 'Gain_1_C/1');
add_line(plantSys, 'Gain_1_C/1',   'Integrator/1');
add_line(plantSys, 'Integrator/1', 'Delay/1');
add_line(plantSys, 'Delay/1',      'T_out/1');

% Feedback: (T - T_amb) / R_th
add_line(plantSys, 'Integrator/1', 'Sum_Tamb/1');
add_line(plantSys, 'T_amb_const/1','Sum_Tamb/2');
add_line(plantSys, 'Sum_Tamb/1',   'Divide_Rth/1');
add_line(plantSys, 'R_th_in/1',    'Divide_Rth/2');
add_line(plantSys, 'Divide_Rth/1', 'Sum_Energy/2');

set_param(plantSys, 'Position', pos(6,3,180,80));

%% ========================================================================
%  REFERENCE SIGNAL
%  ========================================================================
% Use From Workspace block with timeseries
% Create reference timeseries in workspace
ref_ts = timeseries([30; 30; 40; 40; 35; 35], [0; 599; 600; 1199; 1200; 1800]);
assignin('base', 'ref_ts', ref_ts);

add_block('simulink/Sources/From Workspace', [modelName '/Reference'], ...
    'VariableName', 'ref_ts', ...
    'OutputAfterFinalValue', 'Holding final value', ...
    'Position', pos(1,2,80,30));

%% ========================================================================
%  ERROR COMPUTATION
%  ========================================================================
add_block('simulink/Math Operations/Sum', [modelName '/Error_Sum'], ...
    'Inputs', '+-', 'Position', pos(3,2,20,20));

%% ========================================================================
%  ON-OFF CONTROLLER (Relay)
%  ========================================================================
add_block('simulink/Discontinuities/Relay', [modelName '/ON_OFF_Relay'], ...
    'OnSwitchValue', '0.5', ...
    'OffSwitchValue', '-0.5', ...
    'OnOutputValue', '1', ...
    'OffOutputValue', '0', ...
    'Position', pos(4,1,40,30));

%% ========================================================================
%  PI CONTROLLER
%  ========================================================================
add_block('simulink/Continuous/PID Controller', [modelName '/PI_Controller'], ...
    'Controller', 'PI', ...
    'P', num2str(Kp), ...
    'I', num2str(Ki), ...
    'D', '0', ...
    'UpperSaturationLimit', '1', ...
    'LowerSaturationLimit', '0', ...
    'AntiWindupMode', 'clamping', ...
    'SampleTime', '-1', ...
    'Position', pos(4,3,80,40));

%% ========================================================================
%  CONTROLLER SELECTOR (Manual Switch)
%  ========================================================================
add_block('simulink/Signal Routing/Manual Switch', [modelName '/Controller_Switch'], ...
    'Position', pos(5,2,40,30));

%% ========================================================================
%  R_TH INPUT (for disturbance)
%  ========================================================================
% Nominal R_th with optional step disturbance
add_block('simulink/Sources/Step', [modelName '/R_th_Disturbance'], ...
    'Time', num2str(disturbance_time), ...
    'Before', num2str(R_th_nominal), ...
    'After', num2str(R_th_disturbed), ...
    'Position', pos(5,5,60,30));

% Constant R_th for normal runs
add_block('simulink/Sources/Constant', [modelName '/R_th_Nominal'], ...
    'Value', num2str(R_th_nominal), ...
    'Position', pos(4,5,50,20));

add_block('simulink/Signal Routing/Manual Switch', [modelName '/Disturbance_Switch'], ...
    'Position', pos(5,4,40,30));

%% ========================================================================
%  LOGGING (To Workspace)
%  ========================================================================
add_block('simulink/Sinks/To Workspace', [modelName '/Log_T'], ...
    'VariableName', 'T_sim', 'SaveFormat', 'Timeseries', ...
    'Position', pos(8,3,60,25));

add_block('simulink/Sinks/To Workspace', [modelName '/Log_u'], ...
    'VariableName', 'u_sim', 'SaveFormat', 'Timeseries', ...
    'Position', pos(8,2,60,25));

add_block('simulink/Sinks/To Workspace', [modelName '/Log_ref'], ...
    'VariableName', 'ref_sim', 'SaveFormat', 'Timeseries', ...
    'Position', pos(8,1,60,25));

%% ========================================================================
%  SCOPES
%  ========================================================================
add_block('simulink/Sinks/Scope', [modelName '/Temperature_Scope'], ...
    'Position', pos(9,3,40,30));
add_block('simulink/Sinks/Scope', [modelName '/Duty_Scope'], ...
    'Position', pos(9,2,40,30));

%% ========================================================================
%  CLOCK for time logging
%  ========================================================================
add_block('simulink/Sources/Clock', [modelName '/Clock'], ...
    'Position', pos(8,5,30,20));
add_block('simulink/Sinks/To Workspace', [modelName '/Log_time'], ...
    'VariableName', 't_sim', 'SaveFormat', 'Timeseries', ...
    'Position', pos(9,5,60,25));

%% ========================================================================
%  WIRING — Top Level
%  ========================================================================
% Reference → Error Sum (+)
add_line(modelName, 'Reference/1', 'Error_Sum/1', 'autorouting', 'smart');

% Error → ON-OFF Relay
add_line(modelName, 'Error_Sum/1', 'ON_OFF_Relay/1', 'autorouting', 'smart');

% Error → PI Controller
add_line(modelName, 'Error_Sum/1', 'PI_Controller/1', 'autorouting', 'smart');

% ON-OFF → Controller Switch (top)
add_line(modelName, 'ON_OFF_Relay/1', 'Controller_Switch/1', 'autorouting', 'smart');

% PI → Controller Switch (bottom)
add_line(modelName, 'PI_Controller/1', 'Controller_Switch/2', 'autorouting', 'smart');

% Controller Switch → Plant/u
add_line(modelName, 'Controller_Switch/1', 'Plant/1', 'autorouting', 'smart');

% R_th switching
add_line(modelName, 'R_th_Nominal/1', 'Disturbance_Switch/1', 'autorouting', 'smart');
add_line(modelName, 'R_th_Disturbance/1', 'Disturbance_Switch/2', 'autorouting', 'smart');
add_line(modelName, 'Disturbance_Switch/1', 'Plant/2', 'autorouting', 'smart');

% Plant/T → Error Sum (-) (feedback)
add_line(modelName, 'Plant/1', 'Error_Sum/2', 'autorouting', 'smart');

% Logging connections
add_line(modelName, 'Plant/1', 'Log_T/1', 'autorouting', 'smart');
add_line(modelName, 'Plant/1', 'Temperature_Scope/1', 'autorouting', 'smart');
add_line(modelName, 'Controller_Switch/1', 'Log_u/1', 'autorouting', 'smart');
add_line(modelName, 'Controller_Switch/1', 'Duty_Scope/1', 'autorouting', 'smart');
add_line(modelName, 'Reference/1', 'Log_ref/1', 'autorouting', 'smart');
add_line(modelName, 'Clock/1', 'Log_time/1', 'autorouting', 'smart');

%% ========================================================================
%  SIMULATION SETTINGS
%  ========================================================================
set_param(modelName, ...
    'StopTime',      num2str(t_final), ...
    'Solver',        'ode45', ...
    'MaxStep',       '1', ...
    'RelTol',        '1e-4', ...
    'SaveTime',      'on', ...
    'SaveOutput',    'on', ...
    'SignalLogging',  'on');

%% ---- Save the model ----------------------------------------------------
save_system(modelName);
fprintf('\nModel "%s" built and saved successfully.\n', modelName);
fprintf('  → Open it: open_system(''%s'')\n', modelName);
fprintf('  → Run it:  sim(''%s'')\n', modelName);

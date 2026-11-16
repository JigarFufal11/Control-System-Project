%% build_hardware_model.m — Programmatically build Simulink hardware deployment model
%  Creates 'thermal_control_hardware.slx' configured for Arduino Mega 2560
%  with LM35 analog sensor input, L298N dual H-bridge actuator outputs,
%  parallel ON-OFF / PI controllers, bidirectional split logic, and
%  Monitor & Tune (External Mode) scopes.
%
%  This script does NOT touch 'thermal_control.slx' (Phase 1 simulation model).
%
%  Prerequisites:
%    - Simulink Support Package for Arduino Hardware installed
%    - MATLAB R2020b or later recommended
%
%  Author : Control Systems Project Team
%  Date   : November 2026
%  Phase  : 2 — Hardware Implementation
%  -----------------------------------------------------------------------

clear; clc;
fprintf('==============================================================\n');
fprintf('  Phase 2: Hardware Deployment Model Builder\n');
fprintf('  Target : Arduino Mega 2560 + L298N + LM35\n');
fprintf('==============================================================\n\n');

%% ========================================================================
%  EASY-TO-EDIT PARAMETERS (update these once you measure real values)
%  ========================================================================

% ---- Control Parameters (from Phase 1 simulation) ----------------------
T_ref       = 30.0;         % Reference / setpoint temperature [°C]
SampleTime  = 0.1;          % Fixed-step discrete sample time [s]

% PI gains (IMC-tuned: K=15, tau=120, L=3, tau_c=12)
Kp          = 0.4444;       % Proportional gain
Ki          = 0.0037;       % Integral gain [1/s]

% ON-OFF relay hysteresis thresholds
relay_on    =  0.5;         % Switch ON  when error >  +0.5 °C
relay_off   = -0.5;         % Switch OFF when error < −0.5 °C

% ---- Electrical & Power Parameters -------------------------------------
V_supply    = 9.0;          % External DC supply voltage [V]
V_drop      = 2.0;          % L298N internal voltage drop [V]
V_coil      = V_supply - V_drop;    % Effective coil voltage [V]  (7.0)

P_coil_max  = 10.0;         % Heating coil power rating [W] at V_coil
R_coil      = V_coil^2 / P_coil_max;  % Coil resistance [Ω]  (~4.9)
I_max_coil  = V_coil / R_coil;        % Peak coil current [A] (~1.43)

I_limit_L298N = 2.0;        % L298N per-channel continuous current limit [A]

% ---- Derived: Safe PWM saturation limit ---------------------------------
% At full PWM (255), coil sees V_coil → draws I_max_coil.
% If I_max_coil > I_limit_L298N, scale PWM down to stay within safe current.
if I_max_coil <= I_limit_L298N
    MAX_HEATER_PWM = 255;
else
    % PWM ratio = (I_limit * R_coil / V_coil) * 255
    MAX_HEATER_PWM = floor((I_limit_L298N * R_coil / V_coil) * 255);
    MAX_HEATER_PWM = min(MAX_HEATER_PWM, 255);
end

% ---- ADC Scaling (LM35 on A0) ------------------------------------------
ADC_bits    = 10;            % Arduino Mega ADC resolution
V_ref_adc   = 5.0;           % ADC reference voltage [V]
% LM35: 10 mV/°C → T = (ADC_raw / 1024) * 500
ADC_scale   = (V_ref_adc / (2^ADC_bits)) * 100;  % = 500/1024 = 0.48828125

% ---- Pin Assignments (L298N + LM35) ------------------------------------
PIN_SENSOR_A0   = 0;        % LM35 → Analog Pin A0

PIN_ENA_PWM     = 9;        % Channel A: Heater coil enable (PWM)
PIN_IN1         = 7;        % Channel A: direction pin 1 (HIGH)
PIN_IN2         = 8;        % Channel A: direction pin 2 (LOW)

PIN_ENB_PWM     = 10;       % Channel B: Exhaust fan enable (PWM)
PIN_IN3         = 11;       % Channel B: direction pin 1 (HIGH)
PIN_IN4         = 12;       % Channel B: direction pin 2 (LOW)

% ---- ADC Noise Filter ---------------------------------------------------
filter_window   = 5;        % Moving-average window length [samples]

% ---- Safety Limits ------------------------------------------------------
T_max           = 50.0;     % Hard safety cutoff [°C]

%% ---- Print parameter summary -------------------------------------------
fprintf('Control Parameters:\n');
fprintf('  T_ref        = %.1f °C\n', T_ref);
fprintf('  SampleTime   = %.2f s\n', SampleTime);
fprintf('  Kp           = %.4f\n', Kp);
fprintf('  Ki           = %.4f\n', Ki);
fprintf('  Relay ON/OFF = +%.1f / %.1f °C\n', relay_on, relay_off);
fprintf('\nElectrical Parameters:\n');
fprintf('  V_supply     = %.1f V\n', V_supply);
fprintf('  V_coil       = %.1f V  (after %.1f V L298N drop)\n', V_coil, V_drop);
fprintf('  P_coil_max   = %.1f W\n', P_coil_max);
fprintf('  R_coil       = %.1f Ω\n', R_coil);
fprintf('  I_max_coil   = %.2f A\n', I_max_coil);
fprintf('  I_limit_L298N= %.1f A\n', I_limit_L298N);
fprintf('  MAX_HEATER_PWM = %d\n', MAX_HEATER_PWM);
fprintf('\nADC:\n');
fprintf('  ADC_scale    = %.8f  (500/1024)\n', ADC_scale);
fprintf('  Filter       = %d-sample moving average\n', filter_window);
fprintf('\n');

%% ========================================================================
%  MODEL SETUP
%  ========================================================================
modelName = 'thermal_control_hardware';

% Close if already open (don't save old version)
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

% Delete old .slx if it exists (fresh build every time)
slxFile = [modelName '.slx'];
if exist(slxFile, 'file')
    delete(slxFile);
    fprintf('  Deleted old %s\n', slxFile);
end

new_system(modelName);
open_system(modelName);
fprintf('Building model: %s\n\n', modelName);

%% ---- Helper for block positioning -------------------------------------
%  pos(col, row, w, h) → [left top right bottom]
pos = @(col, row, w, h) [(col-1)*140+30, (row-1)*90+30, ...
                          (col-1)*140+30+w, (row-1)*90+30+h];

%% ========================================================================
%  1. SENSOR INPUT — LM35 on Analog Pin A0
%  ========================================================================
fprintf('  [1/7] Sensor input (LM35 on A0)...\n');

% Analog Input block (Arduino)
add_block('simulink/Sources/Constant', [modelName '/Analog_Input_Placeholder'], ...
    'Value', '0', ...
    'Position', pos(1,3,60,30));
set_param([modelName '/Analog_Input_Placeholder'], ...
    'Name', 'ADC_A0_Raw');
% NOTE: Replace this Constant block with the actual Arduino Analog Input
% block once the Simulink Support Package for Arduino is installed:
%   add_block('arduinolib/Analog Input', [modelName '/ADC_A0_Raw'], ...
%       'PinNumber', num2str(PIN_SENSOR_A0), ...
%       'SampleTime', num2str(SampleTime), ...
%       'Position', pos(1,3,60,30));
% The Constant '0' placeholder allows the model to build/check without
% the hardware support package.

% Data Type Conversion: uint16 → double
add_block('simulink/Signal Attributes/Data Type Conversion', ...
    [modelName '/ADC_to_Double'], ...
    'OutDataTypeStr', 'double', ...
    'Position', pos(2,3,60,25));

% Gain block: raw ADC → °C  (× 500/1024)
add_block('simulink/Math Operations/Gain', [modelName '/ADC_Scale'], ...
    'Gain', num2str(ADC_scale, '%.10f'), ...
    'Position', pos(3,3,50,25));

% Moving Average Filter (Discrete FIR Filter)
% Coefficients: [1/N, 1/N, ..., 1/N] with N = filter_window
fir_coeffs = ones(1, filter_window) / filter_window;
add_block('simulink/Discrete/Discrete FIR Filter', ...
    [modelName '/Noise_Filter'], ...
    'Coefficients', mat2str(fir_coeffs), ...
    'SampleTime', num2str(SampleTime), ...
    'Position', pos(4,3,70,30));

% Wire sensor chain
add_line(modelName, 'ADC_A0_Raw/1',   'ADC_to_Double/1', 'autorouting', 'smart');
add_line(modelName, 'ADC_to_Double/1', 'ADC_Scale/1',     'autorouting', 'smart');
add_line(modelName, 'ADC_Scale/1',     'Noise_Filter/1',  'autorouting', 'smart');

% Label the filtered output as T_out
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Tout'], ...
    'GotoTag', 'T_out', ...
    'Position', pos(5,3,50,18));
add_line(modelName, 'Noise_Filter/1', 'Goto_Tout/1', 'autorouting', 'smart');

% From tag for T_out (used in error computation and scope)
add_block('simulink/Signal Routing/From', [modelName '/From_Tout_FB'], ...
    'GotoTag', 'T_out', ...
    'Position', pos(5,5,50,18));

%% ========================================================================
%  2. REFERENCE & ERROR COMPUTATION
%  ========================================================================
fprintf('  [2/7] Reference & error computation...\n');

% Constant block for T_ref (tunable via Monitor & Tune)
add_block('simulink/Sources/Constant', [modelName '/T_ref'], ...
    'Value', num2str(T_ref), ...
    'Position', pos(5,1,50,25));

% Sum block: e = T_ref − T_out
add_block('simulink/Math Operations/Sum', [modelName '/Error_Sum'], ...
    'Inputs', '+-', ...
    'Position', pos(6,3,25,25));

% Wire: T_ref → Error_Sum (+)
add_line(modelName, 'T_ref/1', 'Error_Sum/1', 'autorouting', 'smart');

% Wire: T_out feedback → Error_Sum (−)
add_line(modelName, 'From_Tout_FB/1', 'Error_Sum/2', 'autorouting', 'smart');

%% ========================================================================
%  3. PARALLEL CONTROLLERS + MANUAL SWITCH
%  ========================================================================
fprintf('  [3/7] Controllers (ON-OFF relay + discrete PI)...\n');

% ---- ON-OFF Relay -------------------------------------------------------
add_block('simulink/Discontinuities/Relay', [modelName '/ON_OFF_Relay'], ...
    'OnSwitchValue',  num2str(relay_on), ...
    'OffSwitchValue', num2str(relay_off), ...
    'OnOutputValue',  '255', ...
    'OffOutputValue', '-255', ...
    'Position', pos(7,2,50,30));

% ---- Discrete PI Controller ---------------------------------------------
add_block('simulink/Discrete/Discrete PID Controller', ...
    [modelName '/PI_Controller'], ...
    'Controller', 'PI', ...
    'P', num2str(Kp, '%.6f'), ...
    'I', num2str(Ki, '%.6f'), ...
    'D', '0', ...
    'SampleTime', num2str(SampleTime), ...
    'UpperSaturationLimit', '255', ...
    'LowerSaturationLimit', '-255', ...
    'AntiWindupMode', 'clamping', ...
    'Position', pos(7,4,80,40));

% ---- Manual Switch (Controller Selector) --------------------------------
add_block('simulink/Signal Routing/Manual Switch', ...
    [modelName '/Controller_Switch'], ...
    'Position', pos(8,3,35,30));

% Wire error to both controllers
add_line(modelName, 'Error_Sum/1', 'ON_OFF_Relay/1',   'autorouting', 'smart');
add_line(modelName, 'Error_Sum/1', 'PI_Controller/1',  'autorouting', 'smart');

% Wire controllers to switch
add_line(modelName, 'ON_OFF_Relay/1',   'Controller_Switch/1', 'autorouting', 'smart');
add_line(modelName, 'PI_Controller/1',  'Controller_Switch/2', 'autorouting', 'smart');

% Goto tag for controller output (u)
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_u'], ...
    'GotoTag', 'ctrl_u', ...
    'Position', pos(9,3,50,18));
add_line(modelName, 'Controller_Switch/1', 'Goto_u/1', 'autorouting', 'smart');

%% ========================================================================
%  4. BIDIRECTIONAL SPLIT LOGIC
%  ========================================================================
fprintf('  [4/7] Bidirectional split logic (heater / fan)...\n');

% Create a subsystem for the split logic
splitSys = [modelName '/Split_Logic'];
add_block('simulink/Ports & Subsystems/Subsystem', splitSys, ...
    'Position', pos(10,3,120,70));

% Clean out default contents
delete_line(splitSys, 'In1/1', 'Out1/1');
delete_block([splitSys '/In1']);
delete_block([splitSys '/Out1']);

% ---- Subsystem ports ----
add_block('simulink/Sources/In1', [splitSys '/u'], ...
    'Position', [30 80 60 100]);

add_block('simulink/Sinks/Out1', [splitSys '/Heater_PWM'], ...
    'Position', [550 45 580 65], 'Port', '1');
add_block('simulink/Sinks/Out1', [splitSys '/Fan_PWM'], ...
    'Position', [550 135 580 155], 'Port', '2');

% ---- Heating path: when u > 0 ----
% Switch: if u > 0 → pass u, else 0
add_block('simulink/Signal Routing/Switch', [splitSys '/Heat_Switch'], ...
    'Criteria', 'u2 > Threshold', ...
    'Threshold', '0', ...
    'Position', [200 30 240 80]);

add_block('simulink/Sources/Constant', [splitSys '/Zero_Heat'], ...
    'Value', '0', ...
    'Position', [120 60 150 80]);

% Saturation [0, MAX_HEATER_PWM]
add_block('simulink/Discontinuities/Saturation', [splitSys '/Sat_Heater'], ...
    'UpperLimit', num2str(MAX_HEATER_PWM), ...
    'LowerLimit', '0', ...
    'Position', [280 35 330 65]);

% Cast to uint8
add_block('simulink/Signal Attributes/Data Type Conversion', ...
    [splitSys '/Cast_Heater'], ...
    'OutDataTypeStr', 'uint8', ...
    'RndMeth', 'Floor', ...
    'Position', [360 35 420 65]);

% Wire heating path
add_line(splitSys, 'u/1',            'Heat_Switch/1', 'autorouting', 'smart');
add_line(splitSys, 'u/1',            'Heat_Switch/2', 'autorouting', 'smart');
add_line(splitSys, 'Zero_Heat/1',    'Heat_Switch/3', 'autorouting', 'smart');
add_line(splitSys, 'Heat_Switch/1',  'Sat_Heater/1',  'autorouting', 'smart');
add_line(splitSys, 'Sat_Heater/1',   'Cast_Heater/1', 'autorouting', 'smart');
add_line(splitSys, 'Cast_Heater/1',  'Heater_PWM/1',  'autorouting', 'smart');

% ---- Cooling path: when u < 0 → negate and drive fan ----
% Negate u
add_block('simulink/Math Operations/Gain', [splitSys '/Negate'], ...
    'Gain', '-1', ...
    'Position', [120 125 160 155]);

% Switch: if u < 0 → pass -u, else 0
add_block('simulink/Signal Routing/Switch', [splitSys '/Cool_Switch'], ...
    'Criteria', 'u2 > Threshold', ...
    'Threshold', '0', ...
    'Position', [200 120 240 170]);

add_block('simulink/Sources/Constant', [splitSys '/Zero_Fan'], ...
    'Value', '0', ...
    'Position', [120 160 150 180]);

% Saturation [0, 255]
add_block('simulink/Discontinuities/Saturation', [splitSys '/Sat_Fan'], ...
    'UpperLimit', '255', ...
    'LowerLimit', '0', ...
    'Position', [280 125 330 155]);

% Cast to uint8
add_block('simulink/Signal Attributes/Data Type Conversion', ...
    [splitSys '/Cast_Fan'], ...
    'OutDataTypeStr', 'uint8', ...
    'RndMeth', 'Floor', ...
    'Position', [360 125 420 155]);

% Wire cooling path
add_line(splitSys, 'u/1',           'Negate/1',       'autorouting', 'smart');
add_line(splitSys, 'Negate/1',      'Cool_Switch/1',  'autorouting', 'smart');
add_line(splitSys, 'Negate/1',      'Cool_Switch/2',  'autorouting', 'smart');
add_line(splitSys, 'Zero_Fan/1',    'Cool_Switch/3',  'autorouting', 'smart');
add_line(splitSys, 'Cool_Switch/1', 'Sat_Fan/1',      'autorouting', 'smart');
add_line(splitSys, 'Sat_Fan/1',     'Cast_Fan/1',     'autorouting', 'smart');
add_line(splitSys, 'Cast_Fan/1',    'Fan_PWM/1',      'autorouting', 'smart');

% ---- Wire controller output into Split_Logic ----
add_block('simulink/Signal Routing/From', [modelName '/From_u_split'], ...
    'GotoTag', 'ctrl_u', ...
    'Position', pos(9,4,50,18));
add_line(modelName, 'From_u_split/1', 'Split_Logic/1', 'autorouting', 'smart');

%% ========================================================================
%  5. ACTUATOR OUTPUTS — L298N via Arduino Digital/PWM
%  ========================================================================
fprintf('  [5/7] Actuator outputs (L298N H-bridge)...\n');

% ---- Channel A: Heating Coil ----
% ENA (PWM Pin 9) — receives Heater_PWM from split logic
add_block('simulink/Sinks/Terminator', [modelName '/PWM_Pin9_Heater'], ...
    'Position', pos(12,2,30,20));
% NOTE: Replace with Arduino PWM block:
%   add_block('arduinolib/PWM', [modelName '/PWM_Pin9_Heater'], ...
%       'PinNumber', '9', 'Position', pos(12,2,60,30));

% IN1 → Pin 7 (Constant HIGH)
add_block('simulink/Sources/Constant', [modelName '/IN1_Pin7'], ...
    'Value', '1', ...
    'OutDataTypeStr', 'boolean', ...
    'Position', pos(12,1,50,20));
add_block('simulink/Sinks/Terminator', [modelName '/DO_Pin7'], ...
    'Position', pos(13,1,30,20));
% NOTE: Replace Terminator with Arduino Digital Output:
%   add_block('arduinolib/Digital Output', [modelName '/DO_Pin7'], ...
%       'PinNumber', '7', 'Position', pos(13,1,60,30));
add_line(modelName, 'IN1_Pin7/1', 'DO_Pin7/1', 'autorouting', 'smart');

% IN2 → Pin 8 (Constant LOW)
add_block('simulink/Sources/Constant', [modelName '/IN2_Pin8'], ...
    'Value', '0', ...
    'OutDataTypeStr', 'boolean', ...
    'Position', pos(12,0,50,20));
add_block('simulink/Sinks/Terminator', [modelName '/DO_Pin8'], ...
    'Position', pos(13,0,30,20));
add_line(modelName, 'IN2_Pin8/1', 'DO_Pin8/1', 'autorouting', 'smart');

% ---- Channel B: Exhaust Fan ----
% ENB (PWM Pin 10) — receives Fan_PWM from split logic
add_block('simulink/Sinks/Terminator', [modelName '/PWM_Pin10_Fan'], ...
    'Position', pos(12,4,30,20));
% NOTE: Replace with Arduino PWM block:
%   add_block('arduinolib/PWM', [modelName '/PWM_Pin10_Fan'], ...
%       'PinNumber', '10', 'Position', pos(12,4,60,30));

% IN3 → Pin 11 (Constant HIGH)
add_block('simulink/Sources/Constant', [modelName '/IN3_Pin11'], ...
    'Value', '1', ...
    'OutDataTypeStr', 'boolean', ...
    'Position', pos(12,5,50,20));
add_block('simulink/Sinks/Terminator', [modelName '/DO_Pin11'], ...
    'Position', pos(13,5,30,20));
add_line(modelName, 'IN3_Pin11/1', 'DO_Pin11/1', 'autorouting', 'smart');

% IN4 → Pin 12 (Constant LOW)
add_block('simulink/Sources/Constant', [modelName '/IN4_Pin12'], ...
    'Value', '0', ...
    'OutDataTypeStr', 'boolean', ...
    'Position', pos(12,6,50,20));
add_block('simulink/Sinks/Terminator', [modelName '/DO_Pin12'], ...
    'Position', pos(13,6,30,20));
add_line(modelName, 'IN4_Pin12/1', 'DO_Pin12/1', 'autorouting', 'smart');

% ---- Wire Split_Logic outputs to PWM actuators ----
add_line(modelName, 'Split_Logic/1', 'PWM_Pin9_Heater/1', 'autorouting', 'smart');
add_line(modelName, 'Split_Logic/2', 'PWM_Pin10_Fan/1',   'autorouting', 'smart');

%% ========================================================================
%  6. LIVE TELEMETRY SCOPES (Monitor & Tune)
%  ========================================================================
fprintf('  [6/7] Telemetry scopes...\n');

% ---- Temperature Scope: T_ref and T_out ----
add_block('simulink/Sinks/Scope', [modelName '/Temperature_Scope'], ...
    'NumInputPorts', '2', ...
    'Position', pos(8,6,45,35));

% From tags for scope inputs
add_block('simulink/Signal Routing/From', [modelName '/From_Tout_Scope'], ...
    'GotoTag', 'T_out', ...
    'Position', pos(6,6,50,18));
add_block('simulink/Signal Routing/From', [modelName '/From_Tref_Scope'], ...
    'GotoTag', 'T_ref_tag', ...
    'Position', pos(6,7,50,18));

% Add Goto for T_ref so scope can read it
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Tref'], ...
    'GotoTag', 'T_ref_tag', ...
    'Position', pos(6,1,50,18));
add_line(modelName, 'T_ref/1', 'Goto_Tref/1', 'autorouting', 'smart');

add_line(modelName, 'From_Tref_Scope/1', 'Temperature_Scope/1', 'autorouting', 'smart');
add_line(modelName, 'From_Tout_Scope/1', 'Temperature_Scope/2', 'autorouting', 'smart');

% ---- Duty Scope: Heater PWM % and Fan PWM % ----
add_block('simulink/Sinks/Scope', [modelName '/Duty_Scope'], ...
    'NumInputPorts', '2', ...
    'Position', pos(14,3,45,35));

% Gain blocks to convert uint8 (0-255) → % (0-100)
add_block('simulink/Math Operations/Gain', [modelName '/Heater_Pct'], ...
    'Gain', num2str(100/255, '%.8f'), ...
    'Position', pos(13,2,50,25));
add_block('simulink/Math Operations/Gain', [modelName '/Fan_Pct'], ...
    'Gain', num2str(100/255, '%.8f'), ...
    'Position', pos(13,4,50,25));

% Wire Split_Logic → percentage → Duty_Scope
add_line(modelName, 'Split_Logic/1', 'Heater_Pct/1', 'autorouting', 'smart');
add_line(modelName, 'Split_Logic/2', 'Fan_Pct/1',    'autorouting', 'smart');
add_line(modelName, 'Heater_Pct/1',  'Duty_Scope/1', 'autorouting', 'smart');
add_line(modelName, 'Fan_Pct/1',     'Duty_Scope/2', 'autorouting', 'smart');

%% ========================================================================
%  7. MODEL CONFIGURATION — Arduino Mega 2560, Fixed-Step, External Mode
%  ========================================================================
fprintf('  [7/7] Model configuration...\n');

% ---- Solver: fixed-step discrete ----
set_param(modelName, ...
    'SolverType',            'Fixed-step', ...
    'Solver',                'FixedStepDiscrete', ...
    'FixedStep',             num2str(SampleTime), ...
    'StopTime',              'inf');

% ---- Hardware board configuration ----
% Try to set Arduino Mega 2560 as hardware target.
% This requires Simulink Support Package for Arduino Hardware.
try
    set_param(modelName, 'HardwareBoard', 'Arduino Mega 2560');
    fprintf('    Hardware board set: Arduino Mega 2560\n');
catch ME
    warning('Could not set HardwareBoard (support package may not be installed):\n  %s', ME.message);
    fprintf('    [!] Set HardwareBoard manually: Arduino Mega 2560\n');
end

% ---- External mode (Monitor & Tune) ----
try
    set_param(modelName, 'SimulationMode', 'external');
    fprintf('    Simulation mode set: External (Monitor & Tune)\n');
catch ME
    % External mode might not be settable programmatically on all versions
    fprintf('    [!] Set External Mode manually for Monitor & Tune\n');
end

%% ========================================================================
%  ANNOTATIONS (model documentation inside the diagram)
%  ========================================================================
try
    add_block('built-in/Note', [modelName '/Note_Title'], ...
        'Position', [30 -60 600 -20], ...
        'Text', sprintf(['THERMAL CONTROL HARDWARE — Arduino Mega 2560\n' ...
                          'Phase 2 Hardware Deployment  |  SampleTime = %.2f s  |  ' ...
                          'T_ref = %.1f °C  |  MAX_HEATER_PWM = %d'], ...
                          SampleTime, T_ref, MAX_HEATER_PWM));
catch
    % Note block may not be available in all versions; non-critical
end

%% ========================================================================
%  SAVE MODEL
%  ========================================================================
save_system(modelName);
fprintf('\n==============================================================\n');
fprintf('  Model "%s" built and saved successfully.\n', modelName);
fprintf('  File: %s\n', which(modelName));
fprintf('==============================================================\n\n');

fprintf('NEXT STEPS:\n');
fprintf('  1. Install Simulink Support Package for Arduino Hardware\n');
fprintf('     (if not already installed)\n');
fprintf('  2. Replace placeholder blocks (search for "NOTE:" comments above):\n');
fprintf('     - ADC_A0_Raw        → arduinolib/Analog Input   (Pin A0)\n');
fprintf('     - PWM_Pin9_Heater   → arduinolib/PWM            (Pin 9)\n');
fprintf('     - PWM_Pin10_Fan     → arduinolib/PWM            (Pin 10)\n');
fprintf('     - DO_Pin7           → arduinolib/Digital Output  (Pin 7)\n');
fprintf('     - DO_Pin8           → arduinolib/Digital Output  (Pin 8)\n');
fprintf('     - DO_Pin11          → arduinolib/Digital Output  (Pin 11)\n');
fprintf('     - DO_Pin12          → arduinolib/Digital Output  (Pin 12)\n');
fprintf('  3. Connect Arduino Mega 2560 via USB\n');
fprintf('  4. Deploy: Build, Deploy & Start (Ctrl+B)\n');
fprintf('  5. Monitor: Simulink → Hardware → Monitor & Tune\n');
fprintf('  6. Tune T_ref live via the Constant block in External Mode\n\n');

fprintf('To update heater safety limit for a different coil:\n');
fprintf('  → Edit R_coil or P_coil_max at the top of this script\n');
fprintf('  → Re-run to rebuild with correct MAX_HEATER_PWM\n\n');

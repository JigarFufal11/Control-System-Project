function model_name = build_simulink_model(output_dir)
% BUILD_SIMULINK_MODEL - Programmatically constructs the complete Simulink
% model for Quanser QNET HVAC Trainer 1.0 on NI ELVIS II+ in MATLAB R2026a.
%
% Generates: qnet_hvac_simulink.slx

    if nargin < 1 || isempty(output_dir)
        output_dir = fileparts(mfilename('fullpath'));
    end

    model_name = 'qnet_hvac_simulink';
    model_path = fullfile(output_dir, model_name);

    % Close if already open
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end

    % Create new system
    new_system(model_name);
    open_system(model_name);

    % Configure solver settings for thermal simulation
    set_param(model_name, 'Solver', 'ode45', ...
                          'StopTime', '360.0', ...
                          'RelTol', '1e-4', ...
                          'AbsTol', '1e-6', ...
                          'SaveOutput', 'on', ...
                          'OutputSaveName', 'yout');

    %% 1. Setpoint & Disturbance Sources
    % Baseline ambient temperature: 22 °C
    add_block('simulink/Sources/Constant', [model_name '/T_baseline'], ...
        'Position', [30, 60, 75, 90], 'Value', '22.0');

    % Step 1: Step to 32 °C at t = 20 s (+10 °C)
    add_block('simulink/Sources/Step', [model_name '/Step1'], ...
        'Position', [30, 110, 75, 140], 'Time', '20.0', 'Before', '0.0', 'After', '10.0');

    % Step 2: Step to 42 °C at t = 140 s (+10 °C)
    add_block('simulink/Sources/Step', [model_name '/Step2'], ...
        'Position', [30, 160, 75, 190], 'Time', '140.0', 'Before', '0.0', 'After', '10.0');

    % Step 3: Step down to 36 °C at t = 260 s (-6 °C)
    add_block('simulink/Sources/Step', [model_name '/Step3'], ...
        'Position', [30, 210, 75, 240], 'Time', '260.0', 'Before', '0.0', 'After', '-6.0');

    % Sum of setpoint steps
    add_block('simulink/Math Operations/Sum', [model_name '/Sum_Setpoint'], ...
        'Position', [110, 120, 135, 180], 'Inputs', '++++');

    % Ambient Temperature Source (22.0 °C)
    add_block('simulink/Sources/Constant', [model_name '/T_ambient'], ...
        'Position', [30, 310, 80, 340], 'Value', '22.0');

    %% 2. Controller Subsystems
    % Error Junction
    add_block('simulink/Math Operations/Sum', [model_name '/Sum_Error'], ...
        'Position', [180, 135, 205, 165], 'Inputs', '+-');

    % Proportional-Integral (PI) Controller with Clamping Anti-Windup
    add_block('simulink/Continuous/PID Controller', [model_name '/PI_Controller'], ...
        'Position', [250, 100, 310, 140], ...
        'Controller', 'PI', ...
        'P', '1.35', ...
        'I', '0.042', ...
        'LimitOutput', 'on', ...
        'UpperSaturationLimit', '5.0', ...
        'LowerSaturationLimit', '0.0', ...
        'AntiWindupMode', 'clamping');

    % ON-OFF Relay with Hysteresis (0 to 5 V, Hysteresis = +/- 0.4 °C)
    add_block('simulink/Discontinuities/Relay', [model_name '/Relay_Controller'], ...
        'Position', [250, 175, 310, 215], ...
        'OnSwitchValue', '0.4', ...
        'OffSwitchValue', '-0.4', ...
        'OnOutputValue', '5.0', ...
        'OffOutputValue', '0.0');

    % Manual Switch to choose between PI and ON-OFF control
    add_block('simulink/Signal Routing/Manual Switch', [model_name '/Controller_Select'], ...
        'Position', [360, 135, 395, 175]);

    % Actuator Saturation (0 to 5 V hardware limit)
    add_block('simulink/Discontinuities/Saturation', [model_name '/Actuator_Clamp'], ...
        'Position', [425, 140, 460, 170], ...
        'UpperLimit', '5.0', ...
        'LowerLimit', '0.0');

    %% 3. Thermal Plant Model
    % Gain: Heater Electrical to Thermal Power (K_heater = 22.4 W/V)
    add_block('simulink/Math Operations/Gain', [model_name '/K_heater'], ...
        'Position', [490, 140, 530, 170], 'Gain', '22.4');

    % Thermal Net Power Sum: Q_in - Q_loss
    add_block('simulink/Math Operations/Sum', [model_name '/Sum_Thermal'], ...
        'Position', [560, 145, 585, 170], 'Inputs', '+-');

    % Thermal Capacitance Gain: 1 / C_th (1 / 145.0)
    add_block('simulink/Math Operations/Gain', [model_name '/Thermal_Capacitance'], ...
        'Position', [615, 140, 665, 175], 'Gain', '1/145.0');

    % Thermal Integrator (dT/dt -> T_core)
    add_block('simulink/Continuous/Integrator', [model_name '/Integrator'], ...
        'Position', [695, 140, 730, 175], 'InitialCondition', '22.0');

    % Transport Delay (Air transit time along duct, theta = 2.0 s)
    add_block('simulink/Continuous/Transport Delay', [model_name '/Transport_Delay'], ...
        'Position', [760, 140, 800, 175], 'DelayTime', '2.0', 'InitialOutput', '22.0');

    % Thermal Loss Feedback: (T - T_amb) / R_th
    add_block('simulink/Math Operations/Sum', [model_name '/Sum_DeltaT'], ...
        'Position', [680, 240, 705, 265], 'Inputs', '+-');

    add_block('simulink/Math Operations/Gain', [model_name '/Heat_Loss_Gain'], ...
        'Position', [600, 235, 655, 270], 'Gain', '1/0.224');

    %% 4. Observer & Fault Residual Generator
    % State Observer Transfer Function: K_nom / (tau_nom * s + 1)
    add_block('simulink/Continuous/Transfer Fcn', [model_name '/Observer_Model'], ...
        'Position', [540, 50, 630, 85], ...
        'Numerator', '[7.25]', ...
        'Denominator', '[32.5 1]');

    add_block('simulink/Math Operations/Sum', [model_name '/Sum_Observer_T'], ...
        'Position', [670, 55, 695, 80], 'Inputs', '++');

    % Residual Sum: r(t) = T_meas - T_hat
    add_block('simulink/Math Operations/Sum', [model_name '/Sum_Residual'], ...
        'Position', [840, 70, 865, 95], 'Inputs', '+-');

    %% 5. Scopes and Sinks
    % Scope 1: Temperature Tracking
    add_block('simulink/Sinks/Scope', [model_name '/Scope_Temperature'], ...
        'Position', [900, 130, 940, 170], 'NumInputPorts', '2');

    % Scope 2: Actuator Command Voltage
    add_block('simulink/Sinks/Scope', [model_name '/Scope_Actuator'], ...
        'Position', [490, 220, 530, 260]);

    % Scope 3: Fault Residual
    add_block('simulink/Sinks/Scope', [model_name '/Scope_Residual'], ...
        'Position', [900, 65, 940, 105]);

    % To Workspace Loggers
    add_block('simulink/Sinks/To Workspace', [model_name '/Log_Temp'], ...
        'Position', [900, 200, 960, 230], ...
        'VariableName', 'sim_T_meas', 'SaveFormat', 'Timeseries');

    add_block('simulink/Sinks/To Workspace', [model_name '/Log_Cmd'], ...
        'Position', [490, 280, 550, 310], ...
        'VariableName', 'sim_u_cmd', 'SaveFormat', 'Timeseries');

    %% 6. Wire Subsystem Connections
    % Setpoint profile sums
    add_line(model_name, 'T_baseline/1', 'Sum_Setpoint/1');
    add_line(model_name, 'Step1/1', 'Sum_Setpoint/2');
    add_line(model_name, 'Step2/1', 'Sum_Setpoint/3');
    add_line(model_name, 'Step3/1', 'Sum_Setpoint/4');

    % Setpoint to error junction and scope
    add_line(model_name, 'Sum_Setpoint/1', 'Sum_Error/1');
    add_line(model_name, 'Sum_Setpoint/1', 'Scope_Temperature/1');

    % Error to PI and Relay
    add_line(model_name, 'Sum_Error/1', 'PI_Controller/1');
    add_line(model_name, 'Sum_Error/1', 'Relay_Controller/1');

    % Controller Switch selection
    add_line(model_name, 'PI_Controller/1', 'Controller_Select/1');
    add_line(model_name, 'Relay_Controller/1', 'Controller_Select/2');

    % Controller output to Saturation
    add_line(model_name, 'Controller_Select/1', 'Actuator_Clamp/1');

    % Actuator to Plant, Observer, and Sinks
    add_line(model_name, 'Actuator_Clamp/1', 'K_heater/1');
    add_line(model_name, 'Actuator_Clamp/1', 'Observer_Model/1');
    add_line(model_name, 'Actuator_Clamp/1', 'Scope_Actuator/1');
    add_line(model_name, 'Actuator_Clamp/1', 'Log_Cmd/1');

    % Observer Path
    add_line(model_name, 'Observer_Model/1', 'Sum_Observer_T/1');
    add_line(model_name, 'T_ambient/1', 'Sum_Observer_T/2');
    add_line(model_name, 'Sum_Observer_T/1', 'Sum_Residual/2');

    % Plant Forward Path
    add_line(model_name, 'K_heater/1', 'Sum_Thermal/1');
    add_line(model_name, 'Sum_Thermal/1', 'Thermal_Capacitance/1');
    add_line(model_name, 'Thermal_Capacitance/1', 'Integrator/1');
    add_line(model_name, 'Integrator/1', 'Transport_Delay/1');

    % Feedback Loop from Transport Delay Output
    add_line(model_name, 'Transport_Delay/1', 'Sum_Error/2');
    add_line(model_name, 'Transport_Delay/1', 'Scope_Temperature/2');
    add_line(model_name, 'Transport_Delay/1', 'Log_Temp/1');
    add_line(model_name, 'Transport_Delay/1', 'Sum_Residual/1');
    add_line(model_name, 'Sum_Residual/1', 'Scope_Residual/1');

    % Plant Internal Heat Loss Feedback
    add_line(model_name, 'Integrator/1', 'Sum_DeltaT/1');
    add_line(model_name, 'T_ambient/1', 'Sum_DeltaT/2');
    add_line(model_name, 'Sum_DeltaT/1', 'Heat_Loss_Gain/1');
    add_line(model_name, 'Heat_Loss_Gain/1', 'Sum_Thermal/2');

    % Save and close
    save_system(model_name, [model_path '.slx']);
    close_system(model_name);

    fprintf('Simulink model successfully generated and saved to: %s.slx\n', model_path);

end

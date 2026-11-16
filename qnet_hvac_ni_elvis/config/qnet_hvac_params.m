function p = qnet_hvac_params()
% QNET_HVAC_PARAMS - Configuration parameters for Quanser QNET HVAC Trainer 1.0
% mounted on National Instruments NI ELVIS II / II+.
%
% References:
%   - Quanser QNET HVAC Trainer User Manual & Specifications
%   - National Instruments NI ELVIS II / II+ Hardware Specifications
%   - EE302 Control Systems Laboratory: Problem Statement 4

    p = struct();

    %% Hardware & Thermal Chamber Physics
    p.kit_name           = 'Quanser QNET HVAC Trainer 1.0';
    p.platform_name      = 'NI ELVIS II+ (National Instruments)';
    p.author             = 'EE302 Control Systems Lab Group';
    p.matlab_version     = 'MATLAB R2026a';

    % Thermal parameters
    p.T_amb              = 22.0;       % Baseline ambient room temperature [°C]
    p.T_max_safe         = 60.0;       % Maximum safe chamber operating temperature [°C]
    p.T_min_safe         = 10.0;       % Minimum allowable operating temperature [°C]
    p.T_alarm            = 55.0;       % Alarm / cutoff threshold for safety shutdown [°C]

    % FOPDT Transfer Function Nominal Parameters (at nominal fan command = 2.5 V):
    % G(s) = K / (tau * s + 1) * exp(-theta * s)
    p.K_nom              = 7.25;       % Nominal steady-state gain [°C / V]
    p.tau_nom            = 32.5;       % Nominal thermal time constant [seconds]
    p.theta_nom          = 2.0;        % Pure transport dead time / delay [seconds]

    % Energy-Balance Equivalent Parameters:
    % C_th * dT/dt = Q_in - Q_loss
    % Q_in = K_heater * u(t),  Q_loss = (T - T_amb) / R_th(v_fan)
    p.C_th               = 145.0;      % Chamber thermal capacitance [J / °C]
    p.R_th0              = 0.224;      % Thermal resistance at baseline airflow [°C / W]
    p.K_heater           = 32.366;     % Heater power coefficient [W / V_cmd] (K_nom = K_heater * R_th0 = 7.25 °C/V)
    p.R_heater_nominal   = 29.5;       % Heating element electrical resistance [Ohms]
    p.P_heater_max       = 19.5;       % Maximum electrical power dissipation [Watts]

    % Fan & Convective Airflow Parameters:
    % Fan speed modulates effective thermal resistance R_th and transport delay theta
    p.v_fan_nominal      = 2.5;        % Nominal fan command [V] (0 - 5 V scale)
    p.v_fan_min          = 0.5;        % Minimum fan operating voltage [V]
    p.v_fan_max          = 5.0;        % Maximum fan operating voltage [V]
    p.fan_convection_coeff = 0.35;     % Convection modulation coefficient [1/V]
    p.fan_delay_coeff    = 0.18;       % Airspeed delay scaling coefficient [s/V]

    %% Electrical & DAQ Interface Limits (NI ELVIS II+)
    p.u_min              = 0.0;        % Minimum heater control voltage [V]
    p.u_max              = 5.0;        % Maximum heater control voltage [V]
    p.sensor_sensitivity = 20.0;       % Temperature sensor scale: 20.0 °C / Volt
    p.sensor_offset      = 0.0;        % Voltage offset [V]
    p.sensor_noise_sigma = 0.08;       % Standard deviation of sensor analog noise [°C]
    p.adc_quantization   = 10.0 / 2^16;% 16-bit ADC resolution step [V]

    %% Default Controller Tuning Parameters
    % 1. ON-OFF (Hysteresis / Bang-Bang) Controller
    p.onoff_hysteresis   = 0.8;        % Hysteresis deadband Delta [°C] (+/- 0.4 °C)
    p.onoff_u_high       = 5.0;        % Actuator ON command [V]
    p.onoff_u_low        = 0.0;        % Actuator OFF command [V]

    % 2. Proportional-Integral (PI) Controller with Anti-Windup
    % Tuned via Chien-Hrones-Reswick (CHR) 20% overshoot method for FOPDT plant:
    % K_p = 0.7 * tau / (K * theta) = 0.7 * 32.5 / (7.25 * 2.0) = 1.57 V/°C
    % T_i = 1.2 * tau = 39.0 s -> K_i = K_p / T_i = 0.040 V/(°C*s)
    p.pi_Kp              = 1.35;       % Proportional gain [V / °C]
    p.pi_Ki              = 0.042;      % Integral gain [V / (°C * s)]
    p.pi_Kd              = 0.0;        % Derivative gain [V * s / °C] (PI preferred for noisy thermal systems)
    p.pi_aw_gain         = 0.10;       % Tracking anti-windup back-calculation gain
    p.enable_feedforward = true;       % Ambient feedforward compensation flag

    %% State Observer & Fault Detection Settings
    p.observer_gain_L    = 0.04;       % Luenberger observer correction gain [1/s]
    p.fault_thresh_fan   = 1.5;        % Airflow/Fan failure residual threshold [°C]
    p.fault_thresh_sensor= 1.5;        % Sensor disagreement threshold [°C]
    p.fault_thresh_heater= 1.8;        % Heater degradation residual threshold [°C]
    p.fault_debounce_sec = 2.0;        % Persistence time before latching fault alarm [s]

    %% Standard Simulation & Test Profile
    p.dt                 = 0.1;        % Discrete sampling step [s] (10 Hz rate)
    p.t_final            = 360.0;      % Total experiment run time [s] (6 minutes)
    
    % Multi-step setpoint profile [time_sec, setpoint_degC]
    p.setpoint_profile   = [
           0, 22.0;    % Initial baseline (ambient)
          20, 32.0;    % Step 1: Moderate heating (32 °C)
         140, 42.0;    % Step 2: High heating (42 °C)
         260, 36.0     % Step 3: Cool-down regulation (36 °C)
    ];

    % Disturbance profile: [time_sec, fan_voltage_V]
    p.disturbance_fan_profile = [
           0, 2.5;     % Nominal 2.5 V
         180, 4.5;     % +80% fan speed burst (strong cooling disturbance)
         220, 2.5      % Return to normal airflow
    ];

    % Disturbance profile for ambient: [time_sec, ambient_degC]
    p.disturbance_amb_profile = [
           0, 22.0;
         300, 19.0;    % Cold draft / ambient dip
         330, 22.0
    ];

end

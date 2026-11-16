function [dTdt, T_chamber_next, sensor_meas, plant_state] = qnet_thermal_plant(t, plant_state, u_heater, v_fan, T_amb, params, faults)
% QNET_THERMAL_PLANT - Non-linear first-principles thermal dynamics
% for the Quanser QNET HVAC Trainer 1.0 on NI ELVIS II+.
%
% Energy Balance:
%   C_th * dT/dt = Q_in(t - delay) - Q_loss(t)
%   Q_in(t)      = eta_heater * K_heater * u_heater(t)
%   Q_loss(t)    = (T(t) - T_amb(t)) / R_th(v_fan)
%
% Inputs:
%   t            - Current simulation time [s]
%   plant_state  - State struct holding internal temperatures, delay buffer, etc.
%   u_heater     - Heater command voltage [0 - 5 V]
%   v_fan        - Fan command voltage [0.5 - 5.0 V]
%   T_amb        - Current ambient temperature [°C]
%   params       - Configuration parameters struct from qnet_hvac_params()
%   faults       - (Optional) Fault injection flags struct:
%                  .fan_failure       - Boolean (true = fan stopped/air restricted)
%                  .sensor_bias       - Scalar bias added to chamber sensor [°C]
%                  .heater_degrade    - Scaling factor for heater efficiency (e.g. 0.5)
%
% Outputs:
%   dTdt           - Derivative of chamber temperature [°C / s]
%   T_chamber_next - Next state temperature after integration step [°C]
%   sensor_meas    - Struct containing simulated physical sensor readings:
%                    .T_chamber   - Primary duct thermistor reading [°C]
%                    .T_ambient   - Ambient thermistor reading [°C]
%                    .T_secondary - Downstream thermistor reading [°C]
%                    .V_ai0, V_ai1, V_ai2 - Corresponding NI ELVIS ADC voltages
%   plant_state    - Updated internal plant state struct

    if nargin < 6 || isempty(params)
        params = qnet_hvac_params();
    end
    if nargin < 7 || isempty(faults)
        faults = struct('fan_failure', false, 'sensor_bias', 0.0, 'heater_degrade', 1.0);
    end

    dt = params.dt;

    %% 1. Initialize Plant State on First Call
    if isempty(plant_state) || ~isfield(plant_state, 'T_chamber')
        plant_state = struct();
        plant_state.T_chamber     = T_amb;
        plant_state.T_secondary   = T_amb;
        % Delay buffer for transport lag (theta_nom / dt steps)
        buffer_len = max(1, round(params.theta_nom / dt));
        plant_state.delay_buffer  = repmat(u_heater, 1, buffer_len);
        plant_state.buffer_index  = 1;
    end

    %% 2. Apply Fault Injections to Physical Variables
    % Fault 1: Fan Failure / Airflow Obstruction
    effective_v_fan = v_fan;
    if isfield(faults, 'fan_failure') && faults.fan_failure
        effective_v_fan = 0.25; % Drastic loss of forced airflow convection
    end

    % Fault 2: Heater Degradation / Power Supply Drop
    heater_health = 1.0;
    if isfield(faults, 'heater_degrade') && ~isempty(faults.heater_degrade)
        heater_health = max(0.0, min(1.0, faults.heater_degrade));
    end

    % Sensor bias fault
    bias = 0.0;
    if isfield(faults, 'sensor_bias') && ~isempty(faults.sensor_bias)
        bias = faults.sensor_bias;
    end

    %% 3. Thermal Dynamics Computation
    % Transport delay modeling via ring buffer
    buffer_len = length(plant_state.delay_buffer);
    u_delayed  = plant_state.delay_buffer(plant_state.buffer_index);
    plant_state.delay_buffer(plant_state.buffer_index) = u_heater;
    plant_state.buffer_index = mod(plant_state.buffer_index, buffer_len) + 1;

    % Convection-modulated thermal resistance:
    % Higher fan voltage -> higher airflow velocity -> lower thermal resistance (faster cooling)
    fan_factor = 1.0 + params.fan_convection_coeff * (effective_v_fan - params.v_fan_nominal);
    fan_factor = max(0.15, fan_factor); % Prevent negative or zero resistance
    R_th_eff   = params.R_th0 / fan_factor;

    % Thermal Power Inflow & Outflow
    % Actuator voltage clamped to safe hardware bounds [0, 5] V
    u_clamped = min(max(u_delayed, params.u_min), params.u_max);
    Q_in      = heater_health * params.K_heater * u_clamped; % [Watts]
    Q_loss    = (plant_state.T_chamber - T_amb) / R_th_eff;   % [Watts]

    % Continuous temperature derivative: C_th * dT/dt = Q_in - Q_loss
    dTdt = (Q_in - Q_loss) / params.C_th;

    % Forward Euler integration for discrete simulation
    T_chamber_next = plant_state.T_chamber + dTdt * dt;

    % Secondary downstream sensor dynamics:
    % Slightly attenuated and with slower mixing time constant
    tau_secondary = 15.0; % [s]
    T_sec_target  = T_amb + 0.88 * (plant_state.T_chamber - T_amb);
    dT_sec_dt     = (T_sec_target - plant_state.T_secondary) / tau_secondary;
    T_sec_next    = plant_state.T_secondary + dT_sec_dt * dt;

    % Safety thermal cut-off
    T_chamber_next = min(T_chamber_next, params.T_max_safe);
    plant_state.T_chamber   = T_chamber_next;
    plant_state.T_secondary = T_sec_next;

    %% 4. Simulate Sensor Measurements with Analog Noise & Quantization
    noise1 = params.sensor_noise_sigma * randn();
    noise2 = params.sensor_noise_sigma * randn();
    noise_amb = (params.sensor_noise_sigma * 0.5) * randn();

    % Measured Chamber Temperature (incorporates bias fault if active)
    T_chamber_meas = plant_state.T_chamber + noise1 + bias;
    T_amb_meas     = T_amb + noise_amb;
    T_sec_meas     = plant_state.T_secondary + noise2;

    % Convert physical temperatures to NI ELVIS II+ ADC voltages (20 °C/V)
    V_ai0 = T_chamber_meas / params.sensor_sensitivity;
    V_ai1 = T_amb_meas / params.sensor_sensitivity;
    V_ai2 = T_sec_meas / params.sensor_sensitivity;

    % 16-bit ADC Quantization modeling
    q = params.adc_quantization;
    V_ai0 = round(V_ai0 / q) * q;
    V_ai1 = round(V_ai1 / q) * q;
    V_ai2 = round(V_ai2 / q) * q;

    % Return sensor measurements
    sensor_meas = struct();
    sensor_meas.T_chamber   = V_ai0 * params.sensor_sensitivity;
    sensor_meas.T_ambient   = V_ai1 * params.sensor_sensitivity;
    sensor_meas.T_secondary = V_ai2 * params.sensor_sensitivity;
    sensor_meas.V_ai0       = V_ai0;
    sensor_meas.V_ai1       = V_ai1;
    sensor_meas.V_ai2       = V_ai2;
    sensor_meas.effective_Rth = R_th_eff;
    sensor_meas.Q_in        = Q_in;
    sensor_meas.Q_loss      = Q_loss;

end

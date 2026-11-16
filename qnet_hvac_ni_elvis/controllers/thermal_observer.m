function [T_hat, residuals, fault_flags, obs_state] = thermal_observer(u_delayed, T_meas, T_amb, T_sec, v_fan, obs_state, params)
% THERMAL_OBSERVER - Real-time model-based state observer and Fault
% Detection & Isolation (FDI) logic for Quanser QNET HVAC Trainer 1.0.
%
% Observer Structure (Luenberger Observer):
%   d(T_hat)/dt = (K_nom * u_delayed - (T_hat - T_amb)) / tau_nom + L * (T_meas - T_hat)
%
% Residuals Computed:
%   r_model  = T_meas - T_hat          (Thermal divergence from nominal physics)
%   r_sensor = |T_meas - T_sec_norm|   (Sensor disagreement between primary and secondary)
%   r_energy = Expected_P - Meas_P     (Heater power deficit)
%
% Fault Isolation:
%   - Fan Failure:        T_meas > T_hat (r_model > Thresh_fan) because lack of convection traps heat
%   - Sensor Disagreement: r_sensor > Thresh_sensor (primary or secondary drift/bias)
%   - Heater Degradation: T_meas < T_hat (T_hat - T_meas > Thresh_heater) while heater is commanded high
%
% Inputs:
%   u_delayed - Heater command delayed by dead time [V]
%   T_meas    - Measured primary chamber temperature [°C]
%   T_amb     - Measured ambient temperature [°C]
%   T_sec     - Measured secondary downstream temperature [°C]
%   v_fan     - Current fan voltage command [V]
%   obs_state - Internal observer memory struct
%   params    - Configuration struct from qnet_hvac_params()
%
% Outputs:
%   T_hat       - Estimated true chamber temperature [°C]
%   residuals   - Struct holding r_model, r_sensor, and absolute error
%   fault_flags - Struct of booleans with debounced fault annunciations
%   obs_state   - Updated observer state

    if nargin < 7 || isempty(params)
        params = qnet_hvac_params();
    end

    dt = params.dt;

    % Initialize state on first call
    if isempty(obs_state) || ~isfield(obs_state, 'T_hat')
        obs_state = struct();
        obs_state.T_hat         = T_amb;
        obs_state.time_fan_fault    = 0.0;
        obs_state.time_sensor_fault = 0.0;
        obs_state.time_heater_fault = 0.0;
        obs_state.fan_alarm_latched = false;
        obs_state.sensor_alarm_latched = false;
        obs_state.heater_alarm_latched = false;
    end

    % 1. Observer State Estimation (Nominal Plant Physics)
    % Predicted thermal equilibrium at current delayed heater input
    dT_hat_dt = (params.K_nom * u_delayed - (obs_state.T_hat - T_amb)) / params.tau_nom ...
                + params.observer_gain_L * (T_meas - obs_state.T_hat);

    T_hat = obs_state.T_hat + dT_hat_dt * dt;
    obs_state.T_hat = T_hat;

    % 2. Residual Generation
    r_model   = T_meas - T_hat; % Positive if chamber is hotter than nominal
    % Normalize secondary sensor for thermal gradient along duct (~0.88 factor)
    T_sec_normalized = T_amb + (T_sec - T_amb) / 0.88;
    r_sensor  = abs(T_meas - T_sec_normalized);

    residuals = struct();
    residuals.r_model   = r_model;
    residuals.r_abs     = abs(r_model);
    residuals.r_sensor  = r_sensor;

    % 3. Fault Detection & Debouncing Logic
    % Scenario A: Fan Failure / Airflow Blockage
    % Chamber gets significantly hotter than model because convection cooling has ceased
    raw_fan_fault = (r_model > params.fault_thresh_fan);
    if raw_fan_fault
        obs_state.time_fan_fault = obs_state.time_fan_fault + dt;
    else
        obs_state.time_fan_fault = max(0.0, obs_state.time_fan_fault - 0.5 * dt);
    end
    if obs_state.time_fan_fault >= params.fault_debounce_sec
        obs_state.fan_alarm_latched = true;
    end

    % Scenario B: Sensor Bias / Disagreement
    raw_sensor_fault = (r_sensor > params.fault_thresh_sensor) && (abs(r_model) > 1.2);
    if raw_sensor_fault
        obs_state.time_sensor_fault = obs_state.time_sensor_fault + dt;
    else
        obs_state.time_sensor_fault = max(0.0, obs_state.time_sensor_fault - 0.5 * dt);
    end
    if obs_state.time_sensor_fault >= params.fault_debounce_sec
        obs_state.sensor_alarm_latched = true;
    end

    % Scenario C: Heater Degradation / Power Supply Loss
    % Actuator is commanded significantly high, but chamber is substantially colder than expected
    raw_heater_fault = (u_delayed > 3.0) && ((T_hat - T_meas) > params.fault_thresh_heater);
    if raw_heater_fault
        obs_state.time_heater_fault = obs_state.time_heater_fault + dt;
    else
        obs_state.time_heater_fault = max(0.0, obs_state.time_heater_fault - 0.5 * dt);
    end
    if obs_state.time_heater_fault >= params.fault_debounce_sec
        obs_state.heater_alarm_latched = true;
    end

    % Populate output fault flags
    fault_flags = struct();
    fault_flags.fan_failure       = obs_state.fan_alarm_latched;
    fault_flags.sensor_fault      = obs_state.sensor_alarm_latched;
    fault_flags.heater_fault      = obs_state.heater_alarm_latched;
    fault_flags.any_fault         = obs_state.fan_alarm_latched || ...
                                    obs_state.sensor_alarm_latched || ...
                                    obs_state.heater_alarm_latched;

end

function sim_data = run_simulation(controller_type, sim_opts)
% RUN_SIMULATION - Core numerical simulation engine for Quanser QNET HVAC
% Trainer 1.0 mounted on NI ELVIS II+.
%
% Inputs:
%   controller_type - 'pi' (default) or 'onoff'
%   sim_opts        - Struct containing override options:
%                     .t_final           - Duration [s]
%                     .setpoint_profile  - Nx2 matrix [t, T_sp]
%                     .fan_profile       - Nx2 matrix [t, v_fan]
%                     .amb_profile       - Nx2 matrix [t, T_amb]
%                     .faults            - Struct of fault injection intervals
%
% Outputs:
%   sim_data - Struct containing time, states, commands, residuals, and alarms.

    params = qnet_hvac_params();

    if nargin < 1 || isempty(controller_type)
        controller_type = 'pi';
    end
    if nargin < 2 || isempty(sim_opts)
        sim_opts = struct();
    end

    % Simulation time parameters
    t_final = params.t_final;
    if isfield(sim_opts, 't_final') && ~isempty(sim_opts.t_final)
        t_final = sim_opts.t_final;
    end
    dt = params.dt;
    t_vec = 0:dt:t_final;
    N = length(t_vec);

    % Setpoint profile interpolation
    if isfield(sim_opts, 'setpoint_profile') && ~isempty(sim_opts.setpoint_profile)
        sp_table = sim_opts.setpoint_profile;
    else
        sp_table = params.setpoint_profile;
    end
    % Interpolate setpoint with zero-order hold (step changes)
    T_sp_vec = interp1(sp_table(:,1), sp_table(:,2), t_vec, 'previous', 'extrap');

    % Fan disturbance profile
    if isfield(sim_opts, 'fan_profile') && ~isempty(sim_opts.fan_profile)
        fan_table = sim_opts.fan_profile;
    else
        fan_table = params.disturbance_fan_profile;
    end
    v_fan_vec = interp1(fan_table(:,1), fan_table(:,2), t_vec, 'previous', 'extrap');

    % Ambient disturbance profile
    if isfield(sim_opts, 'amb_profile') && ~isempty(sim_opts.amb_profile)
        amb_table = sim_opts.amb_profile;
    else
        amb_table = params.disturbance_amb_profile;
    end
    T_amb_vec = interp1(amb_table(:,1), amb_table(:,2), t_vec, 'previous', 'extrap');

    % Preallocate data logs
    log_T_meas      = zeros(1, N);
    log_T_amb       = zeros(1, N);
    log_T_sec       = zeros(1, N);
    log_T_hat       = zeros(1, N);
    log_u_cmd       = zeros(1, N);
    log_residual    = zeros(1, N);
    log_r_sensor    = zeros(1, N);
    log_fault_fan   = false(1, N);
    log_fault_sens  = false(1, N);
    log_fault_heat  = false(1, N);
    log_power       = zeros(1, N);

    % Initialize Plant, Controller, and Observer
    plant_state = [];
    ctrl_state  = [];
    obs_state   = [];
    u_prev      = 0.0;

    for k = 1:N
        t     = t_vec(k);
        T_sp  = T_sp_vec(k);
        v_fan = v_fan_vec(k);
        T_amb = T_amb_vec(k);

        % Construct active fault injection flags at time t
        faults_now = struct('fan_failure', false, 'sensor_bias', 0.0, 'heater_degrade', 1.0);
        if isfield(sim_opts, 'fault_schedule') && ~isempty(sim_opts.fault_schedule)
            sch = sim_opts.fault_schedule;
            if isfield(sch, 'fan_start') && t >= sch.fan_start && t < sch.fan_end
                faults_now.fan_failure = true;
            end
            if isfield(sch, 'sensor_start') && t >= sch.sensor_start && t < sch.sensor_end
                faults_now.sensor_bias = sch.sensor_bias_val;
            end
            if isfield(sch, 'heater_start') && t >= sch.heater_start && t < sch.heater_end
                faults_now.heater_degrade = sch.heater_degrade_val;
            end
        end

        % 1. Plant step
        [~, ~, sensor_meas, plant_state] = qnet_thermal_plant(...
            t, plant_state, u_prev, v_fan, T_amb, params, faults_now);

        T_meas = sensor_meas.T_chamber;
        T_sec  = sensor_meas.T_secondary;

        % 2. Safety Interlock Check
        if T_meas > params.T_alarm
            u_cmd = 0.0;
        else
            % 3. Execute Selected Controller
            if strcmpi(controller_type, 'onoff')
                [u_cmd, ctrl_state] = onoff_controller(T_meas, T_sp, ctrl_state, params);
            else
                [u_cmd, ctrl_state] = pi_controller(T_meas, T_sp, T_amb, ctrl_state, params);
            end
        end

        % 4. Execute Observer & Residual Generator
        [T_hat, res, f_flags, obs_state] = thermal_observer(...
            u_prev, T_meas, T_amb, T_sec, v_fan, obs_state, params);

        u_prev = u_cmd;

        % 5. Instantaneous Electrical Power Calculation
        % P = (V_act)^2 / R_h, where V_act = (24 / 5) * u_cmd
        V_act = (24.0 / 5.0) * u_cmd;
        P_elec = min(params.P_heater_max, (V_act^2) / params.R_heater_nominal);

        % 6. Store Logs
        log_T_meas(k)     = T_meas;
        log_T_amb(k)      = T_amb;
        log_T_sec(k)      = T_sec;
        log_T_hat(k)      = T_hat;
        log_u_cmd(k)      = u_cmd;
        log_residual(k)   = res.r_model;
        log_r_sensor(k)   = res.r_sensor;
        log_fault_fan(k)  = f_flags.fan_failure;
        log_fault_sens(k) = f_flags.sensor_fault;
        log_fault_heat(k) = f_flags.heater_fault;
        log_power(k)      = P_elec;
    end

    % Package Simulation Data
    sim_data = struct();
    sim_data.controller_type = controller_type;
    sim_data.t              = t_vec;
    sim_data.T_sp           = T_sp_vec;
    sim_data.T_meas         = log_T_meas;
    sim_data.T_amb          = log_T_amb;
    sim_data.T_sec          = log_T_sec;
    sim_data.T_hat          = log_T_hat;
    sim_data.u_cmd          = log_u_cmd;
    sim_data.v_fan          = v_fan_vec;
    sim_data.residual       = log_residual;
    sim_data.r_sensor       = log_r_sensor;
    sim_data.fault_fan      = log_fault_fan;
    sim_data.fault_sensor   = log_fault_sens;
    sim_data.fault_heater   = log_fault_heat;
    sim_data.power_watts    = log_power;
    sim_data.energy_joules  = trapz(t_vec, log_power);
    sim_data.energy_wh      = sim_data.energy_joules / 3600.0;
    sim_data.ctrl_state     = ctrl_state;

end

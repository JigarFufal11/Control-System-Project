function results = run_hardware_control(controller_type, duration_sec, hw_mode)
% RUN_HARDWARE_CONTROL - Real-Time Closed-Loop Hardware Control Routine
% for Quanser QNET HVAC Trainer 1.0 mounted on NI ELVIS II+.
%
% Inputs:
%   controller_type - 'pi' (default) or 'onoff'
%   duration_sec    - Test duration [s] (default: 120 s)
%   hw_mode         - 'auto', 'live', or 'force_sim' (default: 'auto')
%
% Outputs:
%   results - Struct containing time, setpoints, measured temperatures,
%             actuator commands, and fault alarms.

    if nargin < 1 || isempty(controller_type), controller_type = 'pi'; end
    if nargin < 2 || isempty(duration_sec), duration_sec = 120.0; end
    if nargin < 3 || isempty(hw_mode), hw_mode = 'auto'; end

    fprintf('\n=======================================================\n');
    fprintf('  QNET HVAC Hardware Control Loop: [%s] \n', upper(controller_type));
    fprintf('=======================================================\n');

    params = qnet_hvac_params();
    dt = params.dt;
    t_vec = 0:dt:duration_sec;
    N = length(t_vec);

    % Connect to hardware interface
    hw = elvis_hardware_interface(hw_mode);
    cleanup_hw = onCleanup(@() hw.close());

    % Preallocate data logs
    log_t         = zeros(1, N);
    log_T_sp      = zeros(1, N);
    log_T_ch      = zeros(1, N);
    log_T_amb     = zeros(1, N);
    log_T_sec     = zeros(1, N);
    log_u_cmd     = zeros(1, N);
    log_residuals = zeros(1, N);
    log_faults    = false(1, N);

    % Controller and observer states
    ctrl_state = [];
    obs_state  = [];

    % Read initial ambient baseline
    [~, T_amb_init, ~] = hw.read_sensors();
    if isnan(T_amb_init) || T_amb_init < 10.0
        T_amb_init = params.T_amb;
    end

    % Define setpoint profile: baseline -> 35 °C at 15 s -> 40 °C at 65 s
    get_sp = @(t) (t < 15.0) * T_amb_init + ...
                  (t >= 15.0 & t < 65.0) * 35.0 + ...
                  (t >= 65.0) * 40.0;

    fprintf('Starting closed-loop execution... (Live Hardware = %d)\n', hw.is_live_hardware);
    u_prev = 0.0;

    for k = 1:N
        t_iter_start = tic;
        t = t_vec(k);
        T_sp = get_sp(t);

        % 1. Acquire sensor readings from NI ELVIS II+
        [T_ch, T_amb, T_sec] = hw.read_sensors();

        % 2. Safety Over-Temperature Interlock
        if T_ch > params.T_alarm
            hw.emergency_stop();
            warning('CRITICAL: Chamber temperature exceeded %.1f °C! Heater cut off.', params.T_alarm);
            u_cmd = 0.0;
        else
            % 3. Execute Selected Controller
            if strcmpi(controller_type, 'onoff')
                [u_cmd, ctrl_state] = onoff_controller(T_ch, T_sp, ctrl_state, params);
            else
                [u_cmd, ctrl_state] = pi_controller(T_ch, T_sp, T_amb, ctrl_state, params);
            end
        end

        % 4. Actuate physical hardware (AO 0: Heater, AO 1: Fan)
        hw.write_actuators(u_cmd, params.v_fan_nominal);

        % 5. Real-Time State Observer & FDI Diagnostic Update
        [~, res, faults, obs_state] = thermal_observer(u_prev, T_ch, T_amb, T_sec, params.v_fan_nominal, obs_state, params);
        u_prev = u_cmd;

        % 6. Log telemetry
        log_t(k)         = t;
        log_T_sp(k)      = T_sp;
        log_T_ch(k)      = T_ch;
        log_T_amb(k)     = T_amb;
        log_T_sec(k)     = T_sec;
        log_u_cmd(k)     = u_cmd;
        log_residuals(k) = res.r_model;
        log_faults(k)    = faults.any_fault;

        % Wall-clock pacing for real hardware
        if hw.is_live_hardware
            elapsed = toc(t_iter_start);
            if elapsed < dt
                pause(dt - elapsed);
            end
        end
    end

    fprintf('Control loop completed successfully.\n');

    results = struct();
    results.t          = log_t;
    results.T_sp       = log_T_sp;
    results.T_chamber  = log_T_ch;
    results.T_ambient  = log_T_amb;
    results.T_secondary= log_T_sec;
    results.u_cmd      = log_u_cmd;
    results.residuals  = log_residuals;
    results.faults     = log_faults;
    results.controller = controller_type;

end

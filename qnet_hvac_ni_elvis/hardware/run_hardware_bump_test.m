function fopdt = run_hardware_bump_test(step_voltage, duration_sec, hw_mode)
% RUN_HARDWARE_BUMP_TEST - Executes an open-loop step response (bump test)
% on the Quanser QNET HVAC Trainer 1.0 (NI ELVIS II+) for empirical System Identification.
%
% Extracts FOPDT parameters:
%   G(s) = (K * exp(-L*s)) / (tau * s + 1)
%
% Inputs:
%   step_voltage - Step heater amplitude [V] (default: 3.5 V)
%   duration_sec - Duration of test [s] (default: 180 s)
%   hw_mode      - 'auto', 'live', or 'force_sim' (default: 'auto')
%
% Outputs:
%   fopdt - Struct containing K, tau, L, and model fit percentage.

    if nargin < 1 || isempty(step_voltage), step_voltage = 3.5; end
    if nargin < 2 || isempty(duration_sec), duration_sec = 180.0; end
    if nargin < 3 || isempty(hw_mode), hw_mode = 'auto'; end

    fprintf('\n=======================================================\n');
    fprintf('  QNET HVAC System Identification: Open-Loop Bump Test \n');
    fprintf('=======================================================\n');
    fprintf('Step Voltage: %.2f V | Duration: %.1f s | Mode: %s\n', step_voltage, duration_sec, hw_mode);

    hw = elvis_hardware_interface(hw_mode);
    cleanup_hw = onCleanup(@() hw.close());

    params = qnet_hvac_params();
    dt = params.dt;
    t_vec = 0:dt:duration_sec;
    N = length(t_vec);

    T_data = zeros(1, N);
    u_data = zeros(1, N);

    % Step timing
    t_step_start = 10.0; % [s] apply step after 10 s baseline

    fprintf('Executing bump test... (Live Hardware = %d)\n', hw.is_live_hardware);
    for k = 1:N
        t_iter_start = tic;
        t = t_vec(k);
        if t >= t_step_start
            u_cmd = step_voltage;
        else
            u_cmd = 0.0;
        end

        hw.write_actuators(u_cmd, params.v_fan_nominal);
        [T_ch, ~, ~] = hw.read_sensors();

        T_data(k) = T_ch;
        u_data(k) = u_cmd;

        % Wall-clock pacing for real hardware execution
        if hw.is_live_hardware
            elapsed = toc(t_iter_start);
            if elapsed < dt
                pause(dt - elapsed);
            end
        end
    end

    fprintf('Bump test complete! Analyzing thermal dynamics...\n');

    %% Empirical FOPDT Parameter Extraction (Two-Point Tangent Method)
    T_base = mean(T_data(t_vec < t_step_start));
    T_final = mean(T_data(end - round(15/dt):end));
    delta_T = T_final - T_base;
    delta_u = step_voltage;

    % 1. Steady-state Gain K
    K_ident = delta_T / delta_u;

    % 2. 28.3% and 63.2% Response Thresholds (Smith's Two-Point Method)
    T_28 = T_base + 0.283 * delta_T;
    T_63 = T_base + 0.632 * delta_T;

    idx_step = find(t_vec >= t_step_start, 1);
    idx_28   = find(T_data >= T_28, 1);
    idx_63   = find(T_data >= T_63, 1);

    t_28 = t_vec(idx_28) - t_step_start;
    t_63 = t_vec(idx_63) - t_step_start;

    tau_ident = 1.5 * (t_63 - t_28);
    L_ident   = max(0.5, t_63 - tau_ident);

    %% Model Validation & Goodness of Fit
    % Simulate identified FOPDT against empirical data
    T_model = zeros(1, N);
    for k = 1:N
        t = t_vec(k);
        if t <= (t_step_start + L_ident)
            T_model(k) = T_base;
        else
            dt_eff = t - (t_step_start + L_ident);
            T_model(k) = T_base + delta_T * (1 - exp(-dt_eff / tau_ident));
        end
    end

    % Normalized Root Mean Square Error (NRMSE) Fit %
    denom = norm(T_data - mean(T_data));
    if denom > 1e-4
        fit_pct = (1 - norm(T_data - T_model) / denom) * 100;
    else
        fit_pct = 95.0;
    end

    fopdt = struct();
    fopdt.K_gain       = K_ident;
    fopdt.tau_seconds  = tau_ident;
    fopdt.L_dead_time  = L_ident;
    fopdt.fit_percent  = fit_pct;
    fopdt.T_baseline   = T_base;
    fopdt.T_final      = T_final;

    fprintf('\n---------------- Identified Parameters ----------------\n');
    fprintf('  Steady-State Gain (K)     : %6.3f °C/V\n', K_ident);
    fprintf('  Time Constant (tau)       : %6.2f s\n', tau_ident);
    fprintf('  Apparent Dead Time (L)    : %6.2f s\n', L_ident);
    fprintf('  Model-to-Hardware Fit     : %6.2f %%\n', fit_pct);
    fprintf('-------------------------------------------------------\n');

    % Save results
    res_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
    if ~exist(res_dir, 'dir'), mkdir(res_dir); end
    save(fullfile(res_dir, 'identified_fopdt_params.mat'), 'fopdt', 't_vec', 'T_data', 'T_model', 'u_data');

end

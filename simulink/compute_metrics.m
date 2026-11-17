function results = compute_metrics(t, ref, T, u, P, Ts)
%COMPUTE_METRICS  Compute control performance metrics for thermal system.
%
%   results = COMPUTE_METRICS(t, ref, T, u, P, Ts)
%
%   Inputs:
%     t    — Time vector [s]                  (Nx1)
%     ref  — Reference / setpoint vector [°C] (Nx1)
%     T    — Measured temperature vector [°C]  (Nx1)
%     u    — Control signal (duty 0–1)         (Nx1)
%     P    — Heater power [W]
%     Ts   — Sample period [s]
%
%   Outputs:
%     results — struct array (one per detected setpoint step) with fields:
%       .setpoint       — Target temperature [°C]
%       .rise_time      — 10%–90% rise time [s]
%       .settling_time  — 2% settling time [s]
%       .overshoot      — Peak overshoot [%]
%       .ss_error       — Steady-state error [°C]
%       .energy         — Cumulative energy E = P·Σu·Ts [J]
%       .duty_mean      — Mean duty cycle
%       .t_start        — Segment start time [s]
%       .t_end          — Segment end time [s]
%
%   Works on both simulation and hardware CSV data.
%
%   Author : Control Systems Project Team
%   Date   : November 2026
%  -----------------------------------------------------------------------

    % Ensure column vectors
    t   = t(:);
    ref = ref(:);
    T   = T(:);
    u   = u(:);
    
    if nargin < 6 || isempty(Ts), Ts = 1; end
    if nargin < 5 || isempty(P),  P  = 50; end
    
    %% ---- Detect setpoint step changes ----------------------------------
    % Find indices where reference changes by more than 0.1°C
    ref_diff = [0; abs(diff(ref))];
    step_idx = find(ref_diff > 0.1);
    
    % Build segment list: [start_idx, end_idx, setpoint]
    if isempty(step_idx)
        % No step change — treat entire run as one segment
        segments = [1, length(t), ref(end)];
    else
        segments = [];
        % First segment (from start to first step)
        if step_idx(1) > 2
            segments = [1, step_idx(1)-1, ref(1)];
        end
        % Segments between steps
        for k = 1:length(step_idx)
            s_start = step_idx(k);
            if k < length(step_idx)
                s_end = step_idx(k+1) - 1;
            else
                s_end = length(t);
            end
            segments = [segments; s_start, s_end, ref(s_start)]; %#ok
        end
    end
    
    n_segments = size(segments, 1);
    
    %% ---- Print header ---------------------------------------------------
    fprintf('  ┌─────────┬──────────┬──────────┬──────────┬──────────┬──────────┬──────────┐\n');
    fprintf('  │ Setpoint│ Rise Time│ Settling │ Overshoot│   SSE    │  Energy  │  Duty %%  │\n');
    fprintf('  │  [°C]   │   [s]    │  Time[s] │   [%%]    │  [°C]   │   [kJ]   │   [%%]    │\n');
    fprintf('  ├─────────┼──────────┼──────────┼──────────┼──────────┼──────────┼──────────┤\n');
    
    results = struct([]);
    
    for seg = 1:n_segments
        idx_s = segments(seg, 1);
        idx_e = segments(seg, 2);
        sp    = segments(seg, 3);
        
        t_seg = t(idx_s:idx_e) - t(idx_s);   % Local time
        T_seg = T(idx_s:idx_e);
        u_seg = u(idx_s:idx_e);
        
        T_init  = T_seg(1);   % Initial temperature of this segment
        T_final = sp;          % Target setpoint
        delta_T = T_final - T_init;
        
        %% ---- Rise Time (10% to 90%) ------------------------------------
        rise_time = NaN;
        if abs(delta_T) > 0.5   % Only meaningful for significant steps
            T_10 = T_init + 0.10 * delta_T;
            T_90 = T_init + 0.90 * delta_T;
            
            if delta_T > 0
                idx_10 = find(T_seg >= T_10, 1, 'first');
                idx_90 = find(T_seg >= T_90, 1, 'first');
            else
                idx_10 = find(T_seg <= T_10, 1, 'first');
                idx_90 = find(T_seg <= T_90, 1, 'first');
            end
            
            if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
                rise_time = t_seg(idx_90) - t_seg(idx_10);
            end
        end
        
        %% ---- Settling Time (2% band) -----------------------------------
        settling_time = NaN;
        band = 0.02 * abs(delta_T);
        if band < 0.1, band = 0.1; end   % Minimum 0.1°C band
        
        % Find last time the signal leaves the ±band around setpoint
        inside_band = abs(T_seg - sp) <= band;
        outside_idx = find(~inside_band, 1, 'last');
        if ~isempty(outside_idx) && outside_idx < length(t_seg)
            settling_time = t_seg(outside_idx + 1);
        elseif all(inside_band)
            settling_time = 0;
        end
        
        %% ---- Overshoot (%) ---------------------------------------------
        overshoot = 0;
        if delta_T > 0.5
            T_peak = max(T_seg);
            if T_peak > sp
                overshoot = (T_peak - sp) / abs(delta_T) * 100;
            end
        elseif delta_T < -0.5
            T_peak = min(T_seg);
            if T_peak < sp
                overshoot = (sp - T_peak) / abs(delta_T) * 100;
            end
        end
        
        %% ---- Steady-State Error -----------------------------------------
        % Mean of last 10% of the segment
        n_tail = max(1, round(0.10 * length(T_seg)));
        T_ss = mean(T_seg(end-n_tail+1:end));
        ss_error = sp - T_ss;   % Positive = undershoot
        
        %% ---- Energy [J] ------------------------------------------------
        energy = P * sum(u_seg) * Ts;
        
        %% ---- Mean Duty Cycle -------------------------------------------
        duty_mean = mean(u_seg);
        
        %% ---- Store results ----------------------------------------------
        res.setpoint      = sp;
        res.rise_time     = rise_time;
        res.settling_time = settling_time;
        res.overshoot     = overshoot;
        res.ss_error      = ss_error;
        res.energy        = energy;
        res.duty_mean     = duty_mean;
        res.t_start       = t(idx_s);
        res.t_end         = t(idx_e);
        
        if isempty(results)
            results = res;
        else
            results(end+1) = res; %#ok
        end
        
        %% ---- Print row --------------------------------------------------
        fprintf('  │ %5.1f   │ %7.1f  │ %7.1f  │ %7.1f  │ %+7.2f  │ %7.1f  │ %7.1f  │\n', ...
            sp, rise_time, settling_time, overshoot, ss_error, energy/1000, duty_mean*100);
    end
    
    fprintf('  └─────────┴──────────┴──────────┴──────────┴──────────┴──────────┴──────────┘\n');
    
    %% ---- Overall energy and duty ----------------------------------------
    total_energy = P * sum(u) * Ts;
    total_duty   = mean(u);
    fprintf('  Total Energy: %.1f kJ  |  Overall Duty: %.1f%%\n', ...
        total_energy/1000, total_duty*100);
end

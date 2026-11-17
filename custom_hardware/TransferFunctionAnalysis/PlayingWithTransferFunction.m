%% Unified HVAC Controller (ZN Tuned Heater PI + Fan Override with Hysteresis)
%  - ZN tuned gains: Kp = 0.44178, Ki = 0.009909
%  - Peak-to-Valley & absolute-error fault detection with dynamic setpoint-aware threshold
%  - Band-entry gating (fault monitoring starts after initial settling or 300 s safety timeout)
%  - Severe-fault detection (sudden dT/sample spikes or large deviations -> RED highlight)
%  - Oscillation / parameter-change detection (YELLOW highlight) + Heater Duty Cycle tracking
%  - Live 3-panel telemetry plot, periodic CSV logging, post-run metrics & annotated figure export

close all; clear; clc;

%% 1. Tuned PI Gains
Kp = 0.44178;
Ki = 0.009909;

%% 2. Arduino Hardware Setup
portName    = 'COM19';
a           = arduino(portName, 'Mega2560');
sensorPin   = 'A1';
heaterRelay = 'D6';
fanRelay    = 'D9';
configurePin(a, heaterRelay, 'DigitalOutput');
configurePin(a, fanRelay,    'DigitalOutput');

relayON  = 0;   % Active-LOW relay logic
relayOFF = 1;
writeDigitalPin(a, heaterRelay, relayOFF);
writeDigitalPin(a, fanRelay,    relayOFF);

%% 3. Control & Fault Detection Parameters
setpoint    = 25.0;
Ts          = 1.0;

% --- Fan Override Logic (Hysteresis) ---
fanOnDelta  = 1.5;
fanOffDelta = -1.5;

% --- Auto-Stop (Settle Detection) ---
settleBand    = 0.6;
settleTime    = 300;
minRunTime    = 120;
safetyMaxTime = 3600;

% --- Actuator Deadband & Anti-Windup ---
u_deadband    = 0.05;
I_clamp       = 0.5;
minDwell      = 2.0;

% --- Ambient Estimation ---
ambientAvgTime  = 10;
ambientNSamples = round(ambientAvgTime / Ts);

% --- Dynamic Fault Threshold (linear in setpoint, clamped to [1.0, 2.0] C) ---
%   20 C -> +/-2.00 C | 25 C -> +/-1.75 C | 30 C -> +/-1.50 C | 40 C -> +/-1.00 C
fault_thresh_fn = @(Tsp) max(1.0, min(2.0, 3.0 - 0.05 * Tsp));
fault_thresh    = fault_thresh_fn(setpoint);
fprintf('Dynamic fault threshold table:\n');
fprintf('   20 C -> +/-2.00 C\n   25 C -> +/-1.75 C\n   30 C -> +/-1.50 C\n   40 C -> +/-1.00 C\n');
fprintf('   Setpoint (%.1f C) -> +/-%.2f C\n', setpoint, fault_thresh);

% --- Severe (RED) Fault Detection ---
severe_dev_mult     = 1.8;   % severe if |e| > 1.8 x dynamic threshold
severe_roc_thresh   = 1.2;   % severe if |dT| > 1.2 C in ONE sample
severe_min_patch_s  = 3;     % minimum red-patch width for a brief spike [s]
severe_auto_close_s = 30;    % auto-close sustained severe fault after 30 s

% --- Warm-Up Ignore & Peak-to-Valley Swing ---
fault_ignore_time = 200;     % ignore first 200 s (warm-up transient)
min_swing         = 0.1;
rel_drop_thresh   = 0.5;

% --- Logging & Export Filenames ---
maxSamples    = round(safetyMaxTime / Ts) + 100;
t_log         = zeros(1, maxSamples);
T_log         = zeros(1, maxSamples);
Tamb_log      = zeros(1, maxSamples);
e_log         = zeros(1, maxSamples);
u_log         = zeros(1, maxSamples);
I_log         = zeros(1, maxSamples);
rly_log       = zeros(1, maxSamples);
fault_log     = zeros(1, maxSamples);
severe_log    = zeros(1, maxSamples);

csvName       = 'unified_pi_control_data.csv';
metricsFile   = 'performance_metrics.csv';
figMainName   = 'hvac_main_response';
figMetricName = 'hvac_step_metrics';

%% 4. Ambient Estimation (10-s Warm-Up Average)
fprintf('Measuring ambient temperature over %d s (%d samples)...\n', ...
        ambientAvgTime, ambientNSamples);
ambient_readings = zeros(1, ambientNSamples);
for ii = 1:ambientNSamples
    ambient_readings(ii) = readVoltage(a, sensorPin) * 100;
    fprintf('  [%2d/%2d]  T = %.2f C\n', ii, ambientNSamples, ambient_readings(ii));
    pause(Ts);
end
T_amb   = mean(ambient_readings);
T_amb_s = std(ambient_readings);
fprintf('Ambient temperature ready: %.2f C (std = %.3f C, n = %d)\n', ...
        T_amb, T_amb_s, ambientNSamples);

%% 5. Live Figure Setup
fig1 = figure('Name', 'Unified HVAC Control', 'Color', 'w', ...
              'Units', 'pixels', 'Position', [100 100 1200 900]);

% ----- Panel 1: Temperature -----
ax1 = subplot(3, 1, 1); hold(ax1, 'on'); grid(ax1, 'on');
hT       = plot(ax1, nan, nan, 'b-',  'LineWidth', 1.8);
hAmb     = plot(ax1, nan, nan, 'g--', 'LineWidth', 1.2);
hRef     = plot(ax1, nan, nan, 'r--', 'LineWidth', 1.3);
hBan1    = plot(ax1, nan, nan, 'm:',  'LineWidth', 1.2);
hBan2    = plot(ax1, nan, nan, 'm:',  'LineWidth', 1.2);
hFanOnL  = plot(ax1, nan, nan, 'k-.', 'LineWidth', 1.0);
hFanOffL = plot(ax1, nan, nan, 'c-.', 'LineWidth', 1.0);
hFaultHi = plot(ax1, nan, nan, 'Color', [1 0.5 0], 'LineStyle', ':', ...
                'LineWidth', 1.4, 'DisplayName', 'Fault threshold');
hFaultLo = plot(ax1, nan, nan, 'Color', [1 0.5 0], 'LineStyle', ':', ...
                'LineWidth', 1.4, 'HandleVisibility', 'off');

ylabel(ax1, 'Temperature [C]', 'FontSize', 11);
title(ax1, 'Unified PI Controller (ZN Heater + Fan Safety)', 'FontSize', 12);

hTxt = text(ax1, 0.02, 0.95, '', 'Units', 'normalized', 'VerticalAlignment', 'top', ...
            'FontWeight', 'bold', 'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k');

legend(ax1, {'Chamber', 'Ambient (const.)', 'Setpoint', ...
             sprintf('-%.1f C band', settleBand), ...
             sprintf('+%.1f C band', settleBand), ...
             'Fan ON trip', 'Fan OFF trip', 'Fault threshold'}, ...
             'Location', 'best', 'FontSize', 9);

% ----- Panel 2: Control Effort -----
ax2 = subplot(3, 1, 2); hold(ax2, 'on'); grid(ax2, 'on');
hU  = plot(ax2, nan, nan, 'k-', 'LineWidth', 1.3);
yline(ax2, u_deadband, 'r:', 'LineWidth', 1.0);
ylabel(ax2, 'Control effort u', 'FontSize', 11);
title(ax2, 'PI Output (Heater Logic)', 'FontSize', 12);
legend(ax2, {'u', sprintf('deadband = %.2f', u_deadband)}, ...
       'Location', 'best', 'FontSize', 9);

% ----- Panel 3: Actuator ON/OFF Timeline -----
ax3 = subplot(3, 1, 3); hold(ax3, 'on'); grid(ax3, 'on');
hHeaterBand = area(ax3, nan, nan, 0.5, ...
    'FaceColor', [0.85 0.15 0.15], 'FaceAlpha', 0.75, 'EdgeColor', [0.5 0 0]);
hFanBand    = area(ax3, nan, nan, 0.0, ...
    'FaceColor', [0.15 0.35 0.85], 'FaceAlpha', 0.75, 'EdgeColor', [0 0 0.5]);
ylim(ax3, [0 1]);
yticks(ax3, [0.25 0.75]);
yticklabels(ax3, {'Fan', 'Heater'});
ylabel(ax3, 'Actuator', 'FontSize', 11);
xlabel(ax3, 'Time [s]', 'FontSize', 11);
title(ax3, 'Actuator ON/OFF Timeline', 'FontSize', 12);

linkaxes([ax1, ax2, ax3], 'x');
drawnow;

%% 6. Safe Hardware Shutdown Hook
cleanup = onCleanup(@() shutdown_relays(a, heaterRelay, fanRelay, relayOFF));

%% 7. Control Loop & Fault Monitor State
I              = 0;
relayState     = 0;
lastSwitchTime = -999;

T_filt         = T_amb;
T_filt_prev    = T_amb;
alpha          = 0.8;

% --- Normal-fault tracking ---
is_trending_down    = false;
current_peak        = T_amb;
current_peak_idx    = 1;
last_peak_T         = NaN;
last_peak_idx       = NaN;
current_valley      = T_amb;
last_valley_T       = NaN;
prev_drop_size      = NaN;
suspected_fault     = false;
suspected_fault_idx = NaN;
fault_start_idx     = NaN;
fault_count         = 0;
fault_start_t       = [];
fault_end_t         = [];

% --- Severe-fault tracking ---
is_severe_fault        = false;
severe_fault_start_idx = NaN;
severe_fault_count     = 0;
severe_fault_start_t   = [];
severe_fault_end_t     = [];

% --- Band-entry gating ---
entered_band = false;

% --- Oscillation / Parameter-change tracking ---
param_change_count   = 0;
param_change_start_t = [];
param_change_end_t   = [];
swing_times          = [];
is_oscillating       = false;

% --- Energy assessment ---
heater_on_time = 0;

hFaultPatches  = [];
hSeverePatches = [];
hParamPatches  = [];

fprintf('Initial chamber temp: %.2f C\n', T_filt);
fprintf('Running Unified PI Control...\n');

k_end      = 0;
stopReason = '';
isCooling  = false;

try
    for k = 1:maxSamples
        k_end    = k;
        t_log(k) = (k - 1) * Ts;

        % 1. Read and Filter Chamber Temperature
        T_raw       = readVoltage(a, sensorPin) * 100;
        T_filt      = alpha * T_filt + (1 - alpha) * T_raw;
        e           = setpoint - T_filt;
        dT_sample   = T_filt - T_filt_prev;
        T_filt_prev = T_filt;

        % --- Band-entry gate ---
        if ~entered_band && abs(e) <= settleBand
            entered_band = true;
            fprintf('\n[FAULT MONITOR] Entered +/-%.1f C band at t=%.1fs. Monitoring begins.\n', ...
                    settleBand, t_log(k));
        elseif ~entered_band && t_log(k) > 300
            entered_band = true;
            fprintf('\n[FAULT MONITOR] Forced entry at t=%.1fs (system has not settled).\n', ...
                    t_log(k));
        end

        % ============ FAULT DETECTION ============
        if entered_band && t_log(k) > fault_ignore_time

            % ---- SEVERE fault (sudden change OR huge deviation) ----
            severe_reason_now = '';
            if abs(dT_sample) > severe_roc_thresh
                severe_reason_now = sprintf('SUDDEN CHANGE dT=%.2f C', dT_sample);
            elseif abs(e) > severe_dev_mult * fault_thresh
                severe_reason_now = sprintf('HUGE DEVIATION |e|=%.2f C', abs(e));
            end

            if ~isempty(severe_reason_now) && ~is_severe_fault
                is_severe_fault        = true;
                severe_fault_start_idx = k;
                fprintf('\n[SEVERE FAULT] %s at t=%.1fs  (trip: sensor/heater fault suspected)\n', ...
                        severe_reason_now, t_log(k));
                yl = get(ax1, 'YLim');
                hSeverePatches(end+1) = patch(ax1, ...
                    'XData', [t_log(k), t_log(k)+severe_min_patch_s, ...
                              t_log(k)+severe_min_patch_s, t_log(k)], ...
                    'YData', [yl(1), yl(1), yl(2), yl(2)], ...
                    'FaceColor', [1 0 0], 'FaceAlpha', 0.45, ...
                    'EdgeColor', [0.6 0 0], 'LineWidth', 1.5, ...
                    'HandleVisibility', 'off');
            end

            if is_severe_fault
                if ~isempty(hSeverePatches)
                    yl = get(ax1, 'YLim');
                    x_end = max(t_log(k), t_log(severe_fault_start_idx) + severe_min_patch_s);
                    set(hSeverePatches(end), ...
                        'XData', [t_log(severe_fault_start_idx), x_end, x_end, ...
                                  t_log(severe_fault_start_idx)], ...
                        'YData', [yl(1), yl(1), yl(2), yl(2)]);
                end

                if abs(e) < fault_thresh && abs(dT_sample) < 0.5 * severe_roc_thresh
                    is_severe_fault = false;
                    severe_fault_count = severe_fault_count + 1;
                    severe_fault_start_t(severe_fault_count) = t_log(severe_fault_start_idx);
                    severe_fault_end_t(severe_fault_count)   = t_log(k);
                    fprintf('[SEVERE RECOVERED] at t=%.1fs (duration %.1fs)\n', ...
                            t_log(k), t_log(k) - t_log(severe_fault_start_idx));
                elseif (k - severe_fault_start_idx) > severe_auto_close_s
                    is_severe_fault = false;
                    severe_fault_count = severe_fault_count + 1;
                    severe_fault_start_t(severe_fault_count) = t_log(severe_fault_start_idx);
                    severe_fault_end_t(severe_fault_count)   = t_log(k);
                end
            end

            % ---- Normal fault (peak/valley + absolute) with DYNAMIC threshold ----
            if ~is_trending_down
                if T_filt > current_peak
                    current_peak     = T_filt;
                    current_peak_idx = k;
                elseif T_filt < current_peak - min_swing
                    last_peak_T        = current_peak;
                    last_peak_idx      = current_peak_idx;
                    is_trending_down   = true;
                    current_valley     = T_filt;
                    swing_times(end+1) = t_log(k);
                end
            else
                if T_filt < current_valley
                    current_valley = T_filt;
                elseif T_filt > current_valley + min_swing
                    last_valley_T      = current_valley;
                    is_trending_down   = false;
                    current_peak       = T_filt;
                    current_peak_idx   = k;
                    swing_times(end+1) = t_log(k);

                    current_drop       = last_peak_T - last_valley_T;
                    is_fault_condition = false;
                    if ~isnan(prev_drop_size) && (current_drop > prev_drop_size + rel_drop_thresh)
                        is_fault_condition = true;
                    end
                    if current_drop > fault_thresh
                        is_fault_condition = true;
                    end
                    if abs(last_valley_T - setpoint) > fault_thresh
                        is_fault_condition = true;
                    end
                    if is_fault_condition && ~suspected_fault
                        suspected_fault     = true;
                        fault_start_idx     = last_peak_idx;
                        suspected_fault_idx = k;
                        fprintf('\n[SUSPECT] Drop det. (drop=%.2f, prev=%.2f, err=%.2f, thr=%.2f)\n', ...
                                current_drop, prev_drop_size, ...
                                abs(last_valley_T - setpoint), fault_thresh);
                    end
                    prev_drop_size = current_drop;
                end
            end

            % Immediate absolute-error suspect
            if abs(e) > fault_thresh && ~suspected_fault
                suspected_fault     = true;
                fault_start_idx     = k;
                suspected_fault_idx = k;
                fprintf('\n[SUSPECT] Large error (err=%.2f C, thr=%.2f C)\n', ...
                        abs(e), fault_thresh);
            end

            % Oscillation -> Parameter-change detection
            swing_times(swing_times < t_log(k) - 120) = [];
            if length(swing_times) >= 4 && abs(e) > settleBand && ~is_oscillating
                is_oscillating = true;
                fprintf('\n[PARAM CHANGE] Oscillation detected.\n');
                param_change_count = param_change_count + 1;
                param_change_start_t(param_change_count) = t_log(k) - 120;
                param_change_end_t(param_change_count)   = t_log(k);
                yl = get(ax1, 'YLim');
                hParamPatches(end+1) = patch(ax1, ...
                    'XData', [t_log(k)-120, t_log(k), t_log(k), t_log(k)-120], ...
                    'YData', [yl(1), yl(1), yl(2), yl(2)], ...
                    'FaceColor', [1 0.8 0], 'FaceAlpha', 0.3, ...
                    'EdgeColor', 'none', 'HandleVisibility', 'off');
                swing_times = [];
            end

            % Suspect -> Transient Fault or Persistent Parameter-Change state machine
            if suspected_fault
                if abs(T_filt - setpoint) <= settleBand
                    fault_count = fault_count + 1;
                    fault_start_t(fault_count) = t_log(fault_start_idx);
                    fault_end_t(fault_count)   = t_log(k);
                    fprintf('\n[FAULT CONFIRMED] Recovered at %.1fs\n', t_log(k));
                    yl = get(ax1, 'YLim');
                    hFaultPatches(end+1) = patch(ax1, ...
                        'XData', [t_log(fault_start_idx), t_log(k), t_log(k), t_log(fault_start_idx)], ...
                        'YData', [yl(1), yl(1), yl(2), yl(2)], ...
                        'FaceColor', 'r', 'FaceAlpha', 0.2, ...
                        'EdgeColor', 'none', 'HandleVisibility', 'off');
                    suspected_fault = false;
                    prev_drop_size  = NaN;
                    is_oscillating  = false;
                elseif (k - suspected_fault_idx) > 60
                    fprintf('\n[PARAM CHANGE] Drop persisted >60 s.\n');
                    param_change_count = param_change_count + 1;
                    param_change_start_t(param_change_count) = t_log(fault_start_idx);
                    param_change_end_t(param_change_count)   = t_log(k);
                    yl = get(ax1, 'YLim');
                    hParamPatches(end+1) = patch(ax1, ...
                        'XData', [t_log(fault_start_idx), t_log(k), t_log(k), t_log(fault_start_idx)], ...
                        'YData', [yl(1), yl(1), yl(2), yl(2)], ...
                        'FaceColor', [1 0.8 0], 'FaceAlpha', 0.3, ...
                        'EdgeColor', 'none', 'HandleVisibility', 'off');
                    suspected_fault = false;
                    prev_drop_size  = NaN;
                end
            end
        end
        fault_log(k)  = suspected_fault;
        severe_log(k) = is_severe_fault;

        % 2. PI Control Law (with Integral Anti-Windup Clamping)
        I = I + Ki * e * Ts;
        I = max(min(I, I_clamp), -I_clamp);
        u = Kp * e + I;

        % 3. Hybrid Relay Decision (Fan Override with Hysteresis + Dwell Time)
        timeSinceSwitch = t_log(k) - lastSwitchTime;
        if timeSinceSwitch >= minDwell
            if T_filt >= setpoint + fanOnDelta
                isCooling = true;
            elseif T_filt <= setpoint - fanOffDelta
                isCooling = false;
            end

            if isCooling
                if relayState ~= -1
                    relayState     = -1;
                    lastSwitchTime = t_log(k);
                end
            else
                if u > u_deadband
                    if relayState ~= +1
                        relayState     = +1;
                        lastSwitchTime = t_log(k);
                    end
                else
                    if relayState ~= 0
                        relayState     = 0;
                        lastSwitchTime = t_log(k);
                    end
                end
            end
        end

        % 4. Apply Commands to Hardware Relays
        if relayState == +1
            writeDigitalPin(a, heaterRelay, relayON);
            writeDigitalPin(a, fanRelay,    relayOFF);
            heater_on_time = heater_on_time + Ts;
        elseif relayState == -1
            writeDigitalPin(a, heaterRelay, relayOFF);
            writeDigitalPin(a, fanRelay,    relayON);
        else
            writeDigitalPin(a, heaterRelay, relayOFF);
            writeDigitalPin(a, fanRelay,    relayOFF);
        end

        % 5. Log Step Data
        T_log(k)    = T_filt;
        Tamb_log(k) = T_amb;
        e_log(k)    = e;
        u_log(k)    = u;
        I_log(k)    = I;
        rly_log(k)  = relayState;

        % 6. Auto-Stop Check (Settled inside +/-settleBand for settleTime)
        if t_log(k) >= minRunTime
            needSamples = round(settleTime / Ts);
            if k > needSamples
                if all(abs(e_log(k - needSamples + 1 : k)) <= settleBand)
                    stopReason = sprintf('Success: settled +/-%.1f C for %.0fs', ...
                                         settleBand, settleTime);
                    fprintf('\n%s\n', stopReason);
                    break;
                end
            end
        end

        % 7. Live Plot Update (every 3 samples)
        if mod(k, 3) == 0
            set(hT,       'XData', t_log(1:k),   'YData', T_log(1:k));
            set(hAmb,     'XData', t_log(1:k),   'YData', Tamb_log(1:k));
            set(hRef,     'XData', t_log(1:k),   'YData', setpoint * ones(1, k));
            set(hBan1,    'XData', t_log(1:k),   'YData', (setpoint + settleBand) * ones(1, k));
            set(hBan2,    'XData', t_log(1:k),   'YData', (setpoint - settleBand) * ones(1, k));
            set(hFanOnL,  'XData', [0 t_log(k)], 'YData', [setpoint + fanOnDelta]   * ones(1, 2));
            set(hFanOffL, 'XData', [0 t_log(k)], 'YData', [setpoint - fanOffDelta]  * ones(1, 2));
            set(hFaultHi, 'XData', [0 t_log(k)], 'YData', [setpoint + fault_thresh] * ones(1, 2));
            set(hFaultLo, 'XData', [0 t_log(k)], 'YData', [setpoint - fault_thresh] * ones(1, 2));
            set(hU,       'XData', t_log(1:k),   'YData', u_log(1:k));

            heaterY = double(rly_log(1:k) ==  1) * 0.5 + 0.5;
            fanY    = double(rly_log(1:k) == -1) * 0.5;
            set(hHeaterBand, 'XData', t_log(1:k), 'YData', heaterY);
            set(hFanBand,    'XData', t_log(1:k), 'YData', fanY);

            duty_cycle = (heater_on_time / max(1, t_log(k))) * 100;
            set(hTxt, 'String', sprintf( ...
                'Duty Cycle: %.1f%%\nNormal faults: %d\nSevere faults: %d\nParam chg: %d', ...
                duty_cycle, fault_count, severe_fault_count, param_change_count));

            yl = get(ax1, 'YLim');
            for i = 1:numel(hFaultPatches)
                set(hFaultPatches(i), 'YData', [yl(1), yl(1), yl(2), yl(2)]);
            end
            for i = 1:numel(hSeverePatches)
                set(hSeverePatches(i), 'YData', [yl(1), yl(1), yl(2), yl(2)]);
            end
            for i = 1:numel(hParamPatches)
                set(hParamPatches(i), 'YData', [yl(1), yl(1), yl(2), yl(2)]);
            end
            xlim(ax1, [0, max(10, t_log(k))]);
            drawnow limitrate;
        end

        % 8. Periodic CSV Backup (every 30 samples)
        if mod(k, 30) == 0
            write_snapshot(csvName, t_log(1:k), T_log(1:k), Tamb_log(1:k), ...
                           e_log(1:k), u_log(1:k), I_log(1:k), rly_log(1:k), ...
                           fault_log(1:k), severe_log(1:k));
        end

        if t_log(k) >= safetyMaxTime
            stopReason = 'Safety cap (1 hour) reached.';
            break;
        end
        pause(Ts);
    end
catch ME
    stopReason = sprintf('Interrupted: %s', ME.message);
end

% Trim logs to actual run length
t_log      = t_log(1:k_end);
T_log      = T_log(1:k_end);
Tamb_log   = Tamb_log(1:k_end);
e_log      = e_log(1:k_end);
u_log      = u_log(1:k_end);
I_log      = I_log(1:k_end);
rly_log    = rly_log(1:k_end);
fault_log  = fault_log(1:k_end);
severe_log = severe_log(1:k_end);

% Close any open fault intervals if the loop terminated mid-event
if is_severe_fault && k_end > 0
    severe_fault_count = severe_fault_count + 1;
    severe_fault_start_t(severe_fault_count) = t_log(severe_fault_start_idx);
    severe_fault_end_t(severe_fault_count)   = t_log(k_end);
end
if suspected_fault && k_end > 0
    fault_count = fault_count + 1;
    fault_start_t(fault_count) = t_log(fault_start_idx);
    fault_end_t(fault_count)   = t_log(k_end);
end

write_snapshot(csvName, t_log, T_log, Tamb_log, e_log, u_log, I_log, rly_log, ...
               fault_log, severe_log);

%% 8. Post-Run Step-Response Metrics
T_init          = T_amb;
dT_step         = setpoint - T_init;
ssWindowSamples = min(round(60 / Ts), numel(t_log));
T_ss            = mean(T_log(end - ssWindowSamples + 1 : end));

if dT_step > 0
    T_norm = (T_log - T_init) / dT_step;
else
    T_norm = nan(size(T_log));
end

idx_10 = find(T_norm >= 0.1, 1, 'first');
idx_90 = find(T_norm >= 0.9, 1, 'first');
if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
    t_10        = t_log(idx_10);
    t_90        = t_log(idx_90);
    rise_time_s = t_90 - t_10;
else
    t_10        = NaN;
    t_90        = NaN;
    rise_time_s = NaN;
end

band_exits = find(abs(T_log - setpoint) > settleBand);
if isempty(band_exits)
    settling_time_s = t_log(1);
elseif band_exits(end) < numel(t_log)
    settling_time_s = t_log(band_exits(end) + 1);
else
    settling_time_s = NaN;
end

[T_peak, idx_peak] = max(T_log);
peak_time_s        = t_log(idx_peak);
if dT_step > 0
    overshoot_pct = max(0, (T_peak - setpoint) / dT_step * 100);
else
    overshoot_pct = 0;
end

ss_error_C    = setpoint - T_ss;
final_T_C     = T_log(end);
max_abs_err_C = max(abs(e_log));
runtime_s     = t_log(end);
total_duty    = (heater_on_time / max(Ts, runtime_s)) * 100;

%% 9. Append Metrics Row to CSV
newRow = table( ...
    string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), ...
    setpoint, T_init, final_T_C, T_ss, ...
    rise_time_s, settling_time_s, overshoot_pct, peak_time_s, T_peak, ...
    ss_error_C, max_abs_err_C, T_amb, runtime_s, string(stopReason), ...
    fault_count, severe_fault_count, param_change_count, total_duty, fault_thresh, ...
    'VariableNames', { ...
    'timestamp', 'setpoint_C', 'initial_C', 'final_C', 'steady_state_C', ...
    'rise_time_s', 'settling_time_s', 'overshoot_pct', 'peak_time_s', 'peak_temp_C', ...
    'ss_error_C', 'max_abs_error_C', 'ambient_C', 'runtime_s', 'stop_reason', ...
    'fault_events', 'severe_events', 'param_changes', 'heater_duty_pct', 'fault_thresh_C'});

if exist(metricsFile, 'file')
    try
        oldTbl = readtable(metricsFile, 'TextType', 'string');
        oldTbl.timestamp = string(oldTbl.timestamp);
        newRow.timestamp = string(newRow.timestamp);
        missingInOld = setdiff(newRow.Properties.VariableNames, oldTbl.Properties.VariableNames);
        for m = 1:numel(missingInOld)
            oldTbl.(missingInOld{m}) = nan(height(oldTbl), 1);
        end
        newRow = newRow(:, oldTbl.Properties.VariableNames);
        writetable([oldTbl; newRow], metricsFile);
    catch
        writetable(newRow, metricsFile);
    end
else
    writetable(newRow, metricsFile);
end
fprintf('Metrics appended to: %s\n', metricsFile);

%% 10. Post-Run Annotated Step-Response Figure
fig2 = figure('Name', 'Step Response Metrics', 'Color', 'w', ...
              'Units', 'pixels', 'Position', [150 150 1000 650]);
hold on; grid on;

plot(t_log, T_log, 'b-', 'LineWidth', 1.8);
yline(setpoint,                'r--', 'LineWidth', 1.3);
yline(setpoint + settleBand,   'm:',  'LineWidth', 1.1);
yline(setpoint - settleBand,   'm:',  'LineWidth', 1.1);
yline(setpoint + fault_thresh, 'Color', [1 0.5 0], 'LineStyle', ':', 'LineWidth', 1.2);
yline(setpoint - fault_thresh, 'Color', [1 0.5 0], 'LineStyle', ':', 'LineWidth', 1.2);

% Normal transient fault regions (light red)
for f = 1:fault_count
    if f > numel(fault_end_t) || isnan(fault_end_t(f))
        t_end_patch = t_log(end);
    else
        t_end_patch = fault_end_t(f);
    end
    yl2 = ylim;
    patch([fault_start_t(f), t_end_patch, t_end_patch, fault_start_t(f)], ...
          [yl2(1), yl2(1), yl2(2), yl2(2)], ...
          'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end
if fault_count > 0
    plot(nan, nan, 'r-', 'LineWidth', 8, 'DisplayName', 'Transient fault');
end

% Severe fault regions (bright red, thick dark-red border)
for s = 1:severe_fault_count
    t_a = severe_fault_start_t(s);
    t_b = max(severe_fault_end_t(s), t_a + severe_min_patch_s);
    yl2 = ylim;
    patch([t_a, t_b, t_b, t_a], [yl2(1), yl2(1), yl2(2), yl2(2)], ...
          [1 0 0], 'FaceAlpha', 0.45, 'EdgeColor', [0.6 0 0], 'LineWidth', 1.5, ...
          'HandleVisibility', 'off');
end
if severe_fault_count > 0
    plot(nan, nan, 'Color', [1 0 0], 'LineWidth', 10, 'DisplayName', 'SEVERE fault');
end

% Parameter-change / oscillation regions (yellow)
for p = 1:param_change_count
    if p > numel(param_change_end_t) || isnan(param_change_end_t(p))
        t_end_patch = t_log(end);
    else
        t_end_patch = param_change_end_t(p);
    end
    yl2 = ylim;
    patch([param_change_start_t(p), t_end_patch, t_end_patch, param_change_start_t(p)], ...
          [yl2(1), yl2(1), yl2(2), yl2(2)], [1 0.8 0], 'FaceAlpha', 0.3, ...
          'EdgeColor', 'none', 'HandleVisibility', 'off');
end
if param_change_count > 0
    plot(nan, nan, 'Color', [1 0.8 0], 'LineWidth', 10, 'DisplayName', 'Param change');
end

if ~isnan(t_10)
    plot(t_10, T_log(idx_10), 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 7);
    plot(t_90, T_log(idx_90), 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 7);
    text(t_10, T_log(idx_10), sprintf('  t_{10%%}=%.1fs', t_10), 'FontSize', 9);
    text(t_90, T_log(idx_90), sprintf('  t_{90%%}=%.1fs', t_90), 'FontSize', 9);
end
plot(peak_time_s, T_peak, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 7);
text(peak_time_s, T_peak, sprintf('  Peak %.2f C @%.1fs', T_peak, peak_time_s), ...
     'FontSize', 9, 'Color', 'r');
if ~isnan(settling_time_s)
    xline(settling_time_s, 'g-', 'LineWidth', 1.3);
    xlim([0, min(max(t_log), settling_time_s * 2.5)]);
end

xlabel('Time [s]', 'FontSize', 11);
ylabel('Temperature [C]', 'FontSize', 11);
title('Step-Response Performance Metrics', 'FontSize', 12);
legend('Location', 'best', 'FontSize', 9);

metricTxt = sprintf([ ...
    'Transient faults   : %d\n' ...
    'SEVERE faults      : %d\n' ...
    'Param changes      : %d\n' ...
    'Fault threshold    : +/-%.2f C\n' ...
    'Heater duty cycle  : %.1f %%\n' ...
    'Rise time          : %.2f s\n' ...
    'Settling time      : %.2f s\n' ...
    'Steady-state error : %.3f C'], ...
    fault_count, severe_fault_count, param_change_count, fault_thresh, ...
    total_duty, rise_time_s, settling_time_s, ss_error_C);
annotation('textbox', [0.52 0.14 0.40 0.28], 'String', metricTxt, ...
           'BackgroundColor', 'w', 'EdgeColor', 'k', 'FontName', 'FixedWidth', 'FontSize', 9);

%% 11. Export Figures & Print Summary
save_fig(fig1, figMainName);
save_fig(fig2, figMetricName);

fprintf('\n===== Unified PI Run Summary =====\n');
fprintf('Ambient (averaged)      : %.2f C  (std %.3f C)\n', T_amb, T_amb_s);
fprintf('Stop reason             : %s\n', stopReason);
fprintf('Total runtime           : %.1f s\n', t_log(end));
fprintf('Final Temp              : %.2f C\n', T_log(end));
fprintf('Max |error|             : %.3f C\n', max(abs(e_log)));
fprintf('Rise time (10-90%%)      : %.2f s\n', rise_time_s);
fprintf('Settling time (+/-%.1f C): %.2f s\n', settleBand, settling_time_s);
fprintf('Steady-state error      : %.3f C\n', ss_error_C);
fprintf('Fault threshold used    : +/-%.2f C (dynamic, setpoint %.1f C)\n', fault_thresh, setpoint);
fprintf('TRANSIENT FAULTS        : %d\n', fault_count);
fprintf('SEVERE FAULTS           : %d\n', severe_fault_count);
fprintf('PARAMETER CHANGES       : %d\n', param_change_count);
fprintf('HEATER DUTY CYCLE       : %.1f %%\n', total_duty);
fprintf('Data saved to           : %s\n', csvName);
fprintf('Metrics CSV             : %s\n', metricsFile);
fprintf('===================================\n');

%% ---- Local Helper Functions ----
function write_snapshot(fname, t, T, Tamb, e, u, I, r, f_flag, s_flag)
    Ttbl = table(t(:), T(:), Tamb(:), e(:), u(:), I(:), r(:), f_flag(:), s_flag(:), ...
                 'VariableNames', {'time_s', 'temperature_C', 'ambient_C', ...
                                   'error_C', 'u', 'I', 'relay_cmd', 'fault_flag', 'severe_flag'});
    writetable(Ttbl, fname);
end

function shutdown_relays(a, h, f, off)
    try
        writeDigitalPin(a, h, off);
        writeDigitalPin(a, f, off);
        disp('Hardware safe: Relays OFF.');
    catch
    end
end

function save_fig(figH, baseName)
    pngPath = [baseName '.png'];
    pdfPath = [baseName '.pdf'];
    try
        exportgraphics(figH, pngPath, 'Resolution', 300);
        exportgraphics(figH, pdfPath, 'ContentType', 'vector');
    catch
        print(figH, pngPath, '-dpng', '-r300');
        print(figH, pdfPath, '-dpdf');
    end
    fprintf('Saved figure: %s / %s\n', pngPath, pdfPath);
end
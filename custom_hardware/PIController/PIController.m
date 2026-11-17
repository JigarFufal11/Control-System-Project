%% Unified HVAC Controller (ZN Tuned Heater PI + Fan Override with Hysteresis)
%  - ZN tuned gains: Kp = 0.44178, Ki = 0.009909
%  - Direct physical & sensor fault detection (Codes 1-4):
%      Code 1: LID OPENED        (dT < -0.5 C/s over 10-sample window)
%      Code 2: HEATER FAILURE    (Heater ON > 15s, 15s Temp rise < 0.3 C, T < setpoint - 1.0 C)
%      Code 3: FAN FAILURE       (Fan ON > 15 s, dT >= -0.02 C/s, T > setpoint + 1.0 C)
%      Code 4: SENSOR DISCONNECT / SPIKE (T_raw outside [10, 60] C or |T_raw - T_filt| > 5 C)
%  - Live 3-panel telemetry plot, periodic CSV logging, post-run metrics & annotated figure export

close all; clear; clc;

%% 1. Tuned PI Gains
Kp = 0.44178;
Ki = 0.009909;

%% 2. Arduino Hardware Setup
portName    = 'COM21'; % Ensure this matches your Arduino port
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
setpoint    = 32.0;
Ts          = 1.0;

% --- Fan Override Logic (Hysteresis) ---
fanOnDelta  = 1.5;
fanOffDelta = -1.5;

% --- Auto-Stop (Settle Detection) ---
settleBand    = 0.6;
settleTime    = 300;
minRunTime    = 120;
safetyMaxTime = 600;

% --- Actuator Deadband & Anti-Windup ---
u_deadband    = 0.05;
I_clamp       = 0.5;
minDwell      = 2.0;

% --- Ambient Estimation ---
ambientAvgTime  = 10;
ambientNSamples = round(ambientAvgTime / Ts);

% --- Fault Detection Parameters ---
fault_enable_time = 30;    % Ignore initial 30 s startup transient
heater_fail_time  = 15;    % Heater streak threshold [s]
fan_fail_time     = 15;    % Fan streak threshold [s]

% --- Logging & Export Filenames ---
maxSamples    = round(safetyMaxTime / Ts) + 100;
t_log         = zeros(1, maxSamples);
T_log         = zeros(1, maxSamples);
Tamb_log      = zeros(1, maxSamples);
e_log         = zeros(1, maxSamples);
u_log         = zeros(1, maxSamples);
I_log         = zeros(1, maxSamples);
rly_log       = zeros(1, maxSamples);
fault_log     = zeros(1, maxSamples);   % Stores fault_code (0=Normal, 1..4=Fault)

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

ylabel(ax1, 'Temperature [C]', 'FontSize', 11);
title(ax1, 'Unified PI Controller (ZN Heater + Fan Safety + Fault Detection)', 'FontSize', 12);

hTxt = text(ax1, 0.02, 0.95, '', 'Units', 'normalized', 'VerticalAlignment', 'top', ...
            'FontWeight', 'bold', 'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k');

legend(ax1, {'Chamber', 'Ambient (const.)', 'Setpoint', ...
             sprintf('-%.1f C band', settleBand), ...
             sprintf('+%.1f C band', settleBand), ...
             'Fan ON trip', 'Fan OFF trip'}, ...
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
alpha          = 0.8;

% --- Fault tracking state ---
heaterON_streak  = 0;
fanON_streak     = 0;
fault_active     = false;
active_fault_msg = 'NORMAL';
fault_count      = 0;
fault_start_idx  = NaN;
fault_start_t    = [];
fault_end_t      = [];
fault_labels     = {};
hFaultPatches    = [];

% --- Energy assessment ---
heater_on_time = 0;

fprintf('Initial chamber temp: %.2f C\n', T_filt);
fprintf('Running Unified PI Control... (Fault detection active after %ds)\n\n', fault_enable_time);

% --- LIVE TRACKING HEADER ---
fprintf('t(s)  | Temp   | Mode    | Error | Fault Status\n');
fprintf('-------------------------------------------------\n');

k_end      = 0;
stopReason = '';
isCooling  = false;

try
    for k = 1:maxSamples
        k_end    = k;
        t_log(k) = (k - 1) * Ts;

        % 1. Read and Filter Chamber Temperature
        T_raw  = readVoltage(a, sensorPin) * 100;
        T_filt = alpha * T_filt + (1 - alpha) * T_raw;
        e      = setpoint - T_filt;

        if k > 10
            dT = (T_filt - T_log(k - 10)) / (10 * Ts);
        else
            dT = 0;
        end

        % 2. Fault Detection Algorithm
        fault_code = 0;
        fault_msg  = '';
        
        % Calculate 15s temperature rise for heater failure check
        if k > heater_fail_time
            T_rise_15s = T_filt - T_log(k - heater_fail_time);
        else
            T_rise_15s = 10; % Bogus high value to prevent early triggering
        end
        
        if t_log(k) > fault_enable_time
            if T_raw < 10 || T_raw > 60
                fault_code = 4;
                fault_msg  = 'SENSOR DISCONNECT';
            elseif abs(T_raw - T_filt) > 5
                fault_code = 4;
                fault_msg  = 'SENSOR SPIKE';
            elseif dT < -0.5
                fault_code = 1;
                fault_msg  = 'LID OPENED';
            elseif relayState == 1 && heaterON_streak > heater_fail_time && T_rise_15s < 0.3 && T_filt < setpoint - 1.0
                fault_code = 2;
                fault_msg  = 'HEATER FAILURE';
            elseif relayState == -1 && fanON_streak > fan_fail_time && dT >= -0.02 && T_filt > setpoint + 1.0
                fault_code = 3;
                fault_msg  = 'FAN FAILURE';
            end
        end
        fault_log(k) = fault_code;

        % Fault Status & Live Patch Updates
        if fault_code > 0 && ~fault_active
            fprintf('\n>>> [FAULT DETECTED]: %s (Code %d) at t=%.1fs <<<\n', ...
                    fault_msg, fault_code, t_log(k));
            fault_active     = true;
            active_fault_msg = fault_msg;
            fault_start_idx  = k;
            fault_count      = fault_count + 1;
            fault_start_t(fault_count) = t_log(k);
            fault_end_t(fault_count)   = NaN;
            fault_labels{fault_count}  = fault_msg;

            yl = get(ax1, 'YLim');
            hFaultPatches(end+1) = patch(ax1, ...
                'XData', [t_log(k), t_log(k) + Ts, t_log(k) + Ts, t_log(k)], ...
                'YData', [yl(1), yl(1), yl(2), yl(2)], ...
                'FaceColor', [1 0 0], 'FaceAlpha', 0.25, ...
                'EdgeColor', [0.7 0 0], 'LineWidth', 1.0, ...
                'HandleVisibility', 'off');
        elseif fault_code > 0 && fault_active
            active_fault_msg = fault_msg;
            if ~isempty(hFaultPatches)
                yl = get(ax1, 'YLim');
                set(hFaultPatches(end), ...
                    'XData', [t_log(fault_start_idx), t_log(k), t_log(k), t_log(fault_start_idx)], ...
                    'YData', [yl(1), yl(1), yl(2), yl(2)]);
            end
        elseif fault_code == 0 && fault_active
            fprintf('>>> [RECOVERED]: Back to normal at t=%.1fs <<<\n\n', t_log(k));
            fault_active     = false;
            active_fault_msg = 'NORMAL';
            fault_end_t(fault_count) = t_log(k);
            if ~isempty(hFaultPatches)
                yl = get(ax1, 'YLim');
                set(hFaultPatches(end), ...
                    'XData', [t_log(fault_start_idx), t_log(k), t_log(k), t_log(fault_start_idx)], ...
                    'YData', [yl(1), yl(1), yl(2), yl(2)]);
            end
        end

        % 3. PI Control Law (with Integral Anti-Windup Clamping)
        I = I + Ki * e * Ts;
        I = max(min(I, I_clamp), -I_clamp);
        u = Kp * e + I;

        % 4. Hybrid Relay Decision (Fan Override with Hysteresis + Dwell Time)
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

        % Update Actuator Streak Counters
        if relayState == 1
            heaterON_streak = heaterON_streak + 1;
        else
            heaterON_streak = 0;
        end
        
        if relayState == -1
            fanON_streak = fanON_streak + 1;
        else
            fanON_streak = 0;
        end

        % 5. Apply Commands to Hardware Relays
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

        % 6. Log Step Data
        T_log(k)    = T_filt;
        Tamb_log(k) = T_amb;
        e_log(k)    = e;
        u_log(k)    = u;
        I_log(k)    = I;
        rly_log(k)  = relayState;

        % --- 6.5 LIVE TRACKING IN COMMAND WINDOW (Every 5s) ---
        if mod(k, 5) == 0
            if relayState == 1
                modeStr = 'HEATING';
            elseif relayState == -1
                modeStr = 'COOLING';
            else
                modeStr = 'IDLE';
            end
            
            if fault_code == 0
                faultStr = '-';
            else
                faultStr = ['*** ' fault_msg ' ***'];
            end
            
            fprintf('%4.0fs | %5.2fC | %-7s | %5.2f | %s\n', ...
                    t_log(k), T_filt, modeStr, e, faultStr);
        end

        % 7. Auto-Stop Check (Settled inside +/-settleBand for settleTime with no active fault)
        if t_log(k) >= minRunTime && ~fault_active
            needSamples = round(settleTime / Ts);
            if k > needSamples
                if all(abs(e_log(k - needSamples + 1 : k)) <= settleBand) && ...
                   all(fault_log(k - needSamples + 1 : k) == 0)
                    stopReason = sprintf('Success: settled +/-%.1f C for %.0fs', ...
                                         settleBand, settleTime);
                    fprintf('\n%s\n', stopReason);
                    break;
                end
            end
        end

        % 8. Live Plot Update (every 3 samples)
        if mod(k, 3) == 0
            set(hT,       'XData', t_log(1:k),   'YData', T_log(1:k));
            set(hAmb,     'XData', t_log(1:k),   'YData', Tamb_log(1:k));
            set(hRef,     'XData', t_log(1:k),   'YData', setpoint * ones(1, k));
            set(hBan1,    'XData', t_log(1:k),   'YData', (setpoint + settleBand) * ones(1, k));
            set(hBan2,    'XData', t_log(1:k),   'YData', (setpoint - settleBand) * ones(1, k));
            set(hFanOnL,  'XData', [0 t_log(k)], 'YData', (setpoint + fanOnDelta)  * ones(1, 2));
            set(hFanOffL, 'XData', [0 t_log(k)], 'YData', (setpoint - fanOffDelta) * ones(1, 2));
            set(hU,       'XData', t_log(1:k),   'YData', u_log(1:k));

            heaterY = double(rly_log(1:k) ==  1) * 0.5 + 0.5;
            fanY    = double(rly_log(1:k) == -1) * 0.5;
            set(hHeaterBand, 'XData', t_log(1:k), 'YData', heaterY);
            set(hFanBand,    'XData', t_log(1:k), 'YData', fanY);

            duty_cycle = (heater_on_time / max(1, t_log(k))) * 100;
            if fault_active
                statusColor = [0.85 0 0];
            else
                statusColor = [0 0.55 0];
            end
            
            set(hTxt, 'Color', statusColor, 'String', sprintf( ...
                'STATUS: %s\nDuty Cycle: %.1f%%\nFault events: %d', ...
                active_fault_msg, duty_cycle, fault_count));

            yl = get(ax1, 'YLim');
            for i = 1:numel(hFaultPatches)
                set(hFaultPatches(i), 'YData', [yl(1), yl(1), yl(2), yl(2)]);
            end
            xlim(ax1, [0, max(10, t_log(k))]);
            drawnow limitrate;
        end

        % 9. Periodic CSV Backup (every 30 samples)
        if mod(k, 30) == 0
            write_snapshot(csvName, t_log(1:k), T_log(1:k), Tamb_log(1:k), ...
                           e_log(1:k), u_log(1:k), I_log(1:k), rly_log(1:k), ...
                           fault_log(1:k));
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
t_log     = t_log(1:k_end);
T_log     = T_log(1:k_end);
Tamb_log  = Tamb_log(1:k_end);
e_log     = e_log(1:k_end);
u_log     = u_log(1:k_end);
I_log     = I_log(1:k_end);
rly_log   = rly_log(1:k_end);
fault_log = fault_log(1:k_end);

% Close any open fault interval if stopped mid-fault
if fault_active && k_end > 0 && fault_count > 0 && isnan(fault_end_t(fault_count))
    fault_end_t(fault_count) = t_log(k_end);
end

write_snapshot(csvName, t_log, T_log, Tamb_log, e_log, u_log, I_log, rly_log, fault_log);

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

% Fault breakdown counts (samples per fault code)
n_lid_faults    = sum(fault_log == 1);
n_heater_faults = sum(fault_log == 2);
n_fan_faults    = sum(fault_log == 3);
n_sensor_faults = sum(fault_log == 4);

%% 9. Append Metrics Row to CSV
newRow = table( ...
    string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), ...
    setpoint, T_init, final_T_C, T_ss, ...
    rise_time_s, settling_time_s, overshoot_pct, peak_time_s, T_peak, ...
    ss_error_C, max_abs_err_C, T_amb, runtime_s, string(stopReason), ...
    fault_count, n_lid_faults, n_heater_faults, n_fan_faults, n_sensor_faults, total_duty, ...
    'VariableNames', { ...
    'timestamp', 'setpoint_C', 'initial_C', 'final_C', 'steady_state_C', ...
    'rise_time_s', 'settling_time_s', 'overshoot_pct', 'peak_time_s', 'peak_temp_C', ...
    'ss_error_C', 'max_abs_error_C', 'ambient_C', 'runtime_s', 'stop_reason', ...
    'fault_events', 'lid_fault_s', 'heater_fault_s', 'fan_fault_s', 'sensor_fault_s', 'heater_duty_pct'});

if exist(metricsFile, 'file')
    try
        oldTbl = readtable(metricsFile, 'TextType', 'string');
        oldTbl.timestamp = string(oldTbl.timestamp);
        newRow.timestamp = string(newRow.timestamp);
        if isequal(sort(oldTbl.Properties.VariableNames), sort(newRow.Properties.VariableNames))
            newRow = newRow(:, oldTbl.Properties.VariableNames);
            writetable([oldTbl; newRow], metricsFile);
        else
            writetable(newRow, metricsFile);
        end
    catch
        writetable(newRow, metricsFile);
    end
else
    writetable(newRow, metricsFile);
end
fprintf('Metrics saved to: %s\n', metricsFile);

%% 10. Post-Run Annotated Step-Response Figure
fig2 = figure('Name', 'Step Response Metrics', 'Color', 'w', ...
              'Units', 'pixels', 'Position', [150 150 1000 650]);
hold on; grid on;

plot(t_log, T_log, 'b-', 'LineWidth', 1.8);
yline(setpoint,              'r--', 'LineWidth', 1.3);
yline(setpoint + settleBand, 'm:',  'LineWidth', 1.1);
yline(setpoint - settleBand, 'm:',  'LineWidth', 1.1);

% Highlight detected fault intervals (red shaded regions)
for f = 1:fault_count
    if f > numel(fault_end_t) || isnan(fault_end_t(f))
        t_end_patch = t_log(end);
    else
        t_end_patch = max(fault_end_t(f), fault_start_t(f) + Ts);
    end
    yl2 = ylim;
    patch([fault_start_t(f), t_end_patch, t_end_patch, fault_start_t(f)], ...
          [yl2(1), yl2(1), yl2(2), yl2(2)], ...
          [1 0 0], 'FaceAlpha', 0.25, 'EdgeColor', [0.7 0 0], 'LineWidth', 1.0, ...
          'HandleVisibility', 'off');
    text(fault_start_t(f), yl2(2) - 0.05 * range(yl2), [' ' fault_labels{f}], ...
         'Color', [0.75 0 0], 'FontSize', 8, 'FontWeight', 'bold', 'VerticalAlignment', 'top');
end
if fault_count > 0
    plot(nan, nan, 'Color', [1 0 0], 'LineWidth', 8, 'DisplayName', 'Detected Fault');
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
    'Fault events       : %d\n' ...
    'Lid / Heater / Fan : %ds / %ds / %ds\n' ...
    'Sensor faults      : %ds\n' ...
    'Heater duty cycle  : %.1f %%\n' ...
    'Rise time          : %.2f s\n' ...
    'Settling time      : %.2f s\n' ...
    'Steady-state error : %.3f C'], ...
    fault_count, n_lid_faults, n_heater_faults, n_fan_faults, n_sensor_faults, ...
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
fprintf('TOTAL FAULT EVENTS      : %d\n', fault_count);
fprintf('  - Lid Opened (Code 1) : %d s\n', n_lid_faults);
fprintf('  - Heater Fail (Code 2): %d s\n', n_heater_faults);
fprintf('  - Fan Fail (Code 3)   : %d s\n', n_fan_faults);
fprintf('  - Sensor Fault (Code 4): %d s\n', n_sensor_faults);
fprintf('HEATER DUTY CYCLE       : %.1f %%\n', total_duty);
fprintf('Data saved to           : %s\n', csvName);
fprintf('Metrics CSV             : %s\n', metricsFile);
fprintf('===================================\n');

%% ---- Local Helper Functions ----
function write_snapshot(fname, t, T, Tamb, e, u, I, r, f_code)
    Ttbl = table(t(:), T(:), Tamb(:), e(:), u(:), I(:), r(:), f_code(:), ...
                 'VariableNames', {'time_s', 'temperature_C', 'ambient_C', ...
                                   'error_C', 'u', 'I', 'relay_cmd', 'fault_code'});
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
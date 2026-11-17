    %% Unified HVAC Controller (ZN Tuned Heater PI + Fan Override with Hysteresis)
    %  Uses ZN tuned gains: Kp = 0.44178, Ki = 0.009909
    %  Fan acts as a safety override if temp gets too high.
    %  Ambient temperature is estimated from a 10-s warm-up average of the
    %  chamber sensor (single-sensor setup).
    %  Post-run performance metrics (tr, ts, overshoot, ess) are appended to
    %  performance_metrics.csv and drawn on an annotated figure.
    %  Main figure is 3-panel (Temp / Control effort / Actuator timeline) and
    %  is saved as PNG+PDF for submission.
    close all; clear; clc;
    
    %% 1. Tuned PI Gains (from Heater ZN Experiment)
    Kp = 0.44178;
    Ki = 0.009909;
    
    %% 2. Arduino setup
    portName    = 'COM19';
    a           = arduino(portName, 'Mega2560');
    sensorPin   = 'A1';
    heaterRelay = 'D6';
    fanRelay    = 'D9';
    configurePin(a, heaterRelay, 'DigitalOutput');
    configurePin(a, fanRelay,    'DigitalOutput');
    
    relayON  = 0;
    relayOFF = 1;
    writeDigitalPin(a, heaterRelay, relayOFF);
    writeDigitalPin(a, fanRelay,    relayOFF);
    
    %% 3. Control Parameters
    setpoint    = 25.0;
    Ts          = 1.0;
    
    % --- Fan Override Logic (Hysteresis) ---
    fanOnDelta  = 1.0;
    fanOffDelta = -1.0;
    
    % --- Auto-stop (Settle Detection) ---
    settleBand    = 0.6;
    settleTime    = 600;
    minRunTime    = 120;
    safetyMaxTime = 3600;
    
    % --- Actuator Deadband & Anti-Windup ---
    u_deadband    = 0.05;
    I_clamp       = 0.5;
    minDwell      = 2.0;
    
    % --- Ambient estimation ---
    ambientAvgTime  = 10;
    ambientNSamples = round(ambientAvgTime/Ts);
    
    % --- Logging ---
    maxSamples  = round(safetyMaxTime/Ts) + 100;
    t_log    = zeros(1,maxSamples);
    T_log    = zeros(1,maxSamples);
    Tamb_log = zeros(1,maxSamples);
    e_log    = zeros(1,maxSamples);
    u_log    = zeros(1,maxSamples);
    I_log    = zeros(1,maxSamples);
    rly_log  = zeros(1,maxSamples);
    csvName     = 'unified_pi_control_data.csv';
    metricsFile = 'performance_metrics.csv';
    figMainName = 'hvac_main_response';        % <-- NEW
    figMetricName = 'hvac_step_metrics';       % <-- NEW
    
    %% 4. Ambient Estimation (10-s warm-up average)
    fprintf('Measuring ambient temperature over %d s (%d samples)...\n', ...
            ambientAvgTime, ambientNSamples);
    
    ambient_readings = zeros(1, ambientNSamples);
    for ii = 1:ambientNSamples
        ambient_readings(ii) = readVoltage(a, sensorPin) * 100;
        fprintf('  [%2d/%2d]  T = %.2f °C\n', ii, ambientNSamples, ambient_readings(ii));
        pause(Ts);
    end
    
    T_amb   = mean(ambient_readings);
    T_amb_s = std(ambient_readings);
    
    fprintf('Ambient temperature ready: %.2f °C (std = %.3f °C, n = %d)\n', ...
            T_amb, T_amb_s, ambientNSamples);
    
    %% 5. Live Figure Setup (3-panel, publication sized)
    fig1 = figure('Name','Unified HVAC Control','Color','w', ...
                  'Units','pixels','Position',[100 100 1200 900]);
    
    % ----- Panel 1: Temperature -----
    ax1 = subplot(3,1,1); hold(ax1,'on'); grid(ax1,'on');
    hT      = plot(ax1, nan, nan, 'b-',  'LineWidth', 1.8);
    hAmb    = plot(ax1, nan, nan, 'g--', 'LineWidth', 1.2);
    hRef    = plot(ax1, nan, nan, 'r--', 'LineWidth', 1.3);
    hBan1   = plot(ax1, nan, nan, 'm:',  'LineWidth', 1.2);
    hBan2   = plot(ax1, nan, nan, 'm:',  'LineWidth', 1.2);
    hFanOnL  = plot(ax1, [0 maxSamples*Ts], [setpoint+fanOnDelta  setpoint+fanOnDelta ], 'k-.', 'LineWidth', 1.0);
    hFanOffL = plot(ax1, [0 maxSamples*Ts], [setpoint-fanOffDelta setpoint-fanOffDelta], 'c-.', 'LineWidth', 1.0);
    ylabel(ax1,'Temperature [°C]', 'FontSize', 11);
    title(ax1,'Unified PI Controller (ZN Heater + Fan Safety)', 'FontSize', 12);
    legend(ax1, {'Chamber', 'Ambient (const.)', 'Setpoint', ...
                 sprintf('-%.1f°C band', settleBand), ...
                 sprintf('+%.1f°C band', settleBand), ...
                 'Fan ON trip', 'Fan OFF trip'}, 'Location','best', 'FontSize', 9);
    
    % ----- Panel 2: Control effort -----
    ax2 = subplot(3,1,2); hold(ax2,'on'); grid(ax2,'on');
    hU    = plot(ax2, nan, nan, 'k-', 'LineWidth', 1.3);
    hDead = yline(ax2, u_deadband, 'r:', 'LineWidth', 1.0);
    ylabel(ax2,'Control effort u', 'FontSize', 11);
    title(ax2,'PI Output (Heater Logic)', 'FontSize', 12);
    legend(ax2, {'u', sprintf('deadband = %.2f', u_deadband)}, ...
           'Location','best', 'FontSize', 9);
    
    % ----- Panel 3: Actuator ON/OFF timeline (Gantt-style) -----
    ax3 = subplot(3,1,3); hold(ax3,'on'); grid(ax3,'on');
    % Heater band fills [0.5, 1.0] when ON
    hHeaterBand = area(ax3, nan, nan, 0.5, ...
        'FaceColor', [0.85 0.15 0.15], 'FaceAlpha', 0.75, 'EdgeColor', [0.5 0 0]);
    % Fan band fills [0.0, 0.5] when ON
    hFanBand = area(ax3, nan, nan, 0.0, ...
        'FaceColor', [0.15 0.35 0.85], 'FaceAlpha', 0.75, 'EdgeColor', [0 0 0.5]);
    ylim(ax3, [0 1]);
    yticks(ax3, [0.25 0.75]);
    yticklabels(ax3, {'Fan', 'Heater'});
    ylabel(ax3,'Actuator', 'FontSize', 11);
    xlabel(ax3,'Time [s]', 'FontSize', 11);
    title(ax3,'Actuator ON/OFF Timeline', 'FontSize', 12);
    
    %% 6. Safe Shutdown Hook
    cleanup = onCleanup(@() shutdown_relays(a, heaterRelay, fanRelay, relayOFF));
    
    %% 7. Control Loop State
    I          = 0;
    relayState = 0;
    lastSwitchTime = -999;
    
    T_filt = T_amb;
    alpha  = 0.8;
    
    fprintf('Initial chamber temp: %.2f °C\n', T_filt);
    fprintf('Running Unified PI Control...\n');
    
    k_end      = 0;
    stopReason = '';
    isCooling  = false;
    
    try
        for k = 1:maxSamples
            k_end = k;
    
            % 1. Read and Filter Chamber Temperature
            T_raw  = readVoltage(a, sensorPin) * 100;
            T_filt = alpha * T_filt + (1 - alpha) * T_raw;
            e      = setpoint - T_filt;
    
            % 2. PI Control Law (Heater Logic)
            I = I + Ki * e * Ts;
            I = max(min(I, I_clamp), -I_clamp);
            u = Kp * e + I;
    
            % 3. HYBRID RELAY DECISION
            timeSinceSwitch = (k-1)*Ts - lastSwitchTime;
    
            if timeSinceSwitch >= minDwell
    
                % --- FAN OVERRIDE LOGIC (hysteresis) ---
                if T_filt >= setpoint + fanOnDelta
                    isCooling = true;
                elseif T_filt <= setpoint - fanOffDelta
                    isCooling = false;
                end
    
                if isCooling
                    if relayState ~= -1
                        relayState = -1;
                        lastSwitchTime = (k-1)*Ts;
                    end
                else
                    if u > u_deadband
                        if relayState ~= +1
                            relayState = +1;
                            lastSwitchTime = (k-1)*Ts;
                        end
                    else
                        if relayState ~= 0
                            relayState = 0;
                            lastSwitchTime = (k-1)*Ts;
                        end
                    end
                end
            end
    
            % 4. Apply to Hardware
            if relayState == +1
                writeDigitalPin(a, heaterRelay, relayON);
                writeDigitalPin(a, fanRelay,    relayOFF);
            elseif relayState == -1
                writeDigitalPin(a, heaterRelay, relayOFF);
                writeDigitalPin(a, fanRelay,    relayON);
            else
                writeDigitalPin(a, heaterRelay, relayOFF);
                writeDigitalPin(a, fanRelay,    relayOFF);
            end
    
            % 5. Logging
            t_log(k)    = (k-1)*Ts;
            T_log(k)    = T_filt;
            Tamb_log(k) = T_amb;
            e_log(k)    = e;
            u_log(k)    = u;
            I_log(k)    = I;
            rly_log(k)  = relayState;
    
            % 6. Auto-Stop Detection
            if t_log(k) >= minRunTime
                needSamples = round(settleTime/Ts);
                if k > needSamples
                    recentErr = e_log(k-needSamples+1:k);
                    if all(abs(recentErr) <= settleBand)
                        stopReason = sprintf('Success: Settled within ±%.1f°C for %.0fs', settleBand, settleTime);
                        fprintf('\n%s\n', stopReason);
                        break;
                    end
                end
            end
    
            % 7. Live Plot (all 3 panels)
            if mod(k, 3) == 0
                set(hT,   'XData', t_log(1:k), 'YData', T_log(1:k));
                set(hAmb, 'XData', t_log(1:k), 'YData', Tamb_log(1:k));
                set(hRef, 'XData', t_log(1:k), 'YData', setpoint*ones(1,k));
                set(hBan1,'XData', t_log(1:k), 'YData', (setpoint+settleBand)*ones(1,k));
                set(hBan2,'XData', t_log(1:k), 'YData', (setpoint-settleBand)*ones(1,k));
                set(hU,   'XData', t_log(1:k), 'YData', u_log(1:k));
    
                heaterY = double(rly_log(1:k) == 1) * 0.5 + 0.5;   % 1.0 ON, 0.5 OFF
                fanY    = double(rly_log(1:k) == -1) * 0.5;        % 0.5 ON, 0.0 OFF
                set(hHeaterBand, 'XData', t_log(1:k), 'YData', heaterY);
                set(hFanBand,    'XData', t_log(1:k), 'YData', fanY);
    
                drawnow limitrate;
            end
    
            % 8. Periodic CSV Save
            if mod(k, 30) == 0
                write_snapshot(csvName, t_log(1:k), T_log(1:k), Tamb_log(1:k), ...
                               e_log(1:k), u_log(1:k), I_log(1:k), rly_log(1:k));
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
    
    % Final Save
    t_log    = t_log(1:k_end);
    T_log    = T_log(1:k_end);
    Tamb_log = Tamb_log(1:k_end);
    e_log    = e_log(1:k_end);
    u_log    = u_log(1:k_end);
    I_log    = I_log(1:k_end);
    rly_log  = rly_log(1:k_end);
    write_snapshot(csvName, t_log, T_log, Tamb_log, e_log, u_log, I_log, rly_log);
    
    %% 9. Post-Run Performance Metrics
    T_init  = T_amb;
    dT_step = setpoint - T_init;
    
    ssWindowSamples = min(round(60/Ts), numel(t_log));
    T_ss = mean(T_log(end-ssWindowSamples+1:end));
    
    if dT_step > 0
        T_norm = (T_log - T_init) / dT_step;
    else
        T_norm = nan(size(T_log));
    end
    
    idx_10 = find(T_norm >= 0.1, 1, 'first');
    idx_90 = find(T_norm >= 0.9, 1, 'first');
    if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
        t_10 = t_log(idx_10);  t_90 = t_log(idx_90);
        rise_time_s = t_90 - t_10;
    else
        t_10 = NaN;  t_90 = NaN;  rise_time_s = NaN;
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
    peak_time_s = t_log(idx_peak);
    if dT_step > 0
        overshoot_pct = max(0, (T_peak - setpoint) / dT_step * 100);
    else
        overshoot_pct = 0;
    end
    
    ss_error_C    = setpoint - T_ss;
    final_T_C     = T_log(end);
    max_abs_err_C = max(abs(e_log));
    runtime_s     = t_log(end);
    
    %% 10. Append metrics row to performance_metrics.csv
    newRow = table( ...
        string(datetime('now','Format','yyyy-MM-dd HH:mm:ss')), ...
        setpoint, T_init, final_T_C, T_ss, ...
        rise_time_s, settling_time_s, overshoot_pct, peak_time_s, T_peak, ...
        ss_error_C, max_abs_err_C, T_amb, runtime_s, string(stopReason), ...
        'VariableNames', { ...
        'timestamp','setpoint_C','initial_C','final_C','steady_state_C', ...
        'rise_time_s','settling_time_s','overshoot_pct','peak_time_s','peak_temp_C', ...
        'ss_error_C','max_abs_error_C','ambient_C','runtime_s','stop_reason'});
    
    if exist(metricsFile,'file')
        try
            oldTbl = readtable(metricsFile, 'TextType','string');
            oldTbl.timestamp = string(oldTbl.timestamp);
            newRow.timestamp = string(newRow.timestamp);
            writetable([oldTbl; newRow], metricsFile);
        catch
            writetable(newRow, metricsFile);
        end
    else
        writetable(newRow, metricsFile);
    end
    fprintf('Metrics appended to: %s\n', metricsFile);
    
    %% 11. Post-Run Annotated Step-Response Figure
    fig2 = figure('Name','Step Response Metrics','Color','w', ...
                  'Units','pixels','Position',[150 150 1000 650]);
    hold on; grid on;
    
    plot(t_log, T_log, 'b-', 'LineWidth', 1.8);
    yline(setpoint,            'r--','LineWidth', 1.3);
    yline(setpoint+settleBand, 'm:', 'LineWidth', 1.1);
    yline(setpoint-settleBand, 'm:', 'LineWidth', 1.1);
    
    if ~isnan(t_10)
        plot(t_10, T_log(idx_10), 'ko', 'MarkerFaceColor','g', 'MarkerSize', 7);
        plot(t_90, T_log(idx_90), 'ko', 'MarkerFaceColor','g', 'MarkerSize', 7);
        text(t_10, T_log(idx_10), sprintf('  t_{10%%}=%.1fs', t_10), 'FontSize', 9);
        text(t_90, T_log(idx_90), sprintf('  t_{90%%}=%.1fs', t_90), 'FontSize', 9);
    end
    
    plot(peak_time_s, T_peak, 'ro', 'MarkerFaceColor','r', 'MarkerSize', 7);
    text(peak_time_s, T_peak, sprintf('  Peak %.2f°C @%.1fs', T_peak, peak_time_s), ...
         'FontSize', 9, 'Color','r');
    
    if ~isnan(settling_time_s)
        xline(settling_time_s, 'g-', 'LineWidth', 1.3);
        xlim([0, min(max(t_log), settling_time_s*2.5)]);
    end
    
    xlabel('Time [s]', 'FontSize', 11); ylabel('Temperature [°C]', 'FontSize', 11);
    title('Step-Response Performance Metrics', 'FontSize', 12);
    legend({'Response','Setpoint','Upper band','Lower band'}, ...
           'Location','best', 'FontSize', 9);
    
    metricTxt = sprintf([ ...
        'Rise time (10-90%%): %.2f s\n' ...
        'Settling time (±%.1f°C): %.2f s\n' ...
        'Overshoot       : %.2f %%\n' ...
        'Peak            : %.2f °C @ %.1f s\n' ...
        'Steady-state err: %.3f °C'], ...
        rise_time_s, settleBand, settling_time_s, ...
        overshoot_pct, T_peak, peak_time_s, ss_error_C);
    annotation('textbox',[0.60 0.18 0.32 0.20], 'String', metricTxt, ...
               'BackgroundColor','w', 'EdgeColor','k', 'FontName','FixedWidth', ...
               'FontSize', 9);
    
    %% 12. Export Both Figures for Submission
    save_fig(fig1, figMainName);
    save_fig(fig2, figMetricName);
    
    %% 13. Run Summary
    fprintf('\n===== Unified PI Run Summary =====\n');
    fprintf('Ambient (averaged) : %.2f °C  (std %.3f °C)\n', T_amb, T_amb_s);
    fprintf('Stop reason        : %s\n', stopReason);
    fprintf('Total runtime      : %.1f s\n', t_log(end));
    fprintf('Final Temp         : %.2f °C\n', T_log(end));
    fprintf('Max |error|        : %.3f °C\n', max(abs(e_log)));
    fprintf('Fan ON trip        : %.2f °C\n', setpoint + fanOnDelta);
    fprintf('Fan OFF trip       : %.2f °C\n', setpoint - fanOffDelta);
    fprintf('Rise time (10-90%%)  : %.2f s\n', rise_time_s);
    fprintf('Settling time (±%.1f°C): %.2f s\n', settleBand, settling_time_s);
    fprintf('Overshoot            : %.2f %%\n', overshoot_pct);
    fprintf('Peak temp            : %.2f °C @ %.1f s\n', T_peak, peak_time_s);
    fprintf('Steady-state error   : %.3f °C\n', ss_error_C);
    fprintf('Data saved to      : %s\n', csvName);
    fprintf('Metrics CSV        : %s\n', metricsFile);
    fprintf('Main figure        : %s.{png,pdf}\n', figMainName);
    fprintf('Metrics figure     : %s.{png,pdf}\n', figMetricName);
    fprintf('===================================\n');
    
    %% ---- Local Helpers ----
    function write_snapshot(fname, t, T, Tamb, e, u, I, r)
        Ttbl = table(t(:), T(:), Tamb(:), e(:), u(:), I(:), r(:), ...
                     'VariableNames', {'time_s','temperature_C','ambient_C', ...
                                       'error_C','u','I','relay_cmd'});
        writetable(Ttbl, fname);
    end
    
    function shutdown_relays(a, h, f, off)
        try
            writeDigitalPin(a, h, off); writeDigitalPin(a, f, off);
            disp('Hardware safe: Relays OFF.');
        catch
        end
    end
    
    function save_fig(figH, baseName)
    % Saves a figure as PNG (300 DPI) and PDF (vector). Falls back to print()
    % for older MATLAB releases that lack exportgraphics.
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
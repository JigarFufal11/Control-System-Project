%% ============================================================
%  ON/OFF Temperature Controller — final, auto-stopping
%  Change SETPOINT only. Runs 35 → 30 → 25. No Ctrl+C needed.
% ============================================================
close all; clear; clc;

%% USER CONFIG — change only SETPOINT per run
SETPOINT          = 35.0;   % <-- 35.0 / 30.0 / 25.0
HYSTERESIS        = 0.5;    % °C
T_SAMPLE          = 0.2;    % s
T_END             = 900;    % s — hard safety limit
T_STEADY_AFTER_X  = 200;    % s — steady-state duration required after first cross
MIN_HEATER_CYCLES = 5;      % heater OFF events required before auto-stop
T_MAX_SAFE        = 45.0;   % °C emergency cutoff
COM_PORT          = 'COM19';

relayON = 0; relayOFF = 1;  % active-LOW relay module

%% Console log to disk (survives Ctrl+C, crashes, etc.)
logFile = sprintf('console_SP%d_%s.txt', round(SETPOINT), datestr(now,'yyyymmdd_HHMMSS'));
diary(logFile);

%% Connect
fprintf('\n=== ON/OFF controller ===\n');
fprintf('SP=%.1f C, h=%.2f C, Ts=%.2f s, T_END=%d s, steady=%d s\n\n', ...
    SETPOINT, HYSTERESIS, T_SAMPLE, T_END, T_STEADY_AFTER_X);

a = arduino(COM_PORT, 'Mega2560');
sensorPin = 'A1'; heaterRelay = 'D6'; fanRelay = 'D9';
configurePin(a, heaterRelay, 'DigitalOutput');
configurePin(a, fanRelay,    'DigitalOutput');
writeDigitalPin(a, heaterRelay, relayOFF);
writeDigitalPin(a, fanRelay,    relayOFF);
heaterState = relayOFF; fanState = relayOFF;

%% Ambient (relays OFF)
fprintf('Measuring ambient (2 s)...\n');
pause(2);
ambS = zeros(20,1);
for i = 1:20
    ambS(i) = readVoltage(a, sensorPin)*100;
    pause(0.1);
end
T_amb = mean(ambS(~isnan(ambS)));
if isnan(T_amb) || T_amb < -10 || T_amb > 60
    T_amb = 20.0;
end
fprintf('T_amb = %.2f C\n\n', T_amb);
if SETPOINT <= T_amb + 0.5
    warning('SP %.1f <= ambient %.1f — heater will not fire', SETPOINT, T_amb);
end

%% Pre-allocate
N_max    = ceil(T_END/T_SAMPLE) + 500;
logTime  = zeros(N_max,1); logT = zeros(N_max,1);
logTfilt = zeros(N_max,1); logH = zeros(N_max,1);
logF     = zeros(N_max,1); logS = zeros(N_max,1);
tBuf     = nan(25,1);   % 5 s moving average
k = 0; lastSave = 0; heaterOffCount = 0; T_first_cross = NaN;

RUN_TAG  = sprintf('onoff_SP%d_h%0.2f_%s', round(SETPOINT), HYSTERESIS, datestr(now,'yyyymmdd_HHMMSS'));
CSV_FILE = [RUN_TAG '.csv'];
SUM_FILE = [RUN_TAG '_metrics.csv'];
fprintf('Data:   %s\nMetrics: %s\n\n', CSV_FILE, SUM_FILE);
fprintf('Running. Auto-stops when steady state confirmed.\n');
fprintf('If you press Ctrl+C, data up to the last 10 s is saved.\n\n');

%% Main loop
t0 = tic; stopReason = 'normal end';
try
    while true
        tNow = toc(t0);

        % --- read ---
        Traw = readVoltage(a, sensorPin)*100;
        if isnan(Traw) || Traw < -20 || Traw > 80
            Tf = NaN;
        else
            tBuf = [tBuf(2:end); Traw];
            Tf   = mean(tBuf(~isnan(tBuf)));
        end

        % --- safety ---
        if ~isnan(Tf) && Tf > T_MAX_SAFE
            writeDigitalPin(a, heaterRelay, relayOFF);
            writeDigitalPin(a, fanRelay,    relayON);
            stopReason = sprintf('SAFETY CUTOFF T=%.1f C > %.1f C', Tf, T_MAX_SAFE);
            fprintf('\n!!! %s !!!\n', stopReason);
            break;
        end

        % --- decision ---
        if isnan(Tf)
            newH=relayOFF; newF=relayOFF; sCode=0; sStr='BAD ';
        elseif Tf < (SETPOINT - HYSTERESIS)
            newH=relayON;  newF=relayOFF; sCode= 1; sStr='HEAT';
        elseif Tf > (SETPOINT + HYSTERESIS)
            newH=relayOFF; newF=relayON;  sCode=-1; sStr='COOL';
        else
            newH=relayOFF; newF=relayOFF; sCode= 0; sStr='IDLE';
        end

        % --- actuate ---
        if newH ~= heaterState
            writeDigitalPin(a, heaterRelay, newH);
            if newH == relayOFF, heaterOffCount = heaterOffCount + 1; end
            heaterState = newH;
        end
        if newF ~= fanState
            writeDigitalPin(a, fanRelay, newF);
            fanState = newF;
        end

        % --- log ---
        k = k + 1;
        logTime(k)=tNow; logT(k)=Traw; logTfilt(k)=Tf;
        logH(k)=(heaterState==relayON); logF(k)=(fanState==relayON); logS(k)=sCode;

        % First-cross detection
        if isnan(T_first_cross) && ~isnan(Tf) && Tf >= (SETPOINT - HYSTERESIS)
            T_first_cross = tNow;
            fprintf('\n>>> First cross at t=%.1f s. Countdown: %d s + %d cycles.\n\n', ...
                tNow, T_STEADY_AFTER_X, MIN_HEATER_CYCLES);
        end

        % Print every 5 s
        if mod(k, 25) == 0
            fprintf('t=%6.1fs | T=%6.2f | %s | cycles=%d\n', tNow, Traw, sStr, heaterOffCount);
        end

        % Checkpoint CSV every 10 s
        if (tNow - lastSave) >= 10 && k >= 10
            try
                writetable(table(logTime(1:k), logT(1:k), logTfilt(1:k), ...
                    T_amb*ones(k,1), SETPOINT*ones(k,1), logH(1:k), logF(1:k), logS(1:k), ...
                    'VariableNames',{'t_s','T_C','T_filt_C','T_amb_C','SP_C','heater','fan','status'}), CSV_FILE);
                lastSave = tNow;
            catch
            end
        end

        % --- STOP CONDITIONS ---
        if tNow >= T_END
            stopReason = sprintf('T_END=%d s reached', T_END);
            break;
        end
        if ~isnan(T_first_cross) && (tNow - T_first_cross) >= T_STEADY_AFTER_X ...
                && heaterOffCount >= MIN_HEATER_CYCLES
            stopReason = sprintf('Steady state confirmed: %.0f s after first cross, %d heater cycles', ...
                                 tNow-T_first_cross, heaterOffCount);
            break;
        end

        pause(T_SAMPLE);
    end
catch ME
    stopReason = sprintf('Ctrl+C or error: %s', ME.message);
    fprintf('\nLoop interrupted: %s\n', ME.message);
end
fprintf('\n>>> STOPPED: %s\n', stopReason);

%% Relay off
try
    writeDigitalPin(a, heaterRelay, relayOFF);
    writeDigitalPin(a, fanRelay, relayOFF);
    fprintf('Relays OFF.\n');
catch
    fprintf('!! Could not confirm relays OFF. UNPLUG ARDUINO USB. !!\n');
end

%% Final CSV save
if k >= 5
    try
        writetable(table(logTime(1:k), logT(1:k), logTfilt(1:k), ...
            T_amb*ones(k,1), SETPOINT*ones(k,1), logH(1:k), logF(1:k), logS(1:k), ...
            'VariableNames',{'t_s','T_C','T_filt_C','T_amb_C','SP_C','heater','fan','status'}), CSV_FILE);
        fprintf('Raw CSV saved: %s (%d samples)\n', CSV_FILE, k);
    catch ME
        fprintf('CSV save failed: %s\n', ME.message);
    end
end

%% Plot
if k >= 5
    try
        f = figure('Visible','off');
        subplot(2,1,1);
        plot(logTime(1:k), logT(1:k), 'b.', 'MarkerSize', 3); hold on;
        plot(logTime(1:k), logTfilt(1:k), 'b-', 'LineWidth', 1);
        yline(SETPOINT,'k--','SP');
        yline(SETPOINT+HYSTERESIS,'r:','SP+h');
        yline(SETPOINT-HYSTERESIS,'r:','SP-h');
        yline(T_amb,'g:','T_{amb}');
        grid on; xlabel('t (s)'); ylabel('T (C)');
        title(sprintf('ON/OFF  SP=%.1f  h=%.2f', SETPOINT, HYSTERESIS));
        subplot(2,1,2);
        stairs(logTime(1:k), logH(1:k), 'r','LineWidth',1); hold on;
        stairs(logTime(1:k), logF(1:k), 'b','LineWidth',1);
        ylim([-0.1 1.2]); grid on; xlabel('t (s)'); legend('Heater','Fan');
        saveas(f, [RUN_TAG '.png']); close(f);
        fprintf('Plot saved: %s.png\n', RUN_TAG);
    catch
    end
end

%% Metrics (plain text — guaranteed)
if k >= 20
    t = logTime(1:k); Tf = logTfilt(1:k);
    hOn = logH(1:k); fOn = logF(1:k); s = logS(1:k);
    Ts = median(diff(t)); tEnd = t(end);
    e = Tf - SETPOINT; e(isnan(e)) = 0;

    iX = find(Tf >= SETPOINT, 1, 'first');
    if isempty(iX), riseTime = NaN; else, riseTime = t(iX); end
    [Tpeak, ipk] = max(Tf);
    overshoot = Tpeak - SETPOINT; overshootPct = 100*overshoot/SETPOINT;
    peakTime = t(ipk);
    out = find(abs(Tf - SETPOINT) > 0.5);
    if isempty(out), settleTime = 0;
    elseif out(end) == numel(t), settleTime = NaN;
    else, settleTime = t(out(end)+1); end

    iSS = t >= 0.7*tEnd;
    Tss = Tf(iSS); tSS = t(iSS); TssClean = Tss(~isnan(Tss));
    ssMean = mean(TssClean); ssErr = ssMean - SETPOINT;
    ssStd = std(TssClean); ssP2P = max(TssClean)-min(TssClean);

    TssF = Tf(iSS); tSSF = t(iSS); pkIdx = [];
    for ii = 2:numel(TssF)-1
        if ~isnan(TssF(ii)) && ~isnan(TssF(ii-1)) && ~isnan(TssF(ii+1))
            if TssF(ii) >= TssF(ii-1) && TssF(ii) >= TssF(ii+1) && TssF(ii) > SETPOINT - HYSTERESIS/2
                pkIdx(end+1) = ii; %#ok<AGROW>
            end
        end
    end
    if numel(pkIdx) > 1
        dPk = diff(tSSF(pkIdx)); dPk = dPk(dPk >= 5);
        if isempty(dPk), oscP = NaN; oscF = NaN;
        else, oscP = mean(dPk); oscF = 1/oscP; end
    else, oscP = NaN; oscF = NaN; end

    IAE = sum(abs(e))*Ts; ISE = sum(e.^2)*Ts; ITAE = sum(t.*abs(e))*Ts;
    RMSE = sqrt(mean(e.^2));
    D_h = mean(hOn); D_f = mean(fOn); D_0 = mean(s==0);
    nH = sum(diff([0; hOn]) > 0); nF = sum(diff([0; fOn]) > 0);
    swRate = (nH+nF)/(tEnd/3600);
    tAbove1 = sum(Tf > SETPOINT+1)*Ts; tBelow1 = sum(Tf < SETPOINT-1)*Ts;

    fprintf('\n===== METRICS =====\n');
    fprintf('SP %.2f C | Tamb %.2f C | Duration %.1f s | Samples %d\n', SETPOINT, T_amb, tEnd, k);
    fprintf('RiseTime  %.1f s\n', riseTime);
    fprintf('PeakTime  %.1f s\n', peakTime);
    fprintf('PeakT     %.2f C\n', Tpeak);
    fprintf('Overshoot %.2f C (%.1f%%)\n', overshoot, overshootPct);
    fprintf('Settle    %.1f s\n', settleTime);
    fprintf('SSMean    %.2f C\n', ssMean);
    fprintf('SSError   %+.3f C\n', ssErr);
    fprintf('SSStd     %.3f C\n', ssStd);
    fprintf('SSP2P     %.3f C\n', ssP2P);
    fprintf('OscPeriod %.1f s\n', oscP);
    fprintf('OscFreq   %.3f Hz\n', oscF);
    fprintf('IAE %.2f | ISE %.2f | ITAE %.2f | RMSE %.3f\n', IAE, ISE, ITAE, RMSE);
    fprintf('HeaterDuty %.1f%% | FanDuty %.1f%% | Idle %.1f%%\n', 100*D_h, 100*D_f, 100*D_0);
    fprintf('HeaterON %d | FanON %d | SwitchRate %.1f /hr\n', nH, nF, swRate);
    fprintf('Tmax %.2f C | Tmin %.2f C\n', max(Tf), min(Tf));
    fprintf('TimeAboveSP1 %.1f s | TimeBelowSP1 %.1f s\n', tAbove1, tBelow1);
    fprintf('===================\n');

    % --- write metrics as plain CSV, no table quirks ---
    try
        fid = fopen(SUM_FILE, 'w');
        fprintf(fid, 'Key,Value,Unit\n');
        fprintf(fid, 'Controller,ON/OFF,\n');
        fprintf(fid, 'SP_C,%.2f,C\n', SETPOINT);
        fprintf(fid, 'Tamb_C,%.2f,C\n', T_amb);
        fprintf(fid, 'Hysteresis_C,%.2f,C\n', HYSTERESIS);
        fprintf(fid, 'Ts_s,%.3f,s\n', Ts);
        fprintf(fid, 'Duration_s,%.1f,s\n', tEnd);
        fprintf(fid, 'Samples,%d,\n', k);
        fprintf(fid, 'StopReason,"%s",\n', stopReason);
        fprintf(fid, 'RiseTime_s,%.2f,s\n', riseTime);
        fprintf(fid, 'PeakTime_s,%.2f,s\n', peakTime);
        fprintf(fid, 'PeakT_C,%.3f,C\n', Tpeak);
        fprintf(fid, 'Overshoot_C,%.3f,C\n', overshoot);
        fprintf(fid, 'Overshoot_pct,%.2f,%%\n', overshootPct);
        fprintf(fid, 'SettleTime_s,%.2f,s\n', settleTime);
        fprintf(fid, 'SSMean_C,%.3f,C\n', ssMean);
        fprintf(fid, 'SSError_C,%.3f,C\n', ssErr);
        fprintf(fid, 'SSStd_C,%.3f,C\n', ssStd);
        fprintf(fid, 'SSP2P_C,%.3f,C\n', ssP2P);
        fprintf(fid, 'OscPeriod_s,%.2f,s\n', oscP);
        fprintf(fid, 'OscFreq_Hz,%.4f,Hz\n', oscF);
        fprintf(fid, 'IAE,%.3f,C*s\n', IAE);
        fprintf(fid, 'ISE,%.3f,C^2*s\n', ISE);
        fprintf(fid, 'ITAE,%.3f,C*s^2\n', ITAE);
        fprintf(fid, 'RMSE_C,%.3f,C\n', RMSE);
        fprintf(fid, 'HeaterDuty_pct,%.2f,%%\n', 100*D_h);
        fprintf(fid, 'FanDuty_pct,%.2f,%%\n', 100*D_f);
        fprintf(fid, 'Idle_pct,%.2f,%%\n', 100*D_0);
        fprintf(fid, 'HeaterONCount,%d,\n', nH);
        fprintf(fid, 'FanONCount,%d,\n', nF);
        fprintf(fid, 'SwitchRate_perHr,%.2f,/hr\n', swRate);
        fprintf(fid, 'Tmax_C,%.3f,C\n', max(Tf));
        fprintf(fid, 'Tmin_C,%.3f,C\n', min(Tf));
        fprintf(fid, 'TimeAboveSP1_s,%.2f,s\n', tAbove1);
        fprintf(fid, 'TimeBelowSP1_s,%.2f,s\n', tBelow1);
        fclose(fid);
        fprintf('Metrics saved: %s\n', SUM_FILE);
    catch ME
        fprintf('Metrics save failed: %s\n', ME.message);
    end
end

fprintf('\n=== DONE. Change SETPOINT and re-run for next setpoint. ===\n');
diary off;
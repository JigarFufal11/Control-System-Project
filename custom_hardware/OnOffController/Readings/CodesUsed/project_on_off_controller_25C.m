%% ============================================================
%  ON/OFF Controller - Predictive Cutoff (Perfect Tracking)
%  Change SETPOINT only. Auto-stops at steady state.
% ============================================================
close all; clear; clc;

%% USER CONFIG
SETPOINT          = 25.0;   % <-- Change to 25.0, 30.0, or 35.0
HYSTERESIS        = 1.0;    % Total band = 1.0
T_SAMPLE          = 0.2;    
T_END             = 900;    
T_STEADY_AFTER_X  = 200;    
MIN_HEATER_CYCLES = 5;      
T_MAX_SAFE        = 45.0;   
COM_PORT          = 'COM21';

relayON = 0; relayOFF = 1;  

%% Console log to disk
logFile = sprintf('console_SP%d_%s.txt', round(SETPOINT), datestr(now,'yyyymmdd_HHMMSS'));
diary(logFile);

%% Connect
fprintf('\n=== ON/OFF Controller (Predictive Cutoff) ===\n');
fprintf('SP=%.1f C, h=%.2f C, Ts=%.2f s\n\n', SETPOINT, HYSTERESIS, T_SAMPLE);

a = arduino(COM_PORT, 'Mega2560');
sensorPin = 'A1'; heaterRelay = 'D6'; fanRelay = 'D9';
configurePin(a, heaterRelay, 'DigitalOutput');
configurePin(a, fanRelay,    'DigitalOutput');
writeDigitalPin(a, heaterRelay, relayOFF);
writeDigitalPin(a, fanRelay,    relayOFF);
heaterState = relayOFF; fanState = relayOFF;

%% Ambient
fprintf('Measuring ambient (2 s)...\n');
pause(2);
ambS = zeros(20,1);
for i = 1:20
    ambS(i) = readVoltage(a, sensorPin)*100;
    pause(0.1);
end
T_amb = mean(ambS(~isnan(ambS)));
if isnan(T_amb) || T_amb < -10 || T_amb > 60, T_amb = 20.0; end
fprintf('T_amb = %.2f C\n\n', T_amb);

%% Pre-allocate
N_max    = ceil(T_END/T_SAMPLE) + 500;
logTime  = zeros(N_max,1); logT = zeros(N_max,1);
logTfilt = zeros(N_max,1); logH = zeros(N_max,1);
logF     = zeros(N_max,1); logS = zeros(N_max,1);
tBuf     = nan(5,1);   
k = 0; lastSave = 0; heaterOffCount = 0; T_first_cross = NaN;

% Define filenames for both CSV and MAT
RUN_TAG  = sprintf('onoff_SP%d_h%0.2f_predictive_%s', round(SETPOINT), HYSTERESIS, datestr(now,'yyyymmdd_HHMMSS'));
CSV_FILE = [RUN_TAG '.csv'];
MAT_FILE = [RUN_TAG '.mat'];
SUM_FILE = [RUN_TAG '_metrics.csv'];
SUM_MAT  = [RUN_TAG '_metrics.mat'];

fprintf('Data:    %s\n', CSV_FILE);
fprintf('Metrics: %s\n\n', SUM_FILE);

%% Main loop
t0 = tic; stopReason = 'normal end';
try
    while true
        tNow = toc(t0);

        % --- read & filter ---
        Traw = readVoltage(a, sensorPin)*100;
        if isnan(Traw) || Traw < -20 || Traw > 80
            Tf = NaN;
        else
            tBuf = [tBuf(2:end); Traw];
            Tf   = mean(tBuf(~isnan(tBuf)));
        end

        % --- safety cutoff ---
        if ~isnan(Tf) && Tf > T_MAX_SAFE
            writeDigitalPin(a, heaterRelay, relayOFF);
            writeDigitalPin(a, fanRelay,    relayON);
            stopReason = sprintf('SAFETY CUTOFF T=%.1f C', Tf);
            break;
        end

        % --- PREDICTIVE ON/OFF LOGIC ---
        if isnan(Tf)
            newH=relayOFF; newF=relayOFF; sCode=0; sStr='BAD ';
        elseif Tf < (SETPOINT - HYSTERESIS/2)
            % Force heat if well below setpoint
            newH=relayON;  newF=relayOFF; sCode= 1; sStr='HEAT';
        elseif Tf > (SETPOINT + HYSTERESIS/2)
            % Force cool if well above setpoint
            newH=relayOFF; newF=relayON;  sCode=-1; sStr='COOL';
        else
            % Inside the band: Use predictive cutoff to prevent overshoot
            if k > 5
                % Calculate rate of change (dT/dt) in C/s
                dT = (Tf - logTfilt(k-5)) / (tNow - logTime(k-5));
            else
                dT = 0;
            end
            
            % Predict temperature 15 seconds into the future
            T_pred = Tf + dT * 15; 
            
            if T_pred > SETPOINT
                % Predicted to overshoot -> Turn heater OFF early
                newH=relayOFF; newF=relayOFF; sCode= 0; sStr='IDLE'; 
            elseif Tf < SETPOINT - 0.3
                % Safe to heat
                newH=relayON;  newF=relayOFF; sCode= 1; sStr='HEAT';
            else
                % Holding band
                newH=relayOFF; newF=relayOFF; sCode= 0; sStr='IDLE';
            end
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

        if isnan(T_first_cross) && ~isnan(Tf) && Tf >= (SETPOINT - HYSTERESIS/2)
            T_first_cross = tNow;
            fprintf('\n>>> First cross at t=%.1f s.\n\n', tNow);
        end

        if mod(k, 25) == 0
            fprintf('t=%6.1fs | T=%6.2f | SP=%.1f | %s | cycles=%d\n', tNow, Traw, SETPOINT, sStr, heaterOffCount);
        end

        % --- Checkpoint Save (Every 10s) ---
        if (tNow - lastSave) >= 10 && k >= 10
            try
                T_out = table(logTime(1:k), logT(1:k), logTfilt(1:k), ...
                    T_amb*ones(k,1), SETPOINT*ones(k,1), logH(1:k), logF(1:k), logS(1:k), ...
                    'VariableNames',{'t_s','T_C','T_filt_C','T_amb_C','SP_C','heater','fan','status'});
                writetable(T_out, CSV_FILE); 
                save(MAT_FILE, 'T_out');     
                lastSave = tNow;
            catch, end
        end

        % --- STOP CONDITIONS ---
        if tNow >= T_END
            stopReason = sprintf('T_END=%d s reached', T_END);
            break;
        end
        if ~isnan(T_first_cross) && (tNow - T_first_cross) >= T_STEADY_AFTER_X && heaterOffCount >= MIN_HEATER_CYCLES
            stopReason = sprintf('Steady state confirmed');
            break;
        end

        pause(T_SAMPLE);
    end
catch ME
    stopReason = sprintf('Interrupted: %s', ME.message);
end
fprintf('\n>>> STOPPED: %s\n', stopReason);

%% Relay off
try
    writeDigitalPin(a, heaterRelay, relayOFF);
    writeDigitalPin(a, fanRelay, relayOFF);
catch, end

%% Final Data Save
if k >= 5
    try
        T_out = table(logTime(1:k), logT(1:k), logTfilt(1:k), ...
            T_amb*ones(k,1), SETPOINT*ones(k,1), logH(1:k), logF(1:k), logS(1:k), ...
            'VariableNames',{'t_s','T_C','T_filt_C','T_amb_C','SP_C','heater','fan','status'});
        writetable(T_out, CSV_FILE);
        save(MAT_FILE, 'T_out');
        fprintf('Raw files saved: %s\n', RUN_TAG);
    catch ME
        fprintf('Save failed: %s\n', ME.message);
    end
end

%% Metrics calculation & save
if k >= 20
    t = logTime(1:k); Tf = logTfilt(1:k);
    hOn = logH(1:k); fOn = logF(1:k); s = logS(1:k);
    Ts = median(diff(t)); tEnd = t(end); T0 = Tf(1);
    
    % Rise Time
    step_size = SETPOINT - T0;
    t10_idx = find((Tf - T0) >= 0.1 * step_size, 1, 'first');
    t90_idx = find((Tf - T0) >= 0.9 * step_size, 1, 'first');
    if isempty(t10_idx) || isempty(t90_idx), riseTime = NaN; else, riseTime = t(t90_idx) - t(t10_idx); end

    [Tpeak, ipk] = max(Tf); peakTime = t(ipk);
    overshoot = Tpeak - SETPOINT; overshootPct = 100*overshoot/SETPOINT;
    
    out = find(abs(Tf - SETPOINT) > 0.6);
    if isempty(out), settleTime = 0; elseif out(end) == numel(t), settleTime = NaN; else, settleTime = t(out(end)+1); end

    iSS = t >= 0.7*tEnd; Tss = Tf(iSS); 
    ssMean = mean(Tss); ssErr = ssMean - SETPOINT; ssStd = std(Tss); ssP2P = max(Tss)-min(Tss);
    D_h = mean(hOn); D_f = mean(fOn); D_0 = mean(s==0);

    % Create Metrics Table
    Metrics = table("ON/OFF Predictive", SETPOINT, T_amb, HYSTERESIS, tEnd, ...
                    riseTime, settleTime, overshootPct, Tpeak, ...
                    ssMean, ssErr, ssStd, ssP2P, ...
                    100*D_h, 100*D_f, 100*D_0, ...
        'VariableNames', {'Controller','SP_C','Tamb_C','Hysteresis_C','Duration_s', ...
                          'RiseTime_s','SettleTime_s','Overshoot_pct','PeakT_C', ...
                          'SSMean_C','SSError_C','SSStd_C','SSP2P_C', ...
                          'HeaterDuty_pct','FanDuty_pct','Idle_pct'});

    % Save Metrics as both CSV and MAT
    writetable(Metrics, SUM_FILE);
    save(SUM_MAT, 'Metrics');
    fprintf('Metrics saved: %s\n', RUN_TAG);
end

fprintf('\n=== DONE. ===\n');
diary off;
%% Ziegler-Nichols PI Auto-Tuning for HVAC (Strict 0.6°C Band)
%  Target: ±0.3°C around setpoint (Total 0.6°C band)
%  Heater ON <= 34.7°C | Heater OFF >= 35.0°C
%  Fan ON    >= 35.3°C | Fan OFF    <= 35.0°C
close all; clear; clc;

%% 1. Arduino setup
portName    = 'COM19';             % <-- change if needed
a           = arduino(portName, 'Mega2560');
sensorPin   = 'A1';
heaterRelay = 'D6';
fanRelay    = 'D9';
configurePin(a, heaterRelay, 'DigitalOutput');
configurePin(a, fanRelay,    'DigitalOutput');

relayON  = 0;    % active-low relay
relayOFF = 1;
writeDigitalPin(a, heaterRelay, relayOFF);
writeDigitalPin(a, fanRelay,    relayOFF);

%% 2. Tuning parameters (0.6°C TOTAL BAND)
setpoint    = 35.0;    % °C
bandBelow   = 0.3;     % Heater kicks in at 34.7°C
bandAbove   = 0.3;     % Fan kicks in at 35.3°C
Ts          = 0.5;     % s   sampling time
minDwell    = 3.0;     % s   MUST wait this long between relay clicks (hardware safety)

% --- adaptive stop conditions ---
minCycles       = 5;       
periodCVthresh  = 0.05;    
ampCVthresh     = 0.10;    
minRunTime      = 120;     
safetyMaxTime   = 1800;    

% Preallocate
maxSamples   = round(safetyMaxTime / Ts) + 100;
t_log = zeros(1, maxSamples);
T_log = zeros(1, maxSamples);
T_raw_log = zeros(1, maxSamples);
u_log = zeros(1, maxSamples);

updateEvery = 10;           
saveEvery   = 60;           
csvName     = 'zn_tuning_data_0.6C.csv';

%% 3. Live figure
figure('Name','ZN Relay Tuning - 0.6°C Band','Color','w');

ax1 = subplot(2,1,1); hold(ax1,'on'); grid(ax1,'on');
hRaw = plot(ax1, nan, nan, 'Color', [0.7 0.7 1], 'LineWidth', 1); % Raw ADC jumps
hT   = plot(ax1, nan, nan, 'b-',  'LineWidth', 1.5);              % Filtered
hRef = plot(ax1, nan, nan, 'r--', 'LineWidth', 1.0);
hEd  = plot(ax1, nan, nan, 'g^', 'MarkerSize', 8, 'MarkerFaceColor','g');
hB1  = plot(ax1, [0 maxSamples*Ts], [setpoint-bandBelow setpoint-bandBelow], 'k:', 'LineWidth', 1);
hB2  = plot(ax1, [0 maxSamples*Ts], [setpoint+bandAbove setpoint+bandAbove], 'k:', 'LineWidth', 1);
xlabel(ax1,'Time [s]'); ylabel(ax1,'Temperature [°C]');
legend(ax1, {'Raw (Noisy)','Filtered','Setpoint','Rising edges','Bands'}, 'Location','best');
title(ax1,'ZN Relay Experiment - 0.6°C Total Band');

ax2 = subplot(2,1,2); hold(ax2,'on'); grid(ax2,'on');
hU = stairs(ax2, nan, nan, 'k-', 'LineWidth', 1.2);
xlabel(ax2,'Time [s]'); ylabel(ax2,'Relay cmd');
ylim(ax2,[-1.3 1.3]); yticks(ax2,[-1 0 1]);

%% 4. Safe shutdown hook
cleanup = onCleanup(@() shutdown_relays(a, heaterRelay, fanRelay, relayOFF));

%% 5. Relay experiment
fprintf('Running ZN relay experiment (Total 0.6C Band)...\n');
relayState = 0;      
lastSwitchTime = -999; % Allows immediate switch at t=0
riseEdges  = [];     
tuned      = false;
T_filt     = readVoltage(a, sensorPin) * 100; % Initialize filter
alpha      = 0.9; % High smoothing to interpolate 0.5°C ADC steps
k_end      = 0;

try
    for k = 1:maxSamples
        k_end = k;
        T_raw = readVoltage(a, sensorPin) * 100;   
        
        % --- EMA Filter (crucial for 0.6°C band) ---
        T_filt = alpha * T_filt + (1 - alpha) * T_raw;
        
        T_raw_log(k) = T_raw;
        T_log(k)     = T_filt;
        t_log(k)     = (k-1)*Ts;

        timeSinceSwitch = t_log(k) - lastSwitchTime;

        % --- Relay Logic with 0.3°C Bands ---
        if timeSinceSwitch >= minDwell
            if relayState == +1 && T_filt >= setpoint
                relayState = 0; % Reached 35.0, turn heater OFF
                lastSwitchTime = t_log(k);
            elseif relayState == -1 && T_filt <= setpoint
                relayState = 0; % Cooled to 35.0, turn fan OFF
                lastSwitchTime = t_log(k);
            elseif relayState == 0
                if T_filt <= setpoint - bandBelow
                    relayState = +1; % Dropped to 34.7, turn heater ON
                    lastSwitchTime = t_log(k);
                elseif T_filt >= setpoint + bandAbove
                    relayState = -1; % Rose to 35.3, turn fan ON
                    lastSwitchTime = t_log(k);
                end
            end
        end

        % Apply hardware states
        u = relayState;
        if u == +1
            writeDigitalPin(a, heaterRelay, relayON);
            writeDigitalPin(a, fanRelay,    relayOFF);
        elseif u == -1
            writeDigitalPin(a, heaterRelay, relayOFF);
            writeDigitalPin(a, fanRelay,    relayON);
        else
            writeDigitalPin(a, heaterRelay, relayOFF);
            writeDigitalPin(a, fanRelay,    relayOFF);
        end

        u_log(k) = u;

        % --- detect rising edge (enter heater-ON state) ---
        if k > 1 && u == +1 && u_log(k-1) ~= +1
            riseEdges(end+1) = t_log(k);   %#ok<SAGROW>

            % --- check stability ---
            if numel(riseEdges) >= minCycles + 1 && t_log(k) >= minRunTime
                recentEdges = riseEdges(end-minCycles:end);
                periods     = diff(recentEdges);

                if std(periods)/mean(periods) < periodCVthresh
                    amps = zeros(1, minCycles);
                    for c = 1:minCycles
                        segIdx = (t_log(1:k) >= recentEdges(c)) & ...
                                 (t_log(1:k) <= recentEdges(c+1));
                        segT   = T_log(segIdx);
                        amps(c)= max(segT) - min(segT);
                    end

                    if std(amps)/mean(amps) < ampCVthresh
                        tuned      = true;
                        fprintf('\nDetected %d consistent cycles!\n', minCycles);
                        break;
                    end
                end
            end
        end

        % --- live plot ---
        if mod(k, updateEvery) == 0
            set(hRaw, 'XData', t_log(1:k),   'YData', T_raw_log(1:k));
            set(hT,   'XData', t_log(1:k),   'YData', T_log(1:k));
            set(hRef, 'XData', t_log(1:k),   'YData', setpoint*ones(1,k));
            set(hU,   'XData', t_log(1:k),   'YData', u_log(1:k));
            if ~isempty(riseEdges)
                set(hEd, 'XData', riseEdges, ...
                         'YData', interp1(t_log(1:k), T_log(1:k), riseEdges, 'linear', 'extrap'));
            end
            drawnow limitrate;
        end

        if mod(k, saveEvery) == 0
            write_snapshot(csvName, t_log(1:k), T_raw_log(1:k), T_log(1:k), u_log(1:k));
        end

        if t_log(k) >= safetyMaxTime
            break;
        end

        pause(Ts);
    end
catch ME
    fprintf('Interrupted: %s\n', ME.message);
end

% Trim and save
t_log = t_log(1:k_end); T_log = T_log(1:k_end); T_raw_log = T_raw_log(1:k_end); u_log = u_log(1:k_end);
write_snapshot(csvName, t_log, T_raw_log, T_log, u_log);

%% 6. Post-processing
startIdx = max(round(0.3*numel(T_log)), 1);
T_seg = T_log(startIdx:end);
t_seg = t_log(startIdx:end);
u_seg = u_log(startIdx:end);

edges = find( u_seg(2:end) == 1 & u_seg(1:end-1) ~= 1 ) + 1;
if numel(edges) < 3
    error('Not enough oscillation cycles. Check connections.');
end

per = diff(t_seg(edges));
Pu  = median(per);
Ae  = (max(T_seg) - min(T_seg)) / 2;
A   = 0.5; 

Ku = (4*A) / (pi * max(Ae, eps));

Kp = 0.45 * Ku;
Ti = Pu   / 1.2;
Ki = Kp   / Ti;

fprintf('\n===== ZN PI TUNING RESULTS (0.6°C BAND) =====\n');
fprintf('Ultimate period   Pu = %.4f s\n', Pu);
fprintf('Temp amplitude    Ae = %.4f C\n', Ae);
fprintf('Ultimate gain     Ku = %.5f\n', Ku);
fprintf('Proportional      Kp = %.5f\n', Kp);
fprintf('Integral time     Ti = %.4f s\n', Ti);
fprintf('Integral gain     Ki = %.6f\n', Ki);
fprintf('=============================================\n');

save('pi_gains.mat','Kp','Ki','Ku','Pu','Ae','Ti');

%% ---- local helpers ----
function write_snapshot(fname, t, Traw, Tfilt, u)
    Ttbl = table(t(:), Traw(:), Tfilt(:), u(:), ...
                 'VariableNames', {'time_s','temp_raw_C','temp_filt_C','relay_cmd'});
    writetable(Ttbl, fname);
end
function shutdown_relays(a, h, f, off)
    try
        writeDigitalPin(a, h, off); writeDigitalPin(a, f, off);
    catch; end
end
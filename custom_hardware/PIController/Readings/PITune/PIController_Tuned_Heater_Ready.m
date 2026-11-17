%% Final PI Temperature Controller for HVAC (Corrected)
%  Uses ZN tuned gains: Kp = 0.44178, Ki = 0.009909
%  Anti-windup fixed. Includes Relay Dwell Time to prevent chatter.
close all; clear; clc;

%% 1. Tuned PI Gains
Kp = 0.44178;  
Ki = 0.009909; 

%% 2. Arduino setup
portName    = 'COM19';   % <-- change if needed
a           = arduino(portName, 'Mega2560');
sensorPin   = 'A1';
heaterRelay = 'D6';
fanRelay    = 'D9';
configurePin(a, heaterRelay, 'DigitalOutput');
configurePin(a, fanRelay,    'DigitalOutput');

relayON  = 0;    % active-low
relayOFF = 1;
writeDigitalPin(a, heaterRelay, relayOFF);
writeDigitalPin(a, fanRelay,    relayOFF);

%% 3. Control Parameters
setpoint    = 20.0;    % °C
Ts          = 1.0;     % s

% --- Auto-stop (Settle Detection) ---
settleBand    = 0.3;   
settleTime    = 60;    
minRunTime    = 120;   
safetyMaxTime = 3600;  

% --- Actuator Deadband ---
u_deadband    = 0.05;  

% --- Anti-Windup (CRITICAL FIX) ---
% We clamp the integral much tighter. This prevents the heater 
% from staying locked ON when it reaches the setpoint.
I_clamp       = 0.5;   
minDwell      = 2.0;   % s (Relay must wait 2s before switching again)

% --- Logging ---
maxSamples  = round(safetyMaxTime/Ts) + 100;
t_log   = zeros(1,maxSamples);
T_log   = zeros(1,maxSamples);
e_log   = zeros(1,maxSamples);
u_log   = zeros(1,maxSamples);
I_log   = zeros(1,maxSamples);
rly_log = zeros(1,maxSamples);
csvName = 'pi_control_data.csv';

%% 4. Live Figure Setup
figure('Name','Final PI Control','Color','w');

ax1 = subplot(2,1,1); hold(ax1,'on'); grid(ax1,'on');
hT   = plot(ax1, nan, nan, 'b-',  'LineWidth', 1.5);
hRef = plot(ax1, nan, nan, 'r--', 'LineWidth', 1.0);
hBan1= plot(ax1, nan, nan, 'm:', 'LineWidth', 1.0); 
hBan2= plot(ax1, nan, nan, 'm:', 'LineWidth', 1.0); 
ylabel(ax1,'Temperature [°C]');
title(ax1,'PI Controller - Maintaining 35°C ± 0.3°C');
legend(ax1, {'Measured','Setpoint','+0.3 band','-0.3 band'}, 'Location','best');

ax2 = subplot(2,1,2); hold(ax2,'on'); grid(ax2,'on');
hU = plot(ax2, nan, nan, 'k-', 'LineWidth', 1.2);
yline(ax2, u_deadband, 'r--', 'Deadband');
yline(ax2, -u_deadband, 'r--');
ylabel(ax2,'Control effort (u)'); xlabel(ax2,'Time [s]');
title(ax2,'PI Output: u = Kp*e + I');

%% 5. Safe Shutdown Hook
cleanup = onCleanup(@() shutdown_relays(a, heaterRelay, fanRelay, relayOFF));

%% 6. Control Loop State
I          = 0;
relayState = 0;
lastSwitchTime = -999;
T_filt     = readVoltage(a, sensorPin) * 100;
alpha      = 0.8;  % Filter for LM35 noise
k_end      = 0;
stopReason = '';

fprintf('Running Corrected PI Control...\n');

try
    for k = 1:maxSamples
        k_end = k;
        
        % 1. Read and Filter Temperature
        T_raw  = readVoltage(a, sensorPin) * 100;
        T_filt = alpha * T_filt + (1 - alpha) * T_raw;
        e      = setpoint - T_filt;

        % 2. Integral Action (Proper Anti-Windup)
        I = I + Ki * e * Ts;
        I = max(min(I, I_clamp), -I_clamp);

        % 3. PI Control Law
        u = Kp * e + I;

        % 4. Relay Decision (with Deadband and Dwell Time)
        timeSinceSwitch = (k-1)*Ts - lastSwitchTime;
        
        if timeSinceSwitch >= minDwell
            if u > u_deadband
                relayState = +1; % Heater ON
                lastSwitchTime = (k-1)*Ts;
            elseif u < -u_deadband
                relayState = -1; % Fan ON
                lastSwitchTime = (k-1)*Ts;
            else
                relayState = 0;  % Both OFF
                lastSwitchTime = (k-1)*Ts;
            end
        end

        % Apply to Hardware
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
        t_log(k)   = (k-1)*Ts;
        T_log(k)   = T_filt;
        e_log(k)   = e;
        u_log(k)   = u;
        I_log(k)   = I;
        rly_log(k) = relayState;

        % 6. Auto-Stop Detection (Settled)
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

        % 7. Live Plot
        if mod(k, 3) == 0
            set(hT,   'XData', t_log(1:k), 'YData', T_log(1:k));
            set(hRef, 'XData', t_log(1:k), 'YData', setpoint*ones(1,k));
            set(hBan1,'XData', t_log(1:k), 'YData', (setpoint+settleBand)*ones(1,k));
            set(hBan2,'XData', t_log(1:k), 'YData', (setpoint-settleBand)*ones(1,k));
            set(hU,   'XData', t_log(1:k), 'YData', u_log(1:k));
            drawnow limitrate;
        end

        % 8. Periodic CSV Save
        if mod(k, 30) == 0
            write_snapshot(csvName, t_log(1:k), T_log(1:k), e_log(1:k), u_log(1:k), I_log(1:k), rly_log(1:k));
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
t_log = t_log(1:k_end); T_log = T_log(1:k_end); e_log = e_log(1:k_end);
u_log = u_log(1:k_end); I_log = I_log(1:k_end); rly_log = rly_log(1:k_end);
write_snapshot(csvName, t_log, T_log, e_log, u_log, I_log, rly_log);

%% 7. Run Summary
fprintf('\n===== PI Run Summary =====\n');
fprintf('Stop reason      : %s\n', stopReason);
fprintf('Total runtime    : %.1f s\n', t_log(end));
fprintf('Final Temp       : %.2f °C\n', T_log(end));
fprintf('Max |error|      : %.3f °C\n', max(abs(e_log)));
fprintf('RMSE             : %.3f °C\n', sqrt(mean(e_log.^2)));
fprintf('Data saved to    : %s\n', csvName);
fprintf('===========================\n');

%% ---- Local Helpers ----
function write_snapshot(fname, t, T, e, u, I, r)
    Ttbl = table(t(:), T(:), e(:), u(:), I(:), r(:), ...
                 'VariableNames', {'time_s','temperature_C','error_C','u','I','relay_cmd'});
    writetable(Ttbl, fname);
end

function shutdown_relays(a, h, f, off)
    try
        writeDigitalPin(a, h, off); writeDigitalPin(a, f, off);
        disp('Hardware safe: Relays OFF.');
    catch
    end
end
%% init_params.m — Initialize all workspace parameters for thermal control simulation
%  Run this script before opening/building the Simulink model.
%  All parameters are loaded into the MATLAB base workspace.
%
%  Author : Control Systems Project Team
%  Date   : November 2026
%  -----------------------------------------------------------------------

% clear; clc;
fprintf('=== Thermal Control System — Parameter Initialization ===\n\n');

%% ---- Physical Plant Parameters ----------------------------------------
P     = 50;        % Heater power [W]
T_amb = 25;        % Ambient temperature [°C]

% Placeholder model parameters (replace with fitted values after step test)
K     = 15;        % Static gain: steady-state ΔT per unit duty [°C]
tau   = 120;       % Time constant [s]
L     = 3;         % Transport delay (dead time) [s]

% Derived thermal parameters from the energy-balance model:
%   K = P·R_th  =>  R_th = K/P
%   τ = C·R_th  =>  C    = τ/R_th = τ·P/K
R_th  = K / P;             % Thermal resistance [°C/W]  (0.3)
C_th  = tau * P / K;       % Thermal capacitance [J/°C] (400)

fprintf('  Plant:  P = %.0f W,  T_amb = %.1f °C\n', P, T_amb);
fprintf('  Model:  K = %.1f °C,  tau = %.0f s,  L = %.0f s\n', K, tau, L);
fprintf('  Therm:  R_th = %.4f °C/W,  C_th = %.1f J/°C\n\n', R_th, C_th);

%% ---- Controller Parameters -------------------------------------------
% PI tuning using Internal Model Control (IMC) method:
%   tau_c = desired closed-loop time constant  (>= max(0.1·τ, L))
%   Kp    = τ / (K · (L + tau_c))
%   Ki    = Kp / τ

tau_c = max(0.1 * tau, L);   % Closed-loop time constant [s]
Kp    = tau / (K * (L + tau_c));   % Proportional gain
Ki    = Kp / tau;                  % Integral gain [1/s]
Ts    = 1;                         % Sample / control period [s]

fprintf('  IMC tuning (tau_c = %.1f s):\n', tau_c);
fprintf('    Kp = %.4f\n', Kp);
fprintf('    Ki = %.6f  (Ki·Ts = %.6f)\n\n', Ki, Ki*Ts);

%% ---- MATLAB pidtune (optional, requires Control System Toolbox) -------
try
    s = tf('s');
    G_plant = K * exp(-L*s) / (tau*s + 1);   % FOPTD model
    
    % Use Padé approximation for pidtune (it doesn't handle pure delay)
    G_pade = pade(G_plant, 2);               % 2nd-order Padé
    C_pi = pidtune(G_pade, 'PI');
    
    Kp_auto = C_pi.Kp;
    Ki_auto = C_pi.Ki;
    
    fprintf('  pidtune result:\n');
    fprintf('    Kp = %.4f\n', Kp_auto);
    fprintf('    Ki = %.6f\n\n', Ki_auto);
catch ME
    fprintf('  pidtune skipped: %s\n\n', ME.message);
end

%% ---- ON-OFF Controller Parameters ------------------------------------
hysteresis = 0.5;   % ±0.5 °C deadband around setpoint

%% ---- Reference Signal -------------------------------------------------
% Setpoint sequence: 30 → 40 → 35 °C
%   0   – 600 s :  30 °C
%   600 – 1200 s:  40 °C
%   1200– 1800 s:  35 °C
ref_times  = [0,  600,  600,  1200, 1200, 1800];
ref_values = [30,  30,   40,    40,   35,   35];
ref_ts = timeseries([30; 30; 40; 40; 35; 35], [0; 599; 600; 1199; 1200; 1800]);

%% ---- Disturbance (R_th step) ------------------------------------------
% +30% increase in R_th at t = 900 s to simulate fan slowdown / flap closure
R_th_nominal    = R_th;
R_th_disturbed  = R_th * 1.30;    % +30%
disturbance_time = 900;            % Time of disturbance onset [s]

%% ---- Simulation Settings ----------------------------------------------
t_final   = 1800;      % Total simulation time [s]
solver    = 'ode45';    % ODE solver

%% ---- Safety Limits (for reference) ------------------------------------
T_max     = 50;        % Hard safety cutoff [°C]
T_pla_max = 45;        % Soft limit for PLA chambers [°C]

%% ---- Fault Detection Thresholds ---------------------------------------
residual_threshold     = 3.0;    % |T_meas − T_pred| threshold [°C]
residual_duration      = 5;      % Consecutive seconds above threshold
sensor_disagree_thresh = 1.5;    % |T1 − T2| threshold [°C]
sensor_disagree_dur    = 5;      % Consecutive seconds above threshold

fprintf('  Simulation: t_final = %d s, Ts = %d s\n', t_final, Ts);
fprintf('  Safety:     T_max = %.0f °C\n', T_max);
fprintf('=== Parameters loaded successfully ===\n');

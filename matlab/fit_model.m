%% fit_model.m — Fit a first-order-plus-dead-time (FOPTD) model to step-test data
%
%  Reads a step-test CSV file and fits:
%    G(s) = K · exp(-L·s) / (τ·s + 1)
%
%  Two methods:
%    1. Manual: K = ΔT_ss/Δu, τ = time to 63.2%, L = initial flat time
%    2. System ID Toolbox: tfest with 1 pole + delay
%
%  Input CSV format: time, ref, T1, T2, u, fan_cmd, mode, flags
%
%  Author : Control Systems Project Team
%  -----------------------------------------------------------------------

clear; clc; close all;
fprintf('=== FOPTD Model Fitting from Step Test ===\n\n');

%% ---- Load Data ---------------------------------------------------------
% Select CSV file
if exist('../data/step1.csv', 'file')
    filepath = '../data/step1.csv';
elseif exist('step1.csv', 'file')
    filepath = 'step1.csv';
else
    [fname, fpath] = uigetfile('*.csv', 'Select step-test CSV file');
    if isequal(fname, 0)
        error('No CSV file selected and step1.csv not found.');
    else
        filepath = fullfile(fpath, fname);
    end
end

fprintf('  Loading: %s\n', filepath);
d = readmatrix(filepath);

% Parse columns: time, ref, T1, T2, u, fan_cmd, mode, flags
t_raw = d(:,1) / 1000;       % Convert ms to seconds (if in ms)
T1    = d(:,3);               % Sensor 1 temperature
T2    = d(:,4);               % Sensor 2 temperature
u_raw = d(:,5);               % Duty cycle (0-1)

% If time is already in seconds (values > 1000), don't convert
if t_raw(end) > 10000
    t_raw = d(:,1);
end

T_avg = (T1 + T2) / 2;       % Average temperature

% Normalize time to start at 0
t = t_raw - t_raw(1);

%% ---- Detect the step in u ----------------------------------------------
u_diff = abs(diff(u_raw));
step_idx = find(u_diff > 0.05, 1, 'first');
if isempty(step_idx)
    fprintf('  WARNING: No step detected in duty cycle. Using full data.\n');
    step_idx = 1;
end

% Data after the step
t_step = t(step_idx:end) - t(step_idx);
T_step = T_avg(step_idx:end);
u_step = u_raw(step_idx:end);

T_amb = T_step(1);            % Initial temperature
T_inf = T_step(end);          % Final (steady-state) temperature
delta_u = mean(u_step) - (step_idx > 1) * mean(u_raw(1:step_idx-1));

fprintf('  T_amb = %.2f °C\n', T_amb);
fprintf('  T_ss  = %.2f °C\n', T_inf);
fprintf('  Δu    = %.3f\n', delta_u);

%% ========================================================================
%  METHOD 1: Manual Fitting
%  ========================================================================
fprintf('\n--- Method 1: Manual Parameter Extraction ---\n');

delta_T = T_inf - T_amb;
K_manual = delta_T / delta_u;                 % Static gain [°C per unit duty]

% Time constant: time to reach 63.2% of the rise
T_63 = T_amb + 0.632 * delta_T;
idx_63 = find(T_step >= T_63, 1, 'first');
if ~isempty(idx_63)
    tau_manual = t_step(idx_63);
else
    tau_manual = NaN;
    fprintf('  WARNING: 63.2%% point not reached.\n');
end

% Dead time: time before response starts (T rises more than noise)
noise_level = std(T_step(1:min(10, length(T_step))));  % Noise estimate
if noise_level < 0.05, noise_level = 0.05; end
T_threshold = T_amb + 3 * noise_level;
idx_delay = find(T_step > T_threshold, 1, 'first');
if ~isempty(idx_delay)
    L_manual = t_step(idx_delay);
else
    L_manual = 0;
end

% Correct tau for the delay
tau_manual = tau_manual - L_manual;

fprintf('  K   = %.2f °C\n', K_manual);
fprintf('  tau = %.1f s\n', tau_manual);
fprintf('  L   = %.1f s\n', L_manual);

%% ========================================================================
%  METHOD 2: System Identification Toolbox (tfest)
%  ========================================================================
fprintf('\n--- Method 2: System ID Toolbox (tfest) ---\n');

try
    % Prepare identification data
    Ts_data = mean(diff(t));
    data = iddata(T_step - T_amb, u_step, Ts_data);
    data.InputName  = 'Duty';
    data.OutputName = 'Temperature Rise';
    
    % Fit a first-order transfer function with delay
    % 1 pole, 0 zeros, estimated I/O delay
    opt = tfestOptions('InitializeMethod', 'all');
    G_fit = tfest(data, 1, 0, opt);
    
    % If delay was not estimated, set initial guess and re-estimate
    if G_fit.IODelay < 0.1
        G_fit.IODelay = L_manual;  % Use manual estimate as starting point
        G_fit = tfest(data, 1, 0, 'Ts', 0);
    end
    
    % Extract parameters
    [num, den] = tfdata(G_fit, 'v');
    K_tfest   = num(end) / den(end);      % DC gain = b0/a0
    tau_tfest = 1 / (den(1)/den(end));     % Time constant (if den = [a1 a0])
    % For tf with den = [tau 1], tau = den(1)
    tau_tfest = den(1);
    L_tfest   = G_fit.IODelay;
    
    fprintf('  K   = %.2f °C\n', K_tfest);
    fprintf('  tau = %.1f s\n', tau_tfest);
    fprintf('  L   = %.1f s\n', L_tfest);
    fprintf('  Fit: %.1f%%\n', G_fit.Report.Fit.FitPercent);
    
    % Comparison plot
    figure('Name', 'Model Fit: Measured vs Estimated', 'Position', [100 100 800 500]);
    compare(data, G_fit);
    title('Step Response: Measured vs Fitted FOPTD Model');
    grid on;
    saveas(gcf, 'model_fit_compare.png');
    fprintf('  Plot saved: model_fit_compare.png\n');
    
catch ME
    fprintf('  tfest failed: %s\n', ME.message);
    fprintf('  Using manual values instead.\n');
    K_tfest = K_manual; tau_tfest = tau_manual; L_tfest = L_manual;
end

%% ========================================================================
%  COMPARISON PLOT (Manual fit)
%  ========================================================================
fprintf('\n--- Generating Manual Fit Plot ---\n');

% Simulate the manual FOPTD model
T_model = zeros(size(t_step));
for i = 1:length(t_step)
    if t_step(i) > L_manual
        t_eff = t_step(i) - L_manual;
        T_model(i) = T_amb + K_manual * delta_u * (1 - exp(-t_eff / tau_manual));
    else
        T_model(i) = T_amb;
    end
end

figure('Name', 'Manual FOPTD Fit', 'Position', [100 100 800 500]);
plot(t_step, T_step, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Measured');
hold on;
plot(t_step, T_model, 'r--', 'LineWidth', 1.5, 'DisplayName', ...
    sprintf('Manual FOPTD (K=%.1f, \\tau=%.0f, L=%.0f)', K_manual, tau_manual, L_manual));
xlabel('Time [s]');
ylabel('Temperature [°C]');
title('Step Test: Measured vs Manual FOPTD Fit');
legend('Location', 'best');
grid on;

saveas(gcf, 'model_fit_manual.png');
fprintf('  Plot saved: model_fit_manual.png\n');

%% ---- Summary Table ------------------------------------------------------
fprintf('\n=== Parameter Summary ===\n');
fprintf('  %-12s  %8s  %8s  %8s\n', 'Method', 'K [°C]', 'tau [s]', 'L [s]');
fprintf('  %-12s  %8.2f  %8.1f  %8.1f\n', 'Manual', K_manual, tau_manual, L_manual);
fprintf('  %-12s  %8.2f  %8.1f  %8.1f\n', 'tfest', K_tfest, tau_tfest, L_tfest);

%% ---- Save fitted parameters -------------------------------------------
K_fit = K_tfest; tau_fit = tau_tfest; L_fit = L_tfest;
save('fitted_params.mat', 'K_fit', 'tau_fit', 'L_fit', ...
     'K_manual', 'tau_manual', 'L_manual', ...
     'K_tfest', 'tau_tfest', 'L_tfest');
fprintf('\n  Parameters saved to fitted_params.mat\n');
fprintf('  Update init_params.m with these values for accurate simulations.\n');

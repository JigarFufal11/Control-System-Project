%% plot_overlay.m — Overlay simulation and hardware data for comparison
%
%  Loads simulation results (.mat) and hardware CSV data, plots them on
%  the same axes, and computes fit percentages.
%
%  Author : Control Systems Project Team
%  -----------------------------------------------------------------------

clear; clc; close all;
fprintf('=== Simulation vs Hardware Overlay Plots ===\n\n');

%% ---- Configuration (edit these paths) ----------------------------------
sim_mat_file = '../simulink/results_pi.mat';        % Simulation data
hw_csv_file  = '../data/pi_run.csv';                 % Hardware CSV

% Fallbacks
if ~exist(sim_mat_file, 'file')
    fprintf('  Simulation file not found: %s\n', sim_mat_file);
    [f, p] = uigetfile('*.mat', 'Select simulation .mat file');
    if isequal(f, 0), error('No file selected.'); end
    sim_mat_file = fullfile(p, f);
end

if ~exist(hw_csv_file, 'file')
    fprintf('  Hardware CSV not found: %s\n', hw_csv_file);
    [f, p] = uigetfile('*.csv', 'Select hardware CSV file');
    if isequal(f, 0)
        fprintf('  No hardware file — plotting simulation only.\n');
        hw_csv_file = '';
    else
        hw_csv_file = fullfile(p, f);
    end
end

%% ---- Load Simulation Data ----------------------------------------------
sim_data = load(sim_mat_file);
fn = fieldnames(sim_data);

% Try to find the standard variable names
t_sim = []; T_sim = []; u_sim = []; ref_sim = [];
for i = 1:length(fn)
    name = fn{i};
    val  = sim_data.(name);
    if contains(name, 't_'),   t_sim   = val; end
    if contains(name, 'T_') && ~contains(name, 'amb'), T_sim = val; end
    if contains(name, 'u_'),   u_sim   = val; end
    if contains(name, 'r_'),   ref_sim = val; end
end

fprintf('  Simulation: %d samples, %.0f s\n', length(t_sim), t_sim(end));

%% ---- Load Hardware Data ------------------------------------------------
if ~isempty(hw_csv_file)
    d = readmatrix(hw_csv_file);
    t_hw   = d(:,1) / 1000;            % ms → s (adjust if needed)
    ref_hw = d(:,2);
    T1_hw  = d(:,3);
    T2_hw  = d(:,4);
    u_hw   = d(:,5);
    T_hw   = (T1_hw + T2_hw) / 2;
    
    % Normalize time to start at 0
    t_hw = t_hw - t_hw(1);
    
    fprintf('  Hardware:   %d samples, %.0f s\n', length(t_hw), t_hw(end));
    has_hw = true;
else
    has_hw = false;
end

%% ========================================================================
%  PLOT 1: Temperature Overlay
%  ========================================================================
figure('Name', 'Sim vs HW: Temperature', 'Position', [50 50 1000 700]);

subplot(2,1,1);
plot(t_sim, ref_sim, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
hold on;
plot(t_sim, T_sim, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulation');
if has_hw
    plot(t_hw, T_hw, 'r-', 'LineWidth', 1.2, 'DisplayName', 'Hardware');
end
xlabel('Time [s]');
ylabel('Temperature [°C]');
title('Temperature: Simulation vs Hardware');
legend('Location', 'best');
grid on;

subplot(2,1,2);
plot(t_sim, u_sim, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulation');
hold on;
if has_hw
    plot(t_hw, u_hw, 'r-', 'LineWidth', 1, 'DisplayName', 'Hardware');
end
xlabel('Time [s]');
ylabel('Duty Cycle');
title('Control Signal: Simulation vs Hardware');
legend('Location', 'best');
grid on;
ylim([-0.1, 1.1]);

saveas(gcf, 'overlay_sim_vs_hw.png');
fprintf('\n  Saved: overlay_sim_vs_hw.png\n');

%% ========================================================================
%  COMPUTE FIT PERCENTAGE
%  ========================================================================
if has_hw
    fprintf('\n--- Fit Percentage ---\n');
    
    % Interpolate simulation to hardware time grid
    T_sim_interp = interp1(t_sim, T_sim, t_hw, 'linear', 'extrap');
    
    % Normalized Root Mean Square Error (NRMSE) fit %
    % fit = 100 * (1 - ||y - y_hat|| / ||y - mean(y)||)
    residual = T_hw - T_sim_interp;
    nrmse = norm(residual) / norm(T_hw - mean(T_hw));
    fit_pct = max(0, 100 * (1 - nrmse));
    
    fprintf('  NRMSE Fit: %.1f%%\n', fit_pct);
    fprintf('  Mean Residual: %.2f °C\n', mean(residual));
    fprintf('  Max  Residual: %.2f °C\n', max(abs(residual)));
    fprintf('  Std  Residual: %.2f °C\n', std(residual));
    
    % Residual plot
    figure('Name', 'Residual', 'Position', [50 50 1000 350]);
    plot(t_hw, residual, 'g-', 'LineWidth', 1);
    xlabel('Time [s]');
    ylabel('Residual [°C]');
    title(sprintf('Temperature Residual (Hardware − Simulation) — Fit: %.1f%%', fit_pct));
    grid on;
    yline(0, 'k-');
    yline(mean(residual) + 3*std(residual), 'r--', '+3σ');
    yline(mean(residual) - 3*std(residual), 'r--', '-3σ');
    
    saveas(gcf, 'overlay_residual.png');
    fprintf('  Saved: overlay_residual.png\n');
end

fprintf('\n=== Overlay plots complete ===\n');

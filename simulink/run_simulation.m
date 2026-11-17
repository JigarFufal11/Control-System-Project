%% run_simulation.m — Run all simulation scenarios and produce plots
%  Scenarios:
%    1. ON-OFF control:  30 → 40 → 35 °C
%    2. PI control:      30 → 40 → 35 °C  (no disturbance)
%    3. PI + Disturbance: R_th steps +30% at t = 900 s during 40°C hold
%
%  Prerequisites: Run init_params.m and build_model.m first.
%
%  Author : Control Systems Project Team
%  -----------------------------------------------------------------------

clear; clc; close all;

%% ---- Initialize --------------------------------------------------------
fprintf('=== Running Thermal Control Simulations ===\n\n');
run('init_params.m');

modelName = 'thermal_control';

% Build model if it doesn't exist
if ~exist([modelName '.slx'], 'file')
    fprintf('Model not found. Building...\n');
    run('build_model.m');
end

load_system(modelName);

% Ensure reference timeseries exists
ref_ts = timeseries([30; 30; 40; 40; 35; 35], [0; 599; 600; 1199; 1200; 1800]);

%% ========================================================================
%  SCENARIO 1: ON-OFF Control
%  ========================================================================
fprintf('\n--- Scenario 1: ON-OFF Control ---\n');

% Set controller switch to ON-OFF (relay) — position 0 = top input
set_param([modelName '/Controller_Switch'], 'sw', '0');
% Set disturbance switch to nominal R_th
set_param([modelName '/Disturbance_Switch'], 'sw', '0');

simOut_onoff = sim(modelName, 'StopTime', num2str(t_final));

% Extract data
t_onoff = simOut_onoff.T_sim.Time;
T_onoff = simOut_onoff.T_sim.Data;
u_onoff = simOut_onoff.u_sim.Data;
r_onoff = simOut_onoff.ref_sim.Data;

fprintf('  Simulation complete. %d data points.\n', length(t_onoff));

% Save
save('results_onoff.mat', 't_onoff', 'T_onoff', 'u_onoff', 'r_onoff');

%% ========================================================================
%  SCENARIO 2: PI Control (no disturbance)
%  ========================================================================
fprintf('\n--- Scenario 2: PI Control ---\n');

% Set controller switch to PI — position 1 = bottom input
set_param([modelName '/Controller_Switch'], 'sw', '1');
% Keep disturbance off
set_param([modelName '/Disturbance_Switch'], 'sw', '0');

simOut_pi = sim(modelName, 'StopTime', num2str(t_final));

t_pi = simOut_pi.T_sim.Time;
T_pi = simOut_pi.T_sim.Data;
u_pi = simOut_pi.u_sim.Data;
r_pi = simOut_pi.ref_sim.Data;

fprintf('  Simulation complete. %d data points.\n', length(t_pi));

save('results_pi.mat', 't_pi', 'T_pi', 'u_pi', 'r_pi');

%% ========================================================================
%  SCENARIO 3: PI + Disturbance
%  ========================================================================
fprintf('\n--- Scenario 3: PI + Disturbance (R_th +30%% at t=%ds) ---\n', disturbance_time);

% PI controller, disturbance enabled
set_param([modelName '/Controller_Switch'], 'sw', '1');
set_param([modelName '/Disturbance_Switch'], 'sw', '1');

simOut_dist = sim(modelName, 'StopTime', num2str(t_final));

t_dist = simOut_dist.T_sim.Time;
T_dist = simOut_dist.T_sim.Data;
u_dist = simOut_dist.u_sim.Data;
r_dist = simOut_dist.ref_sim.Data;

fprintf('  Simulation complete. %d data points.\n', length(t_dist));

save('results_disturbance.mat', 't_dist', 'T_dist', 'u_dist', 'r_dist');

%% ========================================================================
%  PLOTS
%  ========================================================================
fprintf('\n--- Generating Plots ---\n');

% Add script directory to path
addpath(fileparts(mfilename('fullpath')));

% ---- Plot 1: ON-OFF vs PI Temperature Comparison ----
fig1 = figure('Name', 'ON-OFF vs PI: Temperature', 'Position', [100 100 900 600], 'Color', 'w');
if exist('theme', 'file')
    theme(fig1, 'light');
end
set(fig1, 'Color', 'w');

ax1 = subplot(2,1,1);
set(ax1, 'Color', 'w');
plot(ax1, t_onoff, r_onoff, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
hold(ax1, 'on');
plot(ax1, t_onoff, T_onoff, 'r-', 'LineWidth', 1.2, 'DisplayName', 'ON-OFF');
plot(ax1, t_pi, T_pi, 'b-', 'LineWidth', 1.2, 'DisplayName', 'PI');
xlabel(ax1, 'Time [s]');
ylabel(ax1, 'Temperature [°C]');
title(ax1, 'Temperature Response: ON-OFF vs PI');
legend(ax1, 'Location', 'best');
grid(ax1, 'on');

ax2 = subplot(2,1,2);
set(ax2, 'Color', 'w');
plot(ax2, t_onoff, u_onoff, 'r-', 'LineWidth', 1, 'DisplayName', 'ON-OFF');
hold(ax2, 'on');
plot(ax2, t_pi, u_pi, 'b-', 'LineWidth', 1, 'DisplayName', 'PI');
xlabel(ax2, 'Time [s]');
ylabel(ax2, 'Duty Cycle');
title(ax2, 'Control Signal: ON-OFF vs PI');
legend(ax2, 'Location', 'best');
grid(ax2, 'on');
ylim(ax2, [-0.1, 1.1]);

exportgraphics(fig1, 'plot_onoff_vs_pi.png', 'BackgroundColor', 'white', 'Resolution', 300);
fprintf('  Saved: plot_onoff_vs_pi.png\n');

% ---- Plot 2: PI with Disturbance ----
fig2 = figure('Name', 'PI with Disturbance', 'Position', [100 100 900 600], 'Color', 'w');
if exist('theme', 'file')
    theme(fig2, 'light');
end
set(fig2, 'Color', 'w');

ax3 = subplot(2,1,1);
set(ax3, 'Color', 'w');
plot(ax3, t_pi, r_pi, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
hold(ax3, 'on');
plot(ax3, t_pi, T_pi, 'b-', 'LineWidth', 1.2, 'DisplayName', 'PI (no disturbance)');
plot(ax3, t_dist, T_dist, 'm-', 'LineWidth', 1.2, 'DisplayName', 'PI (with disturbance)');
xline(ax3, disturbance_time, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Disturbance onset');
xlabel(ax3, 'Time [s]');
ylabel(ax3, 'Temperature [°C]');
title(ax3, 'PI Response with R_{th} Disturbance (+30%)');
legend(ax3, 'Location', 'best');
grid(ax3, 'on');

ax4 = subplot(2,1,2);
set(ax4, 'Color', 'w');
plot(ax4, t_pi, u_pi, 'b-', 'LineWidth', 1, 'DisplayName', 'PI (no disturbance)');
hold(ax4, 'on');
plot(ax4, t_dist, u_dist, 'm-', 'LineWidth', 1, 'DisplayName', 'PI (with disturbance)');
xline(ax4, disturbance_time, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Disturbance onset');
xlabel(ax4, 'Time [s]');
ylabel(ax4, 'Duty Cycle');
title(ax4, 'Control Effort with Disturbance');
legend(ax4, 'Location', 'best');
grid(ax4, 'on');
ylim(ax4, [-0.1, 1.1]);

exportgraphics(fig2, 'plot_pi_disturbance.png', 'BackgroundColor', 'white', 'Resolution', 300);
fprintf('  Saved: plot_pi_disturbance.png\n');

%% ---- Compute Metrics (if compute_metrics exists) ----------------------
if exist('compute_metrics', 'file')
    fprintf('\n--- Performance Metrics ---\n');
    
    fprintf('\n  ON-OFF Controller:\n');
    metrics_onoff = compute_metrics(t_onoff, r_onoff, T_onoff, u_onoff, P, 1);
    
    fprintf('\n  PI Controller:\n');
    metrics_pi = compute_metrics(t_pi, r_pi, T_pi, u_pi, P, 1);
    
    fprintf('\n  PI + Disturbance:\n');
    metrics_dist = compute_metrics(t_dist, r_dist, T_dist, u_dist, P, 1);
    
    save('metrics_all.mat', 'metrics_onoff', 'metrics_pi', 'metrics_dist');
else
    fprintf('\n  compute_metrics.m not found. Skipping metrics.\n');
    fprintf('  Ensure compute_metrics.m is in the simulink directory.\n');
end

fprintf('\n=== All simulations complete ===\n');

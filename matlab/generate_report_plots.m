%% generate_report_plots.m — Master script to generate ALL report figures
%
%  Generates publication-quality plots for:
%    1. Step response model fit
%    2. ON-OFF vs PI comparison
%    3. PI disturbance response (sim vs hardware)
%    4. Fault detection results
%    5. Performance metrics bar charts
%    6. Energy comparison
%
%  Prerequisites: Simulation results (results_*.mat) and hardware CSVs
%
%  Author : Control Systems Project Team
%  -----------------------------------------------------------------------

clear; clc; close all;
fprintf('=== Generating All Report Plots ===\n\n');

% Set default plot style
set(0, 'DefaultAxesFontSize', 11);
set(0, 'DefaultLineLineWidth', 1.5);
set(0, 'DefaultFigureColor', 'w');

% Output directory
outdir = '../plots';
if ~exist(outdir, 'dir'), mkdir(outdir); end

%% ---- Load Simulation Data -----------------------------------------------
fprintf('Loading simulation data...\n');
sim_dir = '../simulink/';

has_onoff = exist([sim_dir 'results_onoff.mat'], 'file');
has_pi    = exist([sim_dir 'results_pi.mat'], 'file');
has_dist  = exist([sim_dir 'results_disturbance.mat'], 'file');

if has_onoff, load([sim_dir 'results_onoff.mat']); end
if has_pi,    load([sim_dir 'results_pi.mat']); end
if has_dist,  load([sim_dir 'results_disturbance.mat']); end

%% ---- Load Hardware Data (if available) ----------------------------------
data_dir = '../data/';
hw_files = struct('onoff', 'onoff_run.csv', 'pi', 'pi_run.csv', ...
                  'dist', 'disturbance_run.csv', 'fault', 'fault_run.csv');

has_hw = false;
if exist([data_dir hw_files.pi], 'file')
    has_hw = true;
    fprintf('Hardware data found.\n');
end

P  = 50;  % Heater power [W]
Ts = 1;   % Sample time [s]

%% ========================================================================
%  FIGURE 1: ON-OFF vs PI Temperature Comparison
%  ========================================================================
if has_onoff && has_pi
    fprintf('  Fig 1: ON-OFF vs PI comparison...\n');
    fig1 = figure('Name', 'Fig1_ONOFF_vs_PI', 'Position', [50 50 1000 750]);
    
    % Temperature subplot
    subplot(3,1,1);
    plot(t_onoff, r_onoff, 'k--', 'DisplayName', 'Reference');
    hold on;
    plot(t_onoff, T_onoff, 'r-', 'DisplayName', 'ON-OFF');
    plot(t_pi, T_pi, 'b-', 'DisplayName', 'PI');
    ylabel('Temperature [°C]');
    title('Controller Comparison: ON-OFF vs PI');
    legend('Location', 'best');
    grid on;
    
    % Duty cycle subplot
    subplot(3,1,2);
    stairs(t_onoff, u_onoff, 'r-', 'LineWidth', 0.8, 'DisplayName', 'ON-OFF');
    hold on;
    plot(t_pi, u_pi, 'b-', 'DisplayName', 'PI');
    ylabel('Duty Cycle');
    legend('Location', 'best');
    grid on;
    ylim([-0.05, 1.05]);
    
    % Error subplot
    subplot(3,1,3);
    plot(t_onoff, r_onoff - T_onoff, 'r-', 'DisplayName', 'ON-OFF Error');
    hold on;
    plot(t_pi, r_pi - T_pi, 'b-', 'DisplayName', 'PI Error');
    yline(0, 'k-');
    xlabel('Time [s]');
    ylabel('Error [°C]');
    legend('Location', 'best');
    grid on;
    
    saveas(fig1, fullfile(outdir, 'fig1_onoff_vs_pi.png'));
    saveas(fig1, fullfile(outdir, 'fig1_onoff_vs_pi.fig'));
    fprintf('    Saved.\n');
end

%% ========================================================================
%  FIGURE 2: PI Disturbance Response
%  ========================================================================
if has_pi && has_dist
    fprintf('  Fig 2: PI disturbance response...\n');
    fig2 = figure('Name', 'Fig2_Disturbance', 'Position', [50 50 1000 600]);
    
    subplot(2,1,1);
    plot(t_pi, r_pi, 'k--', 'DisplayName', 'Reference');
    hold on;
    plot(t_pi, T_pi, 'b-', 'DisplayName', 'PI (nominal)');
    plot(t_dist, T_dist, 'm-', 'DisplayName', 'PI (disturbed)');
    xline(900, 'g--', 'LineWidth', 2, 'DisplayName', 'Disturbance');
    ylabel('Temperature [°C]');
    title('PI Response to R_{th} Disturbance (+30%)');
    legend('Location', 'best');
    grid on;
    
    subplot(2,1,2);
    plot(t_pi, u_pi, 'b-', 'DisplayName', 'PI (nominal)');
    hold on;
    plot(t_dist, u_dist, 'm-', 'DisplayName', 'PI (disturbed)');
    xline(900, 'g--', 'LineWidth', 2, 'DisplayName', 'Disturbance');
    xlabel('Time [s]');
    ylabel('Duty Cycle');
    legend('Location', 'best');
    grid on;
    
    saveas(fig2, fullfile(outdir, 'fig2_disturbance.png'));
    saveas(fig2, fullfile(outdir, 'fig2_disturbance.fig'));
    fprintf('    Saved.\n');
end

%% ========================================================================
%  FIGURE 3: Performance Metrics Bar Charts
%  ========================================================================
if has_onoff && has_pi
    fprintf('  Fig 3: Performance metrics...\n');
    
    % Compute metrics
    addpath('.');  % ensure compute_metrics is accessible
    
    if exist('compute_metrics', 'file')
        m_onoff = compute_metrics(t_onoff, r_onoff, T_onoff, u_onoff, P, Ts);
        m_pi    = compute_metrics(t_pi, r_pi, T_pi, u_pi, P, Ts);
        
        % Find the step to 40°C for comparison (usually segment 2)
        idx_40_on = find([m_onoff.setpoint] == 40, 1);
        idx_40_pi = find([m_pi.setpoint] == 40, 1);
        
        if ~isempty(idx_40_on) && ~isempty(idx_40_pi)
            fig3 = figure('Name', 'Fig3_Metrics', 'Position', [50 50 1000 500]);
            
            metrics_names = {'Rise Time [s]', 'Settling Time [s]', ...
                             'Overshoot [%]', '|SSE| [°C]', 'Energy [kJ]'};
            
            vals_onoff = [m_onoff(idx_40_on).rise_time, ...
                          m_onoff(idx_40_on).settling_time, ...
                          m_onoff(idx_40_on).overshoot, ...
                          abs(m_onoff(idx_40_on).ss_error), ...
                          m_onoff(idx_40_on).energy/1000];
            
            vals_pi = [m_pi(idx_40_pi).rise_time, ...
                       m_pi(idx_40_pi).settling_time, ...
                       m_pi(idx_40_pi).overshoot, ...
                       abs(m_pi(idx_40_pi).ss_error), ...
                       m_pi(idx_40_pi).energy/1000];
            
            x = 1:5;
            bar(x, [vals_onoff; vals_pi]');
            set(gca, 'XTickLabel', metrics_names);
            xtickangle(30);
            ylabel('Value');
            title('Performance Metrics: ON-OFF vs PI (Setpoint 40°C)');
            legend('ON-OFF', 'PI', 'Location', 'best');
            grid on;
            
            saveas(fig3, fullfile(outdir, 'fig3_metrics_bar.png'));
            fprintf('    Saved.\n');
        end
    else
        fprintf('    compute_metrics.m not found, skipping.\n');
    end
end

%% ========================================================================
%  FIGURE 4: Energy Comparison
%  ========================================================================
if has_onoff && has_pi
    fprintf('  Fig 4: Energy comparison...\n');
    fig4 = figure('Name', 'Fig4_Energy', 'Position', [50 50 800 400]);
    
    % Cumulative energy over time
    E_onoff = cumsum(u_onoff) * P * Ts / 1000;   % kJ
    E_pi    = cumsum(u_pi)    * P * Ts / 1000;
    
    plot(t_onoff, E_onoff, 'r-', 'DisplayName', ...
        sprintf('ON-OFF (%.1f kJ)', E_onoff(end)));
    hold on;
    plot(t_pi, E_pi, 'b-', 'DisplayName', ...
        sprintf('PI (%.1f kJ)', E_pi(end)));
    xlabel('Time [s]');
    ylabel('Cumulative Energy [kJ]');
    title('Energy Consumption: ON-OFF vs PI');
    legend('Location', 'best');
    grid on;
    
    saveas(fig4, fullfile(outdir, 'fig4_energy.png'));
    fprintf('    Saved.\n');
end

%% ========================================================================
%  FIGURE 5: Fault Detection (placeholder for hardware data)
%  ========================================================================
fprintf('  Fig 5: Fault detection (template)...\n');
fig5 = figure('Name', 'Fig5_Fault_Detection', 'Position', [50 50 1000 700]);

% Create placeholder with example data
t_ex = 0:1:600;
T_meas = 40 + 0.3*randn(size(t_ex));
T_pred = 40 * ones(size(t_ex));

% Simulate a fault at t=300 (sensor offset)
T_meas(300:end) = T_meas(300:end) + 3;
residual = T_meas - T_pred;
threshold = 1.5;

subplot(3,1,1);
plot(t_ex, T_meas, 'b-', 'DisplayName', 'T_{measured}');
hold on;
plot(t_ex, T_pred, 'r--', 'DisplayName', 'T_{predicted}');
xline(300, 'g--', 'Fault injected', 'LineWidth', 1.5);
ylabel('Temperature [°C]');
title('Fault Detection Example: Sensor Offset');
legend('Location', 'best');
grid on;

subplot(3,1,2);
plot(t_ex, residual, 'g-');
hold on;
yline(threshold, 'r--', 'Threshold');
yline(-threshold, 'r--');
xlabel('Time [s]');
ylabel('Residual [°C]');
title('Model Residual');
grid on;

subplot(3,1,3);
fault_flag = abs(residual) > threshold;
stairs(t_ex, fault_flag, 'r-', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Fault Flag');
title('Fault Detection Flag');
ylim([-0.1, 1.1]);
grid on;

saveas(fig5, fullfile(outdir, 'fig5_fault_detection.png'));
fprintf('    Saved.\n');

%% ---- Done ---------------------------------------------------------------
fprintf('\n=== All report plots generated in: %s ===\n', fullfile(pwd, outdir));
fprintf('Files:\n');
listing = dir(fullfile(outdir, '*.png'));
for i = 1:length(listing)
    fprintf('  %s\n', listing(i).name);
end

function summary = main_run_all(hw_mode)
% MAIN_RUN_ALL - Master Execution Pipeline for Quanser QNET HVAC Trainer 1.0
% on National Instruments NI ELVIS II+ (EE302 Problem Statement 4).
%
% Syntax:
%   summary = main_run_all()         % Auto-detect live hardware or simulation
%   summary = main_run_all('live')   % Run with live NI ELVIS II+ hardware
%   summary = main_run_all('sim')    % Run with digital twin simulation

    close all; clc;

    % Auto-detect hardware mode if not specified
    if nargin < 1 || isempty(hw_mode)
        hw_mode = 'auto';
    end

    fprintf('========================================================================\n');
    fprintf('       EE302 CONTROL SYSTEMS LABORATORY: PROBLEM STATEMENT 4            \n');
    fprintf('    SMART HVAC TEMPERATURE CONTROL AND FAULT DETECTION SYSTEM          \n');
    fprintf('  Platform: NI ELVIS II+ | Trainer: Quanser QNET HVAC Trainer 1.0       \n');
    fprintf('  Hardware Execution Mode: %s\n', upper(hw_mode));
    fprintf('========================================================================\n\n');

    % Set up MATLAB search paths
    base_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(base_dir, 'config'));
    addpath(fullfile(base_dir, 'models'));
    addpath(fullfile(base_dir, 'controllers'));
    addpath(fullfile(base_dir, 'simulation'));
    addpath(fullfile(base_dir, 'hardware'));
    addpath(fullfile(base_dir, 'analysis'));

    %% Step 0: Sensor Calibration Check
    fprintf('>>> STEP 0: Checking Sensor Calibration on NI ELVIS II+...\n');
    cal = calibrate_sensors(10, hw_mode);

    %% Step 1: System Identification (Open-Loop Bump Test)
    fprintf('\n>>> STEP 1: Running System Identification (Open-Loop Bump Test)...\n');
    bump_dur = 45.0; % [s]
    fopdt = run_hardware_bump_test(3.0, bump_dur, hw_mode);
    res_file = fullfile(base_dir, 'results', 'identified_fopdt_params.mat');
    fopdt_data = load(res_file);

    %% Step 2: Head-to-Head Controller Comparison (ON-OFF vs PI)
    fprintf('\n>>> STEP 2: Running Closed-Loop Controller Comparison...\n');
    [metrics_onoff, metrics_pi, sim_onoff, sim_pi] = run_comparison();

    %% Step 3: Disturbance Rejection Test
    fprintf('\n>>> STEP 3: Running Disturbance Rejection Test...\n');
    [sim_dist, dist_metrics] = run_disturbance_test();

    %% Step 4: Fault Detection & Energy Assessment Test
    fprintf('\n>>> STEP 4: Running Fault Detection & Energy Assessment...\n');
    [sim_fault, fault_summary] = run_fault_detection_test();

    %% Step 5: Programmatic Simulink Model Verification
    fprintf('\n>>> STEP 5: Verifying Programmatic Simulink Model Generation...\n');
    build_simulink_model(fullfile(base_dir, 'models'));

    %% Step 6: Generate Publication-Grade Figures
    fprintf('\n>>> STEP 6: Generating Publication-Grade Figures...\n');
    fig_dir = fullfile(base_dir, 'figures');
    generate_plots(sim_onoff, sim_pi, sim_dist, sim_fault, fopdt_data, fig_dir);

    %% Step 7: Assemble Master Summary
    summary = struct();
    summary.calibration    = cal;
    summary.fopdt          = fopdt;
    summary.metrics_onoff  = metrics_onoff;
    summary.metrics_pi     = metrics_pi;
    summary.dist_metrics   = dist_metrics;
    summary.fault_summary  = fault_summary;

    fprintf('\n========================================================================\n');
    fprintf('       MASTER PIPELINE COMPLETE: ALL PS4 DELIVERABLES SATISFIED!        \n');
    fprintf('  Figures saved in : %s\n', fig_dir);
    fprintf('  Metrics saved in : %s\n', fullfile(base_dir, 'results'));
    fprintf('========================================================================\n\n');

end

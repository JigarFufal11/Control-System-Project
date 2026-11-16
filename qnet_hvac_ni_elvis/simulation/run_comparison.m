function [metrics_onoff, metrics_pi, sim_onoff, sim_pi] = run_comparison()
% RUN_COMPARISON - Head-to-Head Comparison: ON-OFF Relay vs Anti-Windup PI
% for Quanser QNET HVAC Trainer 1.0 on NI ELVIS II+.
%
% Fulfills EE302 PS4 Deliverable:
%   "Controller comparison supported by temperature-response plots and a performance table."

    fprintf('\n=======================================================\n');
    fprintf('  EE302 PS4: Running Head-to-Head Controller Comparison\n');
    fprintf('=======================================================\n');

    % 1. Simulate ON-OFF (Hysteresis Bang-Bang)
    fprintf('Simulating ON-OFF Hysteresis Controller...\n');
    sim_onoff = run_simulation('onoff');
    metrics_onoff = calculate_metrics(sim_onoff);

    % 2. Simulate Anti-Windup PI with Feedforward
    fprintf('Simulating Anti-Windup PI Controller with Feedforward...\n');
    sim_pi = run_simulation('pi');
    metrics_pi = calculate_metrics(sim_pi);

    % 3. Print and Export Summary Table
    csv_file = fullfile(fileparts(mfilename('fullpath')), '..', 'results', 'controller_comparison_table.csv');
    export_report_table(metrics_onoff, metrics_pi, csv_file);

end

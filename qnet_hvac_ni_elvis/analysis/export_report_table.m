function tbl = export_report_table(metrics_onoff, metrics_pi, output_csv_path)
% EXPORT_REPORT_TABLE - Formats controller evaluation metrics into a comparative
% summary table and exports to CSV according to EE302 PS4 requirements.
%
% Inputs:
%   metrics_onoff   - Metrics struct for ON-OFF controller
%   metrics_pi      - Metrics struct for Anti-Windup PI controller
%   output_csv_path - (Optional) Path to export CSV

    if nargin < 3 || isempty(output_csv_path)
        output_csv_path = fullfile(fileparts(mfilename('fullpath')), '..', 'results', 'metrics_summary.csv');
    end

    Metric = {
        'Rise Time (t_r, 10-90%) [s]';
        'Settling Time (t_s, 2%) [s]';
        'Peak Overshoot (M_p) [%]';
        'Steady-State Error (e_ss) [°C]';
        'Integral Absolute Error (IAE)';
        'Integral Time Abs Error (ITAE)';
        'Total Energy Consumed [Joules]';
        'Total Energy Consumed [Watt-hours]';
        'Average Duty Cycle [%]';
        'Actuator Switching Count'
    };

    ON_OFF_Control = [
        metrics_onoff.t_rise_sec;
        metrics_onoff.t_settle_sec;
        metrics_onoff.overshoot_pct;
        metrics_onoff.ss_error_degC;
        metrics_onoff.IAE;
        metrics_onoff.ITAE;
        metrics_onoff.energy_Joules;
        metrics_onoff.energy_WattHours;
        metrics_onoff.duty_cycle_pct;
        metrics_onoff.switch_count
    ];

    PI_AntiWindup = [
        metrics_pi.t_rise_sec;
        metrics_pi.t_settle_sec;
        metrics_pi.overshoot_pct;
        metrics_pi.ss_error_degC;
        metrics_pi.IAE;
        metrics_pi.ITAE;
        metrics_pi.energy_Joules;
        metrics_pi.energy_WattHours;
        metrics_pi.duty_cycle_pct;
        metrics_pi.switch_count
    ];

    tbl = table(Metric, ON_OFF_Control, PI_AntiWindup);

    % Print formatted table to Command Window
    fprintf('\n========================================================================\n');
    fprintf('  EE302 PS4: CONTROLLER PERFORMANCE AND ENERGY TRADE-OFF COMPARISON      \n');
    fprintf('  Platform: NI ELVIS II+ | Plant: Quanser QNET HVAC Trainer 1.0         \n');
    fprintf('========================================================================\n');
    disp(tbl);
    fprintf('========================================================================\n\n');

    % Ensure output directory exists and write CSV
    [csv_dir, ~, ~] = fileparts(output_csv_path);
    if ~exist(csv_dir, 'dir'), mkdir(csv_dir); end
    writetable(tbl, output_csv_path);
    fprintf('Performance summary table exported to: %s\n', output_csv_path);

end

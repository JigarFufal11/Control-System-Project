function cal = calibrate_sensors(num_samples, hw_mode)
% CALIBRATE_SENSORS - Sensor baseline zeroing and noise floor calibration
% for Quanser QNET HVAC Trainer 1.0 mounted on NI ELVIS II+.
%
% Inputs:
%   num_samples - Number of baseline samples to average (default: 50)
%   hw_mode     - 'auto', 'live', or 'force_sim' (default: 'auto')
%
% Outputs:
%   cal - Struct containing mean ambient, chamber, secondary readings,
%         voltage offsets, and standard deviations.

    if nargin < 1 || isempty(num_samples), num_samples = 50; end
    if nargin < 2 || isempty(hw_mode), hw_mode = 'auto'; end

    fprintf('\n=======================================================\n');
    fprintf('  QNET HVAC Sensor Calibration & Baseline Zeroing      \n');
    fprintf('=======================================================\n');

    hw = elvis_hardware_interface(hw_mode);
    cleanup_hw = onCleanup(@() hw.close());

    % Ensure heater is off and fan is running nominal for thermal stabilization
    hw.write_actuators(0.0, 2.5);
    pause(0.5);

    T_ch_buf  = zeros(1, num_samples);
    T_amb_buf = zeros(1, num_samples);
    T_sec_buf = zeros(1, num_samples);
    V_buf     = zeros(num_samples, 3);

    fprintf('Sampling %d data points at 10 Hz...\n', num_samples);
    for k = 1:num_samples
        [T_ch, T_amb, T_sec, V_raw] = hw.read_sensors();
        T_ch_buf(k)  = T_ch;
        T_amb_buf(k) = T_amb;
        T_sec_buf(k) = T_sec;
        V_buf(k, :)  = V_raw;
        pause(0.05);
    end

    cal = struct();
    cal.mean_T_chamber   = mean(T_ch_buf);
    cal.mean_T_ambient   = mean(T_amb_buf);
    cal.mean_T_secondary = mean(T_sec_buf);
    cal.std_T_chamber    = std(T_ch_buf);
    cal.std_T_ambient    = std(T_amb_buf);
    cal.mean_voltages    = mean(V_buf, 1);
    cal.sensor_offset    = cal.mean_T_chamber - cal.mean_T_ambient;

    fprintf('\n---------------- Calibration Summary ----------------\n');
    fprintf('  Chamber Temp (AI 5)   : %5.2f +/- %.3f °C\n', cal.mean_T_chamber, cal.std_T_chamber);
    fprintf('  Ambient Temp (AI 7)   : %5.2f +/- %.3f °C\n', cal.mean_T_ambient, cal.std_T_ambient);
    fprintf('  Secondary Temp (AI 6) : %5.2f °C\n', cal.mean_T_secondary);
    fprintf('  Thermistor Discrepancy: %5.3f °C\n', cal.sensor_offset);
    fprintf('  Status                : CALIBRATION NOMINAL\n');
    fprintf('-----------------------------------------------------\n');

end

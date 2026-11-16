function hw = elvis_channel_config(deviceId)
% ELVIS_CHANNEL_CONFIG - Hardware channel mapping and DAQ configuration
% for Quanser QNET HVAC Trainer 1.0 mounted on NI ELVIS II / II+.
%
% Syntax:
%   hw = elvis_channel_config()
%   hw = elvis_channel_config('Dev1')
%
% Inputs:
%   deviceId - String identifier of the NI-DAQmx device (default: 'Dev1')
%
% Outputs:
%   hw - Struct containing channel names, pin references, voltage limits,
%        sampling rates, and calibration scale factors.

    if nargin < 1 || isempty(deviceId)
        deviceId = 'Dev1';
    end

    hw = struct();
    hw.vendor           = 'ni';
    hw.device_id        = deviceId;
    hw.platform_name    = 'National Instruments NI ELVIS II+';
    hw.trainer_board    = 'Quanser QNET HVAC Trainer 1.0';

    %% Analog Input (AI) - Sensors
    % Quanser QNET HVAC Trainer 1.0 routing on NI ELVIS II+:
    %   - AI 5: Chamber Temperature Sensor (20.0 °C / Volt)
    %   - AI 7: Ambient Room Temperature Sensor (20.0 °C / Volt)
    %   - AI 6: Secondary / Downstream Duct Sensor (20.0 °C / Volt)
    hw.ai_channels      = {sprintf('%s/ai5', deviceId), ...
                           sprintf('%s/ai7', deviceId), ...
                           sprintf('%s/ai6', deviceId)};
    hw.ai_names         = {'Chamber_Temp (AI 5)', 'Ambient_Temp (AI 7)', 'Secondary_Duct_Temp (AI 6)'};
    hw.ai_pin_refs      = {'AI 5 (Chamber Thermistor)', ...
                           'AI 7 (Ambient Thermistor)', ...
                           'AI 6 (Duct Thermistor)'};
    hw.ai_voltage_range = [-10.0, 10.0];  % Volts
    hw.ai_terminal_cfg  = 'SingleEnded';  % SingleEnded for QNET standard routing
    hw.temp_sensitivity = 20.0;           % 20.0 °C / Volt
    hw.temp_offset      = 0.0;            % °C

    %% Analog Output (AO) - Actuators
    hw.ao_channels      = {sprintf('%s/ao0', deviceId), ...
                           sprintf('%s/ao1', deviceId)};
    hw.ao_names         = {'Heater_Command', 'Fan_Command'};
    hw.ao_pin_refs      = {'Pin 60 Side A (AO 0)', ...
                           'Pin 60 Side B (AO 1)'};
    hw.ao_voltage_range = [0.0, 5.0];     % Volts (Unipolar safe range for QNET amplifiers)
    hw.ao_safe_defaults = [0.0, 2.5];     % Heater = 0 V (OFF), Fan = 2.5 V (Nominal airflow)

    %% DAQ Timing Parameters
    hw.sample_rate      = 10.0;           % Samples per second [Hz]
    hw.clock_type       = 'SoftwareClock';% Background acquisition or on-demand software clock

    %% Helper Calibration Functions
    % Converts ADC voltage readings to Degrees Celsius
    hw.volts2degC = @(v) v .* hw.temp_sensitivity + hw.temp_offset;
    
    % Converts Degrees Celsius back to expected sensor output voltage
    hw.degC2volts = @(T) (T - hw.temp_offset) ./ hw.temp_sensitivity;

    % Clamps heater actuator voltage to strictly safe operating limits [0, 5] V
    hw.clamp_heater = @(u) min(max(u, hw.ao_voltage_range(1)), hw.ao_voltage_range(2));

    % Clamps fan actuator voltage to [0.5, 5.0] V
    hw.clamp_fan    = @(v) min(max(v, 0.5), 5.0);

end

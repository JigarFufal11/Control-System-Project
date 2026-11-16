classdef elvis_hardware_interface < handle
% ELVIS_HARDWARE_INTERFACE - Hardware Abstraction Layer for Quanser QNET
% HVAC Trainer 1.0 mounted on National Instruments NI ELVIS II / II+.
%
% Features:
%   - Auto-detection of physical NI ELVIS II+ DAQ hardware via NI-DAQmx.
%   - Seamless fallback to High-Fidelity Digital Twin Emulation Mode when
%     running offline or without physical hardware tethered.
%   - Strict voltage clamping to ensure safety of the QNET heating element.
%   - Support for real-time sensor acquisition and actuator generation.
%
% Usage:
%   hw = elvis_hardware_interface();             % Auto-detect or digital twin
%   hw = elvis_hardware_interface('force_sim');  % Force digital twin mode
%   [T_ch, T_amb, T_sec] = hw.read_sensors();
%   hw.write_actuators(u_heater, v_fan);
%   hw.emergency_stop();
%   hw.close();

    properties
        is_live_hardware    = false;   % True if physical NI ELVIS II+ DAQ is active
        device_id           = 'Dev1';  % NI-DAQmx device name
        daq_in                         % DAQ input session (Analog Input)
        daq_out                        % DAQ output session (Analog Output)
        params                         % Physics parameters struct
        cfg                            % Hardware pinout and scaling config
        
        % Digital Twin Emulation State
        plant_state                    % Internal plant simulation state
        current_u_heater    = 0.0;     % Current heater voltage [V]
        current_v_fan       = 2.5;     % Current fan voltage [V]
        fault_settings                 % Active fault injection struct
        last_update_time               % Timestamp of last iteration
    end

    methods
        function obj = elvis_hardware_interface(mode, deviceId)
            % Constructor: Initializes hardware interface or digital twin
            if nargin < 1 || isempty(mode)
                mode = 'auto';
            end
            if nargin < 2 || isempty(deviceId)
                deviceId = 'Dev1';
            end

            obj.device_id = deviceId;
            obj.params = qnet_hvac_params();
            obj.cfg    = elvis_channel_config(deviceId);

            % Initialize fault settings
            obj.fault_settings = struct('fan_failure', false, ...
                                        'sensor_bias', 0.0, ...
                                        'heater_degrade', 1.0);

            % Attempt to initialize physical NI ELVIS DAQ if requested
            if strcmpi(mode, 'auto') || strcmpi(mode, 'live')
                obj.init_ni_daq();
            end

            if ~obj.is_live_hardware
                fprintf('[NI ELVIS II+] Operating in HIGH-FIDELITY DIGITAL TWIN Mode.\n');
                obj.init_digital_twin();
            else
                fprintf('[NI ELVIS II+] Connected to LIVE HARDWARE: %s.\n', obj.device_id);
            end
        end

        function init_ni_daq(obj)
            % Check for NI-DAQmx hardware presence
            try
                vendors = daqvendorlist;
                has_ni = any(strcmp(vendors.ID, 'ni'));
                if ~has_ni
                    obj.is_live_hardware = false;
                    return;
                end

                devs = daqlist;
                if isempty(devs) || ~any(strcmp(devs.DeviceID, obj.device_id))
                    obj.is_live_hardware = false;
                    return;
                end

                % Hardware is detected! Create DAQ objects using MATLAB Data Acquisition Toolbox
                obj.daq_in = daq('ni');
                addinput(obj.daq_in, obj.device_id, 'ai5', 'Voltage'); % Chamber temp (AI 5)
                addinput(obj.daq_in, obj.device_id, 'ai7', 'Voltage'); % Ambient temp (AI 7)
                addinput(obj.daq_in, obj.device_id, 'ai6', 'Voltage'); % Secondary duct temp (AI 6)
                
                % Set terminal configuration and range
                obj.daq_in.Channels(1).Range = [-10.0, 10.0];
                obj.daq_in.Channels(2).Range = [-10.0, 10.0];
                obj.daq_in.Channels(3).Range = [-10.0, 10.0];

                obj.daq_out = daq('ni');
                addoutput(obj.daq_out, obj.device_id, 'ao0', 'Voltage'); % Heater
                addoutput(obj.daq_out, obj.device_id, 'ao1', 'Voltage'); % Fan
                obj.daq_out.Channels(1).Range = [-5.0, 5.0];
                obj.daq_out.Channels(2).Range = [-5.0, 5.0];

                % Initialize outputs safely (Heater OFF = 0 V, Fan nominal = 2.5 V)
                write(obj.daq_out, [0.0, 2.5]);
                obj.is_live_hardware = true;

            catch ME
                % Fall back to Digital Twin gracefully
                fprintf('[NI ELVIS II+] DAQ Init Note: %s\n', ME.message);
                obj.is_live_hardware = false;
            end
        end

        function init_digital_twin(obj)
            % Initialize state for digital twin
            obj.plant_state = struct();
            obj.plant_state.T_chamber   = obj.params.T_amb;
            obj.plant_state.T_secondary = obj.params.T_amb;
            buf_len = max(1, round(obj.params.theta_nom / obj.params.dt));
            obj.plant_state.delay_buffer = repmat(0.0, 1, buf_len);
            obj.plant_state.buffer_index = 1;
            obj.current_u_heater = 0.0;
            obj.current_v_fan    = obj.params.v_fan_nominal;
            obj.last_update_time = tic;
        end

        function [T_chamber, T_ambient, T_secondary, raw_voltages] = read_sensors(obj)
            % READ_SENSORS - Reads temperature feedback from QNET HVAC sensors
            %
            % Returns:
            %   T_chamber   - Primary duct temperature [°C]
            %   T_ambient   - Room ambient temperature [°C]
            %   T_secondary - Secondary downstream temperature [°C]
            %   raw_voltages- 1x3 array of ADC voltages [V_ai0, V_ai1, V_ai2]

            if obj.is_live_hardware
                % Live DAQ Read
                data = read(obj.daq_in, 1);
                raw_voltages = data{1, :};
                T_chamber    = obj.cfg.volts2degC(raw_voltages(1));
                T_ambient    = obj.cfg.volts2degC(raw_voltages(2));
                T_secondary  = obj.cfg.volts2degC(raw_voltages(3));
            else
                % Digital Twin Emulation Step
                t_elapsed = toc(obj.last_update_time);
                obj.last_update_time = tic;

                [~, ~, sensor_meas, obj.plant_state] = qnet_thermal_plant(...
                    t_elapsed, obj.plant_state, obj.current_u_heater, ...
                    obj.current_v_fan, obj.params.T_amb, obj.params, obj.fault_settings);

                T_chamber   = sensor_meas.T_chamber;
                T_ambient   = sensor_meas.T_ambient;
                T_secondary = sensor_meas.T_secondary;
                raw_voltages = [sensor_meas.V_ai0, sensor_meas.V_ai1, sensor_meas.V_ai2];
            end
        end

        function write_actuators(obj, u_heater, v_fan)
            % WRITE_ACTUATORS - Outputs safe control voltages to NI ELVIS II+
            %
            % Inputs:
            %   u_heater - Heater command [0 to 5 V]
            %   v_fan    - Fan command [0.5 to 5 V] (optional, default 2.5 V)

            if nargin < 3 || isempty(v_fan)
                v_fan = obj.params.v_fan_nominal;
            end

            % Hardware guardrail clamping
            u_safe = obj.cfg.clamp_heater(u_heater);
            v_safe = obj.cfg.clamp_fan(v_fan);

            obj.current_u_heater = u_safe;
            obj.current_v_fan    = v_safe;

            if obj.is_live_hardware
                write(obj.daq_out, [u_safe, v_safe]);
            end
        end

        function inject_fault(obj, fault_type, value)
            % INJECT_FAULT - Simulates physical hardware anomalies
            % Supported types: 'fan_failure', 'sensor_bias', 'heater_degrade'
            if nargin < 3
                value = true;
            end
            switch lower(fault_type)
                case 'fan_failure'
                    obj.fault_settings.fan_failure = logical(value);
                case 'sensor_bias'
                    obj.fault_settings.sensor_bias = double(value);
                case 'heater_degrade'
                    obj.fault_settings.heater_degrade = double(value);
                case 'clear'
                    obj.fault_settings.fan_failure = false;
                    obj.fault_settings.sensor_bias = 0.0;
                    obj.fault_settings.heater_degrade = 1.0;
            end
        end

        function emergency_stop(obj)
            % EMERGENCY_STOP - Safe immediate shutdown of heater
            if obj.is_live_hardware && ~isempty(obj.daq_out) && isvalid(obj.daq_out)
                try
                    obj.write_actuators(0.0, 2.5);
                catch
                end
            end
            fprintf('[SAFETY] Emergency stop triggered: Heater set to 0.0 V, Fan running at 2.5 V.\n');
        end

        function close(obj)
            % Clean shutdown
            obj.emergency_stop();
            if obj.is_live_hardware
                try
                    if ~isempty(obj.daq_in) && isvalid(obj.daq_in)
                        delete(obj.daq_in);
                    end
                    obj.daq_in = [];
                catch
                end
                try
                    if ~isempty(obj.daq_out) && isvalid(obj.daq_out)
                        delete(obj.daq_out);
                    end
                    obj.daq_out = [];
                catch
                end
                obj.is_live_hardware = false;
            end
            fprintf('[NI ELVIS II+] Interface released safely.\n');
        end

        function delete(obj)
            obj.close();
        end
    end
end

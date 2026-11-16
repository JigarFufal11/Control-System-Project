function [u_cmd, ctrl_state] = onoff_controller(T_meas, T_sp, ctrl_state, params)
% ONOFF_CONTROLLER - Hysteresis Relay (Bang-Bang) Temperature Controller
% for Quanser QNET HVAC Trainer 1.0 on NI ELVIS II+.
%
% Implements symmetric deadband hysteresis to prevent rapid chatter:
%   u = u_low  (0 V) if T_meas > T_sp + Delta/2
%   u = u_high (5 V) if T_meas < T_sp - Delta/2
%   u = u_prev       if within hysteresis band [T_sp - Delta/2, T_sp + Delta/2]
%
% Inputs:
%   T_meas     - Measured chamber temperature [°C]
%   T_sp       - Current temperature setpoint [°C]
%   ctrl_state - Struct maintaining controller memory (u_prev, switch_count)
%   params     - Configuration struct from qnet_hvac_params()
%
% Outputs:
%   u_cmd      - Actuator command voltage [V] (0 V or 5 V)
%   ctrl_state - Updated controller state struct

    if nargin < 4 || isempty(params)
        params = qnet_hvac_params();
    end

    % Initialize state on first execution
    if isempty(ctrl_state) || ~isfield(ctrl_state, 'u_prev')
        ctrl_state = struct();
        ctrl_state.u_prev       = params.onoff_u_low;
        ctrl_state.switch_count = 0;
        ctrl_state.time_on      = 0.0;
        ctrl_state.total_time   = 0.0;
    end

    half_band = params.onoff_hysteresis / 2.0;
    upper_threshold = T_sp + half_band;
    lower_threshold = T_sp - half_band;

    if T_meas > upper_threshold
        u_cmd = params.onoff_u_low;
    elseif T_meas < lower_threshold
        u_cmd = params.onoff_u_high;
    else
        u_cmd = ctrl_state.u_prev; % Latch inside deadband
    end

    % Track switching events (useful for relay wear assessment)
    if u_cmd ~= ctrl_state.u_prev
        ctrl_state.switch_count = ctrl_state.switch_count + 1;
    end

    if u_cmd > 0.1
        ctrl_state.time_on = ctrl_state.time_on + params.dt;
    end
    ctrl_state.total_time = ctrl_state.total_time + params.dt;
    ctrl_state.u_prev     = u_cmd;

end

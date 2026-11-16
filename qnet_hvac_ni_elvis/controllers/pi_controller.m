function [u_cmd, ctrl_state, debug_info] = pi_controller(T_meas, T_sp, T_amb, ctrl_state, params)
% PI_CONTROLLER - Proportional-Integral (PI) Controller with Clamping Anti-Windup
% and Ambient Feedforward for Quanser QNET HVAC Trainer 1.0 on NI ELVIS II+.
%
% Control Law:
%   e(t)     = T_sp(t) - T_meas(t)
%   u_p(t)   = K_p * e(t)
%   u_ff(t)  = (T_sp(t) - T_amb(t)) / K_nom   (Feedforward equilibrium bias)
%   u_unsat  = u_p(t) + u_i(t) + u_ff(t)
%   u_cmd    = min(max(u_unsat, u_min), u_max)
%
% Anti-Windup Strategy:
%   Conditional Integration (Clamping): Integration is paused if the actuator
%   is saturated and the error sign would drive the integrator further into saturation.
%
% Inputs:
%   T_meas     - Measured chamber temperature [°C]
%   T_sp       - Current temperature setpoint [°C]
%   T_amb      - Ambient room temperature [°C]
%   ctrl_state - Struct maintaining controller memory (integrator, prev_error)
%   params     - Configuration struct from qnet_hvac_params()
%
% Outputs:
%   u_cmd      - Clamped actuator voltage command [0 to 5 V]
%   ctrl_state - Updated controller state
%   debug_info - Struct with u_p, u_i, u_ff, error, and is_saturated flag

    if nargin < 5 || isempty(params)
        params = qnet_hvac_params();
    end

    dt = params.dt;

    % Initialize state on first execution
    if isempty(ctrl_state) || ~isfield(ctrl_state, 'integrator')
        ctrl_state = struct();
        ctrl_state.integrator    = 0.0;
        ctrl_state.prev_error    = 0.0;
        ctrl_state.is_saturated  = false;
        ctrl_state.total_time    = 0.0;
    end

    % 1. Error computation
    error = T_sp - T_meas;

    % 2. Proportional Term
    u_p = params.pi_Kp * error;

    % 3. Feedforward Ambient Compensation (Equilibrium heater bias)
    if isfield(params, 'enable_feedforward') && params.enable_feedforward
        u_ff = max(0.0, (T_sp - T_amb) / params.K_nom);
    else
        u_ff = 0.0;
    end

    % 4. Tentative Integral Term
    u_i = ctrl_state.integrator;

    % 5. Unclamped Control Action
    u_unsat = u_p + u_i + u_ff;

    % 6. Actuator Clamping to NI ELVIS II+ AO bounds [0 to 5 V]
    u_cmd = min(max(u_unsat, params.u_min), params.u_max);
    is_saturated = (u_cmd ~= u_unsat);

    % 7. Clamping Anti-Windup Logic:
    % Only accumulate integral error if:
    %   a) Actuator is NOT saturated, OR
    %   b) Actuator is saturated HIGH (u_cmd == u_max) but error is NEGATIVE (pulling back), OR
    %   c) Actuator is saturated LOW (u_cmd == u_min) but error is POSITIVE (pushing forward).
    accumulate_integral = true;
    if is_saturated
        if (u_unsat >= params.u_max && error > 0) || (u_unsat <= params.u_min && error < 0)
            accumulate_integral = false;
        end
    end

    if accumulate_integral
        ctrl_state.integrator = ctrl_state.integrator + params.pi_Ki * error * dt;
        % Guardrail safety limit on integrator to prevent numerical blowup
        ctrl_state.integrator = min(max(ctrl_state.integrator, -params.u_max), params.u_max);
    end

    % Update state variables
    ctrl_state.prev_error   = error;
    ctrl_state.is_saturated = is_saturated;
    ctrl_state.total_time   = ctrl_state.total_time + dt;

    % Debug info struct
    debug_info = struct();
    debug_info.error        = error;
    debug_info.u_p          = u_p;
    debug_info.u_i          = u_i;
    debug_info.u_ff         = u_ff;
    debug_info.u_unsat      = u_unsat;
    debug_info.is_saturated = is_saturated;

end

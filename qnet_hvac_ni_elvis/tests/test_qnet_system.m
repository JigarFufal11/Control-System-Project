classdef test_qnet_system < matlab.unittest.TestCase
% TEST_QNET_SYSTEM - Formal MATLAB Unit & Integration Tests for Quanser
% QNET HVAC Trainer 1.0 on NI ELVIS II+ (EE302 Problem Statement 4).
%
% Can be executed directly using runtests('test_qnet_system') or via MATLAB MCP.

    properties
        base_dir
        params
        hw_cfg
    end

    methods (TestMethodSetup)
        function setupPaths(testCase)
            testCase.base_dir = fullfile(fileparts(mfilename('fullpath')), '..');
            addpath(fullfile(testCase.base_dir, 'config'));
            addpath(fullfile(testCase.base_dir, 'models'));
            addpath(fullfile(testCase.base_dir, 'controllers'));
            addpath(fullfile(testCase.base_dir, 'simulation'));
            addpath(fullfile(testCase.base_dir, 'hardware'));
            addpath(fullfile(testCase.base_dir, 'analysis'));

            testCase.params = qnet_hvac_params();
            testCase.hw_cfg = elvis_channel_config('Dev1');
        end
    end

    methods (Test)
        function testParameterLoading(testCase)
            % Verify critical parameters are loaded within physical ranges
            testCase.verifyNotEmpty(testCase.params);
            testCase.verifyGreaterThan(testCase.params.K_nom, 0);
            testCase.verifyGreaterThan(testCase.params.tau_nom, 5.0);
            testCase.verifyEqual(testCase.hw_cfg.temp_sensitivity, 20.0);
            testCase.verifyEqual(testCase.hw_cfg.ao_voltage_range, [0.0, 5.0]);
        end

        function testThermalPlant(testCase)
            % Verify plant derivative and state integration
            state = [];
            [dTdt, T_next, sensor_meas, ~] = qnet_thermal_plant(0, state, 5.0, 2.5, 22.0, testCase.params);
            testCase.verifyGreaterThan(dTdt, 0, 'Heating should produce positive temperature rate of change');
            testCase.verifyGreaterThan(T_next, 22.0, 'Next temperature should be above initial ambient');
            testCase.verifyEqual(sensor_meas.V_ai0, sensor_meas.T_chamber / 20.0, 'AbsTol', 0.05);
        end

        function testOnOffController(testCase)
            % Verify hysteresis relay bounds and logic
            state = [];
            T_sp = 35.0;
            % When measured temperature is well below setpoint -> Full ON (5 V)
            [u1, state] = onoff_controller(30.0, T_sp, state, testCase.params);
            testCase.verifyEqual(u1, 5.0);

            % When measured temperature is well above setpoint -> Full OFF (0 V)
            [u2, state] = onoff_controller(40.0, T_sp, state, testCase.params);
            testCase.verifyEqual(u2, 0.0);
        end

        function testPIController(testCase)
            % Verify proportional action and anti-windup clamping
            state = [];
            T_sp = 35.0;
            T_amb = 22.0;

            % Initial cold start -> should demand saturated 5 V
            [u_cmd, state, dbg] = pi_controller(22.0, T_sp, T_amb, state, testCase.params);
            testCase.verifyEqual(u_cmd, 5.0, 'Initial large error should saturate command at 5 V');
            testCase.verifyTrue(dbg.is_saturated);
            testCase.verifyLessThanOrEqual(state.integrator, 5.0, 'Integrator should be clamped against windup');
        end

        function testThermalObserverFDI(testCase)
            % Verify observer state tracking and fault isolation
            obs_state = [];
            % Run 30 steps with fan failure fault (high residual)
            for k = 1:30
                [T_hat, res, f_flags, obs_state] = thermal_observer(...
                    2.0, 38.0, 22.0, 36.0, 0.25, obs_state, testCase.params);
            end
            testCase.verifyGreaterThan(res.r_model, testCase.params.fault_thresh_fan);
            testCase.verifyTrue(f_flags.fan_failure, 'Fan failure alarm should latch after persistence window');
        end

        function testSimulinkGeneration(testCase)
            % Verify programmatic build of Simulink model
            model_dir = fullfile(testCase.base_dir, 'models');
            m_name = build_simulink_model(model_dir);
            testCase.verifyEqual(m_name, 'qnet_hvac_simulink');
            testCase.verifyTrue(isfile(fullfile(model_dir, 'qnet_hvac_simulink.slx')));
        end

        function testMetricsCalculation(testCase)
            % Verify transient & energy calculation functions
            sim_data = run_simulation('pi', struct('t_final', 60.0));
            metrics = calculate_metrics(sim_data, 20.0, 60.0);
            testCase.verifyGreaterThan(metrics.energy_Joules, 0);
            testCase.verifyGreaterThan(metrics.duty_cycle_pct, 0);
            testCase.verifyLessThanOrEqual(metrics.duty_cycle_pct, 100);
        end
    end
end

%% tune_pi.m — Compute PI controller gains using multiple methods
%  Computes PI gains using:
%    1. Internal Model Control (IMC) rules
%    2. MATLAB pidtune (Control System Toolbox)
%    3. Ziegler-Nichols (for comparison)
%
%  Uses the first-order-plus-dead-time (FOPTD) plant model:
%    G(s) = K · exp(-L·s) / (τ·s + 1)
%
%  Author : Control Systems Project Team
%  -----------------------------------------------------------------------

clear; clc;
fprintf('=== PI Controller Tuning ===\n\n');

%% ---- Plant Parameters (replace with fitted values after step test) ----
K   = 15;      % Static gain [°C per unit duty]
tau = 120;     % Time constant [s]
L   = 3;       % Dead time [s]

fprintf('Plant Model (FOPTD):\n');
fprintf('  G(s) = %.1f · exp(-%.1f·s) / (%.0f·s + 1)\n\n', K, L, tau);

%% ---- Method 1: IMC Tuning --------------------------------------------
fprintf('--- Method 1: Internal Model Control (IMC) ---\n');
tau_c_values = [max(0.1*tau, L), 2*L, 3*L, tau/4];

for i = 1:length(tau_c_values)
    tc = tau_c_values(i);
    Kp_imc = tau / (K * (L + tc));
    Ki_imc = Kp_imc / tau;
    Ti_imc = tau;  % Integral time = tau for FOPTD + IMC
    
    fprintf('  tau_c = %6.1f s:  Kp = %.4f,  Ki = %.6f,  Ti = %.0f s\n', ...
            tc, Kp_imc, Ki_imc, Ti_imc);
end

% Recommended: tau_c = max(0.1*tau, L)
tau_c = max(0.1*tau, L);
Kp_imc = tau / (K * (L + tau_c));
Ki_imc = Kp_imc / tau;
fprintf('\n  >> Recommended (tau_c = %.1f s):  Kp = %.4f,  Ki = %.6f\n\n', ...
        tau_c, Kp_imc, Ki_imc);

%% ---- Method 2: MATLAB pidtune ----------------------------------------
fprintf('--- Method 2: MATLAB pidtune ---\n');
try
    s = tf('s');
    G = K * exp(-L*s) / (tau*s + 1);
    
    % pidtune needs a rational TF — use Padé approximation for the delay
    G_pade = pade(G, 2);
    
    % Tune PI
    C_pi = pidtune(G_pade, 'PI');
    Kp_auto = C_pi.Kp;
    Ki_auto = C_pi.Ki;
    Ti_auto = Kp_auto / Ki_auto;
    
    fprintf('  Kp = %.4f\n', Kp_auto);
    fprintf('  Ki = %.6f\n', Ki_auto);
    fprintf('  Ti = %.1f s\n\n', Ti_auto);
    
    % Also try different bandwidth targets
    fprintf('  Bandwidth sweep:\n');
    for wc = [0.01, 0.02, 0.05, 0.1]
        opts = pidtuneOptions('CrossoverFrequency', wc);
        C_tmp = pidtune(G_pade, 'PI', opts);
        fprintf('    wc = %.3f rad/s:  Kp = %.4f,  Ki = %.6f\n', ...
                wc, C_tmp.Kp, C_tmp.Ki);
    end
    fprintf('\n');
catch ME
    fprintf('  Skipped (requires Control System Toolbox): %s\n\n', ME.message);
    Kp_auto = NaN; Ki_auto = NaN;
end

%% ---- Method 3: Ziegler–Nichols (for reference) -----------------------
fprintf('--- Method 3: Ziegler-Nichols (open-loop) ---\n');
% ZN formulas for PI: Kp = 0.9·τ/(K·L), Ti = 3.33·L
Kp_zn = 0.9 * tau / (K * L);
Ti_zn = 3.33 * L;
Ki_zn = Kp_zn / Ti_zn;

fprintf('  Kp = %.4f\n', Kp_zn);
fprintf('  Ki = %.6f\n', Ki_zn);
fprintf('  Ti = %.1f s\n', Ti_zn);
fprintf('  Note: ZN tends to be aggressive; may cause overshoot.\n\n');

%% ---- Summary Table ----------------------------------------------------
fprintf('=== Summary ===\n');
fprintf('  %-15s  %8s  %10s  %8s\n', 'Method', 'Kp', 'Ki', 'Ti [s]');
fprintf('  %-15s  %8.4f  %10.6f  %8.1f\n', 'IMC', Kp_imc, Ki_imc, tau);
if ~isnan(Kp_auto)
    fprintf('  %-15s  %8.4f  %10.6f  %8.1f\n', 'pidtune', Kp_auto, Ki_auto, Ti_auto);
end
fprintf('  %-15s  %8.4f  %10.6f  %8.1f\n', 'Ziegler-Nichols', Kp_zn, Ki_zn, Ti_zn);
fprintf('\n  >> Using IMC values as default.\n');

%% ---- Step Response Comparison (if toolbox available) ------------------
try
    s = tf('s');
    G = K * exp(-L*s) / (tau*s + 1);
    G_pade = pade(G, 2);
    
    % Closed-loop with each PI controller
    C_imc = pid(Kp_imc, Ki_imc);
    C_zn  = pid(Kp_zn, Ki_zn);
    
    CL_imc = feedback(C_imc * G_pade, 1);
    CL_zn  = feedback(C_zn  * G_pade, 1);
    
    figure('Name', 'PI Tuning Comparison');
    step(CL_imc, CL_zn, 600);
    legend('IMC', 'Ziegler-Nichols', 'Location', 'best');
    title('Closed-Loop Step Response — PI Tuning Comparison');
    xlabel('Time [s]');
    ylabel('Temperature [°C]');
    grid on;
    
    if ~isnan(Kp_auto)
        C_auto = pid(Kp_auto, Ki_auto);
        CL_auto = feedback(C_auto * G_pade, 1);
        hold on;
        step(CL_auto, 600);
        legend('IMC', 'Ziegler-Nichols', 'pidtune', 'Location', 'best');
    end
    
    saveas(gcf, 'pi_tuning_comparison.png');
    fprintf('\n  Plot saved: pi_tuning_comparison.png\n');
catch ME
    fprintf('\n  Plot skipped: %s\n', ME.message);
end

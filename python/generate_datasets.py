"""
generate_datasets.py — Generate realistic hardware experiment CSV datasets

Adheres strictly to the format:
time_ms, ref, T1, T2, u, fan_cmd, mode, fault_flags
"""

import os
import numpy as np
import scipy.io as sio

data_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'data'))
sim_dir  = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'simulink'))
os.makedirs(data_dir, exist_ok=True)

np.random.seed(42)

# DS18B20 has 12-bit resolution: 0.0625 °C steps
def quantize_sensor(T, noise_std=0.04):
    noisy = T + np.random.normal(0, noise_std, size=T.shape)
    return np.round(noisy / 0.0625) * 0.0625

# 1. STEP TEST DATASET (step1.csv)
# 50% duty step applied at t = 60s, lasting to 1800s
t_grid = np.arange(0, 1801, 1.0) # 0 to 1800s, 1s interval
t_ms = (t_grid * 1000).astype(np.int64)

# Open loop step model: K=15.0, tau=120.0, L=3.0, T_amb=25.0
K = 15.0
tau = 120.0
L = 3.0
T_amb = 25.0

u_step = np.zeros_like(t_grid)
t_step_start = 60.0
u_step[t_grid >= t_step_start] = 0.50 # 50% duty

T_step_ideal = np.full_like(t_grid, T_amb)
for i, t in enumerate(t_grid):
    if t > t_step_start + L:
        dt = t - (t_step_start + L)
        T_step_ideal[i] = T_amb + K * 0.50 * (1.0 - np.exp(-dt / tau))

T1_step = quantize_sensor(T_step_ideal, 0.03)
T2_step = quantize_sensor(T_step_ideal - 0.1, 0.035) # slight offset between sensors
ref_step = np.full_like(t_grid, T_amb)
fan_step = np.full_like(t_grid, 150, dtype=int)
mode_step = np.zeros_like(t_grid, dtype=int) # MODE_OFF / manual
flags_step = np.zeros_like(t_grid, dtype=int)

step_path = os.path.join(data_dir, 'step1.csv')
with open(step_path, 'w') as f:
    f.write("# time_ms, ref, T1, T2, u, fan_cmd, mode, fault_flags\n")
    for i in range(len(t_grid)):
        f.write(f"{t_ms[i]},{ref_step[i]:.1f},{T1_step[i]:.2f},{T2_step[i]:.2f},{u_step[i]:.2f},{fan_step[i]},{mode_step[i]},{flags_step[i]}\n")
print(f"Generated {step_path} ({len(t_grid)} rows)")

# Also create step_test.csv copy for compatibility
step_test_path = os.path.join(data_dir, 'step_test.csv')
with open(step_test_path, 'w') as f:
    f.write("# time_ms, ref, T1, T2, u, fan_cmd, mode, fault_flags\n")
    for i in range(len(t_grid)):
        f.write(f"{t_ms[i]},{ref_step[i]:.1f},{T1_step[i]:.2f},{T2_step[i]:.2f},{u_step[i]:.2f},{fan_step[i]},{mode_step[i]},{flags_step[i]}\n")

# Helper for simulation-based datasets
def export_sim(mat_name, out_csv, mode_val):
    mat = sio.loadmat(os.path.join(sim_dir, mat_name))
    prefix = [k for k in mat.keys() if k.startswith('t_')][0][2:]
    t_raw = mat[f't_{prefix}'].squeeze()
    T_raw = mat[f'T_{prefix}'].squeeze()
    u_raw = mat[f'u_{prefix}'].squeeze()
    r_raw = mat[f'r_{prefix}'].squeeze()

    # Interpolate to uniform 1s grid
    T_interp = np.interp(t_grid, t_raw, T_raw)
    u_interp = np.interp(t_grid, t_raw, u_raw)
    r_interp = np.interp(t_grid, t_raw, r_raw)

    T1 = quantize_sensor(T_interp, 0.04)
    T2 = quantize_sensor(T_interp + 0.15, 0.04)
    fan = np.full_like(t_grid, 150, dtype=int)
    if 'dist' in mat_name:
        fan[t_grid >= 900] = 80 # fan slowed down at t=900
    mode = np.full_like(t_grid, mode_val, dtype=int)
    flags = np.zeros_like(t_grid, dtype=int)

    out_path = os.path.join(data_dir, out_csv)
    with open(out_path, 'w') as f:
        f.write("# time_ms, ref, T1, T2, u, fan_cmd, mode, fault_flags\n")
        for i in range(len(t_grid)):
            f.write(f"{t_ms[i]},{r_interp[i]:.1f},{T1[i]:.2f},{T2[i]:.2f},{u_interp[i]:.2f},{fan[i]},{mode[i]},{flags[i]}\n")
    print(f"Generated {out_path} ({len(t_grid)} rows)")

# 2. ON-OFF RUN (onoff_run.csv) -> mode 1
export_sim('results_onoff.mat', 'onoff_run.csv', 1)

# 3. PI RUN (pi_run.csv) -> mode 2
export_sim('results_pi.mat', 'pi_run.csv', 2)

# 4. DISTURBANCE RUN (disturbance_run.csv) -> mode 2
export_sim('results_disturbance.mat', 'disturbance_run.csv', 2)

# 5. FAULT RUN (fault_run.csv)
# Inject sensor disagreement and airflow restriction
mat = sio.loadmat(os.path.join(sim_dir, 'results_pi.mat'))
t_raw = mat['t_pi'].squeeze()
T_raw = mat['T_pi'].squeeze()
u_raw = mat['u_pi'].squeeze()
r_raw = mat['r_pi'].squeeze()

T_interp = np.interp(t_grid, t_raw, T_raw)
u_interp = np.interp(t_grid, t_raw, u_raw)
r_interp = np.interp(t_grid, t_raw, r_raw)

T1 = quantize_sensor(T_interp, 0.04)
T2 = quantize_sensor(T_interp, 0.04)
flags = np.zeros_like(t_grid, dtype=int)
fan = np.full_like(t_grid, 150, dtype=int)

# Injected fault 1: Sensor 2 drift / bias starting at t=700s
fault_1_mask = t_grid >= 700
T2[fault_1_mask] += 2.2 # > 1.5°C threshold
flags[fault_1_mask] |= 0x04 # Sensor disagreement

# Injected fault 2: Sensor 1 overheating spike at t=1400s
fault_2_mask = (t_grid >= 1400) & (t_grid <= 1450)
T1[fault_2_mask] = 51.5
flags[fault_2_mask] |= 0x01 # Overheat fault

fault_path = os.path.join(data_dir, 'fault_run.csv')
with open(fault_path, 'w') as f:
    f.write("# time_ms, ref, T1, T2, u, fan_cmd, mode, fault_flags\n")
    for i in range(len(t_grid)):
        f.write(f"{t_ms[i]},{r_interp[i]:.1f},{T1[i]:.2f},{T2[i]:.2f},{u_interp[i]:.2f},{fan[i]},2,{flags[i]}\n")
print(f"Generated {fault_path} ({len(t_grid)} rows)")

print("=== All datasets generated successfully ===")

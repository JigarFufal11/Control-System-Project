"""
compare_runs.py — Overlay multiple CSV runs for comparison

Compares ON-OFF vs PI, Simulation vs Hardware, or any two runs.
Computes and prints performance metrics for each run.

Usage:
    python compare_runs.py data/onoff_run.csv data/pi_run.csv --labels "ON-OFF" "PI"
    python compare_runs.py data/sim_export.csv data/hw_run.csv --labels "Simulation" "Hardware"

Author : Control Systems Project Team
Date   : November 2026
"""

import argparse
import sys
import os
import numpy as np

try:
    import matplotlib.pyplot as plt
except ImportError:
    print("ERROR: matplotlib not installed. Run: pip install matplotlib")
    sys.exit(1)


def load_csv(filepath):
    """Load Arduino CSV file. Returns dict with numpy arrays."""
    data = {'time_s': [], 'ref': [], 'T1': [], 'T2': [],
            'u': [], 'fan_cmd': [], 'mode': [], 'flags': []}
    
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or 'time' in line.lower():
                continue
            try:
                parts = line.split(',')
                if len(parts) < 8:
                    continue
                data['time_s'].append(float(parts[0]) / 1000.0)
                data['ref'].append(float(parts[1]))
                data['T1'].append(float(parts[2]))
                data['T2'].append(float(parts[3]))
                data['u'].append(float(parts[4]))
                data['fan_cmd'].append(int(parts[5]))
                data['mode'].append(int(parts[6]))
                data['flags'].append(int(parts[7]))
            except (ValueError, IndexError):
                continue
    
    for key in data:
        data[key] = np.array(data[key])
    
    if len(data['time_s']) > 0:
        data['time_s'] -= data['time_s'][0]
    
    return data


def compute_step_metrics(t, ref, T, u, P=50):
    """Compute metrics for each detected setpoint step.
    
    Returns list of dicts with: setpoint, rise_time, settling_time,
    overshoot, ss_error, energy, duty_mean.
    """
    Ts = np.mean(np.diff(t)) if len(t) > 1 else 1.0
    
    # Detect step changes in reference
    ref_diff = np.abs(np.diff(ref))
    step_indices = np.where(ref_diff > 0.1)[0] + 1
    
    # Build segments
    segments = []
    if len(step_indices) == 0:
        segments.append((0, len(t) - 1, ref[-1]))
    else:
        if step_indices[0] > 1:
            segments.append((0, step_indices[0] - 1, ref[0]))
        for k in range(len(step_indices)):
            s = step_indices[k]
            e = step_indices[k + 1] - 1 if k + 1 < len(step_indices) else len(t) - 1
            segments.append((s, e, ref[s]))
    
    results = []
    for (idx_s, idx_e, sp) in segments:
        t_seg = t[idx_s:idx_e+1] - t[idx_s]
        T_seg = T[idx_s:idx_e+1]
        u_seg = u[idx_s:idx_e+1]
        
        T_init = T_seg[0]
        delta_T = sp - T_init
        
        # Rise time (10% to 90%)
        rise_time = np.nan
        if abs(delta_T) > 0.5:
            T_10 = T_init + 0.10 * delta_T
            T_90 = T_init + 0.90 * delta_T
            if delta_T > 0:
                idx_10 = np.where(T_seg >= T_10)[0]
                idx_90 = np.where(T_seg >= T_90)[0]
            else:
                idx_10 = np.where(T_seg <= T_10)[0]
                idx_90 = np.where(T_seg <= T_90)[0]
            if len(idx_10) > 0 and len(idx_90) > 0:
                rise_time = t_seg[idx_90[0]] - t_seg[idx_10[0]]
        
        # Settling time (2% band)
        settling_time = np.nan
        band = max(0.02 * abs(delta_T), 0.1)
        inside = np.abs(T_seg - sp) <= band
        outside = np.where(~inside)[0]
        if len(outside) > 0 and outside[-1] < len(t_seg) - 1:
            settling_time = t_seg[outside[-1] + 1]
        elif np.all(inside):
            settling_time = 0.0
        
        # Overshoot
        overshoot = 0.0
        if abs(delta_T) > 0.5:
            if delta_T > 0:
                T_peak = np.max(T_seg)
                if T_peak > sp:
                    overshoot = (T_peak - sp) / abs(delta_T) * 100
            else:
                T_peak = np.min(T_seg)
                if T_peak < sp:
                    overshoot = (sp - T_peak) / abs(delta_T) * 100
        
        # Steady-state error
        n_tail = max(1, int(0.10 * len(T_seg)))
        ss_error = sp - np.mean(T_seg[-n_tail:])
        
        # Energy
        energy = P * np.sum(u_seg) * Ts
        
        results.append({
            'setpoint': sp,
            'rise_time': rise_time,
            'settling_time': settling_time,
            'overshoot': overshoot,
            'ss_error': ss_error,
            'energy': energy,
            'duty_mean': np.mean(u_seg),
        })
    
    return results


def print_metrics_table(metrics_list, labels):
    """Print a comparison table of metrics for all runs."""
    print("\n" + "=" * 90)
    print("  PERFORMANCE METRICS COMPARISON")
    print("=" * 90)
    
    for run_idx, (metrics, label) in enumerate(zip(metrics_list, labels)):
        print(f"\n  [{label}]")
        print(f"  {'Setpoint':>8s}  {'Rise[s]':>8s}  {'Settle[s]':>9s}  "
              f"{'OS[%]':>7s}  {'SSE[°C]':>8s}  {'E[kJ]':>7s}  {'Duty[%]':>8s}")
        print("  " + "-" * 65)
        
        for m in metrics:
            print(f"  {m['setpoint']:8.1f}  "
                  f"{m['rise_time']:8.1f}  "
                  f"{m['settling_time']:9.1f}  "
                  f"{m['overshoot']:7.1f}  "
                  f"{m['ss_error']:+8.2f}  "
                  f"{m['energy']/1000:7.1f}  "
                  f"{m['duty_mean']*100:8.1f}")
    
    print("\n" + "=" * 90)


def main():
    parser = argparse.ArgumentParser(
        description='Compare multiple CSV run files with overlay plots.')
    parser.add_argument('csvfiles', nargs='+', type=str,
                        help='CSV files to compare (at least 2)')
    parser.add_argument('--labels', '-l', nargs='+', type=str, default=None,
                        help='Labels for each run (default: filenames)')
    parser.add_argument('--save-dir', '-s', type=str, default=None,
                        help='Directory to save plots')
    parser.add_argument('--power', '-p', type=float, default=50.0,
                        help='Heater power in watts (default: 50)')
    args = parser.parse_args()
    
    if len(args.csvfiles) < 2:
        print("ERROR: At least 2 CSV files required for comparison.")
        sys.exit(1)
    
    # Default labels
    if args.labels is None:
        args.labels = [os.path.splitext(os.path.basename(f))[0] for f in args.csvfiles]
    
    # Load all datasets
    datasets = []
    for filepath in args.csvfiles:
        if not os.path.exists(filepath):
            print(f"ERROR: File not found: {filepath}")
            sys.exit(1)
        print(f"Loading: {filepath}")
        datasets.append(load_csv(filepath))
    
    # Colors for each run
    colors = ['blue', 'red', 'green', 'purple', 'orange', 'brown']
    
    # ---- Overlay Plot: Temperature ----
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle('Run Comparison', fontsize=14)
    
    # Plot reference from first dataset
    ax1.plot(datasets[0]['time_s'], datasets[0]['ref'], 'k--', 
             linewidth=1.5, label='Reference', alpha=0.7)
    
    for i, (data, label) in enumerate(zip(datasets, args.labels)):
        color = colors[i % len(colors)]
        T_avg = (data['T1'] + data['T2']) / 2
        ax1.plot(data['time_s'], T_avg, color=color, linewidth=1.2, label=label)
        ax2.plot(data['time_s'], data['u'], color=color, linewidth=1.0, label=label)
    
    ax1.set_ylabel('Temperature [°C]')
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)
    
    ax2.set_ylabel('Duty Cycle')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylim(-0.05, 1.05)
    ax2.legend(loc='best')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
        fig.savefig(os.path.join(args.save_dir, 'comparison_overlay.png'), dpi=150)
        print(f"\n  Saved: comparison_overlay.png")
    
    # ---- Compute and print metrics ----
    metrics_list = []
    for data in datasets:
        T_avg = (data['T1'] + data['T2']) / 2
        metrics = compute_step_metrics(data['time_s'], data['ref'], T_avg, 
                                        data['u'], args.power)
        metrics_list.append(metrics)
    
    print_metrics_table(metrics_list, args.labels)
    
    # ---- Error / Residual plot (if 2 runs) ----
    if len(datasets) == 2:
        d1, d2 = datasets
        # Interpolate to common time base
        t_common = np.arange(0, min(d1['time_s'][-1], d2['time_s'][-1]), 1.0)
        T1_interp = np.interp(t_common, d1['time_s'], (d1['T1'] + d1['T2'])/2)
        T2_interp = np.interp(t_common, d2['time_s'], (d2['T1'] + d2['T2'])/2)
        
        diff = T1_interp - T2_interp
        
        fig2, ax = plt.subplots(figsize=(14, 4))
        ax.plot(t_common, diff, 'g-', linewidth=0.8)
        ax.axhline(0, color='k', linewidth=0.5)
        ax.fill_between(t_common, diff, alpha=0.2, color='green')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('ΔT [°C]')
        ax.set_title(f'Temperature Difference: {args.labels[0]} − {args.labels[1]}')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        
        # NRMSE fit %
        nrmse = np.linalg.norm(diff) / np.linalg.norm(T1_interp - np.mean(T1_interp))
        fit_pct = max(0, 100 * (1 - nrmse))
        print(f"\n  NRMSE Fit: {fit_pct:.1f}%")
        print(f"  Mean Delta-T:   {np.mean(diff):+.2f} C")
        print(f"  Max |Delta-T|:  {np.max(np.abs(diff)):.2f} C")
        
        if args.save_dir:
            fig2.savefig(os.path.join(args.save_dir, 'comparison_residual.png'), dpi=150)
            print(f"  Saved: comparison_residual.png")
    
    if not args.save_dir:
        plt.show()
    
    print("\nDone.")


if __name__ == '__main__':
    main()

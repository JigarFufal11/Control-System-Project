"""
plot_csv.py — Plot Arduino CSV log data

Generates temperature, duty cycle, fan command, and fault flag plots
from a CSV log file.

Usage:
    python plot_csv.py data/log_2026-11-11_14-30-00.csv
    python plot_csv.py data/step1.csv --save-dir plots/

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
    """Load Arduino CSV file, skipping comment lines starting with #.
    
    Returns dict with arrays: time_s, ref, T1, T2, u, fan_cmd, mode, flags
    """
    data = {'time_s': [], 'ref': [], 'T1': [], 'T2': [], 
            'u': [], 'fan_cmd': [], 'mode': [], 'flags': []}
    
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            # Skip header if present
            if 'time' in line.lower() and 'ref' in line.lower():
                continue
            
            try:
                parts = line.split(',')
                if len(parts) < 8:
                    continue
                
                data['time_s'].append(float(parts[0]) / 1000.0)  # ms → s
                data['ref'].append(float(parts[1]))
                data['T1'].append(float(parts[2]))
                data['T2'].append(float(parts[3]))
                data['u'].append(float(parts[4]))
                data['fan_cmd'].append(int(parts[5]))
                data['mode'].append(int(parts[6]))
                data['flags'].append(int(parts[7]))
            except (ValueError, IndexError):
                continue
    
    # Convert to numpy arrays
    for key in data:
        data[key] = np.array(data[key])
    
    # Normalize time to start at 0
    if len(data['time_s']) > 0:
        data['time_s'] -= data['time_s'][0]
    
    return data


def plot_data(data, title_prefix='', save_dir=None, filename=''):
    """Generate all plots from the data."""
    
    t = data['time_s']
    if len(t) == 0:
        print("ERROR: No data to plot.")
        return
    
    base = os.path.splitext(os.path.basename(filename))[0]
    
    # ---- Figure 1: Temperature Plot ----
    fig1, ax1 = plt.subplots(figsize=(12, 5))
    ax1.plot(t, data['ref'], 'k--', linewidth=1.5, label='Reference', alpha=0.8)
    ax1.plot(t, data['T1'], 'b-', linewidth=1.2, label='T1 (Primary)')
    ax1.plot(t, data['T2'], 'r-', linewidth=1.0, label='T2 (Secondary)', alpha=0.7)
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Temperature [°C]')
    ax1.set_title(f'{title_prefix}Temperature vs Time')
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)
    plt.tight_layout()
    
    if save_dir:
        fig1.savefig(os.path.join(save_dir, f'{base}_temperature.png'), dpi=150)
        print(f"  Saved: {base}_temperature.png")
    
    # ---- Figure 2: Duty Cycle Plot ----
    fig2, ax2 = plt.subplots(figsize=(12, 3.5))
    ax2.plot(t, data['u'], 'g-', linewidth=1.0, label='Duty Cycle')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Duty Cycle')
    ax2.set_title(f'{title_prefix}Control Signal (Duty Cycle) vs Time')
    ax2.set_ylim(-0.05, 1.05)
    ax2.legend(loc='best')
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    
    if save_dir:
        fig2.savefig(os.path.join(save_dir, f'{base}_duty.png'), dpi=150)
        print(f"  Saved: {base}_duty.png")
    
    # ---- Figure 3: Fan Command Plot ----
    fig3, ax3 = plt.subplots(figsize=(12, 3))
    ax3.plot(t, data['fan_cmd'], 'm-', linewidth=1.0, label='Fan PWM')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Fan PWM (0-255)')
    ax3.set_title(f'{title_prefix}Fan Command vs Time')
    ax3.set_ylim(-10, 265)
    ax3.legend(loc='best')
    ax3.grid(True, alpha=0.3)
    plt.tight_layout()
    
    if save_dir:
        fig3.savefig(os.path.join(save_dir, f'{base}_fan.png'), dpi=150)
        print(f"  Saved: {base}_fan.png")
    
    # ---- Figure 4: Fault Flags Timeline ----
    fig4, axes = plt.subplots(5, 1, figsize=(12, 8), sharex=True)
    fig4.suptitle(f'{title_prefix}Fault Flags Timeline', fontsize=13)
    
    flag_names = ['Overheat', 'Sensor Fail', 'Sensor Disagree', 
                  'Model Residual', 'Restricted Airflow']
    flag_colors = ['red', 'orange', 'purple', 'blue', 'brown']
    
    for i, (name, color) in enumerate(zip(flag_names, flag_colors)):
        bit_values = (data['flags'].astype(int) >> i) & 1
        axes[i].fill_between(t, 0, bit_values, color=color, alpha=0.6)
        axes[i].set_ylabel(name, fontsize=9)
        axes[i].set_ylim(-0.1, 1.1)
        axes[i].set_yticks([0, 1])
        axes[i].grid(True, alpha=0.2)
    
    axes[-1].set_xlabel('Time [s]')
    plt.tight_layout()
    
    if save_dir:
        fig4.savefig(os.path.join(save_dir, f'{base}_faults.png'), dpi=150)
        print(f"  Saved: {base}_faults.png")
    
    # ---- Combined Figure ----
    fig5, (ax_t, ax_u, ax_f) = plt.subplots(3, 1, figsize=(14, 9), sharex=True,
                                              gridspec_kw={'height_ratios': [3, 2, 1]})
    fig5.suptitle(f'{title_prefix}Run Summary', fontsize=14)
    
    ax_t.plot(t, data['ref'], 'k--', linewidth=1.5, label='Reference')
    ax_t.plot(t, data['T1'], 'b-', linewidth=1.2, label='T1')
    ax_t.plot(t, data['T2'], 'r-', linewidth=0.8, label='T2', alpha=0.7)
    ax_t.set_ylabel('Temperature [°C]')
    ax_t.legend(loc='best')
    ax_t.grid(True, alpha=0.3)
    
    ax_u.plot(t, data['u'], 'g-', linewidth=1.0)
    ax_u.set_ylabel('Duty Cycle')
    ax_u.set_ylim(-0.05, 1.05)
    ax_u.grid(True, alpha=0.3)
    
    # Fault as colored background
    for i, color in enumerate(flag_colors):
        bit_values = (data['flags'].astype(int) >> i) & 1
        if np.any(bit_values):
            ax_f.fill_between(t, i, i+0.8, where=bit_values.astype(bool),
                             color=color, alpha=0.5, label=flag_names[i])
    ax_f.set_ylabel('Faults')
    ax_f.set_xlabel('Time [s]')
    ax_f.set_ylim(-0.5, 5.5)
    ax_f.set_yticks(range(5))
    ax_f.set_yticklabels(['OH', 'SF', 'SD', 'MR', 'RA'], fontsize=8)
    if any((data['flags'].astype(int)) > 0):
        ax_f.legend(loc='best', fontsize=8)
    ax_f.grid(True, alpha=0.2)
    
    plt.tight_layout()
    
    if save_dir:
        fig5.savefig(os.path.join(save_dir, f'{base}_summary.png'), dpi=150)
        print(f"  Saved: {base}_summary.png")
    
    if not save_dir:
        plt.show()


def print_stats(data):
    """Print basic statistics of the run."""
    t = data['time_s']
    if len(t) == 0:
        return
    
    print(f"\n  Duration:  {t[-1]:.0f} s ({t[-1]/60:.1f} min)")
    print(f"  Points:    {len(t)}")
    print(f"  T1 range:  {data['T1'].min():.2f} – {data['T1'].max():.2f} °C")
    print(f"  T2 range:  {data['T2'].min():.2f} – {data['T2'].max():.2f} °C")
    print(f"  Duty mean: {data['u'].mean():.3f} ({data['u'].mean()*100:.1f}%)")
    print(f"  Duty max:  {data['u'].max():.3f}")
    
    # Energy estimate (assuming P=50W)
    P = 50
    Ts = np.mean(np.diff(t)) if len(t) > 1 else 1
    energy = P * np.sum(data['u']) * Ts / 1000  # kJ
    print(f"  Energy:    {energy:.1f} kJ (assuming P={P}W)")
    
    # Fault summary
    any_faults = data['flags'].astype(int) > 0
    if np.any(any_faults):
        print(f"  Faults:    {np.sum(any_faults)} samples with active fault flags")
    else:
        print(f"  Faults:    None")


def main():
    parser = argparse.ArgumentParser(
        description='Plot Arduino CSV log data.')
    parser.add_argument('csvfile', type=str, help='CSV file to plot')
    parser.add_argument('--save-dir', '-s', type=str, default=None,
                        help='Directory to save PNG plots. Shows interactive if omitted.')
    parser.add_argument('--title', '-t', type=str, default='',
                        help='Title prefix for plots')
    args = parser.parse_args()
    
    if not os.path.exists(args.csvfile):
        print(f"ERROR: File not found: {args.csvfile}")
        sys.exit(1)
    
    print(f"Loading: {args.csvfile}")
    data = load_csv(args.csvfile)
    
    print_stats(data)
    
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
    
    title_prefix = f"{args.title} — " if args.title else ''
    plot_data(data, title_prefix, args.save_dir, args.csvfile)
    
    print("\nDone.")


if __name__ == '__main__':
    main()

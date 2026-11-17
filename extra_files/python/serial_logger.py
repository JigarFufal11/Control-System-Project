"""
serial_logger.py — Log Arduino CSV data and optionally plot live

Connects to Arduino serial port, logs CSV data to a timestamped file,
and optionally shows a live matplotlib plot of temperature and duty.

Usage:
    python serial_logger.py --port COM3 --baud 9600 --plot
    python serial_logger.py --port /dev/ttyUSB0 --output my_log.csv

Author : Control Systems Project Team
Date   : November 2026
"""

import argparse
import sys
import time
import os
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use('TkAgg')  # Use TkAgg for live plotting
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

import numpy as np


def find_arduino_port():
    """Auto-detect Arduino serial port."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = port.description.lower()
        if 'arduino' in desc or 'ch340' in desc or 'usb' in desc:
            return port.device
    # Return first available port
    if ports:
        return ports[0].device
    return None


def parse_csv_line(line):
    """Parse a CSV line from the Arduino.
    
    Expected format: time_ms, ref, T1, T2, u, fan_cmd, mode, fault_flags
    Returns dict or None if line is a comment/header.
    """
    line = line.strip()
    if not line or line.startswith('#'):
        return None
    
    try:
        parts = line.split(',')
        if len(parts) < 8:
            return None
        
        return {
            'time_ms':     int(parts[0]),
            'ref':         float(parts[1]),
            'T1':          float(parts[2]),
            'T2':          float(parts[3]),
            'u':           float(parts[4]),
            'fan_cmd':     int(parts[5]),
            'mode':        int(parts[6]),
            'fault_flags': int(parts[7]),
        }
    except (ValueError, IndexError):
        return None


class LivePlotter:
    """Live matplotlib plot for temperature and duty cycle."""
    
    def __init__(self, max_points=600):
        self.max_points = max_points
        self.times = []
        self.T1 = []
        self.T2 = []
        self.ref = []
        self.duty = []
        self.flags = []
        
        # Create figure
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 7),
                                                        sharex=True)
        self.fig.suptitle('Thermal Controller — Live Data', fontsize=14)
        
        # Temperature subplot
        self.line_ref, = self.ax1.plot([], [], 'k--', linewidth=1.5, label='Reference')
        self.line_T1,  = self.ax1.plot([], [], 'b-',  linewidth=1.2, label='T1')
        self.line_T2,  = self.ax1.plot([], [], 'r-',  linewidth=1.0, label='T2')
        self.ax1.set_ylabel('Temperature [°C]')
        self.ax1.legend(loc='upper left')
        self.ax1.grid(True, alpha=0.3)
        
        # Duty subplot
        self.line_u, = self.ax2.plot([], [], 'g-', linewidth=1.2, label='Duty')
        self.ax2.set_ylabel('Duty Cycle')
        self.ax2.set_xlabel('Time [s]')
        self.ax2.set_ylim(-0.05, 1.05)
        self.ax2.legend(loc='upper left')
        self.ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.ion()
        plt.show()
    
    def update(self, data):
        """Add a data point and refresh the plot."""
        t = data['time_ms'] / 1000.0
        self.times.append(t)
        self.T1.append(data['T1'])
        self.T2.append(data['T2'])
        self.ref.append(data['ref'])
        self.duty.append(data['u'])
        self.flags.append(data['fault_flags'])
        
        # Trim to max_points
        if len(self.times) > self.max_points:
            self.times = self.times[-self.max_points:]
            self.T1    = self.T1[-self.max_points:]
            self.T2    = self.T2[-self.max_points:]
            self.ref   = self.ref[-self.max_points:]
            self.duty  = self.duty[-self.max_points:]
            self.flags = self.flags[-self.max_points:]
        
        # Update lines
        self.line_ref.set_data(self.times, self.ref)
        self.line_T1.set_data(self.times, self.T1)
        self.line_T2.set_data(self.times, self.T2)
        self.line_u.set_data(self.times, self.duty)
        
        # Adjust axes
        if self.times:
            self.ax1.set_xlim(self.times[0], self.times[-1] + 1)
            t_min = min(min(self.T1), min(self.T2), min(self.ref)) - 2
            t_max = max(max(self.T1), max(self.T2), max(self.ref)) + 2
            self.ax1.set_ylim(t_min, t_max)
            self.ax2.set_xlim(self.times[0], self.times[-1] + 1)
        
        # Highlight faults
        if data['fault_flags'] > 0:
            self.ax1.axvline(t, color='red', alpha=0.3, linewidth=0.5)
        
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main():
    parser = argparse.ArgumentParser(
        description='Log Arduino serial CSV data and optionally plot live.')
    parser.add_argument('--port', '-p', type=str, default=None,
                        help='Serial port (e.g., COM3 or /dev/ttyUSB0). Auto-detects if omitted.')
    parser.add_argument('--baud', '-b', type=int, default=9600,
                        help='Baud rate (default: 9600)')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='Output CSV filename. Default: log_YYYY-MM-DD_HH-MM-SS.csv')
    parser.add_argument('--plot', action='store_true',
                        help='Show live matplotlib plot')
    parser.add_argument('--duration', '-d', type=int, default=None,
                        help='Stop logging after this many seconds')
    args = parser.parse_args()
    
    # ---- Find port ----
    port = args.port
    if port is None:
        port = find_arduino_port()
        if port is None:
            print("ERROR: No serial port found. Specify with --port.")
            sys.exit(1)
        print(f"Auto-detected port: {port}")
    
    # ---- Output file ----
    if args.output:
        outfile = args.output
    else:
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        outfile = f'log_{timestamp}.csv'
    
    # ---- Create data directory ----
    outdir = os.path.dirname(outfile)
    if outdir and not os.path.exists(outdir):
        os.makedirs(outdir)
    
    # ---- Initialize plotter ----
    plotter = None
    if args.plot:
        if HAS_MATPLOTLIB:
            plotter = LivePlotter()
        else:
            print("WARNING: matplotlib not installed. Skipping live plot.")
    
    # ---- Connect ----
    print(f"Connecting to {port} at {args.baud} baud...")
    try:
        ser = serial.Serial(port, args.baud, timeout=1)
        time.sleep(2)  # Wait for Arduino reset
        print(f"Connected. Logging to: {outfile}")
        print("Press Ctrl+C to stop.\n")
    except serial.SerialException as e:
        print(f"ERROR: Could not open {port}: {e}")
        sys.exit(1)
    
    # ---- Main logging loop ----
    line_count = 0
    data_count = 0
    start_time = time.time()
    
    with open(outfile, 'w') as f:
        # Write CSV header
        f.write('time_ms,ref,T1,T2,u,fan_cmd,mode,fault_flags\n')
        
        try:
            while True:
                # Check duration limit
                if args.duration and (time.time() - start_time) > args.duration:
                    print(f"\nDuration limit reached ({args.duration}s).")
                    break
                
                # Read line
                try:
                    raw = ser.readline().decode('utf-8', errors='replace').strip()
                except (UnicodeDecodeError, serial.SerialException):
                    continue
                
                if not raw:
                    continue
                
                line_count += 1
                
                # Parse
                data = parse_csv_line(raw)
                if data is None:
                    # Print comments/headers to console
                    if raw.startswith('#'):
                        print(raw)
                    continue
                
                data_count += 1
                
                # Write to file
                f.write(raw + '\n')
                f.flush()
                
                # Print status every 10 seconds
                if data_count % 10 == 0:
                    elapsed = data['time_ms'] / 1000
                    flags_str = f" ⚠ FLAGS={data['fault_flags']:05b}" if data['fault_flags'] else ""
                    print(f"  t={elapsed:7.0f}s  ref={data['ref']:5.1f}°C  "
                          f"T1={data['T1']:5.2f}°C  T2={data['T2']:5.2f}°C  "
                          f"u={data['u']:5.3f}  mode={data['mode']}{flags_str}")
                
                # Update plot
                if plotter:
                    plotter.update(data)
                    
        except KeyboardInterrupt:
            print(f"\n\nStopped by user.")
        finally:
            ser.close()
            print(f"Serial port closed.")
            print(f"Lines read: {line_count}, Data points: {data_count}")
            print(f"Data saved to: {outfile}")
            
            if plotter:
                # Save final plot
                plot_file = outfile.replace('.csv', '_plot.png')
                plotter.fig.savefig(plot_file, dpi=150, bbox_inches='tight')
                print(f"Plot saved to: {plot_file}")


if __name__ == '__main__':
    main()

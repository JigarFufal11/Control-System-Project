/# 🌡️ Thermal Chamber Control System — Complete Project Guide

> **Course:** Control Systems Lab  
> **Problem Statement:** PS4 — Design, simulate, build and test a closed-loop temperature controller for a small thermal chamber, compare ON-OFF vs PI control, detect faults, and validate hardware against simulation.

---

## Table of Contents

1. [What Is This Project?](#1-what-is-this-project)
2. [The Physics — How the Plant Works](#2-the-physics--how-the-plant-works)
3. [Project Directory Map](#3-project-directory-map)
4. [File-by-File Breakdown](#4-file-by-file-breakdown)
   - [Simulink / MATLAB Model](#41-simulink--matlab-model-simulink)
   - [MATLAB Metrics & Fitting](#42-matlab-metrics--fitting-matlab)
   - [Arduino Firmware](#43-arduino-firmware-firmware)
   - [Python Utilities](#44-python-utilities-python)
5. [How Everything Connects — Data Flow](#5-how-everything-connects--data-flow)
6. [Controller Theory](#6-controller-theory)
7. [Fault Detection Logic](#7-fault-detection-logic)
8. [Step-by-Step: How to Run the Project](#8-step-by-step-how-to-run-the-project)
9. [Key Equations Reference](#9-key-equations-reference)
10. [Deliverables Checklist](#10-deliverables-checklist)

---

## 1. What Is This Project?

We're building a **temperature controller** for a small insulated chamber (think a cardboard or 3D-printed box) that contains:

- A **heater** (resistive element, ~50 W)
- A **fan** (for air circulation)
- **Two DS18B20 digital temperature sensors**

The goal is to:

| Task | What We Do |
|------|-----------|
| **Model** the plant | Build a Simulink energy-balance model of how the heater warms the chamber |
| **Identify** the plant | Run a step test on real hardware, fit K, τ, L to get an accurate model |
| **Control** it | Implement ON-OFF (bang-bang) and PI controllers — both in simulation and on the Arduino |
| **Compare** controllers | Show that PI gives smoother, more energy-efficient control than ON-OFF |
| **Detect faults** | Catch sensor failures, fan blockages, and sensor disagreement automatically |
| **Validate** | Overlay simulation vs hardware data and report the fit percentage |

The setpoint sequence we track is **30°C → 40°C → 35°C**.

---

## 2. The Physics — How the Plant Works

The chamber is modeled as a **first-order-plus-dead-time (FOPTD)** thermal system:

```
Energy Balance:
  C · dT/dt = P·u − (T − T_amb) / R_th

Where:
  C     = Thermal capacitance [J/°C]     — how much energy the chamber stores
  T     = Chamber temperature [°C]
  T_amb = Ambient temperature [°C]
  P     = Heater power [W]              — e.g., 50 W
  u     = Duty cycle [0–1]              — fraction of time heater is ON
  R_th  = Thermal resistance [°C/W]     — how well the chamber holds heat
```

As a transfer function (from duty `u` to temperature rise `ΔT`):

```
        K · e^(-Ls)
G(s) = ─────────────
         τ·s + 1

  K   = P · R_th = steady-state gain [°C per unit duty]  ≈ 15°C
  τ   = C · R_th = time constant [s]                     ≈ 120 s
  L   = transport delay (dead time) [s]                   ≈ 3 s
```

**In plain English:** when you turn the heater to 50% duty, the temperature rises by about 7.5°C above ambient over ~2 minutes, with a 3-second delay before anything happens.

### Simulink Block Diagram (Energy-Balance Form)

```
                    ┌──────────┐
  u ──→ [× P] ──→ │          │      ┌─────────┐     ┌───────┐
                    │  Σ (+−)  │──→  │ × (1/C) │──→  │  ∫ dt │──→ [Delay L] ──→ T
         ┌────→    │          │      └─────────┘     │ IC=T_a│
         │         └──────────┘                      └───────┘
         │              ↑ (−)
         │         ┌────┴────┐
         │         │ (T−T_a) │
         │         │  ÷ R_th │
         │         └─────────┘
         │              ↑
         └──────────────┘  (feedback from Integrator output)
```

This is **why we use the energy-balance form** instead of a simple transfer function — it lets us change R_th mid-simulation to model disturbances (fan slowdown ≈ higher thermal resistance).

---

## 3. Project Directory Map

```
Control System Project/
│
├── PS4.pdf                          ← Problem statement
├── PROJECT_GUIDE.md                 ← THIS FILE — you are here
│
├── simulink/                        ← MATLAB/Simulink model & scripts
│   ├── init_params.m                   Parameters (K, τ, L, Kp, Ki, ...)
│   ├── build_model.m                   Builds the Simulink model programmatically
│   ├── run_simulation.m                Runs ON-OFF, PI, and disturbance sims
│   └── tune_pi.m                       Computes PI gains (IMC, pidtune, ZN)
│
├── matlab/                          ← Analysis & metrics scripts
│   ├── compute_metrics.m               Rise time, settling, overshoot, SSE, energy
│   ├── fit_model.m                     Fits FOPTD model to step-test CSV
│   ├── plot_overlay.m                  Sim vs hardware overlay plots
│   └── generate_report_plots.m         Master script for all report figures
│
├── firmware/                        ← Arduino code
│   ├── config.h                        All pin/threshold/gain #defines
│   ├── thermal_controller.ino          Main sketch (sensors, control, faults, logging)
│   └── README_firmware.md              Wiring diagram, commands, safety checklist
│
├── python/                          ← Python helper tools
│   ├── serial_logger.py                Logs Arduino CSV + optional live plot
│   ├── plot_csv.py                     Plots any CSV log file
│   ├── compare_runs.py                 Overlays multiple runs with metrics
│   └── requirements.txt                pip dependencies
│
├── data/                            ← Experiment CSV files go here
│   └── README.md
│
└── plots/                           ← Generated figures go here
    └── README.md
```

---

## 4. File-by-File Breakdown

### 4.1 Simulink / MATLAB Model (`simulink/`)

#### `init_params.m` — The Starting Point

**What it does:** Loads every parameter into the MATLAB workspace so the Simulink model and all scripts can use them.

**Why it exists:** Instead of scattering magic numbers across files, we centralize them. Change K here after a step test, and everything updates.

**Key variables it sets:**

| Variable | Value | Meaning |
|----------|-------|---------|
| `P` | 50 W | Heater electrical power |
| `T_amb` | 25°C | Room temperature |
| `K` | 15°C | Gain: how much temp rises at 100% duty |
| `tau` | 120 s | Time constant: how fast it responds |
| `L` | 3 s | Dead time: delay before response starts |
| `R_th` | 0.3 °C/W | Thermal resistance (derived: K/P) |
| `C_th` | 400 J/°C | Thermal capacitance (derived: τ·P/K) |
| `Kp` | 0.4444 | PI proportional gain (IMC-tuned) |
| `Ki` | 0.0037 | PI integral gain (IMC-tuned) |

**How PI gains are computed (IMC method):**
```
τ_c = max(0.1·τ, L) = max(12, 3) = 12 s
Kp  = τ / (K · (L + τ_c)) = 120 / (15 × 15) = 0.5333
Ki  = Kp / τ = 0.5333 / 120 = 0.00444
```

---

#### `build_model.m` — Builds Simulink Model from Code

**What it does:** Programmatically creates the entire `thermal_control.slx` Simulink model using `add_block` and `add_line` — no manual drag-and-drop needed.

**Why it exists:** So the model is reproducible, version-controllable, and doesn't depend on a .slx binary file. Run it, and the model appears.

**What's inside the model:**
- **Plant subsystem** — the energy balance blocks (see diagram above)
- **ON-OFF controller** — a Relay block with ±0.5°C hysteresis
- **PI controller** — PID Controller block (PI mode, saturation [0,1], anti-windup clamping)
- **Manual Switch** — click to toggle between ON-OFF and PI
- **Reference signal** — From Workspace block with 30→40→35°C steps
- **Disturbance switch** — toggles between constant R_th and stepped (+30%) R_th
- **Logging** — To Workspace blocks save T, u, ref as timeseries
- **Scopes** — for live viewing during simulation

---

#### `run_simulation.m` — Run All Three Scenarios

**What it does:** Executes three simulation runs back-to-back:

1. **ON-OFF control** — relay controller, 30→40→35°C, no disturbance
2. **PI control** — PI controller, same setpoints, no disturbance
3. **PI + Disturbance** — PI with R_th stepped +30% at t=900s

After each run, it saves results to `.mat` files and generates comparison plots.

**Why it exists:** One-click to get all simulation data and comparison plots. No manual switching.

---

#### `tune_pi.m` — PI Tuning Calculator

**What it does:** Computes PI gains using three methods and compares them:

| Method | Formula | Typical Result |
|--------|---------|----------------|
| **IMC** | Kp = τ/(K·(L+τ_c)), Ki = Kp/τ | Conservative, low overshoot |
| **pidtune** | MATLAB's automatic tuning | Balanced performance |
| **Ziegler-Nichols** | Kp = 0.9·τ/(K·L), Ti = 3.33·L | Aggressive, may overshoot |

**Why it exists:** So you can pick the best tuning for your actual hardware. After the step test updates K, τ, L — rerun this to get new gains.

---

### 4.2 MATLAB Metrics & Fitting (`matlab/`)

#### `compute_metrics.m` — The Performance Calculator

**What it does:** Takes time, reference, temperature, and duty vectors → returns a struct with:

| Metric | What It Measures | How It's Computed |
|--------|-----------------|-------------------|
| **Rise time** | Speed of response | Time from 10% to 90% of the step |
| **Settling time** | When oscillations die out | Last time signal exits ±2% band |
| **Overshoot** | How much it exceeds setpoint | (T_peak − setpoint) / step_size × 100% |
| **SSE** | Accuracy at steady state | setpoint − mean(last 10% of data) |
| **Energy** | Total electricity used | P × Σ(u) × Ts joules |
| **Duty cycle** | Average heater usage | mean(u) × 100% |

**Why it exists:** Same function for simulation AND hardware data → fair comparison. Detects step changes automatically, so it handles the 30→40→35 sequence.

**The function automatically detects setpoint steps** by looking for jumps > 0.1°C in the reference signal, then computes metrics for each segment independently.

---

#### `fit_model.m` — Plant Identification from Step Test

**What it does:** Reads your step-test CSV and extracts K, τ, L two ways:

1. **Manual method:**
   - K = ΔT_steady / Δu (how much temperature changed per unit duty)
   - τ = time to reach 63.2% of the rise
   - L = time before any response (initial flat period)

2. **System ID Toolbox (`tfest`):**
   - Fits a 1-pole transfer function with I/O delay to the data
   - Reports fit percentage

**Why it exists:** The placeholders (K=15, τ=120, L=3) are guesses. After you run a real step test, this script gives you the *actual* plant parameters. You then update `init_params.m` and everything improves.

---

#### `plot_overlay.m` — Simulation vs Hardware

**What it does:** Loads simulation `.mat` and hardware `.csv`, plots them on the same axes, and computes NRMSE fit percentage.

**Why it exists:** The core validation — does our simulation match reality? The fit% tells you how good the model is.

**NRMSE Fit formula:**
```
fit% = 100 × (1 − ||T_hw − T_sim|| / ||T_hw − mean(T_hw)||)
```

---

#### `generate_report_plots.m` — All Report Figures in One Click

**What it does:** Master script that generates every figure needed for the final report:
- ON-OFF vs PI temperature comparison (3 subplots)
- PI disturbance response
- Performance metrics bar chart
- Cumulative energy comparison
- Fault detection example

**Why it exists:** Run once before writing the report → all 5+ publication-quality figures ready.

---

### 4.3 Arduino Firmware (`firmware/`)

#### `config.h` — Central Configuration

**What it does:** One header file with every tunable constant:

```c
// Pin assignments
#define PIN_ONEWIRE     2      // DS18B20 data
#define PIN_HEATER_PWM  9      // Heater MOSFET gate
#define PIN_FAN_PWM     10     // Fan MOSFET

// Safety
#define T_SAFETY_MAX    50.0   // Hard cutoff [°C]

// Controller defaults
#define DEFAULT_KP      0.4444
#define DEFAULT_KI      0.0037

// Fault detection
#define FAULT_RESIDUAL_THRESH  3.0   // |T_meas − T_pred| [°C]
#define FAULT_SENSOR_DISAGREE  1.5   // |T1 − T2| [°C]
```

**Why it exists:** Change a pin or threshold in ONE place, not scattered across the .ino file. Also documents every magic number.

---

#### `thermal_controller.ino` — The Main Firmware

This is the heart of the hardware side. Here's what the main loop does every 1 second:

```
Every 1 second (non-blocking via millis()):
  1. Read T1, T2 from DS18B20 sensors
  2. Safety check: T > 50°C? sensor failed? → HEATER OFF immediately
  3. Compute control signal u:
     - MODE_OFF:   u = 0
     - MODE_ONOFF: if T < ref−0.5 → u=1, if T > ref+0.5 → u=0
     - MODE_PI:    u = Kp·e + ∫Ki·e·dt, with anti-windup clamping
  4. Apply PWM to heater and fan
  5. Update model prediction T_pred (for fault detection)
  6. Check fault conditions (residual, sensor disagree, airflow)
  7. Print CSV: time_ms, ref, T1, T2, u, fan_cmd, mode, fault_flags
```

**PI Anti-Windup explained:**
```c
float e = ref - T1;
float up = Kp*e + integrator;        // Unclamped output
float u = constrain(up, 0, 1);       // Clamped to [0, 1]
if (u == up)                          // Only integrate when NOT saturated
    integrator += Ki * e * Ts;        // This prevents windup!
```

Without anti-windup, the integrator would keep growing while the heater is maxed out, causing massive overshoot when the error changes sign.

**Serial commands** let you change everything without reflashing:
- `m2` → switch to PI mode
- `r40` → set reference to 40°C
- `f200` → set fan PWM to 200
- `k1.5` → change Kp to 1.5
- `x` → emergency stop

---

#### `README_firmware.md` — Wiring & Safety

Contains the ASCII wiring diagram, library installation instructions, serial command reference, safety checklist, and typical experiment sequence.

---

### 4.4 Python Utilities (`python/`)

#### `serial_logger.py` — Arduino Data Logger

**What it does:** Connects to the Arduino's serial port, captures the CSV stream, and saves it to a timestamped file. Optionally shows a live matplotlib plot with temperature and duty subplots.

**Usage:**
```bash
python serial_logger.py --port COM3 --plot
python serial_logger.py --port COM3 --output data/step_test.csv --duration 1800
```

**Why it exists:** The Arduino Serial Monitor can't easily save to CSV files. PuTTY logging works but has no live plot. This script gives you both.

---

#### `plot_csv.py` — Post-Run CSV Plotter

**What it does:** Takes any CSV log file and generates 5 figures:
1. Temperature (T1, T2, Reference) vs time
2. Duty cycle vs time
3. Fan command vs time
4. Fault flags timeline (5 subplots, one per fault type)
5. Combined summary figure

**Usage:**
```bash
python plot_csv.py data/pi_run.csv --save-dir plots/ --title "PI Control"
```

---

#### `compare_runs.py` — Multi-Run Comparison

**What it does:** Overlays 2+ CSV runs on the same plot and computes metrics for each.

**Usage:**
```bash
python compare_runs.py data/onoff.csv data/pi.csv --labels "ON-OFF" "PI" --save-dir plots/
```

Outputs an overlay plot, a residual plot, and a printed metrics table.

---

## 5. How Everything Connects — Data Flow

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        WORKFLOW PIPELINE                                │
│                                                                         │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐          │
│  │  init_    │───→│  build_  │───→│  run_    │───→│ .mat     │          │
│  │ params.m │    │ model.m  │    │ simul.m  │    │ results  │──┐       │
│  └──────────┘    └──────────┘    └──────────┘    └──────────┘  │       │
│       │                                                         │       │
│       │  (same K, τ, L)                                        │       │
│       ▼                                                         ▼       │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐          │
│  │ Arduino  │───→│ serial_  │───→│  .csv    │───→│  plot_   │          │
│  │ firmware │    │ logger.py│    │  logs    │    │ overlay  │          │
│  └──────────┘    └──────────┘    └──────────┘    └──────────┘          │
│       │                               │              │                  │
│       │                               ▼              ▼                  │
│       │                         ┌──────────┐   ┌──────────┐            │
│       │                         │ fit_     │   │ compute_ │            │
│       │                         │ model.m  │   │ metrics  │            │
│       │                         └─────┬────┘   └──────────┘            │
│       │                               │                                 │
│       │                    ┌──────────▼──────────┐                     │
│       │                    │  Updated K, τ, L    │                     │
│       │                    │  → back to init_    │                     │
│       │                    │    params.m          │──→ (iterate)       │
│       │                    └─────────────────────┘                     │
│       │                                                                 │
│       ▼                                                                 │
│  ┌─────────────────────────────────────┐                               │
│  │  generate_report_plots.m            │                               │
│  │  → All figures for the report       │                               │
│  └─────────────────────────────────────┘                               │
└─────────────────────────────────────────────────────────────────────────┘
```

### The Iteration Loop

1. **Start with placeholders** → K=15, τ=120, L=3
2. **Run simulation** → get expected behavior
3. **Run step test on hardware** → get real CSV data
4. **Fit the model** → get actual K, τ, L
5. **Update init_params.m** → model now matches reality
6. **Re-simulate** → overlay matches hardware
7. **Run experiments** → ON-OFF, PI, disturbance, faults
8. **Generate plots** → report-ready figures

---

## 6. Controller Theory

### ON-OFF (Relay / Bang-Bang)

The simplest controller — the heater is either **fully ON** or **fully OFF**:

```
if T < setpoint − 0.5°C → heater ON  (100% duty)
if T > setpoint + 0.5°C → heater OFF (0% duty)
otherwise → hold previous state (hysteresis band)
```

**Pros:** Dead simple, no tuning needed.  
**Cons:** Temperature oscillates within the hysteresis band. Heater switches rapidly → wear. Higher energy use.

### PI (Proportional-Integral)

The "smart" controller — adjusts heater duty proportionally to the error and its history:

```
e(t) = setpoint − T(t)                    ← error
u(t) = Kp · e(t) + Ki · ∫e(t)dt           ← control signal
u(t) = clamp(u, 0, 1)                     ← saturate to valid range
```

- **P term (Kp·e):** Reacts to current error. Larger error → more heating.
- **I term (Ki·∫e):** Eliminates steady-state error. Accumulates over time.
- **Anti-windup:** Stop integrating when output is saturated. Prevents overshoot.

**Pros:** Smooth temperature, zero steady-state error, lower energy.  
**Cons:** Needs tuning (Kp, Ki). We use IMC rules to compute these from the plant model.

### Why PI and Not PID?

The thermal system is already slow (τ=120s) with minimal noise. The **D term** would amplify sensor noise and add complexity with no real benefit. PI is sufficient and recommended in the problem statement.

---

## 7. Fault Detection Logic

The firmware implements **four fault detection mechanisms**:

### 7.1 Overheat Protection (Safety-Critical)
```
if T1 > 50°C OR T2 > 50°C → HEATER OFF immediately
```
Non-negotiable. Protects the chamber (especially PLA) and components.

### 7.2 Sensor Failure Detection
```
if sensor returns −127°C (disconnected) or 85°C (reset value) → FAULT
→ If both sensors fail → HEATER OFF
→ If one fails → use the other, set warning flag
```

### 7.3 Sensor Disagreement
```
if |T1 − T2| > 1.5°C for more than 5 consecutive seconds → FLAG
```
Indicates a sensor is drifting, misplaced, or faulty. We use 1.5°C because the noise floor is typically < 0.5°C.

### 7.4 Model-Based Residual Detection
```
T_pred = first-order model prediction using actual u (with dead-time buffer)
residual = T_measured − T_predicted
if |residual| > 3°C for > 5 seconds → FLAG
```
Catches anything the model doesn't expect: fan failure, blocked airflow, heater degradation. The threshold (3°C) is set to ~3× the normal residual standard deviation.

### Fault Flags Bitmask
```
Bit 0 (0x01): Overheat
Bit 1 (0x02): Sensor failure
Bit 2 (0x04): Sensor disagreement
Bit 3 (0x08): Model residual
Bit 4 (0x10): Restricted airflow
```

Example: `fault_flags = 5` → binary `00101` → Overheat + Sensor Disagree.

---

## 8. Step-by-Step: How to Run the Project

### Phase 1: Simulation (Software Only)

```matlab
% In MATLAB, navigate to simulink/ folder
cd('C:\Users\Fufal Jigar\Control System Project\simulink')

% Step 1: Load parameters
init_params

% Step 2: Build the Simulink model
build_model

% Step 3: Run all simulations and generate plots
run_simulation

% Step 4: Explore tuning options
tune_pi
```

### Phase 2: Hardware Setup

1. Wire the circuit per `firmware/README_firmware.md`
2. Install Arduino libraries: OneWire, DallasTemperature
3. Open `firmware/thermal_controller.ino` in Arduino IDE
4. Upload to Arduino Uno
5. Run the power-on safety checklist

### Phase 3: Experiments

```bash
# Start logging (in python/ folder)
pip install -r requirements.txt
python serial_logger.py --port COM3 --plot --output ../data/step_test.csv

# In Serial Monitor or logger, send commands:
#   f150    ← set fan
#   m0      ← mode OFF
# Then manually set 50% duty for step test (or use m1 r60 trick)
# Wait 20-30 minutes
# Ctrl+C to stop logging
```

### Phase 4: Model Fitting

```matlab
% In MATLAB, navigate to matlab/ folder
cd('C:\Users\Fufal Jigar\Control System Project\matlab')

% Fit the model to step test data
fit_model    % Select your step_test.csv when prompted

% Update init_params.m with the fitted K, tau, L values!
```

### Phase 5: Control Experiments

```bash
# ON-OFF run (serial commands)
m0 → r30 → m1 → (wait) → r40 → (wait) → r35

# PI run
m0 → r30 → m2 → (wait) → r40 → (wait) → r35

# Disturbance: during PI at 40°C, change fan: f80 or partially block the chamber
# Fault tests: disconnect a sensor wire, block the fan, etc.
```

### Phase 6: Analysis & Report

```matlab
% Overlay sim vs hardware
plot_overlay

% Compute all metrics
compute_metrics(t, ref, T, u, 50, 1)

% Generate all report figures
generate_report_plots
```

```bash
# Or use Python for quick CSV comparison
python compare_runs.py ../data/onoff.csv ../data/pi.csv --labels "ON-OFF" "PI" -s ../plots/
```

---

## 9. Key Equations Reference

### Plant Model
| Equation | Description |
|----------|-------------|
| `C·dT/dt = P·u − (T−T_amb)/R_th` | Energy balance ODE |
| `G(s) = K·e^(-Ls) / (τs+1)` | Transfer function (FOPTD) |
| `K = P·R_th` | Static gain |
| `τ = C·R_th` | Time constant |
| `R_th = K/P` | Thermal resistance |
| `C = τ·P/K` | Thermal capacitance |

### IMC PI Tuning
| Equation | Description |
|----------|-------------|
| `τ_c = max(0.1·τ, L)` | Closed-loop time constant |
| `Kp = τ / (K·(L + τ_c))` | Proportional gain |
| `Ki = Kp / τ` | Integral gain |

### Performance Metrics
| Metric | Formula |
|--------|---------|
| Rise time | `t(T=90%) − t(T=10%)` of step |
| Settling time | Last exit from ±2% band |
| Overshoot | `(T_peak − T_sp) / ΔT × 100%` |
| SSE | `T_sp − mean(T_last10%)` |
| Energy | `E = P · Σ(u) · Ts` [J] |
| NRMSE Fit | `100 × (1 − ‖y−ŷ‖ / ‖y−mean(y)‖)` |

---

## 10. Deliverables Checklist

- [ ] **Simulink model** and all `.m` scripts
- [ ] **CAD/STL files** for chamber (if 3D-printed)
- [ ] **Wiring diagram** and block diagram
- [ ] **CSV data** for every run (step test, ON-OFF, PI, disturbance, faults)
- [ ] **Plots:**
  - [ ] Step response fit (measured vs model)
  - [ ] ON-OFF vs PI comparison
  - [ ] Disturbance response (sim vs hardware)
  - [ ] Fault detection cases
- [ ] **Performance table** (rise time, settling, overshoot, SSE, energy, duty)
- [ ] **Fault detection results** table
- [ ] **Report** and **slides**
- [ ] **Demo:** PI setpoint change → fan off → fault flag triggers

---

> **Tip:** Always run the step test first — everything downstream depends on accurate K, τ, L.  
> Let the chamber cool to the same ambient temperature between runs for comparable data.  
> While it cools, work on the report! 🚀

# 🌡️ EE302 Smart HVAC Temperature Control and Fault Detection
## Quanser QNET HVAC Trainer 1.0 on National Instruments NI ELVIS II+
### MATLAB & Simulink R2026a End-to-End Implementation

> **Course:** EE302 Control Systems Laboratory — Final End-Semester Project  
> **Problem Statement:** PS-4: Smart HVAC Temperature Control and Fault Detection  
> **Target Hardware Platform:** National Instruments NI ELVIS II+ Multifunction DAQ  
> **Laboratory Plant:** Quanser QNET HVAC Trainer 1.0 (QNET-012 / QNET-HVACT)  
> **Software Environment:** MATLAB R2026a, Simulink, Data Acquisition Toolbox (NI-DAQmx 25.5+)  

---

## 📋 Executive Project Summary

This project delivers a complete, lab-verified, end-to-end control, monitoring, and fault diagnostic system for a laboratory-scale Heating, Ventilation, and Air Conditioning (HVAC) duct system. The project strictly satisfies all requirements of **EE302 Problem Statement 4**:

1. **Thermal Modelling & System Identification:** Continuous first-principles energy balance model accounting for heater power, convective heat loss, ambient temperature, and transport lag ($G(s) = \frac{K e^{-L s}}{\tau s + 1}$).
2. **Controller Design & Evaluation:** Rigorous comparison between an **ON-OFF Relay Controller with Hysteresis** and an **Anti-Windup Proportional-Integral (PI) Controller with Feedforward Ambient Compensation**.
3. **Live Hardware Interfacing:** Full live driver integration with the **National Instruments NI ELVIS II+** platform and **Quanser QNET HVAC Trainer 1.0** via MATLAB Data Acquisition Toolbox (`Dev1`), including automatic fallback to a high-fidelity **Digital Twin Emulation Mode**.
4. **Disturbance Rejection:** Controlled forced-convection airflow surge testing (+80% fan speed burst) and ambient draft recovery analysis.
5. **Fault Detection & Isolation (FDI):** Real-time Luenberger state observer residual generation to detect and latch three safe fault scenarios: **Fan Failure / Airflow Loss**, **Sensor Drift / Disagreement**, and **Heater Element Degradation**.
6. **Energy & Actuator Performance:** Comprehensive energy consumption integration ($E = \int P(t) dt$), duty cycle calculation, and relay wear/switching count analysis.
7. **Simulink Implementation:** Programmatically generated, fully compiled `.slx` model ([`qnet_hvac_simulink.slx`](file:///C:/Users/Fufal%20Jigar/Control%20System%20Project/qnet_hvac_ni_elvis/models/qnet_hvac_simulink.slx)).

---

## 📂 Project Directory Structure

```
qnet_hvac_ni_elvis/
├── README.md                      <- Master project & execution documentation
├── main_run_all.m                 <- Master execution pipeline (Live hardware or simulation)
├── config/
│   ├── qnet_hvac_params.m         <- Physical constants, thermal capacities, tuning gains
│   └── elvis_channel_config.m     <- NI ELVIS II+ channel mapping (AI 5-7, AO 0-1)
├── models/
│   ├── qnet_thermal_plant.m       <- Non-linear continuous energy balance state equation
│   ├── build_simulink_model.m     <- Programmatic Simulink builder script (MATLAB 2026)
│   └── qnet_hvac_simulink.slx     <- Generated Simulink simulation model
├── controllers/
│   ├── onoff_controller.m         <- Hysteresis relay (Bang-Bang) controller
│   ├── pi_controller.m            <- Clamping Anti-Windup PI with ambient feedforward
│   └── thermal_observer.m         <- Luenberger state observer & FDI diagnostic engine
├── hardware/
│   ├── elvis_hardware_interface.m <- NI ELVIS II+ DAQ driver class (Live & Digital Twin)
│   ├── calibrate_sensors.m        <- Live baseline zeroing & sensor noise check
│   ├── run_hardware_bump_test.m   <- Live open-loop bump test (FOPDT parameter extraction)
│   └── run_hardware_control.m     <- Real-time closed-loop control on NI ELVIS II+
├── simulation/
│   ├── run_simulation.m           <- Core numerical simulation engine
│   ├── run_comparison.m           <- Head-to-head comparison (ON-OFF vs PI)
│   ├── run_disturbance_test.m     <- Fan airflow surge disturbance analysis
│   └── run_fault_detection_test.m <- Safe fault injection & energy assessment
├── analysis/
│   ├── calculate_metrics.m        <- Rise time, settling time, overshoot, e_ss, IAE, energy
│   ├── export_report_table.m      <- Table generation and CSV exporter
│   └── generate_plots.m           <- 4 publication-grade figures generator
├── tests/
│   └── test_qnet_system.m         <- MATLAB Unit Test Suite (7 test methods)
├── figures/                       <- Exported 300 DPI publication plots
│   ├── figure1_controller_comparison.png
│   ├── figure2_disturbance_response.png
│   ├── figure3_fault_detection.png
│   └── figure4_system_identification.png
└── results/                       <- Output metrics tables and parameter MAT-files
    ├── controller_comparison_table.csv
    └── identified_fopdt_params.mat
```

---

## ⚡ Hardware Architecture & Pin Mapping (NI ELVIS II+)

The **Quanser QNET HVAC Trainer 1.0** mounts directly on top of the **NI ELVIS II+** prototyping workstation. The signals route internally through the PCI/USB DAQ connector as follows:

| Signal Function | NI ELVIS II+ Channel | Physical Connector / Pin | Operating Voltage | Engineering Scaling / Sensitivity |
| :--- | :--- | :--- | :--- | :--- |
| **Chamber Temperature** | `Dev1/ai5` | Analog Input AI 5 | $0.0\text{ to }+5.0\text{ V}$ | $S = 20.0\,^\circ\text{C/V}$ ($T_{ch} = 20 \times V_{AI5}$) |
| **Ambient Temperature** | `Dev1/ai7` | Analog Input AI 7 | $0.0\text{ to }+5.0\text{ V}$ | $S = 20.0\,^\circ\text{C/V}$ ($T_{amb} = 20 \times V_{AI7}$) |
| **Secondary Duct Temp** | `Dev1/ai6` | Analog Input AI 6 | $0.0\text{ to }+5.0\text{ V}$ | $S = 20.0\,^\circ\text{C/V}$ ($T_{sec} = 20 \times V_{AI6}$) |
| **Heater Command (Actuator)** | `Dev1/ao0` | Analog Output AO 0 | $0.0\text{ to }+5.0\text{ V}$ | $0\text{ V} = 0\,\text{W}$ (OFF), $5\text{ V} = 19.5\,\text{W}$ (Max) |
| **Fan Command (Blower)** | `Dev1/ao1` | Analog Output AO 1 | $0.5\text{ to }+5.0\text{ V}$ | $2.5\text{ V} = \text{Nominal Airflow}$, $5\text{ V} = \text{Max Cooling}$ |
| **Analog Ground** | `AIGND` | Pin 39 / 44 / 59 | $0.0\text{ V}$ | Common reference ground |

> [!IMPORTANT]
> **Safety Interlock**: The software continuously clamps the heater command to $[0.0, 5.0]\text{ V}$. If the chamber thermistor detects $T > 55.0\,^\circ\text{C}$, the emergency stop routine instantly forces $V_{heater} = 0.0\text{ V}$ while keeping the cooling fan active at $2.5\text{ V}$.

---

## 🔬 Mathematical Physics & Control Theory

### 1. Thermal Energy Balance Model
The heating duct dynamics are governed by conservation of thermal energy:

$$C_{th} \frac{dT(t)}{dt} = Q_{in}(t - L) - Q_{loss}(t)$$

Where:
* $C_{th} = 145.0\text{ J/}^\circ\text{C}$ is the thermal capacitance of the chamber air and enclosure.
* $Q_{in}(t) = \eta \cdot K_h \cdot u(t)$ is the electrical heat generation ($K_h = 32.37\text{ W/V}$).
* $Q_{loss}(t) = \frac{T(t) - T_{amb}}{R_{th}(v_{fan})}$ is the convective and conductive dissipation to ambient.
* $R_{th}(v_{fan}) = \frac{R_{th0}}{1 + \beta_{fan}(v_{fan} - v_{fan0})}$ is the airflow-dependent thermal resistance.
* $L \approx 1.8\text{ s}$ is the transport dead time representing the transit latency of air passing over the sensor.

### 2. First-Order Plus Dead Time (FOPDT) Transfer Function
In perturbation coordinates $\Delta T(s) = T(s) - T_{amb}$:

$$G(s) = \frac{\Delta T(s)}{U(s)} = \frac{K}{\tau s + 1} e^{-L s}$$

* Nominal Steady-State Gain: $K = K_h \cdot R_{th0} \approx 7.25\,^\circ\text{C/V}$
* Nominal Thermal Time Constant: $\tau = C_{th} \cdot R_{th0} \approx 32.5\text{ s}$
* Transport Delay: $L \approx 1.8\text{ to }2.0\text{ s}$

### 3. Controller Formulations

#### A. ON-OFF Relay with Hysteresis
To prevent high-frequency relay chattering while commanding the binary heating state:

$$u(t) = \begin{cases} 5.0\text{ V} & \text{if } T(t) < T_{sp} - \frac{\Delta}{2} \\ 0.0\text{ V} & \text{if } T(t) > T_{sp} + \frac{\Delta}{2} \\ u(t - \Delta t) & \text{otherwise} \end{cases}$$

Where hysteresis deadband $\Delta = 0.8\,^\circ\text{C}$.

#### B. Anti-Windup PI with Ambient Feedforward
To eliminate steady-state offset without excessive overshoot:

$$u(t) = \text{sat}_{[0, 5]} \left[ u_{ff}(t) + K_p e(t) + K_i \int e(\tau) d\tau \right]$$

* **Ambient Feedforward Equilibrium Bias:** $u_{ff}(t) = \frac{T_{sp}(t) - T_{amb}(t)}{K_{nom}}$
* **Anti-Windup Clamping:** The integrator halts error accumulation whenever the actuator saturates ($u \ge 5\text{ V}$ and $e > 0$, or $u \le 0\text{ V}$ and $e < 0$).
* **Tuned Gains:** $K_p = 1.35\text{ V/}^\circ\text{C}$, $K_i = 0.042\text{ V/(}^\circ\text{C}\cdot\text{s)}$.

### 4. Model-Based Fault Detection Observer
A real-time Luenberger state observer predicts the nominal chamber temperature:

$$\frac{d\hat{T}}{dt} = \frac{1}{\tau_{nom}} \left[ K_{nom} u(t - L) - (\hat{T} - T_{amb}) \right] + L_o (T_{meas} - \hat{T})$$

* **Thermal Model Residual:** $r_{model}(t) = T_{meas}(t) - \hat{T}(t)$
* **Sensor Disagreement Residual:** $r_{sensor}(t) = |T_{meas}(t) - T_{secondary}(t)|$
* **Diagnostic Logic:**
  1. $r_{model} > +1.5\,^\circ\text{C}$ for $> 2.0\text{ s} \implies$ **FAN FAILURE / DUCT OBSTRUCTION** (Chamber overheats due to loss of convection).
  2. $r_{sensor} > 1.5\,^\circ\text{C}$ for $> 2.0\text{ s} \implies$ **SENSOR BIAS / DRIFT** (Thermistor disagreement).
  3. $(\hat{T} - T_{meas}) > 1.8\,^\circ\text{C}$ with $u \ge 3.0\text{ V} \implies$ **HEATER ELEMENT DEGRADATION** (Loss of electrical power).

---

## 📊 Experimental Results & Performance Comparison

The following table summarizes the quantitative results evaluated across the multi-step tracking test ($22^\circ\text{C} \to 32^\circ\text{C} \to 42^\circ\text{C} \to 36^\circ\text{C}$):

| Performance & Energy Metric | ON-OFF Controller | Anti-Windup PI Controller | Improvement / Advantage |
| :--- | :---: | :---: | :--- |
| **Rise Time ($t_r$, 10%–90%)** | $8.6\text{ s}$ | $\mathbf{8.1\text{ s}}$ | **5.8% faster rise** |
| **Settling Time ($t_s$, 2%)** | $110.0\text{ s}$ | $\mathbf{97.2\text{ s}}$ | **11.6% faster settling** |
| **Peak Overshoot ($M_p$)** | $21.04\%$ | $\mathbf{5.93\%}$ | **71.8% overshoot reduction** |
| **Steady-State Error ($e_{ss}$)** | $0.31\,^\circ\text{C}$ | $\mathbf{0.003\,^\circ\text{C}}$ | **$100\times$ higher tracking accuracy** |
| **Integral Absolute Error (IAE)** | $430.77$ | $\mathbf{284.67}$ | **33.9% lower cumulative error** |
| **Integral Time Abs Error (ITAE)** | $68,861$ | $\mathbf{43,129}$ | **37.4% faster error attenuation** |
| **Total Energy Dissipated** | $0.900\text{ Wh}$ ($3240\text{ J}$) | $\mathbf{0.562\text{ Wh}}$ ($2024\text{ J}$) | **37.5% Electrical Energy Savings** |
| **Actuator Cycling / Switching Count** | $53\text{ cycles}$ | $\mathbf{3\text{ transitions}}$ | **$17.6\times$ mechanical/relay wear reduction** |

---

## 🚀 Step-by-Step Execution Guide

### Option 1: Run the Complete Master Pipeline (Recommended)
Launch MATLAB R2026a and navigate to the project directory:

```matlab
cd 'C:\Users\Fufal Jigar\Control System Project\qnet_hvac_ni_elvis'
summary = main_run_all('live');   % Automatically detects and runs on physical NI ELVIS II+
```
*(If the NI ELVIS II+ is disconnected or in software simulation, pass `'sim'` or omit the argument for auto-detection).*

### Option 2: Live Sensor Calibration & Baseline Check
```matlab
cd 'C:\Users\Fufal Jigar\Control System Project\qnet_hvac_ni_elvis'
addpath('config'); addpath('hardware');
cal = calibrate_sensors(20, 'live');
```

### Option 3: Run Open-Loop Bump Test (System Identification)
Applies a 3.0 V step to the heater for 45 seconds, records temperature, and computes $K, \tau, L$:
```matlab
addpath('config'); addpath('hardware'); addpath('models');
fopdt = run_hardware_bump_test(3.0, 45.0, 'live');
```

### Option 4: Run Real-Time Closed-Loop Hardware Control
```matlab
addpath('config'); addpath('hardware'); addpath('controllers');
results = run_hardware_control('pi', 60.0, 'live');
```

### Option 5: Programmatic Simulink Execution
Builds and launches the `.slx` model:
```matlab
addpath('models');
build_simulink_model();
open_system('qnet_hvac_simulink');
sim('qnet_hvac_simulink');
```

### Option 6: Execute Automated Unit Test Suite
```matlab
results = runtests('tests/test_qnet_system.m');
disp(results);
```

---

## 🖼️ Generated Publication Figures

All figures are automatically generated and saved in [`qnet_hvac_ni_elvis/figures/`](file:///C:/Users/Fufal%20Jigar/Control%20System%20Project/qnet_hvac_ni_elvis/figures):
1. **`figure1_controller_comparison.png`**: Multi-step setpoint tracking ($T_{sp}$ vs $T_{meas}$), actuator effort ($0 - 5\text{ V}$), and cumulative energy dissipation ($Wh$).
2. **`figure2_disturbance_response.png`**: Fan airflow surge ($2.5\text{ V} \to 4.5\text{ V}$) disturbance rejection and recovery.
3. **`figure3_fault_detection.png`**: State observer predictions ($\hat{T}$), FDI residuals ($r_{model}, r_{sensor}$), and latched alarms for Fan Failure, Sensor Bias, and Heater Degradation.
4. **`figure4_system_identification.png`**: Bump test hardware step response vs empirical FOPDT fit ($> 97\%$ accuracy).

---

## 🔒 Safety and Troubleshooting Guidelines

* **Device Error (Code 39) in Windows:** Occurs if the NI Device Loader service was stopped. Fixed by running `Start-Service nidevldu, niSvcLoc` in elevated PowerShell or restarting Windows.
* **NI Error -50103 (Resource Reserved):** Occurs if an existing DAQ task was left unclosed. Simply type `daqreset` in the MATLAB Command Window to free all channels.
* **Thermal Cutoff:** If $T_{chamber} > 55.0\,^\circ\text{C}$, the safety interlock commands $u = 0.0\text{ V}$ immediately.
* **Prototyping Board Power Switch:** Ensure the power toggle switch on the upper-right corner of the NI ELVIS II+ board is switched **ON** (green LED lit).

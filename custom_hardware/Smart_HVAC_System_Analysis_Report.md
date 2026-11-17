# Comprehensive Technical Analysis: Smart HVAC Temperature Control & Fault Detection System

**Course & Project:** EE302 – Control Systems Laboratory | Problem Statement 4 (Group 7)  
**Institution:** School of Computing and Electrical Engineering, Indian Institute of Technology Mandi  
**Authors / Project Owners:** Rathod Dhruv Shailesh (B24242), Dumpa Mohan Sai (B24243), Fufal Jigar Dineshbhai (B24244), Geetanjali Saraswat (B24245)  
**Instructor:** Dr. Yanumula Venkata Karteek  
**Analyzed Repository:** `Smart_HVAC_System`  
**Reference Document:** [ControlFinalProject.pdf](file:///C:/Users/Fufal%20Jigar/Downloads/New%20folder/Smart_HVAC_System/ControlFinalProject.pdf)

---

## 1. Executive Summary & Scope Clarification

### 1.1 Context & Scope Alignment
The reference document, `ControlFinalProject.pdf`, presents a dual-platform study covering:
1. **Platform A (Custom 3D-Printed Laboratory Prototype):** An Arduino Mega2560-driven custom insulated cylindrical thermal chamber with discrete active-LOW relays, resistive heating coil, axial cooling fan, and an LM35 temperature sensor.
2. **Platform B (Commercial Continuous Benchmark):** A Quanser QNET HVAC Trainer 1.0 mounted on a National Instruments NI ELVIS II+ DAQ platform.

> [!IMPORTANT]
> **Repository Scope Boundary:**  
> The physical files, MATLAB scripts, experimental logs, telemetry datasets, and figure assets present in this repository (`Smart_HVAC_System`) correspond **exclusively to Platform A (the Custom 3D-Printed Chamber Prototype)**. Platform B is covered in the attached report as an external comparative benchmark, but its hardware models and raw datasets are not part of this project directory.  
> As instructed, this analysis focuses strictly on the components, code, experimental runs, transfer function analyses, and diagnostics that **depict the physical project implemented in this repository (Platform A)**.

### 1.2 Summary of Findings
The project represents a complete, highly disciplined hardware-in-the-loop (HIL) control engineering workflow:
- **Custom Hardware Fabricated:** 3D-printed insulated cylindrical duct with a removable curved hatch for physical disturbance testing, axial forced-air fan, and slotted exhaust baffle.
- **Dual Actuation & Electrical Characterization:** 14.00 W resistive heater (8.0 V, 1.75 A) and 0.51 W axial DC fan (3.0 V, 0.17 A) controlled via active-LOW opto-isolated relays.
- **Discrete ON/OFF Hysteresis Regulation:** Evaluated across predictive and centered configurations at 25.0°C and 30.0°C. Proved that improper threshold centering induces severe actuator fighting (33.1% energy penalty, 4.301°C limit cycles).
- **Ziegler–Nichols Relay Auto-Tuning:** In-situ Åström–Hägglund relay oscillation experiment at 35.0°C yielding $K_u = 0.9817$, $P_u = 53.50\text{ s}$, and synthesizing tuned PI gains ($K_p = 0.44178$, $K_i = 0.009909\text{ s}^{-1}$).
- **Discrete Anti-Windup PI with Supervisory Fan Override:** Integrates deadband relay mapping ($u_{\text{deadband}} = 0.05$), relay contact dwell protection ($t_{\text{dwell}} = 2.0\text{ s}$), and anti-windup clamping ($I \in [-0.5, +0.5]$). Achieved sub-0.2°C steady-state regulation.
- **Physical Thermal Saturation & Parameter Identification:** Identified an asymptotic physical ceiling of $39.20^\circ\text{C}$ at 100% duty cycle, calculating lumped thermal resistance $R_{\text{th},A} = 1.48^\circ\text{C/W}$ and capacitance $C_{\text{th},A} = 94.1\text{ J/}^\circ\text{C}$.
- **Structural Disturbance Rejection:** Demonstrated automated integral adaptation during permanent insulation hatch removal at $t \approx 390\text{ s}$, shifting duty cycle from 50.9% (7.13 W) to 71.2% (9.97 W) to maintain 24.93°C.
- **Real-Time Deterministic Fault Diagnostics:** A four-code diagnostic state machine detecting lid removal (Code 1), heater failure (Code 2), fan failure (Code 3), and sensor disconnect/spikes (Code 4), with 0 false alarms nominally and successful safe isolation during fault injection.
- **Frequency-Domain Stability Validation:** Formulated the continuous FOPDT plant $G_A(s) = \frac{16.65}{138.96s + 1} e^{-13.90s}$, proving robust stability with Gain Margin $\text{GM} = 5.56\text{ dB}$ and Phase Margin $\text{PM} = 30.62^\circ$.

---

## 2. Hardware Architecture & Electrical Interface

### 2.1 Physical Chamber Assembly
The chamber was CAD-designed and 3D-printed in polymer to satisfy laboratory enclosure requirements:
- **Axial Forced-Air Inlet Flange:** Houses a 3.0 V brushless DC cooling fan mounted coaxially with the cylindrical duct.
- **Slotted Exhaust Baffle:** Restricts natural air escape, preventing convective heat washout during static heating.
- **Internal Heating Coil & Sensor Hub:** A low-voltage resistive nichrome coil and LM35 precision analog temperature sensor ($10\text{ mV/}^\circ\text{C}$) are suspended centrally with radial clearance from the polymer walls to avoid thermal melting.
- **Removable Curved Insulation Hatch:** A precision-fitted curved middle hatch can be detached manually to inject repeatable physical heat-loss disturbances.

```
+-------------------------------------------------------------------------+
|                      3D-PRINTED THERMAL CHAMBER                         |
|                                                                         |
|  [3.0V DC Fan] ---> [Heater Coil (14W)] ---> [LM35 Sensor] ---> [Baffle] |
|   (Inlet Flange)       (Central Mount)        (A1 Analog)       (Exhaust) |
|                              ^                                          |
|                              |                                          |
|              [Removable Curved Insulation Hatch]                        |
+-------------------------------------------------------------------------+
```

### 2.2 Electrical Circuit & DAQ Mapping
The system interfaces an **Arduino Mega2560** over USB Serial (`COM19` / `COM21`) at 115200 baud to MATLAB:
- **Sensor Input (`A1`):** Analog voltage from LM35 converted to temperature via:
  $$T_{\text{raw}}(t) = 100 \cdot V_{\text{sens}}(t) \quad [^\circ\text{C}]$$
- **Heater Actuator (`D6`):** Active-LOW relay switching an 8.0 V DC power rail across $R_{\text{heater}} = 4.57\ \Omega$:
  $$P_{\text{heater}} = 8.0\text{ V} \times 1.75\text{ A} = 14.00\text{ W}$$
- **Cooling Fan Actuator (`D9`):** Active-LOW relay switching a 3.0 V DC power rail across the fan:
  $$P_{\text{fan}} = 3.0\text{ V} \times 0.17\text{ A} = 0.51\text{ W}$$
- **Energy Accounting Model:**
  $$E_{\text{total}} = P_{\text{heater}} \left(\frac{D_h}{100} t_{\text{run}}\right) + P_{\text{fan}} \left(\frac{D_f}{100} t_{\text{run}}\right) \quad [\text{J}]$$
  $$P_{\text{avg}} = \frac{E_{\text{total}}}{t_{\text{run}}} \quad [\text{W}]$$

---

## 3. Detailed Technical Analysis of Subsystems

```
Smart_HVAC_System/
│
├── OnOffController/                  # Bang-bang hysteresis regulation
│   ├── OnOffController.m             # Master automated script (auto-settle stop)
│   └── Readings/                     # Datasets across 3 setpoint/threshold cases
│
├── PIController/                     # Discrete Anti-Windup PI & FDI
│   ├── PIController.m                # Master unified controller + 4-code fault machine
│   └── Readings/                     # Tuning, tracking, disturbance, and fault runs
│
├── TransferFunctionAnalysis/         # System identification & frequency domain
│   ├── PlayingWithTransferFunction.m # Simulation & analysis scripts
│   └── matlab_workspace_variables... # Exact workspace variables (TF, margins, poles)
│
└── AllImages/                        # High-resolution plots, schematics, and photos
```

---

### 3.1 Subsystem 1: Discrete ON/OFF Hysteresis Controller

Located in [`Smart_HVAC_System/OnOffController/`](file:///C:/Users/Fufal%20Jigar/Downloads/New%20folder/Smart_HVAC_System/OnOffController/).

#### Control Law
Implemented at sampling period $T_s = 0.2\text{ s}$ with a 25-sample (5.0 s) moving-average filter $T_f(t)$ and emergency thermal cutoff at $T_{\text{max}} = 45.0^\circ\text{C}$:
$$u(t) = \begin{cases} 
\text{HEAT (Heater ON, Fan OFF)}, & T_f(t) < \text{SP} - h \\ 
\text{COOL (Heater OFF, Fan ON)}, & T_f(t) > \text{SP} + h \\ 
\text{IDLE (Heater OFF, Fan OFF)}, & |T_f(t) - \text{SP}| \le h 
\end{cases}$$

#### Experimental Case Comparison
The codebase documents three distinct experimental configurations:

| Performance / Energy Metric | Case 1: Predictive ($25^\circ\text{C}$) | Case 2: Centered ($25^\circ\text{C}$) | Case 3: Centered ($30^\circ\text{C}$) |
| :--- | :---: | :---: | :---: |
| **Setpoint ($\text{SP}$)** | $25.00^\circ\text{C}$ | $25.00^\circ\text{C}$ | $30.00^\circ\text{C}$ |
| **Ambient Temperature ($T_{\text{amb}}$)** | $20.19^\circ\text{C}$ | $19.65^\circ\text{C}$ | $19.40^\circ\text{C}$ |
| **Hysteresis Band ($h$)** | $1.00^\circ\text{C}$ | $1.00^\circ\text{C}$ | $1.00^\circ\text{C}$ |
| **Total Duration ($t_{\text{run}}$)** | $263.15\text{ s}$ | $269.33\text{ s}$ | $354.20\text{ s}$ |
| **Rise Time ($t_r$, 10–90%)** | $46.95\text{ s}$ | $67.73\text{ s}$ | $119.00\text{ s}$ |
| **Settling Time ($t_s$)** | $137.96\text{ s}$ | $268.81\text{ s}$ | $158.84\text{ s}$ |
| **Peak Temperature ($T_{\text{peak}}$)** | $25.42^\circ\text{C}$ | $27.86^\circ\text{C}$ | $30.21^\circ\text{C}$ |
| **Overshoot ($\%OS$)** | **$1.66\%$** | **$11.44\%$** | **$0.68\%$** |
| **Steady-State Mean ($T_{ss}$)** | $24.70^\circ\text{C}$ | $25.11^\circ\text{C}$ | $29.91^\circ\text{C}$ |
| **Steady-State Error ($e_{ss}$)** | $-0.298^\circ\text{C}$ | $+0.107^\circ\text{C}$ | $-0.092^\circ\text{C}$ |
| **Steady-State Std. Dev.** | $0.105^\circ\text{C}$ | $1.228^\circ\text{C}$ | $0.097^\circ\text{C}$ |
| **Peak-to-Peak Ripple** | $0.587^\circ\text{C}$ | **$4.301^\circ\text{C}$** | $0.587^\circ\text{C}$ |
| **Heater / Fan / Idle Duty** | $50.81\% / 0.00\% / 49.19\%$ | $65.25\% / 22.81\% / 11.95\%$ | $86.53\% / 0.00\% / 13.47\%$ |
| **Heater / Fan Energy** | $1871.9\text{ J} / 0.0\text{ J}$ | $2460.3\text{ J} / 31.3\text{ J}$ | $4290.8\text{ J} / 0.0\text{ J}$ |
| **Total Energy Dissipated** | **$1871.9\text{ J}$ ($0.520\text{ Wh}$)** | **$2491.7\text{ J}$ ($0.692\text{ Wh}$)** | **$4290.8\text{ J}$ ($1.192\text{ Wh}$)** |
| **Average Power ($P_{\text{avg}}$)** | $7.11\text{ W}$ | $9.25\text{ W}$ | $12.11\text{ W}$ |

#### Critical Engineering Insight
- **Actuator Fighting Penalty:** In Case 2, heating until $\text{SP} + h$ ignored thermal transport lag, pushing temperature to $27.86^\circ\text{C}$. This triggered active fan cooling ($22.81\%$ duty), which over-chilled the chamber, forcing the heater back on. This actuator conflict expanded peak-to-peak ripple to **$4.301^\circ\text{C}$** and increased electrical energy consumption by **$33.1\%$** over Case 1.
- **Natural Convective Damping:** In Case 3 ($\text{SP} = 30.0^\circ\text{C}$), the thermal gradient above ambient ($\Delta T = 10.60^\circ\text{C}$) provided sufficient passive convective cooling, damping overshoot to $0.68\%$ without needing the fan.

---

### 3.2 Subsystem 2: Ziegler–Nichols Relay Auto-Tuning

Located in [`Smart_HVAC_System/PIController/Readings/PITune/`](file:///C:/Users/Fufal%20Jigar/Downloads/New%20folder/Smart_HVAC_System/PIController/Readings/PITune/).

#### Identification Protocol
To synthesize an optimal feedback controller without an empirical model, an Åström–Hägglund relay auto-tuner was deployed in [`PITune.m`](file:///C:/Users/Fufal%20Jigar/Downloads/New%20folder/Smart_HVAC_System/PIController/Readings/PITune/PITune.m):
- Operating point: $\text{SP} = 35.0^\circ\text{C}$.
- Relay hysteresis: $\pm 0.3^\circ\text{C}$ (total $0.6^\circ\text{C}$ band: Heater ON $\le 34.7^\circ\text{C}$, Heater OFF $\ge 35.0^\circ\text{C}$; Fan ON $\ge 35.3^\circ\text{C}$, Fan OFF $\le 35.0^\circ\text{C}$).
- Safety dwell time: $t_{\text{dwell}} = 3.0\text{ s}$ between relay clicks.
- Filter: Exponential Moving Average ($\alpha = 0.9$, $T_s = 0.5\text{ s}$):
  $$T_{\text{filt}}[k] = \alpha T_{\text{filt}}[k-1] + (1-\alpha) T_{\text{raw}}[k]$$
- Describing function formulas ($A = 0.5$):
  $$K_u = \frac{4A}{\pi A_e}, \quad K_p = 0.45 K_u, \quad T_i = \frac{P_u}{1.2}, \quad K_i = \frac{K_p}{T_i}$$

#### Tuning Trials Summary

| Parameter | Symbol / Unit | Trial 1 (`pi_gains_01.mat`) | Trial 2 (`pi_gains_02.mat`) [Selected] |
| :--- | :---: | :---: | :---: |
| **Oscillation Amplitude** | $A_e\ [^\circ\text{C}]$ | $0.6257$ | **$0.6485$** |
| **Ultimate Period** | $P_u\ [\text{s}]$ | $60.5000$ | **$53.5000$** |
| **Ultimate Gain** | $K_u\ [-]$ | $1.0175$ | **$0.9817$** |
| **Proportional Gain** | $K_p\ [-]$ | $0.4579$ | **$0.44178$** |
| **Integral Time** | $T_i\ [\text{s}]$ | $50.4167$ | **$44.5833$** |
| **Integral Gain** | $K_i\ [\text{s}^{-1}]$ | $0.0091$ | **$0.009909$** |

Trial 2 exhibited strict cycle periodicity ($P_u = 53.50\text{ s}$) and was selected as the universal gain set for all closed-loop PI deployments.

---

### 3.3 Subsystem 3: Discrete Anti-Windup PI with Supervisory Fan Override

Located in [`Smart_HVAC_System/PIController/PIController.m`](file:///C:/Users/Fufal%20Jigar/Downloads/New%20folder/Smart_HVAC_System/PIController/PIController.m).

#### Algorithmic Architecture
1. **Sampling & Filtering ($T_s = 1.0\text{ s}$):**
   $$T_{\text{filt}}[k] = 0.8 T_{\text{filt}}[k-1] + 0.2 T_{\text{raw}}[k], \quad e[k] = \text{SP} - T_{\text{filt}}[k]$$
2. **Integral Anti-Windup Clamping:**
   $$I[k] = \text{clamp}\left(I[k-1] + K_i e[k] T_s, -0.5, +0.5\right)$$
   $$u[k] = K_p e[k] + I[k]$$
3. **Actuator Deadband & Switching Dwell:**
   Relay contact chatter is prevented by enforcing $t_{\text{dwell}} \ge 2.0\text{ s}$ between state transitions and a heater engagement deadband $u_{\text{deadband}} = 0.05$.
4. **Supervisory Fan Hysteresis Override:**
   - Active Cooling Latch: If $T_{\text{filt}} \ge \text{SP} + 1.5^\circ\text{C}$, latch `isCooling = true` $\implies$ Fan ON, Heater OFF.
   - Release Latch: When $T_{\text{filt}} \le \text{SP} - 1.5^\circ\text{C}$, latch `isCooling = false`.
   - Normal Heating: If `isCooling == false` and $u[k] > 0.05 \implies$ Heater ON, Fan OFF; else IDLE.

```
                  +-----------------------------------+
                  |   Traw -> EMA Filter (alpha=0.8)  |
                  +-----------------+-----------------+
                                    | Tfilt
                                    v
                     e[k] = Setpoint - Tfilt
                                    |
            +-----------------------+-----------------------+
            |                                               |
            v                                               v
     Proportional Path                               Integral Path
       Kp * e[k]                             I[k] = clamp(I + Ki*e*Ts, +/-0.5)
            |                                               |
            +-----------------------+-----------------------+
                                    |
                                    v
                            u[k] = P + I
                                    |
               +--------------------+--------------------+
               | Supervisory Check:                      |
               | Tfilt >= SP + 1.5 C ?                   |
               +---------+-------------------------------+
                         |
           YES ----------+---------- NO
            |                         |
            v                         v
     [FAN RELAY ON]             u[k] > 0.05 & Dwell >= 2s ?
     (Heater OFF)                     |
                               YES ---+--- NO
                                |           |
                                v           v
                        [HEATER RELAY ON]  [IDLE (ALL OFF)]
```

---

### 3.4 Subsystem 4: Multi-Regime Tracking, High-Setpoint, & Saturation Tests

#### Bidirectional Tracking Performance
Documented in [`PIRunDropTill25C`](file:///C:/Users/Fufal%20Jigar/Downloads/New%20folder/Smart_HVAC_System/PIController/Readings/PIRunDropTill25C/) and [`PIRunRiseTill25C`](file:///C:/Users/Fufal%20Jigar/Downloads/New%20folder/Smart_HVAC_System/PIController/Readings/PIRunRiseTill25C/):

- **Active Cooling Down-Step ($33.72^\circ\text{C} \to 25.00^\circ\text{C}$):** The supervisory fan override engaged immediately, extracting heat rapidly until $26.0^\circ\text{C}$ ($t \approx 31\text{ s}$). The PI controller then seamlessly modulated heater pulses to arrest convective undershoot, settling at $t_s = 219.00\text{ s}$ with **$e_{ss} = -0.068^\circ\text{C}$** and **$0.00\%$ upward overshoot**.
- **Heating Up-Step ($17.94^\circ\text{C} \to 30.00^\circ\text{C}$):** Achieved $t_r = 115.00\text{ s}$, settling at $t_s = 567.00\text{ s}$ with $\%OS = 6.91\%$ ($T_{\text{peak}} = 30.83^\circ\text{C}$, comfortably below the $31.5^\circ\text{C}$ fan trigger) and **$e_{ss} = +0.182^\circ\text{C}$**.

#### High Setpoint ($35.0^\circ\text{C}$) Regulation
Documented in [`PIRun16to35`](file:///C:/Users/Fufal%20Jigar/Downloads/New%20folder/Smart_HVAC_System/PIController/Readings/PIRun16to35/):
- **Warm Start ($T_{\text{init}} = 30.74^\circ\text{C}$):** $t_r = 118.0\text{ s}$, $t_s = 550.0\text{ s}$, $e_{ss} = -0.209^\circ\text{C}$, Duty = $90.73\%$, Energy = $8497.8\text{ J}$.
- **Cold Start 1 ($T_{\text{init}} = 17.60^\circ\text{C}$):** $t_r = 159.0\text{ s}$, $t_s = 795.0\text{ s}$, $e_{ss} = -0.105^\circ\text{C}$, Duty = $92.01\%$, Energy = $11773.6\text{ J}$.
- **Cold Start 2 ($T_{\text{init}} = 16.52^\circ\text{C}$):** $t_r = 417.0\text{ s}$, $t_s = 787.0\text{ s}$, $e_{ss} = -0.138^\circ\text{C}$, Duty = $100.0\%$, Energy = $12684.0\text{ J}$. Rejected an external ambient thermal dip at $t \approx 700\text{ s}$.

#### Physical Thermal Ceiling & Lumped Parameter Identification
Documented in [`MaxTemp39`](file:///C:/Users/Fufal%20Jigar/Downloads/New%20folder/Smart_HVAC_System/PIController/Readings/MaxTemp39/):
- Setpoint commanded: $\text{SP} = 40.0^\circ\text{C}$ from $T_{\text{amb}} = 18.52^\circ\text{C}$.
- The heater saturated continuously at $100\%$ duty cycle ($14.00\text{ W}$ for $2005.0\text{ s}$, consuming $28070.0\text{ J}$).
- The chamber temperature reached an asymptotic steady-state physical equilibrium at:
  $$T_{\text{eq}} = 39.20 \pm 0.31^\circ\text{C} \implies \Delta T_{\text{eq}} = 20.68^\circ\text{C}$$
- Lumped thermal parameter extraction:
  $$R_{\text{th},A} = \frac{\Delta T_{\text{eq}}}{P_{\text{heater}}} = \frac{20.68^\circ\text{C}}{14.00\text{ W}} = 1.48^\circ\text{C/W}$$
  $$G_{\text{th},A} = \frac{1}{R_{\text{th},A}} = 0.677\text{ W/}^\circ\text{C}$$
  Combining with plant time constant $\tau_A = 138.96\text{ s}$:
  $$C_{\text{th},A} = \frac{\tau_A}{R_{\text{th},A}} = \frac{138.96\text{ s}}{1.48^\circ\text{C/W}} = 94.1\text{ J/}^\circ\text{C}$$

---

### 3.5 Subsystem 5: Disturbance Rejection & Real-Time Fault Diagnostics

#### Structural Disturbance: Permanent Insulation Hatch Removal
Documented in [`FAULTBASEDREADINGS/25C_Permenant_FaultReturning_CoverRemoval/`](file:///C:/Users/Fufal%20Jigar/Downloads/New%20folder/Smart_HVAC_System/PIController/Readings/FAULTBASEDREADINGS/25C_Permenant_FaultReturning_CoverRemoval/):
- At $t \approx 390\text{ s}$ while regulating at $\text{SP} = 25.0^\circ\text{C}$ ($T_{\text{amb}} = 18.13^\circ\text{C}$), the curved 3D-printed insulation hatch was permanently detached.
- The temperature dipped to a transient minimum of $23.96^\circ\text{C}$.
- The integral term automatically adapted, increasing the heater duty cycle by **$+39.8\%$** (from $50.9\%$ [$7.13\text{ W}$] to $71.2\%$ [$9.97\text{ W}$]).
- The chamber re-stabilized at **$24.93^\circ\text{C}$**, maintaining the setpoint within $\pm 0.6^\circ\text{C}$ despite the permanent structural compromise.

| Operating Phase | Time Window | Mean Temp | Temp Range | Heater Duty | Average Power / Energy |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Nominal Pre-Fault** | $100 - 390\text{ s}$ | $25.09^\circ\text{C}$ | $24.45 - 26.11^\circ\text{C}$ | $50.9\%$ | $7.13\text{ W}$ ($2066.5\text{ J}$) |
| **Disturbance Transient** | $390 - 650\text{ s}$ | $25.04^\circ\text{C}$ | $23.96 - 25.91^\circ\text{C}$ | $56.5\%$ | $7.91\text{ W}$ ($2056.6\text{ J}$) |
| **Re-Stabilized Regime** | $650 - 820\text{ s}$ | $24.93^\circ\text{C}$ | $24.46 - 25.37^\circ\text{C}$ | **$71.2\%$** | **$9.97\text{ W}$ ($1694.6\text{ J}$)** |
| **Overall Run** | $0 - 869\text{ s}$ | $24.80^\circ\text{C}$ | $18.22 - 26.11^\circ\text{C}$ | $60.8\%$ | $8.51\text{ W}$ ($7396.9\text{ J}$) |

#### Deterministic Four-Code Fault Diagnostic Engine
Integrated in [`PIController.m`](file:///C:/Users/Fufal%20Jigar/Downloads/New%20folder/Smart_HVAC_System/PIController/PIController.m) (active after $t > 30\text{ s}$ startup lockout):

1. **Code 1 (LID OPENED):**
   $$\frac{dT}{dt} < -0.5^\circ\text{C/s} \quad \text{over a 10-sample window}$$
2. **Code 2 (HEATER FAILURE):**
   $$\text{Heater ON streak} > 15\text{ s} \quad \wedge \quad \Delta T_{15s} < 0.3^\circ\text{C} \quad \wedge \quad T_{\text{filt}} < \text{SP} - 1.0^\circ\text{C}$$
3. **Code 3 (FAN FAILURE):**
   $$\text{Fan ON streak} > 15\text{ s} \quad \wedge \quad \frac{dT}{dt} \ge -0.02^\circ\text{C/s} \quad \wedge \quad T_{\text{filt}} > \text{SP} + 1.0^\circ\text{C}$$
4. **Code 4 (SENSOR DISCONNECT / SPIKE):**
   $$T_{\text{raw}} \notin [10^\circ\text{C}, 60^\circ\text{C}] \quad \vee \quad |T_{\text{raw}} - T_{\text{filt}}| > 5.0^\circ\text{C}$$

#### Experimental Fault Validation
- **Nominal Validation at $\text{SP} = 28.0^\circ\text{C}$ ([`PI28CwithFinalCode`](file:///C:/Users/Fufal%20Jigar/Downloads/New%20folder/Smart_HVAC_System/PIController/Readings/PI28CwithFinalCode/)):**
  Zero false alarms ($N_{\text{fault}} = 0$). $t_r = 87.0\text{ s}$, $t_s = 247.0\text{ s}$, $\%OS = 9.31\%$, $T_{ss} = 27.98^\circ\text{C}$, $e_{ss} = +0.019^\circ\text{C}$, Duty = $69.60\%$, Energy = $5320.0\text{ J}$.
- **Sequential Fault Injection at $\text{SP} = 32.0^\circ\text{C}$ ([`PI32CwithFinalCode_simulationShowingFault`](file:///C:/Users/Fufal%20Jigar/Downloads/New%20folder/Smart_HVAC_System/PIController/Readings/PI32CwithFinalCode_simulationShowingFault/)):**
  1. *Heater Power Disconnection ($t = 85 - 123\text{ s}$):* Flagged **Code 2** for 75 cumulative seconds.
  2. *Sensor Wire Disconnection ($t = 166 - 190\text{ s}$):* Voltage dropped to rail/disconnected, instantly triggering **Code 4** (24 s total), which engaged the cooling fan for 22 s as a hardware safety precaution.
  3. *Reconnection EMA Filter Tail ($t = 190 - 200\text{ s}$):* As the raw sensor reconnected, the filtered value decayed rapidly ($dT/dt < -0.5^\circ\text{C/s}$), momentarily triggering **Code 1** (10 s total).
  4. *Closed-Loop Recovery:* Integral anti-windup clamping ($[-0.5, +0.5]$) prevented the controller from saturating during the fault episodes, allowing the system to achieve full re-stabilization at $t_s = 525.00\text{ s}$ with **$e_{ss} = -0.133^\circ\text{C}$** ($T_{ss} = 32.13^\circ\text{C}$).

---

### 3.6 Subsystem 6: FOPDT Modeling & Frequency-Domain Stability Analysis

Located in [`Smart_HVAC_System/TransferFunctionAnalysis/`](file:///C:/Users/Fufal%20Jigar/Downloads/New%20folder/Smart_HVAC_System/TransferFunctionAnalysis/).

#### Plant Transfer Function Synthesis
From the ultimate oscillation parameters ($K_u = 0.9817$, $P_u = 53.5005\text{ s} \implies \omega_u = \frac{2\pi}{P_u} = 0.11744\text{ rad/s}$):
Assuming the typical thermal transport ratio $\tau_A / L_A = 10$, the frequency-domain phase criterion at the ultimate frequency is:
$$\angle G_A(j\omega_u) = -\arctan(\omega_u \tau_A) - \omega_u L_A = -\pi$$
Substituting $x = \omega_u L_A$:
$$\arctan(10x) + x = \pi \implies x = 1.6320$$
$$L_A = \frac{1.6320}{0.11744} = 13.90\text{ s}, \quad \tau_A = 10 L_A = 138.96\text{ s}$$
The magnitude condition yields:
$$|G_A(j\omega_u)| = \frac{K_A}{\sqrt{1 + (\omega_u \tau_A)^2}} = \frac{1}{K_u} \implies K_A = \frac{\sqrt{1 + 16.32^2}}{0.9817} = 16.65$$

Hence, the continuous-time plant model and PI controller are:
$$G_A(s) = \frac{16.65}{138.96s + 1} e^{-13.90s}$$
$$C_A(s) = \frac{0.44178s + 0.009909}{s} = 0.44178 \frac{s + 0.02243}{s}$$

#### Stability Margins & Singularity Mapping
From [`matlab_workspace_variables_save.mat`](file:///C:/Users/Fufal%20Jigar/Downloads/New%20folder/Smart_HVAC_System/TransferFunctionAnalysis/matlab_workspace_variables_save.mat):
- **Singularities:**
  - Integrator Pole: $s = 0$
  - Compensator Zero: $z_c = -0.0224\text{ rad/s}$
  - Dominant Plant Pole: $p_1 = -0.0072\text{ rad/s}$ ($s = -1/\tau_A$)
- **Root Locus Crossing:** Using a 2nd-order Padé approximation for the delay $e^{-13.90s}$, the root locus crosses the imaginary axis precisely at $K = K_u = 0.9817$ and $\omega = \pm j0.1174\text{ rad/s}$.
- **Loop Transfer Function $L_A(s) = C_A(s)G_A(s)$ Stability Margins:**
  - **Gain Margin (GM):** **$5.56\text{ dB}$** (Gain ratio $= 1.897$) at phase crossover frequency $\omega_{cg} = 0.1026\text{ rad/s}$.
  - **Phase Margin (PM):** **$30.62^\circ$** at gain crossover frequency $\omega_{cp} = 0.0565\text{ rad/s}$.
  - **Closed-Loop Poles:** $s \in \{-0.4024, -0.0287 \pm j0.0699, -0.0321\}$. All poles lie strictly in the open left-half complex plane, ensuring asymptotic stability.

---

## 4. Complete Repository File & Asset Catalog

The table below catalogs the files in the repository and directly connects them to their role in the system and their corresponding location in the attached project report:

| Path in Repository | Description / Type | Corresponding Report Section & Figure |
| :--- | :--- | :--- |
| `ControlFinalProject.pdf` | Master 14-page IEEE final project report | Primary reference document |
| **AllImages/** | High-resolution figure archive | |
| `AllImages/Hardware1.jpeg` | Complete hardware testbench photo | Fig. 1(a) (Hardware testbench) |
| `AllImages/Hardware2.jpeg` | Exhaust baffle & wiring detail photo | Fig. 1(b) (Baffle & wiring) |
| `AllImages/Hardware3.jpeg` | Interior chamber & detached hatch photo | Fig. 1(c) (Chamber interior) |
| `AllImages/CircuitDiagram...Final.png` | Complete circuit & power distribution wiring schematic | Fig. 2 (Electrical wiring schematic) |
| `AllImages/OnOff_25_with_normal...png` | ON/OFF Case 1 step response & relay timeline | Fig. 3 (Predictive 25°C step response) |
| `AllImages/onoff_25_withlowfanthreshold.png`| ON/OFF Case 2 step response (actuator fighting) | Fig. 4 (Centered 25°C step response) |
| `AllImages/onoff_sp_30_updated.png` | ON/OFF Case 3 step response (convective damping) | Fig. 5 (Centered 30°C step response) |
| `AllImages/PITune_01.jpg` | ZN relay auto-tuning Trial 1 ($P_u = 60.5\text{ s}$) | Fig. 6 (ZN Trial 1 plot) |
| `AllImages/PITune_2.jpg` | ZN relay auto-tuning Trial 2 ($P_u = 53.5\text{ s}$) | Fig. 7 (ZN Trial 2 plot) |
| `AllImages/Run1_30C_PIfinal.jpg` | PI closed-loop 3-panel step response ($30^\circ\text{C}$) | Fig. 8 (3-panel 30°C PI response) |
| `AllImages/Run1_30C_PIfinal_i.jpg` | Annotated step metrics ($t_r=115\text{s}, t_s=567\text{s}$) | Fig. 9 (Step metrics at 30°C) |
| `AllImages/Run2_25C_Final_PI.png` | Active cooling down-step ($33.72 \to 25^\circ\text{C}$) | Fig. 10 (Cooling down-step) |
| `AllImages/Run2_25C_PIfinal_3.jpg` | Sustained 600 s regulation verification ($25^\circ\text{C}$) | Fig. 11 (600 s steady timeline) |
| `AllImages/MaximumTemp39_Graph.png` | 100% heater saturation test ($39.20^\circ\text{C}$ ceiling) | Fig. 14 (Saturation ceiling test) |
| `AllImages/GraphPlotted.jpg` | Insulation hatch removal disturbance response | Fig. 15 (Disturbance adaptation plot) |
| `AllImages/hvac_main_response.png` | Nominal PI + diagnostic 3-panel run ($28^\circ\text{C}$) | Fig. 16 (Nominal 28°C telemetry) |
| `AllImages/hvac_step_metrics.png` | Nominal 28°C annotated metrics ($e_{ss}=+0.019^\circ\text{C}$)| Fig. 17 (Nominal 28°C metrics) |
| **OnOffController/** | Discrete ON/OFF control modules | Section III |
| `OnOffController.m` | Universal auto-settling bang-bang script | Section III-A & III-B |
| `Readme.txt` | Configuration notes | Section III |
| `Readings/CodesUsed/` | Modified scripts for 25°C and 30°C runs | Table II |
| `Readings/onoff_25/` | Case 1 predictive cutoff raw CSV & metrics | Fig. 3 & Table II (Case 1) |
| `Readings/onoff_25_withfanthreshold...`| Case 2 centered threshold raw CSV & metrics | Fig. 4 & Table II (Case 2) |
| `Readings/onoff_30deg/` | Case 3 centered 30°C raw CSV & metrics | Fig. 5 & Table II (Case 3) |
| **PIController/** | Master PI controller & experimental trials | Sections IV, V, VI |
| `PIController.m` | Final unified PI controller + 4-code FDI | Section IV-B & VI-B |
| `Readme.txt` | Hierarchical design notes | Section IV |
| `Readings/PITune/` | ZN relay auto-tuning scripts, CSVs, MATs | Section IV-A, Figs. 6-7, Table III |
| `Readings/PIRunRiseTill25C/` | 30°C step-up raw telemetry & metrics | Section V-A, Figs. 8-9, Table IV |
| `Readings/PIRunDropTill25C/` | 25°C step-down raw telemetry & metrics | Section V-A, Figs. 10-11, Table IV |
| `Readings/PIRun16to35/` | 35°C cold/warm regulation & dip recovery | Section V-B, Figs. 12-13, Table V |
| `Readings/MaxTemp39/` | 40°C saturation run to 39.20°C ceiling | Section V-B, Fig. 14, Table V |
| `Readings/FAULTBASEDREADINGS/` | Insulation cover removal raw data & script | Section VI-A, Fig. 15, Table VI |
| `Readings/PI28CwithFinalCode/` | Nominal 28°C diagnostic run raw data | Section VI-B, Figs. 16-17, Table VII |
| `Readings/PI32CwithFinalCode...` | 32°C sequential fault injection raw data | Section VI-B, Fig. 18, Table VII |
| **TransferFunctionAnalysis/** | System identification & stability analysis | Section VII |
| `PlayingWithTransferFunction.m` | Step metric calculation & telemetry tool | Section VII |
| `matlab_workspace_variables_save.mat` | MATLAB workspace containing plant TF, margins, poles | Section VII, Table VIII, Eqs. 9-10 |
| `fig_pz_PI.png / .pdf` | PI controller pole-zero s-plane map | Fig. 19(a) (Compensator pole-zero) |
| `fig_pz_loop.png / .pdf` | Delay-free open-loop pole-zero map | Fig. 19(b) (Loop pole-zero) |
| `fig_rlocus.png / .pdf` | Root locus with 2nd-order Padé approximation | Fig. 20 (Padé root locus) |
| `fig_bode.png / .pdf` | Exact-delay Bode frequency response plot | Fig. 21 (Bode plot) |

---

## 5. Synthesis & Key Engineering Takeaways

1. **Discrete Actuation Dynamics:**
   Unlike continuous systems where actuators can apply fractional voltages, relay-based HVAC prototypes suffer from contact wear and high-frequency switching chatter. The project resolves this by enforcing a minimum $2.0\text{ s}$ dwell time, deadband mapping ($u > 0.05$), and integral clamping ($[-0.5, +0.5]$).
2. **Preventing Actuator Conflicts:**
   The comparison between Case 1 and Case 2 in the ON/OFF analysis demonstrates that overlapping heater and fan threshold regions result in destructive actuator fighting, causing a **$33.1\%$ increase in energy dissipation** and severe limit cycling. In contrast, the unified PI controller's supervisory fan override operates with clean directional hysteresis ($\Delta_{\text{fan}} = 1.5^\circ\text{C}$), completely preventing concurrent or rapid alternating actuation.
3. **Integral Adaptation vs. Disturbances:**
   During physical insulation removal, the PI controller seamlessly adjusted the steady-state heater duty cycle from $50.9\%$ to $71.2\%$ (+39.8% power boost), restoring the chamber to $24.93^\circ\text{C}$ ($e_{ss} = -0.07^\circ\text{C}$), demonstrating practical closed-loop robustness.
4. **Deterministic Fault Isolation:**
   When relying on low-cost single-point thermometry (LM35) and discrete relays, rule-based diagnostic state machines with debounce windows and streak timers provide effective hardware fault detection (detecting heater failure, open hatch, and sensor spikes). However, as observed during reconnection transients, large sensor spikes require careful handling in the filter state to prevent temporary false-positive lid-open alarms.
5. **Rigorous Verification:**
   Every experimental claim made in the report—from the $39.20^\circ\text{C}$ thermal ceiling and $53.50\text{ s}$ ultimate period to the $5.56\text{ dB}$ Gain Margin—is backed by raw, reproducible CSV datasets and MATLAB workspace variables in this repository.

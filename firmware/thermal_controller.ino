/*
 * thermal_controller.ino — Arduino firmware for thermal chamber control
 *
 * Features:
 *   - Two DS18B20 temperature sensors on a single OneWire bus
 *   - PWM heater control via MOSFET
 *   - PWM fan control
 *   - Three control modes: OFF, ON-OFF (relay), PI with anti-windup
 *   - Non-blocking 1-second control loop
 *   - Model-based fault detection (residual, sensor disagreement, airflow)
 *   - Serial commands for live tuning (no reflash needed)
 *   - CSV output for data logging
 *   - 50°C hard safety cutoff
 *
 * Wiring:
 *   DS18B20 data → pin 2 (4.7kΩ pull-up to 5V)
 *   Heater MOSFET gate → pin 9 (via 100Ω, 10kΩ gate-to-GND)
 *   Fan MOSFET → pin 10
 *   Common GND for Arduino, supply, MOSFETs, sensors
 *
 * Serial Commands:
 *   m0 / m1 / m2  — Set mode (OFF / ON-OFF / PI)
 *   r35.0         — Set reference temperature
 *   f200          — Set fan PWM (0-255)
 *   k1.5          — Set Kp
 *   i0.01         — Set Ki
 *   p             — Print current parameters
 *   x             — Emergency stop (mode OFF, heater OFF)
 *
 * CSV Output (every 1s):
 *   time_ms, ref, T1, T2, u, fan_cmd, mode, fault_flags
 *
 * Author : Control Systems Project Team
 * Date   : November 2026
 * -----------------------------------------------------------------------
 */

#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>
#include "config.h"

// ========================================================================
//  GLOBAL VARIABLES
// ========================================================================

// ---- Sensors ----
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);
DeviceAddress sensor1Addr, sensor2Addr;
float T1 = 0.0, T2 = 0.0;         // Current sensor readings [°C]
bool sensor1OK = false, sensor2OK = false;
int numSensors = 0;

// ---- Controller State ----
int     controlMode = MODE_OFF;     // Current control mode
float   ref         = DEFAULT_SETPOINT;   // Setpoint [°C]
float   Kp          = DEFAULT_KP;         // Proportional gain
float   Ki          = DEFAULT_KI;         // Integral gain
float   integrator  = 0.0;               // PI integral accumulator
float   u           = 0.0;               // Control output (0-1)
int     fanPWM      = DEFAULT_FAN_PWM;    // Fan PWM command (0-255)

// ---- Model-Based Fault Detection ----
float   T_pred      = MODEL_TAMB;    // Predicted temperature [°C]
float   residual    = 0.0;           // T_meas - T_pred
int     residualCnt = 0;            // Consecutive seconds above threshold
int     disagCnt    = 0;            // Sensor disagreement counter
uint8_t faultFlags  = 0;            // Fault bitmask

// ---- Timing ----
unsigned long lastControlTime = 0;
unsigned long startTime       = 0;

// ---- Delay Buffer for Dead Time (model prediction) ----
#define DELAY_BUFFER_SIZE  10       // Seconds of delay buffer
float uBuffer[DELAY_BUFFER_SIZE];
int   uBufferIdx = 0;

// ========================================================================
//  SETUP
// ========================================================================
void setup() {
    // ---- Serial ----
    Serial.begin(SERIAL_BAUD);
    while (!Serial) { ; }  // Wait for USB serial
    
    // ---- Pin Setup ----
    pinMode(PIN_HEATER_PWM, OUTPUT);
    pinMode(PIN_FAN_PWM, OUTPUT);
    analogWrite(PIN_HEATER_PWM, 0);   // Heater OFF at startup
    analogWrite(PIN_FAN_PWM, fanPWM);
    
    // ---- Sensor Init ----
    sensors.begin();
    numSensors = sensors.getDeviceCount();
    
    Serial.print("# Sensors found: ");
    Serial.println(numSensors);
    
    if (numSensors >= 1) {
        sensors.getAddress(sensor1Addr, 0);
        sensors.setResolution(sensor1Addr, SENSOR_RESOLUTION);
    }
    if (numSensors >= 2) {
        sensors.getAddress(sensor2Addr, 1);
        sensors.setResolution(sensor2Addr, SENSOR_RESOLUTION);
    }
    
    // Set to non-blocking (async) mode
    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();   // Start first conversion
    
    // ---- Initialize delay buffer ----
    for (int i = 0; i < DELAY_BUFFER_SIZE; i++) {
        uBuffer[i] = 0.0;
    }
    
    // ---- Print CSV Header ----
    Serial.println("# Thermal Controller Initialized");
    Serial.print("# Mode: "); Serial.print(controlMode);
    Serial.print(", Ref: "); Serial.print(ref);
    Serial.print(", Kp: "); Serial.print(Kp, 4);
    Serial.print(", Ki: "); Serial.println(Ki, 6);
    Serial.println("# time_ms,ref,T1,T2,u,fan_cmd,mode,fault_flags");
    
    startTime = millis();
    lastControlTime = millis();
    
    // Wait for first conversion
    delay(800);
}

// ========================================================================
//  MAIN LOOP
// ========================================================================
void loop() {
    unsigned long now = millis();
    
    // ---- Process serial commands (always) ----
    processSerialCommands();
    
    // ---- Control loop at Ts = 1 s (non-blocking) ----
    if (now - lastControlTime >= CONTROL_PERIOD_MS) {
        lastControlTime = now;
        
        // ---- 1. Read Sensors ----
        readSensors();
        
        // ---- 2. Safety Check ----
        faultFlags = 0;
        safetyCheck();
        
        // ---- 3. Compute Control Signal ----
        if (!(faultFlags & (FLAG_OVERHEAT | FLAG_SENSOR_FAIL))) {
            computeControl();
        }
        // If safety tripped, u is already set to 0 in safetyCheck
        
        // ---- 4. Apply Outputs ----
        applyOutputs();
        
        // ---- 5. Model-Based Fault Detection ----
        updateModelPrediction();
        checkFaults();
        
        // ---- 6. Log Data (CSV) ----
        logData(now - startTime);
        
        // ---- 7. Request next temperature conversion ----
        sensors.requestTemperatures();
    }
}

// ========================================================================
//  SENSOR READING
// ========================================================================
void readSensors() {
    sensor1OK = false;
    sensor2OK = false;
    
    if (numSensors >= 1) {
        float reading = sensors.getTempC(sensor1Addr);
        if (reading != DEVICE_DISCONNECTED_C && 
            reading > T_SENSOR_MIN && reading < T_SENSOR_MAX &&
            reading != SENSOR_RESET_VALUE) {
            T1 = reading;
            sensor1OK = true;
        }
    }
    
    if (numSensors >= 2) {
        float reading = sensors.getTempC(sensor2Addr);
        if (reading != DEVICE_DISCONNECTED_C && 
            reading > T_SENSOR_MIN && reading < T_SENSOR_MAX &&
            reading != SENSOR_RESET_VALUE) {
            T2 = reading;
            sensor2OK = true;
        }
    }
    
    // If only one sensor, duplicate its reading
    if (numSensors == 1) {
        T2 = T1;
        sensor2OK = sensor1OK;
    }
}

// ========================================================================
//  SAFETY CHECK
// ========================================================================
void safetyCheck() {
    // ---- Overheat ----
    if ((sensor1OK && T1 > T_SAFETY_MAX) || 
        (sensor2OK && T2 > T_SAFETY_MAX)) {
        faultFlags |= FLAG_OVERHEAT;
        u = 0.0;
        integrator = 0.0;
        analogWrite(PIN_HEATER_PWM, 0);  // Immediate shutoff
    }
    
    // ---- Sensor failure ----
    if (!sensor1OK && !sensor2OK) {
        faultFlags |= FLAG_SENSOR_FAIL;
        u = 0.0;
        integrator = 0.0;
        analogWrite(PIN_HEATER_PWM, 0);  // Immediate shutoff
    }
    
    // If only one sensor failed, use the other
    if (!sensor1OK && sensor2OK) {
        T1 = T2;   // Use backup sensor
        faultFlags |= FLAG_SENSOR_FAIL;
        // Don't shut off — still have one good reading
    } else if (sensor1OK && !sensor2OK) {
        T2 = T1;
        faultFlags |= FLAG_SENSOR_FAIL;
    }
}

// ========================================================================
//  CONTROL COMPUTATION
// ========================================================================
void computeControl() {
    float e = ref - T1;   // Error (using primary sensor)
    
    switch (controlMode) {
        case MODE_OFF:
            u = 0.0;
            integrator = 0.0;
            break;
            
        case MODE_ONOFF:
            // ON-OFF with hysteresis (±0.5°C deadband)
            if (T1 < ref - HYSTERESIS) {
                u = 1.0;
            } else if (T1 > ref + HYSTERESIS) {
                u = 0.0;
            }
            // else: hold previous value (hysteresis band)
            integrator = 0.0;   // Reset for when switching to PI
            break;
            
        case MODE_PI:
            {
                // PI with anti-windup (clamping)
                float up = Kp * e + integrator;
                u = constrain(up, DUTY_MIN, DUTY_MAX);
                
                // Only integrate when not saturated (anti-windup)
                if (u == up) {
                    integrator += Ki * e * (CONTROL_PERIOD_MS / 1000.0);
                }
            }
            break;
            
        default:
            u = 0.0;
            break;
    }
    
    // Clamp output
    u = constrain(u, DUTY_MIN, DUTY_MAX);
}

// ========================================================================
//  APPLY OUTPUTS
// ========================================================================
void applyOutputs() {
    // Heater: map duty 0-1 to PWM 0-255
    int heaterPWM = (int)(u * 255.0);
    analogWrite(PIN_HEATER_PWM, heaterPWM);
    
    // Fan: direct PWM
    analogWrite(PIN_FAN_PWM, fanPWM);
}

// ========================================================================
//  MODEL-BASED FAULT DETECTION
// ========================================================================
void updateModelPrediction() {
    // First-order model prediction:
    //   dT_pred/dt = (1/tau) * (K*P*u_delayed - (T_pred - T_amb))
    //   T_pred[k+1] = T_pred[k] + (Ts/tau) * (K*u_delayed - (T_pred - T_amb)/R_th * (1/P))
    //
    // Simplified discrete update:
    //   T_pred[k+1] = T_pred[k] + (Ts/tau) * (K * u_delayed - (T_pred[k] - T_amb))
    
    float Ts = CONTROL_PERIOD_MS / 1000.0;
    
    // Get delayed control signal
    int delaySteps = (int)(MODEL_L / Ts);
    if (delaySteps >= DELAY_BUFFER_SIZE) delaySteps = DELAY_BUFFER_SIZE - 1;
    int delayIdx = (uBufferIdx - delaySteps + DELAY_BUFFER_SIZE) % DELAY_BUFFER_SIZE;
    float u_delayed = uBuffer[delayIdx];
    
    // Update prediction (Euler integration)
    float dT = (Ts / MODEL_TAU) * (MODEL_K * u_delayed - (T_pred - MODEL_TAMB));
    T_pred += dT;
    
    // Store current u in delay buffer
    uBuffer[uBufferIdx] = u;
    uBufferIdx = (uBufferIdx + 1) % DELAY_BUFFER_SIZE;
    
    // Compute residual
    float T_meas = (sensor1OK && sensor2OK) ? (T1 + T2) / 2.0 : T1;
    residual = T_meas - T_pred;
}

void checkFaults() {
    // ---- Model residual check ----
    if (fabs(residual) > FAULT_RESIDUAL_THRESH) {
        residualCnt++;
        if (residualCnt >= FAULT_RESIDUAL_DUR) {
            faultFlags |= FLAG_MODEL_RESIDUAL;
        }
    } else {
        residualCnt = 0;
    }
    
    // ---- Sensor disagreement ----
    if (sensor1OK && sensor2OK && numSensors >= 2) {
        if (fabs(T1 - T2) > FAULT_SENSOR_DISAGREE) {
            disagCnt++;
            if (disagCnt >= FAULT_SENSOR_DIS_DUR) {
                faultFlags |= FLAG_SENSOR_DISAGREE;
            }
        } else {
            disagCnt = 0;
        }
    }
    
    // ---- Restricted airflow detection ----
    // If in steady state (small error) but duty is much higher than expected
    float e = fabs(ref - T1);
    if (e < 1.0 && u > FAULT_AIRFLOW_RATIO) {
        // Check if duty is significantly higher than what the model predicts
        float expected_duty = (ref - MODEL_TAMB) / MODEL_K;
        if (u > expected_duty * 1.5 && expected_duty > 0.1) {
            faultFlags |= FLAG_RESTRICTED_AIRFLOW;
        }
    }
}

// ========================================================================
//  DATA LOGGING (CSV)
// ========================================================================
void logData(unsigned long elapsed_ms) {
    // Format: time_ms, ref, T1, T2, u, fan_cmd, mode, fault_flags
    Serial.print(elapsed_ms);
    Serial.print(",");
    Serial.print(ref, 2);
    Serial.print(",");
    Serial.print(T1, 2);
    Serial.print(",");
    Serial.print(T2, 2);
    Serial.print(",");
    Serial.print(u, 4);
    Serial.print(",");
    Serial.print(fanPWM);
    Serial.print(",");
    Serial.print(controlMode);
    Serial.print(",");
    Serial.println(faultFlags);
}

// ========================================================================
//  SERIAL COMMAND PROCESSING
// ========================================================================
void processSerialCommands() {
    if (!Serial.available()) return;
    
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.length() == 0) return;
    
    char first = cmd.charAt(0);
    String value = cmd.substring(1);
    
    switch (first) {
        case 'm':   // Mode: m0, m1, m2
        case 'M':
            {
                int newMode = value.toInt();
                if (newMode >= MODE_OFF && newMode <= MODE_PI) {
                    controlMode = newMode;
                    integrator = 0.0;   // Reset integrator on mode change
                    Serial.print("# Mode: ");
                    Serial.println(controlMode);
                } else {
                    Serial.println("# Error: mode must be 0, 1, or 2");
                }
            }
            break;
            
        case 'r':   // Reference: r35.0
        case 'R':
            {
                float newRef = value.toFloat();
                if (newRef >= 20.0 && newRef <= T_PLA_MAX) {
                    ref = newRef;
                    Serial.print("# Ref: ");
                    Serial.println(ref, 1);
                } else {
                    Serial.print("# Error: ref must be 20-");
                    Serial.println(T_PLA_MAX, 0);
                }
            }
            break;
            
        case 'f':   // Fan PWM: f200
        case 'F':
            {
                int newFan = value.toInt();
                if (newFan >= 0 && newFan <= 255) {
                    fanPWM = newFan;
                    Serial.print("# Fan: ");
                    Serial.println(fanPWM);
                } else {
                    Serial.println("# Error: fan must be 0-255");
                }
            }
            break;
            
        case 'k':   // Kp: k1.5
        case 'K':
            {
                float newKp = value.toFloat();
                if (newKp >= 0.0 && newKp <= 100.0) {
                    Kp = newKp;
                    Serial.print("# Kp: ");
                    Serial.println(Kp, 4);
                }
            }
            break;
            
        case 'i':   // Ki: i0.01
        case 'I':
            {
                float newKi = value.toFloat();
                if (newKi >= 0.0 && newKi <= 10.0) {
                    Ki = newKi;
                    integrator = 0.0;   // Reset integrator
                    Serial.print("# Ki: ");
                    Serial.println(Ki, 6);
                }
            }
            break;
            
        case 'p':   // Print parameters
        case 'P':
            Serial.println("# --- Current Parameters ---");
            Serial.print("#   Mode: "); Serial.println(controlMode);
            Serial.print("#   Ref:  "); Serial.println(ref, 1);
            Serial.print("#   Kp:   "); Serial.println(Kp, 4);
            Serial.print("#   Ki:   "); Serial.println(Ki, 6);
            Serial.print("#   Fan:  "); Serial.println(fanPWM);
            Serial.print("#   T1:   "); Serial.println(T1, 2);
            Serial.print("#   T2:   "); Serial.println(T2, 2);
            Serial.print("#   u:    "); Serial.println(u, 4);
            Serial.print("#   Flags:"); Serial.println(faultFlags, BIN);
            Serial.print("#   I_acc:"); Serial.println(integrator, 4);
            break;
            
        case 'x':   // Emergency stop
        case 'X':
            controlMode = MODE_OFF;
            u = 0.0;
            integrator = 0.0;
            analogWrite(PIN_HEATER_PWM, 0);
            Serial.println("# EMERGENCY STOP — Heater OFF");
            break;
            
        default:
            Serial.print("# Unknown command: ");
            Serial.println(cmd);
            Serial.println("# Commands: m[0-2] r[temp] f[0-255] k[Kp] i[Ki] p x");
            break;
    }
}

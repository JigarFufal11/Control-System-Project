/*
 * config.h — Configuration and tunable constants for the thermal controller
 *
 * All pin assignments, safety limits, controller defaults, model parameters,
 * and fault detection thresholds in one place.
 *
 * Author : Control Systems Project Team
 * Date   : November 2026
 * -----------------------------------------------------------------------
 */

#ifndef CONFIG_H
#define CONFIG_H

// ========================================================================
//  PIN ASSIGNMENTS
// ========================================================================
#define PIN_ONEWIRE     2       // DS18B20 data pin (4.7kΩ pull-up to 5V)
#define PIN_HEATER_PWM  9       // Heater MOSFET gate (via 100Ω resistor)
#define PIN_FAN_PWM     10      // Fan MOSFET/transistor gate

// ========================================================================
//  SENSOR CONFIGURATION
// ========================================================================
#define SENSOR_RESOLUTION   12      // DS18B20 resolution: 9, 10, 11, or 12 bits
                                    // 12-bit → 0.0625°C, ~750ms conversion
#define SENSOR_READ_TIMEOUT 1000    // Max ms to wait for sensor reading
#define SENSOR_FAIL_VALUE   -127.0  // DallasTemperature returns this on failure
#define SENSOR_RESET_VALUE  85.0    // DS18B20 power-on reset value (ignore)

// ========================================================================
//  TIMING
// ========================================================================
#define CONTROL_PERIOD_MS   1000    // Control loop period Ts = 1 s
#define SERIAL_BAUD         9600    // Serial communication baud rate

// ========================================================================
//  SAFETY LIMITS (DO NOT INCREASE)
// ========================================================================
#define T_SAFETY_MAX        50.0    // Hard cutoff temperature [°C]
#define T_PLA_MAX           45.0    // Soft limit for PLA chambers [°C]
#define T_SENSOR_MIN        -10.0   // Below this = sensor fault
#define T_SENSOR_MAX        60.0    // Above this = sensor fault

// ========================================================================
//  CONTROLLER DEFAULTS
// ========================================================================
// PI controller (IMC-tuned for K=15, tau=120, L=3)
#define DEFAULT_KP          0.4444  // Proportional gain: tau/(K*(L+tau_c))
#define DEFAULT_KI          0.0037  // Integral gain: Kp/tau
#define DEFAULT_SETPOINT    30.0    // Initial setpoint [°C]
#define DEFAULT_FAN_PWM     150     // Default fan PWM (0-255)

// ON-OFF hysteresis
#define HYSTERESIS          0.5     // ±0.5°C deadband

// Duty limits
#define DUTY_MIN            0.0     // Minimum duty cycle
#define DUTY_MAX            1.0     // Maximum duty cycle

// ========================================================================
//  PLANT MODEL PARAMETERS (for fault detection)
// ========================================================================
#define MODEL_K             15.0    // Static gain [°C per unit duty]
#define MODEL_TAU           120.0   // Time constant [s]
#define MODEL_L             3.0     // Dead time [s] (used as buffer index)
#define MODEL_P             50.0    // Heater power [W]
#define MODEL_TAMB          25.0    // Assumed ambient temperature [°C]

// ========================================================================
//  FAULT DETECTION THRESHOLDS
// ========================================================================
#define FAULT_RESIDUAL_THRESH   3.0     // |T_meas − T_pred| threshold [°C]
#define FAULT_RESIDUAL_DUR      5       // Consecutive seconds to confirm [s]
#define FAULT_SENSOR_DISAGREE   1.5     // |T1 − T2| threshold [°C]
#define FAULT_SENSOR_DIS_DUR    5       // Consecutive seconds to confirm [s]
#define FAULT_AIRFLOW_RATIO     0.5     // If duty < this × nominal → restricted

// Fault flag bitmask
#define FLAG_OVERHEAT           0x01    // Bit 0: temperature > T_SAFETY_MAX
#define FLAG_SENSOR_FAIL        0x02    // Bit 1: sensor read failure
#define FLAG_SENSOR_DISAGREE    0x04    // Bit 2: |T1 − T2| > threshold
#define FLAG_MODEL_RESIDUAL     0x08    // Bit 3: model residual too large
#define FLAG_RESTRICTED_AIRFLOW 0x10    // Bit 4: possible blocked airflow

// ========================================================================
//  CONTROL MODES
// ========================================================================
#define MODE_OFF    0
#define MODE_ONOFF  1
#define MODE_PI     2

#endif // CONFIG_H

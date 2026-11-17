# Firmware — Thermal Controller for Arduino

## Wiring Diagram

```
                        +5V (Arduino)
                         │
                        [4.7kΩ]  Pull-up resistor
                         │
  Arduino Pin 2 ────────┤──── DS18B20 #1 (DQ pin)
                         │
                         ├──── DS18B20 #2 (DQ pin)
                         │
                        GND

  DS18B20 Pinout (flat side facing you):
  ┌───────┐
  │ 1 2 3 │
  └───────┘
    │ │ │
    G D V
    N Q D
    D   D

                     +12V (or heater supply)
                       │
                     [FUSE]  (rated for heater current)
                       │
                    ┌──────┐
                    │HEATER│
                    └──┬───┘
                       │ (Drain)
                    ┌──┴──┐
  Pin 9 ──[100Ω]──│MOSFET│  (N-channel, e.g., IRLZ44N or IRL540)
                    │ Gate │
                    └──┬──┘
                [10kΩ] │ (Source)
                    │  │
                   GND GND

                     +12V
                       │
                    ┌──────┐
                    │  FAN  │
                    └──┬───┘
                    ┌──┴──┐         ┌──┐
  Pin 10 ─────────│MOSFET│    ──┤▶├── Flyback diode
                    │      │    │  └──┘  (across fan)
                    └──┬──┘    │
                       │       │
                      GND     +12V

  COMMON GROUND: Arduino GND = Supply GND = MOSFET Sources = Sensor GND
```

## Required Libraries

Install these via Arduino IDE Library Manager:

1. **OneWire** by Paul Stoffregen (v2.3+)
2. **DallasTemperature** by Miles Burton (v3.9+)

Or install via CLI:
```bash
arduino-cli lib install "OneWire"
arduino-cli lib install "DallasTemperature"
```

## Files

| File | Description |
|------|-------------|
| `thermal_controller.ino` | Main Arduino sketch |
| `config.h` | All tunable constants and pin definitions |

## Serial Commands (9600 baud)

| Command | Description | Example |
|---------|-------------|---------|
| `m0` | Mode: OFF (heater disabled) | `m0` |
| `m1` | Mode: ON-OFF (relay with ±0.5°C hysteresis) | `m1` |
| `m2` | Mode: PI (with anti-windup) | `m2` |
| `r<temp>` | Set reference temperature [°C] | `r40` |
| `f<pwm>` | Set fan PWM (0-255) | `f200` |
| `k<val>` | Set proportional gain Kp | `k1.5` |
| `i<val>` | Set integral gain Ki | `i0.01` |
| `p` | Print current parameters | `p` |
| `x` | Emergency stop | `x` |

## CSV Output Format

```
time_ms, ref, T1, T2, u, fan_cmd, mode, fault_flags
```

Fault flags (bitmask):
- Bit 0 (`0x01`): Overheat (T > 50°C)
- Bit 1 (`0x02`): Sensor read failure
- Bit 2 (`0x04`): Sensor disagreement (|T1−T2| > 1.5°C)
- Bit 3 (`0x08`): Model residual too large
- Bit 4 (`0x10`): Restricted airflow detected

## Safety Checklist

Before **every** power-on:

- [ ] Check all wiring connections (power off)
- [ ] Verify common ground between Arduino, supply, MOSFETs, sensors
- [ ] Confirm fuse is rated correctly and installed
- [ ] Check MOSFET gate resistors are in place
- [ ] Verify flyback diode across fan (correct polarity!)

First power-on tests:

1. [ ] Both sensors read plausible room temperature
2. [ ] Heater at 10% duty for 30s: warms gently, no smell, MOSFET stays cool
3. [ ] Fan spins and responds to PWM changes
4. [ ] **Pull a sensor wire**: heater must shut off immediately
5. [ ] Send `x` command: heater shuts off

## Uploading

```bash
# Arduino CLI
arduino-cli compile --fqbn arduino:avr:uno firmware/
arduino-cli upload --fqbn arduino:avr:uno --port COM3 firmware/

# Or use Arduino IDE: File → Open → thermal_controller.ino → Upload
```

## Typical Experiment Sequence

```
# Step test (20-30 min)
m0          # Start in OFF mode
f150        # Set fan
r30         # Set reference (doesn't matter in step test)
m2          # Switch to PI... or for open-loop step test:

# For open-loop step test, modify code to set u=0.5 directly
# Or use: m1 and r60 (ON-OFF will stay ON since 60 > T_room)

# PI control run
m0          # Reset
r30         # Set 30°C
m2          # Start PI
# Wait for settling...
r40         # Step to 40°C
# Wait for settling...
r35         # Step to 35°C

# ON-OFF control run
m0          # Reset
r30         # Set 30°C
m1          # Start ON-OFF
r40
r35
```

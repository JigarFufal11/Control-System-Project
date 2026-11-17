# 📦 Intermediate & Historical Project Files (`extra_files/`)

This directory contains intermediate artifacts, early exploratory models, test logs, and development scripts created during earlier stages of the **Control Systems (EE302) Smart HVAC Project**.

They have been preserved for full technical reproducibility and historical traceability, while the main production codebase is structured into the three primary pillars:

1. **[`../simulink/`](../simulink/)**: Pure software simulation, Simulink models, and IMC/ZN tuning.
2. **[`../qnet_hvac_ni_elvis/`](../qnet_hvac_ni_elvis/)**: Industrial Quanser QNET HVAC on NI ELVIS II+ DAQ (hardware and Digital Twin).
3. **[`../custom_hardware/`](../custom_hardware/)**: Custom physical hardware chamber (Arduino/MOSFET), experimental logs, On-Off and PI controllers, transfer function analysis, and lab reports.

---

### Contents of `extra_files/`:

- **`data/`**: Early open-loop bump test and telemetry CSV logs (`step1.csv`, `onoff_run.csv`, `pi_run.csv`, `disturbance_run.csv`, `fault_run.csv`).
- **`firmware/`**: Initial standalone Arduino C++ firmware (`thermal_controller.ino`, `config.h`).
- **`hardware/`**: Early breadboard Simulink deployment model (`thermal_control_hardware.slx`).
- **`matlab/`**: Early parameter extraction and evaluation scripts (`fit_model.m`, `generate_report_plots.m`, `plot_overlay.m`).
- **`plots/`**: Early 300 DPI comparative and diagnostic plots.
- **`python/`**: Standalone Python serial logger (`serial_logger.py`), dataset generator (`generate_datasets.py`), and CSV plotting utilities.
- **`testing/`**: Simscape Electrical 5V-to-12V BJT/MOSFET driver stage simulation (`heater_driver_test.slx`).
- **`PROJECT_GUIDE.md`**: Step-by-step master development guide.
- **`PROJECT_EXECUTION_DOCUMENTATION.md`**: Detailed technical execution and verification log.
- **`PS4.pdf`**: Original EE302 Control Systems Lab Problem Statement 4.

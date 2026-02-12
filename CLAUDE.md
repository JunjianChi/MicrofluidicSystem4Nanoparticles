# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP-IDF firmware for an ESP32-based microfluidic control system. The system integrates a Sensirion SLF3S liquid flow sensor, an MP6 micropump driver, and a PID controller for closed-loop flow control. Target chip is ESP32; the project uses ESP-IDF v5.3.

## Build & Flash Commands

All commands run from the `Firmware/` directory. ESP-IDF toolchain must be installed and `idf.py` available on PATH.

```bash
idf.py build              # Build the project
idf.py -p COM3 flash      # Flash to ESP32 (default port COM3 on Windows)
idf.py -p COM3 monitor    # Serial monitor (Ctrl+] to exit)
idf.py menuconfig         # Configure Kconfig options
idf.py fullclean          # Clean build artifacts
idf.py set-target esp32   # Set target chip
```

## Architecture

### Module Layers

The firmware follows a layered architecture with three levels:

1. **Hardware Abstraction** (`i2c_interface.c/h`) - Board-level I2C master driver wrapping ESP-IDF I2C APIs. All sensor/device drivers use this instead of calling ESP-IDF I2C functions directly. Pin assignments come from Kconfig via `sensor_config.h`.

2. **Device Drivers** - Per-peripheral modules:
   - `SLF3S_flow_sensor.c/h` - Sensirion SLF3S liquid flow sensor driver (I2C addr 0x08). Supports SLF3S-0600F (0-600 ul/min) and SLF3S-1300F (0-40 ml/min) models with different scale factors.
   - `mp6_driver.c/h` - MP6 micropump driver (placeholder, to be implemented)
   - `pid_controller.c/h` - PID closed-loop controller (placeholder, to be implemented)
   - `serial_comm.c/h` - Serial/WiFi communication (placeholder, to be implemented)

3. **Configuration** (`sensor_config.c/h`) - Centralizes all hardware config. Reads Kconfig values to populate a `system_config_t` struct with sensor enable flags, pin assignments, pump driver type, and communication settings.

4. **Application** (`main.c`) - Entry point (`app_main`). Initializes I2C, scans bus, starts flow sensor, and runs a continuous measurement loop.

### Configuration System (Kconfig)

Runtime configuration is managed through ESP-IDF's Kconfig system (`Firmware/main/Kconfig.projbuild`). Use `idf.py menuconfig` to change:

- Sensor enable/disable (pressure, temperature, flow)
- Flow sensor model selection (SLF3S-0600F vs SLF3S-1300F)
- Pump driver type (Low/High/Standard MP-Driver)
- Communication method (Serial vs WiFi)
- Pin assignments (I2C SDA/SCL, MP driver enable/clock)
- WiFi credentials (when WiFi communication is selected)

Default pins: I2C SDA=GPIO21, SCL=GPIO22, MP Enable=GPIO14, MP Clock=GPIO27.

### Dev Container

A devcontainer config exists at `Firmware/.devcontainer/` with the ESP-IDF toolchain and QEMU for emulation.

## Conventions

- All functions return `esp_err_t` and use ESP-IDF error codes (`ESP_OK`, `ESP_FAIL`, etc.)
- Logging uses `ESP_LOGx` macros with a per-module `TAG` string
- Header files use `extern "C"` guards for C++ compatibility
- Files use Doxygen-style `@file`/`@brief`/`@param` comment blocks
- License: MIT (SPDX identifier in each file header)

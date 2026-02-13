# Firmware Development Rules

## Locked Files (DO NOT MODIFY)

| File | Description | Reason |
|------|-------------|--------|
| i2c_interface.c/h | I2C hardware abstraction layer | Tested, all devices depend on this module |
| SLF3S_flow_sensor.c/h | Flow sensor driver | Debugged and verified with hardware |
| mcp4726_dac.c/h | MCP4726 DAC driver | VDD=4.734V calibrated, ACK behavior adapted |
| mp6_driver.c/h | MP6 micropump driver | Three-channel control verified |
| sensor_config.c/h | System configuration | Configuration entry point for all modules |
| Kconfig.projbuild | menuconfig definitions | Defines all configurable parameters |

## Modifiable Files

| File | Current State | To Implement |
|------|---------------|-------------|
| main.c | DAC test code | Rewrite as FreeRTOS multi-task application |
| serial_comm.c/h | Empty placeholder | Serial command parsing + data reporting |
| pid_controller.c/h | Empty placeholder | PID closed-loop control algorithm |

## Architecture Rules

1. I2C communication goes through the `i2c_interface` module (exception: mcp4726 uses low-level API for ACK handling)
2. Pins and parameters are obtained from `sensor_config.h`, never hardcoded
3. Pump control goes through `mp6_driver` API, never directly operating GPIO/DAC
4. New modules follow the existing file header format (author, version, MIT license)

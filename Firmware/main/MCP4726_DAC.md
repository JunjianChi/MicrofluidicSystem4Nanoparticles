# MCP4726 DAC Driver

## Overview

MCP4726 12-bit DAC driver for ESP-IDF, used to set MP-Driver amplitude voltage (0.35-1.3V). Ported from the Arduino reference code in the MP-Driver Electronic Driver Datasheet V1.7.

- I2C address: `0x61`
- Resolution: 12-bit (4096 levels)
- Calibrated VDD: 4.734V

## Files

| File | Description |
|------|-------------|
| `mcp4726_dac.h` | Header - defines, API declarations |
| `mcp4726_dac.c` | Implementation - I2C communication, voltage conversion |

## API

### `esp_err_t mcp4726_init(void)`

Initialize DAC. Must call `i2c_interface_init()` first.

1. Writes VREF register (`0x08`, `0x02`) to select VDD as reference
2. Writes voltage register (`0x00`, `0x00`, `0x00`) to set output to 0V

### `esp_err_t mcp4726_set_raw(uint16_t value)`

Write raw DAC value directly.

- `value`: 0-4095 (clamped internally)
- Sends 3 bytes: `[0x00, value>>8, value&0xFF]`

### `esp_err_t mcp4726_set_voltage(float voltage)`

Set output voltage. Internally converts to raw value and calls `mcp4726_set_raw()`.

- `voltage`: 0 ~ VDD (volts), clamped internally
- Conversion: `bit = voltage / VDD * 4096` (VDD = 4.734V)

Example:
```
0.35V -> bit = 0.35 / 4.734 * 4096 = 303
1.30V -> bit = 1.30 / 4.734 * 4096 = 1124
```

## Usage Example (main.c)

```c
#include "i2c_interface.h"
#include "mcp4726_dac.h"

i2c_interface_init();
mcp4726_init();
mcp4726_set_voltage(0.80f);  // output 0.80V
```

## Issues Encountered & Solutions

### Issue 1: DAC init failed (I2C NACK)

**Symptom**: `mcp4726_init()` returned `ESP_FAIL` on VREF config write. I2C scan found device at 0x61 but data write failed.

**Root cause**: Originally used `i2c_interface_write()` which wraps `i2c_master_write_to_device()`. This function checks ACK on every byte. The MCP4726 on this board NACKs data bytes but still accepts them (Arduino Wire library does not abort on data NACK).

**Solution**: Switched to low-level `i2c_cmd_link` API with byte-by-byte writes. ACK check enabled only on address byte (`true`), disabled on data bytes (`false`):

```c
i2c_master_write_byte(cmd, (MCP4726_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);   // address: check ACK
i2c_master_write_byte(cmd, MCP4726_REG_VREF, false);                             // data: no ACK check
i2c_master_write_byte(cmd, 0x02, false);                                          // data: no ACK check
```

This matches Arduino Wire behavior where `endTransmission()` sends all bytes regardless of per-byte NACK.

### Issue 2: ~0.5V voltage offset

**Symptom**: Setting 1.30V produced ~0.8V output (offset of ~0.5V).

**Root cause**: The voltage-to-bit formula assumed `VDD = 5.0V`, but the actual supply voltage to MCP4726 on the board is lower. The formula `bit = voltage / VDD * 4096` produced incorrect DAC values.

**Solution**: Measured actual VDD with multimeter and calibrated the constant. Changed `MCP4726_VDD` from `5.0f` to `4.734f` in the header:

```c
#define MCP4726_VDD  4.734f  // Measured actual VDD
```

### Architecture decision: DAC separated from MP6 driver

The original `mp6_driver.c` had DAC code embedded as static functions. This was refactored into a standalone `mcp4726_dac` module so that:

1. DAC can be tested independently in `main.c`
2. `mp6_driver.c` calls `mcp4726_set_voltage()` for amplitude, no DAC internals
3. Clear separation: DAC handles bit/voltage conversion, MP6 handles amplitude/voltage mapping

## I2C Register Map

| Register | Address | Format | Description |
|----------|---------|--------|-------------|
| VREF | `0x08` | 1 byte (0x02 = VDD ref) | Voltage reference selection |
| VOLTAGE | `0x00` | 2 bytes (MSB, LSB) | DAC output value |

## Arduino Reference Code (from MP-Driver Datasheet V1.7)

```cpp
#define DAC_I2C_ADDRESS 0x61
#define I2C_DACVOLTAGE  0x00
#define I2C_DACVREF     0x08

// Init
Wire.beginTransmission(DAC_I2C_ADDRESS);
Wire.write(I2C_DACVREF);     // 0x08
Wire.write(0x02);
Wire.endTransmission();

Wire.beginTransmission(DAC_I2C_ADDRESS);
Wire.write(I2C_DACVOLTAGE);  // 0x00
Wire.write(0);
Wire.write(0);
Wire.endTransmission();

// Set DAC value
void setDAC(uint16_t _dacValue) {
    Wire.beginTransmission(DAC_I2C_ADDRESS);
    Wire.write(I2C_DACVOLTAGE);
    Wire.write(_dacValue >> 8);
    Wire.write(_dacValue & 0x00FF);
    Wire.endTransmission();
}
```

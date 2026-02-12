# 代码实现详解 Implementation Details

## 文件结构

```
Firmware/main/
├── main.c                  # 应用入口
├── sensor_config.h/c       # 系统配置（Kconfig 映射）
├── i2c_interface.h/c       # I2C 硬件抽象层
├── SLF3S_flow_sensor.h/c   # SLF3S 流量传感器驱动
├── mcp4726_dac.h/c         # MCP4726 DAC 驱动（已实现）
├── mp6_driver.h/c          # MP6 微泵驱动（已实现）
├── pid_controller.h/c      # PID 控制器（待实现）
├── serial_comm.h/c         # 串口/WiFi 通信（待实现）
├── Kconfig.projbuild       # menuconfig 配置定义
└── MCP4726_DAC.md          # DAC 驱动开发记录
```

---

## 1. main.c — 应用入口

**职责**：系统初始化与主控制循环。

**执行流程**：

```
app_main()
    │
    ├── i2c_interface_init()          ← 初始化 I2C 总线
    ├── i2c_interface_scan()          ← 扫描总线上的设备（调试用）
    ├── slf3s_init(&flow_sensor)      ← 初始化流量传感器 handle
    ├── slf3s_start_measurement()     ← 发送开始测量命令
    │   └── (失败时回退到 simple 模式)
    ├── vTaskDelay(500ms)             ← 等待传感器稳定
    │
    └── while(1) 主循环
        ├── slf3s_read_flow()         ← 读取流量值 (µl/min)
        ├── ESP_LOGI() 打印           ← 输出到串口监视器
        └── vTaskDelay(100ms)         ← 采样间隔 ~10Hz
```

**对应硬件**：当前仅驱动 SLF3S 流量传感器，未来主循环将加入 PID 计算和 MP6 泵控制。

---

## 2. sensor_config.h/c — 系统配置

**职责**：将 Kconfig 编译期宏统一映射为 C 常量和结构体，供所有模块使用。

**核心数据结构**：

```c
typedef struct {
    bool pressure_enabled;          // 来自 CONFIG_PRESSURE_SENSOR_ENABLE
    bool temperature_enabled;       // 来自 CONFIG_TEMPERATURE_SENSOR_ENABLE
    bool flow_enabled;              // 来自 CONFIG_FLOW_SENSOR_ENABLE
    pump_driver_type_t pump_driver; // LOW / HIGH / STANDARD
    comm_type_t comm_type;          // SERIAL / WIFI
    struct { i2c_sda, i2c_scl, mp_enable, mp_clock } pins;
    struct { ssid, password, server_addr, server_port } wifi; // 仅 WiFi 模式
} system_config_t;
```

**Kconfig → 宏映射关系**：

| Kconfig 选项 | 编译宏 | 使用位置 |
|---|---|---|
| `CONFIG_I2C_SDA_PIN` (默认 21) | `I2C_MASTER_SDA_IO` | i2c_interface.c |
| `CONFIG_I2C_SCL_PIN` (默认 22) | `I2C_MASTER_SCL_IO` | i2c_interface.c |
| `CONFIG_MP_DRIVER_ENABLE_PIN` (默认 14) | `MP_DRIVER_ENABLE_IO` | mp6_driver.c |
| `CONFIG_MP_DRIVER_CLOCK_PIN` (默认 27) | `MP_DRIVER_CLOCK_IO` | mp6_driver.c |
| `CONFIG_FLOW_SENSOR_0600F` / `1300F` | `FLOW_SENSOR_SCALE_FACTOR` | SLF3S_flow_sensor.c |

**对应硬件**：本模块不直接操作硬件，而是定义所有硬件相关的参数。

---

## 3. i2c_interface.h/c — I2C 硬件抽象层

**职责**：封装 ESP-IDF 的 I2C master API，为上层传感器驱动提供统一接口。

**对应硬件**：ESP32 的 I2C0 外设，连接到总线上所有 I2C 传感器。

**关键参数**：
- 总线频率：100 kHz
- 超时：1000 ms
- 端口：`I2C_NUM_0`

**API 与用途**：

| 函数 | 说明 | 调用者 |
|------|------|--------|
| `i2c_interface_init()` | 配置 SDA/SCL 引脚，安装 I2C 驱动 | main.c |
| `i2c_interface_write()` | 向指定地址设备发送字节 | SLF3S 驱动发命令 |
| `i2c_interface_read()` | 从指定地址设备读取字节 | SLF3S 驱动读数据 |
| `i2c_interface_write_read()` | 先写寄存器地址再读数据 | 通用寄存器型传感器 |
| `i2c_interface_write_reg()` | 写入指定寄存器 | 通用寄存器型传感器 |
| `i2c_interface_read_reg()` | 读取指定寄存器 | 通用寄存器型传感器 |
| `i2c_interface_probe()` | 检测设备是否在线 | i2c_interface_scan |
| `i2c_interface_scan()` | 遍历 0x03-0x77 扫描所有设备 | main.c（调试） |

**设计要点**：
- SLF3S 不使用寄存器地址模式，而是直接的 write/read 命令模式，所以驱动使用 `i2c_interface_write()` 和 `i2c_interface_read()` 而不是 `write_reg/read_reg`。
- `write_reg/read_reg` 是为未来的压力/温度传感器预留的（这类传感器通常使用寄存器寻址）。

---

## 4. SLF3S_flow_sensor.h/c — 流量传感器驱动

**职责**：驱动 Sensirion SLF3S 系列液体流量传感器。

**对应硬件**：Sensirion SLF3S-0600F 或 SLF3S-1300F，I2C 地址 0x08。

### 支持的传感器型号

| 型号 | 量程 | Scale Factor | 适用场景 |
|------|------|-------------|---------|
| SLF3S-0600F | 0 ~ 600 µl/min | 10000 | 微流控芯片低流量 |
| SLF3S-1300F | 0 ~ 40 ml/min | 500 | 较大流量应用 |

### I2C 命令协议

传感器使用 Sensirion 专有的命令协议（非标准寄存器模式）：

| 命令 | 字节序列 | 说明 |
|------|---------|------|
| Soft Reset | `0x06` | 复位传感器 |
| Start Measurement | `0x36` + `calibration_byte` | 开始连续测量 |
| Stop Measurement | `0x3F 0xF9` | 停止测量 |
| Read Product ID | `0x36 0x7C` → `0xE1 0x02` | 读取产品编号（两步） |

### 数据读取流程

```
传感器连续模式下 → I2C 读取 3 字节 → [MSB] [LSB] [CRC]
                                          │
                  raw_value = (MSB << 8) | LSB   (int16_t, 有符号)
                                          │
                  flow_rate = raw_value / scale_factor   (µl/min)
```

### 两种启动模式

1. **`slf3s_start_measurement()`（完整模式）**：
   - 停止已有测量 → 读取产品编号 → 验证型号 → 软复位 → 开始测量
   - 会自动检测实际连接的传感器型号，并与 Kconfig 配置比对（不匹配时发出警告，但仍使用配置的 scale factor）

2. **`slf3s_start_measurement_simple()`（简化模式）**：
   - 软复位 → 开始测量
   - 跳过产品识别，用于完整模式失败时的降级方案

### 标定模式

通过 `slf3s_set_calibration()` 切换标定液体：
- `0x08` — 水 (Water)，默认
- `0x15` — 异丙醇 (IPA)

标定字节在 Start Measurement 命令中发送给传感器。

---

## 5. mcp4726_dac.h/c — MCP4726 DAC 驱动（2026-02-12 实现）

**职责**：通过 I2C 控制 MCP4726 DAC 输出电压，供 MP-Driver 板作为 amplitude 输入。

**对应硬件**：MCP4726 12-bit DAC，I2C 地址 0x61，VDD = 4.734V（实测校准值）。

**API**：

| 函数 | 说明 |
|------|------|
| `mcp4726_init()` | 配置 VREF=VDD，输出归零 |
| `mcp4726_set_raw(uint16_t value)` | 写入 12-bit 原始值 (0-4095) |
| `mcp4726_set_voltage(float voltage)` | 设置输出电压，内部转换 `bit = voltage / VDD * 4096` |

**I2C 协议**（从 Arduino 参考代码移植）：

```
Init:    [0x61+W] [0x08] [0x02]        ← VREF 寄存器 = VDD
         [0x61+W] [0x00] [0x00] [0x00]  ← 电压输出 = 0
Set DAC: [0x61+W] [0x00] [MSB]  [LSB]  ← 电压输出 = value
```

**关键实现细节**：
- 使用底层 `i2c_cmd_link` API 逐字节发送，数据字节 ACK check 设为 `false`（匹配 Arduino Wire 行为）
- VDD 校准值 4.734V（非理论值 5.0V），解决了 ~0.5V 偏移问题
- 详见 `MCP4726_DAC.md`

---

## 6. mp6_driver.h/c — MP6 微泵驱动（2026-02-12 实现）

**职责**：控制 MP6 压电微泵，三路控制信号：amplitude（DAC）、frequency（PWM）、enable（GPIO）。

**对应硬件**：Bartels MP6 微泵 + MP-Driver 驱动板。

**控制链路**：

```
ESP32                    MP-Driver 板               MP6 泵
┌─────────────┐         ┌──────────┐         ┌──────────┐
│ MCP4726 DAC ├─ I2C ──→│ Amplitude│────────→│          │
│ GPIO27 PWM  ├─ 95% ──→│ Clock    │────────→│  压电泵  │
│ GPIO14      ├─ HIGH ──→│ Enable   │────────→│          │
└─────────────┘         └──────────┘         └──────────┘
```

**API**：

| 函数 | 说明 | 内部实现 |
|------|------|---------|
| `mp6_init(handle)` | 初始化 DAC + GPIO + PWM | 调 `mcp4726_init()`，配 GPIO14，初始化 LEDC |
| `mp6_start(handle)` | 启动泵 | enable→HIGH, clock 启动 |
| `mp6_stop(handle)` | 关机 | DAC→0V, enable→LOW, clock 停 |
| `mp6_set_amplitude(handle, 80-250)` | 设振幅 | 80-250 → 0.35-1.3V → `mcp4726_set_voltage()` |
| `mp6_set_frequency(handle, 25-226)` | 设频率 | LEDC PWM 95% duty |

**参数范围**：

| 参数 | 范围 | 对应硬件 |
|------|------|---------|
| amplitude | 80-250 | → 0.35-1.3V DAC → MP-Driver 80-250V 压电输出 |
| frequency | 25-226 Hz | → GPIO27 PWM 95% duty → MP-Driver clock |
| enable | 0/1 | → GPIO14 HIGH=运行, LOW=关断 |

**MP-Driver 类型说明**（Kconfig 可选）：
- **Low MP-Driver**：低流量、低压应用
- **High MP-Driver**：高流量、高压应用
- **Standard MP-Driver**：通用（默认）

---

## 7. pid_controller.h/c — PID 控制器（待实现）

**职责**：实现经典 PID 闭环控制算法。

**控制逻辑**：
```
error = setpoint - measured_flow
output = Kp * error + Ki * ∫error·dt + Kd * d(error)/dt
```

**待实现功能**：
- PID 参数结构体 (Kp, Ki, Kd, 积分限幅, 输出限幅)
- `pid_init()` — 初始化 PID 参数
- `pid_compute()` — 输入当前流量，输出控制量
- `pid_reset()` — 重置积分累积
- 抗积分饱和 (anti-windup)
- 输出直接映射为 MP6 驱动参数

**数据流**：
```
SLF3S 流量读数 (µl/min) → pid_compute() → 控制量 → mp6_set_output()
```

---

## 8. serial_comm.h/c — 通信模块（待实现）

**职责**：与上位机 (PC/App) 通信，支持串口或 WiFi 两种模式。

**通信模式**（Kconfig 选择）：
- **Serial**：通过 USB-UART，默认模式
- **WiFi**：TCP socket 连接到指定服务器（IP + 端口）

**待实现功能**：
- 接收上位机指令（设定流量、修改 PID 参数、启停控制）
- 上传传感器数据（流量、压力、温度实时数据流）
- 通信协议定义（数据帧格式）

---

## 9. Kconfig.projbuild — 配置菜单定义

**职责**：定义 `idf.py menuconfig` 的菜单结构，编译时生成 `sdkconfig.h` 中的 `CONFIG_*` 宏。

**菜单结构**：

```
Custom Configuration
├── Sensor Configuration
│   ├── Enable Pressure Sensor     (bool, 默认 y)
│   ├── Enable Temperature Sensor  (bool, 默认 y)
│   ├── Enable Flow Sensor         (bool, 默认 y)
│   └── Flow Sensor Product Model  (choice: 0600F / 1300F)
├── Pump Driver Type               (choice: Low / High / Standard)
├── Communication Method           (choice: Serial / WiFi)
├── Pin Configuration
│   ├── I2C SDA Pin                (int, 默认 21)
│   ├── I2C SCL Pin                (int, 默认 22)
│   ├── MP Driver Enable Pin       (int, 默认 14)
│   └── MP Driver Clock Pin        (int, 默认 27)
└── WiFi Configuration             (仅 WiFi 模式可见)
    ├── WiFi SSID
    ├── WiFi Password
    ├── Service Address
    └── Service Port
```

---

## 模块间依赖关系

```
main.c
  ├── i2c_interface.h        (初始化 I2C)
  ├── mcp4726_dac.h          (DAC 测试，已验证)
  ├── mp6_driver.h           (泵控制，已实现)
  ├── SLF3S_flow_sensor.h    (读流量，已实现)
  ├── sensor_config.h        (读配置)
  ├── [pid_controller.h]     (待接入)
  └── [serial_comm.h]        (待接入)

mp6_driver.c
  ├── mcp4726_dac.h          (amplitude → DAC 电压)
  ├── sensor_config.h        (enable/clock 引脚)
  └── driver/ledc.h          (clock PWM)

mcp4726_dac.c
  ├── i2c_interface.h        (I2C 底层通信)
  └── driver/i2c.h           (i2c_cmd_link 逐字节发送)

SLF3S_flow_sensor.c
  ├── i2c_interface.h        (I2C 读写)
  └── sensor_config.h        (scale factor, 型号)

i2c_interface.c
  └── sensor_config.h        (SDA/SCL 引脚)

sensor_config.c
  └── sensor_config.h        (Kconfig 宏)
```

所有传感器驱动通过 `i2c_interface` 访问硬件，不直接调用 ESP-IDF I2C API。新增 I2C 传感器时应遵循相同模式。

**例外**：`mcp4726_dac.c` 使用底层 `i2c_cmd_link` API 而非 `i2c_interface_write()`，因为 MCP4726 需要数据字节 ACK check 关闭才能正常通信。

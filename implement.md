# 代码实现详解 Implementation Details

> Updated: 2026-02-20

## 文件结构

```
Firmware/main/
├── main.c                  # 应用入口、FreeRTOS 任务、命令 handler
├── main.h                  # 系统状态结构体、命令 handler 声明
├── sensor_config.h/c       # 系统配置（Kconfig 映射）
├── i2c_interface.h/c       # I2C 硬件抽象层
├── SLF3S_flow_sensor.h/c   # SLF3S 流量传感器驱动
├── mcp4726_dac.h/c         # MCP4726 DAC 驱动
├── mp6_driver.h/c          # MP6 微泵驱动
├── pid_controller.h/c      # PID 闭环控制器
├── serial_comm.h/c         # 串口通信协议解析
├── Kconfig.projbuild       # menuconfig 配置定义
└── MCP4726_DAC.md          # DAC 驱动开发记录

HostPC/
├── main_gui.py             # PyQt5 GUI 主窗口
├── microfluidic_api.py     # Python 串口 API 封装
└── requirements.txt        # Python 依赖
```

---

## 1. main.c / main.h — 应用入口与系统状态

**职责**：系统初始化、FreeRTOS 任务创建、命令 handler 实现。

### 系统状态结构体 (main.h)

```c
struct system_state {
    system_mode_t mode;             // MANUAL / PID
    bool pump_on;
    uint16_t amplitude;             // 80-250
    uint16_t frequency;             // 25-300 Hz

    float current_flow;             // 流量 (µl/min)
    float current_temperature;      // 传感器温度 (°C)
    uint16_t sensor_flags;          // 信号标志位 (air-in-line, high flow)

    float pid_target;
    uint32_t pid_elapsed_s;
    uint32_t pid_duration_s;

    bool stream_enabled;
    bool air_in_line;               // 边沿触发：气泡检测
    bool high_flow;                 // 边沿触发：流量超量程

    bool pump_available;            // 热插拔硬件检测
    bool sensor_available;
    bool pressure_available;

    mp6_handle_t pump;
    slf3s_handle_t flow_sensor;
    pid_controller_t pid;
};
```

### 启动流程

```
app_main()
    ├── memset(&g_state, 0)                 ← 零初始化
    ├── g_state.frequency = 100             ← 默认频率 100Hz
    ├── pid_init(&g_state.pid)              ← PID 默认增益
    ├── xSemaphoreCreateMutex()             ← 状态互斥锁
    ├── hw_init()                           ← 非致命硬件初始化
    │   ├── serial_comm_init()              ← UART0 (必须成功)
    │   ├── i2c_interface_init()            ← I2C 总线
    │   ├── i2c_interface_scan()            ← 枚举设备
    │   ├── probe 0x61 → mp6_init()         ← pump_available
    │   ├── probe 0x08 → slf3s_init/start() ← sensor_available
    │   └── probe 0x76                      ← pressure_available
    ├── xTaskCreate(serial_cmd_task)         ← 优先级 5
    └── xTaskCreate(sensor_read_task)        ← 优先级 6
```

**关键设计**：hw_init() 永远返回 ESP_OK，任何硬件缺失只记日志不阻塞启动。任务始终创建。

### 命令 Handler

每个 handler 由 serial_comm_process_cmd() 调用，内部加锁操作 g_state：

| Handler | 功能 |
|---------|------|
| `app_cmd_pump_on()` | mp6_start + pump_on=true |
| `app_cmd_pump_off()` | mp6_stop + pump_on=false |
| `app_cmd_set_amplitude()` | mp6_set_amplitude + 更新 state |
| `app_cmd_set_frequency()` | mp6_set_frequency + 更新 state |
| `app_cmd_pid_start()` | 启动泵（若未开）+ pid_start + 切 MODE_PID |
| `app_cmd_pid_stop()` | pid_stop + mp6_stop + 切 MODE_MANUAL |
| `app_cmd_pid_target()` | pid_set_target（运行中改目标） |
| `app_cmd_pid_tune()` | pid_set_gains |
| `app_cmd_status()` | serial_comm_send_status |
| `app_cmd_scan()` | i2c_interface_scan + serial_comm_send_scan |
| `app_cmd_set_calibration()` | 停测量 → 切标定液 → 重启测量 |

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

**职责**：驱动 Sensirion SLF3S 系列液体流量传感器，支持流量/温度/标志位读取。

**对应硬件**：Sensirion SLF3S-0600F 或 SLF3S-1300F，I2C 地址 0x08。

### 支持的传感器型号

| 型号 | 量程 | Scale Factor | 适用场景 |
|------|------|-------------|---------|
| SLF3S-0600F | 0 ~ 600 µl/min | 10.0 | 微流控芯片低流量 |
| SLF3S-1300F | 0 ~ 40 ml/min | 500.0 | 较大流量应用 |

### 数据结构

```c
typedef struct {
    float flow;             // 流量 (µl/min)
    float temperature;      // 温度 (°C)
    uint16_t flags;         // 信号标志位
} slf3s_measurement_t;
```

### 信号标志位

| Flag | Bit | 含义 |
|------|-----|------|
| `SLF3S_FLAG_AIR_IN_LINE` | bit 0 | 管路里有气泡，读数不准 |
| `SLF3S_FLAG_HIGH_FLOW` | bit 1 | 流量超出量程 |
| `SLF3S_FLAG_EXP_SMOOTHING` | bit 5 | 指数平滑已启用 |

### 读取模式

| 函数 | 读取字节 | 返回数据 |
|------|---------|---------|
| `slf3s_read_flow()` | 3 bytes | 仅流量 |
| `slf3s_read_measurement()` | 9 bytes | 流量 + 温度 + 标志位 |

sensor_read_task 使用 `slf3s_read_measurement()` 获取完整数据，包括温度和标志位检测。

### 两种启动模式

1. **`slf3s_start_measurement()`（完整模式）**：
   - 停止已有测量 → 读取产品编号 → 验证型号 → 软复位 → 开始测量
   - 会自动检测实际连接的传感器型号

2. **`slf3s_start_measurement_simple()`（简化模式）**：
   - 软复位 → 开始测量
   - 跳过产品识别，用于降级或标定切换后重启

### 标定模式

通过 `slf3s_set_calibration()` 切换标定液体：
- `0x08` — 水 (Water)，默认
- `0x15` — 异丙醇 (IPA)

串口命令 `CAL WATER` / `CAL IPA` 触发切换，过程：停测量 → 改标定 → 重启测量。

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

## 6. mp6_driver.h/c — MP6 微泵驱动

**职责**：控制 MP6 压电微泵，三路控制信号：amplitude（DAC）、frequency（PWM）、enable（GPIO）。

**对应硬件**：Bartels MP6 微泵 + MP-Driver 驱动板。

**控制链路**：

```
ESP32                    MP-Driver 板               MP6 泵
┌─────────────┐         ┌──────────┐         ┌──────────┐
│ MCP4726 DAC ├─ I2C ──→│ AMP      │─ P+/P- →│          │
│ GPIO27 PWM  ├─ 95% ──→│ CLK      │────────→│  压电泵  │
│ GPIO14      ├─ HIGH ──→│ Enable   │────────→│          │
└─────────────┘         └──────────┘         └──────────┘
```

- **AMP 引脚**：DAC 输出 0.35~1.3V → MP-Driver 内部升压 → P+/P- 输出高压交流 (~80-250Vpp)
- **CLK 引脚**：PWM 时钟信号，每个周期触发一次泵动作
- **P+/P-**：输出到 MP6 压电片两极的高压交流信号，频率与 CLK 一致
- MP-Driver 内部有升压开关电路（~1kHz 开关频率），示波器看 P+ 会看到高频纹波叠加在低频驱动包络上

**API**：

| 函数 | 说明 | 内部实现 |
|------|------|---------|
| `mp6_init(handle)` | 初始化 DAC + GPIO + PWM | 调 `mcp4726_init()`，配 GPIO14，初始化 LEDC |
| `mp6_start(handle)` | 启动泵 | enable→HIGH, clock 启动 |
| `mp6_stop(handle)` | 关机 | DAC→0V, enable→LOW, clock 停 |
| `mp6_set_amplitude(handle, 80-250)` | 设振幅 | 80-250 → 0.35-1.3V → `mcp4726_set_voltage()` |
| `mp6_set_frequency(handle, 25-300)` | 设频率 | LEDC PWM 95% duty |

**参数范围**：

| 参数 | 范围 | 对应硬件 |
|------|------|---------|
| amplitude | 80-250 | → 0.35-1.3V DAC → MP-Driver → P+/P- 高压交流输出 |
| frequency | 25-300 Hz | → GPIO27 PWM 95% duty → MP-Driver CLK |
| enable | 0/1 | → GPIO14 HIGH=运行, LOW=关断 |

**LEDC PWM 配置**：
- 定时器：`LEDC_TIMER_0`，低速模式
- 分辨率：10-bit（1024 级），最大频率 ~976Hz（远超 300Hz 上限）
- 占空比：固定 95%（`972/1024`），这是 MP-Driver 时钟信号要求
- 频率调节：`ledc_set_freq()` 改变硬件分频系数，自动生效

**MP-Driver 类型说明**（Kconfig 可选）：
- **Low MP-Driver**：低流量、低压应用
- **High MP-Driver**：高流量、高压应用（CLK 需 4× 目标频率）
- **Standard MP-Driver**：通用（默认，CLK = 输出频率 1:1）

---

## 7. pid_controller.h/c — PID 闭环控制器

**职责**：经典 PID 算法，输出映射为 MP6 泵振幅值，含抗积分饱和与偏差检测。

**配置常量**：

| 常量 | 值 | 含义 |
|------|-----|------|
| `PID_OUTPUT_MIN` | 80.0 | 最小输出（= MP6_AMPLITUDE_MIN） |
| `PID_OUTPUT_MAX` | 250.0 | 最大输出（= MP6_AMPLITUDE_MAX） |
| `PID_INTEGRAL_LIMIT` | 500.0 | 积分限幅（抗 windup） |
| `PID_FLOW_ERR_PERCENT` | 20% | 偏差报警阈值 |
| `PID_FLOW_ERR_TIME_S` | 10s | 持续偏差时间 |

**API**：

| 函数 | 说明 |
|------|------|
| `pid_init(pid)` | 初始化默认增益 |
| `pid_start(pid, target, duration)` | 启动 PID（duration=0 无限） |
| `pid_stop(pid)` | 停止并重置内部状态 |
| `pid_set_gains(pid, kp, ki, kd)` | 设置 PID 增益 |
| `pid_set_target(pid, target)` | 运行中更新目标 |
| `pid_compute(pid, measured, dt)` | 计算一次 PID 输出（每 100ms 调一次） |
| `pid_tick_second(pid)` | 秒计时器，检查 duration 到期 |
| `pid_check_flow_deviation(pid, measured, dt)` | 偏差累积检测 |

**控制流程**（在 sensor_read_task 中）：

```
每 100ms:
    flow = slf3s_read_measurement()
    output = pid_compute(flow, 0.1)     ← PID 计算
    mp6_set_amplitude(round(output))    ← 输出 → 泵振幅

    if pid_check_flow_deviation(flow):
        send EVENT FLOW_ERR             ← 偏差报警

    每 1s:
        if pid_tick_second():
            auto stop → EVENT PID_DONE  ← 时间到
```

PID 输出直接映射为泵振幅值（80-250）。频率在 PID START 时固定（默认 100Hz），PID 只调振幅。

---

## 8. serial_comm.h/c — 串口通信模块

**职责**：UART0 命令解析、数据流推送、事件通知。

**UART 配置**：115200 baud, 8N1, RX buffer 256B, TX 阻塞写。

### 命令解析 (serial_comm_process_cmd)

逐行解析 ASCII 命令，分发到 app_cmd_* handler：

| 命令 | 校验 | 调用 |
|------|------|------|
| `PUMP ON` | pump_available, !PID | app_cmd_pump_on |
| `PUMP OFF` | pump_available | app_cmd_pump_off (PID 模式下自动 PID STOP) |
| `AMP <80-250>` | pump_available, !PID, 范围检查 | app_cmd_set_amplitude |
| `FREQ <25-300>` | pump_available, !PID, 范围检查 | app_cmd_set_frequency |
| `PID START <target> <duration>` | pump + sensor available | app_cmd_pid_start |
| `PID STOP` | 无 | app_cmd_pid_stop |
| `PID TARGET <value>` | 必须 PID 模式 | app_cmd_pid_target |
| `PID TUNE <Kp> <Ki> <Kd>` | 参数解析 | app_cmd_pid_tune |
| `STATUS` | 无 | app_cmd_status |
| `SCAN` | 无 | app_cmd_scan |
| `STREAM ON/OFF` | 无 | 直接设 state->stream_enabled |
| `CAL <WATER\|IPA>` | sensor_available, !PID | app_cmd_set_calibration |

### 输出函数

| 函数 | 格式 |
|------|------|
| `send_ok()` | `OK\n` |
| `send_err(reason)` | `ERR <reason>\n` |
| `send_data(flow, temp)` | `D <flow> <temperature>\n` |
| `send_status(state)` | `S <mode> <pump> <amp> <freq> <flow> <target> <elapsed> <duration> <pump_hw> <sensor_hw> <pressure_hw> <temp>\n` |
| `send_scan(devices, count)` | `SCAN [addr1 addr2 ...]\n` |
| `send_event_pid_done()` | `EVENT PID_DONE\n` |
| `send_event_flow_err(target, actual)` | `EVENT FLOW_ERR <target> <actual>\n` |
| `send_event_air_in_line()` | `EVENT AIR_IN_LINE\n` |
| `send_event_high_flow()` | `EVENT HIGH_FLOW\n` |

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

## 10. Host PC — microfluidic_api.py

**职责**：Python 串口 API 封装，线程安全，后台 listener 线程。

**后台 listener 线程分类逻辑**：

```
收到一行
  ├── 匹配 ESP-IDF 日志正则 [EWIDV] (\d+) ... → 丢弃
  ├── 以 "D " 开头 → on_data(flow, temperature) 回调
  ├── == "EVENT PID_DONE" → on_pid_done() 回调
  ├── == "EVENT AIR_IN_LINE" → on_air_in_line() 回调
  ├── == "EVENT HIGH_FLOW" → on_high_flow() 回调
  ├── 以 "EVENT FLOW_ERR" 开头 → on_flow_err(target, actual) 回调
  ├── 以 OK / ERR / S / SCAN 开头 → 命令响应（唤醒等待线程）
  └── 其他 → 丢弃（启动垃圾数据）
```

**API 方法**：

| 方法 | 对应命令 |
|------|---------|
| `pump_on()` / `pump_off()` | PUMP ON/OFF |
| `set_amplitude(80-250)` | AMP |
| `set_frequency(25-300)` | FREQ |
| `pid_start(target, duration)` | PID START |
| `pid_stop()` | PID STOP |
| `pid_set_target(target)` | PID TARGET |
| `pid_tune(kp, ki, kd)` | PID TUNE |
| `stream_on()` / `stream_off()` | STREAM ON/OFF |
| `set_calibration("WATER"/"IPA")` | CAL |
| `get_status()` → SystemStatus | STATUS |
| `scan_i2c()` → List[int] | SCAN |

---

## 11. Host PC — main_gui.py (PyQt5)

**职责**：完整的图形化控制界面。

### UI 布局

```
┌─ Left Panel ──────────────┐  ┌─ Right Panel ──────────────────────┐
│ [Connection]              │  │ [Sensor Data]                       │
│  COM port, Connect/DC     │  │  Flow Rate 图表 / Pressure 图表 Tab │
│                           │  │  PID 目标虚线、暂停/恢复            │
│ [Mode]                    │  │                                     │
│  Manual / PID 切换        │  │ [Status Bar]                        │
│                           │  │  Mode|Pump|Flow|Temp|HW 状态        │
│ [Manual Control]          │  │                                     │
│  Amplitude slider 80-250  │  │ [Data Recording]                    │
│  Frequency slider 25-300  │  │  Record / Stop / Export CSV         │
│  PUMP ON / PUMP OFF       │  │                                     │
│                           │  │ [Alerts]                             │
│ [PID Control]             │  │  AIR_IN_LINE / HIGH_FLOW / FLOW_ERR │
│  Target, Duration, Gains  │  │                                     │
│  START / STOP             │  │ [Communication Log]                  │
│                           │  │  TX/RX 原始报文                      │
│ [Tools]                   │  │                                     │
│  I2C Scan, STATUS         │  │                                     │
│  Calibration: Water/IPA   │  │                                     │
└───────────────────────────┘  └─────────────────────────────────────┘
```

### 关键机制

- **SignalBridge**：API 回调（后台线程）通过 pyqtSignal 转到 GUI 线程
- **Debounce**：振幅/频率滑块 150ms 防抖后才发命令
- **定时器**：图表刷新 100ms、STATUS 轮询 1s、连接检查 2s、硬件热插拔扫描 5s
- **边沿触发 Alert**：AIR_IN_LINE 和 HIGH_FLOW 只在状态变化时弹一次
- **数据记录**：录制 (timestamp, flow, temperature) 元组，导出为 CSV

---

## 模块间依赖关系

```
main.c
  ├── main.h               (系统状态定义)
  ├── i2c_interface.h       (初始化 I2C)
  ├── mcp4726_dac.h         (DAC 测试)
  ├── mp6_driver.h          (泵控制)
  ├── SLF3S_flow_sensor.h   (读流量/温度/标志)
  ├── sensor_config.h       (读配置)
  ├── pid_controller.h      (PID 计算)
  └── serial_comm.h         (命令解析、数据推送)

serial_comm.c
  ├── main.h                (访问完整 system_state)
  ├── mp6_driver.h          (范围常量 MP6_FREQ_MIN/MAX)
  └── driver/uart.h         (UART I/O)

mp6_driver.c
  ├── mcp4726_dac.h         (amplitude → DAC 电压)
  ├── sensor_config.h       (enable/clock 引脚)
  └── driver/ledc.h         (clock PWM)

mcp4726_dac.c
  ├── i2c_interface.h       (I2C 底层通信)
  └── driver/i2c.h          (i2c_cmd_link 逐字节发送)

SLF3S_flow_sensor.c
  ├── i2c_interface.h       (I2C 读写)
  └── sensor_config.h       (scale factor, 型号)

pid_controller.c
  └── pid_controller.h      (纯算法，无硬件依赖)

i2c_interface.c
  └── sensor_config.h       (SDA/SCL 引脚)
```

所有传感器驱动通过 `i2c_interface` 访问硬件，不直接调用 ESP-IDF I2C API。新增 I2C 传感器时应遵循相同模式。

**例外**：`mcp4726_dac.c` 使用底层 `i2c_cmd_link` API 而非 `i2c_interface_write()`，因为 MCP4726 需要数据字节 ACK check 关闭。

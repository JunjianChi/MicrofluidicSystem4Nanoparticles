# 微流控系统控制框架 Overview

> Firmware V3.0.0 | Host PC GUI V1.0.0 | Updated: 2026-02-20

## 系统目标

基于 ESP32 的闭环微流控控制系统。核心任务是精确控制微通道中的液体流量：通过流量传感器实时测量，PID 控制器计算误差，MP6 微泵执行驱动，形成完整的闭环控制回路。上位机 GUI 提供手动控制、PID 模式、实时流量图表和数据记录功能。

## 系统框图

```
  ┌─────────────────────┐          UART (115200, 8N1)
  │    Host PC (GUI)    │◄──────────────────────────────────┐
  │  main_gui.py        │                                   │
  │  microfluidic_api.py│                                   │
  └─────────────────────┘                                   │
                                                            │
                    ┌───────────────────────────────────────────────────┐
                    │                    ESP32 主控                      │
                    │                                                   │
                    │   ┌──────────────┐       ┌──────────────┐         │
                    │   │serial_cmd_task│       │sensor_read_  │         │
                    │   │ (优先级 5)    │       │task (优先级6) │         │
                    │   │ 解析PC命令    │       │ 10Hz采样     │         │
                    │   └──────┬───────┘       │ PID计算      │         │
                    │          │               │ 数据流推送    │         │
                    │          ▼               └──────┬───────┘         │
                    │   ┌──────────────┐              │                 │
                    │   │ serial_comm  │              │                 │
                    │   │ (协议解析)    │              │                 │
                    │   └──────────────┘              │                 │
                    │                                 │                 │
                    │   ┌─────────────────────────────┘                 │
                    │   │                                               │
                    │   │    g_state (mutex 保护)                       │
                    │   │    ├─ mode: MANUAL / PID                     │
                    │   │    ├─ pump_on, amplitude, frequency          │
                    │   │    ├─ current_flow, pid_target               │
                    │   │    └─ stream_enabled                         │
                    │   │                                               │
                    │   ├──────────────────┬───────────────┐            │
                    │   ▼                  ▼               ▼            │
                    │ ┌────────────┐ ┌──────────┐  ┌──────────────┐    │
                    │ │pid_control │ │mp6_driver │  │SLF3S_flow    │    │
                    │ │(PID计算)   │ │(泵驱动)   │  │sensor(传感器)│    │
                    │ └────────────┘ └─────┬────┘  └──────┬───────┘    │
                    │                  GPIO │ PWM     I2C  │            │
                    └──────────────────┼───┼──────────────┼────────────┘
                                       │   │              │
                                  ┌────┘   │    ┌─────────┘
                                  ▼        ▼    ▼
                            ┌──────────┐ ┌──────────┐
                            │ MCP4726  │ │ SLF3S    │
                            │ DAC(振幅) │ │ 流量传感器│
                            │ (0x61)   │ │ (0x08)   │
                            └──────────┘ └──────────┘
                              MP6 微泵
                           (Enable+Clock)
```

## FreeRTOS 任务架构

系统运行 2 个 FreeRTOS 任务，通过 mutex 共享 `g_state`：

### serial_cmd_task (优先级 5)

```
while(1) {
    serial_comm_read_line()         ← 阻塞等待 UART 命令
    serial_comm_process_cmd()       ← 解析 → 调用 handler → 回复 OK/ERR/S/SCAN
}
```

- 阻塞式：无命令时不占 CPU
- 每个 handler 内部 `STATE_LOCK()` / `STATE_UNLOCK()` 保护共享状态
- PID 模式下拒绝手动控制命令 (回 `ERR PID_ACTIVE`)

### sensor_read_task (优先级 6，更高)

```
每 100ms (vTaskDelayUntil):
    0. 每 5s: 热插拔 re-probe (pump/sensor/pressure)
    1. slf3s_read_measurement() → flow, temperature, flags
    2. STATE_LOCK()
    3. g_state.current_flow = flow
       g_state.current_temperature = temperature
       边沿检测: AIR_IN_LINE / HIGH_FLOW → send EVENT
    4. if MODE_PID:
        ├─ pid_compute(flow) → output
        ├─ mp6_set_amplitude(output)
        ├─ 偏差检测 → EVENT FLOW_ERR
        └─ 每秒: pid_tick_second() → 到时自动停止 → EVENT PID_DONE
    5. if stream_enabled: send "D <flow> <temp>"
       STATE_UNLOCK()
```

- 优先级更高，可以抢占 serial_cmd_task，保证 10Hz 采样精度
- PID 计算和传感器读取在同一 task，保证实时性
- Duration 到期后自动停止 PID → 回到 MANUAL 模式
- 传感器标志位边沿触发：只在状态从 0→1 变化时发一次事件

## 串口协议 (UART0, 115200 8N1)

### PC → ESP32 命令

| 命令 | 说明 | 响应 |
|------|------|------|
| `PUMP ON` | 启动泵 (MANUAL模式) | `OK` / `ERR PID_ACTIVE` |
| `PUMP OFF` | 停止泵 (PID模式下自动PID STOP) | `OK` |
| `AMP <80-250>` | 设置振幅 | `OK` / `ERR PID_ACTIVE` |
| `FREQ <25-300>` | 设置频率 (Hz) | `OK` / `ERR PID_ACTIVE` |
| `PID START <target> <duration>` | 启动PID (duration=0无限) | `OK` |
| `PID STOP` | 停止PID，回MANUAL | `OK` |
| `PID TARGET <value>` | 运行中改目标 | `OK` / `ERR NOT_PID_MODE` |
| `PID TUNE <Kp> <Ki> <Kd>` | 设置PID增益 | `OK` |
| `STATUS` | 查询状态 | `S <mode> <pump> <amp> <freq> <flow> <target> <elapsed> <duration> <pump_hw> <sensor_hw> <pressure_hw> <temp>` |
| `SCAN` | I2C扫描 | `SCAN [addr1 addr2 ...]` |
| `STREAM ON/OFF` | 开/关数据流 | `OK` |
| `CAL <WATER\|IPA>` | 切换标定液体 | `OK` / `ERR SENSOR_UNAVAIL` |

### ESP32 → PC 异步消息

| 消息 | 触发条件 |
|------|---------|
| `D <flow> <temperature>` | stream开启时，10Hz推送流量和温度 |
| `EVENT PID_DONE` | PID duration到期自动停止 |
| `EVENT FLOW_ERR <target> <actual>` | 流量持续偏离目标 |
| `EVENT AIR_IN_LINE` | 管路气泡检测（边沿触发） |
| `EVENT HIGH_FLOW` | 流量超出传感器量程（边沿触发） |

## 闭环控制逻辑

```
设定流量(Setpoint) ──►(+)──► PID 控制器 ──► MP6 泵振幅 (80-250)
                       │ -                        │
                       │                          ▼
                       │                     微流控通道
                       │                          │
                       └──── SLF3S 流量测量 ◄──────┘
                              (10Hz采样)
```

PID 输出直接映射为泵振幅值。频率在 PID START 时固定（默认100Hz），PID 只调振幅。

## Host PC 软件

### microfluidic_api.py

Python API 封装，线程安全。后台 listener 线程持续读取 UART：
- `D <flow> <temp>` → `on_data(flow, temperature)` 回调
- `EVENT PID_DONE` → `on_pid_done` 回调
- `EVENT AIR_IN_LINE` → `on_air_in_line` 回调
- `EVENT HIGH_FLOW` → `on_high_flow` 回调
- `EVENT FLOW_ERR` → `on_flow_err(target, actual)` 回调
- `OK/ERR/S/SCAN` → 命令响应
- ESP-IDF 日志行 (`[EWIDV] (timestamp)`) → 丢弃
- 其他 → 丢弃（启动垃圾数据）

额外 API：
- `set_calibration("WATER"/"IPA")` — 切换流量传感器标定液体

### main_gui.py (PyQt5)

| 功能 | 说明 |
|------|------|
| Connection | COM端口选择、连接/断开、自动检测断连 |
| Mode切换 | Manual / PID 模式选择，QStackedWidget 只显示当前模式 |
| Manual模式 | 振幅(80-250)/频率(25-300)滑块、PUMP ON/OFF、150ms防抖即时更新 |
| PID模式 | 目标流量、持续时间、PID增益、START/STOP |
| 传感器图表 | pyqtgraph实时曲线（Flow Rate / Pressure Tab），PID目标虚线，暂停/恢复 |
| Status Bar | Mode、Pump、Flow、Temp、Driver/Sensor/Pressure HW 状态 |
| Alerts | AIR_IN_LINE / HIGH_FLOW / FLOW_ERR 边沿触发告警 |
| 数据记录 | 录制(timestamp, flow, temperature)/停止/导出CSV |
| Tools | I2C Scan, STATUS 查询, Calibration (Water/IPA) 切换 |
| 硬件热插拔 | 每5s I2C SCAN，自动检测设备连接/断开，UI 实时更新 |
| Communication Log | TX/RX 原始报文日志 |

## 硬件组成

| 硬件 | 型号/说明 | 接口 | I2C 地址 | 状态 |
|------|----------|------|---------|------|
| 主控 | ESP32 | - | - | 已实现 |
| 流量传感器 | Sensirion SLF3S-0600F / 1300F | I2C | 0x08 | 已实现 |
| 微泵 | MP6 微型压电泵 | GPIO (Enable + Clock PWM) | - | 已实现 |
| DAC | MCP4726 (泵振幅控制) | I2C | 0x61 | 已实现 |
| 压力传感器 | 待定 (预留 MS5837 类) | I2C | 0x76 | 待实现 |
| 温度传感器 | 待定 (预留 TMP117 类) | I2C | 0x48 | 待实现 |

## 默认引脚分配

| 功能 | GPIO | 说明 |
|------|------|------|
| I2C SDA | GPIO 21 | I2C 数据线，外接上拉电阻 |
| I2C SCL | GPIO 22 | I2C 时钟线，外接上拉电阻 |
| MP6 Enable | GPIO 14 | 微泵使能信号 |
| MP6 Clock | GPIO 27 | 微泵时钟/PWM 信号 |

所有引脚可通过 `idf.py menuconfig` → "Custom Configuration" → "Pin Configuration" 修改。

## 软件分层

```
┌──────────────────────────────────────────────────┐
│  Host PC: main_gui.py (PyQt5) + microfluidic_api│  上位机
├──────────────────────────────────────────────────┤
│               UART 协议 (ASCII)                   │  通信层
├──────────────────────────────────────────────────┤
│          Application (main.c)                    │  应用层：RTOS任务、命令分发
│   serial_cmd_task  |  sensor_read_task           │
├──────────────────────────────────────────────────┤
│   pid_controller   │  serial_comm                │  功能层：PID控制、协议解析
├──────────────────────────────────────────────────┤
│ SLF3S_flow_sensor  │  mp6_driver  │  mcp4726_dac │  驱动层：传感器/执行器
├──────────────────────────────────────────────────┤
│              i2c_interface                        │  HAL层：I2C 硬件抽象
├──────────────────────────────────────────────────┤
│          sensor_config (Kconfig)                 │  配置层：引脚、传感器参数
├──────────────────────────────────────────────────┤
│            ESP-IDF / FreeRTOS                    │  平台层：RTOS、驱动框架
└──────────────────────────────────────────────────┘
```

## 开发状态

- [x] I2C 硬件抽象层
- [x] SLF3S 流量传感器驱动（含自动检测型号、温度读取、标志位检测）
- [x] MCP4726 DAC 驱动（校准 VDD=4.734V）
- [x] MP6 微泵驱动 (Enable + Clock PWM 25-300Hz + DAC 振幅控制)
- [x] PID 闭环控制器（抗 windup、偏差检测、定时自动停止）
- [x] 串口通信协议 (ASCII 行终止, 12 条命令 + 5 种异步事件)
- [x] FreeRTOS 双任务架构 (serial_cmd_task + sensor_read_task)
- [x] Kconfig 配置系统
- [x] 硬件热插拔检测（固件 5s re-probe + GUI 5s I2C SCAN）
- [x] 流量传感器标定切换 (Water/IPA)
- [x] 传感器标志位检测 (AIR_IN_LINE, HIGH_FLOW 边沿触发事件)
- [x] Host PC Python API (microfluidic_api.py, 线程安全)
- [x] Host PC PyQt5 GUI (Manual/PID模式、实时图表、数据记录、Alerts、热插拔)
- [ ] 压力传感器驱动
- [ ] WiFi 通信模式

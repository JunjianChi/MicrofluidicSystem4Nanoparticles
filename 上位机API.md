# 上位机串口通信 API V1.0.0

## 系统架构

```
┌──────────────────────────────┐       UART 115200 8N1       ┌──────────────────────────────┐
│         上位机 (Python)       │ ◄──────── \n 终止 ────────► │         ESP32 Firmware        │
│                              │                              │                              │
│  main_gui.py (PyQt5 GUI)    │   >> PUMP ON                 │  serial_comm.c               │
│       │                      │   << OK                      │    serial_comm_process_cmd() │
│  microfluidic_api.py         │   >> AMP 200                 │       │                      │
│    MicrofluidicController    │   << OK                      │       ├─► mp6_driver          │
│    ├ pump_on/off()           │   >> STATUS                  │       │    ├ amplitude (DAC)   │
│    ├ set_amplitude/freq()    │   << S MANUAL 1 200 100 ...  │       │    ├ frequency (LEDC)  │
│    ├ pid_start/stop/tune()   │   << D 12.50                 │       │    └ enable (GPIO)     │
│    ├ stream_on/off()         │   << EVENT PID_DONE          │       ├─► SLF3S_sensor         │
│    ├ get_status()            │                              │       │    └ flow_read()       │
│    └ scan_i2c()              │                              │       └─► pid_controller       │
│                              │                              │            ├ setpoint          │
│  Callbacks:                  │                              │            └ compute()         │
│    on_data(flow)             │                              │                              │
│    on_pid_done()             │                              │  sensor_read_task (10Hz)      │
│    on_flow_err(tgt, act)     │                              │    └─► serial_comm_send_data()│
└──────────────────────────────┘                              └──────────────────────────────┘
```

## 通信配置

| 参数 | 值 |
|------|-----|
| 接口 | UART0 (USB 串口) |
| 波特率 | 115200 |
| 数据位 | 8 |
| 校验 | None |
| 停止位 | 1 |
| 编码 | ASCII |
| 命令终止符 | `\n` |
| 命令最大长度 | 128 字节 |
| 默认响应超时 | 2 秒 |

## 工作模式

系统有两种互斥的工作模式：

| 模式 | 标识 | 说明 |
|------|------|------|
| **MANUAL** | `MODE_MANUAL` | 手动控制泵参数 (amplitude/frequency)，上位机直接设定 |
| **PID** | `MODE_PID` | 闭环 PID 控制，固件自动调节 amplitude 追踪目标流量 |

- 上电默认 MANUAL
- 进入 PID 模式: `PID START`
- 退出 PID 模式: `PID STOP` 或 `PUMP OFF` 或 PID duration 到期
- PID 模式下 `AMP`, `FREQ`, `PUMP ON` 命令被拒绝 (返回 `ERR PID_ACTIVE`)

## PC → ESP32 命令

### 泵控制 (Manual Mode)

| 命令 | 格式 | 响应 | 说明 |
|------|------|------|------|
| 泵启动 | `PUMP ON\n` | `OK\n` | enable→HIGH, clock 启动。PID 模式下返回 ERR |
| 泵停止 | `PUMP OFF\n` | `OK\n` | DAC→0V, enable→LOW, clock 停。若在 PID 模式则同时退出 PID |
| 设置振幅 | `AMP <value>\n` | `OK\n` | value: 80-250。PID 模式下返回 ERR |
| 设置频率 | `FREQ <value>\n` | `OK\n` | value: 25-226 Hz。PID 模式下返回 ERR |

### PID 控制

| 命令 | 格式 | 响应 | 说明 |
|------|------|------|------|
| PID 启动 | `PID START <target> <duration>\n` | `OK\n` | target: 目标流量 (ul/min, >0)；duration: 秒 (0=无限) |
| PID 停止 | `PID STOP\n` | `OK\n` | 停止 PID，回到 MANUAL，泵停止 |
| 修改目标 | `PID TARGET <value>\n` | `OK\n` | 运行中修改目标流量。仅 PID 模式有效 |
| 设置增益 | `PID TUNE <Kp> <Ki> <Kd>\n` | `OK\n` | 设置 PID 三个增益参数。任何模式下均可调用 |

### 数据流控制

| 命令 | 格式 | 响应 | 说明 |
|------|------|------|------|
| 开启数据流 | `STREAM ON\n` | `OK\n` | 启动 10Hz 流量数据上报 (`D <flow>`) |
| 关闭数据流 | `STREAM OFF\n` | `OK\n` | 停止流量数据上报 |

### 查询命令

| 命令 | 格式 | 响应 | 说明 |
|------|------|------|------|
| 查询状态 | `STATUS\n` | `S ...` (见下) | 返回完整系统状态 |
| I2C 扫描 | `SCAN\n` | `SCAN ...` (见下) | 扫描 I2C 总线，返回设备地址列表 |

## ESP32 → PC 响应

### 命令响应 (同步)

每条命令发送后，ESP32 返回一行响应：

| 类型 | 格式 | 示例 |
|------|------|------|
| 成功 | `OK\n` | `OK\n` |
| 失败 | `ERR <reason>\n` | `ERR INVALID_ARG\n`, `ERR PID_ACTIVE\n` |
| 状态 | `S <mode> <pump> <amp> <freq> <flow> <target> <elapsed> <duration>\n` | `S MANUAL 1 200 100 12.50 0.00 0 0\n` |
| I2C 扫描 | `SCAN [<addr1> <addr2> ...]\n` | `SCAN 08 61\n` 或 `SCAN\n` (无设备) |

### STATUS 响应字段

`S <mode> <pump> <amp> <freq> <flow> <target> <elapsed> <duration>`

| 字段 | 位置 | 类型 | 说明 |
|------|------|------|------|
| `S` | 0 | 固定 | 状态响应标识 |
| mode | 1 | string | `MANUAL` 或 `PID` |
| pump | 2 | 0/1 | 泵状态 (0=OFF, 1=ON) |
| amp | 3 | int | 当前 amplitude (80-250) |
| freq | 4 | int | 当前频率 (25-226 Hz) |
| flow | 5 | float | 当前流量读数 (ul/min) |
| target | 6 | float | PID 目标流量 (MANUAL 模式下为 0.00) |
| elapsed | 7 | int | PID 已运行时间 (秒) |
| duration | 8 | int | PID 设定时长 (秒, 0=无限) |

### SCAN 响应

`SCAN [<addr1> <addr2> ...]`

- 地址为两位十六进制 (大写)，空格分隔
- 无设备时只返回 `SCAN`
- 示例: `SCAN 08 61` → 发现 0x08 和 0x61 两个设备

### 异步消息 (ESP32 主动上报)

这些消息不由命令触发，ESP32 在特定条件下主动发送：

| 类型 | 格式 | 触发条件 |
|------|------|---------|
| 数据流 | `D <flow>\n` | `STREAM ON` 后，10Hz 持续上报 |
| PID 完成 | `EVENT PID_DONE\n` | PID duration 到期，自动停止 |
| 流量异常 | `EVENT FLOW_ERR <target> <actual>\n` | 实际流量持续偏离目标值 |

## 消息分类 (上位机解析逻辑)

上位机后台监听线程按以下优先级分类收到的每一行：

```
收到一行
  ├─ 以 "D " 开头        → 数据流，触发 on_data(flow) 回调
  ├─ == "EVENT PID_DONE"  → PID 完成事件，触发 on_pid_done() 回调
  ├─ 以 "EVENT FLOW_ERR" 开头 → 流量异常事件，触发 on_flow_err(target, actual) 回调
  └─ 其他                 → 命令响应 (OK / ERR / S ... / SCAN ...)，交给等待中的命令
```

## 硬件控制链路

```
PC 发送 AMP 200
  → serial_comm_process_cmd() 解析
    → mp6_set_amplitude(&pump, 200)
      → voltage = 0.35 + (200-80) * 0.95/170 = 1.02V
        → mcp4726_set_voltage(1.02)
          → DAC I2C 写入
            → MP-Driver amplitude 输入
              → 泵输出 ~200V 压电驱动

PC 发送 FREQ 100
  → serial_comm_process_cmd() 解析
    → mp6_set_frequency(&pump, 100)
      → LEDC PWM 100Hz 95% duty (GPIO27)
        → MP-Driver clock 输入

PC 发送 PUMP OFF
  → serial_comm_process_cmd() 解析
    → mp6_stop(&pump)
      → mcp4726_set_voltage(0)    → DAC 输出 0V (amplitude 归零)
      → gpio_set_level(14, LOW)   → MP-Driver enable 拉低
      → clock PWM duty = 0        → 时钟停止

PC 发送 PID START 15.00 600
  → serial_comm_process_cmd() 解析
    → 模式切换 MANUAL → PID
    → pid_controller_start(target=15.00, duration=600)
    → mp6_start(&pump)
    → sensor_read_task 中: pid_compute() → mp6_set_amplitude() 自动调节
    → 600 秒后 → serial_comm_send_event_pid_done()
```

## Python 上位机使用示例

### 方式一: API 库直接调用

```python
from microfluidic_api import MicrofluidicController

def on_data(flow):
    print(f"Flow: {flow:.2f} ul/min")

def on_pid_done():
    print("[EVENT] PID_DONE")

def on_flow_err(target, actual):
    print(f"[EVENT] FLOW_ERR target={target:.2f} actual={actual:.2f}")

with MicrofluidicController("COM3") as ctrl:
    ctrl.on_data = on_data
    ctrl.on_pid_done = on_pid_done
    ctrl.on_flow_err = on_flow_err

    # 查询状态
    status = ctrl.get_status()
    # → SystemStatus(mode='MANUAL', pump_on=False, amplitude=200, ...)

    # I2C 扫描
    devices = ctrl.scan_i2c()
    # → [0x08, 0x61]

    # 手动模式
    ctrl.set_amplitude(200)
    ctrl.set_frequency(100)
    ctrl.pump_on()
    ctrl.stream_on()      # 开启 10Hz 数据流
    time.sleep(5)          # on_data 回调接收数据
    ctrl.stream_off()
    ctrl.pump_off()

    # PID 模式
    ctrl.pid_tune(1.0, 0.1, 0.01)
    ctrl.pid_start(target_flow=15.0, duration_s=600)
    ctrl.stream_on()
    time.sleep(600)        # on_data + on_pid_done 回调
    # PID 到期自动停止，或手动:
    ctrl.pid_stop()
```

### 方式二: GUI 上位机

```bash
cd HostPC
pip install -r requirements.txt
python main_gui.py
```

## 文件结构

```
HostPC/
├── microfluidic_api.py    # 串口通信 API 层 (MicrofluidicController)
├── main_gui.py            # PyQt5 GUI 主窗口
└── requirements.txt       # Python 依赖 (PyQt5, pyqtgraph, pyserial)

Firmware/main/
├── serial_comm.h/.c       # UART 命令解析 + 数据/事件上报
├── mp6_driver.h/.c        # 微泵驱动 (amplitude/frequency/enable)
├── mcp4726_dac.h/.c       # DAC 驱动 (I2C, amplitude→电压)
├── SLF3S_flow_sensor.h/.c # 流量传感器驱动 (I2C, 10Hz 读取)
├── pid_controller.h/.c    # PID 闭环控制器
└── main.c                 # FreeRTOS 任务编排
```

## 固件接口函数 (serial_comm.h)

| 函数 | 说明 |
|------|------|
| `serial_comm_init()` | 初始化 UART0 |
| `serial_comm_read_line(buf, max_len)` | 阻塞读取一行命令 |
| `serial_comm_send(fmt, ...)` | 格式化发送 |
| `serial_comm_send_ok()` | 发送 `OK\n` |
| `serial_comm_send_err(reason)` | 发送 `ERR <reason>\n` |
| `serial_comm_send_data(flow)` | 发送 `D <flow>\n` |
| `serial_comm_send_status(state)` | 发送 `S <mode> <pump> <amp> <freq> <flow> <target> <elapsed> <duration>\n` |
| `serial_comm_send_scan(devices, count)` | 发送 `SCAN <addr1> <addr2> ...\n` |
| `serial_comm_send_event_pid_done()` | 发送 `EVENT PID_DONE\n` |
| `serial_comm_send_event_flow_err(target, actual)` | 发送 `EVENT FLOW_ERR <target> <actual>\n` |
| `serial_comm_process_cmd(state, cmd_line)` | 解析并执行一条命令 |

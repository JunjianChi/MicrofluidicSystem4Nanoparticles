# 上位机串口通信 API 设计

## 系统架构

```
上位机 (Python)  ←── UART 115200 ──→  ESP32 main.c
                                         │
                                   serial_comm.c   ← 解析命令/上报数据
                                         │
                          ┌──────────────┼──────────────┐
                          │              │              │
                    mp6_driver      SLF3S_sensor   pid_controller
                     ├ amplitude    ├ flow_read()   ├ setpoint
                     │  └→ mcp4726_dac              └ compute()
                     ├ frequency
                     │  └→ LEDC PWM (GPIO27)
                     └ enable
                        └→ GPIO14
```

## main.c 对接设计

```c
static mp6_handle_t pump;
static slf3s_handle_t flow_sensor;

void app_main(void)
{
    /* 1. 硬件初始化 */
    i2c_interface_init();
    mp6_init(&pump);
    slf3s_init(&flow_sensor);
    slf3s_start_measurement(&flow_sensor);

    /* 2. 启动 FreeRTOS 任务 */
    xTaskCreate(serial_cmd_task,  ...);   // 解析上位机命令
    xTaskCreate(sensor_read_task, ...);   // 传感器周期读取 + 数据上报

    /* 3. app_main 返回, 任务接管 */
}
```

### 命令处理任务 (serial_cmd_task)

```c
void serial_cmd_task(void *arg)
{
    char line[64];
    while (1) {
        // 从 UART 读一行
        serial_read_line(line, sizeof(line));

        if (strcmp(line, "PUMP ON") == 0) {
            mp6_start(&pump);
            serial_send("OK\n");
        }
        else if (strcmp(line, "PUMP OFF") == 0) {
            mp6_stop(&pump);           // DAC→0V, enable→LOW, clock停
            serial_send("OK\n");
        }
        else if (sscanf(line, "AMP %hu", &val) == 1) {
            mp6_set_amplitude(&pump, val);
            serial_send("OK\n");
        }
        else if (sscanf(line, "FREQ %hu", &val) == 1) {
            mp6_set_frequency(&pump, val);
            serial_send("OK\n");
        }
        // ... TFLOW, STATUS 等
    }
}
```

### 传感器读取任务 (sensor_read_task)

```c
void sensor_read_task(void *arg)
{
    while (1) {
        float flow = 0;
        if (slf3s_read_flow(&flow_sensor, &flow) == ESP_OK) {
            // 泵运行时上报数据给上位机
            if (pump.is_running) {
                serial_printf("D %.2f\n", flow);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz
    }
}
```

## 通信配置

- 接口: UART0 (USB 串口)
- 波特率: 115200
- 数据格式: 8N1
- 命令终止符: `\n`

## PC → ESP32 命令

| 命令 | 格式 | 参数范围 | 说明 |
|------|------|---------|------|
| 泵启动 | `PUMP ON` | - | enable→HIGH, clock启动 |
| 泵关机 | `PUMP OFF` | - | DAC→0V, enable→LOW, clock停 |
| 振幅 | `AMP <value>` | 80-250 | 设置 amplitude → DAC → MP-Driver |
| 频率 | `FREQ <value>` | 25-226 Hz | 设置 clock PWM 频率 |
| 目标流量 | `TFLOW <value>` | µl/min | PID 目标值 (待实现) |
| 查询状态 | `STATUS` | - | 请求完整状态 |

## ESP32 → PC 响应

| 类型 | 格式 | 示例 | 说明 |
|------|------|------|------|
| 成功 | `OK\n` | `OK\n` | 命令执行成功 |
| 失败 | `ERR <reason>\n` | `ERR INVALID_ARG\n` | 命令执行失败 |
| 状态 | `S <pump> <amp> <freq> <flow>\n` | `S 1 200 100 12.50\n` | STATUS 回复 |
| 数据 | `D <flow>\n` | `D 12.50\n` | 传感器数据 (泵开启时 10Hz) |

### 状态字段说明

`S <pump> <amp> <freq> <flow>`

| 字段 | 类型 | 说明 |
|------|------|------|
| pump | 0/1 | 泵状态 (0=OFF, 1=ON) |
| amp | int | 当前 amplitude (80-250) |
| freq | int | 当前频率 (25-226 Hz) |
| flow | float | 当前流量读数 (µl/min) |

## 硬件控制链路

```
PC 发送 AMP 200
  → serial_cmd_task 解析
    → mp6_set_amplitude(&pump, 200)
      → voltage = 0.35 + (200-80) * 0.95/170 = 1.02V
        → mcp4726_set_voltage(1.02)
          → DAC I2C 写入
            → MP-Driver amplitude 输入
              → 泵输出 ~200V 压电驱动

PC 发送 FREQ 100
  → serial_cmd_task 解析
    → mp6_set_frequency(&pump, 100)
      → LEDC PWM 100Hz 95% duty (GPIO27)
        → MP-Driver clock 输入

PC 发送 PUMP OFF
  → serial_cmd_task 解析
    → mp6_stop(&pump)
      → mcp4726_set_voltage(0)    → DAC 输出 0V (amplitude 归零)
      → gpio_set_level(14, LOW)   → MP-Driver enable 拉低
      → clock PWM duty = 0        → 时钟停止
```

## Python 上位机示例

```python
import serial
import time

ser = serial.Serial('COM3', 115200, timeout=1)

# 设置参数 (泵未启动时也可以预设)
ser.write(b'AMP 200\n')
print(ser.readline())  # OK

ser.write(b'FREQ 100\n')
print(ser.readline())  # OK

# 启动泵
ser.write(b'PUMP ON\n')
print(ser.readline())  # OK

# 读取流量数据 (泵开启后 10Hz 上报)
for i in range(50):
    line = ser.readline().decode().strip()
    if line.startswith('D '):
        flow = float(line.split()[1])
        print(f"Flow: {flow} ul/min")

# 运行中调节参数
ser.write(b'AMP 180\n')
print(ser.readline())  # OK

ser.write(b'FREQ 80\n')
print(ser.readline())  # OK

# 查询状态
ser.write(b'STATUS\n')
print(ser.readline())  # S 1 180 80 12.50

# 停止泵
ser.write(b'PUMP OFF\n')
print(ser.readline())  # OK

ser.close()
```

## 模块实现状态

| 功能 | 固件模块 | 调用链 | 状态 |
|------|---------|--------|------|
| PUMP ON | mp6_driver.c | `mp6_start()` → enable HIGH + clock start | 已实现 |
| PUMP OFF | mp6_driver.c | `mp6_stop()` → DAC 0V + enable LOW + clock stop | 已实现 |
| AMP | mp6_driver.c → mcp4726_dac.c | `mp6_set_amplitude()` → `mcp4726_set_voltage()` | 已实现 |
| FREQ | mp6_driver.c | `mp6_set_frequency()` → LEDC PWM | 已实现 |
| D (流量数据) | SLF3S_flow_sensor.c | `slf3s_read_flow()` | 已实现 |
| 命令解析 | serial_comm.c | UART 读行 → 字符串匹配 → 调用驱动 | **待实现** |
| STATUS | serial_comm.c | 读取 pump handle 状态 → 格式化输出 | **待实现** |
| TFLOW (PID) | pid_controller.c | `pid_compute()` → `mp6_set_amplitude()` | **待实现** |

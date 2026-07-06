# STM32F103ZE 执行端工程

本目录为比赛作品的 STM32 执行端代码，配合 RK3588 主程序 `x4.py` 使用。RK3588 负责视觉检测、MQTT 上报和上层决策，STM32 负责串口命令解析、舵机控制、继电器控制、传感器采集和 TFT 屏幕显示。

## 工程信息

- MCU：STM32F103ZE
- 开发环境：Keil uVision
- 工程文件：`Template.uvprojx`
- 主程序：`User/main.c`
- 串口协议解析：`Hardware/Communication.c`
- 舵机控制：`Hardware/Servo.c`、`Hardware/PWM.c`
- 串口驱动：`Hardware/Serial.c`
- 屏幕与触摸：`Hardware/tftlcd.c`、`Hardware/Display.c`、`Hardware/ui.c`、`Hardware/touch.c`
- 传感器：`Hardware/bsp_si7021.c`、`Hardware/bsp_sm5388.c`

## 目录说明

```text
firmware/stm32/
├── APP/                 # 应用层模块
├── Hardware/            # 串口、舵机、PWM、屏幕、传感器等硬件驱动
├── Libraries/           # CMSIS 与 STM32F10x 标准外设库
├── Public/              # 公共模块
├── System/              # 延时等系统基础模块
├── User/                # main.c 与中断配置
├── RTE/                 # Keil RTE 设备配置
├── DebugConfig/         # Keil 调试配置
└── Template.uvprojx     # Keil 工程文件
```

## 串口协议

STM32 通过串口接收 RK3588 发来的 JSON 指令，常用指令如下：

```json
{"servo1": 90, "servo2": 0, "servo3": 90, "servo4": 0}
```

```json
{"auto": 0}
{"auto": 1}
```

```json
{"switch": 0}
{"switch": 1}
```

```json
{"action": "home"}
{"action": "status"}
```

也保留传统字符串命令，如 `S1G1`、`S1HOME`、`ALLHOME` 等，具体处理逻辑见 `Hardware/Communication.c` 和 `Hardware/Servo.c`。

## 引脚与外设

- 串口：PA9 TX，PA10 RX
- 舵机：PA1、PA2、PA3、PB6、PB7、PB9
- 继电器：PB8
- 温湿度/风速风向等传感器：见 `User/main.c` 和 `Hardware/` 中对应驱动
- TFT 屏幕：FSMC 并口屏，相关驱动见 `Hardware/tftlcd.c`

## 构建说明

1. 使用 Keil uVision 打开 `Template.uvprojx`。
2. 选择目标 `STM32F103ZE`。
3. 编译并下载到 STM32F103ZE 开发板。

`Obj/`、`Debug/`、`Release/` 等编译产物不提交到仓库。


#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include <stdint.h>

// JSON命令处理
uint8_t ProcessJSONCommand(char* json_str);

// 串口命令处理
void ProcessSerialCommand(void);

// JSON数据发送函数
void SendServoAnglesJSON4_Safe(float angle1, float angle2, float angle3, float angle4);
void SendTempHumidityJSON_Safe(float temperature, float humidity);
void SendWindDataJSON_Safe(float wind_speed, const char* wind_direction);
void SendModeJSON_Safe(void);
void SendSystemStatusJSON_Safe(const char* message);

// 外部变量声明
extern uint8_t moshiqiehuan;      // 舵机1自动模式标志(0=自动,1=手动)
extern uint8_t servo1stop;        // 舵机1停止标志
extern uint8_t relay_switch;      // 继电器开关
extern uint8_t box_value;         // box状态

// box舵机平滑运动更新
void BoxServo_Update(void);
// box是否已运动到位（供设备控制页显示“执行中/已到位”）
uint8_t BoxServo_InPosition(void);

// ============================================================
// 运行状态跟踪（供触摸 UI 各页读取；均在本模块内维护）
// ============================================================

// ---- 串口命令跟踪 ----
extern char     g_last_cmd[40];       // 最近一次收到的命令原文（截断）
extern uint32_t g_last_cmd_sec;       // 最近命令时间戳（相对开机的秒数）
extern uint8_t  g_last_cmd_ok;        // 0=失败, 1=成功, 2=无效JSON
extern uint16_t g_err_invalid_json;   // "Invalid JSON" 计数
extern uint16_t g_err_unknown_cmd;    // "Unknown command" 计数
extern uint16_t g_err_sensor;         // 传感器读取失败计数
extern char     g_last_error[40];     // 最近一次错误描述

// ---- 传感器遥测（由 main 循环通过 Comm_UpdateSensors 更新）----
extern uint8_t  g_sensor_ok;          // 最近一次温湿度读取是否正常
extern uint32_t g_sensor_sec;         // 最近一次传感器更新时间戳（秒）
extern float    g_temp, g_humi, g_wind;
extern char     g_wind_dir[4];
extern float    g_temp_min, g_temp_max;
extern float    g_humi_min, g_humi_max;
extern float    g_wind_max;

// ---- 报警阈值（RAM，可在设置页调整）----
extern float    g_th_temp_high;       // 高温报警阈值
extern float    g_th_humi_high;       // 高湿报警阈值
extern float    g_th_wind_high;       // 风速过高报警阈值

// ---- 运行时长（秒，由 main 每秒自增）----
extern volatile uint32_t g_uptime_sec;

// 更新温湿度遥测并维护最值、状态、错误计数
void Comm_UpdateTH(float temp, float humi, uint8_t th_ok);
// 更新风速风向遥测并维护最大值
void Comm_UpdateWind(float wind, const char* wind_dir);

#endif

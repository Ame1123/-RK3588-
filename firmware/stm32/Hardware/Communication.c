#include "Communication.h"
#include "Serial.h"
#include "Servo.h"
#include "cJSON.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// 全局变量定义
uint8_t moshiqiehuan = 1;   // 0=自动模式, 1=手动模式
uint8_t servo1stop = 0;     // 舵机1停止标志
uint8_t relay_switch = 0;   // 继电器开关
uint8_t box_value = 0;      // box状态：1=servo5转至100度/servo6转至25度, 0=servo5转至25度/servo6转至100度

// ---- 运行状态跟踪（详见 Communication.h）----
char     g_last_cmd[40]     = "(none)";
uint32_t g_last_cmd_sec     = 0;
uint8_t  g_last_cmd_ok      = 1;
uint16_t g_err_invalid_json = 0;
uint16_t g_err_unknown_cmd  = 0;
uint16_t g_err_sensor       = 0;
char     g_last_error[40]   = "(none)";

uint8_t  g_sensor_ok = 1;
uint32_t g_sensor_sec = 0;
float    g_temp = 0, g_humi = 0, g_wind = 0;
char     g_wind_dir[4] = "N";
float    g_temp_min = 999, g_temp_max = -999;
float    g_humi_min = 999, g_humi_max = -999;
float    g_wind_max = 0;

float    g_th_temp_high = 40.0f;   // 默认高温报警阈值(℃)
float    g_th_humi_high = 90.0f;   // 默认高湿报警阈值(%)
float    g_th_wind_high = 20.0f;   // 默认风速报警阈值(m/s)

volatile uint32_t g_uptime_sec = 0;

// JSON命令处理 - 详见main.c注释
uint8_t ProcessJSONCommand(char* json_str)
{
	cJSON *json = cJSON_Parse(json_str);
	if (json == NULL)
	{
		// JSON解析失败,检查是否看起来像JSON格式(以{开头)
		if (json_str[0] == '{' || json_str[0] == '[')
		{
			// 是JSON格式但解析失败,返回2表示JSON格式错误
			return 2;
		}
		// 不是JSON格式,返回0让传统命令处理
		return 0;
	}
	
	uint8_t processed = 0;
	
	// servo1角度设置
	cJSON *servo1_item = cJSON_GetObjectItem(json, "servo1");
	if (servo1_item != NULL && servo1_item->type == cJSON_Number)
	{
		float angle = (float)servo1_item->valuedouble;
		if (angle >= 0 && angle <= 180 && moshiqiehuan == 1)
		{
			Servo1_Angle = angle;
			processed = 1;
			Serial_Printf("{\"servo1_set\":%.1f}\r\n", angle);
		}
	}

	// servo2角度设置
	cJSON *servo2_item = cJSON_GetObjectItem(json, "servo2");
	if (servo2_item != NULL && servo2_item->type == cJSON_Number)
	{
		float angle = (float)servo2_item->valuedouble;
		if (angle >= 0 && angle <= 180)
		{
			Servo2_Angle = angle;
			processed = 1;
			Serial_Printf("{\"servo2_set\":%.1f}\r\n", angle);
		}
	}

	// servo3角度设置(限位90-180)
	cJSON *servo3_item = cJSON_GetObjectItem(json, "servo3");
	if (servo3_item != NULL && servo3_item->type == cJSON_Number)
	{
		float angle = (float)servo3_item->valuedouble;
		if (angle >= 90 && angle <= 180)
		{
			Servo3_Angle = angle;
			processed = 1;
			Serial_Printf("{\"servo3_set\":%.1f}\r\n", angle);
		}
	}
	
	// servo4角度设置
	cJSON *servo4_item = cJSON_GetObjectItem(json, "servo4");
	if (servo4_item != NULL && servo4_item->type == cJSON_Number)
	{
		float angle = (float)servo4_item->valuedouble;
		if (angle >= 0 && angle <= 180)
		{
			Servo4_Angle = angle;
			processed = 1;
			Serial_Printf("{\"servo4_set\":%.1f}\r\n", angle);
		}
	}
	
	// 自动模式切换
	cJSON *auto_item = cJSON_GetObjectItem(json, "auto");
	if (auto_item == NULL) auto_item = cJSON_GetObjectItem(json, "auto_mode");
	if (auto_item != NULL && auto_item->type == cJSON_Number)
	{
		int mode = (int)auto_item->valuedouble;
		if (mode == 0 || mode == 1)
		{
			moshiqiehuan = mode;
			processed = 1;
			Serial_Printf("{\"auto_set\":%d}\r\n", mode);
		}
	}

	// servo1stop命令
	cJSON *servo1stop_item = cJSON_GetObjectItem(json, "servo1stop");
	if (servo1stop_item != NULL && moshiqiehuan == 0)
	{
		int stopval = (int)servo1stop_item->valuedouble;
		if (stopval == 0 || stopval == 1)
		{
			servo1stop = stopval;
			processed = 1;
			Serial_Printf("{\"servo1stop_set\":%d}\r\n", stopval);
		}
	}
	
	// 继电器开关
	cJSON *switch_item = cJSON_GetObjectItem(json, "switch");
	if (switch_item != NULL && switch_item->type == cJSON_Number)
	{
		int sw = (int)switch_item->valuedouble;
		if (sw == 0 || sw == 1)
		{
			relay_switch = sw;
			processed = 1;
			Serial_Printf("{\"switch_set\":%d}\r\n", sw);
		}
	}

	// box控制
	cJSON *box_item = cJSON_GetObjectItem(json, "box");
	if (box_item != NULL && box_item->type == cJSON_Number)
	{
		int bv = (int)box_item->valuedouble;
		if (bv == 0 || bv == 1)
		{
			box_value = (uint8_t)bv;
			processed = 1;
			Serial_Printf("{\"box\":%d}\r\n", bv);
		}
	}
	
	// 传统命令
	cJSON *command_item = cJSON_GetObjectItem(json, "command");
	if (command_item != NULL && command_item->type == cJSON_String)
	{
		if (Servo_ProcessCommand(command_item->valuestring))
		{
			processed = 1;
			Serial_Printf("{\"command_executed\":\"%s\"}\r\n", command_item->valuestring);
		}
	}
	
	// 动作命令
	cJSON *action_item = cJSON_GetObjectItem(json, "action");
	if (action_item != NULL && action_item->type == cJSON_String)
	{
		if (strcmp(action_item->valuestring, "home") == 0)
		{
			Servo1_Angle = 90.0f;
			Servo2_Angle = 90.0f;
			Servo3_Angle = 90.0f;
			Servo4_Angle = 0.0f;
			processed = 1;
			Serial_Printf("{\"action_executed\":\"home\"}\r\n");
		}
		else if (strcmp(action_item->valuestring, "status") == 0)
		{
			Serial_Printf("{\"servo1\":%.1f,\"servo2\":%.1f,\"servo3\":%.1f,\"servo4\":%.1f,\"auto\":%d,\"status\":\"ok\"}\r\n",
				Servo1_Angle, Servo2_Angle, Servo3_Angle, Servo4_Angle, moshiqiehuan);
			processed = 1;
		}
	}
	
	cJSON_Delete(json);
	return processed;
}

// 记录最近一次命令原文（截断到缓冲区大小）
static void Comm_RecordLastCmd(const char* cmd)
{
	uint8_t i;
	for (i = 0; i < sizeof(g_last_cmd) - 1 && cmd[i]; i++)
	{
		g_last_cmd[i] = cmd[i];
	}
	g_last_cmd[i] = '\0';
	g_last_cmd_sec = g_uptime_sec;
}

// 串口命令处理
void ProcessSerialCommand(void)
{
	if (Serial_RxFlag == 1)
	{
		uint8_t result = ProcessJSONCommand(Serial_RxPacket);

		Comm_RecordLastCmd(Serial_RxPacket);

		if (result == 2)
		{
			// JSON格式错误
			Serial_Printf("{\"error\":\"Invalid JSON format\"}\r\n");
			g_last_cmd_ok = 2;
			g_err_invalid_json++;
			strcpy(g_last_error, "Invalid JSON");
		}
		else if (result == 0)
		{
			// 不是JSON,尝试传统命令
			if (!Servo_ProcessCommand(Serial_RxPacket))
			{
				Serial_Printf("{\"error\":\"Unknown command\"}\r\n");
				g_last_cmd_ok = 0;
				g_err_unknown_cmd++;
				strcpy(g_last_error, "Unknown command");
			}
			else
			{
				g_last_cmd_ok = 1;
			}
		}
		else
		{
			// result == 1，JSON命令已成功处理
			g_last_cmd_ok = 1;
		}

		Serial_RxFlag = 0;
	}
}

// 更新温湿度遥测，维护最值、状态与错误计数
void Comm_UpdateTH(float temp, float humi, uint8_t th_ok)
{
	g_sensor_ok = th_ok;
	g_sensor_sec = g_uptime_sec;
	if (th_ok)
	{
		g_temp = temp;
		g_humi = humi;
		if (temp < g_temp_min) g_temp_min = temp;
		if (temp > g_temp_max) g_temp_max = temp;
		if (humi < g_humi_min) g_humi_min = humi;
		if (humi > g_humi_max) g_humi_max = humi;
	}
	else
	{
		g_err_sensor++;
		strcpy(g_last_error, "SI7021 read error");
	}
}

// 更新风速风向遥测并维护最大值
void Comm_UpdateWind(float wind, const char* wind_dir)
{
	g_wind = wind;
	if (wind > g_wind_max) g_wind_max = wind;
	if (wind_dir)
	{
		uint8_t i;
		for (i = 0; i < sizeof(g_wind_dir) - 1 && wind_dir[i]; i++)
			g_wind_dir[i] = wind_dir[i];
		g_wind_dir[i] = '\0';
	}
}

// JSON数据发送函数
void SendServoAnglesJSON4_Safe(float angle1, float angle2, float angle3, float angle4)
{
	Serial_Printf("{\"servo1\":%.1f,\"servo2\":%.1f,\"servo3\":%.1f,\"servo4\":%.1f}\r\n", 
		angle1, angle2, angle3, angle4);
}

void SendTempHumidityJSON_Safe(float temperature, float humidity)
{
	Serial_Printf("{\"temp\":%.2f,\"humi\":%.2f}\r\n", temperature, humidity);
}

void SendWindDataJSON_Safe(float wind_speed, const char* wind_direction)
{
	Serial_Printf("{\"su\":%.2f,\"xiang\":\"%s\"}\r\n", wind_speed, wind_direction);
}

void SendModeJSON_Safe(void)
{
	Serial_Printf("{\"auto\":%d,\"servo1stop\":%d,\"switch\":%d}\r\n", 
		moshiqiehuan, servo1stop, relay_switch);
}

void SendSystemStatusJSON_Safe(const char* message)
{
	Serial_Printf("{\"status\":\"%s\"}\r\n", message);
}

// box目标角度（文件级，供 BoxServo_InPosition 判断到位）
static float servo5_target = 25.0f;
static float servo6_target = 100.0f;

// box舵机平滑运动更新，每50ms调用一次
void BoxServo_Update(void)
{
	static uint8_t last_box = 0;

	// 仅在box状态发生变化时更新目标角度
	if (box_value != last_box)
	{
		if (box_value == 1)
		{
			servo5_target = 100.0f;	// servo5转至100度
			servo6_target = 25.0f;	// servo6转至25度
		}
		else
		{
			servo5_target = 25.0f;	// servo5转回25度
			servo6_target = 100.0f;	// servo6转回100度
		}
		last_box = box_value;
	}

	// 平滑推进舵机5 (4度/50ms，25到100度约1秒内到位)
	const float step = 4.0f;
	if (Servo5_Angle < servo5_target)
	{
		Servo5_Angle += step;
		if (Servo5_Angle > servo5_target) Servo5_Angle = servo5_target;
	}
	else if (Servo5_Angle > servo5_target)
	{
		Servo5_Angle -= step;
		if (Servo5_Angle < servo5_target) Servo5_Angle = servo5_target;
	}

	// 平滑推进舵机6
	if (Servo6_Angle < servo6_target)
	{
		Servo6_Angle += step;
		if (Servo6_Angle > servo6_target) Servo6_Angle = servo6_target;
	}
	else if (Servo6_Angle > servo6_target)
	{
		Servo6_Angle -= step;
		if (Servo6_Angle < servo6_target) Servo6_Angle = servo6_target;
	}
}

// box是否已运动到位（两个舵机都到达目标角度）
uint8_t BoxServo_InPosition(void)
{
	return (Servo5_Angle == servo5_target && Servo6_Angle == servo6_target) ? 1 : 0;
}

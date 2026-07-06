#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Servo.h"
#include "Serial.h"
#include "bsp_si7021.h"
#include "bsp_i2c.h"
#include "bsp_sm5388.h"
#include "SysTick.h"
#include "tftlcd.h"
#include "Display.h"
#include "ui.h"
#include "Communication.h"
#include <stdio.h>

#define LCD_BOOT_ANIMATION_ENABLE 0
#define LCD_STATIC_STABLE_TEST_ENABLE 0

#if LCD_STATIC_STABLE_TEST_ENABLE
static void LCD_DrawStableTestPage(void)
{
	u16 w = tftlcd_data.width;
	u16 h = tftlcd_data.height;

	LCD_Clear(BLACK);

	LCD_Fill(0, 0, 59, 59, RED);
	LCD_Fill(w - 60, 0, w - 1, 59, GREEN);
	LCD_Fill(0, h - 60, 59, h - 1, BLUE);
	LCD_Fill(w - 60, h - 60, w - 1, h - 1, YELLOW);
	LCD_Fill((w / 2) - 30, (h / 2) - 30, (w / 2) + 29, (h / 2) + 29, CYAN);

	LCD_Fill(0, 72, w - 1, 115, DARKBLUE);
	FRONT_COLOR = WHITE;
	BACK_COLOR = DARKBLUE;
	LCD_ShowString(20, 84, w - 40, 24, 24, (u8 *)"LCD DOT TEST");

	FRONT_COLOR = WHITE;
	BACK_COLOR = BLACK;
	LCD_ShowString(18, h - 82, w - 36, 16, 16, (u8 *)"Write only, RD high, no refresh");
	LCD_ShowString(18, h - 58, w - 36, 16, 16, (u8 *)"If this still flickers: add caps/resistors");
}
#endif

/**
  * 文件说明：
  *   - 本文件为四舵机PWM控制主程序，支持JSON与传统命令双协议。
  *   - 舵机角度全局变量（在Servo.c中定义，这里声明）
  *     extern float Servo1_Angle; // 舵机1角度全局变量
  *     extern float Servo2_Angle; // 舵机2角度全局变量
  *     extern float Servo3_Angle; // 舵机3角度全局变量（上电默认90度，限位90-180度）
  *     extern float Servo4_Angle; // 舵机4角度全局变量（PB6引脚）
  *   - 这些变量可被主机直接访问和控制（已在Servo.h声明，这里无需重复声明）。
  *
  * 硬件引脚分配：
  *   舵机：PA1(舵机1), PA2(舵机2), PA3(舵机3), PB6(舵机4)
  *   传感器：PA4/PA5(I2C-SI7021温湿度), PA6(SM5388风速), PA7(SM5388风向)
  *   串口：PA9(TX), PA10(RX)
  *   继电器：PB8
  *   TFTLCD（FSMC）：PD0/1/4/5/8-10/14/15, PE7-15, PG0/12, PB0(背光)
  *
  * JSON命令格式说明：
  *   1. 设置单个舵机角度：
  *      {"servo1": 90}           - 设置舵机1到90度
  *      {"servo2": 45}           - 设置舵机2到45度
  *      {"servo3": 135}          - 设置舵机3到135度（限位90-180度）
  *      {"servo4": 60}           - 设置舵机4到60度（PB6引脚）
  *   2. 同时设置多个舵机：
  *      {"servo1": 90, "servo2": 45, "servo3": 60, "servo4": 60}  - 同时设置四个舵机
  *   3. 使用传统命令：
  *      {"command": "S1G1"}      - 执行舵机1档位1命令
  *      {"command": "ALLHOME"}   - 执行四舵机归零命令
  *
  *   4. 快捷动作命令：
  *      {"action": "home"}       - 舵机1归零到90度，舵机2归零到90度，舵机3归零到90度，舵机4归零到0度
  *      {"action": "status"}     - 查询当前四个舵机状态
  *   5. 舵机1自动模式切换：
  *      {"auto": 0}         - 启用舵机1自动模式(10秒循环0-180-0度)
  *      {"auto": 1}         - 启用舵机1手动模式(接收指令控制)
  *   6. 继电器控制：
  *      {"switch": 0}       - 继电器关闭
  *      {"switch": 1}       - 继电器高电平启动
  *
  * 响应格式：
  *   - 成功时返回确认消息，如：{"servo1_set": 90}, {"servo4_set": 60}
  *   - 错误时返回错误信息，如：{"error": "Unknown command format"}
  *   - 状态查询返回：{"servo1": 90, "servo2": 0, "servo3": 90, "servo4": 0, "auto": 1, "servo1_mode": "manual", "status": "ok"}
  *   - 自动模式切换：{"auto_set": 0, "description": "Servo1 auto mode enabled"}
  * 
  * 传感器数据上报（每秒自动发送）：
  *   - 舵机角度：{"servo1":90.0,"servo2":0.0,"servo3":90.0,"servo4":0.0}
  *   - 温湿度：{"temp":25.30,"humi":45.60}
  *   - 风速风向：{"su":12.50,"xiang":"S"}
  *   - 模式状态：{"auto":1,"servo1stop":0,"switch":0}
  * 
  *  非JSON传统命令说明（直接串口发送字符串）：
  *   - S1G1/S1G2/S1G3/S1G4/S1HOME   控制舵机1档位/归零
  *   - S2G1/S2G2/S2G3/S2G4/S2HOME   控制舵机2档位/归零
  *   - S3G1/S3G2/S3G3/S3G4/S3HOME   控制舵机3档位/归零（限位90-180度，HOME为90度）
  *   - S4G1/S4G2/S4G3/S4G4/S4HOME   控制舵机4档位/归零（PB6引脚）
  *   - BOTH1/BOTH2/BOTH3/BOTH4/BOTHOME   同时控制舵机1和2
  *   - ALL1/ALL2/ALL3/ALL4/ALLHOME      同时控制四舵机
  *   - 发送如 S1G1 + 回车 即可直接控制，无需JSON封装
  *
  * 舵机1自动旋转功能：
  *   - moshiqiehuan = 0: 舵机1自动模式，10秒内从0-180-0度循环
  *   - moshiqiehuan = 1: 舵机1手动模式，接收指令控制
  *   - JSON切换命令：{"auto": 0} 或 {"auto": 1}
  */

/**
  * 函    数：舵机1自动旋转控制
  * 参    数：无
  * 返 回 值：无
  * 说    明：当moshiqiehuan=0时，控制舵机1在10秒内完成0-180-0度循环
  */
void Servo1_AutoRotate(void)
{
	static float currentAngle = 0.0f;
	static int direction = 0; // 0: 0->180, 1: 180->0
	static uint8_t last_mode = 1;

	// 匀速参数
const float step = 180.0f / 143.0f; // 143周期完成180度，速度降低约30%，周期与主循环50ms一致

	if (moshiqiehuan == 0)  // 自动模式
	{
		// 如果刚切换到自动模式，记录当前位置，先归零
		if (last_mode != 0)
		{
			// 进入自动模式，方向先归零
			direction = -1; // -1表示归零阶段
		}

		if (servo1stop == 1)
		{
			// 停止自动旋转，保持当前位置
			// 不改变Servo1_Angle，currentAngle保持
			last_mode = 0;
			return;
		}

		if (direction == -1)
		{
			// 归零阶段，匀速归零
			if (currentAngle > step)
			{
				currentAngle -= step;
				if (currentAngle < 0.0f) currentAngle = 0.0f;
				Servo1_Angle = currentAngle;
			}
			else if (currentAngle < -step)
			{
				currentAngle += step;
				if (currentAngle > 0.0f) currentAngle = 0.0f;
				Servo1_Angle = currentAngle;
			}
			else
			{
				currentAngle = 0.0f;
				Servo1_Angle = 0.0f;
				direction = 0; // 开始0->180
			}
		}
		else if (direction == 0)
		{
			// 0->180
			currentAngle += step;
			if (currentAngle >= 180.0f)
			{
				currentAngle = 180.0f;
				direction = 1;
			}
			Servo1_Angle = currentAngle;
		}
		else if (direction == 1)
		{
			// 180->0
			currentAngle -= step;
			if (currentAngle <= 0.0f)
			{
				currentAngle = 0.0f;
				direction = 0;
			}
			Servo1_Angle = currentAngle;
		}
		last_mode = 0;
	}
	else
	{
		// 切换到手动模式时，记录当前位置
		currentAngle = Servo1_Angle;
		last_mode = 1;
	}
}

// 继电器IO初始化（B8高电平启动）
void Relay_GPIO_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_8); // 默认低电平关闭
}

// 继电器控制函数
void Relay_GPIO_Set(uint8_t state)
{
	if(state)
		GPIO_SetBits(GPIOB, GPIO_Pin_8); // 高电平启动
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_8); // 低电平关闭
}

int main(void)
{
	/*模块初始化*/
	Servo_Init();
	Serial_Init();
	I2C_GPIO_Config();
	SM5388_Init();      // 风速风向传感器初始化
	Relay_GPIO_Init();  // 继电器IO初始化

	/* TFTLCD + 触摸屏初始化 */
	SysTick_Init(72);   // LCD/触摸延时依赖（72MHz HCLK）
	TFTLCD_Init();      // FSMC + LCD 控制器初始化（HX8357DN 320x480）

	// 设置初始位置
	Servo_UpdateAngles();	// 更新舵机到初始位置(舵机1=90，舵机2=0，舵机3=90，舵机4=0，舵机5=25，舵机6=100)

	// 播放开机动画
#if LCD_BOOT_ANIMATION_ENABLE
	Display_BootAnimation();
#endif

	// 初始化触摸屏并绘制首页（含首次校准）
#if LCD_STATIC_STABLE_TEST_ENABLE
	LCD_DrawStableTestPage();
#else
	UI_Init();
#endif

	// 发送启动信息
	SendSystemStatusJSON_Safe("Quad Servo Control System Started");
	// Serial_Printf("Quad Servo Control System Started\r\n");
	// Serial_Printf("Servo1 Commands: S1G1, S1G2, S1G3, S1G4, S1HOME\r\n");
	// Serial_Printf("Servo2 Commands: S2G1, S2G2, S2G3, S2G4, S2HOME\r\n");
	// Serial_Printf("Servo3 Commands: S3G1, S3G2, S3G3, S3G4, S3HOME\r\n");
	// Serial_Printf("Servo4 Commands: S4G1, S4G2, S4G3, S4G4, S4HOME\r\n");
	// Serial_Printf("Both Servos: BOTH1, BOTH2, BOTH3, BOTH4, BOTHOME\r\n");
	// Serial_Printf("All Servos: ALL1, ALL2, ALL3, ALL4, ALLHOME\r\n");
	// Serial_Printf("Command Format: Send command directly (e.g., S1G1 + Enter)\r\n");
	SendSystemStatusJSON_Safe("Temperature & Humidity monitoring enabled (SI7021)");
	SendSystemStatusJSON_Safe("Wind Speed & Direction monitoring enabled (SM5388)");
	// Serial_Printf("Temperature & Humidity monitoring enabled (SI7021)\r\n");
	// Serial_Printf("Current Positions - Servo1: %.0f degrees, Servo2: %.0f degrees\r\n", Angle1, Angle2);
	
	// 发送全局变量控制信息
	SendSystemStatusJSON_Safe("Global servo angle variables ready for host control");
	SendSystemStatusJSON_Safe("JSON command support enabled");
	SendSystemStatusJSON_Safe("Supported formats: servo1-4, action:home/status, auto:0/1");
	SendSystemStatusJSON_Safe("Servo1 auto mode: auto:0 (auto), auto:1 (manual)");
	
	// 可选：测试全局变量控制（取消注释以启用测试）
	// ExampleDirectServoControl();	// 运行示例控制函数
	
	// 可选：测试JSON命令解析（取消注释以启用测试）
	// ProcessJSONCommand("{\"servo1\":45}");		// 测试设置舵机1
	// ProcessJSONCommand("{\"action\":\"status\"}");	// 测试状态查询
	
	// ...existing code...
	
	// 让三类回报错开时序,互不重叠
	static uint32_t tick = 0;
	static uint8_t last_relay_logged = 0;   /* 触摸 UI 已接管继电器状态显示 */
	static uint32_t sec_cnt = 0;            /* 1秒计数器 */

	(void)last_relay_logged;

	while (1)
	{
		// 处理串口命令
		ProcessSerialCommand();

		// box舵机5/6控制（状态变化时触发平滑运动）
		BoxServo_Update();

		// 舵机1自动旋转控制
		Servo1_AutoRotate();

		// 更新舵机角度
		Servo_UpdateAngles();

		// 触摸屏轮询（约 50ms 一次）
#if !LCD_STATIC_STABLE_TEST_ENABLE
		UI_Poll();

		// 触摸 UI 渲染（全量/动态自动判断）
		UI_Render();
#endif

		// 角度、温湿度、风速风向、模式回报都在1秒内各自回报一次,且错开时序
		// tick周期为20(1秒),每50ms tick++
		if (tick == 0)
		{
			// 回报舵机角度
			SendServoAnglesJSON4_Safe(Servo_GetAngle1(), Servo_GetAngle2(), Servo_GetAngle3(), Servo_GetAngle4());
		}
		else if (tick == 5)
		{
			// 读取并回报温湿度
			float temperature = Si7021_Measure(TEMP_NOHOLD_MASTER);
			float humidity = Si7021_Measure(HUMI_NOHOLD_MASTER);
			uint8_t th_ok = (temperature < 100 && humidity < 100 && temperature > -50 && humidity > -10.0f);
			if (th_ok)
			{
				SendTempHumidityJSON_Safe(temperature, humidity);
			}
			else
			{
				Serial_Printf("{\"error\":\"SI7021 sensor read error\",\"temp_raw\":%.2f,\"humi_raw\":%.2f}\r\n",
					temperature, humidity);
			}
			Comm_UpdateTH(temperature, humidity, th_ok);
		}
		else if (tick == 10)
		{
			// 读取并回报风速风向
			float wind_speed = SM5388_GetWindSpeed();
			const char* wind_direction_str = SM5388_GetWindDirectionString();
			SendWindDataJSON_Safe(wind_speed, wind_direction_str);
			Comm_UpdateWind(wind_speed, wind_direction_str);
		}
		else if (tick == 15)
		{
			// 回报模式状态
			SendModeJSON_Safe();
		}
		else if (tick == 16)
		{
			// 回报box状态（间隔50ms，避免与模式状态包粘连）
			Serial_Printf("{\"box\":%d}\r\n", box_value);
		}

		// 继电器控制
		Relay_GPIO_Set(relay_switch);

		// 运行时长计时（每秒自增）
		if (++sec_cnt >= 20)   /* 20 × 50ms = 1秒 */
		{
			g_uptime_sec++;
			sec_cnt = 0;
		}

		Delay_ms(50);
		tick++;
		if (tick >= 20) tick = 0; // 1秒循环
	}
}


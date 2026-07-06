#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "Servo.h"
#include <string.h>

// 舵机角度全局变量
float Servo1_Angle = 90.0f;		// 第一个舵机角度变量，初始为90度
float Servo2_Angle = 0.0f;		// 第二个舵机角度变量，初始为0度
float Servo3_Angle = 90.0f;		// 第三个舵机角度变量，初始为90度，限位90-180度
float Servo4_Angle = 0.0f;		// 第四个舵机角度变量，初始为0度
float Servo5_Angle = 25.0f;		// 第五个舵机角度变量，初始为25度 (PB7)
float Servo6_Angle = 100.0f;		// 第六个舵机角度变量，初始为100度 (PB9)

/**
  * 函    数：舵机初始化
  * 参    数：无
  * 返 回 值：无
  */
void Servo_Init(void)
{
	PWM_Init();									//初始化舵机的底层PWM
}

/**
  * 函    数：舵机设置角度
  * 参    数：ServoID 舵机编号（SERVO_1、SERVO_2、SERVO_3或SERVO_4）
  * 参    数：Angle 要设置的舵机角度，范围：0~180
  * 返 回 值：无
  */
void Servo_SetAngle(uint8_t ServoID, float Angle)
{
	uint16_t Compare = Angle / 180 * 2000 + 500;	// 设置占空比
													// 将角度线性变换，对应到舵机要求的占空比范围上
	
	if (ServoID == SERVO_1)
	{
		PWM_SetCompare2(Compare);					// 控制第一个舵机（PA1）
	}
	else if (ServoID == SERVO_2)
	{
		PWM_SetCompare3(Compare);					// 控制第二个舵机（PA2）
	}
	else if (ServoID == SERVO_3)
	{
		PWM_SetCompare4(Compare);					// 控制第三个舵机（PA3）
	}
	else if (ServoID == SERVO_4)
	{
		PWM_SetCompare4_TIM4(Compare);				// 控制第四个舵机（PB6）
	}	else if (ServoID == SERVO_5)
	{
		PWM_SetCompare_TIM4CH2(Compare);				// 控制第五个舵机（PB7）
	}
	else if (ServoID == SERVO_6)
	{
		PWM_SetCompare_TIM4CH4(Compare);				// 控制第六个舵机（PB9）
	}}

/**
  * 函    数：更新舵机角度
  * 参    数：无
  * 返 回 值：无
  * 说    明：将内部角度变量应用到实际舵机
  */
void Servo_UpdateAngles(void)
{
	Servo_SetAngle(SERVO_1, Servo1_Angle);	// 设置第一个舵机的角度
	Servo_SetAngle(SERVO_2, Servo2_Angle);	// 设置第二个舵机的角度
	Servo_SetAngle(SERVO_3, Servo3_Angle);	// 设置第三个舵机的角度
	Servo_SetAngle(SERVO_4, Servo4_Angle);	// 设置第四个舵机的角度
	Servo_SetAngle(SERVO_5, Servo5_Angle);	// 设置第五个舵机的角度
	Servo_SetAngle(SERVO_6, Servo6_Angle);	// 设置第六个舵机的角度
}

/**
  * 函    数：获取舵机1当前角度
  * 参    数：无
  * 返 回 值：舵机1的当前角度
  */
float Servo_GetAngle1(void)
{
	return Servo1_Angle;
}

/**
  * 函    数：获取舵机2当前角度
  * 参    数：无
  * 返 回 值：舵机2的当前角度
  */
float Servo_GetAngle2(void)
{
	return Servo2_Angle;
}

/**
  * 函    数：获取舵机3当前角度
  * 参    数：无
  * 返 回 值：舵机3的当前角度
  */
float Servo_GetAngle3(void)
{
	return Servo3_Angle;
}

/**
  * 函    数：获取舵机4当前角度
  * 参    数：无
  * 返 回 值：舵机4的当前角度
  */
float Servo_GetAngle4(void)
{
	return Servo4_Angle;
}

/**
  * 函    数：直接设置舵机1角度
  * 参    数：angle 要设置的角度（0-180度）
  * 返 回 值：无
  */
void Servo_SetAngle1(float angle)
{
	if (angle >= 0 && angle <= 180)
	{
		Servo1_Angle = angle;
	}
}

/**
  * 函    数：直接设置舵机2角度
  * 参    数：angle 要设置的角度（0-180度）
  * 返 回 值：无
  */
void Servo_SetAngle2(float angle)
{
	if (angle >= 0 && angle <= 180)
	{
		Servo2_Angle = angle;
	}
}

/**
  * 函    数：直接设置舵机3角度
  * 参    数：angle 要设置的角度（90-180度，限位）
  * 返 回 值：无
  */
void Servo_SetAngle3(float angle)
{
	if (angle >= 90 && angle <= 180)  // 舵机3限位90-180度
	{
		Servo3_Angle = angle;
	}
}

/**
  * 函    数：直接设置舵机4角度
  * 参    数：angle 要设置的角度（0-180度）
  * 返 回 值：无
  */
void Servo_SetAngle4(float angle)
{
	if (angle >= 0 && angle <= 180)
	{
		Servo4_Angle = angle;
	}
}

/**
  * 函    数：直接设置舵机5角度
  * 参    数：angle 要设置的角度（0-80度）
  * 返 回 值：无
  */
void Servo_SetAngle5(float angle)
{
	if (angle >= 0 && angle <= 80)
	{
		Servo5_Angle = angle;
	}
}

/**
  * 函    数：直接设置舵机6角度
  * 参    数：angle 要设置的角度（0-80度）
  * 返 回 值：无
  */
void Servo_SetAngle6(float angle)
{
	if (angle >= 0 && angle <= 80)
	{
		Servo6_Angle = angle;
	}
}

/**
  * 函    数：处理舵机控制命令
  * 参    数：command 要处理的命令字符串
  * 返 回 值：1表示命令被识别并处理，0表示未知命令
  */
uint8_t Servo_ProcessCommand(char* command)
{
	// 舵机1控制命令
	if (strcmp(command, GEAR_1) == 0)
	{
		Servo1_Angle = SERVO1_ANGLE_GEAR_1;
		return 1;
	}
	else if (strcmp(command, GEAR_2) == 0)
	{
		Servo1_Angle = SERVO1_ANGLE_GEAR_2;
		return 1;
	}
	else if (strcmp(command, GEAR_3) == 0)
	{
		Servo1_Angle = SERVO1_ANGLE_GEAR_3;
		return 1;
	}
	else if (strcmp(command, GEAR_4) == 0)
	{
		Servo1_Angle = SERVO1_ANGLE_GEAR_4;
		return 1;
	}
	else if (strcmp(command, GEAR_HOME) == 0)
	{
		Servo1_Angle = SERVO1_ANGLE_HOME;
		return 1;
	}
	// 舵机2控制命令
	else if (strcmp(command, SERVO2_GEAR_1) == 0)
	{
		Servo2_Angle = SERVO2_ANGLE_GEAR_1;
		return 1;
	}
	else if (strcmp(command, SERVO2_GEAR_2) == 0)
	{
		Servo2_Angle = SERVO2_ANGLE_GEAR_2;
		return 1;
	}
	else if (strcmp(command, SERVO2_GEAR_3) == 0)
	{
		Servo2_Angle = SERVO2_ANGLE_GEAR_3;
		return 1;
	}
	else if (strcmp(command, SERVO2_GEAR_4) == 0)
	{
		Servo2_Angle = SERVO2_ANGLE_GEAR_4;
		return 1;
	}
	else if (strcmp(command, SERVO2_HOME) == 0)
	{
		Servo2_Angle = SERVO2_ANGLE_HOME;
		return 1;
	}
	// 舵机3控制命令
	else if (strcmp(command, SERVO3_GEAR_1) == 0)
	{
		Servo3_Angle = SERVO3_ANGLE_GEAR_1;
		return 1;
	}
	else if (strcmp(command, SERVO3_GEAR_2) == 0)
	{
		Servo3_Angle = SERVO3_ANGLE_GEAR_2;
		return 1;
	}
	else if (strcmp(command, SERVO3_GEAR_3) == 0)
	{
		Servo3_Angle = SERVO3_ANGLE_GEAR_3;
		return 1;
	}
	else if (strcmp(command, SERVO3_GEAR_4) == 0)
	{
		Servo3_Angle = SERVO3_ANGLE_GEAR_4;
		return 1;
	}
	else if (strcmp(command, SERVO3_HOME) == 0)
	{
		Servo3_Angle = SERVO3_ANGLE_HOME;
		return 1;
	}
	// 舵机4控制命令
	else if (strcmp(command, SERVO4_GEAR_1) == 0)
	{
		Servo4_Angle = SERVO4_ANGLE_GEAR_1;
		return 1;
	}
	else if (strcmp(command, SERVO4_GEAR_2) == 0)
	{
		Servo4_Angle = SERVO4_ANGLE_GEAR_2;
		return 1;
	}
	else if (strcmp(command, SERVO4_GEAR_3) == 0)
	{
		Servo4_Angle = SERVO4_ANGLE_GEAR_3;
		return 1;
	}
	else if (strcmp(command, SERVO4_GEAR_4) == 0)
	{
		Servo4_Angle = SERVO4_ANGLE_GEAR_4;
		return 1;
	}
	else if (strcmp(command, SERVO4_HOME) == 0)
	{
		Servo4_Angle = SERVO4_ANGLE_HOME;
		return 1;
	}
	// 两个舵机同时控制命令
	else if (strcmp(command, BOTH_GEAR_1) == 0)
	{
		Servo1_Angle = SERVO1_ANGLE_GEAR_1;
		Servo2_Angle = SERVO2_ANGLE_GEAR_1;
		return 1;
	}
	else if (strcmp(command, BOTH_GEAR_2) == 0)
	{
		Servo1_Angle = SERVO1_ANGLE_GEAR_2;
		Servo2_Angle = SERVO2_ANGLE_GEAR_2;
		return 1;
	}
	else if (strcmp(command, BOTH_GEAR_3) == 0)
	{
		Servo1_Angle = SERVO1_ANGLE_GEAR_3;
		Servo2_Angle = SERVO2_ANGLE_GEAR_3;
		return 1;
	}
	else if (strcmp(command, BOTH_GEAR_4) == 0)
	{
		Servo1_Angle = SERVO1_ANGLE_GEAR_4;
		Servo2_Angle = SERVO2_ANGLE_GEAR_4;
		return 1;
	}
	else if (strcmp(command, BOTH_HOME) == 0)
	{
		Servo1_Angle = SERVO1_ANGLE_HOME;
		Servo2_Angle = SERVO2_ANGLE_HOME;
		return 1;
	}
	// 三个舵机同时控制命令
	else if (strcmp(command, ALL_GEAR_1) == 0)
	{
		Servo1_Angle = SERVO1_ANGLE_GEAR_1;
		Servo2_Angle = SERVO2_ANGLE_GEAR_1;
		Servo3_Angle = SERVO3_ANGLE_GEAR_1;
		Servo4_Angle = SERVO4_ANGLE_GEAR_1;
		return 1;
	}
	else if (strcmp(command, ALL_GEAR_2) == 0)
	{
		Servo1_Angle = SERVO1_ANGLE_GEAR_2;
		Servo2_Angle = SERVO2_ANGLE_GEAR_2;
		Servo3_Angle = SERVO3_ANGLE_GEAR_2;
		Servo4_Angle = SERVO4_ANGLE_GEAR_2;
		return 1;
	}
	else if (strcmp(command, ALL_GEAR_3) == 0)
	{
		Servo1_Angle = SERVO1_ANGLE_GEAR_3;
		Servo2_Angle = SERVO2_ANGLE_GEAR_3;
		Servo3_Angle = SERVO3_ANGLE_GEAR_3;
		Servo4_Angle = SERVO4_ANGLE_GEAR_3;
		return 1;
	}
	else if (strcmp(command, ALL_GEAR_4) == 0)
	{
		Servo1_Angle = SERVO1_ANGLE_GEAR_4;
		Servo2_Angle = SERVO2_ANGLE_GEAR_4;
		Servo3_Angle = SERVO3_ANGLE_GEAR_4;
		Servo4_Angle = SERVO4_ANGLE_GEAR_4;
		return 1;
	}
	else if (strcmp(command, ALL_HOME) == 0)
	{
		Servo1_Angle = SERVO1_ANGLE_HOME;
		Servo2_Angle = SERVO2_ANGLE_HOME;
		Servo3_Angle = SERVO3_ANGLE_HOME;
		Servo4_Angle = SERVO4_ANGLE_HOME;
		return 1;
	}
	
	return 0;	// 未知命令
}

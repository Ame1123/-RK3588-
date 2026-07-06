#ifndef __SERVO_H
#define __SERVO_H

// 舵机编号定义
#define SERVO_1		0		// 第一个舵机（原有引脚）
#define SERVO_2		1		// 第二个舵机（PA2引脚）
#define SERVO_3		2		// 第三个舵机（PA3引脚）
#define SERVO_4		3		// 第四个舵机（PB6引脚）
#define SERVO_5		4		// 第五个舵机（PB7引脚）
#define SERVO_6		5		// 第六个舵机（PB9引脚）

// 档位宏定义 - 舵机1
#define GEAR_1		"S1G1"		// 档位1命令
#define GEAR_2		"S1G2"		// 档位2命令  
#define GEAR_3		"S1G3"		// 档位3命令
#define GEAR_4		"S1G4"		// 档位4命令
#define GEAR_HOME	"S1HOME"	// 归零命令

// 档位宏定义 - 舵机2
#define SERVO2_GEAR_1	"S2G1"		// 舵机2档位1命令
#define SERVO2_GEAR_2	"S2G2"		// 舵机2档位2命令
#define SERVO2_GEAR_3	"S2G3"		// 舵机2档位3命令
#define SERVO2_GEAR_4	"S2G4"		// 舵机2档位4命令
#define SERVO2_HOME		"S2HOME"	// 舵机2归零命令

// 档位宏定义 - 舵机3
#define SERVO3_GEAR_1	"S3G1"		// 舵机3档位1命令
#define SERVO3_GEAR_2	"S3G2"		// 舵机3档位2命令
#define SERVO3_GEAR_3	"S3G3"		// 舵机3档位3命令
#define SERVO3_GEAR_4	"S3G4"		// 舵机3档位4命令
#define SERVO3_HOME		"S3HOME"	// 舵机3归零命令

// 档位宏定义 - 舵机4
#define SERVO4_GEAR_1	"S4G1"		// 舵机4档位1命令
#define SERVO4_GEAR_2	"S4G2"		// 舵机4档位2命令
#define SERVO4_GEAR_3	"S4G3"		// 舵机4档位3命令
#define SERVO4_GEAR_4	"S4G4"		// 舵机4档位4命令
#define SERVO4_HOME		"S4HOME"	// 舵机4归零命令

// 同时控制两个舵机的命令
#define BOTH_GEAR_1		"BOTH1"		// 两个舵机同时档位1
#define BOTH_GEAR_2		"BOTH2"		// 两个舵机同时档位2
#define BOTH_GEAR_3		"BOTH3"		// 两个舵机同时档位3
#define BOTH_GEAR_4		"BOTH4"		// 两个舵机同时档位4
#define BOTH_HOME		"BOTHOME"	// 两个舵机同时归零

// 同时控制三个舵机的命令
#define ALL_GEAR_1		"ALL1"		// 三个舵机同时档位1
#define ALL_GEAR_2		"ALL2"		// 三个舵机同时档位2
#define ALL_GEAR_3		"ALL3"		// 三个舵机同时档位3
#define ALL_GEAR_4		"ALL4"		// 三个舵机同时档位4
#define ALL_HOME		"ALLHOME"	// 三个舵机同时归零

// 舵机1档位角度定义
#define SERVO1_ANGLE_GEAR_1		0		// 舵机1档位1角度：0度
#define SERVO1_ANGLE_GEAR_2		60		// 舵机1档位2角度：60度
#define SERVO1_ANGLE_GEAR_3		120		// 舵机1档位3角度：120度
#define SERVO1_ANGLE_GEAR_4		180		// 舵机1档位4角度：180度
#define SERVO1_ANGLE_HOME		90		// 舵机1归零角度：90度

// 舵机2档位角度定义
#define SERVO2_ANGLE_GEAR_1		0		// 舵机2档位1角度：0度
#define SERVO2_ANGLE_GEAR_2		60		// 舵机2档位2角度：60度
#define SERVO2_ANGLE_GEAR_3		120		// 舵机2档位3角度：120度
#define SERVO2_ANGLE_GEAR_4		180		// 舵机2档位4角度：180度
#define SERVO2_ANGLE_HOME		0		// 舵机2归零角度：0度

// 舵机3档位角度定义
#define SERVO3_ANGLE_GEAR_1		90		// 舵机3档位1角度：90度（限位下限）
#define SERVO3_ANGLE_GEAR_2		120		// 舵机3档位2角度：120度
#define SERVO3_ANGLE_GEAR_3		150		// 舵机3档位3角度：150度
#define SERVO3_ANGLE_GEAR_4		180		// 舵机3档位4角度：180度（限位上限）
#define SERVO3_ANGLE_HOME		90		// 舵机3归零角度：90度（上电默认）

// 舵机4档位角度定义
#define SERVO4_ANGLE_GEAR_1		0		// 舵机4档位1角度：0度
#define SERVO4_ANGLE_GEAR_2		60		// 舵机4档位2角度：60度
#define SERVO4_ANGLE_GEAR_3		120		// 舵机4档位3角度：120度
#define SERVO4_ANGLE_GEAR_4		180		// 舵机4档位4角度：180度
#define SERVO4_ANGLE_HOME		0		// 舵机4归零角度：0度

// 全局变量声明（舵机角度）
extern float Servo1_Angle;		// 舵机1角度全局变量
extern float Servo2_Angle;		// 舵机2角度全局变量
extern float Servo3_Angle;		// 舵机3角度全局变量
extern float Servo4_Angle;		// 舵机4角度全局变量
extern float Servo5_Angle;		// 舵机5角度全局变量
extern float Servo6_Angle;		// 舵机6角度全局变量

void Servo_Init(void);
void Servo_SetAngle(uint8_t ServoID, float Angle);
void Servo_UpdateAngles(void);
float Servo_GetAngle1(void);
float Servo_GetAngle2(void);
float Servo_GetAngle3(void);
float Servo_GetAngle4(void);
// 三舵机JSON回报函数声明
void SendServoAnglesJSON3(float angle1, float angle2, float angle3);
void Servo_SetAngle1(float angle);		// 直接设置舵机1角度
void Servo_SetAngle2(float angle);		// 直接设置舵机2角度
void Servo_SetAngle3(float angle);		// 直接设置舵机3角度（限位90-180度）
void Servo_SetAngle4(float angle);		// 直接设置舵机4角度
void Servo_SetAngle5(float angle);		// 直接设置舵机5角度
void Servo_SetAngle6(float angle);		// 直接设置舵机6角度
uint8_t Servo_ProcessCommand(char* command);

#endif

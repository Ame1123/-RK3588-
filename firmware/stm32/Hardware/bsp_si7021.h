#ifndef	__BSP_SI7021_H__
#define	__BSP_SI7021_H__

#include "stm32f10x.h"

#define	SI7021_W_ADDR				0x80		//写地址
#define	SI7021_R_ADDR				0x81		//读地址
#define SI7021_SLAVE_ADDR		0x80		//设备地址

#define	HUMI_HOLD_MASTER		0xE5		//测量相对湿度，保持主模式
#define	HUMI_NOHOLD_MASTER	0xF5		//测量相对湿度，不保持主模式
#define	TEMP_HOLD_MASTER		0xE3		//测量温度，保持主模式
#define	TEMP_NOHOLD_MASTER	0xF3		//测量温度，不保持主模式

#define	READ_TEMP_from_Previous_RH		0xE0				//读取先前RH测量的温度值
#define RESET_SI7021									0xFE				//重置
#define WRITE_RH_T_USER_Register1			0xE6				//写RH/T用户寄存器1
#define READ_RH_T_USER_Register1			0xE7				//读RH/T用户寄存器1
#define	READ_Electronic_ID_1st_Byte		0xFA 0x0F		//读取电子ID第1字节
#define	READ_Electronic_ID_2nd_Byte		0xFC 0xC9		//读取电子ID第2字节	需要checksum byte
#define READ_Firmware_Revision				0x84 0xB8		//读取固件版本

float Si7021_Measure(uint8_t Cmd);
float Si7021_TEMP_Measure(void);
uint8_t Read_Si7021_Firmware_Revision(void);

#endif	/* __BSP_SI7021_H__ */

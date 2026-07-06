#include "bsp_si7021.h"
#include "bsp_i2c.h"
#include "Delay.h"

float Si7021_Measure(uint8_t Cmd)
{
	uint16_t data ;
	float value;
	I2C_Start();
	I2C_SendByte(SI7021_W_ADDR);
	if(I2C_WaitAck()) return 1;
	I2C_SendByte(Cmd);
	if(I2C_WaitAck()) return 2;
	Delay_ms(20);
	I2C_Start();
	I2C_SendByte(SI7021_R_ADDR);
	if(I2C_WaitAck()) return 3;
	data = I2C_ReadByte(1);
	data = data<<8;
	data = data + I2C_ReadByte(0);
	I2C_Stop();
	if(Cmd == TEMP_NOHOLD_MASTER)
	{
		value = 175.72 * data / 65536.0 - 46.85;
	}
	if(Cmd == HUMI_NOHOLD_MASTER)
	{
		value = 125.00f * data / 65536 - 6;
	}
	return value;
}

float Si7021_TEMP_Measure(void)
{
	uint16_t data ;
	float value;
	I2C_Start();
	I2C_SendByte(SI7021_W_ADDR);
	if(I2C_WaitAck()) return 1;
	I2C_SendByte(READ_TEMP_from_Previous_RH);
	if(I2C_WaitAck()) return 2;
	//SysTick_Delay_ms_INT(20);
	I2C_Start();
	I2C_SendByte(SI7021_R_ADDR);
	if(I2C_WaitAck()) return 3;
	data = I2C_ReadByte(1);
	data = data<<8;
	data = data + I2C_ReadByte(0);
	I2C_Stop();
	value = 175.72 * data / 65536.0 - 46.85;
	return value;
}

uint8_t Read_Si7021_Firmware_Revision(void)
{
	uint8_t FWREV=0;
	I2C_Start();
	I2C_SendByte(SI7021_W_ADDR);
	if(I2C_WaitAck()) return 1;
	I2C_SendByte(0x84);
	if(I2C_WaitAck()) return 1;
	I2C_SendByte(0xB8);
	if(I2C_WaitAck()) return 1;
	Delay_ms(20);
	I2C_Start();
	I2C_SendByte(SI7021_R_ADDR);
	if(I2C_WaitAck()) return 1;
	FWREV = I2C_ReadByte(1);
	if(I2C_WaitAck()) return 1;
	//if(!I2C_WaitAck()) return 1;
	I2C_Stop();
	return FWREV;
}

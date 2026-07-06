#include "bsp_i2c.h"

/*
**************************************************
* 函 数 名: I2C_GPIO_Config
* 功能说明: 配置I2C总线的GPIO，采用模拟IO的方式实现
* 形 参：无
* 返 回 值: 无
**************************************************
*/
void I2C_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开GPIO 时钟 */
	RCC_APB2PeriphClockCmd(SDA_GPIO_CLK|SCL_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; /* 开漏输出 */
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA,GPIO_Pin_4);
  GPIO_SetBits(GPIOA,GPIO_Pin_5);
}

/*
************************************************
* 函 数 名: I2C_Delay
* 功能说明: I2C总线位延迟，最快400KHz
* 形 参：无
* 返 回 值: 无
************************************************
*/
void I2C_Delay(void)
{
	uint8_t i;
	for(i=0;i<10;i++);	//10是一个经验值
	/*
	工作条件：CPU 主频72MHz ，MDK 编译环境，1 级优化
	循环次数为10时，SCL 频率 = 205KHz
	循环次数为7 时，SCL 频率 = 347KHz，SCL 高电平时间1.5us， SCL 低电平时间2.87us
	循环次数为5 时，SCL 频率 = 421KHz，SCL 高电平时间1.25us，SCL 低电平时间2.375us
	*/
}

/*
***********************************************
* 函 数 名: I2C_Start
* 功能说明: CPU发起I2C总线启动信号
* 形 参：无
* 返 回 值: 无
***********************************************
*/
void I2C_Start(void)
{
	/* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
	I2C_SDA_1();
	I2C_SCL_1();
	I2C_Delay();
	I2C_SDA_0();
	I2C_Delay();
	I2C_SCL_0();
}

/*
***********************************************
* 函 数 名: I2C_Stop
* 功能说明: CPU发起I2C总线停止信号
* 形 参：无
* 返 回 值: 无
***********************************************
*/
void I2C_Stop(void)
{
	/* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
	I2C_SDA_0();
	I2C_Delay();
	I2C_SCL_1();
	I2C_SDA_1();
}

/*
*************************************************
* 函 数 名: I2C_SendByte
* 功能说明: CPU向I2C总线设备发送8bit数据
* 形 参：_		ucByte ： 等待发送的字节
* 返 回 值: 无
*************************************************
*/
void I2C_SendByte(uint8_t ucByte)
{
	uint8_t i;

	/* 先发送字节的高位bit7 */
	for (i = 0; i < 8; i++)
	{
		if (ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		I2C_Delay();
		I2C_SCL_1();
		I2C_Delay();
		I2C_SCL_0();
		if (i == 7)
		{
			I2C_SDA_1(); // 释放总线
		}
		ucByte <<= 1; /* 左移一个bit */
		I2C_Delay();
	}
}

/*
**********************************************
* 函 数 名: I2C_ReadByte
* 功能说明: CPU从I2C总线设备读取8bit数据
* 形 参：		ack：	1：发送应答信号；0：发送非应答信号
* 返 回 值: 读到的数据
**********************************************
*/
uint8_t I2C_ReadByte(unsigned char ack)
{
	uint8_t i,value=0;
	for(i=0;i<8;i++ )
	{
		value<<=1;
		I2C_SCL_1();
		I2C_Delay();
		if(I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_0();
		I2C_Delay();
	}
	if (!ack)
		I2C_Nack();
	else
		I2C_Ack();
	return value;
}

/*
*************************************************
* 函 数 名: I2C_WaitAck
* 功能说明:	CPU产生一个时钟，并读取器件的ACK应答信号
* 形 参：无
* 返 回 值: 返回0表示正确应答，返回1表示无器件响应
*************************************************
*/
uint8_t I2C_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1(); /* CPU 释放SDA 总线 */
	I2C_Delay();
	I2C_SCL_1(); /* CPU 驱动SCL = 1,
	此时器件会返回ACK 应答 */
	I2C_Delay();
	if(I2C_SDA_READ())
	{ /* CPU 读取SDA 口线状态 */
		re = 1;
	} else
	{
		re = 0;
	}
	I2C_SCL_0();
	I2C_Delay();
	return re;
}

/*
************************************************
* 函 数 名: I2C_Ack
* 功能说明: CPU产生一个ACK信号
* 形 参：无
* 返 回 值: 无
************************************************
*/
void I2C_Ack(void)
{
	I2C_SDA_0(); /* CPU 驱动SDA = 0 */
	I2C_Delay();
	I2C_SCL_1(); /* CPU 产生1 个时钟 */
	I2C_Delay();
	I2C_SCL_0();
	I2C_Delay();
	I2C_SDA_1(); /* CPU 释放SDA 总线 */
}

/*
**********************************************
* 函 数 名: I2C_Nack
* 功能说明: CPU产生1个NACK信号
* 形 参：无
* 返 回 值: 无
**********************************************
*/
void I2C_Nack(void)
{
  I2C_SDA_1(); /* CPU 驱动SDA = 1 */
	I2C_Delay();
	I2C_SCL_1(); /* CPU 产生1 个时钟 */
	I2C_Delay();
	I2C_SCL_0();
	I2C_Delay();
	I2C_SDA_1(); /* CPU 释放SDA 总线 */
}

/*
***************************************************
* 函 数 名: I2C_CheckDevice
* 功能说明: 检测I2C总线设备，CPU向发送设备地址，
* 然后读取设备应答来判断该设备是否存在
* 形 参：_Address：设备的I2C总线地址
* 返 回 值: 返回值0表示正确，返回1 表示未探测到
***************************************************
*/


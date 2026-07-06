#ifndef __BSP_I2C_H__
#define __BSP_I2C_H__

#include "stm32f10x.h"

/*SCL--PA2*/
#define SCL_GPIO_PORT   GPIOA
#define SCL_GPIO_CLK 	  RCC_APB2Periph_GPIOA
#define SCL_GPIO_PIN		GPIO_Pin_4

/*SDA--PA3*/
#define SDA_GPIO_PORT   GPIOA
#define SDA_GPIO_CLK 	  RCC_APB2Periph_GPIOA
#define SDA_GPIO_PIN		GPIO_Pin_5

#define	I2C_SCL_1()			GPIO_SetBits	(SCL_GPIO_PORT,SCL_GPIO_PIN)
#define	I2C_SCL_0()			GPIO_ResetBits(SCL_GPIO_PORT,SCL_GPIO_PIN)
#define I2C_SCL_READ()	GPIO_ReadInputDataBit(SCL_GPIO_PORT,SCL_GPIO_PIN)

#define	I2C_SDA_1()			GPIO_SetBits	(SDA_GPIO_PORT,SDA_GPIO_PIN)
#define	I2C_SDA_0()			GPIO_ResetBits(SDA_GPIO_PORT,SDA_GPIO_PIN)
#define I2C_SDA_READ()	GPIO_ReadInputDataBit(SDA_GPIO_PORT,SDA_GPIO_PIN)

void 		I2C_GPIO_Config(void);
void 		I2C_Delay(void);
void 		I2C_Start(void);
void 		I2C_Stop(void);
void 		I2C_SendByte(uint8_t ucByte);
uint8_t I2C_ReadByte(unsigned char ack);
uint8_t I2C_WaitAck(void);
void 		I2C_Ack(void);
void 		I2C_Nack(void);

#endif /* __BSP_I2C_H__ */

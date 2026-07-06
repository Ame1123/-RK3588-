#ifndef _24cxx_H
#define _24cxx_H

#include "system.h"

/*
 * 24cxx —— 内部 Flash 版“EEPROM”模拟
 *
 * 说明：
 *   原工程用板载 AT24C02（I2C EEPROM）保存电阻触摸屏校准参数。该 I2C 引脚
 *   与本工程舵机4(PB6)存在潜在冲突，且原始 24cxx 驱动已不可恢复，因此改用
 *   STM32 片内 Flash 模拟 EEPROM：对外仍暴露 AT24CXX_* 接口，touch.c 无需改动。
 *
 *   数据落在片内 Flash 最后一页（STM32F103ZE = 512KB，页大小 2KB）：
 *     0x0807F800 ~ 0x0807FFFF
 *   仅在触摸校准时写入（频率极低），无擦写寿命顾虑。
 *
 * 地址空间：0 ~ (EE_SIZE-1) 字节，与 AT24C02 用法一致（touch.c 用到 200~213）。
 */

#define EE_SIZE   512   /* 模拟 EEPROM 容量（字节），须为偶数 */

void AT24CXX_Init(void);                                    /* 载入 Flash 内容到 RAM 影子区 */
u8   AT24CXX_ReadOneByte(u16 ReadAddr);                     /* 读 1 字节 */
void AT24CXX_WriteOneByte(u16 WriteAddr, u8 DataToWrite);   /* 写 1 字节（立即回写 Flash） */
void AT24CXX_WriteLenByte(u16 WriteAddr, u32 DataToWrite, u8 Len); /* 写 Len 字节（低字节在前） */
u32  AT24CXX_ReadLenByte(u16 ReadAddr, u8 Len);            /* 读 Len 字节（低字节在前） */

#endif

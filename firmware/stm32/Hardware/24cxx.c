#include "24cxx.h"
#include "stm32f10x_flash.h"

/*
 * 内部 Flash 版 EEPROM 模拟实现，详见 24cxx.h。
 *
 * 策略：RAM 影子区 s_shadow[EE_SIZE] 作为工作副本；
 *   - Init 时从 Flash 读入影子区；
 *   - 读操作直接返回影子区内容；
 *   - 写操作更新影子区并整页回写 Flash（擦除 + 半字编程）。
 * 校准写入极少发生，整页回写实现简单可靠。
 */

/* STM32F103ZE：512KB Flash，2KB/页，取最后一页 */
#define EE_FLASH_ADDR   ((u32)0x0807F800)

static u8 s_shadow[EE_SIZE];

/* 从 Flash 载入影子区 */
void AT24CXX_Init(void)
{
	u16 i;
	const volatile u8 *src = (const volatile u8 *)EE_FLASH_ADDR;
	for (i = 0; i < EE_SIZE; i++)
	{
		s_shadow[i] = src[i];
	}
}

/* 将影子区整页回写 Flash */
static void AT24CXX_Flush(void)
{
	u16 i;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	FLASH_ErasePage(EE_FLASH_ADDR);
	/* 半字（16bit）编程，EE_SIZE 保证为偶数 */
	for (i = 0; i < EE_SIZE; i += 2)
	{
		u16 halfword = (u16)s_shadow[i] | ((u16)s_shadow[i + 1] << 8);
		FLASH_ProgramHalfWord(EE_FLASH_ADDR + i, halfword);
	}
	FLASH_Lock();
}

u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{
	if (ReadAddr >= EE_SIZE) return 0;
	return s_shadow[ReadAddr];
}

void AT24CXX_WriteOneByte(u16 WriteAddr, u8 DataToWrite)
{
	if (WriteAddr >= EE_SIZE) return;
	if (s_shadow[WriteAddr] == DataToWrite) return;   /* 无变化则不必擦写 */
	s_shadow[WriteAddr] = DataToWrite;
	AT24CXX_Flush();
}

/* 写 Len(1~4) 字节，低字节在前；仅在末字节写完后回写一次 Flash */
void AT24CXX_WriteLenByte(u16 WriteAddr, u32 DataToWrite, u8 Len)
{
	u8 t;
	u8 changed = 0;
	for (t = 0; t < Len; t++)
	{
		u16 addr = WriteAddr + t;
		u8  val  = (u8)(DataToWrite >> (8 * t)) & 0xFF;
		if (addr < EE_SIZE && s_shadow[addr] != val)
		{
			s_shadow[addr] = val;
			changed = 1;
		}
	}
	if (changed) AT24CXX_Flush();
}

/* 读 Len(1~4) 字节，低字节在前 */
u32 AT24CXX_ReadLenByte(u16 ReadAddr, u8 Len)
{
	u32 temp = 0;
	u8  t;
	for (t = 0; t < Len; t++)
	{
		u16 addr = ReadAddr + (Len - 1 - t);
		temp <<= 8;
		if (addr < EE_SIZE) temp += s_shadow[addr];
	}
	return temp;
}

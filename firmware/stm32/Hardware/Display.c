#include "stm32f10x.h"
#include "Display.h"
#include "tftlcd.h"
#include "Delay.h"
#include <stdio.h>
#include <string.h>

/*
 * Display 模块 —— TFTLCD
 *
 * 屏幕规格：320 × 480（竖屏，原点在左上角，X 向右，Y 向下）
 *
 * 实时画面布局（竖屏 320 × 480）：
 *   y =   0 ~  39   蓝色标题栏  "STM32 Control"
 *   y =  50 ~ 130   舵机区域    Servo1/2/3 + 继电器
 *   y = 140 ~ 220   传感器区域  Temp / Humi
 *   y = 230 ~ 310   风速风向区  Wind Speed / Wind Dir
 *   y = 320 ~ 479   状态日志区  Display_LogMessage 滚动输出
 */

/* 颜色快捷 */
#define BG_COLOR        WHITE
#define TITLE_BG        DARKBLUE
#define TITLE_FG        WHITE
#define LABEL_FG        BLACK
#define VALUE_FG        BLUE
#define ACCENT_FG       RED
#define BORDER_FG       GRAY

/* 上一次显示的数值缓存 —— 仅在变化时刷新区域，避免闪烁 */
static float    s_last_servo1 = -999, s_last_servo2 = -999, s_last_servo3 = -999;
static uint8_t  s_last_relay  = 0xFF;
static float    s_last_temp   = -999, s_last_humi   = -999;
static float    s_last_wind   = -999;
static char     s_last_dir[4] = {0};
static uint8_t  s_ui_drawn    = 0;

/* 底部状态日志区（y = 320 ~ 479） */
#define LOG_AREA_TOP     320                 /* 日志区顶部 y */
#define LOG_AREA_BOTTOM  479                 /* 日志区底部 y（屏高 480-1） */
#define LOG_TITLE_Y      328                 /* "Log" 表头 y */
#define LOG_FIRST_Y      350                 /* 首行文本 y */
#define LOG_LINE_H       18                  /* 行高（16 号字 + 行距） */
#define LOG_MAX_LINES    7                   /* (479-350)/18 ≈ 7 行 */
#define LOG_MAX_CHARS    26                  /* 单行最多字符（16 号字宽 8px，(W-24)/8≈37，此处保守留 26） */

static char     s_log_lines[LOG_MAX_LINES][LOG_MAX_CHARS + 1];
static uint8_t  s_log_count = 0;             /* 当前已用行数 */

/* 把数值绘制到指定区域（先用背景色清矩形再画字） */
static void draw_value_int(u16 x, u16 y, u16 w, u16 h, int value, u8 size, u16 color)
{
	char buf[16];
	sprintf(buf, "%d", value);
	LCD_Fill(x, y, x + w - 1, y + h - 1, BG_COLOR);
	FRONT_COLOR = color;
	BACK_COLOR  = BG_COLOR;
	LCD_ShowString(x, y, w, h, size, (u8 *)buf);
}

static void draw_value_float1(u16 x, u16 y, u16 w, u16 h, float value, u8 size, u16 color)
{
	char buf[16];
	sprintf(buf, "%.1f", value);
	LCD_Fill(x, y, x + w - 1, y + h - 1, BG_COLOR);
	FRONT_COLOR = color;
	BACK_COLOR  = BG_COLOR;
	LCD_ShowString(x, y, w, h, size, (u8 *)buf);
}

static void draw_text(u16 x, u16 y, u16 w, u16 h, const char *str, u8 size, u16 color)
{
	LCD_Fill(x, y, x + w - 1, y + h - 1, BG_COLOR);
	FRONT_COLOR = color;
	BACK_COLOR  = BG_COLOR;
	LCD_ShowString(x, y, w, h, size, (u8 *)str);
}

/*-----------------------------------------------------------------------------
 * 开机动画
 *---------------------------------------------------------------------------*/
void Display_BootAnimation(void)
{
	u16 i;

	/* 1. 清屏 + 渐进式标题 */
	LCD_Clear(BLACK);

	FRONT_COLOR = WHITE;
	BACK_COLOR  = BLACK;
	LCD_ShowString(110, 100, 200, 30, 24, (u8 *)"STM32");
	Delay_ms(150);

	FRONT_COLOR = CYAN;
	LCD_ShowString(60, 140, 220, 30, 24, (u8 *)"Control System");
	Delay_ms(300);

	/* 2. 进度条 */
	FRONT_COLOR = WHITE;
	LCD_ShowString(120, 200, 200, 16, 16, (u8 *)"LOADING...");

	/* 进度条边框 */
	FRONT_COLOR = WHITE;
	LCD_DrawRectangle(40, 230, 280, 254);

	/* 进度条内填充 */
	for (i = 0; i <= 100; i += 5)
	{
		u16 fillEnd = 42 + (236 * i / 100);
		LCD_Fill(42, 232, fillEnd, 252, GREEN);

		/* 进度百分比 */
		{
			char buf[8];
			sprintf(buf, "%3d%%", i);
			FRONT_COLOR = YELLOW;
			BACK_COLOR  = BLACK;
			LCD_ShowString(135, 270, 60, 16, 16, (u8 *)buf);
		}
		Delay_ms(20);
	}
	Delay_ms(150);

	/* 3. 系统检测列表 */
	LCD_Clear(BLACK);
	FRONT_COLOR = WHITE;
	BACK_COLOR  = BLACK;
	LCD_ShowString(80, 60, 200, 24, 24, (u8 *)"System Init");

	LCD_ShowString(40, 120, 200, 16, 16, (u8 *)"Servo  ......");
	Delay_ms(150);
	FRONT_COLOR = GREEN;
	LCD_ShowString(170, 120, 60, 16, 16, (u8 *)"OK");

	FRONT_COLOR = WHITE;
	LCD_ShowString(40, 150, 200, 16, 16, (u8 *)"Sensor ......");
	Delay_ms(150);
	FRONT_COLOR = GREEN;
	LCD_ShowString(170, 150, 60, 16, 16, (u8 *)"OK");

	FRONT_COLOR = WHITE;
	LCD_ShowString(40, 180, 200, 16, 16, (u8 *)"Comm   ......");
	Delay_ms(150);
	FRONT_COLOR = GREEN;
	LCD_ShowString(170, 180, 60, 16, 16, (u8 *)"OK");

	Delay_ms(300);

	/* 4. READY 闪烁 */
	for (i = 0; i < 3; i++)
	{
		FRONT_COLOR = YELLOW;
		BACK_COLOR  = BLACK;
		LCD_ShowString(110, 260, 200, 24, 24, (u8 *)"READY!");
		Delay_ms(200);
		LCD_Fill(110, 260, 250, 290, BLACK);
		Delay_ms(120);
	}

	/* 5. 标记需要在下次 UpdateData 时绘制静态 UI */
	s_ui_drawn = 0;
}

/*-----------------------------------------------------------------------------
 * 静态界面（标题、表头、分隔线）
 *---------------------------------------------------------------------------*/
void Display_DrawStaticUI(void)
{
	u16 W = tftlcd_data.width;

	LCD_Clear(BG_COLOR);

	/* 标题栏 */
	LCD_Fill(0, 0, W - 1, 39, TITLE_BG);
	FRONT_COLOR = TITLE_FG;
	BACK_COLOR  = TITLE_BG;
	LCD_ShowString(10, 8, W - 20, 24, 24, (u8 *)"STM32 Control");

	/* 区块边框与表头 */
	FRONT_COLOR = BORDER_FG;
	LCD_DrawRectangle(5, 50, W - 6, 130);                    /* 舵机区 */
	LCD_DrawRectangle(5, 140, W - 6, 220);                   /* 温湿度区 */
	LCD_DrawRectangle(5, 230, W - 6, 310);                   /* 风速风向区 */
	LCD_DrawRectangle(5, LOG_AREA_TOP, W - 6, LOG_AREA_BOTTOM); /* 状态日志区 */

	/* 舵机区表头 */
	FRONT_COLOR = LABEL_FG;
	BACK_COLOR  = BG_COLOR;
	LCD_ShowString(15, 58, 200, 16, 16, (u8 *)"Servo & Relay");

	LCD_ShowString(15, 80, 60, 16, 16, (u8 *)"S1:");
	LCD_ShowString(110, 80, 60, 16, 16, (u8 *)"S2:");
	LCD_ShowString(210, 80, 60, 16, 16, (u8 *)"S3:");
	LCD_ShowString(15, 105, 60, 16, 16, (u8 *)"Relay:");

	/* 温湿度区表头 */
	LCD_ShowString(15, 148, 200, 16, 16, (u8 *)"Temp & Humidity");

	LCD_ShowString(15, 175, 60, 24, 24, (u8 *)"T:");
	LCD_ShowString(170, 175, 60, 24, 24, (u8 *)"H:");

	/* 风速风向区表头 */
	LCD_ShowString(15, 238, 200, 16, 16, (u8 *)"Wind");

	LCD_ShowString(15, 265, 60, 24, 24, (u8 *)"WS:");
	LCD_ShowString(170, 265, 60, 24, 24, (u8 *)"WD:");

	/* 状态日志区表头 */
	FRONT_COLOR = LABEL_FG;
	BACK_COLOR  = BG_COLOR;
	LCD_ShowString(15, LOG_TITLE_Y, 200, 16, 16, (u8 *)"Log");

	/* 重绘已缓存的日志行（静态 UI 被重画后需要恢复） */
	{
		u8 i;
		FRONT_COLOR = VALUE_FG;
		for (i = 0; i < s_log_count; i++)
		{
			LCD_ShowString(15, LOG_FIRST_Y + i * LOG_LINE_H, W - 24, 16, 16,
			               (u8 *)s_log_lines[i]);
		}
	}

	/* 重置缓存，强制下次 UpdateData 全部刷一遍 */
	s_last_servo1 = s_last_servo2 = s_last_servo3 = -999;
	s_last_relay  = 0xFF;
	s_last_temp   = s_last_humi = -999;
	s_last_wind   = -999;
	s_last_dir[0] = 0;

	s_ui_drawn = 1;
}

/*-----------------------------------------------------------------------------
 * 实时数据更新
 *---------------------------------------------------------------------------*/
void Display_UpdateData(float servo1, float servo2, float servo3, uint8_t relay,
                        float temp, float humi, float wind_speed, const char* wind_dir)
{
	if (!s_ui_drawn)
	{
		Display_DrawStaticUI();
	}

	/* 舵机角度（每个 60 像素宽，16 号字够容纳 "180"） */
	if (servo1 != s_last_servo1)
	{
		draw_value_int(45, 80, 60, 16, (int)servo1, 16, VALUE_FG);
		s_last_servo1 = servo1;
	}
	if (servo2 != s_last_servo2)
	{
		draw_value_int(140, 80, 60, 16, (int)servo2, 16, VALUE_FG);
		s_last_servo2 = servo2;
	}
	if (servo3 != s_last_servo3)
	{
		draw_value_int(240, 80, 60, 16, (int)servo3, 16, VALUE_FG);
		s_last_servo3 = servo3;
	}

	/* 继电器状态（彩色 ON/OFF） */
	if (relay != s_last_relay)
	{
		if (relay)
		{
			draw_text(80, 105, 80, 16, "ON ", 16, GREEN);
		}
		else
		{
			draw_text(80, 105, 80, 16, "OFF", 16, GRAY);
		}
		s_last_relay = relay;
	}

	/* 温度（带一位小数） */
	if (temp != s_last_temp)
	{
		draw_value_float1(50, 175, 100, 24, temp, 24, ACCENT_FG);
		s_last_temp = temp;
	}

	/* 湿度 */
	if (humi != s_last_humi)
	{
		draw_value_float1(205, 175, 100, 24, humi, 24, VALUE_FG);
		s_last_humi = humi;
	}

	/* 风速 */
	if (wind_speed != s_last_wind)
	{
		draw_value_float1(60, 265, 100, 24, wind_speed, 24, VALUE_FG);
		s_last_wind = wind_speed;
	}

	/* 风向 */
	if (wind_dir != 0 && strncmp(wind_dir, s_last_dir, 3) != 0)
	{
		draw_text(215, 265, 90, 24, wind_dir, 24, ACCENT_FG);
		strncpy(s_last_dir, wind_dir, 3);
		s_last_dir[3] = 0;
	}
}

/*-----------------------------------------------------------------------------
 * 底部状态日志：追加一行，写满后向上滚动
 *---------------------------------------------------------------------------*/
void Display_LogMessage(const char* msg)
{
	u16 W = tftlcd_data.width;
	u8  i;

	if (msg == 0)
	{
		return;
	}

	/* 静态 UI 还没画时先补画一次，保证日志区边框/表头存在 */
	if (!s_ui_drawn)
	{
		Display_DrawStaticUI();
	}

	/* 已满则整体上移一行，腾出最后一行 */
	if (s_log_count >= LOG_MAX_LINES)
	{
		for (i = 0; i < LOG_MAX_LINES - 1; i++)
		{
			strcpy(s_log_lines[i], s_log_lines[i + 1]);
		}
		s_log_count = LOG_MAX_LINES - 1;
	}

	/* 拷贝新行（截断到最大字符数） */
	strncpy(s_log_lines[s_log_count], msg, LOG_MAX_CHARS);
	s_log_lines[s_log_count][LOG_MAX_CHARS] = 0;
	s_log_count++;

	/* 清空文本区并重绘所有行（区域小，整体重绘最简单可靠） */
	LCD_Fill(6, LOG_FIRST_Y, W - 7, LOG_AREA_BOTTOM - 1, BG_COLOR);
	FRONT_COLOR = VALUE_FG;
	BACK_COLOR  = BG_COLOR;
	for (i = 0; i < s_log_count; i++)
	{
		LCD_ShowString(15, LOG_FIRST_Y + i * LOG_LINE_H, W - 24, 16, 16,
		               (u8 *)s_log_lines[i]);
	}
}

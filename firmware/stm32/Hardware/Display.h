#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "stm32f10x.h"

/*
 * Display 模块（TFTLCD）
 *  - 屏幕：3.5寸 320x480 彩屏（HX8357DN，竖屏 320 宽 × 480 高）
 *  - 由 main.c 调用：开机动画 + 实时数据刷新
 */

/* 开机动画（Logo + 进度条 + 自检列表） */
void Display_BootAnimation(void);

/* 实时数据界面：清屏并绘制静态背景与表头（首次进入时调用一次即可） */
void Display_DrawStaticUI(void);

/* 更新实时数据显示（内部按脏值判断，仅变化项重绘，避免闪烁） */
void Display_UpdateData(float servo1, float servo2, float servo3, uint8_t relay,
                        float temp, float humi, float wind_speed, const char* wind_dir);

/* 在底部预留区追加一行状态日志（自动滚动，最多保留若干行） */
void Display_LogMessage(const char* msg);

#endif

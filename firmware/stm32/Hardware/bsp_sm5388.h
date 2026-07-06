#ifndef __BSP_SM5388_H
#define __BSP_SM5388_H

#include "stm32f10x.h"

// ============ 软件滤波配置 ============
// ADC采样次数(8-32推荐，次数越多精度越高但速度越慢)
#define SM5388_ADC_SAMPLE_COUNT  16

// 低通滤波权重(0.5-0.9推荐，越大响应越快但抖动越大)
#define SM5388_FILTER_WEIGHT     0.7f

// 分压比例(根据实际电路调整)
// 分压电路: R1=10kΩ, R2=20kΩ
// Vout = Vin * R2/(R1+R2) = Vin * 0.6667
// 反推系数: 1/0.6667 = 1.5
#define SM5388_VOLTAGE_RATIO     1.5f

// ======================================

// SM5388风速风向传感器初始化
void SM5388_Init(void);

// 读取风速(m/s)
float SM5388_GetWindSpeed(void);

// 读取风向(度)
float SM5388_GetWindDirection(void);

// 读取风向(方位字符串: N/NE/E/SE/S/SW/W/NW)
const char* SM5388_GetWindDirectionString(void);

// 风向校准(可选) - 手动设置8个方向的校准偏移值
void SM5388_SetDirectionCalibration(float north_offset, float east_offset, 
                                     float south_offset, float west_offset);

#endif

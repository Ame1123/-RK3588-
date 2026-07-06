#include "bsp_sm5388.h"

/**
  * SM5388风速风向传感器驱动 - 增强版
  * 使用ADC读取模拟电压
  * PA6 - 风速电压输入 (ADC1_IN6)
  * PA7 - 风向电压输入 (ADC1_IN7)
  * 
  * 计算公式:
  * 风速 = (电压 - 0) * 30 / 5
  * 风向 = (电压 - 0) * 360 / 5
  * 
  * 优化措施:
  * 1. 多次采样平均 - 减少随机噪声
  * 2. 去除极值 - 抑制脉冲干扰
  * 3. 低通滤波 - 平滑数据变化
  * 4. 校准补偿 - 修正系统误差
  */

// 校准偏移值(度)
static float calibration_offset[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // N, E, S, W

/**
  * 函数: SM5388_SetDirectionCalibration
  * 功能: 设置风向校准偏移值
  * 参数: north_offset - 北方向偏移(度)
  *       east_offset  - 东方向偏移(度)
  *       south_offset - 南方向偏移(度)
  *       west_offset  - 西方向偏移(度)
  * 说明: 如果发现某个方向测量值偏差,可通过此函数校准
  *       例如: 北方向实际0度显示5度,则设置north_offset=-5
  */
void SM5388_SetDirectionCalibration(float north_offset, float east_offset, 
                                     float south_offset, float west_offset)
{
	calibration_offset[0] = north_offset;
	calibration_offset[1] = east_offset;
	calibration_offset[2] = south_offset;
	calibration_offset[3] = west_offset;
}

/**
  * 函数: SM5388_Init
  * 功能: 初始化ADC用于读取SM5388传感器
  */
void SM5388_Init(void)
{
	// 开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	// 配置ADC时钟分频(不超过14MHz)
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); // 72MHz/6 = 12MHz
	
	// 配置GPIO为模拟输入
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// 配置ADC
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	// 使能ADC
	ADC_Cmd(ADC1, ENABLE);
	
	// ADC校准
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
}

/**
  * 函数: SM5388_ReadADC
  * 功能: 读取指定ADC通道的值(带多次采样平均)
  * 参数: channel - ADC通道号
  * 返回: ADC采样值(0-4095)
  */
static uint16_t SM5388_ReadADC(uint8_t channel)
{
	uint32_t sum = 0;
	uint16_t samples[SM5388_ADC_SAMPLE_COUNT];
	uint8_t i, j;
	
	// 配置通道
	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_55Cycles5);
	
	// 采集多次数据
	for(i = 0; i < SM5388_ADC_SAMPLE_COUNT; i++)
	{
		// 启动转换
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		
		// 等待转换完成
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
		
		// 读取结果
		samples[i] = ADC_GetConversionValue(ADC1);
	}
	
	// 冒泡排序(用于中值滤波)
	for(i = 0; i < SM5388_ADC_SAMPLE_COUNT - 1; i++)
	{
		for(j = 0; j < SM5388_ADC_SAMPLE_COUNT - 1 - i; j++)
		{
			if(samples[j] > samples[j + 1])
			{
				uint16_t temp = samples[j];
				samples[j] = samples[j + 1];
				samples[j + 1] = temp;
			}
		}
	}
	
	// 去掉最大最小值，对中间值求平均(避免极端值影响)
	for(i = 2; i < SM5388_ADC_SAMPLE_COUNT - 2; i++)
	{
		sum += samples[i];
	}
	
	return (uint16_t)(sum / (SM5388_ADC_SAMPLE_COUNT - 4));
}

/**
  * 函数: SM5388_GetWindSpeed
  * 功能: 读取风速(m/s)
  * 返回: 风速值(0-30m/s)
  */
float SM5388_GetWindSpeed(void)
{
	uint16_t adc_value = SM5388_ReadADC(ADC_Channel_6);
	
	// 转换为电压(0-3.3V对应0-4095)
	float voltage = (float)adc_value * 3.3f / 4095.0f;
	
	// 转换为风速(0-5V对应0-30m/s)
	// 由于ADC参考电压是3.3V，需要做电压转换
	// 如果外部有分压电路，则: voltage_actual = voltage * 5.0 / 3.3
	// 这里假设使用分压电路将0-5V转换为0-3.3V
	float voltage_actual = voltage * 5.0f / 3.3f;
	
	// 计算风速
	float wind_speed = voltage_actual * 30.0f / 5.0f;
	
	return wind_speed;
}

/**
  * 函数: SM5388_GetWindDirection
  * 功能: 读取风向(度) - 带软件校准和滤波
  * 返回: 风向值(0-360度)
  */
float SM5388_GetWindDirection(void)
{
	static float last_direction = 0.0f;  // 上次测量值(用于平滑)
	
	uint16_t adc_value = SM5388_ReadADC(ADC_Channel_7);
	
	// 转换为电压(0-3.3V对应0-4095)
	float voltage = (float)adc_value * 3.3f / 4095.0f;
	
	// 转换为实际电压(使用配置的分压比例)
	float voltage_actual = voltage * SM5388_VOLTAGE_RATIO;
	
	// 计算风向
	float wind_direction = voltage_actual * 360.0f / 5.0f;
	
	// 应用校准偏移(根据角度范围选择偏移值)
	float offset = 0.0f;
	if (wind_direction >= 315.0f || wind_direction < 45.0f)
		offset = calibration_offset[0];  // 北
	else if (wind_direction >= 45.0f && wind_direction < 135.0f)
		offset = calibration_offset[1];  // 东
	else if (wind_direction >= 135.0f && wind_direction < 225.0f)
		offset = calibration_offset[2];  // 南
	else
		offset = calibration_offset[3];  // 西
	
	wind_direction += offset;
	
	// 一阶低通滤波(使用配置的权重)
	wind_direction = wind_direction * SM5388_FILTER_WEIGHT + last_direction * (1.0f - SM5388_FILTER_WEIGHT);
	
	// 限制在0-360度范围内
	while (wind_direction >= 360.0f) wind_direction -= 360.0f;
	while (wind_direction < 0.0f) wind_direction += 360.0f;
	
	// 保存本次测量值
	last_direction = wind_direction;
	
	return wind_direction;
}

/**
  * 函数: SM5388_GetWindDirectionString
  * 功能: 读取风向(方位字符串)
  * 返回: 风向方位字符串 (N/NE/E/SE/S/SW/W/NW)
  * 说明: 将360度分为8个方位，每个方位45度
  *       北东北、东东北归并到东北(NE)
  *       其他细分方位同理归并
  */
const char* SM5388_GetWindDirectionString(void)
{
	float angle = SM5388_GetWindDirection();
	
	// 8个主方位，每个方位45度
	// 北(N): 337.5-22.5度
	// 东北(NE): 22.5-67.5度
	// 东(E): 67.5-112.5度
	// 东南(SE): 112.5-157.5度
	// 南(S): 157.5-202.5度
	// 西南(SW): 202.5-247.5度
	// 西(W): 247.5-292.5度
	// 西北(NW): 292.5-337.5度
	
	if (angle >= 337.5f || angle < 22.5f)
	{
		return "N";   // 北
	}
	else if (angle >= 22.5f && angle < 67.5f)
	{
		return "NE";  // 东北 (包含北东北22.5-45、东北45、东东北45-67.5)
	}
	else if (angle >= 67.5f && angle < 112.5f)
	{
		return "E";   // 东
	}
	else if (angle >= 112.5f && angle < 157.5f)
	{
		return "SE";  // 东南
	}
	else if (angle >= 157.5f && angle < 202.5f)
	{
		return "S";   // 南
	}
	else if (angle >= 202.5f && angle < 247.5f)
	{
		return "SW";  // 西南
	}
	else if (angle >= 247.5f && angle < 292.5f)
	{
		return "W";   // 西
	}
	else // angle >= 292.5f && angle < 337.5f
	{
		return "NW";  // 西北
	}
}

/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2024 YaoYandong,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : my_math.c  
  ** @Brief         : 数学函数
  **--------------------------------------------------------------------------**
  ** @Author Data   : Depressed 2024-04-10
  ** @Version       : v1.0				
  **--------------------------------------------------------------------------**
  ** @Modfier Data  : None
  ** @Version       : None
  ** @Description   : None
  **--------------------------------------------------------------------------**
  ** @Function List : None
  **--------------------------------------------------------------------------**
  ** @Attention     : None
  **--------------------------------------------------------------------------**
  ******************************************************************************
  *
 **/
#include "my_math.h"

// 四舍五入float_to_uint8
uint8_t float_to_uint8_round(float f)
{
	// 加0.5后转换为整数实现四舍五入
	uint32_t temp = (uint32_t)(f + 0.5f);

	// 确保值在0-255范围内
	if (temp > 255)
		return 255;

	return (uint8_t)temp;
}

uint8_t float_2_u8(float source,float source_max,float source_min,uint8_t target_max,uint8_t target_min,bool flip)
{
	float proportion = 0;
	uint8_t ret;
	float temp;
	if(source_max<= source_min)
	{
		return 0;
	}
	temp = CONSTRAIN(source,source_max,source_min);
	proportion = (float)(temp - source_min)/(float)(source_max - source_min);
	if(flip)
	{
		ret  = (target_max-target_min)*(1-proportion)+target_min;
	}else
	{
		ret  = (proportion*(target_max-target_min))+target_min;
	}
	return ret;
}


// 初始化滤波器
void initLowPassFilter(LowPassFilter* filter, float alpha) {
    filter->alpha = alpha;
    filter->prev_output = 0.0f;  // 初始滤波输出
}

// 一阶低通滤波函数
float lowPassFilter(LowPassFilter* filter, float input) {
    // 根据滤波公式计算输出
    filter->prev_output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;
    return filter->prev_output;
}
//清除滤波器数据
void clearLowPassFilter(LowPassFilter* filter) {
    filter->prev_output = 0.0f;  // 初始滤波输出
}
void init_Mean_Filter(Mean_Filter *pHandle,bool filter_volid,uint8_t depth)
{
	pHandle->filter_volid = filter_volid;
	pHandle->first_in = true;
	pHandle->filter_depth_maximum = 17;
	
	pHandle->filter_depth = CONSTRAIN(depth,pHandle->filter_depth_maximum,0);
	pHandle->data_index = 0;
}
void clear_mean_filter(Mean_Filter *pHandle)
{
	memset(pHandle->data,0,17);
	pHandle->data_index = 0;
	pHandle->first_in = true;
}
float mean_filter(Mean_Filter *pHandle,float input)
{
	if(pHandle->filter_volid == true)
	{
		if(pHandle->first_in == true)
		{
			memset(pHandle->data,input,pHandle->filter_depth);
			pHandle->first_in = false;
		}else
		{
			pHandle->data[pHandle->data_index] = input;
			if(pHandle->data_index<pHandle->filter_depth)
				{
					pHandle->data_index++;
				}else
				{
					pHandle->data_index = 0;
				}
		}
		pHandle->sum = 0;
		for(int i = 0;i<pHandle->filter_depth;i++)
		{
			pHandle->sum += pHandle->data[i];
		}
		pHandle->prev_output = pHandle->sum/pHandle->filter_depth;
	}
	else
	{
		clear_mean_filter(pHandle);
		return input;
	}
	return pHandle->prev_output;
}














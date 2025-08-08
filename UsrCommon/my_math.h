/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2024 YaoYandong,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : my_math.h  
  ** @Description   : None
  **--------------------------------------------------------------------------**
  ** @Author        : Depressed	  
  ** @Version       : v1.0				
  ** @Creat Date    : 2024-04-10  
  **--------------------------------------------------------------------------**
  ** @Modfier       : None
  ** @Version       : None
  ** @Modify Date   : None
  ** @Description   : None
  **--------------------------------------------------------------------------**
  ** @Function List : None
  **--------------------------------------------------------------------------**
  ** @Attention     : None
  **--------------------------------------------------------------------------**
  ******************************************************************************
  *
 **/
 
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __MY_MATH_H_
#define __MY_MATH_H_
#include "stdint.h"
#include "stdbool.h"
#include "math.h"
/**
  * @brief  Macro to compute logarithm of two
  */
#define LOG2(x) \
  ((x) == 65535 ? 16 : \
   ((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2*2*2 ? 15 : \
    ((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2*2 ? 14 : \
     ((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2 ? 13 : \
      ((x) == 2*2*2*2*2*2*2*2*2*2*2*2 ? 12 : \
       ((x) == 2*2*2*2*2*2*2*2*2*2*2 ? 11 : \
        ((x) == 2*2*2*2*2*2*2*2*2*2 ? 10 : \
         ((x) == 2*2*2*2*2*2*2*2*2 ? 9 : \
          ((x) == 2*2*2*2*2*2*2*2 ? 8 : \
           ((x) == 2*2*2*2*2*2*2 ? 7 : \
            ((x) == 2*2*2*2*2*2 ? 6 : \
             ((x) == 2*2*2*2*2 ? 5 : \
              ((x) == 2*2*2*2 ? 4 : \
               ((x) == 2*2*2 ? 3 : \
                ((x) == 2*2 ? 2 : \
                 ((x) == 2 ? 1 : \
                  ((x) == 1 ? 0 : -1)))))))))))))))))

/**
  * @brief  Trigonometrical functions type definition
  */

  uint8_t float_to_uint8_round(float f);

/*限幅*/
#define CONSTRAIN(x,max,min) (x>max?max:(x<min?min:x))
#define MAX_LIM(a,a_max) (a>a_max?a_max:a)//限制不大于a_max
#define MIN_LIM(a,a_min) (a<a_min?a_min:a)//限制不小于a_min

#define GET_MAXIMUM(a,b) (a>b?a:b)//取两者最大值
#define GET_MINIMUM(a,b) (a<b?a:b)//取两者最小值
/*求绝对值*/
#define ABS(x) ((x) < 0 ? -(x) : (x))

/*时间戳*/
typedef struct
{
	volatile uint32_t last_time;				//上一次接收指令的时刻
	volatile uint32_t current_time;		//本次接收指令的时刻
	volatile uint32_t time_interval;   //指令间隔时间
	
	uint32_t timeout_interval;//指令接收超时时间
	bool timeout_flag;				//超时标志
	bool cacl_speed_step;			//需要计算一次速度
}Time_Stamp;

typedef struct {
    float alpha;    // 滤波系数
    float prev_output;  // 上一时刻的滤波输出
} LowPassFilter;


typedef struct
{
	bool first_in;
	bool filter_volid;		//滤波器输出要是无效的话数据直接输出，并置起first_in_flag;
	uint8_t filter_depth;
	uint8_t filter_depth_maximum;
	uint8_t data_index;	//
	float data[17];
	float sum;
	float prev_output;  // 上一时刻的滤波输出
}Mean_Filter;

uint32_t get_cycle(Time_Stamp* pTime_Stamp);
void timeout_detection(Time_Stamp* pTime_Stamp);
void initLowPassFilter(LowPassFilter* filter, float alpha);
float lowPassFilter(LowPassFilter* filter, float input);
void clearLowPassFilter(LowPassFilter* filter);

void init_Mean_Filter(Mean_Filter *pHandle,bool filter_volid,uint8_t depth);
void clear_mean_filter(Mean_Filter *pHandle);
float mean_filter(Mean_Filter *pHandle,float input);
uint8_t float_2_u8(float source,float source_max,float source_min,uint8_t target_max,uint8_t target_min,bool flip);

#endif


/******************************** END OF FILE *********************************/


 


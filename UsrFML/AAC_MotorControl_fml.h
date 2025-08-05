/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2024 YaoYandong,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : Inspire_MotorControl_fml.h  
  ** @Description   : None
  **--------------------------------------------------------------------------**
  ** @Author        : Depressed	  
  ** @Version       : v1.0				
  ** @Creat Date    : 2025-06-06  
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

#ifndef __INSPIRE__MOTOR_CONTROL_FML_H_
#define __INSPIRE__MOTOR_CONTROL_FML_H_
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "AAC_drv.h"
#include "upper_comm_protocol_fml.h"
#include "my_math.h"

/*参数定义区域开始*/
/*1默认位置*/
#define DEFAULT_POSITION 1000

/*2最小限位*/
#define MIN_LIMIT_POSITION 0

/*3最大限位*/
#define MAX_LIMIT_POSITION 1600

/*4最大过流阈值值设置*/
#define OVER_CURRENT_TH_MAX 3000

/*5最小过流阈值设置*/
#define OVER_CURRENT_TH_MIN 300

/*6过流阈值*/
#define OVER_CURRENT_TH 2000

/*7过温保护设置*/
#define OVER_TEMP_TH 800

/*8回温启动设置*/
//手指过温阈值定义
#define RECOVER_TEMP (OVER_TEMP_TH-50) 	

//手指最大速度定义
#define SPEED_MAX 100

/*10最小速度步进值设置*/
#define SPEED_MIN 0.105

/*11最小速度步进值设置*/
//手指默认速度定义//
#define DEFAULT_SPEED (150*SPEED_MIN)     

/*12手指运动角度定义*/
/*
tip可达最大区域
|
|_____________
|							\
|							 \ ----->死区
|								\
|								 |
|________________|_____________>pitch
             |
					TIP_DEAD_ZONE	
关联变量 TIP_DEAD_ZONE
*/

/******************************************手指过流阈值定义**************************/
#define PITCH_ANGLE_MAX 			90.0
#define PITCH_ANGLE_MIN 			0.0
#define PITCH_POS_MIN			 	0 

/******************************************手指运动角度******************************/
#define PITCH_DEAD_ZONE      0.50


//转换系数 
//ACC 16mm 行程电机
#define ANGLE_2_POS 					1600.0/90.0
#define POS_2_ANGLE 					90.0/1600.0

//ACC 10mm行程电机
#define ANGLE_2_POS_10       1600.0/90.0   
#define POS_2_ANGLE_10       90.0/1600.0


#define ROLL_ANGLE_MAX 			90.0
#define ROLL_ANGLE_MIN 			0.0
#define ROLL_POS_MIN 				0 
 

/*********************************************************************/

#define OVER_CURRENT_DETECTION_TH 5		//电机过流检测连续检测阈值
#define MOTOR_CONTROL_TIMEOUT_INTERVAL_YS 200*100 //单位0.01ms,手指位置跟随控制超时时间
#define HAND_CONTROL_TIMEOUT_INTERVAL_YS 200*100  //单位0.01ms,手指位置跟随控制超时时间

#define FOLLOW_POS_SYNC 3
#define FOLLOW_POS_TIMEOUT_TH 40	//单位ms
#define POS_SYNC_POSITION_MODE 0
#define POS_SYNC_FOLLOW_MODE 1

#define	 YS_MCC_FILTER_DEPTH 17

/*参数定义区域结束*/
typedef enum
{
	INVALID_YS_ID = 0,
	YS_ID_1 = 1,
	YS_ID_2 = 2,
	YS_ID_3 = 3,
	YS_ID_4 = 4,
	YS_ID_5 = 5,
	YS_ID_6 = 6,
	YS_ID_MAXIMUM,
}YS_ACTUATOR_ID;
/*位置控制模式*/
typedef enum
{
	YS_INVALID_POSITION_MODE = 0,
	YS_POSITION_MODE = 1,						//位置模式
	RS_FOLLOW_MODE = 2							//跟随模式
}POSITIN_CONTROL_MODE;	

/*初始化数据*/
typedef struct
{
	uint16_t default_pos;  			//如果自动回写失败，读不到电机位置,则将该位置作为默认目标位置
	uint16_t pos_limit_min;			//底层位置限制值，要对发指令前的位置设定做出限制，该值一般情况下不改变，代表着推杆在应用场景中到达不了的位置，不考虑组合联动破坏
	uint16_t pos_limit_max;			//底层位置限制值，要对发指令前的位置设定做出限制，该值一般情况下不改变，代表着推杆在应用场景中到达不了的位置，不考虑组合联动破坏
	uint16_t over_current_th;		//过流保护默认设置值
	uint16_t overtemp_th;				//过温保护默认设置
	uint16_t recover_temp_th;		//回温启动默认设置
	uint16_t current_max;				//过流保护最大值
	uint16_t current_min;				//过流保护最小值
	float speed_max;						//斜坡控速最大
	float speed_min;						//斜坡控速最小			
}YS_init;	

/*指令*/
typedef struct
{
	int16_t position_ref;								//位置设置指令，该指令不一定会向下设置，int16_t是必须的，方便计算航向角差分值
	uint16_t last_position_ref;					//上一次位置实际设定值
	uint16_t position_ref_real;					//限幅后的输出
	float slope_position_f;							//中间计算值浮点,用于斜坡控制，速度控制//本地变量
	float speed_step;										//速度加减步进值设定值
	float speed_step_real;							//速度加减步进值实际输出值
	int16_t over_current_th_ref;				//过流保护设置，该值作为堵转保护的实现。
	int16_t last_over_current_th_ref;	
	uint16_t temperature_th;
	bool clear_fault;										//清除故障
	
	uint8_t follow_pos_sync_con;				//位置跟随模式控制
	uint8_t control_mode;								//控制模式，0：位置模式，1：跟随模式
}YS_Cmd;
typedef enum
{
	LOOCKED_ROTOR_BIT = 0,		     //电机堵转
	//OVERTEMP_BIT = 1,					   //电机过温
	OVER_CURRENT_BIT = 1,			     //电机过流
	MOTOR_ABNORMAL_BIT = 2,				 //电机异常(失速)
	VOLATABNORMAL_BIT=3,           //电压异常
	CURRENT_ABNORMAL_BIT=4,        //电流自检异常
	POS_CHECK_ABNORMAL_BIT =5,     //位置自检异常
	SOFT_OVER_CURRENT_BIT = 6,		 //软件判断过流
}YS_Fault_Code_Bit;

/*状态，反馈数据*/
typedef struct
{
	int16_t last_position;			//上一控制周期位置，用于计算电机速度
	int16_t current_position;		//当前位置，向外反馈
	int16_t current;						//电流，向外反馈
	uint8_t fault_code;					//错误码。向外反馈
	int16_t speed;							//增量计算出的速度
	int8_t temperature;					//温度
}YS_Sta;
/*过流检测*/
typedef struct
{
	uint16_t count;									//过流事件计数
	uint16_t count_th;							//连续计数阈值
	int16_t real_over_current_th;	//根据运行速度补偿后的过流检测判断阈值，
	int8_t oc_dir;									//过流时指令方向记录，正数正向，负数反向
	int8_t last_oc_dir;							//上一次的过流时指令方向记录，正数正向，负数反向
	bool en_forword;								//允许正向运动
	bool en_backword;								//允许反向运动
	bool block;											//已经处于过流停机状态，该标志位置起不再判断过流
	uint16_t ov_position;						//过流位置
}YS_OC_Detection;
typedef struct
{
	bool en_forword;								//允许正向运动
	bool en_backword;								//允许反向运动
	bool block;											//已经处于过流停机状态，该标志位置起不再判断过流
}YS_OC2_Detection;
typedef struct
{
	//两种思路
	
	//思路1，给定轨迹是连续的，获取给定轨迹上一时刻的速度，如果这个速度低于设定速度，主动降低电机运动速度（降的比理论值小一些）
	
	//思路2，计算当前给定位置任务需要耗费的时间，如果在手控制周期内按设定速度到达不了，则保持设定速度，如果能到达了，降低设定速度
	bool enable_speed_adj;
	uint16_t motor_pos_real;					//考虑返回滞后性的当前tip电机值
	float control_cycle_ratio;
	bool slacken_flag;									//降速标志
	int16_t pos_inc;									//指尖位置增量
	float cmd_inc_speed_step;					//本次位置指令与上次位置指令的差计算得到的速度
	float cmd_cur_pos_speed_step;			//本次位置指令与当前位置的差计算得到的速度
	
	float adj_speed_step;							//综合前两者最终的调节速度
	//为了速度连续得到的期望速度
}Speed_Adjustment_ys;


/*因时推杆电机*/
typedef struct
{
	bool is_actuator_valid;					//电机有效性设置				
	YS_ACTUATOR_ID id;							//电机id
	bool is_actuator_online;				//电机通信层面是否在线
	bool en_control_freeze;					//是否开启控制冻结功能，指令没有变化的话，不去控制电机，减少指令帧数量，节省时间
	POSITIN_CONTROL_MODE pos_mode; 	//电机位置控制模式
	YS_init init;										//初始化
	YS_Cmd *cmd;										//命令
	YS_Sta *sta;										//状态
	YS_OC_Detection *oc;						//过流
	YS_OC2_Detection *oc2;						//过流
	Time_Stamp *motor_control_cycle;	//电机控制周期
	LowPassFilter speed_step_real_filter;
	Mean_Filter MCC_mean_filter;					//电机控制周期滤波器	
	Speed_Adjustment_ys speed_adj;
	float mean_cmd_time_interval;
	Mean_Filter speed_mean_filter;
}YS_Actuator_Unit;
/*
大拇指由四个因时推杆控制，控制指尖的定为A，控制根部的定为B，控制yaw和roll的差分对为C和D
其它手指均由3个因时推杆控制，从大拇指方向向手托方向依次定位ABC,B负责指尖的控制，控制pitch和yaw的差分对电机为A和C
*/
typedef struct
{
	YS_Actuator_Unit thumb_A; 
	YS_Actuator_Unit index_A;
	YS_Actuator_Unit middle_A;
	YS_Actuator_Unit ring_A;
	YS_Actuator_Unit little_A; 
	YS_Actuator_Unit thumb_B;
	YS_Actuator_Unit reserve;
}YS_Actuator;
//控制指令
typedef struct
{
	YS_Cmd thumb_A;
	YS_Cmd index_A;
	YS_Cmd middle_A;
	YS_Cmd ring_A;
	YS_Cmd little_A;
	YS_Cmd thumb_B;
	YS_Cmd reserve;
}YS_Cmd_All;
typedef struct
{
	YS_OC_Detection thumb_A;
	YS_OC_Detection index_A;
	YS_OC_Detection middle_A;
	YS_OC_Detection ring_A;
	YS_OC_Detection little_A;
	YS_OC_Detection thumb_B;
	YS_OC_Detection reserve;
}YS_Oc_All;
typedef struct
{
	YS_OC2_Detection thumb_A;
	YS_OC2_Detection index_A;
	YS_OC2_Detection middle_A;
	YS_OC2_Detection ring_A;
	YS_OC2_Detection little_A;
	YS_OC2_Detection thumb_B;
	YS_OC2_Detection reserve;
}YS_Oc2_All;
typedef struct
{
	YS_Sta thumb_A;
	YS_Sta index_A;
	YS_Sta middle_A;
	YS_Sta ring_A;
	YS_Sta little_A;
	YS_Sta thumb_B;
	YS_Sta reserve;
}YS_Sta_All;

typedef struct
{
	Time_Stamp thumb_A;
	Time_Stamp index_A;
	Time_Stamp middle_A;
	Time_Stamp ring_A;
	Time_Stamp little_A;
	Time_Stamp thumb_B;
	Time_Stamp reserve;
}YS_Motor_Control_Cycle_All;
//关节手指//指尖顺着航向角反方向，遵循右手系，航向角绕Z，俯仰角绕Y
typedef struct
{
	float angle;
	bool dir_flip;
	float angle_max;
	float angle_min;
	uint32_t pos_max;//对应最小角度的位置
	uint32_t pos_min;
}Angle_Pos;


//手指的角度状态
typedef struct
{
	int16_t mean_value;
	int16_t difference_value;
	float pitch;
	float roll;
	uint8_t fault;
	float current;
	float speed;
	uint8_t rotor_lock_count_th;
	uint8_t pitch_temperature;
	uint8_t roll_temperature;
}Finger_Angle_Sta;


/*手指ID*/
typedef enum
{
	INVALID_FINGER_ID = 0,
	FINGER_THUMB = 1,
	FINGER_INDEX = 2,
	FINGER_MIDDLE = 3,
	FINGER_RING = 4,
	FINGER_LITTLE = 5,
	FINGER_ID_MAXIMUM,
}FINGER_ID;


/*手指类型 ：根据每个手指有几个电机进行分类*/
typedef enum
{
	INVALID_FINFER_TYPE = 0,
	FINGER_1M = 1,
	FINGER_2M = 2,
}FINGER_TYPE;


/*手指*/
typedef struct 
{
	FINGER_ID finger_id;
	FINGER_TYPE finger_type;
	YS_Actuator_Unit *actuator_A;		//绑定关节 手指与执行器（电机）进行绑定
	YS_Actuator_Unit *actuator_B;		//绑定关节
	Finger_Angle_Sta sta;						//手指角度状态
	uint16_t mean_value;
	int16_t difference_value;
	
	Angle_Pos pitch;								//手掌中心建立3维正交坐标系，右手系，										
	Angle_Pos roll;
	
	float last_pitch;								//从can获取指令这一层面的上一个控制值，用于调速，计算指令速度										
	float last_roll;
	
	
	Time_Stamp pitch_cmd_cycle;			//指令接收时间戳	
	Time_Stamp roll_cmd_cycle;
	
	bool clear_fault;								//承接清除故障指令分配给电机
	uint8_t speed;									//承接速度设定指令分配给电机
	uint8_t current;								//承接电流设定指令分配给电机
	uint8_t rotor_lock_count_th;		//承接过流判定阈值指令分配给电机
	
	uint8_t pitch_temperature;
	uint8_t  roll_temperature;
	
	uint8_t pitch_speed;
	uint8_t  roll_speed;
	
	uint8_t pitch_current;
	uint8_t  roll_current;
	
	float pitch_duty;								//用于指尖在指根伸直时限制弯曲程度
	
	float pitch_dead_zone;					//用于指根在指尖弯曲时限制伸直程度
	
	bool at_fault_sta;							//用于控制弯曲和侧摆的差分对发生了故障
	
}Finger;


typedef struct
{
	Finger thumb;
	Finger index;
	Finger middle;
	Finger ring;
	Finger little;
}Hand;

extern Inspire_Data inspire_data[YS_ID_MAXIMUM];
extern YS_Actuator ys_actuator;
extern YS_Cmd_All ys_cmd_all;
extern YS_Sta_All ys_sta_all;
extern YS_Oc2_All ys_oc2_all;
extern YS_Motor_Control_Cycle_All ys_motor_control_cycle_all;
extern Hand hand;

bool inspire_motor_rx_decode(YS_Actuator *pActuator,Inspire_Comm *pComm,Inspire_Data *inspire_data);
uint32_t inspire_motor_init(Hand *pHand,YS_Actuator *pActuator,Inspire_Comm *pComm,Inspire_Data *inspire_data);
uint32_t inspire_motor_control(YS_Actuator *pActuator,Inspire_Comm *pComm,Inspire_Data *inspire_data);
uint32_t over_current_detection(YS_Actuator *pActuator);
uint32_t over_current_detection2(YS_Actuator *pActuator);
bool updata_motor_sta(YS_Actuator *pActuator,Inspire_Comm *pComm,Inspire_Data *inspire_data);
uint32_t clear_fault(YS_Actuator *pActuator,Inspire_Comm *pComm,Inspire_Data *inspire_data);
bool hand_planner(Hand *pHand,YS_Actuator *pActuator,YS_Cmd_All *pCmd);
void ys_get_cmd(Hand *pHand,YS_Actuator *pActuator,Upper_Request *pRequest,Protocol_Aux_Data *aux_data);
void ys_set_status(Hand *pHand,Inspire_Data *pInspre ,Lower_Response *lower_response);
bool updata_hand_sta(Hand *pHand,YS_Actuator *pActuator,YS_Sta_All *pSta);
void hand_control_timeout_detection(Hand *pHand,YS_Actuator *pActuator);
float speed_step_adjust(Finger *pFingle,YS_Actuator_Unit *actuator,Time_Stamp *cmd_cycle,Time_Stamp *motor_control_cycle,uint16_t angle_pos,uint16_t last_pos);
#endif


/******************************** END OF FILE *********************************/


 



/**
 ********************************** Copyright *********************************
 *
 ** (C) Copyright 2022-2024 YaoYandong,China.
 ** All Rights Reserved.
 *
 ******************************************************************************
 **--------------------------------------------------------------------------**
 ** @FileName      : upper_comm_protocol_fml.h
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

#ifndef __UPPER_COMM_PROTOCOL_FML_H_
#define __UPPER_COMM_PROTOCOL_FML_H_
#include "stdint.h"
#include "stdbool.h"
#include "tip_sensor_comm_fml.h"
#include "HWK_touch_sensor_fml.h"
#include "data_structure.h"
#include "upper_can_comm_drv.h"
#include "config.h"
#include "parameter_fml.h"
#define RETURN_POSITION (0x00000001 << 0) // 位置1
#define RETURN_PRESS1 (0x00000001 << 1)		// 压力1
// #define RETURN_PRESS2       (0x00000001<<2)			//压力2
// #define RETURN_POSITION2 	(0x00000001<<3)			//位置2
#define RETURN_SPEED (0x00000001 << 4) // 速度1
// #define RETURN_SPEED1		(0x00000001<<5)			//速度2
#define RETURN_ERROR (0x00000001 << 6)		// 帧错误
#define RETURN_ALL_DATA (0x00000001 << 7) // 长帧整包发送
#define RETURN_TEMP1 (0x00000001 << 8)		// 温度1
#define RETURN_TEMP2		(0x00000001<<9)			//电流
#define RETURN_ERROR_CODE1 (0x00000001 << 10) // 帧错误1
// #define RETURN_ERROR_CODE2	(0x00000001<<11)			//帧错误2
#define RETURN_ACCELERATION (0x00000001 << 12) // 电机加速度帧返回

#define RETURN_NONE (0x00000000)

#define FRAME_HEAD_485 0X55 // 485帧头

#define CRC8_POLYNOMIAL 0x31

#define RETURN_THUMB_TOUCH (0x00000001 << 0)	//
#define RETURN_INDEX_TOUCH (0x00000001 << 1)	//
#define RETURN_MIDDLE_TOUCH (0x00000001 << 2) //
#define RETURN_RING_TOUCH (0x00000001 << 3)		//
#define RETURN_LITTLE_TOUCH (0x00000001 << 4) //
#define RETURN_PALM_TOUCH (0x00000001 << 5)		//

#pragma pack(1)

typedef enum
{
	NONE					 = 0,
	HALF_GRAB		   = 1,
	FINGER_CIRCLE  = 2,
	HAND_CIRCLE	   = 3,
	ELLIPSE  		   = 4,
	TUTTING  		   = 5,
	SIX      		   = 6,
	SEVEN    		   = 7,
	EIGHT    		   = 8,
	NINE     		   = 9,
	TEN      		   = 10,
	ROCK					 = 11,
	SCISSORS			 = 12,
	PAPER    		   = 13,
	PACK					 = 14,
}HAND_ACTION_TYPE;
	
typedef struct
{
	HAND_ACTION_TYPE action_type;
	HAND_ACTION_TYPE last_action_type;
	uint8_t speed;						//轨迹规划速度
	uint8_t amplitude;				//动作幅值
	uint8_t number_of_turns;	//圈数
	
	uint8_t finger_num;				//单手指动作时指定的手指序号。部分指令用到
	
	bool action_end;					//动作执行完毕
	
}Hand_Action;

typedef enum
{
	INVALID_COMM_INTERFACE = 0, // 无效的接口选择						|
	COMM_485 = 1,								// 485接口									|
	COMM_CAN = 2,								// can接口									|
	COMM_NET = 3,								// 网口										|
} Comm_Interface;

// 用一个字节表示的矩阵大小
typedef struct
{
	uint8_t column : 4; // 低四位列
	uint8_t row : 4;		// 高四位行

} Matrix_Size_u8;

typedef struct
{
	uint8_t pitch_angle;						//手指根部弯曲
	uint8_t speed_ref;							//速度控制
	uint8_t over_current_th;				//过流设置
	uint8_t clear_fault;						//清除错误
	uint8_t rotor_lock_count_th;		//堵转事件检测次数判定阈值
	uint8_t pitch_temperature;			//温度
	
	uint8_t pitch_current;					//温度
			
	uint8_t pitch_speed;						//温度

}Finger_Upper_Cmd;
typedef struct
{
	uint8_t pitch_angle;

	uint8_t fault;
	uint8_t speed;
	uint8_t current;
	uint8_t pitch_temperature;			//温度
	
	uint8_t pitch_current;					//电流
	
	uint8_t pitch_speed;						//速度
}Finger_Lower_Sta;

// 为了照顾can传输的传输长度，此处数据类型都定义成uint8_t,在使用时进行单位转换
typedef struct
{
	Finger_Upper_Cmd thumb;
	Finger_Upper_Cmd index;
	Finger_Upper_Cmd middle;
	Finger_Upper_Cmd ring;
	Finger_Upper_Cmd little;	
	Finger_Upper_Cmd thumb_yaw;
	Finger_Tip_Data_All *pTip_Data_All;
	Hand_Action *pAction;
	uint8_t matrix_sensor_index;
	Matrix_Size_u8 matrix_size;
}Upper_Request;

// 为了照顾can传输的传输长度，此处数据类型都定义成uint8_t,提醒上位机在使用时进行单位转换
//为了照顾can传输的传输长度，此处数据类型都定义成uint8_t,提醒上位机在使用时进行单位转换
typedef struct
{
	Finger_Lower_Sta thumb;
	Finger_Lower_Sta index;
	Finger_Lower_Sta middle;
	Finger_Lower_Sta ring;
	Finger_Lower_Sta little;
	Finger_Lower_Sta thumb_yaw;
	Finger_Tip_Data_All *pTip_Data_All;
	Hand_Action *pAction;
	
	HWK_Hand_Sensor *hwk_hand_sensor;
	uint8_t matrix_sensor_index;
	Matrix_Size_u8 matrix_size;
}Lower_Response;
// 协议的帧属性
typedef enum
{
	INVALID_FRAME_PROPERTY = 0x00, // 无效的can帧属性
	JOINT_POSITION_RCO = 0x01,		 // 关节位置
	MAX_PRESS_RCO = 0x02,					 // 最大压力
	MAX_PRESS_RCO1 = 0x03,				 // 其它数据
	JOINT_POSITION2_RCO = 0X04,		 // 关节位置2
	SPEED_RCO = 0X05,							 // 五个手指的速度
	SPEED_RCO1 = 0X06,						 // 五个手指的速度
	ACCELERATION_RCO = 0X07,			 // 七个电机手指的加速度

	Hand_Normal_Force = 0x20,					// 五个手指的法向压力
	Hand_Tangential_Force = 0x21,			// 五个手指的切向压力
	Hand_Tangential_Force_Dir = 0x22, // 五个手指的切向方向
	Hand_Approach_Inc = 0x23,					// 五个手指指接近感应

	Thumb_All_Data = 0x28,	// 大拇指所有数据
	Index_All_Data = 0x29,	// 食指所有数据
	Middle_All_Data = 0x30, // 中指所有数据
	Ring_All_Data = 0x31,		// 无名指所有数据
	Little_All_Data = 0x32, // 小拇指所有数据

	Temp1 = 0x33,           //温度
	Temp2 = 0x34,   				//电流

	Error_Code1 = 0x35,  
	Error_Code2 = 0x36,

	Home_Rewrite = 0x38,
	Home_Rewrite2 = 0x39,

	WHOLE_FRAME = 0X40, // 整帧传输							|返回一字节帧属性+整个结构体

	Version = 0x64, // 版本号
	SYS_RESET = 0x65,
	ERROR_CODE = 0x66,

	// 华威科、福莱传感器接口指令
	TOUCH_SENSOR_TYPE = 0XB0,
	THUMB_TOUCH = 0XB1,	 // 大拇指
	INDEX_TOUCH = 0XB2,	 // 食指
	MIDDLE_TOUCH = 0XB3, // 中指
	RING_TOUCH = 0XB4,	 // 无名指
	LITTLE_TOUCH = 0XB5, // 小拇指
	PALM_TOUCH = 0XB6,	 // 手掌

	// 配置命令·CONFIG
	HAND_UID = 0XC0,							// 设备唯一标识码
	HAND_HARDWARE_VERSION = 0XC1, // 硬件版本
	HAND_SOFTWARE_VERSION = 0XC2, // 软件版本
	CAN_ID_SET = 0xC3,						// 修改id
	LOCK_ROTOR_THRESHOLD = 0xC4,	// 锁转阈值
	LOCK_ROTOR_TIME = 0xC5,				// 锁转时间
	LOCK_ROTOR_TORQUE = 0xC6,			// 锁转扭矩
	HAND_FACTORY_RESET = 0XCE,		// 恢复出厂设置
	SAVE_PARAMETER = 0xCF,				// 保存
} FRAME_PROPERTY;

typedef struct
{
	uint8_t comm_id;
	Comm_Interface comm_interface; // 标记数据接收时用到的通信接口，在发返回帧的时候用该接口
	uint32_t return_frame_makers;	 // 按位标记要返回的返回帧 |位置 bit0|压力 bit1|其它数据 bit2|暂空bit3|暂空bit4|暂空bit5|暂空bit6|暂空bit7|
	FRAME_PROPERTY frame_property;
	uint32_t return_touch_sensor_makers;	
} Protocol_Aux_Data;
// 上位机控制协议
typedef struct
{
	Upper_Request *upper_request;
	Lower_Response *lower_response;
	Protocol_Aux_Data *protocol_aux_data;
} Upper_Protocol;

/*can*/
/*帧结构
|0			|1		|3		|4			|5			|6			|7			|
|帧属性	|			|			|				|				|				|				|
|				|			|			|				|				|				|				|
*/
/*RCO RETURN CURRENT DATA ONCE;N NOT RETURN*/

#pragma pack()

#endif
extern Upper_Request upper_request;
extern Lower_Response lower_response;
extern Upper_Protocol upper_protocol;
extern Protocol_Aux_Data protocol_aux_data;
/*485*/
//void comm_485_parser(Upper_485_Transmit *rs485_trans, Upper_Request *upper_request, Protocol_Aux_Data *aux_data);
//void comm_485_send(Upper_485_Transmit *rs485_trans, Lower_Response *lower_response, FRAME_PROPERTY frame_property);
//void event_485_dispose(void);
/*can*/
//void comm_can_parser(Upper_Can_Transmit *can_trans, Upper_Request *upper_request, Protocol_Aux_Data *aux_data);
//void comm_can_send(Upper_Can_Transmit *can_trans, Lower_Response *lower_response, FRAME_PROPERTY frame_property);
void event_can_dispose(void);
void comm_can_touch_sensor_parser(Can_Rx_Queue *pRx_queue, Upper_Request *pRequest, Lower_Response *pResponse, Protocol_Aux_Data *aux_data);
void comm_can_touch_sensor_send(Upper_Can_Transmit *can_trans, Lower_Response *pResponse, FRAME_PROPERTY frame_property);
void event_can_tip_sensor_dispose(void);
void init_request_data(Upper_Request *request,Lower_Response *pResponse);
// void comm_modbus_touch_sensor_get(Modbus_485 *modbus, Lower_Response *pResponse, FRAME_PROPERTY frame_property);

/******************************** END OF FILE *********************************/

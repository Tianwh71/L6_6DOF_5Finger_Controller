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
#define RETURN_POSITION (0x00000001 << 0) // λ��1
#define RETURN_PRESS1 (0x00000001 << 1)		// ѹ��1
// #define RETURN_PRESS2       (0x00000001<<2)			//ѹ��2
// #define RETURN_POSITION2 	(0x00000001<<3)			//λ��2
#define RETURN_SPEED (0x00000001 << 4) // �ٶ�1
// #define RETURN_SPEED1		(0x00000001<<5)			//�ٶ�2
#define RETURN_ERROR (0x00000001 << 6)		// ֡����
#define RETURN_ALL_DATA (0x00000001 << 7) // ��֡��������
#define RETURN_TEMP1 (0x00000001 << 8)		// �¶�1
#define RETURN_TEMP2		(0x00000001<<9)			//����
#define RETURN_ERROR_CODE1 (0x00000001 << 10) // ֡����1
// #define RETURN_ERROR_CODE2	(0x00000001<<11)			//֡����2
#define RETURN_ACCELERATION (0x00000001 << 12) // ������ٶ�֡����

#define RETURN_NONE (0x00000000)

#define FRAME_HEAD_485 0X55 // 485֡ͷ

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
	uint8_t speed;						//�켣�滮�ٶ�
	uint8_t amplitude;				//������ֵ
	uint8_t number_of_turns;	//Ȧ��
	
	uint8_t finger_num;				//����ָ����ʱָ������ָ��š�����ָ���õ�
	
	bool action_end;					//����ִ�����
	
}Hand_Action;

typedef enum
{
	INVALID_COMM_INTERFACE = 0, // ��Ч�Ľӿ�ѡ��						|
	COMM_485 = 1,								// 485�ӿ�									|
	COMM_CAN = 2,								// can�ӿ�									|
	COMM_NET = 3,								// ����										|
} Comm_Interface;

// ��һ���ֽڱ�ʾ�ľ����С
typedef struct
{
	uint8_t column : 4; // ����λ��
	uint8_t row : 4;		// ����λ��

} Matrix_Size_u8;

typedef struct
{
	uint8_t pitch_angle;						//��ָ��������
	uint8_t speed_ref;							//�ٶȿ���
	uint8_t over_current_th;				//��������
	uint8_t clear_fault;						//�������
	uint8_t rotor_lock_count_th;		//��ת�¼��������ж���ֵ
	uint8_t pitch_temperature;			//�¶�
	
	uint8_t pitch_current;					//�¶�
			
	uint8_t pitch_speed;						//�¶�

}Finger_Upper_Cmd;
typedef struct
{
	uint8_t pitch_angle;

	uint8_t fault;
	uint8_t speed;
	uint8_t current;
	uint8_t pitch_temperature;			//�¶�
	
	uint8_t pitch_current;					//����
	
	uint8_t pitch_speed;						//�ٶ�
}Finger_Lower_Sta;

// Ϊ���չ�can����Ĵ��䳤�ȣ��˴��������Ͷ������uint8_t,��ʹ��ʱ���е�λת��
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

// Ϊ���չ�can����Ĵ��䳤�ȣ��˴��������Ͷ������uint8_t,������λ����ʹ��ʱ���е�λת��
//Ϊ���չ�can����Ĵ��䳤�ȣ��˴��������Ͷ������uint8_t,������λ����ʹ��ʱ���е�λת��
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
// Э���֡����
typedef enum
{
	INVALID_FRAME_PROPERTY = 0x00, // ��Ч��can֡����
	JOINT_POSITION_RCO = 0x01,		 // �ؽ�λ��
	MAX_PRESS_RCO = 0x02,					 // ���ѹ��
	MAX_PRESS_RCO1 = 0x03,				 // ��������
	JOINT_POSITION2_RCO = 0X04,		 // �ؽ�λ��2
	SPEED_RCO = 0X05,							 // �����ָ���ٶ�
	SPEED_RCO1 = 0X06,						 // �����ָ���ٶ�
	ACCELERATION_RCO = 0X07,			 // �߸������ָ�ļ��ٶ�

	Hand_Normal_Force = 0x20,					// �����ָ�ķ���ѹ��
	Hand_Tangential_Force = 0x21,			// �����ָ������ѹ��
	Hand_Tangential_Force_Dir = 0x22, // �����ָ��������
	Hand_Approach_Inc = 0x23,					// �����ָָ�ӽ���Ӧ

	Thumb_All_Data = 0x28,	// ��Ĵָ��������
	Index_All_Data = 0x29,	// ʳָ��������
	Middle_All_Data = 0x30, // ��ָ��������
	Ring_All_Data = 0x31,		// ����ָ��������
	Little_All_Data = 0x32, // СĴָ��������

	Temp1 = 0x33,           //�¶�
	Temp2 = 0x34,   				//����

	Error_Code1 = 0x35,  
	Error_Code2 = 0x36,

	Home_Rewrite = 0x38,
	Home_Rewrite2 = 0x39,

	WHOLE_FRAME = 0X40, // ��֡����							|����һ�ֽ�֡����+�����ṹ��

	Version = 0x64, // �汾��
	SYS_RESET = 0x65,
	ERROR_CODE = 0x66,

	// �����ơ������������ӿ�ָ��
	TOUCH_SENSOR_TYPE = 0XB0,
	THUMB_TOUCH = 0XB1,	 // ��Ĵָ
	INDEX_TOUCH = 0XB2,	 // ʳָ
	MIDDLE_TOUCH = 0XB3, // ��ָ
	RING_TOUCH = 0XB4,	 // ����ָ
	LITTLE_TOUCH = 0XB5, // СĴָ
	PALM_TOUCH = 0XB6,	 // ����

	// �������CONFIG
	HAND_UID = 0XC0,							// �豸Ψһ��ʶ��
	HAND_HARDWARE_VERSION = 0XC1, // Ӳ���汾
	HAND_SOFTWARE_VERSION = 0XC2, // ����汾
	CAN_ID_SET = 0xC3,						// �޸�id
	LOCK_ROTOR_THRESHOLD = 0xC4,	// ��ת��ֵ
	LOCK_ROTOR_TIME = 0xC5,				// ��תʱ��
	LOCK_ROTOR_TORQUE = 0xC6,			// ��תŤ��
	HAND_FACTORY_RESET = 0XCE,		// �ָ���������
	SAVE_PARAMETER = 0xCF,				// ����
} FRAME_PROPERTY;

typedef struct
{
	uint8_t comm_id;
	Comm_Interface comm_interface; // ������ݽ���ʱ�õ���ͨ�Žӿڣ��ڷ�����֡��ʱ���øýӿ�
	uint32_t return_frame_makers;	 // ��λ���Ҫ���صķ���֡ |λ�� bit0|ѹ�� bit1|�������� bit2|�ݿ�bit3|�ݿ�bit4|�ݿ�bit5|�ݿ�bit6|�ݿ�bit7|
	FRAME_PROPERTY frame_property;
	uint32_t return_touch_sensor_makers;	
} Protocol_Aux_Data;
// ��λ������Э��
typedef struct
{
	Upper_Request *upper_request;
	Lower_Response *lower_response;
	Protocol_Aux_Data *protocol_aux_data;
} Upper_Protocol;

/*can*/
/*֡�ṹ
|0			|1		|3		|4			|5			|6			|7			|
|֡����	|			|			|				|				|				|				|
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

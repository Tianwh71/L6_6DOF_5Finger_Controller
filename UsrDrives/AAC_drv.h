/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2024 YaoYandong,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : ACC_drv.h  
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

#ifndef __INSPIRE_DRV_H_
#define __INSPIRE_DRV_H_
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "usart.h"
#define YSBROADCAST_ID 0XFF				//广播id
#define YS_INS_FRAME_HEAD 0X55AA//指令帧帧头
#define YS_ANS_FRAME_HEAD 0XAA55//应答帧帧头
#define YS_DATA_LEN_MAXIMUM 60
#define	YS_TRANSMIT_LEN_MAXIMUM (YS_DATA_LEN_MAXIMUM+7)//传输层数据最大长度
#define INSTRUCTION_INTERVAL_TIME (45)
typedef struct
{
	__IO uint8_t send_buf[YS_TRANSMIT_LEN_MAXIMUM];
	__IO uint8_t send_len;
	__IO bool send_en_flag;
	__IO bool need_block;					//需要阻塞标志，如果需要等待返回帧，需要主动置起该标志位。阻塞操作完成后自动清除
	__IO int32_t block_count;			//阻塞状态计数，发送时阻塞+1，接收到数据后阻塞-1，如果该值在0~1之间变化，那么通信运行时序稳定。电机也未掉线。
	__IO uint32_t block_timeout_count;	//阻塞时间溢出
	uint32_t nop_count;	//指令之间需要有一定延迟，不然推杆不响应指令。
	uint32_t usart_rx_fault; //串口接收发生错误计数
	uint8_t receive_buf[YS_TRANSMIT_LEN_MAXIMUM]; 
	uint8_t receive_len;
	__IO bool receive_flag;
}YS_transmit;

/*
帧头			帧长度		ID号			指令号			控制表索引			数据段			校验和
2字节			一字节		一字节		一字节			一字节					n字节				一字节
*/
/*
帧长度 = n+2
*/
typedef enum
{
	CMD_RD = 0X01,										//查询控制表内的数据
	CMD_WR = 0X02,										//向控制表内写入数据
	CMD_POSITION_R = 0X21,						//定位模式（反馈状态信息） 
	CMD_POSITION_NR = 0X03,						//定位模式（无反馈）
	CMD_FOLLOW_R = 0X20,							//随动模式（反馈状态信息）
	CMD_FOLLOW_NR = 0X19,							//随动模式（无反馈）
	CMD_BROADCAST_POSITION_NR = 0XF2,	//广播定位模式（无反馈）内含N个电缸位置信息
	CMD_BROADCAST_FOLLOW_NR = 0XF3,		//广播随动模式（无反馈）内含N个电缸位置信息
	CMD_MC = 0X04,										//实现对电缸的功能控制		
}CMD_TYPE;
typedef enum
{/*															偏移地址			名称						注释																	权限				默认值		*/
	YS_TABLE_HEAD				= 0,			/*0~1					表头						保留																	--					0xAA 0X55	*/
	YS_ID								= 2,			/*2~3					电缸ID					1~254 255为广播地址										R/W					0X01			*/
	YS_SOFT_VERSION			=	4,			/*4~5				  软件版本号			    																	R					  --				*/
	YS_BAUDRATE					= 6,			/*6~7					串口波特率			1-19200 2-57600 3- 115200 4-921600		R/W					4					*/
	YS_OVER_CURRENT_TH 	= 8,			/*8~9				  过流保护设置		可设置范围300~1500mA									R/W					1500			*/
	YS_POSITION_REF			= 10,			/*10~11				目标位置设置		0~1600mm/1000mm												R/W					--				*/
	YS_CURRENT_POSITION = 12,			/*12~13				当前位置				-20~16200															R						--				*/
	YS_SPEED_LIMIT			= 14,			/*14~15			  最大速度	      5~25mm/s		                          R/W					--				*/
	YS_OFFLINE_FLAG			= 16,			/*16~17				下线检测标志	  0x001					                        R						--				*/
	YS_RESERVE1					=	18,			/*18~254		  保留						保留																	--					--				*/
}MEMORY_TABLE;
typedef enum
{
	RUNNING = 0X04,					//工作
	STOP = 0X23,						//急停
	PAUSE = 0X14,						//暂停
	SAVE = 0X20,						//参数装订
	QUERY_STATUS = 0X22,		//查询电缸状态信息（bit）包括目标位置、当前位置、温度、电机驱动电流以及异常信息
	CLEAR_FAULT = 0X1E,			//故障清除，当电缸发生过流、堵转、电机异常故障时可通过该指令清除故障码，恢复电缸的正常工作
}Single_Control_Instruction;
typedef struct
{
	uint16_t head;
	uint8_t len;
	uint8_t id;
	CMD_TYPE cmd_type;
	MEMORY_TABLE table;
	uint8_t data[YS_DATA_LEN_MAXIMUM];
	uint8_t sum_check;
}Instruction_Frame;

typedef struct
{
	uint16_t head;
	uint8_t len;
	uint8_t id;
	CMD_TYPE cmd_type;
	MEMORY_TABLE table;
	uint8_t data[YS_DATA_LEN_MAXIMUM];
	uint8_t sum_check;
}Answer_Frame;
typedef union
{
	struct
  {
		uint8_t locked_rotor:1;				//堵转
 	  uint8_t overcurrent:1;				//过流
	  uint8_t motor_abnormal:1;			//电机异常（失速）
	  uint8_t volatile_abnormal:1;  //电压异常
	  uint8_t CurAbnormal:1;        //电流自检异常
	  uint8_t PosCheckFail:1;       //位置自检异常	
		uint8_t reserved:2;           //占位
	}FaultBit;
	uint8_t u8FaultInfo;
}Fault;


typedef struct
{
	uint16_t id;                 //id
	uint16_t soft_version;
	uint16_t baudrate;						//波特率
	uint16_t over_current_th;		  //过流保护设置
	uint16_t target_position;		  //目标位置设置
	uint16_t current_position;		//当前位置
	uint16_t speed_limit;				  //速度最大值
	uint16_t offline_flag;				//下线标志
}Config_Data;
typedef struct
{
	uint16_t target_position;     //目标位置
	int16_t current_position;		  //当前位置
//	int8_t tempture;						//温度 ACC 没有温度反馈
	int16_t current;						  //电流
	uint8_t  u8VolaValue;         //电压值
	uint8_t u8MaxSpeedLimit;           //最大直线速度（0~25mm/s）
	uint16_t u16OC_Limit;         //过流保护阈值
	//int16_t force;							//力传感其数据  ACC 没有力反馈
	Fault   stFault;							//电机错误信息反馈
	uint16_t internal_data1;		  //内部数据1
	uint16_t internal_data2;		  //内部数据2
}Status_Data;
typedef struct
{
	Config_Data config_data;
	Status_Data status_data;
}Inspire_Data;
typedef struct
{
	uint8_t no_available_data:1;		//没有可用的数据
	uint8_t unreasonable_length:1;	//不合理的数据长度
	uint8_t no_match_head:1;				//帧头不符
	uint8_t no_match_data_len:1;		//不符合解析过程的帧长度
	uint8_t no_match_id:1;					//id不符
	uint8_t no_match_cmd_type:1;		//没有相符合的指令类型
	uint8_t no_match_table_index:1;	//没有匹配的控制表索引
	uint8_t no_match_sum_check:1;		//不正确的和校验
}Decode_Fault;
typedef struct
{
	uint32_t send_time;					//发送时刻
	uint32_t receive_time;			//接收时刻
	uint8_t get_sem_error;			//获取信号量超时错误
	Decode_Fault decode_fault;	//解码过程故障
}Comm_Detection;

typedef enum
{
	INVALID_MODE = 0,
	WAIT_FLAG = 1,
	WAIT_SEM_NOP = 2,
}TRANS_BLOCK_MODE;
typedef struct
{
	Instruction_Frame *instruction_frame;
	Answer_Frame *answer_frame;
	YS_transmit *ys_transmit;
	Comm_Detection comm_detection;
	TRANS_BLOCK_MODE trans_block_mode;	//传输时序阻塞模式，收发过程在374us左右，如果不想追求极致的高频控制，使用延时去控制时序间隔更适合.
}Inspire_Comm;



uint8_t sum_check(uint8_t *buf,uint8_t len);


extern Instruction_Frame instruction_frame;
extern Answer_Frame answer_frame;
extern YS_transmit ys_transmit_2;
extern YS_transmit ys_transmit_3;
extern YS_transmit ys_transmit_4;
extern Inspire_Comm inspire_comm[3];

void ys_usart_user_init(void);
bool HAL_UART2_Receive_IDLE(UART_HandleTypeDef *huart,YS_transmit *tran_handle, uint16_t Size);
void clear_usart2_fault(void);
bool build_ins_frame(Inspire_Comm *pComm	,	uint8_t id	,	CMD_TYPE cmd_type	,	MEMORY_TABLE memory_table	,	uint8_t *data	,	uint8_t data_len);
bool decode_rd_ans_frame(Inspire_Comm *pComm,Inspire_Data *inspire_data);
bool decode_mc_ans_frame(Inspire_Comm *pComm,Inspire_Data *inspire_data);
bool decode_ans_frame(Inspire_Comm *pComm);

//api
void read_table(Inspire_Comm *pComm,uint8_t id,MEMORY_TABLE table);
void write_table(Inspire_Comm *pComm,uint8_t id,MEMORY_TABLE table,uint8_t* data);
void position_mode_r(Inspire_Comm *pComm,uint8_t id,int16_t target_position);
void position_mode_nr(Inspire_Comm *pComm,uint8_t id,uint16_t target_position);
void follow_mode_r(Inspire_Comm *pComm,uint8_t id,int16_t target_position);
void follow_mode_nr(Inspire_Comm *pComm,uint8_t id,uint16_t target_position);


void ys_work(Inspire_Comm *pComm,uint8_t id);
void ys_stop(Inspire_Comm *pComm,uint8_t id);
void ys_pause(Inspire_Comm *pComm,uint8_t id);
void ys_save(Inspire_Comm *pComm,uint8_t id);
void ys_query_status(Inspire_Comm *pComm,uint8_t id);
void ys_clear_fault(Inspire_Comm *pComm,uint8_t id);

void Acc_cmd_test(void);

#endif


/******************************** END OF FILE *********************************/


 


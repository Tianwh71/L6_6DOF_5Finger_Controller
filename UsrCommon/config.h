/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2024 YaoYandong,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : config.h  
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

#ifndef __CONFIG_H_
#define __CONFIG_H_

#define __DEBUG__  
#ifdef __DEBUG__  
#define DEBUG(format,...) printf("File: "__FILE__", Line: %05d: "format"/n", __LINE__, ##__VA_ARGS__)  
#else  
#define DEBUG(format,...)  
#endif  


/*1、是否启用运行指示灯*/
#define INDICATOR_LIGHT_ENABLE 1		//只影响最底层控制灯亮不亮，而不影响led灯的功能逻辑，该宏的目的是为了减小光污染
/*2、向上位机压力返回数据源*/
#define	MOTOR_FEEDBACK_LOAD 1				//电机返回的负载
#define VIRTUAL_FORCE_SENSOR 2			//虚拟力传感数据来源
#define FORCE_DATA_SOURCE MOTOR_FEEDBACK_LOAD
/*3、力控模式*/
#define MOTOR_SELF_LIMIT 1						//电机自带最大转矩功能限制
#define FILM_PRESSURE_SOFT_LIMIT 2		//薄膜传感器一旦检测到超过阈值，电机停止运动
#define FILM_PRESSURE_SOFT_CONTROL 3	//电机在向目标位置运动过程中会保持一个设定的压力值，这种模式在负载是有阻尼特性时有用，但实现起来有诸多限制
#define FORCE_CONTROL_MODE MOTOR_SELF_LIMIT
/*4、手类型，左手还是右手*/
#define LEFT_HAND 1			//左手
#define RIGHT_HAND 2		//右手

//#if !defined(HAND_TYPE) 
#define HAND_TYPE RIGHT_HAND 
//#warning "config hand id"
//#endif


/*5、本节点id.弃用，默认id为0x01,广播id 0xff*/
#if (HAND_TYPE == LEFT_HAND)
	#define SELF_ID 0x28
#elif (HAND_TYPE == RIGHT_HAND)
	#define SELF_ID 0x27	
#endif

#define DEFAULT_HAND_ID 0X01					//默认id
#define DEVICE_BROADCAST_ID 0XFF			//广播id
/*6、485通信方式帧类型*/
#define SHORT_FRAME_485 1
#define LONG_FRAME_485 2

#define FRAME_TYPE_485 LONG_FRAME_485

/*7、飞特数据收发时序阻塞方式*/
#define FT_WAIT_FLAG 1
#define FT_WAIT_SEM 2
#define FT_TRANS_BLOCK_MODE FT_WAIT_FLAG

/*8、设备参数*/
//硬件版本
#define ALIENTEKF_F407 0X0001								//正点原子探索者开发板
#define END_CONTROL_BOARD_V1_0 0X0002				//末端控制板
#define ROBOT_CONTROL_BOARD_V1_0 0X0003 		//控制6路捷源推杆电机的控制板
#define ROBOT_CONTROL_BOARD_V2_0 0X0004			//留出两路485对外留出一路can,搭配多版本载板使用的控制板
#define ROBOT_CONTROL_BOARD_V3_0 0X0005			//留出指尖传感器串口，对外485，等五个串口的板子,搭配多版本载板使用的控制板
//软件版本
#define ROBOT_HAND_BASE_FREEMODBUS 				0X0001   	//依赖正点原子探索者开发板，移植freemodbus尝试控制捷源电机的版本，因读回的数据未按照预期存入二维数组，排查耗费时间，遂暂时放弃
#define ROBOT_HAND_BASE_COMMAND 					0X0002   	//六个捷源推杆的六自由度机械手，使用正点原子开发板，自己构造协议指令进行modbus通信
#define ROBOT_HAND_BASE_COMMAND_f103 			0X0003   	//六个捷源推杆的六自由度机械手，使用末端控制板
#define ROBOT_HAND_BASE_COMMAND_f103_new 	0X0004 		//六个捷源推杆的六自由度机械手，使用v1方形控制板
#define JY5_FT1_NEW_PCB										0X0005		//五个捷源推杆一个飞特舵机的六自由度机械手，使用v1方形控制板
#define JY5_FT1_NEW_PCB_V2 								0X0006		//五个捷源推杆一个飞特舵机的六自由度机械手，使用v2控制板及相关载板
#define FT9 															0x0007		//九个飞特舵机的九自由度机械手
#define FT2_YS14													0X0008		//两个飞特舵机14个因时推杆的十六自由度机械手
#define FT5_KM5														0X0009		//五个飞特舵机5个kingmax舵机的十自由度机械手
#define FT1_YS5														0x000A		//1个飞特舵机5个因时推杆的6自由度电机
#define YS16															0X000B		//16个因时电机的16自由度电机
#define FT10															0x000C		//10个飞特舵机的10自由度电机
#define YS17_REAL_16											0x000D		//17个因时推杆机的16自由度电机,但忽略了指尖
#define YS_REAL16_TIP_MIN_LIMIT						0x000E		//17个因时推杆机的16自由度电机，限制了指尖最小值
#define FT2_YS14_TIP_SENSOR								0x000F		//2个飞特舵机14个因时推杆机的16自由度电机，复用了串口一读取指尖传感器数据
#define FT2_YS14_MODBUS										0x0010		//2个飞特舵机14个因时推杆机的16自由度电机，增加moudbus协议接口，增加主动降速功能，使位置运动顺滑
#define FT2_YS14_MODBUS_THUMB_CHANGE			0x0011		//2个飞特舵机14个因时推杆机的16自由度电机，大拇指有所变化
				#define FT2_YS14_FIRST_VRSION 								0x0001					//初版
				#define FT2_YS14_CHANGE_CMD 									0x0002					//增加一个读取指令
				#define FT2_YS14_USE_HLS		 									0x0003					//使用hls3606舵机
#define FT2_YS14_QUICK										0x0012		//2个飞特舵机14个因时推杆机的16自由度电机，使用快速电机
				#define FT2_YS14_QUICK_FIRST_VRSION 					0x0001					//初版
				#define FT_HLS_2_YS14_QUICK									  0x0002					//使用两个hls系列的飞特舵机
#define FT18_4FINGER											0x0013		//18个飞特舵机构成的四指手
#define FT20_5FINGER											0x0014		//20个飞特舵机构成的五指手
				#define FT20_5FINGER_FIRST_VRSION 0x0001					//初版
#define FT21_5FINGER											0x0015		//21个飞特舵机构成的五指手
				#define FT21_5FINGER_FIRST_VRSION 						0x0001					//初版
				#define FT21_5FINGER_WITH_TOUCH_SENSOR				0x0002					//带有指尖传感器的21自由度手
#define FT2_YS15													0x0016		//2个飞特舵机15个因时推杆机的17自由度电机
				#define FT2_YS15_FIRST_VRSION 								0x0001					//初版
				#define FT2_YS15_WHTH_WRIST 									0x0002					//带手腕版本
#define FT2_YS14_IAP											0x0017		//2个飞特舵机1个因时推杆机的16自由度手带iap功能
				#define FT2_YS14_IAP_FIRST_VRSION 						0x0001					//初版
#define FT21_5FINGER_SERIES								0x0018		//21个飞特舵机5手指串联型机械手
				#define FT21_5FINGER_SERIES_FIRST_VRSION 			0x0001					//初版
				#define FT17_5FINGER_SERIES_USE_HLS 			    0x0002					//使用HLS系列舵机
				
				
#define UID 0X0001//设备唯一识别码
#define DEVICE_HARDWARE_VERSION ROBOT_CONTROL_BOARD_V3_0
#define DEVICE_SOFTWARE_VERSION ((FT2_YS14_MODBUS_THUMB_CHANGE<<16)+FT2_YS14_USE_HLS)

/*9、485接口所使用协议*/
#define SELF_PROTOCOL_485 1
#define MODBUS_485  2

#define PORT_485_PROTOCOL MODBUS_485

/*10、使用的指尖传感器*/
typedef enum
{
	TASHAN_GATHER = 1,
	HUAWEIKE = 2,
	FULAI = 3,
}Tip_Sensor_Select;

extern Tip_Sensor_Select tip_sensor_select;
#endif	


/******************************** END OF FILE *********************************/


 


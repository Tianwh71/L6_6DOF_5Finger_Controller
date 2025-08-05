/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2024 YaoYandong,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : parameter_fml.h  
  ** @Description   : None
  **--------------------------------------------------------------------------**
  ** @Author        : Depressed	  
  ** @Version       : v1.0				
  ** @Creat Date    : 2024-09-10  
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

#ifndef __PARAMETER_FML_H_
#define __PARAMETER_FML_H_
#include "stdint.h"
#include "stdbool.h"
#include "string.h"

#define VERIFICATION_CODE 0X1234
#define FLASH_SAVE_OFFSET 0x100			//参数保存到flash的相对于flash保存的偏移值。
#define CONFIG_SAVE_ADDR (FLASH_SAVE_ADDR+FLASH_SAVE_OFFSET)
#define CHANGE_PASSWORD	0X161616		//修改密码
#pragma pack(2)

//小端
typedef struct
{
	uint32_t valiaity_check;			//有效性判断，该变量需要与预设值相等
	uint32_t unique_identifier;		//设备独一无二id,二次修改需要密码 uid
	uint32_t hardware_version;		//硬件版本号，二次修改需要密码
	uint32_t software_version;  	//软件版本号。二次修改需要密码
	uint8_t local_can_id;					//设备ID，CAN接口id 支持修改
	uint8_t local_modbus_id;			//设备ID，modbus接口id支持修改
	uint16_t modbus_baudrate;			//modbus波特率,代号
}Parameter;
#pragma pack()
typedef struct
{
	Parameter *default_para;	//默认参数
	Parameter *current_para;	//当前生效的参数
	Parameter modify_para;		//修改参数
	Parameter temp_para;			//临时存储从flash中读出的参数
}Config;
#pragma pack()



extern Config config;
extern Parameter default_parameter;
extern Parameter current_parameter;
bool read_parameter(Config *pConfig);
bool write_parameter(Config *pConfig);
bool factory_reset_parameter(Config *pConfig);
#endif


/******************************** END OF FILE *********************************/


 


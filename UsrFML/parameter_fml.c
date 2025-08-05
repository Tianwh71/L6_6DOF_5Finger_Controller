/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2024 YaoYandong,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : parameter_fml.c  
  ** @Brief         : None
  **--------------------------------------------------------------------------**
  ** @Author Data   : Depressed 2024-09-10
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
#include "parameter_fml.h"
#include "config.h"
Parameter default_parameter = 
{
	.valiaity_check = VERIFICATION_CODE,			//有效性判断，该变量需要与预设值相等
	.unique_identifier = UID,									//设备独一无二id,二次修改需要密码
	.hardware_version = DEVICE_HARDWARE_VERSION,		//硬件版本号，二次修改需要密码
	.software_version = DEVICE_SOFTWARE_VERSION,  	//软件版本号。二次修改需要密码
	.local_can_id = DEFAULT_HAND_ID,						//设备ID，支持修改
	.local_modbus_id = 1,			//设备ID，modbus接口id支持修改
	.modbus_baudrate = RATE_256000,			//modbus波特率
};
Parameter current_parameter;
Config config =
{
	.default_para = &default_parameter,
	.current_para = &current_parameter,
};

//
bool read_parameter(Config *pConfig)
{
	bool ret = true;
//todo 读出参数，判断有效性，无效赋值默认参数，赋给current,保存参数
									//有效直接赋给current
	 stmflash_read(CONFIG_SAVE_ADDR,(uint16_t*)&pConfig->temp_para, sizeof(Parameter)/2);
	 if(pConfig->temp_para.valiaity_check == VERIFICATION_CODE)//flash中的是有效数据
	 {
		 *pConfig->current_para = pConfig->temp_para;
			pConfig->modify_para = *pConfig->current_para;
		 ret = true;
	 }
	 else
	 {
		  *pConfig->current_para = *pConfig->default_para;
		  stmflash_write(CONFIG_SAVE_ADDR, (uint16_t *)pConfig->default_para, sizeof(Parameter)/2);
			ret = false;
	 }	
	 return ret;
}
bool write_parameter(Config *pConfig)
{
	bool ret = true;
	//读出flash中的数据，如果当前参数与读出参数有差别，执行一次flash写入，避免flash无效擦除及写入操作。
	stmflash_read(CONFIG_SAVE_ADDR,(uint16_t*)&pConfig->temp_para, sizeof(Parameter)/2);
	if(pConfig->current_para->valiaity_check == VERIFICATION_CODE)
	{
		if (memcmp((void*)&pConfig->temp_para,(void*)&pConfig->modify_para,sizeof(Parameter)) == 0)
		{
			ret =  false;
		}else
		{
			 stmflash_write(CONFIG_SAVE_ADDR, (uint16_t*)&pConfig->modify_para, sizeof(Parameter)/2);//参数发生改变，写入到flash
			ret = true;
		}
	}
	return ret;
}
bool factory_reset_parameter(Config *pConfig)
{
	//将默认参数赋值给当前参数，将当前参数保存到flash.
	*pConfig->current_para = *pConfig->default_para;
	stmflash_write(CONFIG_SAVE_ADDR, (uint16_t*)pConfig->current_para, sizeof(Parameter)/2);//参数发生改变，写入到flash
	return true;
}


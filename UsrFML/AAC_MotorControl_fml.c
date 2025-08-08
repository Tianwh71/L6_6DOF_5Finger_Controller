/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2024 YaoYandong,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : Inspire_MotorControl_fml.c  
  ** @Brief         : None
  **--------------------------------------------------------------------------**
  ** @Author Data   : Depressed 2025-06-06
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
#include "AAC_MotorControl_fml.h"
#include "my_math.h"
#include "cmsis_os.h"
#include "task.h"
#include "upper_comm_protocol_fml.h"
Inspire_Data inspire_data[RS_ID_MAXIMUM] = {0};
RS_Cmd_All RS_Cmd_all = 
{
	.thumb_A	=
	{
		.speed_step = DEFAULT_SPEED,
		.control_mode = POS_SYNC_FOLLOW_MODE,
	},
	.thumb_B  =
	{
		.speed_step = DEFAULT_SPEED,
		.control_mode = POS_SYNC_FOLLOW_MODE,
	},
	.index_A  =
	{
		.speed_step = DEFAULT_SPEED,
		.control_mode = POS_SYNC_FOLLOW_MODE,
	},
	.middle_A =
	{
		.speed_step = DEFAULT_SPEED,
		.control_mode = POS_SYNC_FOLLOW_MODE,
	},
	.ring_A   =
	{
		.speed_step = DEFAULT_SPEED,
		.control_mode = POS_SYNC_FOLLOW_MODE,
	},
	.little_A =
	{
		.speed_step = DEFAULT_SPEED,
		.control_mode = POS_SYNC_FOLLOW_MODE,
	},
};
RS_Sta_All RS_Sta_all;
YS_Oc_All ys_oc_all = 
{
	.thumb_A = 
	{
	 .count_th = OVER_CURRENT_DETECTION_TH,
	 .en_forword = true,
	 .en_backword = true,
	 .block = false,
	},
	.thumb_B = 
	{
	 .count_th = OVER_CURRENT_DETECTION_TH,
	 .en_forword = true,
	 .en_backword = true,
	 .block = false,
	},
	.index_A = 
	{
	 .count_th = OVER_CURRENT_DETECTION_TH,
	 .en_forword = true,
	 .en_backword = true,
		.block = false,
	},
	.middle_A = 
	{
	 .count_th = OVER_CURRENT_DETECTION_TH,
	 .en_forword = true,
	 .en_backword = true,
	 .block = false,
	},
	.ring_A = 
	{
	 .count_th = OVER_CURRENT_DETECTION_TH,
	 .en_forword = true,
	 .en_backword = true,
	 .block = false,
	},
	.little_A = 
	{
	 .count_th = OVER_CURRENT_DETECTION_TH,
	 .en_forword = true,
	 .en_backword = true,
	 .block = false,
	},
};

YS_Motor_Control_Cycle_All ys_motor_control_cycle_all = 
{
		.thumb_A = 
	{
	 .timeout_interval = MOTOR_CONTROL_TIMEOUT_INTERVAL_YS,
	},
	.thumb_B = 
	{
	 .timeout_interval = MOTOR_CONTROL_TIMEOUT_INTERVAL_YS,
	},
	.index_A = 
	{
	 .timeout_interval = MOTOR_CONTROL_TIMEOUT_INTERVAL_YS,
	},
	.middle_A = 
	{
	 .timeout_interval = MOTOR_CONTROL_TIMEOUT_INTERVAL_YS,
	},
	.ring_A = 
	{
	 .timeout_interval = MOTOR_CONTROL_TIMEOUT_INTERVAL_YS,
	},
	.little_A = 
	{
	.timeout_interval = MOTOR_CONTROL_TIMEOUT_INTERVAL_YS,
	},
};
RS_Actuator rs_actuator = 
{
	/*大拇指弯曲*/
	.thumb_A = 
	{
		.is_actuator_valid = true,
		.id = RS_ID_1,
		.is_actuator_online = true,
		.en_control_freeze = false,
		.pos_mode = RS_FOLLOW_MODE,
		.init = 
		{
			.default_pos = DEFAULT_POSITION,
			.pos_limit_min = MIN_LIMIT_POSITION,
			.pos_limit_max = MAX_LIMIT_POSITION,
			.over_current_th = OVER_CURRENT_TH,
			.overtemp_th = OVER_TEMP_TH,
			.recover_temp_th = RECOVER_TEMP,
			.current_max = OVER_CURRENT_TH_MAX,
			.current_min = OVER_CURRENT_TH_MIN,
			.speed_max = SPEED_MIN*255,
			.speed_min = SPEED_MIN,
		},
		.cmd = &RS_Cmd_all.thumb_A,
		.sta = &RS_Sta_all.thumb_A,
		.oc = &ys_oc_all.thumb_A,
		.motor_control_cycle = &ys_motor_control_cycle_all.thumb_A,
		.speed_adj = 
		{
			.enable_speed_adj = true,
		},
	},
	//大拇指横摆
	.thumb_B = 
	{
		.is_actuator_valid = true,
		.id = RS_ID_6,
		.is_actuator_online = true,
		.en_control_freeze = false,
		.pos_mode = RS_FOLLOW_MODE,
		.init = 
		{
			.default_pos = DEFAULT_POSITION,
			.pos_limit_min = MIN_LIMIT_POSITION,
			.pos_limit_max = MAX_LIMIT_POSITION,
			.over_current_th = OVER_CURRENT_TH,
			.overtemp_th = OVER_TEMP_TH,
			.recover_temp_th = RECOVER_TEMP,
			.current_max = OVER_CURRENT_TH_MAX,
			.current_min = OVER_CURRENT_TH_MIN,
			.speed_max = SPEED_MIN*255,
			.speed_min = SPEED_MIN,
		},
		.cmd = &RS_Cmd_all.thumb_B,
		.sta = &RS_Sta_all.thumb_B,
		.oc = &ys_oc_all.thumb_B,
		.motor_control_cycle = &ys_motor_control_cycle_all.thumb_B,
		.speed_adj = 
		{
			.enable_speed_adj = true,
		},
	},
	//食指弯曲
	.index_A = 
	{
		.is_actuator_valid = true,
		.id = RS_ID_2,
		.is_actuator_online = true,
		.en_control_freeze = false,
		.pos_mode = RS_FOLLOW_MODE,
		.init = 
		{
			.default_pos = DEFAULT_POSITION,
			.pos_limit_min = MIN_LIMIT_POSITION,
			.pos_limit_max = MAX_LIMIT_POSITION,
			.over_current_th = OVER_CURRENT_TH,
			.overtemp_th = OVER_TEMP_TH,
			.recover_temp_th = RECOVER_TEMP,
			.current_max = OVER_CURRENT_TH_MAX,
			.current_min = OVER_CURRENT_TH_MIN,
			.speed_max = SPEED_MIN*255,
			.speed_min = SPEED_MIN,
		},
		.cmd = &RS_Cmd_all.index_A,
		.sta = &RS_Sta_all.index_A,
		.oc = &ys_oc_all.index_A,
		.motor_control_cycle = &ys_motor_control_cycle_all.index_A,
		.speed_adj = 
		{
			.enable_speed_adj = true,
		},
	},
	//中指弯曲
		.middle_A = 
	{
		.is_actuator_valid = true,
		.id = RS_ID_3,
		.is_actuator_online = true,
		.en_control_freeze = false,
		.pos_mode = RS_FOLLOW_MODE,
		.init = 
		{
			.default_pos = DEFAULT_POSITION,
			.pos_limit_min = MIN_LIMIT_POSITION,
			.pos_limit_max = MAX_LIMIT_POSITION,
			.over_current_th = OVER_CURRENT_TH,
			.overtemp_th = OVER_TEMP_TH,
			.recover_temp_th = RECOVER_TEMP,
			.current_max = OVER_CURRENT_TH_MAX,
			.current_min = OVER_CURRENT_TH_MIN,
			.speed_max = SPEED_MIN*255,
			.speed_min = SPEED_MIN,
		},
		.cmd = &RS_Cmd_all.middle_A,
		.sta = &RS_Sta_all.middle_A,
		.oc = &ys_oc_all.middle_A,
		.motor_control_cycle = &ys_motor_control_cycle_all.middle_A,
		.speed_adj = 
		{
			.enable_speed_adj = true,
		},
	},
	//无名指弯曲
		.ring_A = 
	{
		.is_actuator_valid = true,
		.id = RS_ID_4,
		.is_actuator_online = true,
		.en_control_freeze = false,
		.pos_mode = RS_FOLLOW_MODE,
		.init = 
		{
			.default_pos = DEFAULT_POSITION,
			.pos_limit_min = MIN_LIMIT_POSITION,
			.pos_limit_max = MAX_LIMIT_POSITION,
			.over_current_th = OVER_CURRENT_TH,
			.overtemp_th = OVER_TEMP_TH,
			.recover_temp_th = RECOVER_TEMP,
			.current_max = OVER_CURRENT_TH_MAX,
			.current_min = OVER_CURRENT_TH_MIN,
			.speed_max = SPEED_MIN*255,
			.speed_min = SPEED_MIN,
		},
		.cmd = &RS_Cmd_all.ring_A,
		.sta = &RS_Sta_all.ring_A,
		.oc = &ys_oc_all.ring_A,
		.motor_control_cycle = &ys_motor_control_cycle_all.ring_A,
		.speed_adj = 
		{
			.enable_speed_adj = true,
		},
	},
	//小拇指弯曲
		.little_A = 
	{
		.is_actuator_valid = true,
		.id = RS_ID_5,
		.is_actuator_online = true,
		.en_control_freeze = false,
		.pos_mode = RS_FOLLOW_MODE,
		.init = 
		{
			.default_pos = DEFAULT_POSITION,
			.pos_limit_min = MIN_LIMIT_POSITION,
			.pos_limit_max = MAX_LIMIT_POSITION,
			.over_current_th = OVER_CURRENT_TH,
			.overtemp_th = OVER_TEMP_TH,
			.recover_temp_th = RECOVER_TEMP,
			.current_max = OVER_CURRENT_TH_MAX,
			.current_min = OVER_CURRENT_TH_MIN,
			.speed_max = SPEED_MIN*255,
			.speed_min = SPEED_MIN*5,  //电机最小速度是5
		},
		.cmd = &RS_Cmd_all.little_A,
		.sta = &RS_Sta_all.little_A,
		.oc = &ys_oc_all.little_A,
		.motor_control_cycle = &ys_motor_control_cycle_all.little_A,
		.speed_adj = 
		{
			.enable_speed_adj = true,
		},
	},
		.reserve = 
	{
		.is_actuator_valid = true,
		.id = RS_ID_MAXIMUM,
		.is_actuator_online = true,
		.en_control_freeze = false,
		.pos_mode = RS_FOLLOW_MODE,
		.init = 
		{
			.default_pos = DEFAULT_POSITION,
			.pos_limit_min = MIN_LIMIT_POSITION,
			.pos_limit_max = MAX_LIMIT_POSITION,
			.over_current_th = OVER_CURRENT_TH,
			.overtemp_th = OVER_TEMP_TH,
			.recover_temp_th = RECOVER_TEMP,
			.current_max = OVER_CURRENT_TH_MAX,
			.current_min = OVER_CURRENT_TH_MIN,
			.speed_max = SPEED_MIN*255,
			.speed_min = SPEED_MIN,
		},
		.cmd = &RS_Cmd_all.reserve,
		.sta = &RS_Sta_all.reserve,
		.oc = &ys_oc_all.reserve,
		.motor_control_cycle = &ys_motor_control_cycle_all.reserve,
		.speed_adj = 
			{
				.enable_speed_adj = true,
			},
	},
};



Hand hand = 
{
	.thumb = 
	{
		.actuator = &rs_actuator.thumb_A,	//绑定关节
		.at_fault_sta = false,
		.pitch = 
		{
			.angle = (PITCH_ANGLE_MAX+PITCH_ANGLE_MIN)/2,
			.dir_flip = false,
			.angle_max= PITCH_ANGLE_MAX,
			.angle_min= PITCH_ANGLE_MIN,
		  .pos_min = PITCH_POS_MIN,
		},
		.pitch_cmd_cycle = 
		{
			.timeout_interval = HAND_CONTROL_TIMEOUT_INTERVAL_YS,
		},
		.speed =DEFAULT_SPEED,	//速度控制预设值单位转换
		.pitch_speed =DEFAULT_SPEED,
		
	  .current = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.pitch_current = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.rotor_lock_count_th = OVER_CURRENT_DETECTION_TH,
	},
	.index =
	{	
		.actuator = &rs_actuator.index_A,		//绑定关节
		.at_fault_sta = false,
		.pitch = 
				{
					.angle = (PITCH_ANGLE_MAX + PITCH_ANGLE_MIN)/2,
					.dir_flip = false,
					.angle_max =PITCH_ANGLE_MAX,			//
					.angle_min =PITCH_ANGLE_MIN,			//
					.pos_min = PITCH_POS_MIN,
				},
		.pitch_cmd_cycle = 
		{
			.timeout_interval = HAND_CONTROL_TIMEOUT_INTERVAL_YS,
		},
		.speed =DEFAULT_SPEED,	//速度控制预设值单位转换
		.pitch_speed =DEFAULT_SPEED,
		.current = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.pitch_current = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.rotor_lock_count_th = OVER_CURRENT_DETECTION_TH,
	},
		.middle =
	{	
		.actuator = &rs_actuator.middle_A,		//绑定关节
		.at_fault_sta = false,
		.pitch = 
				{
					.angle = (PITCH_ANGLE_MAX + PITCH_ANGLE_MIN)/2,
					.dir_flip = false,
					.angle_max =PITCH_ANGLE_MAX,			//
					.angle_min =PITCH_ANGLE_MIN,			//
					.pos_min = PITCH_POS_MIN,
				},
		.pitch_cmd_cycle = 
		{
			.timeout_interval = HAND_CONTROL_TIMEOUT_INTERVAL_YS,
		},
		.speed =DEFAULT_SPEED,	//速度控制预设值单位转换
		.pitch_speed =DEFAULT_SPEED,
		.current = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.pitch_current = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.rotor_lock_count_th = OVER_CURRENT_DETECTION_TH,
	},
		.ring =
	{	
		.actuator = &rs_actuator.ring_A,		//绑定关节指根
		.at_fault_sta = false,
		.pitch = 
				{
					.angle = (PITCH_ANGLE_MAX + PITCH_ANGLE_MIN)/2,
					.dir_flip = false,
					.angle_max =PITCH_ANGLE_MAX,			//
					.angle_min =PITCH_ANGLE_MIN,			//
					.pos_min = PITCH_POS_MIN,
				},
		.pitch_cmd_cycle = 
		{
			.timeout_interval = HAND_CONTROL_TIMEOUT_INTERVAL_YS,
		},
		.speed =DEFAULT_SPEED,	//速度控制预设值单位转换
		.pitch_speed =DEFAULT_SPEED,
		.current = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.pitch_current = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.rotor_lock_count_th = OVER_CURRENT_DETECTION_TH,
	},
		.little =
	{	
		.actuator = &rs_actuator.little_A,		//绑定关节指根
		.at_fault_sta = false,
		.pitch = 
		{
			.angle = (PITCH_ANGLE_MAX + PITCH_ANGLE_MIN)/2,
			.dir_flip = false,
			.angle_max =PITCH_ANGLE_MAX,			//
			.angle_min =PITCH_ANGLE_MIN,			//
			.pos_min = PITCH_POS_MIN,
		},
		.pitch_cmd_cycle = 
		{
			.timeout_interval = HAND_CONTROL_TIMEOUT_INTERVAL_YS,
		},
	  .speed =DEFAULT_SPEED,	//速度控制预设值单位转换
		.pitch_speed =DEFAULT_SPEED,
		.current = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.pitch_current = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.rotor_lock_count_th = OVER_CURRENT_DETECTION_TH,
	},
	.thumb_yaw = 
	{
		.actuator = &rs_actuator.thumb_B,	//绑定关节
		.at_fault_sta = false,
		.pitch = 
		{
			.angle = (PITCH_ANGLE_MAX+PITCH_ANGLE_MIN)/2,
			.dir_flip = false,
			.angle_max= PITCH_ANGLE_MAX,
			.angle_min= PITCH_ANGLE_MIN,
		  .pos_min = PITCH_POS_MIN,
		},
		.pitch_cmd_cycle = 
		{
			.timeout_interval = HAND_CONTROL_TIMEOUT_INTERVAL_YS,
		},
		.speed =DEFAULT_SPEED,	//速度控制预设值单位转换
		.pitch_speed =DEFAULT_SPEED,
		
	  .current = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.pitch_current = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.rotor_lock_count_th = OVER_CURRENT_DETECTION_TH,
	},

};

uint8_t *temperature_cmd_map[RS_ID_MAXIMUM] = {
	&hand.thumb. pitch_temperature,  
	&hand.index. pitch_temperature, 
	&hand.middle.pitch_temperature, 
	&hand.ring.  pitch_temperature, 
	&hand.little.pitch_temperature,
	&hand.thumb_yaw.pitch_temperature,
};
uint8_t *current_cmd_map[RS_ID_MAXIMUM] = {
	&hand.thumb.pitch_current, 	  
	&hand.index.pitch_current,
	&hand.middle.pitch_current,
	&hand.ring.pitch_current,
	&hand.little.pitch_current,
	&hand.thumb_yaw.pitch_current,
};
uint8_t *speed_cmd_map[RS_ID_MAXIMUM] = {
	&hand.thumb.pitch_speed,    
	&hand.index.pitch_speed,
	&hand.middle.pitch_speed,
	&hand.ring. pitch_speed,
	&hand.little.pitch_speed,
	&hand.thumb_yaw.pitch_speed,
};

//AAC电机协议解析
bool inspire_motor_rx_decode(RS_Actuator *pActuator,Inspire_Comm *pComm,Inspire_Data *pInspire_data)
{
	bool ret;
		switch(pComm->answer_frame->cmd_type)
	{
			case CMD_RD:
			{
				decode_rd_ans_frame(pComm,pInspire_data);
				ret =  true;
			}break;
			case CMD_MC:
			case CMD_POSITION_R:
			case CMD_FOLLOW_R:
			{
				decode_mc_ans_frame(pComm,pInspire_data);
				updata_motor_sta(pActuator,pComm,pInspire_data);
				ret =  true;
			}break;
			default:
			{
			ret =  false;
			}break;
	}
	return ret;
}
uint32_t inspire_motor_init(Hand *pHand,RS_Actuator *pActuator,Inspire_Comm *pComm,Inspire_Data *pInspire_data)
{
	uint32_t ret;//按位返回每个电机是否初始化过程正常，在线
	static uint16_t temp_pos_ref;
	uint8_t write_table_data[2];
	RS_Actuator_Unit *pActuator_unit = (RS_Actuator_Unit*) pActuator;
	Finger *pFingle = (Finger *)&pHand->thumb;
	Inspire_Comm *current_comm = NULL;
	for(int i = RS_ID_1;i<RS_ID_MAXIMUM;i++)
	{
		if(pActuator_unit[i-1].is_actuator_valid == false)
		{
			ret |= (0x00000001<<(i-1));
			continue;
		}
		// 选择通道：0-串口2，1-串口3，2-串口4
		if (i == RS_ID_1 || i == RS_ID_2)
			current_comm = &pComm[0];  // 串口2
		else if (i == RS_ID_3 || i == RS_ID_4)
			current_comm = &pComm[1];  // 串口3
		else if (i == RS_ID_5 || i == RS_ID_6)
			current_comm = &pComm[2];  // 串口4
		initLowPassFilter(&pActuator_unit[i-1].speed_step_real_filter,0.1);
		init_Mean_Filter(&pActuator_unit[i-1].MCC_mean_filter,true,YS_MCC_FILTER_DEPTH);
		init_Mean_Filter(&pActuator_unit[i-1].speed_mean_filter,true,YS_MCC_FILTER_DEPTH);
		//1、获取数据,判断在线与否
		for(int j = 0;j<15;j++)
		{
			ys_query_status(current_comm,i); //有概率没返回，多读几次
			osDelay(10);
			read_table(current_comm,i,YS_ID);//该数据对判定执行器在线比较关键，多读几次
			osDelay(10);
//			read_table(pComm,i,YS_TABLE_HEAD);
//			osDelay(100);
		}
		if(inspire_data[i].config_data.id == i)
		{
				pActuator_unit[i-1].is_actuator_online = true;
		}
		else
		{
				pActuator_unit[i-1].is_actuator_online = false;
				ret |= (0x00000001<<(i-1));
		}
		//2、读取配置数据
		read_table(current_comm,i,YS_BAUDRATE);
		osDelay(2);
		read_table(current_comm,i,YS_CURRENT_POSITION);
		osDelay(2);
		read_table(current_comm,i,YS_OVER_CURRENT_TH);
		osDelay(2);
		read_table(current_comm,i,YS_POSITION_REF);
		osDelay(2);

		
		//3、保存配置

		//4、写入配置（过流保护设置）
		write_table_data[0] = pActuator_unit[i-1].init.over_current_th&0x00ff;//过流
		write_table_data[1] = (pActuator_unit[i-1].init.over_current_th&0xff00)>>8;
		write_table(current_comm,i,YS_OVER_CURRENT_TH,write_table_data);
		

//			ys_save(pComm,i);
		osDelay(30);//需要留出时间保存数据到flash,如果不留出时间电机可能不响应后续指令
		
		//5、位置回写
  	//	if((pActuator_unit[i-1].is_actuator_online) == true&& (inspire_data[i].status_data.tempture!=0)) //ACC没有温度反馈
		
		//将当前手指（电机）的位置更新到上位机控制数据中作为初始数据
		if((pActuator_unit[i-1].is_actuator_online) == true)
		{
			temp_pos_ref = CONSTRAIN(inspire_data[i].status_data.current_position,pActuator_unit[i-1].init.pos_limit_max,pActuator_unit[i-1].init.pos_limit_min);
			pActuator_unit[i-1].cmd->position_ref = temp_pos_ref;//回写目标位置
			pActuator_unit[i-1].cmd->last_position_ref = temp_pos_ref;//回写目标位置
			pActuator_unit[i-1].cmd->slope_position_f = temp_pos_ref;//回写目标位置
		}
		else//未读到数据,取电机行程中点作为上电回写目标设定值
		{
			temp_pos_ref = CONSTRAIN(pActuator_unit[i-1].init.default_pos,pActuator_unit[i-1].init.pos_limit_max,pActuator_unit[i-1].init.pos_limit_min);
			pActuator_unit[i-1].cmd->position_ref = temp_pos_ref;
			pActuator_unit[i-1].cmd->last_position_ref = temp_pos_ref;//回写目标位置
			pActuator_unit[i-1].cmd->slope_position_f = temp_pos_ref;//回写目标位置
			ret |= (0x00000001<<(i-1));
		}
		//6、让电机处于工作状态（使能电机）
		ys_work(current_comm,i);
		osDelay(1);
	}
	//反算关节位置-根据当前电机的位置计算出当前关节的位置角度
	updata_hand_sta(&hand,pActuator,&RS_Sta_all);
	for(int i = RS_ID_1;i<RS_ID_MAXIMUM;i++ )
	{
		pFingle[i-1].pitch.angle =  CONSTRAIN(pFingle[i-1].sta.pitch,pFingle[i-1].pitch.angle_max,pFingle[i-1].pitch.angle_min);
		pFingle[i-1].last_pitch = pFingle[i-1].pitch.angle;
	}
	return ret;
}
uint32_t over_current_detection(RS_Actuator *pActuator)
{
	uint32_t ret;//按位返回每个电机是否初始化过程正常，在线
	RS_Actuator_Unit *pActuator_unit = (RS_Actuator_Unit*) pActuator;
	for(int i = RS_ID_1;i<RS_ID_MAXIMUM;i++)
	{
		if(pActuator_unit[i-1].is_actuator_valid == false)
		{
			ret |= (0x00000001<<(i-1));
		}
		if((pActuator_unit[i-1].oc->en_forword == true)&&(pActuator_unit[i-1].oc->en_backword == true))
		{
			pActuator_unit[i-1].oc->real_over_current_th = pActuator_unit[i-1].cmd->over_current_th_ref + ABS(pActuator_unit[i-1].sta->speed*8);
			if(abs(pActuator_unit[i-1].sta->current) > pActuator_unit[i-1].oc->real_over_current_th)
			{
				pActuator_unit[i-1].oc->count++;
				/*得到导致堵转的运动指令方向*/
					if((pActuator_unit[i-1].cmd->position_ref - pActuator_unit[i-1].sta->current_position)>5){
						pActuator_unit[i-1].oc->oc_dir++;	
					}else if((pActuator_unit[i-1].cmd->position_ref - pActuator_unit[i-1].sta->current_position)<-5){
						pActuator_unit[i-1].oc->oc_dir--;
					}
				if(pActuator_unit[i-1].oc->count > pActuator_unit[i-1].oc->count_th)
				{
					pActuator_unit[i-1].oc->ov_position = pActuator_unit[i-1].sta->current_position;
					pActuator_unit[i-1].sta->fault_code |= (1<<SOFT_OVER_CURRENT_BIT);//置起软件过流标志位
					if((pActuator_unit[i-1].oc->oc_dir)>0)
					{
						pActuator_unit[i-1].oc->en_forword = false;
					}else if((pActuator_unit[i-1].oc->oc_dir)<0)
					{
						pActuator_unit[i-1].oc->en_backword = false;
					}else
					{
						//死区
					}
				pActuator_unit[i-1].oc->count = 0;
				pActuator_unit[i-1].oc->last_oc_dir = pActuator_unit[i-1].oc->oc_dir;
				pActuator_unit[i-1].oc->oc_dir = 0;
				}
			}else
			{
				if(pActuator_unit[i-1].oc->count>0)
				{
					pActuator_unit[i-1].oc->count--;
				}
			}
		}
		else if((pActuator_unit[i-1].oc->en_backword == true)||(pActuator_unit[i-1].oc->en_forword == true))
		{
			if(((pActuator_unit[i-1].cmd->position_ref - pActuator_unit[i-1].sta->current_position)>5)&&(pActuator_unit[i-1].oc->en_backword == false))
			{
				pActuator_unit[i-1].oc->en_backword = true; 
				pActuator_unit[i-1].sta->fault_code &= 0<<SOFT_OVER_CURRENT_BIT;//清除软件过流标志位				
			}
			else if(((pActuator_unit[i-1].cmd->position_ref - pActuator_unit[i-1].sta->current_position)<-5)&&(pActuator_unit[i-1].oc->en_forword == false))
			{
				pActuator_unit[i-1].oc->en_forword = true;
				pActuator_unit[i-1].sta->fault_code &= 0<<SOFT_OVER_CURRENT_BIT;//清除软件过流标志位
			}
			else if(ABS(pActuator_unit[i-1].cmd->position_ref - pActuator_unit[i-1].sta->current_position)<5)
			{
				pActuator_unit[i-1].oc->en_backword = true; 
				pActuator_unit[i-1].oc->en_forword = true;
				pActuator_unit[i-1].sta->fault_code &= 0<<SOFT_OVER_CURRENT_BIT;//清除软件过流标志位
					
			}
		}
		else
		{
		
		}
	}
	return ret;
}

float temp_speed_step_max;
uint16_t dec_range;
uint32_t inspire_motor_control(RS_Actuator *pActuator,Inspire_Comm *pComm,Inspire_Data *pInspire_data)
{
	uint32_t ret;//按位返回每个电机是否初始化过程正常，在线
	float slope_position_f,speed_step;
	int16_t position_ref;
	uint8_t write_table_data[2];
	RS_Actuator_Unit *pActuator_unit = (RS_Actuator_Unit*) pActuator;
	Inspire_Comm * current_comm = NULL;
	

	for(int i = RS_ID_1;i<RS_ID_MAXIMUM;i++)
	{
		if(pActuator_unit[i-1].is_actuator_valid == false)
		{
			ret |= (0x00000001<<(i-1));
			continue;
		}
		// 选择通道：0-串口2，1-串口3，2-串口4
		if (i == RS_ID_1 || i == RS_ID_2)
			current_comm = &pComm[0];  // 串口2
		else if (i == RS_ID_3 || i == RS_ID_4)
			current_comm = &pComm[1];  // 串口3
		else if (i == RS_ID_5 || i == RS_ID_6)
			current_comm = &pComm[2];  // 串口4
		/*过流设置*/
		if(pActuator_unit[i-1].cmd->over_current_th_ref != pActuator_unit[i-1].cmd->last_over_current_th_ref)
		{
			  write_table_data[0] = pActuator_unit[i-1].cmd->over_current_th_ref&0x00ff;//过流
			  write_table_data[1] = (pActuator_unit[i-1].cmd->over_current_th_ref&0xff00)>>8;
			  write_table(current_comm,i,YS_OVER_CURRENT_TH,write_table_data);
			  pActuator_unit[i-1].cmd->last_over_current_th_ref = pActuator_unit[i-1].cmd->over_current_th_ref;
		}
		/*速度设置*/	
		if(pActuator_unit[i-1].cmd->speed_step != pActuator_unit[i-1].cmd->last_speed_step)
		{
			write_table_data[0] = (uint16_t)pActuator_unit[i-1].cmd->speed_step&0x00ff;//过流
			write_table_data[1] = ((uint16_t)pActuator_unit[i-1].cmd->speed_step&0xff00)>>8;
			write_table(current_comm,i,YS_SPEED_LIMIT,write_table_data);
			pActuator_unit[i-1].cmd->last_speed_step = pActuator_unit[i-1].cmd->speed_step;
		}
		/*位置设置*/	
		if((pActuator_unit[i-1].oc->en_forword == true)&&(pActuator_unit[i-1].oc->en_backword == true))//没有过流
		{
				pActuator_unit[i-1].cmd->position_ref_real = CONSTRAIN(pActuator_unit[i-1].cmd->position_ref,pActuator_unit[i-1].init.pos_limit_max,pActuator_unit[i-1].init.pos_limit_min);
				pActuator_unit[i-1].oc->block = false;
		}
		else if(((pActuator_unit[i-1].oc->en_backword == true)||(pActuator_unit[i-1].oc->en_forword == true))&&(pActuator_unit[i-1].oc->block == false))//过流
		{
				if(pActuator_unit[i-1].oc->last_oc_dir<0)
				{
				    pActuator_unit[i-1].cmd->position_ref_real = pActuator_unit[i-1].sta->current_position+20;
					  pActuator_unit[i-1].cmd->slope_position_f = pActuator_unit[i-1].cmd->position_ref_real;
				}
				else if(pActuator_unit[i-1].oc->last_oc_dir>0){
				    pActuator_unit[i-1].cmd->position_ref_real = pActuator_unit[i-1].sta->current_position-20;
				    pActuator_unit[i-1].cmd->slope_position_f = pActuator_unit[i-1].cmd->position_ref_real;
				}
				pActuator_unit[i-1].oc->block = true;//该标志位置起不再判断过流
		}
		else
		{
				if(pActuator_unit[i-1].sta->current > 0.3 * pActuator_unit[i-1].cmd->over_current_th_ref)
				{
				    pActuator_unit[i-1].cmd->position_ref_real = pActuator_unit[i-1].sta->current_position;
//				pActuator_unit[i-1].cmd->slope_position_f = pActuator_unit[i-1].cmd->position_ref_real;
				}
		}
		position_ref = pActuator_unit[i-1].cmd->position_ref_real;
		if(pActuator_unit[i-1].pos_mode == RS_FOLLOW_MODE)
		{
					slope_position_f = pActuator_unit[i-1].cmd->slope_position_f; //提取上次位置计算输出值
//					speed_step = pActuator_unit[i-1].cmd->speed_step;							//提取速度控制位置增量
					speed_step = pActuator_unit[i-1].cmd->speed_step_real;							//提取速度控制位置增量
					if(pActuator_unit[i-1].cmd->control_mode == POS_SYNC_POSITION_MODE)
					{
//						dec_range = float_2_u16((pActuator_unit[i-1].cmd->speed_step_real/pActuator_unit[i-1].init.speed_max),
//																		1.0,0.0,115,40,false);
//						if(ABS(pActuator_unit[i-1].cmd->position_ref_real - pActuator_unit[i-1].sta->current_position)<dec_range){
//							temp_speed_step_max = GET_MINIMUM(pActuator_unit[i-1].cmd->speed_step_real,pActuator_unit[i-1].init.speed_max/3);
//							speed_step = float_2_float((float)(ABS(pActuator_unit[i-1].cmd->position_ref_real - pActuator_unit[i-1].sta->current_position))/(float)dec_range
//									,1.0,0,temp_speed_step_max,9*pActuator_unit[i-1].init.speed_min,false);
//////							speed_step = (float)(ABS(pActuator_unit[i-1].cmd->position_ref_real - pActuator_unit[i-1].sta->current_position))/(float)dec_range*temp_speed_step_max;
//							pActuator_unit[i-1].cmd->slope_position_f = CONSTRAIN(position_ref,slope_position_f+speed_step,slope_position_f-speed_step);
//						}else{
//							pActuator_unit[i-1].cmd->slope_position_f = CONSTRAIN(position_ref,slope_position_f+speed_step,slope_position_f-speed_step);
//						}

						pActuator_unit[i-1].cmd->slope_position_f = CONSTRAIN(position_ref,slope_position_f+speed_step,slope_position_f-speed_step);
						//if(pActuator_unit[i-1].cmd->last_position_ref != pActuator_unit[i-1].cmd->slope_position_f)
						{
							pActuator_unit[i-1].cmd->last_position_ref = (uint16_t)pActuator_unit[i-1].cmd->slope_position_f;
						  follow_mode_r(current_comm,i,pActuator_unit[i-1].cmd->last_position_ref);
						}
						get_cycle(pActuator_unit[i-1].motor_control_cycle);
					}
					else if(pActuator_unit[i-1].cmd->control_mode == POS_SYNC_FOLLOW_MODE)//通过控制指令的周期判应该使用跟随模式
					{
						if(pActuator_unit[i-1].cmd->follow_pos_sync_con == FOLLOW_POS_SYNC)//新收到相关运动任务，立即控制电机
						{
							pActuator_unit[i-1].cmd->slope_position_f = position_ref;//CONSTRAIN(position_ref,slope_position_f+speed_step,slope_position_f-speed_step);
							pActuator_unit[i-1].cmd->last_position_ref = (uint16_t)pActuator_unit[i-1].cmd->slope_position_f;
							follow_mode_r(current_comm,i,pActuator_unit[i-1].cmd->last_position_ref);
							//get_cycle(pActuator_unit[i-1].motor_control_cycle);
							pActuator_unit[i-1].cmd->follow_pos_sync_con--;
						}
						else if((pActuator_unit[i-1].cmd->follow_pos_sync_con < FOLLOW_POS_SYNC)&&
										(pActuator_unit[i-1].cmd->follow_pos_sync_con > 0))//未及时收到新指令
						{
							pActuator_unit[i-1].cmd->slope_position_f = position_ref;//CONSTRAIN(position_ref,slope_position_f+speed_step,slope_position_f-speed_step);
							pActuator_unit[i-1].cmd->last_position_ref = (uint16_t)pActuator_unit[i-1].cmd->slope_position_f;
							follow_mode_r(current_comm,i,pActuator_unit[i-1].cmd->last_position_ref);
							//get_cycle(pActuator_unit[i-1].motor_control_cycle);
							pActuator_unit[i-1].cmd->follow_pos_sync_con--;
						}
						else if(pActuator_unit[i-1].cmd->follow_pos_sync_con == 0)	
						{
							pActuator_unit[i-1].cmd->control_mode = POS_SYNC_POSITION_MODE;
						}
						else
						{
							//空操作
						}
					}
				}
				else if(pActuator_unit[i-1].pos_mode == RS_POSITION_MODE)
				{
					position_mode_r(current_comm,i,pActuator_unit[i-1].cmd->position_ref);
					//get_cycle(pActuator_unit[i-1].motor_control_cycle);
					pActuator_unit[i-1].cmd->last_position_ref = pActuator_unit[i-1].cmd->position_ref;
				}
		}	
	return true;
}
/*把电机的状态数据更新到推杆执行器中存储*/
bool updata_motor_sta(RS_Actuator *pActuator,Inspire_Comm *pComm,Inspire_Data *pInspire_data)
{
	uint16_t l_u16MotorMaxPos = 0;
	
	RS_Actuator_Unit *pActuator_unit = (RS_Actuator_Unit*) pActuator;
	pActuator_unit[pComm->answer_frame->id-1].sta->last_position = pActuator_unit[pComm->answer_frame->id-1].sta->current_position;//保存上一次位置值
	
	l_u16MotorMaxPos = pActuator_unit[pComm->answer_frame->id-1].init.pos_limit_max;
	//电机当前位置
	pActuator_unit[pComm->answer_frame->id-1].sta->current_position =  CONSTRAIN(pInspire_data[pComm->answer_frame->id].status_data.current_position,l_u16MotorMaxPos,0);
	//电机当前电流
	pActuator_unit[pComm->answer_frame->id-1].sta->current =  pInspire_data[pComm->answer_frame->id].status_data.current;
	//pActuator_unit[pComm->answer_frame->id-1].sta->temperature = inspire_data[pComm->answer_frame->id].status_data.tempture; /ACC没有温度反馈
  //电机当前错误信息 	pActuator_unit[pComm->answer_frame->id-1].sta->fault_code &= 0xC0;//清除前5bit的值
	pActuator_unit[pComm->answer_frame->id-1].sta->fault_code |= (pInspire_data[pComm->answer_frame->id].status_data.stFault.u8FaultInfo & 0X3F);
	
	if(pActuator_unit[pComm->answer_frame->id-1].sta->last_position == 0)//第一次计算
	{
		pActuator_unit[pComm->answer_frame->id-1].sta->last_position = pActuator_unit[pComm->answer_frame->id-1].sta->current_position;
	}
	pActuator_unit[pComm->answer_frame->id-1].sta->speed = pActuator_unit[pComm->answer_frame->id-1].sta->current_position - pActuator_unit[pComm->answer_frame->id-1].sta->last_position;
	
	return true;
}

uint32_t clear_fault(RS_Actuator *pActuator,Inspire_Comm *pComm,Inspire_Data *pInspire_data)
{
	uint32_t ret;//按位返回每个电机是否清除过故障
	RS_Actuator_Unit *pActuator_unit = (RS_Actuator_Unit*) pActuator;
	for(int i = RS_ID_1;i<RS_ID_MAXIMUM;i++)
	{
		if(pActuator_unit[i-1].cmd->clear_fault == true)
		{
			pActuator_unit[i-1].oc->en_backword = true;
			pActuator_unit[i-1].oc->en_forword = true;
			pActuator_unit[i-1].sta->fault_code &= 0<<SOFT_OVER_CURRENT_BIT;//清除软件过流标志位
			ys_clear_fault(pComm,i);
			ys_work(pComm,i);
			ret |= (0x00000001<<(i-1));
			pActuator_unit[i-1].cmd->clear_fault = false;
		}
	}
	return ret;
}

bool hand_init(Hand *pHand,RS_Actuator *pActuator,RS_Cmd_All *pCmd)
{
	/*通过推杆位置反算手指位置，回写手指角度*/

	return true;
}
bool hand_planner(Hand *pHand,RS_Actuator *pActuator,RS_Cmd_All *pCommand)
{
	Finger *pFingle = (Finger *)&pHand->thumb;
	static float pitch_angle;
	static float speed_step_temp;
	uint16_t over_current_temp;
	uint16_t temp_position_ref;
	uint16_t temp_last_position_ref;
	/*指令动作分解区域*/
	for(int i = RS_ID_1; i < RS_ID_MAXIMUM; i++)
	{
		pitch_angle = CONSTRAIN(pFingle[i-1].pitch.angle,pFingle[i-1].pitch.angle_max,pFingle[i-1].pitch.angle_min);/*弯曲控制,给定角度限幅*/
		pFingle[i-1].actuator->cmd->speed_step = (float)pFingle[i-1].pitch_speed * pFingle[i-1].actuator->init.speed_min;  
		
		temp_position_ref = (pitch_angle + pFingle[i-1].pitch.angle_min)*ANGLE_2_POS+pFingle[i-1].pitch.pos_min ;
		temp_last_position_ref = (pFingle[i-1].last_pitch + pFingle[i-1].pitch.angle_min)*ANGLE_2_POS+pFingle[i-1].pitch.pos_min ;
		speed_step_adjust(&pFingle[i-1],pFingle[i-1].actuator,&pFingle[i-1].pitch_cmd_cycle,pFingle[i-1].actuator->motor_control_cycle,temp_position_ref,temp_last_position_ref);
		
		pFingle[i-1].actuator->cmd->position_ref = temp_position_ref;		/*B电机赋值*/
		
		pFingle[i-1].actuator->cmd->clear_fault = pFingle[i-1].clear_fault;
		pFingle[i-1].clear_fault = 0; //赋值给cmd之后清除指令，达到接收清除错误命令只清除一次错误
		over_current_temp = ((float)pFingle[i-1].pitch_current *(pFingle[i-1].actuator->init.current_max-pFingle[i-1].actuator->init.current_min)/255.0)+pFingle[i-1].actuator->init.current_min;	
		pFingle[i-1].actuator->cmd->over_current_th_ref = over_current_temp;
		pFingle[i-1].actuator->oc->count_th = pFingle[i-1].rotor_lock_count_th;
		pFingle[i-1].actuator->cmd->temperature_th = pFingle[i-1].pitch_temperature;
	}
	return true;
}

bool updata_hand_sta(Hand *pHand,RS_Actuator *pActuator,RS_Sta_All *pSta)
{
/*变量定义及初始化区开始*/
	uint8_t motor_move_count;
	Finger *pFingle = (Finger *)&pHand->thumb;
	Finger_Angle_Sta *pFinger_sta;	
	uint16_t pos_a;
/*变量定义及初始化区结束*/	
	for(int i =RS_ID_1; i<RS_ID_MAXIMUM; i++)
	{
		pFinger_sta =  &pFingle[i-1].sta;
		
		pos_a = CONSTRAIN(pFingle[i-1].actuator->sta->current_position,1600,pFingle[i-1].pitch.pos_min);
		pFinger_sta->pitch = CONSTRAIN((pos_a - pFingle[i-1].pitch.pos_min)*POS_2_ANGLE,pFingle[i-1].pitch.angle_max,pFingle[i-1].pitch.angle_min );
		
		pFinger_sta->fault = pFingle[i-1].actuator->sta->fault_code;
		
		pFinger_sta->rotor_lock_count_th =  pFingle[i-1].actuator->oc->count_th;
		/*计算三个电机的平均电流*/
		pFinger_sta->current =  pFingle[i-1].actuator->sta->current;
		/*按增量计算速度*/
		 motor_move_count = 0;
		if(ABS(pFingle[i-1].actuator->sta->speed)>2)
			{				motor_move_count++;			}
			if(motor_move_count!=0)
			{
				pFinger_sta->speed = (ABS(pFingle[i-1].actuator->sta->speed))/motor_move_count;
			}
			else
			{
				pFinger_sta->speed = 0;
			}
			pFinger_sta->pitch_temperature = MIN_LIM(pFingle[i-1].actuator->sta->temperature,0);
	}
	return true;
}
void ys_get_cmd(Hand *pHand,RS_Actuator *pActuator,Upper_Request *pRequest,Protocol_Aux_Data *aux_data)
{
	Finger *pFingle = (Finger *)&pHand->thumb;
	Finger_Upper_Cmd *pFinger_cmd = (Finger_Upper_Cmd*) pRequest;
	RS_Actuator_Unit *pActuator_unit = (RS_Actuator_Unit*) pActuator;
	static float pitch_angle;
	switch(aux_data->frame_property)
	{
		case JOINT_POSITION_RCO:
		{
			for(int i = RS_ID_1; i<RS_ID_MAXIMUM; i++)
			{
				
				pitch_angle = ((pFingle[i-1].pitch.angle_max-pFingle[i].pitch.angle_min) * pFinger_cmd[i-1].pitch_angle / 255) + pFingle[i-1].pitch.angle_min;
				if(i == RS_ID_1 || i == RS_ID_6)
				{
					pitch_angle = ((pFingle[i-1].pitch.angle_max-pFingle[i].pitch.angle_min) * (255 - pFinger_cmd[i-1].pitch_angle) / 255) + pFingle[i-1].pitch.angle_min;
				}
				 if(fabs(pitch_angle-pFingle[i-1].pitch.angle)>0.1)
				 {
						pFingle[i-1].last_pitch = pFingle[i].pitch.angle;
						pFingle[i-1].pitch.angle= pitch_angle;
						pFingle[i-1].actuator->cmd->follow_pos_sync_con = FOLLOW_POS_SYNC;
				 }
				 //get_cycle(&pFingle[i-1].pitch_cmd_cycle);			
			}
		}
		break;
		case SPEED_RCO:
		{
			for(int i = RS_ID_1; i < RS_ID_MAXIMUM-1; i++)
			{
				pFingle[i-1].speed = pFinger_cmd[i-1].speed_ref;
				pFingle[i-1].pitch_speed = pFinger_cmd[i-1].pitch_speed;
			}
		}
		break;
		default:
		break;
	}
}
//设置AAC电机状态
void ys_set_status(Hand *pHand,Inspire_Data *pInspre ,Lower_Response *lower_response)
{
	Finger *pFingle = (Finger *)&pHand->thumb;
	RS_Actuator_Unit *actuator = NULL;
	UNUSED(pFingle);
	Finger_Lower_Sta *pSta = (Finger_Lower_Sta *)&lower_response->thumb;

	for(int i =RS_ID_1; i < RS_ID_MAXIMUM; i++)
	{
		pSta[i-1].pitch_angle = (pFingle[i-1].pitch.angle - pFingle[i-1].pitch.angle_min)*255/(pFingle[i-1].pitch.angle_max - pFingle[i-1].pitch.angle_min);
		if(i == RS_ID_1 || i == RS_ID_6)
		{
			pSta[i-1].pitch_angle = (pFingle[i-1].pitch.angle_max - pFingle[i-1].pitch.angle)*255/(pFingle[i-1].pitch.angle_max - pFingle[i-1].pitch.angle_min);
		}
		actuator = pFingle[i-1].actuator;
		pSta[i-1].pitch_speed = float_2_u8(actuator->sta->speed,actuator->init.speed_max,actuator->init.speed_min,255,0,false);
		pSta[i-1].pitch_current = float_2_u8(actuator->sta->current,actuator->init.current_max,0,255,0,false);
		pSta[i-1].pitch_temperature =  MIN_LIM(actuator->sta->temperature,0);
		
					
		pSta[i-1].speed = pFingle[i-1].pitch_speed;
		pSta[i-1].current = pFingle[i-1].pitch_current;
		pSta[i-1].fault |= pFingle[i-1].clear_fault;
				
	}
}

//手部控制及电机控制超时检测
void hand_control_timeout_detection(Hand *pHand,RS_Actuator *pActuator)
{
	Finger *pFingle = (Finger *)&pHand->thumb;
	RS_Actuator_Unit *pActuator_unit = (RS_Actuator_Unit*) pActuator;
	Time_Stamp* pTime_Stamp;
	for(int i =RS_ID_1; i<RS_ID_MAXIMUM; i++)
	{
		pTime_Stamp = &pFingle[i-1].pitch_cmd_cycle;
		//timeout_detection(pTime_Stamp);
		//timeout_detection(pTime_Stamp);
		//timeout_detection(pTime_Stamp);
		//timeout_detection(pTime_Stamp);
	}
	for(int i = RS_ID_1;i<RS_ID_MAXIMUM;i++)
	{
		if(pActuator_unit[i-1].is_actuator_valid == true)
		{
			pTime_Stamp = pActuator_unit[i-1].motor_control_cycle;
			//timeout_detection(pTime_Stamp);
		}
	}
}

/*
  pFingle		            手指数据结构，含有电机的目标控制信息
  actuator		          当前电机执行单元
  cmd_cycle	            上一条指令到这一条指令之间的时间信息
  motor_control_cycle		电机实际运行控制周期信息
  pos	uint16_t	        当前目标位置
  last_pos		          上一次的目标位置
*/
float speed_step_adjust(Finger *pFingle, RS_Actuator_Unit *actuator, Time_Stamp *cmd_cycle, Time_Stamp *motor_control_cycle, uint16_t pos, uint16_t last_pos)
{
    float distance = ABS((int16_t)pos - (int16_t)actuator->sta->current_position);
    float speed_step = actuator->cmd->speed_step;
    float speed_max = actuator->init.speed_max;
    float speed_min = actuator->init.speed_min;

    // 上位机255时直接全速
    if (pFingle->pitch_speed == 255) {
        actuator->cmd->speed_step_real = 100;
        return speed_max;
    }

    // 距离大于某阈值时全速，小于阈值时减速
    if (distance > 3 * speed_step * speed_min) {
        actuator->cmd->speed_step_real = speed_max;
    } else if (distance > speed_step * speed_min) {
        actuator->cmd->speed_step_real = speed_step;
    } else {
        actuator->cmd->speed_step_real = speed_min;
    }
    return actuator->cmd->speed_step_real;
}

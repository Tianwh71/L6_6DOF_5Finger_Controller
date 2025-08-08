/**
 ********************************** Copyright *********************************
 *
 ** (C) Copyright 2022-2024 YaoYandong,China.
 ** All Rights Reserved.
 *
 ******************************************************************************
 **--------------------------------------------------------------------------**
 ** @FileName      : upper_comm_protocol_fml.c
 ** @Brief         : 上位机通信协议构建，编解码
 **--------------------------------------------------------------------------**
 ** @Author Data   : Depressed 2024-04-10
 ** @Version       : v1.0
 **--------------------------------------------------------------------------**
 ** @Modfier Data  : None
 ** @Version       : None
 ** @Description   : 485，can,以太网都依赖这一个协议层
 **--------------------------------------------------------------------------**
 ** @Function List : None
 **--------------------------------------------------------------------------**
 ** @Attention     : None
 **--------------------------------------------------------------------------**
 ******************************************************************************
 *
 **/
#include "upper_comm_protocol_fml.h"
#include "string.h"
#include "data_structure.h"
#include "event_groups.h"
#include "semphr.h"
#include "stochastic_pooling.h"
#include "upper_can_comm_drv.h"
#include <stdlib.h>
#include "my_math.h"
#include "Common.h"
#include "AAC_MotorControl_fml.h"

Hand_Action hand_action = 
{
	 .speed = 0x10,						//轨迹规划速度
	 .amplitude = 0x80,				//动作幅值
	 .number_of_turns = 0x64,	//圈数
	 .finger_num = THUMB,			//单手指动作时指定的手指序号。部分指令用到
	 .action_end = false,			//动作执行完毕
};
Upper_Request upper_request = 
{
	.thumb = 
	{
	 .speed_ref =DEFAULT_SPEED,				//速度控制
	 .over_current_th = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.rotor_lock_count_th = OVER_CURRENT_DETECTION_TH,
	},
	.index = 
	{
	 .speed_ref =DEFAULT_SPEED,				//速度控制
	 .over_current_th = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.rotor_lock_count_th = OVER_CURRENT_DETECTION_TH,
	},
	.middle = 
	{
	 .speed_ref =DEFAULT_SPEED,				//速度控制
	 .over_current_th = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.rotor_lock_count_th = OVER_CURRENT_DETECTION_TH,
	},
	.ring = 
	{
	 .speed_ref =DEFAULT_SPEED,				//速度控制
	 .over_current_th = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.rotor_lock_count_th = OVER_CURRENT_DETECTION_TH,
	},
	.little = 
	{
	 .speed_ref =DEFAULT_SPEED,				//速度控制
	 .over_current_th = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
	.rotor_lock_count_th = OVER_CURRENT_DETECTION_TH,
	},
	.thumb_yaw = 
	{
	 .speed_ref =DEFAULT_SPEED,				//速度控制
	 .over_current_th = (OVER_CURRENT_TH- OVER_CURRENT_TH_MIN)*255/(OVER_CURRENT_TH_MAX-OVER_CURRENT_TH_MIN),	//过流设置
		.rotor_lock_count_th = OVER_CURRENT_DETECTION_TH,
	},
	.pTip_Data_All = &finger_tip_data_all,
	.pAction = &hand_action,
};
Lower_Response lower_response = 
{
	.pTip_Data_All = &finger_tip_data_all,
	.pAction = &hand_action,
	.hwk_hand_sensor = &hwk_hand_sensor,
};
Protocol_Aux_Data protocol_aux_data;
Upper_Protocol upper_protocol= 
{
	.upper_request = &upper_request,
	.lower_response = &lower_response,
	.protocol_aux_data = &protocol_aux_data,
};

/*
 *
 * CRC-8 formula from page 14 of SHT spec pdf
 *
 * Test data 0xBE, 0xEF should yield 0x92
 *
 * Initialization data 0xFF
 * Polynomial 0x31 (x8 + x5 +x4 +1)
 * Final XOR 0x00
 */
uint8_t Crc8(uint8_t *data, uint8_t nbrOfBytes)
{
	uint8_t crc8 = 0xFF, bit = 0;
	while (nbrOfBytes--)
	{
		crc8 ^= *data++;
		for (bit = 8; bit > 0; --bit)
		{
			crc8 = (crc8 & 0x80)
								 ? (crc8 << 1) ^ CRC8_POLYNOMIAL
								 : (crc8 << 1);
		}
	}
	return crc8;
}

// can接收解析
void comm_can_parser(Upper_Can_Transmit *can_trans, Upper_Request *upper_request, Protocol_Aux_Data *aux_data)
{
	Finger_Upper_Cmd *pFinger_cmd = (Finger_Upper_Cmd*) upper_request;
	if (can_trans->receive_flag == false)
	{
		return;
	}
	aux_data->comm_interface = COMM_CAN; // 标记使用的是can接口
	switch ((FRAME_PROPERTY)can_trans->receive_buf[0])
	{
	case JOINT_POSITION_RCO: /*上位侧写入一次关节位置，返回一次关节位置*/
	{
		if (can_trans->receive_len == 7)
		{
			upper_request->thumb.pitch_angle = (can_trans->receive_buf[1] < 5) ? 5 : can_trans->receive_buf[1];   //由于硬件限位增加边界检查
			upper_request->index.pitch_angle = can_trans->receive_buf[2];
			upper_request->middle.pitch_angle = can_trans->receive_buf[3];
			upper_request->ring.pitch_angle = can_trans->receive_buf[4];
			upper_request->little.pitch_angle = can_trans->receive_buf[5];
			upper_request->thumb_yaw.pitch_angle = (can_trans->receive_buf[6] < 15) ? 15 : can_trans->receive_buf[6];
			aux_data->return_frame_makers |= RETURN_POSITION;
			aux_data->frame_property = JOINT_POSITION_RCO;
		}
		else if (can_trans->receive_len == 1)
		{
			aux_data->return_frame_makers |= RETURN_POSITION;
		}
		else
		{
			aux_data->return_frame_makers |= RETURN_ERROR;
			aux_data->frame_property = ERROR_CODE;
		}
	}
	break;
	case MAX_PRESS_RCO:
	{
		
	}
	break;
	case SPEED_RCO:
	{
		if(can_trans->receive_len == 6)	{
				for(int i = 0; i < 6; i++){
						pFinger_cmd[i].pitch_speed = can_trans->receive_buf[i+1];
						pFinger_cmd[i].speed_ref =can_trans->receive_buf[i+1];
					}
			aux_data->return_frame_makers |= RETURN_SPEED;
			aux_data->frame_property = SPEED_RCO;
		}
		else
		{
			aux_data->return_frame_makers |= RETURN_ERROR;
			aux_data->frame_property = ERROR_CODE;
		}
	}
	break;
	case Temp1:
	{
		if(can_trans->receive_len == 1)
		{
			aux_data->return_frame_makers |= RETURN_TEMP1;
			aux_data->frame_property = Temp1;
		}
	}
	break;
	case Temp2:
	{
			aux_data->return_frame_makers |= RETURN_TEMP2;
			aux_data->frame_property = Temp2;
	}
	break;
	case Version:
	{
		if(can_trans->receive_len == 1)
		{
			uint8_t Version_data[8];
			Version_data[0] = Version;					// 功能码
			Version_data[1] = Hand_Freedom;			// 手自由度
			Version_data[2] = Hand_Version;			// 手机械版本
			Version_data[3] = Hand_Number;			// 手机械版本序号			
			Version_data[4] = Hand_Direction;		// 左右手对应LR的ASCII码
			Version_data[5] = SoftWare_Version; // 软件版本编号
			Version_data[6] = HardWare_Version; // 硬件版本编号
			Version_data[7] = Revision_State;		// 修订状态
			
			FDCAN2_Send_Msg(Version_data, 8, SELF_ID); // 发送
		}
	}
	break;
	case Error_Code1:
	{
		if (can_trans->receive_len == 1)
		{
			aux_data->return_frame_makers |= RETURN_ERROR_CODE1;
			aux_data->frame_property = Error_Code1;
		}
		else
		{
			aux_data->return_frame_makers |= RETURN_ERROR;
			aux_data->frame_property = ERROR_CODE;
		}
	}
	break;
	case SYS_RESET:
	{
				__HAL_RCC_CLEAR_RESET_FLAGS();
			  osDelay(500);
			  HAL_NVIC_SystemReset();
	}
	break;
	default:
		break;
}
}
// can发送
void comm_can_send(Upper_Can_Transmit *can_trans, Lower_Response *lower_response, FRAME_PROPERTY frame_property)
{
	switch (frame_property)
	{
		case JOINT_POSITION_RCO:
		{
			can_trans->send_buf[0] = JOINT_POSITION_RCO;
			can_trans->send_buf[1] = lower_response->thumb.pitch_angle; // 大拇指关节角度
			can_trans->send_buf[2] = lower_response->index.pitch_angle;
			can_trans->send_buf[3] = lower_response->middle.pitch_angle;
			can_trans->send_buf[4] = lower_response->ring.pitch_angle;
			can_trans->send_buf[5] = lower_response->little.pitch_angle;
			can_trans->send_buf[6] = lower_response->thumb_yaw.pitch_angle;
			FDCAN2_Send_Msg(can_trans->send_buf, 7, SELF_ID);
		}
		break;
		case MAX_PRESS_RCO:
		{
	//		can_trans->send_buf[0] = MAX_PRESS_RCO;
	//		can_trans->send_buf[1] = lower_response->curr_pressure_1; // 大拇指力矩
	//		can_trans->send_buf[2] = lower_response->curr_pressure_2;
	//		can_trans->send_buf[3] = lower_response->curr_pressure_3;
	//		can_trans->send_buf[4] = lower_response->curr_pressure_4;
	//		can_trans->send_buf[5] = lower_response->curr_pressure_5;
	//		can_trans->send_buf[6] = lower_response->curr_pressure_6;
	//		FDCAN2_Send_Msg(can_trans->send_buf, 7, SELF_ID);
		}
		break;
		case SPEED_RCO:
		{
			can_trans->send_buf[0] = JOINT_POSITION_RCO;
			can_trans->send_buf[1] = lower_response->thumb.pitch_speed; // 大拇指关节角度
			can_trans->send_buf[2] = lower_response->index.pitch_speed;
			can_trans->send_buf[3] = lower_response->middle.pitch_speed;
			can_trans->send_buf[4] = lower_response->ring.pitch_speed;
			can_trans->send_buf[5] = lower_response->little.pitch_speed;
			can_trans->send_buf[6] = lower_response->thumb_yaw.pitch_speed;
			FDCAN2_Send_Msg(can_trans->send_buf, 7, SELF_ID);
		}
		break;

		case Temp1:
		{
//			can_trans->send_buf[0] = Temp1;
//			can_trans->send_buf[1] = lower_response->thumb.pitch_temperature;
//			can_trans->send_buf[2] = lower_response->index.pitch_temperature;
//			can_trans->send_buf[3] = lower_response->middle.pitch_temperature;
//			can_trans->send_buf[4] = lower_response->ring.pitch_temperature;
//			can_trans->send_buf[5] = lower_response->little.pitch_temperature;
//			can_trans->send_buf[6] = lower_response->thumb_yaw.pitch_temperature;
//			FDCAN2_Send_Msg(can_trans->send_buf, 7, SELF_ID);
		}
		break;
		case Temp2:
		{
		}
		break;
		case Error_Code1:
		{
		}
		break;
		default:
			break;
	}
}
void event_can_dispose(void)
{
	comm_can_parser(&upper_can_transmit, &upper_request, &protocol_aux_data); // can解析
	// comm_can_touch_sensor_parser(&can_rx_get_queue, &upper_request, &lower_response, &protocol_aux_data);
	/*飞特*/
	ys_get_cmd(&hand,&rs_actuator,&upper_request,&protocol_aux_data);
	// 在发送之前准备好数据
	if (protocol_aux_data.return_frame_makers & RETURN_POSITION)
	{
		comm_can_send(&upper_can_transmit, &lower_response, JOINT_POSITION_RCO);
	}
	if (protocol_aux_data.return_frame_makers & RETURN_PRESS1)
	{
		comm_can_send(&upper_can_transmit, &lower_response, MAX_PRESS_RCO);
	}
	if (protocol_aux_data.return_frame_makers & RETURN_SPEED)
	{
		comm_can_send(&upper_can_transmit, &lower_response, SPEED_RCO);
	}
	if (protocol_aux_data.return_frame_makers & RETURN_ACCELERATION)
	{
		comm_can_send(&upper_can_transmit, &lower_response, ACCELERATION_RCO);
	}
	if (protocol_aux_data.return_frame_makers & RETURN_TEMP1)
	{
		comm_can_send(&upper_can_transmit, &lower_response, Temp1);
	}
	if (protocol_aux_data.return_frame_makers & RETURN_TEMP2)
	{
		comm_can_send(&upper_can_transmit, &lower_response, Temp2);
	}
	if (protocol_aux_data.return_frame_makers & RETURN_ERROR_CODE1)
	{
		comm_can_send(&upper_can_transmit, &lower_response, Error_Code1);
	}

	if (protocol_aux_data.return_frame_makers & RETURN_ERROR)
	{
		comm_can_send(&upper_can_transmit, &lower_response, ERROR_CODE);
	}

	protocol_aux_data.return_frame_makers = RETURN_NONE;

	xSemaphoreGive(UpperControlSemHandle); // 收到上位机控制指令，通过信号量通知电机控制任务
}

void event_can_tip_sensor_dispose(void)
{
	comm_can_touch_sensor_parser(&can_rx_get_queue, &upper_request, &lower_response, &protocol_aux_data);
	if (protocol_aux_data.return_touch_sensor_makers & RETURN_THUMB_TOUCH)
	{
		comm_can_touch_sensor_send(&upper_can_transmit, &lower_response, THUMB_TOUCH);
	}
	if (protocol_aux_data.return_touch_sensor_makers & RETURN_INDEX_TOUCH)
	{
		comm_can_touch_sensor_send(&upper_can_transmit, &lower_response, INDEX_TOUCH);
	}
	if (protocol_aux_data.return_touch_sensor_makers & RETURN_MIDDLE_TOUCH)
	{
		comm_can_touch_sensor_send(&upper_can_transmit, &lower_response, MIDDLE_TOUCH);
	}
	if (protocol_aux_data.return_touch_sensor_makers & RETURN_RING_TOUCH)
	{
		comm_can_touch_sensor_send(&upper_can_transmit, &lower_response, RING_TOUCH);
	}
	if (protocol_aux_data.return_touch_sensor_makers & RETURN_LITTLE_TOUCH)
	{
		comm_can_touch_sensor_send(&upper_can_transmit, &lower_response, LITTLE_TOUCH);
	}
	protocol_aux_data.return_touch_sensor_makers = RETURN_NONE;
}

void comm_can_touch_sensor_send(Upper_Can_Transmit *can_trans, Lower_Response *pResponse, FRAME_PROPERTY frame_property)
{
	// 请求返回的矩阵大小
	int rows = 12, cols = 6;
	int row_pool, col_pool;
	int cmd_rows = GET_MINIMUM(12, pResponse->matrix_size.row);
	int cmd_cols = GET_MINIMUM(8, pResponse->matrix_size.column);
	row_pool = rows / cmd_rows;
	col_pool = cols / cmd_cols;
	int new_rows = rows / row_pool;
	int new_cols = cols / col_pool;
	float **output = allocate_matrix(new_rows, new_cols);
	float temperature = 2.0f; // 温度参数越大，权重分布越平缓
	can_trans->send_len = 0;
	//	uint16_t temp_u12;
	Matrix_Size_u8 matrix_size;
	float **input = allocate_matrix(rows, cols);
	switch (frame_property)
	{
		//	case TOUCH_SENSOR_TYPE:
		//	{
		//		can_trans->send_buf[0] = TOUCH_SENSOR_TYPE;
		//		can_trans->send_buf[1] = tip_sensor_select;
		//		can_trans->send_len = 2;
		//		FDCAN2_Send_Msg(can_trans->send_buf, can_trans->send_len, SELF_ID);
		//	}
		//	break;
	case THUMB_TOUCH:
	{
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				input[i][j] = pResponse->hwk_hand_sensor->thumb.data[i][j];
			}
		}
	}
	break;
	case INDEX_TOUCH:
	{
		for (int i = 0; i < rows; i++)
		{
			int reversed_i = rows-1-i;
			for (int j = 0; j < cols; j++)
			{
				int reversed_j = cols-1-j;
				input[reversed_i][reversed_j] = pResponse->hwk_hand_sensor->index.data[i][j];
			}
		}
	}
	break;
	case MIDDLE_TOUCH:
	{
		for (int i = 0; i < rows; i++)
		{
			int reversed_i = rows-1-i;
			for (int j = 0; j < cols; j++)
			{
				int reversed_j = cols-1-j;
				input[reversed_i][reversed_j] = pResponse->hwk_hand_sensor->middle.data[i][j];
			}
		}
	}
	break;
	case RING_TOUCH:
	{
		for (int i = 0; i < rows; i++)
		{
			int reversed_i = rows-1-i;
			for (int j = 0; j < cols; j++)
			{
				int reversed_j = cols-1-j;
				input[reversed_i][reversed_j] = pResponse->hwk_hand_sensor->ring.data[i][j];
			}
		}
	}
	break;
	case LITTLE_TOUCH:
	{
		for (int i = 0; i < rows; i++)
		{
			int reversed_i = rows-1-i;
			for (int j = 0; j < cols; j++)
			{
				int reversed_j = cols-1-j;
				input[reversed_i][reversed_j] = pResponse->hwk_hand_sensor->little.data[i][j];
			}
		}
	}
	break;
	case PALM_TOUCH:
	{

		free_matrix(input, rows);
		rows = 12;
		cols = 8;
		int row_pool, col_pool;
		cmd_rows = GET_MINIMUM(12, pResponse->matrix_size.row);
		cmd_cols = GET_MINIMUM(8, pResponse->matrix_size.column);
		row_pool = rows / cmd_rows;
		col_pool = cols / cmd_cols;
		new_rows = rows / row_pool;
		new_cols = cols / col_pool;
		float **output = allocate_matrix(new_rows, new_cols);
		input = allocate_matrix(rows, cols);

		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				input[i][j] = pResponse->hwk_hand_sensor->palm.data[i][j];
			}
		}
	}
	break;                
	default:
	{
		goto ret;
	}
		//	break;
	}
	weighted_avg_pooling_variable_temp(input, rows, cols, row_pool, col_pool, output, temperature);
	uint8_t *send_buf_ful = malloc(new_rows * new_cols * sizeof(uint8_t *));
	can_trans->send_buf[0] = frame_property;
	can_trans->send_len++;
	for (int i = 0; i < new_rows; i++)
	{
		for (int j = 0; j < new_cols; j++)
		{
#if (C2_DATA_WIDTH == C2_DATA_WIDTH_1_BYTE)
			send_buf_ful[i * new_cols + j] = output[i][j];
#elif (C2_DATA_WIDTH == C2_DATA_WIDTH_2_BYTE)
			temp_u12 = MAX_LIM(output[i][j], 0xFFF);
			send_buf_ful[i * new_cols + j] = (temp_u12 / 16);
#endif
			if (can_trans->send_len == 1)
			{
				matrix_size.row = i;
				matrix_size.column = j;
				can_trans->send_buf[1] = *(uint8_t *)&matrix_size;
				can_trans->send_len++;
			}
			can_trans->send_buf[can_trans->send_len] = send_buf_ful[i * new_cols + j];
			can_trans->send_len++;
			if (can_trans->send_len == 8)
			{

				FDCAN2_Send_Msg(can_trans->send_buf, can_trans->send_len, SELF_ID);
				can_trans->send_len = 1;
				osDelay(1); // 发送完一帧数据后，延时1ms，等待CAN总线空闲
										// 如果阻塞未正常工作，可以加1ms延时
			}
		}
	}
	if (can_trans->send_len > 2)
	{
		FDCAN2_Send_Msg(can_trans->send_buf, can_trans->send_len, SELF_ID);
		can_trans->send_len = 0;
	}

	free(send_buf_ful);
	send_buf_ful = NULL;
ret:
	free_matrix(input, rows);
	free_matrix(output, new_rows);
	;
}

// 仅解析指尖传感器部分
void comm_can_touch_sensor_parser(Can_Rx_Queue *pRx_queue, Upper_Request *pRequest, Lower_Response *pResponse, Protocol_Aux_Data *aux_data)
{
	if (pRx_queue->receive_flag == false)
	{
		return;
	}
	aux_data->comm_interface = COMM_CAN; // 标记使用的是can接口
	if (((FRAME_PROPERTY)pRx_queue->receive_buf[0] >= THUMB_TOUCH) && ((FRAME_PROPERTY)pRx_queue->receive_buf[0] <= PALM_TOUCH))
	{
		// aux_data->frame_property = (FRAME_PROPERTY)pRx_queue->receive_buf[0];
		pRx_queue->receive_flag = false; // 数据已被使用
		if (pRx_queue->receive_len == 2)
		{
			pRequest->matrix_size.row = (pRx_queue->receive_buf[1] & 0XF0) >> 4;
			pRequest->matrix_size.column = pRx_queue->receive_buf[1] & 0X0F;
			pResponse->matrix_size = pRequest->matrix_size;
		}
		else
		{
			pResponse->matrix_size.row = 1;
			pResponse->matrix_size.column = 1;
		}
		switch ((FRAME_PROPERTY)pRx_queue->receive_buf[0])
		{
		case THUMB_TOUCH:
		{
			aux_data->return_touch_sensor_makers |= RETURN_THUMB_TOUCH;
		}
		break;
		case INDEX_TOUCH:
		{
			aux_data->return_touch_sensor_makers |= RETURN_INDEX_TOUCH;
		}
		break;
		case MIDDLE_TOUCH:
		{
			aux_data->return_touch_sensor_makers |= RETURN_MIDDLE_TOUCH;
		}
		break;
		case RING_TOUCH:
		{
			aux_data->return_touch_sensor_makers |= RETURN_RING_TOUCH;
		}
		break;
		case LITTLE_TOUCH:
		{
			aux_data->return_touch_sensor_makers |= RETURN_LITTLE_TOUCH;
		}
		break;
		case PALM_TOUCH:
		{
			aux_data->return_touch_sensor_makers |= RETURN_PALM_TOUCH;
		}
		break;

		default:
		{
			pRx_queue->receive_flag = true; // 更正数据状态，数据未被使用
		}
		break;
		}
	}
}
/*
位置回写，在获取关节数据之后，将电机当前位置作为目标位置，
*/
void init_request_data(Upper_Request *request,Lower_Response *pResponse)
{
	Finger_Upper_Cmd *pFinger_cmd = (Finger_Upper_Cmd*) request;
	Finger_Lower_Sta *pFinger_sta = (Finger_Lower_Sta *)&pResponse->thumb;
	/*外部*/
	Finger *pFinger = (Finger*) &hand;
	/*外部*/
	for(int i = 0;i<6;i++)
	{
		pFinger_cmd[i].pitch_angle = pFinger_sta[i].pitch_angle;
		
		pFinger_cmd[i].speed_ref = pFinger[i].speed;
		pFinger_cmd[i].pitch_speed = pFinger[i].pitch_speed;
		
		pFinger_cmd[i].over_current_th = pFinger[i].current;
		pFinger_cmd[i].pitch_current = pFinger[i].pitch_current;
		
		pFinger_cmd[i].pitch_temperature = pFinger[i].pitch_temperature;
		
		pFinger_cmd[i].clear_fault = 0 ;
		pFinger_cmd[i].rotor_lock_count_th = pFinger[i].rotor_lock_count_th;
	}
}

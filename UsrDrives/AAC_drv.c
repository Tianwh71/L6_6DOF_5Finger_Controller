/**
  ********************************** Copyright *********************************
  *
  ** (C) Copyright 2022-2024 YaoYandong,China.
  ** All Rights Reserved.
  *                              
  ******************************************************************************
  **--------------------------------------------------------------------------**
  ** @FileName      : ACC_drv.c  
  ** @Brief         : None
  **--------------------------------------------------------------------------**
  ** @Author Data   : Depressed 2025-6-6
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
#include "AAC_drv.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "AAC_MotorControl_fml.h"
#include "data_structure.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <stdlib.h>

Instruction_Frame instruction_frame_2 = {0};
Instruction_Frame instruction_frame_3 = {0};
Instruction_Frame instruction_frame_4 = {0};
Answer_Frame answer_frame_2 = {0} ;
Answer_Frame answer_frame_3 = {0} ;
Answer_Frame answer_frame_4 = {0} ;
YS_transmit ys_transmit_2 = 
{
	.nop_count = INSTRUCTION_INTERVAL_TIME,
};
YS_transmit ys_transmit_3 = 
{
	.nop_count = INSTRUCTION_INTERVAL_TIME,
};
YS_transmit ys_transmit_4 = 
{
	.nop_count = INSTRUCTION_INTERVAL_TIME,
};
Inspire_Comm inspire_comm[3] = {
		{
			.instruction_frame = &instruction_frame_2,
			.ys_transmit = &ys_transmit_2,
			.answer_frame = &answer_frame_2,
			.trans_block_mode = WAIT_SEM_NOP,

		}, // 关于电机控制、数据存储的总结构体。串口2
		{
			.instruction_frame = &instruction_frame_3,
			.ys_transmit = &ys_transmit_3,
			.answer_frame = &answer_frame_3,
			.trans_block_mode = WAIT_SEM_NOP,
		}, // 关于电机控制、数据存储的总结构体。串口3
		{
			.instruction_frame = &instruction_frame_4,
			.ys_transmit = &ys_transmit_4,
			.answer_frame = &answer_frame_4,
			.trans_block_mode = WAIT_SEM_NOP,

		} // 关于电机控制、数据存储的总结构体。串口4
	};
//串口初始化
void ys_usart_user_init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t *)ys_transmit_2.receive_buf, YS_TRANSMIT_LEN_MAXIMUM);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,(uint8_t *)ys_transmit_3.receive_buf, YS_TRANSMIT_LEN_MAXIMUM);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4,(uint8_t *)ys_transmit_4.receive_buf, YS_TRANSMIT_LEN_MAXIMUM);
	xSemaphoreGive(YS_Trans_SemHandle);//第一次初始化，释放一次信号量
};

//串口空闲中断接收
bool HAL_UART_Receive_IDLE(UART_HandleTypeDef *huart,YS_transmit *tran_handle, uint16_t Size)
{
	  bool ret = false;
	  BaseType_t xHigherPriorityTaskWoken=pdFALSE;
	
		__HAL_UART_CLEAR_IDLEFLAG(huart); //清除中断标志
		HAL_UART_DMAStop(huart);//停止DMA接收

		tran_handle->receive_len = Size;  //记录接收数据长度
		tran_handle->receive_flag = true;  //置起接收标志
		// 完整帧接收完成，触发事件
		switch((uint32_t)huart->Instance) {
				case USART2_BASE:
						ret = (xEventGroupSetBitsFromISR(UART_ReceivedEventHandle, RX_UART2_EVENT, &xHigherPriorityTaskWoken) == pdPASS);
						break;
				case USART3_BASE:
						ret = (xEventGroupSetBitsFromISR(UART_ReceivedEventHandle, RX_UART3_EVENT, &xHigherPriorityTaskWoken) == pdPASS);
						break;
				case UART4_BASE:
						ret = (xEventGroupSetBitsFromISR(UART_ReceivedEventHandle, RX_UART4_EVENT, &xHigherPriorityTaskWoken) == pdPASS);
						break;
                }
		 if(xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

	return ret;
}
void clear_usart2_fault(void)
{
		if (((huart2.Instance->ISR & UART_FLAG_FE) != RESET))
	{
		__HAL_UART_CLEAR_FEFLAG(&huart2);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ys_transmit_2.receive_buf, YS_DATA_LEN_MAXIMUM);
	}
	if (((huart2.Instance->ISR & UART_FLAG_ORE) != RESET))
	{
		__HAL_UART_CLEAR_OREFLAG(&huart2);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ys_transmit_2.receive_buf, YS_DATA_LEN_MAXIMUM);
	}	
	if (((huart2.Instance->ISR & UART_FLAG_NE) != RESET))
	{
		__HAL_UART_CLEAR_NEFLAG(&huart2);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ys_transmit_2.receive_buf, YS_DATA_LEN_MAXIMUM);	
	}	

}
void clear_usart3_fault(void)
{
		if (((huart3.Instance->ISR & UART_FLAG_FE) != RESET))
	{
		__HAL_UART_CLEAR_FEFLAG(&huart3);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, ys_transmit_3.receive_buf, YS_DATA_LEN_MAXIMUM);
	}
	if (((huart3.Instance->ISR & UART_FLAG_ORE) != RESET))
	{
		__HAL_UART_CLEAR_OREFLAG(&huart3);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, ys_transmit_3.receive_buf, YS_DATA_LEN_MAXIMUM);
	}	
	if (((huart3.Instance->ISR & UART_FLAG_NE) != RESET))
	{
		__HAL_UART_CLEAR_NEFLAG(&huart3);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, ys_transmit_3.receive_buf, YS_DATA_LEN_MAXIMUM);
	}	

}

void clear_usart4_fault(void)
{
		if (((huart4.Instance->ISR & UART_FLAG_FE) != RESET))
	{
		__HAL_UART_CLEAR_FEFLAG(&huart4);
		
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, ys_transmit_4.receive_buf, YS_DATA_LEN_MAXIMUM);
	}
	if (((huart4.Instance->ISR & UART_FLAG_ORE) != RESET))
	{
		__HAL_UART_CLEAR_OREFLAG(&huart4);
		
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, ys_transmit_4.receive_buf, YS_DATA_LEN_MAXIMUM);
	}	
	if (((huart4.Instance->ISR & UART_FLAG_NE) != RESET))
	{
		__HAL_UART_CLEAR_NEFLAG(&huart4);
		
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, ys_transmit_4.receive_buf, YS_DATA_LEN_MAXIMUM);
	}	

}
static uint8_t sum_check(uint8_t *buf,uint8_t len)
{
	uint8_t sum = 0;
	for(int i= 0;i< len;i++)
	{
		sum+=buf[i];
	}
	return sum;
}

bool build_ins_frame(Inspire_Comm *pComm	,	uint8_t id	,	CMD_TYPE cmd_type	,	MEMORY_TABLE memory_table	,	uint8_t *data	,	uint8_t data_len)
{
	pComm->instruction_frame->head = YS_INS_FRAME_HEAD;
	pComm->instruction_frame->len = data_len+2;
	pComm->instruction_frame->id = id;
	pComm->instruction_frame->cmd_type = cmd_type;
	pComm->instruction_frame->table = memory_table; 
	memcpy(pComm->instruction_frame->data,data,data_len);
	
	pComm->ys_transmit->send_buf[0] = (YS_INS_FRAME_HEAD&0xff00)>>8;
	pComm->ys_transmit->send_buf[1] = YS_INS_FRAME_HEAD&0x00ff;
	pComm->ys_transmit->send_buf[2] = data_len+2;

	
	pComm->ys_transmit->send_buf[3] = id;
	pComm->ys_transmit->send_buf[4] = cmd_type;
	pComm->ys_transmit->send_buf[5] = memory_table;
	memcpy((void*)&pComm->ys_transmit->send_buf[6],data,data_len);
	
	pComm->instruction_frame->sum_check = sum_check((uint8_t*)&pComm->ys_transmit->send_buf[2],data_len+4);
	pComm->ys_transmit->send_buf[data_len+6] = pComm->instruction_frame->sum_check;	
	pComm->ys_transmit->send_len = data_len+7;
	//发送区
	
	//获取信号量
	pComm->ys_transmit->send_en_flag = true;
	if(id == YS_ID_1 || id == YS_ID_2)
	{
		HAL_UART_Transmit_DMA(&huart2,(void*)pComm->ys_transmit->send_buf,pComm->ys_transmit->send_len);
	}
	else if(id == YS_ID_3 || id == YS_ID_4)
	{
		HAL_UART_Transmit_DMA(&huart3,(void*)pComm->ys_transmit->send_buf,pComm->ys_transmit->send_len);
	}
	else if(id == YS_ID_5 || id == YS_ID_6)
	{
		HAL_UART_Transmit_DMA(&huart4,(void*)pComm->ys_transmit->send_buf,pComm->ys_transmit->send_len);
	}
	return true;
}
//ACC读取内存表信息解析
bool decode_rd_ans_frame(Inspire_Comm *pComm,Inspire_Data *pInspire_data)
{
	if(pComm->answer_frame->cmd_type != CMD_RD)
	{
		pComm->comm_detection.decode_fault.no_match_cmd_type = 1;
		return false;
	}
	if(pComm->answer_frame->len>(2+2))//内存表数据分布十分分散，不适合连续读取，所以只解析读取字节和半字等情况
	{
		pComm->comm_detection.decode_fault.no_match_data_len = 1;
		return false;
	}
	switch(pComm->answer_frame->table)
	{
		case YS_ID://电缸ID
		{
			pInspire_data[pComm->answer_frame->id].config_data.id = pComm->answer_frame->data[0]+(pComm->answer_frame->data[1]<<8);
		}break;
		case YS_SOFT_VERSION://软件版本号
		{
			pInspire_data[pComm->answer_frame->id].config_data.soft_version = pComm->answer_frame->data[0]+(pComm->answer_frame->data[1]<<8);
		}break;
		case YS_BAUDRATE://串口波特率
		{
			pInspire_data[pComm->answer_frame->id].config_data.baudrate = pComm->answer_frame->data[0]+(pComm->answer_frame->data[1]<<8);
		}break;
		case YS_OVER_CURRENT_TH://过流保护设置
		{
			pInspire_data[pComm->answer_frame->id].config_data.over_current_th = pComm->answer_frame->data[0]+(pComm->answer_frame->data[1]<<8);
		}break;
		case YS_POSITION_REF://目标位置
		{
			pInspire_data[pComm->answer_frame->id].config_data.target_position = pComm->answer_frame->data[0]+(pComm->answer_frame->data[1]<<8);
		}break;
		case YS_CURRENT_POSITION://当前位置
		{
			pInspire_data[pComm->answer_frame->id].config_data.current_position = pComm->answer_frame->data[0]+(pComm->answer_frame->data[1]<<8);
		}break;
		case YS_SPEED_LIMIT://最大速度
		{
			pInspire_data[pComm->answer_frame->id].config_data.speed_limit = pComm->answer_frame->data[0]+(pComm->answer_frame->data[1]<<8);
		}break;
		case YS_OFFLINE_FLAG://下线检测标志
		{
			pInspire_data[pComm->answer_frame->id].config_data.offline_flag = pComm->answer_frame->data[0]+(pComm->answer_frame->data[1]<<8);
		}break;
		default:
		{
			pComm->comm_detection.decode_fault.no_match_table_index = 1;//不合理的内存表返回
		}break;
	}
	return true;
	
}

//ACC电机状态信息数据解析
bool decode_mc_ans_frame(Inspire_Comm *pComm,Inspire_Data *pInspire_data)
{
	static uint8_t parameter1,parameter2;
	uint8_t *rx_buf = (uint8_t *)pComm->ys_transmit->receive_buf;
	if(pComm->answer_frame->len != 0x11)
	{
		pComm->comm_detection.decode_fault.no_match_data_len = 1;
		return false;
	}
	parameter1 = rx_buf[5]; //滑动变阻器当前值
	parameter2 = rx_buf[6];
	UNUSED(parameter1);
	UNUSED(parameter2);
	pInspire_data[pComm->answer_frame->id].status_data.target_position = rx_buf[7]+(rx_buf[8]<<8);  //目标位置
	pInspire_data[pComm->answer_frame->id].status_data.current_position = rx_buf[9]+(rx_buf[10]<<8);//当前位置
	pInspire_data[pComm->answer_frame->id].status_data.u8VolaValue = rx_buf[11];//电压值
	pInspire_data[pComm->answer_frame->id].status_data.current = abs(rx_buf[12]+(rx_buf[13]<<8));//电流值
//	  rx_buf[14] //保留
	pInspire_data[pComm->answer_frame->id].status_data.stFault.u8FaultInfo = rx_buf[15];//错误信息
	pInspire_data[pComm->answer_frame->id].status_data.u8MaxSpeedLimit = rx_buf[16]; //最大速度限制
	pInspire_data[pComm->answer_frame->id].status_data.u16OC_Limit = rx_buf[17] + (rx_buf[18] << 8); //过流保护阈值
	// rx_buf[19-20] 电机版本号
	return true;
}
//按指令类型区分读指令
bool decode_ans_frame(Inspire_Comm *pComm)
{
	uint8_t sum;
	pComm->comm_detection.decode_fault.no_available_data = 0;
	pComm->comm_detection.decode_fault.unreasonable_length = 0;
	pComm->comm_detection.decode_fault.no_match_data_len = 0;
	pComm->comm_detection.decode_fault.no_match_head = 0;
	pComm->comm_detection.decode_fault.no_match_id = 0;
	pComm->comm_detection.decode_fault.no_match_cmd_type = 0;
	pComm->comm_detection.decode_fault.no_match_table_index = 0;
	pComm->comm_detection.decode_fault.no_match_sum_check = 0;
	if(pComm->ys_transmit->receive_flag == false)//校验接收帧
	{
		pComm->comm_detection.decode_fault.no_available_data = 1;
		return false;
	}
	if(pComm->ys_transmit->receive_len<8)//校验帧长度
	{
		pComm->comm_detection.decode_fault.unreasonable_length = 1;
		return false;
	}
	pComm->answer_frame->head =( pComm->ys_transmit->receive_buf[0]<<0x08)+(pComm->ys_transmit->receive_buf[1]);
	if(pComm->answer_frame->head != YS_ANS_FRAME_HEAD) //校验帧头
	{
		pComm->comm_detection.decode_fault.no_match_head = 1;
		return false;
	}
	pComm->answer_frame->sum_check = pComm->ys_transmit->receive_buf[pComm->ys_transmit->receive_len-1];
	sum = sum_check((void*)&pComm->ys_transmit->receive_buf[2],pComm->ys_transmit->receive_len-3);
	if(sum!=pComm->answer_frame->sum_check)//校验码验证
	{
		pComm->comm_detection.decode_fault.no_match_sum_check = 1;
		return false;
	}
	pComm->answer_frame->len = pComm->ys_transmit->receive_buf[2];		
	pComm->answer_frame->id = pComm->ys_transmit->receive_buf[3];
	pComm->answer_frame->cmd_type = (CMD_TYPE)pComm->ys_transmit->receive_buf[4];
	pComm->answer_frame->table = (MEMORY_TABLE)pComm->ys_transmit->receive_buf[5];
	if(pComm->ys_transmit->receive_len>7)
	{
		memcpy(pComm->answer_frame->data,(void*)&pComm->ys_transmit->receive_buf[6],(pComm->answer_frame->len-2));
	}
	else
	{
		pComm->comm_detection.decode_fault.unreasonable_length = 1;
		return false;
	}
	return true;
}
/*读指令,综合考虑只适合读单一内存*/
void read_table(Inspire_Comm *pComm,uint8_t id,MEMORY_TABLE table)
{
	uint8_t read_len = 0;
	YS_transmit *tran_handle = pComm->ys_transmit;
	BaseType_t ret;
	switch(table)
	{
		case YS_ID:
		case YS_SOFT_VERSION:
		case YS_BAUDRATE:
		case YS_OVER_CURRENT_TH:
		case YS_POSITION_REF:
		case YS_CURRENT_POSITION:
		case YS_SPEED_LIMIT:		
		case YS_OFFLINE_FLAG:			
		{
			read_len = 0;
		}break;
		default:
		{
			//不支持读取
		}
	}
	if(pComm->trans_block_mode == WAIT_SEM_NOP)
	{
		clear_usart2_fault();
		clear_usart3_fault();
		clear_usart4_fault();
		build_ins_frame(pComm,id,CMD_RD,table,&read_len,1);
		tran_handle->need_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(YS_Trans_SemHandle,1);//占用信号量，等待返回帧释放
		if(ret == pdTRUE)
		{
			
		}else if(ret == pdFALSE)
		{
			tran_handle->block_timeout_count++;//阻塞时间溢出
		}	

	}else if(pComm->trans_block_mode == WAIT_FLAG)
	{
		clear_usart2_fault();
		clear_usart3_fault();
		clear_usart4_fault();
		build_ins_frame(pComm,id,CMD_RD,table,&read_len,10);
		osDelay(1);
	}
}
/*写指令,综合考虑只适合写单一内存*/
void write_table(Inspire_Comm *pComm,uint8_t id,MEMORY_TABLE table,uint8_t* data)
{
	uint8_t write_len = 0;
	YS_transmit *tran_handle = pComm->ys_transmit;
	switch(table)
	{
		case YS_ID:
		case YS_BAUDRATE:
		case YS_OVER_CURRENT_TH:
		case YS_POSITION_REF:
		case YS_SPEED_LIMIT:		
		case YS_OFFLINE_FLAG:			
		{
			write_len = 2;
		}break;
		default:
		{
			//不支持读取
		}		
	}
	build_ins_frame(pComm,id,CMD_WR,table,data,write_len);
	for(int i = 0;i<tran_handle->nop_count;i++)
	{
		__nop();
	}
}

/*定位模式（反馈状态信息）*/
void position_mode_r(Inspire_Comm *pComm,uint8_t id,int16_t target_position)
{
	BaseType_t ret;
	uint8_t temp[2];
	YS_transmit *tran_handle = pComm->ys_transmit;
	temp[0] = target_position&0x00ff;
	temp[1] = (target_position&0xff00)>>8;
	if(pComm->trans_block_mode == WAIT_SEM_NOP)
	{
		//clear_usart2_fault();
		build_ins_frame(pComm,id,CMD_POSITION_R,YS_POSITION_REF,temp,2);
		tran_handle->need_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(YS_Trans_SemHandle,1);//占用信号量，等待返回帧释放
		if(ret == pdTRUE)
		{
			
		}else if(ret == pdFALSE)
		{
			tran_handle->block_timeout_count++;//阻塞时间溢出
		}	
	}else if(pComm->trans_block_mode == WAIT_FLAG)
	{
		clear_usart2_fault();
		build_ins_frame(pComm,id,CMD_POSITION_R,YS_POSITION_REF,temp,2);
		osDelay(1);
	}
}

/*定位模式（无反馈）*/
void position_mode_nr(Inspire_Comm *pComm,uint8_t id,uint16_t target_position)
{
	uint8_t temp[2];
	YS_transmit *tran_handle = pComm->ys_transmit;
	temp[0] = target_position&0x00ff;
	temp[1] = (target_position&0xff00)>>8;
	build_ins_frame(pComm,id,CMD_POSITION_NR,YS_POSITION_REF,temp,2);
	for(int i = 0;i<tran_handle->nop_count;i++)
	{
		__nop();
	}
}

/*随动模式（反馈状态信息）*/
void follow_mode_r(Inspire_Comm *pComm,uint8_t id,int16_t target_position)
{
	BaseType_t ret;
	uint8_t temp[2];
	YS_transmit *tran_handle = pComm->ys_transmit;
	temp[0] = target_position&0x00ff;
	temp[1] = (target_position&0xff00)>>8;
	if(pComm->trans_block_mode == WAIT_SEM_NOP)
	{
		clear_usart2_fault();
		build_ins_frame(pComm,id,CMD_FOLLOW_R,YS_POSITION_REF,temp,2);
		tran_handle->need_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(YS_Trans_SemHandle,3);//占用信号量，等待返回帧释放
		if(ret == pdTRUE)
		{
			
		}else if(ret == pdFALSE)
		{
			tran_handle->block_timeout_count++;//阻塞时间溢出
		}	
	}else if(pComm->trans_block_mode == WAIT_FLAG)
	{
		clear_usart2_fault();
		build_ins_frame(pComm,id,CMD_FOLLOW_R,YS_POSITION_REF,temp,2);
		osDelay(1);
	}
}
/*随动模式（无反馈）*/
void follow_mode_nr(Inspire_Comm *pComm,uint8_t id,uint16_t target_position)
{
	uint8_t temp[2];
	YS_transmit *tran_handle = pComm->ys_transmit;
	temp[0] = target_position&0x00ff;
	temp[1] = (target_position&0xff00)>>8;
	build_ins_frame(pComm,id,CMD_FOLLOW_NR,YS_POSITION_REF,temp,2);
	for(int i = 0;i<tran_handle->nop_count;i++)
	{
		__nop();
	}

}
/*广播定位模式（无反馈）*/
void broadcast_position_mode_nr(Inspire_Comm *pComm,uint8_t *id,uint16_t *target_position,uint8_t num)
{

}
/*广播随动模式（无反馈）*/
void broadcast_follow_mode_nr(Inspire_Comm *pComm,uint8_t *id,uint16_t *target_position,uint8_t num)
{

}

/*单控指令工作*/
void ys_work(Inspire_Comm *pComm,uint8_t id)
{
	BaseType_t ret;
	uint8_t temp[1];
	temp[0] = RUNNING;
	YS_transmit *tran_handle = pComm->ys_transmit;
	if(pComm->trans_block_mode == WAIT_SEM_NOP)
	{
		clear_usart2_fault();
		build_ins_frame(pComm,id,CMD_MC,YS_TABLE_HEAD,temp,1);
		tran_handle->need_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(YS_Trans_SemHandle,1);//占用信号量，等待返回帧释放
		if(ret == pdTRUE)
		{
			
		}else if(ret == pdFALSE)
		{
			tran_handle->block_timeout_count++;//阻塞时间溢出
		}	
	}else if(pComm->trans_block_mode == WAIT_FLAG)
	{
		clear_usart2_fault();
		build_ins_frame(pComm,id,CMD_MC,YS_TABLE_HEAD,temp,1);
		osDelay(1);
	}
}

/*单控指令急停*/
void ys_stop(Inspire_Comm *pComm,uint8_t id)
{
	BaseType_t ret;
	uint8_t temp[1];
	temp[0] = STOP;
	YS_transmit *tran_handle = pComm->ys_transmit;
	if(pComm->trans_block_mode == WAIT_SEM_NOP)
	{
		clear_usart2_fault();
		build_ins_frame(pComm,id,CMD_MC,YS_TABLE_HEAD,temp,1);
		tran_handle->need_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(YS_Trans_SemHandle,1);//占用信号量，等待返回帧释放
		if(ret == pdTRUE)
		{
			
		}else if(ret == pdFALSE)
		{
			tran_handle->block_timeout_count++;//阻塞时间溢出
		}	
	}else if(pComm->trans_block_mode == WAIT_FLAG)
	{
		clear_usart2_fault();
		build_ins_frame(pComm,id,CMD_MC,YS_TABLE_HEAD,temp,1);
		osDelay(1);
	}
}

/*单控指令暂停*/
void ys_pause(Inspire_Comm *pComm,uint8_t id)
{
	BaseType_t ret;
	uint8_t temp[1];
	temp[0] = PAUSE;
	YS_transmit *tran_handle = pComm->ys_transmit;
	if(pComm->trans_block_mode == WAIT_SEM_NOP)
	{
		clear_usart2_fault();
		build_ins_frame(pComm,id,CMD_MC,YS_TABLE_HEAD,temp,1);
		tran_handle->need_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(YS_Trans_SemHandle,1);//占用信号量，等待返回帧释放
		if(ret == pdTRUE)
		{
			
		}else if(ret == pdFALSE)
		{
			tran_handle->block_timeout_count++;//阻塞时间溢出
		}	
	}else if(pComm->trans_block_mode == WAIT_FLAG)
	{
		clear_usart2_fault();
		build_ins_frame(pComm,id,CMD_MC,YS_TABLE_HEAD,temp,1);
		osDelay(1);
	}
}

/*单控指令参数装订*/
void ys_save(Inspire_Comm *pComm,uint8_t id)
{
	BaseType_t ret;
	uint8_t temp[1];
	temp[0] = SAVE;
	YS_transmit *tran_handle = pComm->ys_transmit;
	if(pComm->trans_block_mode == WAIT_SEM_NOP)
	{
		clear_usart2_fault();
		build_ins_frame(pComm,id,CMD_MC,YS_TABLE_HEAD,temp,1);
		tran_handle->need_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(YS_Trans_SemHandle,1);//占用信号量，等待返回帧释放
		if(ret == pdTRUE)
		{
			
		}else if(ret == pdFALSE)
		{
			tran_handle->block_timeout_count++;//阻塞时间溢出
		}	
	}else if(pComm->trans_block_mode == WAIT_FLAG)
	{
		clear_usart2_fault();
		build_ins_frame(pComm,id,CMD_MC,YS_TABLE_HEAD,temp,1);
		osDelay(1);
	}
}

/*单控指令查询状态信息*/
void ys_query_status(Inspire_Comm *pComm,uint8_t id)
{
	BaseType_t ret;
	uint8_t temp[1];
	temp[0] = QUERY_STATUS;
	YS_transmit *tran_handle = pComm->ys_transmit;
	if(pComm->trans_block_mode == WAIT_SEM_NOP)
	{
		clear_usart2_fault();
		clear_usart3_fault();
		clear_usart4_fault();
		build_ins_frame(pComm,id,CMD_MC,YS_TABLE_HEAD,temp,1);
		tran_handle->need_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(YS_Trans_SemHandle,1);//占用信号量，等待返回帧释放
		if(ret == pdTRUE)
		{
			
		}else if(ret == pdFALSE)
		{
			tran_handle->block_timeout_count++;//阻塞时间溢出
		}	
	}else if(pComm->trans_block_mode == WAIT_FLAG)
	{
		clear_usart2_fault();
		clear_usart3_fault();
		clear_usart4_fault();
		build_ins_frame(pComm,id,CMD_MC,YS_TABLE_HEAD,temp,1);
		osDelay(1);
	}
	
}

/*单控指令故障清除*/
void ys_clear_fault(Inspire_Comm *pComm,uint8_t id)
{
	BaseType_t ret;
	uint8_t temp[1];
	temp[0] = CLEAR_FAULT;
	YS_transmit *tran_handle = pComm->ys_transmit;
	if(pComm->trans_block_mode == WAIT_SEM_NOP)
	{
		clear_usart2_fault();
		build_ins_frame(pComm,id,CMD_MC,YS_TABLE_HEAD,temp,1);
			tran_handle->need_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(YS_Trans_SemHandle,1);//占用信号量，等待返回帧释放
		if(ret == pdTRUE)
		{
			
		}else if(ret == pdFALSE)
		{
			tran_handle->block_timeout_count++;//阻塞时间溢出
		}	
	}else if(pComm->trans_block_mode == WAIT_FLAG)
	{
		clear_usart2_fault();
		build_ins_frame(pComm,id,CMD_MC,YS_TABLE_HEAD,temp,1);
		osDelay(1);
	}
}

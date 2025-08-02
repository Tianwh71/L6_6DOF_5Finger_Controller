#include "TJ_MotorDrive.h"
#include "Common.h"
#include "data_structure.h"
#include "my_math.h"
#include "semphr.h"

tjData tjMotorParsedData[TJ_MOTOR_NUM_MAX] = {0}; // 存放电机解析后的数据
tjTransmit_t stTjMotorTransmit = {0};							// 发送接收相关的结构
tjAnswerFrame_t stTjMotorAnswerDataTmp = {0};			// 存放电机数据的中间中间结构体
tjComm_t stTjCommAllInfo = {
		.pstTjTransmit = &stTjMotorTransmit,
		.pstTjAnswer_frame = &stTjMotorAnswerDataTmp,

}; // 关于电机控制、数据存储的总结构体。

tjAnswerFrame_t uart2_rx_buf = {0};
tjAnswerFrame_t uart3_rx_buf = {0};
tjAnswerFrame_t uart4_rx_buf = {0};



HandTargetPosition_t tjTarPosition = {
		.thumb_target_position = 1000,
		.index_target_position = 2000,
		.middle_target_position = 2000,
		.ring_target_position = 2000,
		.little_target_position = 2000,
		.thumb_aux_target_position = 1000,
};

// 计算电机的校验和
uint8_t TJ_CalChecksum(const uint8_t *pData, uint16_t DataLen)
{
	uint8_t l_u8SumVal = 0;
	for (uint8_t i = 0; i < DataLen; i++)
	{
		l_u8SumVal += pData[i];
	}
	return l_u8SumVal;
}

/**
  * @brief  处理UART IDLE中断接收数据
  * @param  huart: UART句柄指针
  * @param  tran_handle: 传输控制结构体
  * @param  Size: 接收到的数据长度
  * @retval 是否成功处理
  */
bool HAL_UART_Receive_IDLE(UART_HandleTypeDef *huart, tjAnswerFrame_t *tran_handle, uint16_t Size)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    bool ret = false;
    // 1. 获取DMA接收的实际数据长度和位置
		uint16_t dma_remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx); // 获取DMA剩余未传输的字节数
		uint16_t calculated_size = TJ_DATA_RECV_LEN_MAX - dma_remaining; // 计算已接收的字节数
    uint16_t actual_size = Size;
    
    // 智能选择最可靠的长度值（经验公式）
    uint16_t reliable_size = (calculated_size > 0 && calculated_size <= TJ_DATA_RECV_LEN_MAX) ? 
                           calculated_size : actual_size;

    __HAL_UART_CLEAR_IDLEFLAG(huart);
    uint8_t *rx_buf = tran_handle->u8receive_data;

    // 2. 帧头检测（仅当未开始拼帧时）
    if(!tran_handle->assembling) {
        // 搜索范围限制为可靠数据长度
        for(uint16_t i = 0; i <= reliable_size - 2; i++) {
            uint16_t header = (tran_handle->u8receive_data[i] << 8) | 
                             tran_handle->u8receive_data[i+1];
            
            if(header == YS_ANS_FRAME_HEAD) {
                tran_handle->assembling = true;
                tran_handle->u16Head = header;
                tran_handle->u8Len = 0;
                
                // 从找到帧头的位置开始处理
                uint16_t copy_len = min(FRAME_SIZE, reliable_size - i);
							  reliable_size = reliable_size - i;
							  tran_handle->frame_start_pos = i;     //记录帧头在缓冲区中的位置
                tran_handle->u8Len = 0;
                break;
            }
        }
    }

    // 3. 安全拼帧处理
    if(tran_handle->assembling) {
        // 计算需要追加的数据量
				uint16_t data_start = tran_handle->frame_start_pos ;
				uint16_t available = reliable_size - tran_handle->frame_start_pos;
        uint16_t remaining = FRAME_SIZE - tran_handle->u8Len;
        uint16_t copy_len = min(remaining, reliable_size);

        if(copy_len > 0) {
            memcpy(&tran_handle->receive_buf[tran_handle->u8Len],
                  &tran_handle->u8receive_data[data_start],
                  copy_len);
            
            tran_handle->u8Len += copy_len;
            tran_handle->last_receive_time = HAL_GetTick();
        }
				tran_handle->frame_start_pos = 0;

        
        // 检查是否收到完整帧
        if(tran_handle->u8Len >= FRAME_SIZE) {
            // 校验和验证（校验范围为第2字节到第20字节，校验和在第21字节）
            uint8_t checksum = TJ_CalChecksum(&tran_handle->receive_buf[2], FRAME_SIZE-3);
            if(checksum == tran_handle->receive_buf[FRAME_SIZE-1]) {
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
            }
            
        }
    }
    
    // 检查拼帧超时
    if(tran_handle->assembling && 
       (HAL_GetTick() - tran_handle->last_receive_time > FRAME_TIMEOUT_MS)) {
        memset(tran_handle, 0, sizeof(tjAnswerFrame_t));
    }
		//memset(tran_handle->u8receive_data, 0, TJ_DATA_RECV_LEN_MAX);
    if(xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    
    return ret;
}




// 电机命令组帧函数
uint8_t TJ_BuildCmd(tjComm_t *pTjComm, uint8_t Mid, uint8_t CmdType, uint8_t MemIndex, const uint8_t *pData, uint8_t DataLen)
{
	uint8_t *l_pDataSend = (uint8_t *)pTjComm->pstTjTransmit->send_buf;
	l_pDataSend[0] = 0x55;
	l_pDataSend[1] = 0xAA;
	l_pDataSend[2] = DataLen + 2;
	l_pDataSend[3] = Mid;

	l_pDataSend[4] = CmdType;
	l_pDataSend[5] = MemIndex;

	memcpy(&l_pDataSend[6], pData, DataLen);

	l_pDataSend[6 + DataLen] = TJ_CalChecksum(&l_pDataSend[2], DataLen + 4);

	pTjComm->pstTjTransmit->send_len = 7 + DataLen;
	// id  0\1
	if (Mid == 1 || Mid == 2)
	{
		HAL_UART_Transmit_DMA(&huart2, (void *)l_pDataSend, pTjComm->pstTjTransmit->send_len);
	}
	if (Mid == 3 || Mid == 4)
	{
		HAL_UART_Transmit_DMA(&huart3, (void *)l_pDataSend, pTjComm->pstTjTransmit->send_len);
	}
	if (Mid == 5 || Mid == 6)
	{
		HAL_UART_Transmit_DMA(&huart4, (void *)l_pDataSend, pTjComm->pstTjTransmit->send_len);
	}
}


bool ft_wait_ack_delay(tjComm_t *pTjComm,uint32_t time_ref)
{

	if (pTjComm->Tjtrans_block_mode == WAIT_FLAG)
	{
		pTjComm->pstTjTransmit->block_timeout_count = 0;
		do{ 
				osDelay(1);
				pTjComm->pstTjTransmit->block_timeout_count++;
				if((pTjComm->pstTjTransmit->block_timeout_count >= time_ref) || (pTjComm->pstTjTransmit->block_timeout_count > pTjComm->pstTjTransmit->block_timeout_count))
						pTjComm->pstTjTransmit->usart_rx_fault++;
						return false;
			}
			while(pTjComm->pstTjTransmit->bNeed_block == true);//正在等待数据时
		}
   else if (pTjComm->Tjtrans_block_mode == WAIT_SEM_NOP)
	 {
		//xSemaphoreTake(FT_ACK_SEMHandle,(TickType_t)time_ref);	
		for(int i =0;i<10;i++)
		{
			__NOP();
		}
	}
		//执行到这可能刚接收到数据，接下来可能有发射动作，需要加一点延时
//		for(int i =0;i<15;i++)
//		{
//			__NOP();
//		}
		return true;
}
// TJ读取内存表信息解析
bool decode_rd_ans_frame(tjComm_t *pTjComm, tjData *pstTjData)
{
	tjAnswerFrame_t *l_pstSrcFrame = pTjComm->pstTjAnswer_frame;
	// tjTransmit_t *l_pstDesData = pTjComm->pstTjTransmit;

	if (pTjComm->pstTjAnswer_frame->stCmdType != TJ_CMD_RD)
	{
		pTjComm->stTjCommDetection.decode_fault.no_match_cmd_type = 1;
		return false;
	}
	if (pTjComm->pstTjAnswer_frame->u8Len > (2 + 2)) // 内存表数据分布十分分散，不适合连续读取，所以只解析读取字节和半字等情况
	{
		pTjComm->stTjCommDetection.decode_fault.no_match_data_len = 1;
		return false;
	}
	switch (l_pstSrcFrame->stTableIndex)
	{
	case TJ_TORQUE_CMD_TYPE: // 电机力矩模式类型
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.u16TorqueCmdType = l_pstSrcFrame->receive_buf[0];
	}
	break;
	case TJ_POLE_PAIRS: // 电机极对数
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.u16PolePairs = l_pstSrcFrame->receive_buf[0] + (l_pstSrcFrame->receive_buf[1] << 8);
	}
	break;
	case TJ_MOTOR_ID: // 电机ID
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.u8MotorID = l_pstSrcFrame->receive_buf[0];
	}
	break;
	case TJ_UART_BAUD: // 波特率
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.u8UartBaud = l_pstSrcFrame->receive_buf[0];
	}
	break;
	case TJ_OTP_THRESHOLD: // 过温保护
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.fOtpThreshold = Cmn_bytes_to_float(l_pstSrcFrame->receive_buf, ENDIAN_LITTLE);
	}
	break;
	case TJ_OTP_RECOVERY: // 过温启动设置
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.fOtpRecovery = Cmn_bytes_to_float(l_pstSrcFrame->receive_buf, ENDIAN_LITTLE);
	}
	break;
	case TJ_TARGET_POSITION: // 目标位置设置
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.u16TargetPosition = l_pstSrcFrame->receive_buf[0] + (l_pstSrcFrame->receive_buf[1] << 8);
	}
	break;
	case TJ_FORCE_SENSOR_CAL: // 力传感器位置校准设置
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.u16ForceSensorCal = l_pstSrcFrame->receive_buf[0] + (l_pstSrcFrame->receive_buf[1] << 8);
	}
	break;
	case TJ_OCP_THRESHOLD: // 过流保护电流
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.fOcpThreshold = Cmn_bytes_to_float(l_pstSrcFrame->receive_buf, ENDIAN_LITTLE);
	}
	break;
	case TJ_RESERVED_22: // 预留
	{
		//  pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.offline_flag = l_pstSrcFrame->au8data[0]+(l_pstSrcFrame->au8data[1]<<8);
	}
	break;
	case TJ_MCU_OTP: // MCU过温保护
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.fMcuOtp = Cmn_bytes_to_float(l_pstSrcFrame->receive_buf, ENDIAN_LITTLE);
	}
	break;
	case TJ_POS_KP: // 位置环Kp
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.fPosKp = Cmn_bytes_to_float(l_pstSrcFrame->receive_buf, ENDIAN_LITTLE);
	}
	break;
	case TJ_POS_KD: // 位置环Kd
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.fPosKd = Cmn_bytes_to_float(l_pstSrcFrame->receive_buf, ENDIAN_LITTLE);
	}
	break;
	case TJ_TORQUE_CONSTANT: // 力矩系数
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.fTorqueConstant = Cmn_bytes_to_float(l_pstSrcFrame->receive_buf, ENDIAN_LITTLE);
	}
	break;
	case TJ_ENC_OFFSET: // 初始角度
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.fEncoderOffset = Cmn_bytes_to_float(l_pstSrcFrame->receive_buf, ENDIAN_LITTLE);
	}
	break;
	case TJ_RUN_MODE: // 运行模式
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.u16RunMode = l_pstSrcFrame->receive_buf[0] + (l_pstSrcFrame->receive_buf[1] << 8);
	}
	break;
	case TJ_CTRL_STATE: // 控制状态
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.u16CtrlState = l_pstSrcFrame->receive_buf[0] + (l_pstSrcFrame->receive_buf[1] << 8);
	}
	break;
	case TJ_FAULT_STATUS: // 故障字节
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.u32FaultStatus = Cmn_bytes_to_uint(l_pstSrcFrame->receive_buf, ENDIAN_LITTLE);
	}
	break;
	case TJ_CSP_KP: // loc_kp
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.u32CspKp = Cmn_bytes_to_uint(l_pstSrcFrame->receive_buf, ENDIAN_LITTLE);
	}
	break;
	case TJ_VEL_KP: // 速度环 Kp
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.u32SpdKp = Cmn_bytes_to_uint(l_pstSrcFrame->receive_buf, ENDIAN_LITTLE);
	}
	break;
	case TJ_VEL_KI: // 速度环Ki
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.fSpdKi = Cmn_bytes_to_float(l_pstSrcFrame->receive_buf, ENDIAN_LITTLE);
	}
	break;
	case TJ_MAX_VELOCITY: // 最大速度
	{
		pstTjData[l_pstSrcFrame->u8ID].stTjConfig_data.u32MaxVelocity = Cmn_bytes_to_uint(l_pstSrcFrame->receive_buf, ENDIAN_LITTLE);
	}
	break;
	default:
	{
		pTjComm->stTjCommDetection.decode_fault.no_match_table_index = 1; // 不合理的内存表返回
	}
	break;
	}
	return true;
}

// TJ电机状态信息数据解析
bool decode_mc_ans_frame(tjAnswerFrame_t *tj_Answer, tjData *pTjData)
{
	uint8_t sum;
	static uint8_t parameter1, parameter2;
	if(tj_Answer == NULL || pTjData == NULL || tj_Answer->u8Len == 0) 
	{
        return false;
  }
	if(tj_Answer->u8Len > 22 ) {
        pTjData->decode_state = LENGTH_ERROR;
        return false;
   }
	uint8_t *rx_buf = (uint8_t *)tj_Answer->receive_buf;
	
	tj_Answer->u16Head = (rx_buf[0]<<0x08) +rx_buf[1];
	if(tj_Answer->u16Head != YS_ANS_FRAME_HEAD) //校验帧头
	{
		pTjData->decode_state = HEAD_NO_MATCH;
		return false;
	}
	tj_Answer->u8SumCheck = rx_buf[tj_Answer->u8Len-1];
	sum = TJ_CalChecksum((void*)&rx_buf[2],tj_Answer->u8Len-3);
	if(sum != tj_Answer->u8SumCheck)//校验码验证
	{
		return false;
	}
	
	parameter1 = rx_buf[5]; // 滑动变阻器当前值
	parameter2 = rx_buf[6];
	UNUSED(parameter1);
	UNUSED(parameter2);
	
	pTjData->stTjConfig_data.u8MotorID = rx_buf[3];
	pTjData->stTjStatus_data.u16Target_position= rx_buf[7] + (rx_buf[8] << 8);	 // 目标位置
  pTjData->stTjStatus_data.u16Current_position = rx_buf[9] + (rx_buf[10] << 8); // 当前位置
	pTjData->stTjStatus_data.u8Tempture = rx_buf[11];															//电机温度
	pTjData->stTjStatus_data.u16Current = rx_buf[12] + (rx_buf[13] << 8); // 电流值
	pTjData->stTjStatus_data.u16SpeedRpm = rx_buf[14] + (rx_buf[16] << 8);
	pTjData->stTjStatus_data.stFault.u8FaultInfo = rx_buf[15]; // 错误信息
	
	pTjData->decode_state = PARSING_SUCCESS;

	return true;
}

// 按指令类型区分读指令
bool decode_ans_frame(tjComm_t *pTjComm)
{
	uint8_t l_u8Sum;

	if (pTjComm->pstTjTransmit->bReceive_flag == false) // 校验接收帧
	{
		pTjComm->stTjCommDetection.decode_fault.no_available_data = 1;
		return false;
	}
	if (pTjComm->pstTjTransmit->receive_len < 8) // 校验帧长度
	{
		pTjComm->stTjCommDetection.decode_fault.unreasonable_length = 1;
		return false;
	}
	if (pTjComm->pstTjTransmit->receive_buf[0] == 0xAA && pTjComm->pstTjTransmit->receive_buf[1] == 0x55) // 校验帧头
	{
		pTjComm->stTjCommDetection.decode_fault.no_match_head = 1;
		return false;
	}

	pTjComm->pstTjAnswer_frame->u8SumCheck = pTjComm->pstTjTransmit->receive_buf[pTjComm->pstTjTransmit->receive_len - 1];
	l_u8Sum = TJ_CalChecksum((void *)&pTjComm->pstTjTransmit->receive_buf[2], pTjComm->pstTjTransmit->receive_len - 3);
	if (l_u8Sum != pTjComm->pstTjAnswer_frame->u8SumCheck) // 校验码验证
	{
		pTjComm->stTjCommDetection.decode_fault.no_match_sum_check = 1;
		return false;
	}
	pTjComm->pstTjAnswer_frame->u8Len = pTjComm->pstTjTransmit->receive_buf[2];
	pTjComm->pstTjAnswer_frame->u8ID = pTjComm->pstTjTransmit->receive_buf[3];
	pTjComm->pstTjAnswer_frame->stCmdType = (tjCmdTy_t)pTjComm->pstTjTransmit->receive_buf[4];

	if (pTjComm->pstTjAnswer_frame->stCmdType == TJ_CMD_RD || pTjComm->pstTjAnswer_frame->stCmdType == TJ_CMD_WR)
	{
		pTjComm->pstTjAnswer_frame->stTableIndex = (tjMemoryTable_t)pTjComm->pstTjTransmit->receive_buf[5];
		if (pTjComm->pstTjTransmit->receive_len > 7)
		{
			memcpy(pTjComm->pstTjAnswer_frame->receive_buf, (void *)&pTjComm->pstTjTransmit->receive_buf[6], (pTjComm->pstTjAnswer_frame->u8Len - 2));
		}
		else
		{
			pTjComm->stTjCommDetection.decode_fault.unreasonable_length = 1;
			return false;
		}
	}

	return true;
}
/*读指令,综合考虑只适合读单一内存*/
void read_table(tjComm_t *pTjComm, uint8_t id, tjColTable_index MemtableIndex)
{
	uint8_t l_au8ReadByteNum[2] = {0x00, 0x00};
	tjTransmit_t *tran_handle = pTjComm->pstTjTransmit;
	BaseType_t ret;
	switch (MemtableIndex)
	{
	case TJ_TARGET_KP:
	case TJ_TARGET_KD:
	case TJ_LIMIT_I:
	case TJ_TORQUE_ACK:
	case TJ_SPEED_ACK:
	{
		
	}
	default:
	{
		// 不支持读取
	}
	}
	if (pTjComm->Tjtrans_block_mode == WAIT_SEM_NOP)
	{
		//定位控制只有0x37可以控制运动，其他都可以读取到位置速度电流
		TJ_BuildCmd(pTjComm, id, TJ_CMD_POSITION_R, MemtableIndex, l_au8ReadByteNum, 2);
		tran_handle->bNeed_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(TJ_Trans_SemHandle,1);//占用信号量，等待返回帧释放
		if (ret == pdTRUE)
		{
		}
		else if (ret == pdFALSE)
		{
			tran_handle->block_timeout_count++; // 阻塞时间溢出
		}
	}
}
/*写指令,综合考虑只适合写单一内存*/
void write_table(tjComm_t *pTjComm, uint8_t id, tjMemoryTable_t MemTableIndex, uint8_t *data)
{
	uint8_t l_u8write_len = 0;
	tjTransmit_t *tran_handle = pTjComm->pstTjTransmit;
	switch (MemTableIndex)
	{
	case TJ_MOTOR_ID:
	case TJ_UART_BAUD:
	{
		l_u8write_len = 2;
	}
	break;
	case TJ_RUN_MODE:
	case TJ_CTRL_STATE:
	case TJ_TORQUE_CMD_TYPE:
	case TJ_POLE_PAIRS:
	{
		l_u8write_len = 2;
	}
	break;

	case TJ_OTP_RECOVERY:
	case TJ_OTP_THRESHOLD:
	case TJ_TARGET_POSITION:
	case TJ_FORCE_SENSOR_CAL:
	case TJ_OCP_THRESHOLD:
	case TJ_QDEC_TX_MODE:
	case TJ_MCU_OTP:
	case TJ_POS_KP:
	case TJ_POS_KD:
	case TJ_TORQUE_CONSTANT:
	case TJ_ENC_OFFSET:
	case TJ_FAULT_STATUS:
	case TJ_CSP_KP:
	case TJ_VEL_KP:
	case TJ_VEL_KI:
	case TJ_MAX_VELOCITY:
	{
		l_u8write_len = 4;
	}
	break;
	case TJ_RESERVED_22:
	{
	}
	break;
	}
	TJ_BuildCmd(pTjComm, id, TJ_CMD_WR, MemTableIndex, data, l_u8write_len);
}

/*定位模式（反馈状态信息）*/
void position_mode_r(tjComm_t *pTjComm, uint8_t id, uint16_t target_position)
{
	BaseType_t ret;
	uint8_t temp[2];
	tjTransmit_t *tran_handle = pTjComm->pstTjTransmit;
	temp[0] = target_position & 0x00ff;
	temp[1] = (target_position & 0xff00) >> 8;
	if (pTjComm->Tjtrans_block_mode == WAIT_SEM_NOP)
	{
		TJ_BuildCmd(pTjComm, id, TJ_CMD_POSITION_R, TJ_TARGET_POSITION, temp, 2);
		tran_handle->bNeed_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(TJ_Trans_SemHandle,1);//占用信号量，等待返回帧释放
		if(ret == pdTRUE)
		{

		}else if(ret == pdFALSE)
		{
			tran_handle->block_timeout_count++;//阻塞时间溢出
		}
	}
}

/*广播定位模式（无反馈）*/
void broadcast_position_mode_nr(tjComm_t *pTjComm, uint8_t *id, uint16_t *target_position, uint8_t num)
{
}
/*广播随动模式（无反馈）*/
void broadcast_follow_mode_nr(tjComm_t *pTjComm, uint8_t *id, uint16_t *target_position, uint8_t num)
{
}

/*单控指令工作*/
void ys_work(tjComm_t *pTjComm, uint8_t id)
{
	BaseType_t ret;
	uint8_t l_au8Data[2] = "0";
	l_au8Data[1] = TJ_SIG_RUNNING;
	tjTransmit_t *tran_handle = pTjComm->pstTjTransmit;
	if (pTjComm->Tjtrans_block_mode == WAIT_SEM_NOP)
	{
		TJ_BuildCmd(pTjComm, id, TJ_CMD_MC, 0, l_au8Data, 2);
		tran_handle->bNeed_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(TJ_Trans_SemHandle,1);//占用信号量，等待返回帧释放
		if (ret == pdTRUE)
		{
		}
		else if (ret == pdFALSE)
		{
			tran_handle->block_timeout_count++; // 阻塞时间溢出
		}
	}
	//	else if(pTjComm->trans_block_mode == WAIT_FLAG)
	//	{
	//		clear_usart2_fault();
	//		TJ_BuildCmd(pTjComm,id,CMD_MC,TJ_TABLE_HEAD,temp,1);
	//		osDelay(1);
	//	}
}

/*单控指令急停*/
void ys_stop(tjComm_t *pTjComm, uint8_t id)
{
	BaseType_t ret;
	uint8_t l_au8Data[2] = "0";
	l_au8Data[1] = TJ_SIG_STOP;
	tjTransmit_t *tran_handle = pTjComm->pstTjTransmit;

	if (pTjComm->Tjtrans_block_mode == WAIT_SEM_NOP)
	{
		TJ_BuildCmd(pTjComm, id, TJ_CMD_MC, 0, l_au8Data, 2);
		tran_handle->bNeed_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(TJ_Trans_SemHandle,1);//占用信号量，等待返回帧释放
		if (ret == pdTRUE)
		{
		}
		else if (ret == pdFALSE)
		{
			tran_handle->block_timeout_count++; // 阻塞时间溢出
		}
	}
}

/*单控指令暂停*/
void ys_pause(tjComm_t *pTjComm, uint8_t id)
{
	BaseType_t ret;
	uint8_t l_au8Data[2] = "0";
	l_au8Data[1] = TJ_SIG_PAUSE;
	tjTransmit_t *tran_handle = pTjComm->pstTjTransmit;

	if (pTjComm->Tjtrans_block_mode == WAIT_SEM_NOP)
	{
		
		TJ_BuildCmd(pTjComm, id, TJ_CMD_MC, 0, l_au8Data, 2);
		tran_handle->bNeed_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(TJ_Trans_SemHandle,1);//占用信号量，等待返回帧释放
		if (ret == pdTRUE)
		{
		}
		else if (ret == pdFALSE)
		{
			tran_handle->block_timeout_count++; // 阻塞时间溢出
		}
	}
}

/*单控指令参数装订*/
void ys_save(tjComm_t *pTjComm, uint8_t id)
{
	BaseType_t ret;
	uint8_t l_au8Data[2] = "0";
	l_au8Data[1] = TJ_SIG_SAVE;
	tjTransmit_t *tran_handle = pTjComm->pstTjTransmit;

	if (pTjComm->Tjtrans_block_mode == WAIT_SEM_NOP)
	{
		TJ_BuildCmd(pTjComm, id, TJ_CMD_MC, 0, l_au8Data, 2);
		tran_handle->bNeed_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(TJ_Trans_SemHandle,1);//占用信号量，等待返回帧释放
		if (ret == pdTRUE)
		{
		}
		else if (ret == pdFALSE)
		{
			tran_handle->block_timeout_count++; // 阻塞时间溢出
		}
	}
}

/*单控指令查询状态信息*/
void TJ_query_status(tjComm_t *pTjComm, uint8_t id)
{
	BaseType_t ret;
	uint8_t l_au8Data[2] = "0";
	l_au8Data[1] = TJ_SIG_QUERY_STATUS;
	tjTransmit_t *tran_handle = pTjComm->pstTjTransmit;

	if (pTjComm->Tjtrans_block_mode == WAIT_SEM_NOP)
	{
		TJ_BuildCmd(pTjComm, id, TJ_CMD_MC, 0, l_au8Data, 2);
		tran_handle->bNeed_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(TJ_Trans_SemHandle,1);//占用信号量，等待返回帧释放
		if (ret == pdTRUE)
		{
		}
		else if (ret == pdFALSE)
		{
			tran_handle->block_timeout_count++; // 阻塞时间溢出
		}
	}
}

/*单控指令故障清除*/
void TJ_clear_fault(tjComm_t *pTjComm, uint8_t id)
{
	BaseType_t ret;
	uint8_t l_au8Data[2] = "0";
	l_au8Data[1] = TJ_SIG_CLEAR_FAULT;
	tjTransmit_t *tran_handle = pTjComm->pstTjTransmit;

	if (pTjComm->Tjtrans_block_mode == WAIT_SEM_NOP)
	{
		TJ_BuildCmd(pTjComm, id, TJ_CMD_MC, 0, l_au8Data, 2);
		tran_handle->bNeed_block = true;
		tran_handle->block_count++;
		ret = xSemaphoreTake(TJ_Trans_SemHandle,1);//占用信号量，等待返回帧释放
		if (ret == pdTRUE)
		{
		}
		else if (ret == pdFALSE)
		{
			tran_handle->block_timeout_count++; // 阻塞时间溢出
		}
	}
}

// void Acc_cmd_test(void)
//{
//	 static char flag_i = 2;
//	 static int32_t motorCount = 0;
//    switch(flag_i)
//	 {
//		 case 0:
//		 {
//		   position_mode_r(&tjComm_t,0x05,1600);
//			 osDelay(10);
//			 position_mode_r(&tjComm_t,0x06,0);
//			 osDelay(10);
//			 position_mode_r(&tjComm_t,0x07,1600);
//			 osDelay(1500);
//			 flag_i = 1;
//			 motorCount++;
//			 break;
//		 }
//		 case 1:
//		 {
//			 position_mode_r(&tjComm_t,0x05,0);
//			 osDelay(10);
//        position_mode_r(&tjComm_t,0x07,0);
//        osDelay(10);
//			 position_mode_r(&tjComm_t,0x06,1000);
//			 osDelay(1500);
//			 flag_i = 0;
//			 motorCount++;
//		   break;
//		 }
//		  case 2:
//		 {
//			 position_mode_r(&tjComm_t,0x05,1600);
//			 osDelay(10);
//			 position_mode_r(&tjComm_t,0x07,1600);
//			 osDelay(1000);
//			 flag_i = 3;
//			 motorCount++;
//			 break;
//		 }
//		 case 3:
//		 {
//			 position_mode_r(&tjComm_t,0x05,1250);
//			 osDelay(300);
//        position_mode_r(&tjComm_t,0x05,1600);
//        osDelay(20);
//			 position_mode_r(&tjComm_t,0x07,1250);
//        osDelay(300);
//			 position_mode_r(&tjComm_t,0x07,1600);
//			 osDelay(20);
//			 flag_i = 3;
//			 motorCount++;
//		   break;
//		 }
//		 default:
//	   {
//		   break;
//		 }
//	 }
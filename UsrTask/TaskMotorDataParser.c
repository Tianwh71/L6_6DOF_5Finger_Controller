#include "TaskMotorDataParser.h"
#include "Common.h"
#include "fdcan.h"
#include "data_structure.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "AAC_drv.h"
#include "AAC_MotorControl_fml.h"

static EventBits_t xEvent;

void MotorDataParserTaskFun(void *argument)
{
  /* USER CODE BEGIN MotorDataParserTaskFun */
  /* Infinite loop */
	static int32_t AccMotorRecvCount = 0;
  for (;;)
  {
			xEvent = xEventGroupWaitBits(UART_ReceivedEventHandle,  RX_UART2_EVENT | RX_UART3_EVENT | RX_UART4_EVENT, pdTRUE, pdFALSE, (TickType_t)portMAX_DELAY);

			/* USART2 */
			if((xEvent & RX_UART2_EVENT) != 0)
			{
				if(decode_ans_frame(&inspire_comm[0]) == true)
				{
					
						AccMotorRecvCount++;
						inspire_motor_rx_decode(&rs_actuator,&inspire_comm[0],inspire_data);
				}
				ys_transmit_2.receive_flag = false;
				memset((void *)ys_transmit_2.receive_buf,0,YS_TRANSMIT_LEN_MAXIMUM);
				if(ys_transmit_2.need_block == true)
				{
					ys_transmit_2.block_count --;
					ys_transmit_2.need_block = false;
					xSemaphoreGive(YS_Trans_SemHandle);
				}
				
			}
			
			/* USART3 */
			if((xEvent & RX_UART3_EVENT) != 0)
			{
				if(decode_ans_frame(&inspire_comm[1]) == true)
				{
					
						AccMotorRecvCount++;
						inspire_motor_rx_decode(&rs_actuator,&inspire_comm[1],inspire_data);
				}
				ys_transmit_2.receive_flag = false;
				memset((void *)ys_transmit_2.receive_buf,0,YS_TRANSMIT_LEN_MAXIMUM);
				if(ys_transmit_2.need_block == true)
				{
					ys_transmit_2.block_count --;
					ys_transmit_2.need_block = false;
					xSemaphoreGive(YS_Trans_SemHandle);
				}
				
			}
			
			/* UART4 */
			if((xEvent & RX_UART4_EVENT) != 0)
			{
				if(decode_ans_frame(&inspire_comm[2]) == true)
				{
						AccMotorRecvCount++;
						inspire_motor_rx_decode(&rs_actuator,&inspire_comm[2],inspire_data);
				}
				ys_transmit_2.receive_flag = false;
				memset((void *)ys_transmit_2.receive_buf,0,YS_TRANSMIT_LEN_MAXIMUM);
				if(ys_transmit_2.need_block == true)
				{
					ys_transmit_2.block_count --;
					ys_transmit_2.need_block = false;
					xSemaphoreGive(YS_Trans_SemHandle);
				}
			}
			
    }
  /* USER CODE END MotorDataParserTaskFun */
}

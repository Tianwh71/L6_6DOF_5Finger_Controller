#include "TaskMotorDataParser.h"
#include "TJ_MotorDrive.h"
#include "Common.h"
#include "fdcan.h"
#include "data_structure.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "TJ_Motor_Control_fml.h"
#include "event_groups.h"


static EventBits_t xEvent;

void MotorDataParserTaskFun(void *argument)
{
  /* USER CODE BEGIN MotorDataParserTaskFun */
  /* Infinite loop */
  for (;;)
  {
			xEvent = xEventGroupWaitBits(UART_ReceivedEventHandle,  RX_UART2_EVENT | RX_UART3_EVENT | RX_UART4_EVENT, pdTRUE, pdFALSE, (TickType_t)portMAX_DELAY);

			/* USART2 */
			if((xEvent & RX_UART2_EVENT) != 0)
			{
				decode_mc_ans_frame(&uart2_rx_buf, tjMotorParsedData);
				rx_data_2tj_servo(tjMotorParsedData, tj_servo);
				uart2_rx_buf.u8Len = 0;
				
			}
			
			/* USART3 */
			if((xEvent & RX_UART3_EVENT) != 0)
			{
				decode_mc_ans_frame(&uart3_rx_buf, tjMotorParsedData);
				rx_data_2tj_servo(tjMotorParsedData, tj_servo);
				uart3_rx_buf.u8Len = 0;
				
			}
			
			/* UART4 */
			if((xEvent & RX_UART4_EVENT) != 0)
			{
				decode_mc_ans_frame(&uart4_rx_buf, tjMotorParsedData);
				rx_data_2tj_servo(tjMotorParsedData, tj_servo);
				uart4_rx_buf.u8Len = 0;
				
			}
			if(stTjCommAllInfo.pstTjTransmit->bNeed_block == true)
			{
				stTjCommAllInfo.pstTjTransmit->block_count --;
				stTjCommAllInfo.pstTjTransmit->bNeed_block = false;
				xSemaphoreGive(TJ_Trans_SemHandle);
			}
    }
  /* USER CODE END MotorDataParserTaskFun */
}

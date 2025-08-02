
#include "TaskMotorControl.h"
#include "Common.h"
#include "TJ_MotorDrive.h"
#include "TJ_Motor_Control_fml.h"
uint16_t target_position[7] = {1000, 2000, 2000, 2000, 2000, 2000, 1000};
void MotorControlTaskFun(void *argument)
{
	 // 启动初始接收
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_buf.u8receive_data, TJ_DATA_RECV_LEN_MAX);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_buf.u8receive_data, TJ_DATA_RECV_LEN_MAX);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_rx_buf.u8receive_data, TJ_DATA_RECV_LEN_MAX);
	stTjCommAllInfo.Tjtrans_block_mode = WAIT_SEM_NOP; // 设置为非阻塞模式
	osDelay(1000);
	//下发读取帧获取当前位置
	//TJ_Init_Pos(tj_servo, &tj_control_data);
	for (;;)
	{
		TJ_Motor_Control(tj_servo, &tj_control_data); // 控制电机
		osDelay(1);
	}
	/* USER CODE END MotorControlTaskFun */
}

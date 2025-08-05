
#include "TaskMotorControl.h"
#include "Common.h"
#include "AAC_drv.h"
#include "AAC_MotorControl_fml.h"
void MotorControlTaskFun(void *argument)
{
	static uint32_t RS_init_failure = 0;
	// 启动初始接收
	ys_usart_user_init();
	//下发读取帧获取当前位置
	osDelay(2350);//该延时必须，飞特舵机实测上电后需要等待这么一段时间才能稳定读到电机数据，只有读到了电机位置，才不会影响位置回写功能//待定//不一定是时间问题
	RS_init_failure = inspire_motor_init(&hand,&rs_actuator,inspire_comm,inspire_data);
	/*更新电机角度到指令返回值*/
  ys_set_status(&hand,inspire_data,&lower_response);
	init_request_data(&upper_request,&lower_response);
	for (;;)
	{
		hand_planner(&hand,&rs_actuator,&RS_Cmd_all);
		inspire_motor_control(&rs_actuator,inspire_comm,inspire_data);
		//updata_hand_sta(&hand,&rs_actuator,&RS_Sta_all);
		//follow_mode_r(inspire_comm,1,1600);
		osDelay(1);
	}
	/* USER CODE END MotorControlTaskFun */
}

#include "ControlTask.h"
#include "Driver_LiftMechanism.h"
#include "Driver_Chassis.h"
#include "Driver_Manipulator.h"
#include "Driver_GuideWheel.h"
#include "Driver_Sensor.h"
#include "Driver_Manipulator.h"
#include "Driver_Beltraise.h"
#include "CanBusTask.h"
#include "StatusMachine.h"
#include "imu.h"
#include "test_imu.h"
#include "Driver_ImageSensor.h"

uint32_t time_tick_1ms = 0;
void ControlLoopTaskInit(void)
{
	time_tick_1ms = 0;
	StatusMachine_Init();
	//��е������������ �Ĵ洢ֵ
	
}
void ControlTask(void)
{
//	static uint32_t tick = 0;����֮ǰ��д������һЩ����
	time_tick_1ms++;


	Manipulator_Control();
	if(time_tick_1ms%4==0)
	{
		Chassis_Control();
		LiftMachanism_Control();
	}
	if(time_tick_1ms%10 == 0)
	{
		GuideWheel_Control();
	}
	
	InfraredSensor_StateGet();


	ImageSensor_Control();
	IMU_Task();/////

	
}



void Drivers_Control_Task(void const * argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	ControlLoopTaskInit();
  /* Infinite loop */
  for(;;)
  {

			
			ControlTask();
			
			vTaskDelayUntil(&xLastWakeTime,1/portTICK_RATE_MS);//��ʱ��������̬

	

		
		
		
		
  }
  /* USER CODE END Can_Send_Task */
}

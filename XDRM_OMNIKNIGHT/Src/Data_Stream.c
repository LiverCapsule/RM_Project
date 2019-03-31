#include "Data_Stream.h"






void Data_Process_Task(void const * argument)//过两天会把canbus和裁判系统和minipc都放在这里//minipc和裁判系统用信号量改变频率
{

  /* USER CODE BEGIN Led_Toggle_Task */
	
		
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
    
		
		vTaskDelayUntil(&xLastWakeTime,10/portTICK_RATE_MS);

  }
  /* USER CODE END Led_Toggle_Task */
}

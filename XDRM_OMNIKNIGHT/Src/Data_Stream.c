#include "Data_Stream.h"






void Data_Process_Task(void const * argument)//��������canbus�Ͳ���ϵͳ��minipc����������//minipc�Ͳ���ϵͳ���ź����ı�Ƶ��
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

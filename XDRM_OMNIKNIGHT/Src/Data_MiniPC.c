#include "Data_MiniPC.h"
#include "stm32f4xx_hal.h"

int8_t PC_Data[8] = {0};

#include "BSP_USART.h"

Mini_PC_Data_t Mini_Pc_Data;

//uint8_t Mini_PC_Data = 0;

void Info_Rc_MiniPC(void)
{
	
	
//	HAL_UART_Receive(&huart2,&Mini_PC_Data,1,1);
	if(PC_Data[1] == 1)//识别到
	{
		Mini_Pc_Data.up_or_down =  PC_Data[2];
		Mini_Pc_Data.left_or_right = (PC_Data[3]>>4);///这是16位的值
		Mini_Pc_Data.front_or_back = PC_Data[4];
	}
	else
		Mini_Pc_Data.left_or_right = 0;

}


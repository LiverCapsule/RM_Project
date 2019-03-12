#include "BSP.h"

#include "BSP_ADC.h"
#include "BSP_CAN.h"
#include "BSP_CRC.h"
#include "BSP_DAC.h"
#include "BSP_DMA.h"
#include "BSP_I2C.h"
#include "BSP_SPI.h"
#include "BSP_TIM.h"
#include "BSP_USART.h"
#include "BSP_GPIO.h"

void BSP_Init(void)
{
	
	MX_GPIO_Init();
	MX_CAN1_Init();
	MX_DMA_Init();
	MX_SPI4_Init();
	MX_TIM4_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	
	MX_USART6_UART_Init();
	
	
}





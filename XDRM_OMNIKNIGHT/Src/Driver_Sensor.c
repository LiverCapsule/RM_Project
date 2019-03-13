#include "Driver_Sensor.h"

uint16_t InfraredState_back;
uint16_t InfraredState_front;
void InfraredSensor_StateGet(void)//红外传感器数据获取
{
	InfraredState_back = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
	InfraredState_front  = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2);
}

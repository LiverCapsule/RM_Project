#ifndef __DRIVER_SENSOR_H_
#define __DRIVER_SENSOR_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stdint.h"
#define FrontIs
extern uint16_t InfraredState_back;
extern uint16_t InfraredState_front;

void InfraredSensor_StateGet(void);

#endif



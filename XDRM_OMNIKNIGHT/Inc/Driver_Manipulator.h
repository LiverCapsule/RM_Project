#ifndef __DRIVER_MANIPULATOR
#define __DRIVER_MANIPULATOR

#include "config.h"



#define ARM_PINCH   	  HAL_GPIO_WritePin(GPIOI,GPIO_PIN_9,GPIO_PIN_SET);
#define ARM_UNPINCH   HAL_GPIO_WritePin(GPIOI,GPIO_PIN_9,GPIO_PIN_RESET);

#define	ARM_RAISE				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET);
#define	ARM_DESCEND		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET);


void Manipulator_Control(void);
extern float ArmMotorAngleRef;
#endif



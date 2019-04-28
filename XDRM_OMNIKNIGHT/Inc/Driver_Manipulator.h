#ifndef __DRIVER_MANIPULATOR
#define __DRIVER_MANIPULATOR

#include "config.h"



#define ARM_PINCH   	  HAL_GPIO_WritePin(GPIOI,GPIO_PIN_9,GPIO_PIN_SET);//用toggle_pin会挺方便的
#define ARM_UNPINCH   HAL_GPIO_WritePin(GPIOI,GPIO_PIN_9,GPIO_PIN_RESET);//但是这样写更加清楚

#define	ARM_AHEAD				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET);
#define	ARM_BACK		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET);




typedef enum
{
	Arm_Locked,
	Arm_NormalRCMode,
	Arm_KeyMouseMode,//这里是自己控制
	Arm_Auto_Get_Egg,//下面是一类
	Arm_Auto_Get_Eggs,
	Arm_Auto_Pull_Eggs,//岛上取弹两种策略，一种是先把全部拉回来，一种是拉回来就取
	Arm_Auto_Get_I_Egg,	
	Arm_Auto_Get_I_Eggs,
	Arm_Auto_Give_Egg,
}Arm_OperateMode_e;


void Manipulator_Control(void);
extern float ArmMotorAngleRef;

extern Arm_OperateMode_e Arm_OperateMode;

#endif



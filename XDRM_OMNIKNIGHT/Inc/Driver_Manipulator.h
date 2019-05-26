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

#define ARM_PINCH_CMD 	1
#define ARM_UNPINCH_CMD 2
#define ARM_AHEAD_CMD 	3
#define ARM_BACK_CMD 		4
#define ARM_RAISE_CMD	  5//
#define ARM_HRAISE_CMD  6//抬到最高
#define ARM_LFALL_CMD		7//降到最低
#define ARM_FALL_CMD    8//
#define ARM_ROTATE_I_CMD  9//刚开始默认为收回状态,朝内
#define ARM_ROTATE_O_CMD  10//
#define ARM_ROTATE_M_CMD  11
#define ARM_LEFT_CMD   12
#define ARM_RIGHT_CMD  13
#define ARM_MIDDLE_CMD 14
#define ARM_GIVE_CMD   15
#define ARM_OPEN_CMD   16
#define ARM_ROTATE_H_CMD 17 //为了抬高弹药箱所作的翻转
#define ARM_MOVE_END_CMD 18
#define ARM_I_LOW_CMD 19
#define ARM_DELAY_CMD 20
#define ARM_ROTATE_MAX_CMD 21
#define ARM_WAIT_CM_CMD 22
#define ARM_ROTATE_AWAY_CMD 23
extern uint32_t CM_AngleMark;

extern uint8_t Arm_Move[40];
extern uint8_t arm_move_i;
void Manipulator_Control(void);
extern float ARM_RotateMotorRefAngle;
extern float ARM_LiftMotorRefAngle;
extern float ARM_TransMotorRefAngle;
extern Arm_OperateMode_e Arm_OperateMode;

#endif



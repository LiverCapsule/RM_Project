#ifndef __DRIVER_MANIPULATOR
#define __DRIVER_MANIPULATOR

#include "config.h"



#define ARM_PINCH   	  HAL_GPIO_WritePin(GPIOI,GPIO_PIN_9,GPIO_PIN_SET);//��toggle_pin��ͦ�����
#define ARM_UNPINCH   HAL_GPIO_WritePin(GPIOI,GPIO_PIN_9,GPIO_PIN_RESET);//��������д�������

#define	ARM_AHEAD				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET);
#define	ARM_BACK		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET);




typedef enum
{
	Arm_Locked,
	Arm_NormalRCMode,
	Arm_KeyMouseMode,//�������Լ�����
	Arm_Auto_Get_Egg,//������һ��
	Arm_Auto_Get_Eggs,
	Arm_Auto_Pull_Eggs,//����ȡ�����ֲ��ԣ�һ�����Ȱ�ȫ����������һ������������ȡ
	Arm_Auto_Get_I_Egg,	
	Arm_Auto_Get_I_Eggs,
	Arm_Auto_Give_Egg,
}Arm_OperateMode_e;


void Manipulator_Control(void);
extern float ArmMotorAngleRef;

extern Arm_OperateMode_e Arm_OperateMode;

#endif



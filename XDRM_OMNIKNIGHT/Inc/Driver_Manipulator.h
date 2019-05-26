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

#define ARM_PINCH_CMD 	1
#define ARM_UNPINCH_CMD 2
#define ARM_AHEAD_CMD 	3
#define ARM_BACK_CMD 		4
#define ARM_RAISE_CMD	  5//
#define ARM_HRAISE_CMD  6//̧�����
#define ARM_LFALL_CMD		7//�������
#define ARM_FALL_CMD    8//
#define ARM_ROTATE_I_CMD  9//�տ�ʼĬ��Ϊ�ջ�״̬,����
#define ARM_ROTATE_O_CMD  10//
#define ARM_ROTATE_M_CMD  11
#define ARM_LEFT_CMD   12
#define ARM_RIGHT_CMD  13
#define ARM_MIDDLE_CMD 14
#define ARM_GIVE_CMD   15
#define ARM_OPEN_CMD   16
#define ARM_ROTATE_H_CMD 17 //Ϊ��̧�ߵ�ҩ�������ķ�ת
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



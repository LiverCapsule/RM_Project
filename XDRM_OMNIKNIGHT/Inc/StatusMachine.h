#ifndef __STATUSMACHINE_H
#define __STATUSMACHINE_H


#include "config.h"


typedef enum
{
	Up_Island,
	Down_Island,
	//�Զ��Ļ��Զ�1������䣬�Զ����ϡ����µڶ��ź͵��µ�һ��
	Fetch_I_Eggs,//���ϼ����µڶ�������//�������Ƿ���Ҫ�е㲻ͬ
	Fetch_I_Egg,//���ϼ����µڶ���һ��
	Fetch_Eggs,//���µ�һ������
	Fetch_Egg,//����һ��
}Auto_Manual_Act_e;



typedef enum
{
    Up_Island_Stop,
    Up_Island_Prepare,
    /****��?�̨���??������?����??�¡��䨬?****/
    Up_Island_BeltDown_First,
    Up_Island_GuideWheelAdvance_First,
    Up_Island_BeltUp_First,
    Up_Island_ChassisAdvance_First,
    /****��?�̨�?t?������?����??�¡��䨬?****/
    Up_Island_BeltDown_Twice,
    Up_Island_GuideWheelAdvance_Twice,
    Up_Island_BeltUp_Twice,
}UpIslandStateTypeDef;

typedef enum
{
    Down_Island_stop,
    Down_Island_Prepare,
    /****??�̨�?t?������?����??�¡��䨬?****/
    Down_Island_ChassisBack_First,
    Down_Island_BeltDown_First,
    Down_Island_GuideWheelBack_First,
    Down_Island_BeltUp_First,
    /****??�̨���??������?����??�¡��䨬?****/
    Down_Island_ChassisBack_Twice,
    Down_Island_BeltDown_Twice,
    Down_Island_GuideWheelBack_Twice,
    Down_Island_BeltUp_Twice,
}DownIslandStateTypeDef;










void StatusMachine_Init(void);
void StatusMachine(void const * argument);
extern InputMode_e	InputMode;
extern GuideWheelModeTypeDef    GuideWheelMode;
extern BeltModeTypeDef          BeltMode;
extern ChassisModeTypeDef       ChassisMode;
extern UpIslandStateTypeDef     UpIslandState;
extern DownIslandStateTypeDef   DownIslandState;
extern uint8_t BM_AngelGet;

#endif


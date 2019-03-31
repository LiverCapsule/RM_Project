#ifndef __STATUSMACHINE_H
#define __STATUSMACHINE_H


#include "config.h"


typedef enum
{
	Up_Island,
	Down_Island,
	//自动的话自动1箱和三箱，自动岛上、岛下第二排和岛下第一排
	Fetch_I_Eggs,//岛上及岛下第二排三箱//这两个是否需要有点不同
	Fetch_I_Egg,//岛上及岛下第二排一箱
	Fetch_Eggs,//岛下第一排三箱
	Fetch_Egg,//岛下一箱
}Auto_Manual_Act_e;



typedef enum
{
    Up_Island_Stop,
    Up_Island_Prepare,
    /****é?μúò??úì¨?×μ??÷×′ì?****/
    Up_Island_BeltDown_First,
    Up_Island_GuideWheelAdvance_First,
    Up_Island_BeltUp_First,
    Up_Island_ChassisAdvance_First,
    /****é?μú?t?úì¨?×μ??÷×′ì?****/
    Up_Island_BeltDown_Twice,
    Up_Island_GuideWheelAdvance_Twice,
    Up_Island_BeltUp_Twice,
}UpIslandStateTypeDef;

typedef enum
{
    Down_Island_stop,
    Down_Island_Prepare,
    /****??μú?t?úì¨?×μ??÷×′ì?****/
    Down_Island_ChassisBack_First,
    Down_Island_BeltDown_First,
    Down_Island_GuideWheelBack_First,
    Down_Island_BeltUp_First,
    /****??μúò??úì¨?×μ??÷×′ì?****/
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


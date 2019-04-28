#ifndef __STATUSMACHINE_H
#define __STATUSMACHINE_H

#include "config.h"






typedef enum
{
    Up_Island_Stop,
    Up_Island_Prepare,
    /****¨¦?¦Ì¨²¨°??¨²¨¬¡§?¡Á¦Ì??¡Â¡Á¡ä¨¬?****/
    Up_Island_BeltDown_First,
    Up_Island_GuideWheelAdvance_First,
    Up_Island_BeltUp_First,
    Up_Island_ChassisAdvance_First,
    /****¨¦?¦Ì¨²?t?¨²¨¬¡§?¡Á¦Ì??¡Â¡Á¡ä¨¬?****/
    Up_Island_BeltDown_Twice,
    Up_Island_GuideWheelAdvance_Twice,
    Up_Island_BeltUp_Twice,
}UpIslandStateTypeDef;

typedef enum
{
    Down_Island_stop,
    Down_Island_Prepare,
    /****??¦Ì¨²?t?¨²¨¬¡§?¡Á¦Ì??¡Â¡Á¡ä¨¬?****/
    Down_Island_ChassisBack_First,
    Down_Island_BeltDown_First,
    Down_Island_GuideWheelBack_First,
    Down_Island_BeltUp_First,
    /****??¦Ì¨²¨°??¨²¨¬¡§?¡Á¦Ì??¡Â¡Á¡ä¨¬?****/
    Down_Island_ChassisBack_Twice,
    Down_Island_BeltDown_Twice,
    Down_Island_GuideWheelBack_Twice,
    Down_Island_BeltUp_Twice,
}DownIslandStateTypeDef;

void StatusMachine_Init(void);
void StatusMachine(void const * argument);
extern UpIslandStateTypeDef     UpIslandState;
//extern DownIslandStateTypeDef   DownIslandState;
extern uint8_t BM_AngleGet;

#endif


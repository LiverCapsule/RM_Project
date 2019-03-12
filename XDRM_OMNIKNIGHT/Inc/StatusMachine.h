#ifndef __STATUSMACHINE_H
#define __STATUSMACHINE_H

#include "Driver_Beltraise.h"
#include "Driver_Chassis.h"
#include "Driver_GuideWheel.h"
#include "Driver_Manipulator.h"
#include "Driver_Remote.h"
#include "Driver_Sensor.h"

#include "config.h"

typedef enum
{
    Stop_Move_Mode,
    Normal_Move_Mode,
    Auto_Up_Island_Mode,
    Auto_Down_Island_Mode,
    Auto_Get_Box_Mode
}CarMoveModeTypeDef;

typedef enum
{
    Up_Island_Stop,
    Up_Island_Prepare,
    /****上第一节台阶的各状态****/
    Up_Island_BeltDown_First,
    Up_Island_GuideWheelAdvance_First,
    Up_Island_BeltUp_First,
    Up_Island_ChassisAdvance_First,
    /****上第二节台阶的各状态****/
    Up_Island_BeltDown_Twice,
    Up_Island_GuideWheelAdvance_Twice,
    Up_Island_BeltUp_Twice,
}UpIslandStateTypeDef;

typedef enum
{
    Down_Island_stop,
    Down_Island_Prepare,
    /****下第二节台阶的各状态****/
    Down_Island_ChassisBack_First,
    Down_Island_BeltDown_First,
    Down_Island_GuideWheelBack_First,
    Down_Island_BeltUp_First,
    /****下第一节台阶的各状态****/
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


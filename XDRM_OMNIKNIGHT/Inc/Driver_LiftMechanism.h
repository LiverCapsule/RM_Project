#ifndef __DRIVER_LIFTMECHANISM_H
#define __DRIVER_LIFTMECHANISM_H


#include "config.h"



#define THRESHOLD   9000

typedef enum
{
	Lift_Locked,
  Lift_NormalRCMode,
  Lift_KeyMouseMode,
  Lift_Auto_UpIsland,
	Lift_Auto_DownIsland,
	Lift_Auto_UpStep,
}LiftMechanismMode_e;




void LM_Get_SpeedRef(void);
void LM_Set_Current(void);
void LM_PID_Task(void);
void LM_Get_PID(void);

void LiftMachanism_Control(void);


extern LiftMechanismMode_e LiftMechanismMode;

#endif


#ifndef __STATUSMACHINE_H
#define __STATUSMACHINE_H

#include "Driver_Beltraise.h"
#include "Driver_Chassis.h"
#include "Driver_GuideWheel.h"
#include "Driver_Manipulator.h"
#include "Driver_Remote.h"
#include "Driver_Sensor.h"

#include "config.h"


void StatusMachine_Init(void);
void StatusMachine(void const * argument);
extern InputMode_e	InputMode;


#endif


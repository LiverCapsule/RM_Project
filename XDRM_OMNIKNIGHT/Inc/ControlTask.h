#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include "config.h"

#define PREPARE_TIME_TICK_MS 4000



void ControlLoopTaskInit(void);
void Drivers_Control_Task(void const * argument);


#endif




#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include "config.h"

#define PREPARE_TIME_TICK_MS 4000

typedef enum
{
    PREPARE_STATE,          //ÉÏµçºó³õÊ¼×´Ì¬
    NORMAL_RC_STATE,        //Ò£¿ØÆ÷¿ØÖÆ×´Ì¬
    KEYBOARD_RC_STATE,      //¼üÅÌ¿ØÖÆ×´Ì¬
    STOP_STATE,             //Í£Ö¹×´Ì¬
}WorkState_e;

void ControlLoopTaskInit(void);
void Drivers_Control_Task(void const * argument);


#endif




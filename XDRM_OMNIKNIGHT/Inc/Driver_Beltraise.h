#ifndef __Driver_BELTRAISE_H
#define __Driver_BELTRAISE_H


#include "config.h"



#define THRESHOLD   9000

typedef enum
{
  Normal_Rc_BeltMove,
  Normal_Key_BeltMove,
  BeltMove_Stop,
	Belt_Up,
	Belt_Down,
  Auto_Up_Island_BeltMove,
  Auto_Down_Island_BeltMove
}BeltModeTypeDef;



void Belt_Control(void);
void BM_Get_SpeedRef(void);
void BM_Set_Current(void);
void BM_PID_Task(void);
void BM_Get_PID(void);

extern BeltModeTypeDef BeltMode;

#endif


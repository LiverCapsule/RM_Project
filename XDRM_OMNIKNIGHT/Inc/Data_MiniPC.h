#ifndef __DATA_MINIPC_H
#define __DATA_MINIPC_H



#include "stdint.h"


typedef struct 
{
	int8_t up_or_down;
	int8_t left_or_right;
	int8_t front_or_back;
}Mini_PC_Data_t;

void Info_Rc_MiniPC(void);



#endif




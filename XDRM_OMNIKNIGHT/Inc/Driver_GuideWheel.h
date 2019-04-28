#ifndef __GUIDEWHEEL_H
#define __GUIDEWHEEL_H

#include "config.h"


typedef enum
{
	GuideWheel_Locked,
	GuideWheel_NormalRCMode,
	GuideWheel_KeyMouseMode,
	GuideWheel_Auto_UpIsland,//上岛时导轮其实就是向前
	GuideWheel_Auto_DownIsland,//下岛时就是向后
}GuideWheelMode_e;

void MotorSpeedSet(void);
void GuideWheel_Control(void);
extern GuideWheelMode_e GuideWheelMode;
	
void GuideWheel_Move_Advance(void);
void GuideWheel_Move_Back(void);
#endif


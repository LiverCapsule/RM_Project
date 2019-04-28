#ifndef __GUIDEWHEEL_H
#define __GUIDEWHEEL_H

#include "config.h"


typedef enum
{
	GuideWheel_Locked,
	GuideWheel_NormalRCMode,
	GuideWheel_KeyMouseMode,
	GuideWheel_Auto_UpIsland,//�ϵ�ʱ������ʵ������ǰ
	GuideWheel_Auto_DownIsland,//�µ�ʱ�������
}GuideWheelMode_e;

void MotorSpeedSet(void);
void GuideWheel_Control(void);
extern GuideWheelMode_e GuideWheelMode;
	
void GuideWheel_Move_Advance(void);
void GuideWheel_Move_Back(void);
#endif


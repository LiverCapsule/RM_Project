#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "stdint.h"
//#include "PID.h"


#define CHASSIS_SPEED_ATTENUATION (1.0f)
#define Ang2Rad(m)  (m/180.0f*M_PI)

typedef __packed struct
{
    int16_t Y;
    int16_t X;
    int16_t Omega;
}ChassisSpeed_Ref_TypeDef;

typedef struct
{
	float ChassisAngle;
	int16_t ChassisWheelSpeedRef[4];
	ChassisSpeed_Ref_TypeDef ChassisSpeedRef;
}ChassisDataTypeDef;


typedef enum
{
    Chassis_Locked,
    Chassis_RC_Normal,
		Chassis_Key_Normal,
    Chassis_Auto,
}ChassisMode_e;



void Chassis_Control(void);
void CM_Get_SpeedRef(void);
void CalculateWheelSpeed(float vx, float vy, float omega, float radian, uint16_t maxspeed);
void CM_Set_Current(void);
void CM_Calc_Output(void);
void CM_Get_PID(void);


extern ChassisDataTypeDef ChassisData;
extern ChassisMode_e ChassisMode;



#endif




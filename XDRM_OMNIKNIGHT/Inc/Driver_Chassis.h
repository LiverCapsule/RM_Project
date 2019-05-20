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
{		Chassis_Locked,
    Chassis_NormalRCMode,
    Chassis_KeyMouseMode,
		Chassis_Auto_CaliForEgg,
//		Chassis_Advance,
//		Chassis_Back,

//
		Chassis_Auto_UpIsland,
		Chassis_Auto_DownIsland,

}ChassisMode_e;

#define NORMAL_FORWARD_BACK_SPEED 			400
#define NORMAL_LEFT_RIGHT_SPEED   			300
#define HIGH_FORWARD_BACK_SPEED 			700//600
#define HIGH_LEFT_RIGHT_SPEED   			600//500
#define LOW_FORWARD_BACK_SPEED 				180
#define LOW_LEFT_RIGHT_SPEED   				180

#define MOUSE_LR_RAMP_TICK_COUNT			50
#define MOUSR_FB_RAMP_TICK_COUNT			300
#define KEY_FACTOR_NORMAL             15
void Chassis_Control(void);
void CM_Get_SpeedRef(void);
void CalculateWheelSpeed(float vx, float vy, float omega, float radian, uint16_t maxspeed);
void CM_Set_Current(void);
void CM_Calc_Output(void);
void CM_Get_PID(void);


extern ChassisDataTypeDef ChassisData;
extern ChassisMode_e ChassisMode;



#endif




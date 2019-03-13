#ifndef __DRIVERLIB_PID_H
#define __DRIVERLIB_PID_H





#include "Config.h"


#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\



#define YAW_POSITION_KP_DEFAULTS  10
#define YAW_POSITION_KI_DEFAULTS  0
#define YAW_POSITION_KD_DEFAULTS  0

#define YAW_SPEED_KP_DEFAULTS  25//  25
#define YAW_SPEED_KI_DEFAULTS  0
#define YAW_SPEED_KD_DEFAULTS  0

// avoid bang --->  position:20.0  speed:19.0
//big bang   22.5 20.0
#define PITCH_POSITION_KP_DEFAULTS  20.0
#define PITCH_POSITION_KI_DEFAULTS  0
#define PITCH_POSITION_KD_DEFAULTS  0

#define PITCH_SPEED_KP_DEFAULTS  7.0	//7
#define PITCH_SPEED_KI_DEFAULTS  0
#define PITCH_SPEED_KD_DEFAULTS  0

#define CHASSIS_SPEED_KP_DEFAULTS  50//60
#define CHASSIS_SPEED_KI_DEFAULTS  0
#define CHASSIS_SPEED_KD_DEFAULTS  0

#define CHASSIS_ROTATE_KP_DEFAULTS  0//4
#define CHASSIS_ROTATE_KI_DEFAULTS  0
#define CHASSIS_ROTATE_KD_DEFAULTS  0

#define SHOOT_SPEED_KP_DEFAULTS  1.5
#define SHOOT_SPEED_KI_DEFAULTS  0
#define SHOOT_SPEED_KD_DEFAULTS  0





#define	IMU_POSITION_KP_DEFAULT 2//这个值和定时器的频率有关//牵扯到很多东西
#define	IMU_POSITION_KI_DEFAULT 0//不可加积分
#define IMU_POSITION_KD_DEFAULT 0


enum
{
	POSITION_PID,
	DELTA_PID,
	VAGUE_PID,
	OTHER,
};




#define IMU_HEAT_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0,0,0},\
	IMU_POSITION_KP_DEFAULT,\
	IMU_POSITION_KI_DEFAULT,\
	IMU_POSITION_KD_DEFAULT,\
	0,\
	0,\
	0,\
	2500,\
	2500,\
	2500,\
	0,\
	2500,\
	0,\
	DELTA_PID,\
	&PID_Calc,\
	&PID_Reset,\
}//如果改了定时器3的预装值，记得或许要改pid输出最大值//还有上面的pid系数最大值也要注意


#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0,0,0},\
	CHASSIS_SPEED_KP_DEFAULTS,\
	CHASSIS_SPEED_KI_DEFAULTS,\
	CHASSIS_SPEED_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4000,\
	4000,\
	4000,\
	0,\
	4000,\
	0,\
	POSITION_PID,\
	&PID_Calc,\
	&PID_Reset,\
}


#define BELT_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0,0,0},\
	CHASSIS_SPEED_KP_DEFAULTS,\
	CHASSIS_SPEED_KI_DEFAULTS,\
	CHASSIS_SPEED_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	13000,\
	13000,\
	13000,\
	0,\
	13000,\
	0,\
	POSITION_PID,\
	&PID_Calc,\
	&PID_Reset,\
}
#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0,0,0},\
	CHASSIS_ROTATE_KP_DEFAULTS,\
	CHASSIS_ROTATE_KI_DEFAULTS,\
	CHASSIS_ROTATE_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	600,\
	600,\
	600,\
	0,\
	600,\
	0,\
	POSITION_PID,\
	&PID_Calc,\
	&PID_Reset,\
}




typedef __packed struct PID_Regulator_t//这个有点奇怪。。是因为下面有函数指针！所以是为什么呢
{
	float ref;
	float fdb;
	float err[4];
	float kp;
	float ki;
	float kd;
	float output_kp;
	float output_ki;
	float output_kd;
	float output_kpMax;//每个pid默认的最大输出再做考虑
	float output_kiMax;
	float output_kdMax;
	float output;
	float outputMax;
	float index;
  uint8_t type;
	void (*Calc)(struct PID_Regulator_t *pid);//函数指针
	void (*Reset)(struct PID_Regulator_t *pid);
}PID_Regulator_t;

extern PID_Regulator_t CMRotatePID; 
extern PID_Regulator_t CM1SpeedPID;
extern PID_Regulator_t CM2SpeedPID;
extern PID_Regulator_t CM3SpeedPID;
extern PID_Regulator_t CM4SpeedPID;
//extern PID_Regulator_t BeltM1PositionPID;
extern PID_Regulator_t LBMSpeedPID;
//extern PID_Regulator_t BeltM2PositionPID;
extern PID_Regulator_t RBMSpeedPID;
extern PID_Regulator_t AMRotatePID;

void PID_Reset(PID_Regulator_t *pid);
void PID_Calc(PID_Regulator_t *pid);
float PID_Task(PID_Regulator_t *PID_Stucture, float ref, float fdb);

#endif


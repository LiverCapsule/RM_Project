#ifndef __DRIVERLIB_PID_H
#define __DRIVERLIB_PID_H





#include "Config.h"




#define CHASSIS_SPEED_KP_DEFAULTS  15//60
#define CHASSIS_SPEED_KI_DEFAULTS  0
#define CHASSIS_SPEED_KD_DEFAULTS  0

#define CHASSIS_ROTATE_KP_DEFAULTS  0//4
#define CHASSIS_ROTATE_KI_DEFAULTS  0
#define CHASSIS_ROTATE_KD_DEFAULTS  0



#define POSITION_KP_DEFAULTS  10.0
#define POSITION_KI_DEFAULTS  0
#define POSITION_KD_DEFAULTS  0


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
}//������˶�ʱ��3��Ԥװֵ���ǵû���Ҫ��pid������ֵ//���������pidϵ�����ֵҲҪע��


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
	8000,\
	8000,\
	8000,\
	0,\
	8000,\
	0,\
	POSITION_PID,\
	&PID_Calc,\
	&PID_Reset,\
}//���̵����������Ƶ���������һ��



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
	1000,\
	600,\
	600,\
	0,\
	1000,\
	0,\
	POSITION_PID,\
	&PID_Calc,\
	&PID_Reset,\
}



#define LIFTCHAIN_MOTOR_SPEED_PID_DEFAULT \
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
	9000,\
	8000,\
	8000,\
	0,\
	13000,\
	0,\
	POSITION_PID,\
	&PID_Calc,\
	&PID_Reset,\
}


#define LIFTCHAIN_MOTOR_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0,0,0},\
	POSITION_KP_DEFAULTS,\
	POSITION_KI_DEFAULTS,\
	POSITION_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	2000,\
	800,\
	800,\
	0,\
	1000,\
	0,\
	POSITION_PID,\
	&PID_Calc,\
	&PID_Reset,\
}


#define ROTATE_MOTOR_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0,0,0},\
	POSITION_KP_DEFAULTS,\
	POSITION_KI_DEFAULTS,\
	POSITION_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	5000,\
	800,\
	800,\
	0,\
	1000,\
	0,\
	POSITION_PID,\
	&PID_Calc,\
	&PID_Reset,\
}

typedef __packed struct PID_Regulator_t//����е���֡�������Ϊ�����к���ָ�룡������Ϊʲô��
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
	float output_kpMax;//ÿ��pidĬ�ϵ���������������
	float output_kiMax;
	float output_kdMax;
	float output;
	float outputMax;
	float index;
  uint8_t type;
	void (*Calc)(struct PID_Regulator_t *pid);//����ָ��
	void (*Reset)(struct PID_Regulator_t *pid);
}PID_Regulator_t;



extern PID_Regulator_t CMRotatePID; 
extern PID_Regulator_t CM1SpeedPID;
extern PID_Regulator_t CM2SpeedPID;
extern PID_Regulator_t CM3SpeedPID;
extern PID_Regulator_t CM4SpeedPID;

extern PID_Regulator_t LCM1SpeedPID;
extern PID_Regulator_t LCM2SpeedPID;
extern PID_Regulator_t LCM3SpeedPID;
extern PID_Regulator_t LCM4SpeedPID;
extern PID_Regulator_t LCM5SpeedPID;
extern PID_Regulator_t LCM6SpeedPID;

extern PID_Regulator_t AMRotatePID;
extern PID_Regulator_t AMMovePID;
extern PID_Regulator_t GM1SpeedPID;
extern PID_Regulator_t GM2SpeedPID;


void PID_Reset(PID_Regulator_t *pid);
void PID_Calc(PID_Regulator_t *pid);
float PID_Task(PID_Regulator_t *PID_Stucture, float ref, float fdb);

#endif


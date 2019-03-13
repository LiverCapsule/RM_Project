#ifndef __SUPERVISETASK_H
#define __SUPERVISETASK_H

#include "stm32f4xx.h"
#include "main.h"
#include "cmsis_os.h"
#include "config.h"
#include "task.h"

#define LOST_ERROR_RC               0u
#define LOST_ERROR_MOTOR1		    1u				
#define LOST_ERROR_MOTOR2		    2u			
#define LOST_ERROR_MOTOR3		    3u			
#define LOST_ERROR_MOTOR4		    4u
#define LOST_ERROR_MOTOR_LBELT      5u
#define LOST_ERROR_MOTOR_RBELT      6u
#define LOST_ERROR_MOTOR_ARM        7u
#define LOST_ERROR_CAN_TX           8u
#define LOST_ERROR_CAN_RX              9u

typedef __packed struct  Lost_Counter_Max
{
	const uint16_t LOST_ERROR_RC_MAX;
	const uint16_t LOST_ERROR_MOTOR1_MAX;
	const uint16_t LOST_ERROR_MOTOR2_MAX;
	const uint16_t LOST_ERROR_MOTOR3_MAX;
	const uint16_t LOST_ERROR_MOTOR4_MAX;
	const uint16_t LOST_ERROR_MOTOR_LBELT_MAX;
	const uint16_t LOST_ERROR_MOTOR_RBELT_MAX;
    const uint16_t LOST_ERROR_MOTOR_ARM_MAX;
    const uint16_t LOST_ERROR_CAN_TX_MAX;
    const uint16_t LOST_ERROR_CAN_RX_MAX;
}Lost_Counter_Max;			//看门狗溢出值

#define THREAD_IN  0u
#define THREAD_OUT 1u
typedef struct 
{
	uint32_t previousTime;
	uint32_t excutionTime;
	uint32_t deltaTime;
}ThreadMonitor_t;

#define LOST_ERROR_ALL (LOST_ERROR_RC|LOST_ERROR_IMU|LOST_ERROR_MOTOR1|LOST_ERROR_MOTOR2|LOST_ERROR_MOTOR3|LOST_ERROR_MOTOR4|LOST_ERROR_GIMBAL_YAW|LOST_ERROR_GIMBAL_YAW|LOST_ERROR_DEADLOCK|LOST_ERROR_NOCALI)

extern uint32_t lost_err;
extern uint32_t lost_counter[40];


extern uint16_t DBUSFrameRate;
extern uint16_t DBUSFrameCounter;

extern uint16_t ChassisFrameRate[4];
extern uint16_t ChassisFrameCounter[4];

extern uint16_t LbeltFrameRate;
extern uint16_t LbeltFrameCounter;

extern uint16_t RbeltFrameRate;
extern uint16_t RbeltFrameCounter;

extern uint16_t MotorArmFrameRate;
extern uint16_t MotorArmFrameCounter;

extern uint16_t CAN_Send_FrameRate;
extern uint16_t CAN_Send_FrameCounter;

extern uint16_t CAN_Res_FrameRate;
extern uint16_t CAN_Res_FrameCounter;

extern uint16_t UART2FrameRate;
extern uint16_t UART2FrameCounter;

void SuperviseTask(void);

void Set_Error_Flag(uint32_t index);
void Reset_Error_Flag(uint32_t index);

uint8_t Is_Lost_Error_Set(uint32_t err_code);
uint8_t Is_Serious_Error(void);
void ThreadMonitor(ThreadMonitor_t* obj,uint8_t Mode);
void Monitor_ThreadCreate(osPriority taskPriority);

#endif



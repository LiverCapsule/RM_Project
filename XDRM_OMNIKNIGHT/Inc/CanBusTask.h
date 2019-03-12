#ifndef __CANBUSTASK_H
#define __CANBUSTASK_H



#include "Config.h"


#define CAN_BUS1_MOTOR1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS1_MOTOR2_FEEDBACK_MSG_ID           0x202
#define CAN_BUS1_MOTOR3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS1_MOTOR4_FEEDBACK_MSG_ID           0x204
#define CAN_BUS1_BELTMOTOR1_FEEDBACK_MSG_ID				0x205
#define CAN_BUS1_BELTMOTOR2_FEEDBACK_MSG_ID				0x206
#define CAN_BUS1_ARMMOTOR_FEEDBACK_MSG_ID					0x207


#define CAN_SEND_NUM 3


#define RATE_BUF_SIZE 6
typedef struct{
	int16_t	 	speed_rpm;//转速
	int16_t  	real_current;//实际转矩
	uint8_t  	Temperature;
	uint16_t 	angle;//角度
	uint16_t  lastangle;
	int32_t   ecd_angle;
	int16_t   round_cnt;
}Measure;


typedef struct{
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;													//两次编码器之间的差值
	int32_t temp_count;                   //计数用
	uint8_t buf_count;								//滤波更新buf用
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];	
	int32_t round_cnt;										//圈数
	int32_t filter_rate;											//速度
	float ecd_angle;											//角度
}Encoder;


void Can_Send_CM(int16_t iq_1,int16_t iq_2,int16_t iq_3,int16_t iq_4);
void Can_Send(void);
void Can_Send_Task(void const * argument);
void Can_Msg_Process(void);
void Can_Send_BM(int16_t iq_1,int16_t iq_2,int16_t iq_3,int16_t iq_4);

extern Measure Motor1_Measure;
extern Measure Motor2_Measure;
extern Measure Motor3_Measure;
extern Measure Motor4_Measure;

extern Measure LBeltM_Measure;
extern Measure RBeltM_Measure;
extern Measure ARMM_Measure;

extern volatile Encoder TurntableEncoder;
#endif



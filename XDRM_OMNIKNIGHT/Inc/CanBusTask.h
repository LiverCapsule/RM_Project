#ifndef __CANBUSTASK_H
#define __CANBUSTASK_H



#include "Config.h"

/*以下是CAN1总线上的ID，为抓取及基本运动所用电机*/
#define CAN_BUS1_CHASSISMOTOR1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS1_CHASSISMOTOR2_FEEDBACK_MSG_ID           0x202
#define CAN_BUS1_CHASSISMOTOR3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS1_CHASSISMOTOR4_FEEDBACK_MSG_ID           0x204
#define CAN_BUS1_LIFTCHAINMOTOR5_FEEDBACK_MSG_ID         0x205//上面的两个链条电机左低右高
#define CAN_BUS1_LIFTCHAINMOTOR6_FEEDBACK_MSG_ID         0x206
#define CAN_BUS1_FLIPARMMOTOR_FEEDBACK_MSG_ID						 0x207
#define CAN_BUS1_MOVEARMMOTOR_FEEDBACK_MSG_ID						 0x208
				
/*以下是CAN2总线上的ID，为上岛部分所用电机*/
#define CAN_BUS2_LIFTCHAINMOTOR1_FEEDBACK_MSG_ID         0x201//四个链条电机id顺序与麦轮电机相同，从左上角开始顺时针
#define CAN_BUS2_LIFTCHAINMOTOR2_FEEDBACK_MSG_ID         0x202
#define CAN_BUS2_LIFTCHAINMOTOR3_FEEDBACK_MSG_ID         0x203
#define CAN_BUS2_LIFTCHAINMOTOR4_FEEDBACK_MSG_ID         0x204
#define CAN_BUS2_GUIDEMOTOR1_FEEDBACK_MSG_ID             0x205//左低右高
#define CAN_BUS2_GUIDEMOTOR2_FEEDBACK_MSG_ID             0x206


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


void CAN1_Send_CM(int16_t iq_1,int16_t iq_2,int16_t iq_3,int16_t iq_4);
void CAN1_Send_LM(int16_t iq_1,int16_t iq_2,int16_t iq_3,int16_t iq_4);
void CAN2_Send_GM(int16_t iq_1,int16_t iq_2);
void CAN2_Send_LM(int16_t iq_1,int16_t iq_2,int16_t iq_3,int16_t iq_4);
void CAN_Send(void);
void CAN1_Msg_Process(void);
void CAN2_Msg_Process(void);
void Can_Send_Task(void const * argument);

extern Measure Chassis_Motor1_Measure;
extern Measure Chassis_Motor2_Measure;
extern Measure Chassis_Motor3_Measure;
extern Measure Chassis_Motor4_Measure;

extern Measure LiftChain_Motor1_Measure;//1,2,3,4电机顺序和底盘一样，左上角开始顺时针
extern Measure LiftChain_Motor2_Measure;
extern Measure LiftChain_Motor3_Measure;
extern Measure LiftChain_Motor4_Measure;

extern Measure LiftChain_Motor5_Measure;//5，6位抬机械臂的链条电机
extern Measure LiftChain_Motor6_Measure;

extern Measure Guide_Motor1_Measure;
extern Measure Guide_Motor2_Measure;

extern Measure FlipArm_Motor_Measure;//7为左右移动机
extern Measure MoveArm_Motor_Measure;





extern volatile Encoder TurntableEncoder;
#endif



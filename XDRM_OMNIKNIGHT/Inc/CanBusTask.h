#ifndef __CANBUSTASK_H
#define __CANBUSTASK_H



#include "Config.h"

/*������CAN1�����ϵ�ID��Ϊץȡ�������˶����õ��*/
#define CAN_BUS1_CHASSISMOTOR1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS1_CHASSISMOTOR2_FEEDBACK_MSG_ID           0x202
#define CAN_BUS1_CHASSISMOTOR3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS1_CHASSISMOTOR4_FEEDBACK_MSG_ID           0x204
#define CAN_BUS1_LIFTCHAINMOTOR5_FEEDBACK_MSG_ID         0x205//��������������������Ҹ�
#define CAN_BUS1_LIFTCHAINMOTOR6_FEEDBACK_MSG_ID         0x206
#define CAN_BUS1_FLIPARMMOTOR_FEEDBACK_MSG_ID						 0x207
#define CAN_BUS1_MOVEARMMOTOR_FEEDBACK_MSG_ID						 0x208
				
/*������CAN2�����ϵ�ID��Ϊ�ϵ��������õ��*/
#define CAN_BUS2_LIFTCHAINMOTOR1_FEEDBACK_MSG_ID         0x201//�ĸ��������id˳�������ֵ����ͬ�������Ͻǿ�ʼ˳ʱ��
#define CAN_BUS2_LIFTCHAINMOTOR2_FEEDBACK_MSG_ID         0x202
#define CAN_BUS2_LIFTCHAINMOTOR3_FEEDBACK_MSG_ID         0x203
#define CAN_BUS2_LIFTCHAINMOTOR4_FEEDBACK_MSG_ID         0x204
#define CAN_BUS2_GUIDEMOTOR1_FEEDBACK_MSG_ID             0x205//����Ҹ�
#define CAN_BUS2_GUIDEMOTOR2_FEEDBACK_MSG_ID             0x206


#define CAN_SEND_NUM 3


#define RATE_BUF_SIZE 6
typedef struct{
	int16_t	 	speed_rpm;//ת��
	int16_t  	real_current;//ʵ��ת��
	uint8_t  	Temperature;
	uint16_t 	angle;//�Ƕ�
	uint16_t  lastangle;
	int32_t   ecd_angle;
	int16_t   round_cnt;
}Measure;


typedef struct{
	int32_t raw_value;   									//���������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
	int32_t ecd_value;                       //��������������ı�����ֵ
	int32_t diff;													//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                   //������
	uint8_t buf_count;								//�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	
	int32_t round_cnt;										//Ȧ��
	int32_t filter_rate;											//�ٶ�
	float ecd_angle;											//�Ƕ�
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

extern Measure LiftChain_Motor1_Measure;//1,2,3,4���˳��͵���һ�������Ͻǿ�ʼ˳ʱ��
extern Measure LiftChain_Motor2_Measure;
extern Measure LiftChain_Motor3_Measure;
extern Measure LiftChain_Motor4_Measure;

extern Measure LiftChain_Motor5_Measure;//5��6λ̧��е�۵��������
extern Measure LiftChain_Motor6_Measure;

extern Measure Guide_Motor1_Measure;
extern Measure Guide_Motor2_Measure;

extern Measure FlipArm_Motor_Measure;//7Ϊ�����ƶ���
extern Measure MoveArm_Motor_Measure;





extern volatile Encoder TurntableEncoder;
#endif



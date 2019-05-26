#include "Driver_Manipulator.h"
#include "CanBusTask.h"
#include "Driver_Remote.h"
#include "DriverLib_PID.h"

#define DELAY_MOVEMENT 1


extern TIM_HandleTypeDef htim8;

#define ROTATE_M_ANGLE 1400//1600			  //����Ϊ1000,����1700,���Ϲ��Դ�,���Է�����С
#define ROTATE_ANGLE 3250//3200//         //3500,���ϻ���㹻��ʱ�䵽��Ŀ��ֵ,ˮƽ3300
#define ROTATE_M_ANGLE_DELTA -0
#define ROTATE_ANGLE_DELTA 100
#define ROTATE_ANGLE_MAX 3400

#define ROTATE_ANGLE_AWAY 2700
#define MOVE_ANGLE_R 0//25000
#define MOVE_ANGLE_M 0//����ȥ���������ɶ�14000


#define LIFT_ANGLE 6450//6700//6100//���Ŀ��Ϊ-ֵ ָ̧�����м�
#define LIFT_ANGLE_DELTA1 -0
#define LIFT_H_ANGLE 8800
#define LIFT_ANGLE_DELTA 2250//2500

//����ʱ�����߶�һ��,̧���߶Ⱥͷ�ת�Ƕȶ��ø�


Arm_OperateMode_e Arm_OperateMode;

uint8_t arm_move_i = 0;//��е�۵�ʵʱ����


uint8_t Arm_Fetch_Egg[40] = {ARM_RAISE_CMD,ARM_ROTATE_O_CMD,ARM_PINCH_CMD,ARM_HRAISE_CMD,ARM_ROTATE_M_CMD,ARM_ROTATE_AWAY_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_LFALL_CMD,ARM_MOVE_END_CMD};//MIDLE//����Ϊȫ�ֱ�������ʱ���Ѹ�ֵ��Ԫ�⣬����Ϊ��ԪΪ0

																																																		//����MIDDLE��ɾ����						
uint8_t Arm_Fetch_Egg2[40] ={ARM_RAISE_CMD,ARM_ROTATE_O_CMD,ARM_PINCH_CMD,ARM_HRAISE_CMD,ARM_ROTATE_M_CMD,ARM_ROTATE_O_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_FALL_CMD,\
														 ARM_WAIT_CM_CMD,ARM_ROTATE_O_CMD,ARM_PINCH_CMD,ARM_HRAISE_CMD,ARM_ROTATE_M_CMD,ARM_ROTATE_O_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_FALL_CMD,\
														 ARM_WAIT_CM_CMD,ARM_ROTATE_O_CMD,ARM_PINCH_CMD,ARM_HRAISE_CMD,ARM_ROTATE_M_CMD,ARM_ROTATE_O_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_LFALL_CMD,ARM_MOVE_END_CMD};//ȡ�����������ղ����󣬵���������Χ��(��һ��)								//�����ƽ���Ǳ���ס��
//ȡ���һ�����ڻ�е������ʱ�ȴ�,����ˮƽ�ƶ�һ�κ㶨����

uint8_t Arm_Pull_I_Egg[40]	={ARM_RAISE_CMD,ARM_ROTATE_MAX_CMD,ARM_AHEAD_CMD,ARM_PINCH_CMD,\
	ARM_BACK_CMD,ARM_DELAY_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_LFALL_CMD,ARM_MOVE_END_CMD};													 
																														//�ȷ��ˣ����׵ȴ�

	uint8_t Arm_Fetch_I_Egg[40] = {ARM_RAISE_CMD,ARM_ROTATE_O_CMD,ARM_PINCH_CMD,ARM_HRAISE_CMD,ARM_ROTATE_M_CMD,ARM_ROTATE_AWAY_CMD,ARM_UNPINCH_CMD,ARM_ROTATE_I_CMD,ARM_LFALL_CMD,ARM_MOVE_END_CMD};
	
//�����eggsָ����������ҩ��


//uint8_t
//ͨ��״̬�����

float ARM_RotateMotorRefAngle = 0;
float ARM_LiftMotorRefAngle = 0;
float ARM_TransMotorRefAngle = 0;

#include "CanbusTask.h"
float AM_SPEED_C = 1;

//ARM_PINCH   	  
//ARM_UNPINCH   

//ARM_AHEAD				
//ARM_BACK		
extern AutoMovement_e AutoMovement;


float Motor_EcdAngleSet(float Target,Measure *mea)
{
	Target += mea->ecd_angle;
	return	Target;
}

extern float ALM_SPEED_C;
extern float AM_MOVE_C;

float ARM_LiftMotorRefSpeed = 0;
float ARM_MoveMotorRefSpeed = 0;

//״̬���У���μ���˳��ö���
uint8_t Arm_Move[40] = {0};

void Arm_Movement_Split(void)//�����ֿ�
{
	switch(Arm_OperateMode)
	{
		case Arm_Auto_Get_Egg:
		{
			for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
				Arm_Move[move_n] = Arm_Fetch_Egg[move_n];
			}
		}break;
		case Arm_Auto_Get_Eggs:
		{
			for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
					Arm_Move[move_n] = 0;

				Arm_Move[move_n] = Arm_Fetch_Egg2[move_n];
			}
		}break;
		case Arm_Auto_Get_I_Egg:
		{
			for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
				Arm_Move[move_n] = Arm_Fetch_I_Egg[move_n];
			}
		}break;
		case Arm_Auto_Get_I_Eggs:
		{
				for(uint8_t move_n = 0;move_n < 40;move_n++)
			{				
				Arm_Move[move_n] = 0;

//				Arm_Move[move_n] = Arm_Fetch_I_Egg2[move_n];
			}
		}break;
		case Arm_Auto_Pull_Eggs:
		{
			for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
					Arm_Move[move_n] = 0;

				Arm_Move[move_n] = Arm_Pull_I_Egg[move_n];
			}
		}break;

		case Arm_Locked:
		{
			for(uint8_t move_n = 0;move_n < 40;move_n++)
			{
				Arm_Move[move_n] = 0;
			}
		}break;
		case Arm_NormalRCMode:
		{
			ARM_LiftMotorRefSpeed = RC_CtrlData.rc.ch3*AM_SPEED_C;//����
			ARM_MoveMotorRefSpeed = RC_CtrlData.rc.ch2;//*AM_SPEED_C;//���������,�����
			
			
			
		}break;
		case Arm_KeyMouseMode:
		{//����ģʽ�»�е��ƽ�Ƶ�������ֶ�����,������Ҫ�ںܶ�ط��Ի�е�۵�ˮƽλ�ý��й�λ//��Bʱ�Ƿ���Ҫ��֪
			if(Remote_CheckJumpKey(KEY_W)&& RC_CtrlData.mouse.press_r == 1)
			{
				ARM_LiftMotorRefAngle = LIFT_ANGLE;
			}
			else if(Remote_CheckJumpKey(KEY_S)&& RC_CtrlData.mouse.press_r == 1)
			{
				ARM_LiftMotorRefAngle = 0;//Ӧ��û����
			}
			
			ARM_RotateMotorRefAngle = 0;
			ARM_TransMotorRefAngle = 0;
			
			if(Remote_CheckJumpKey(KEY_R)&&RC_CtrlData.mouse.press_r == 0&&RC_CtrlData.mouse.press_l == 1)
			{
				static uint32_t egg_mark2 = 0;
				if(ARM_LiftMotorRefAngle ==0)//ʵ���п��ܻ���һЩ����
				{
					egg_mark2 = xTaskGetTickCount();
				
				}
				ARM_LiftMotorRefAngle =  LIFT_H_ANGLE+1800;//10600
				
				if(xTaskGetTickCount() - egg_mark2 >1000)
				{
				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
				}
				
//				TIM8->CCR1 = 2400;
			
			}
			if(Remote_CheckJumpKey(KEY_R)&&RC_CtrlData.mouse.press_r == 1&&RC_CtrlData.mouse.press_l == 0)
			{
	//			static uint32_t egg_mark = 0;
	//			if(TIM8->CCR1 == 2400)//����һ����ʱ
	//			{
	//				egg_mark = xTaskGetTickCount();
	//			
	//			}
	//			TIM8->CCR1 = 1200;
	//			if(xTaskGetTickCount() - egg_mark >1000)
	//			{
	//				ARM_LiftMotorRefAngle = 0;
	//			}
				
				
				static uint32_t egg_mark = 0;
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == 1)//����һ����ʱ
				{
					egg_mark = xTaskGetTickCount();
				
				}
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
				if(xTaskGetTickCount() - egg_mark >400)
				{
				  ARM_LiftMotorRefAngle = 0;
				}
			}
			
			
		}
		default:
			break;
	}
}

uint32_t CM_AngleMark = 0;
uint32_t time_mark_fb = 0;
uint32_t time_mark_bf = 0;

float arm_average_angle = 0;
uint16_t time_tick_arm = 0;
uint16_t pinstate1 = 0;

uint8_t Egg_Box_Held = 0;

uint8_t last_arm_move_i = 0;

uint32_t arm_time_mark = 0;

void ArmPart_Get_Movement(void)
{//����
	arm_average_angle = (abs(LiftChain_Motor6_Measure.ecd_angle)+abs(LiftChain_Motor5_Measure.ecd_angle))/2;
	
	if(Arm_OperateMode != Arm_NormalRCMode && Arm_OperateMode != Arm_KeyMouseMode &&AutoMovement != Auto_NoMovement)//��ʵ����д�����ұ��⡣��
	{
		switch (Arm_Move[arm_move_i])
		{
			case ARM_PINCH_CMD:
			{
				ARM_PINCH;			//��������ĵ�ŷ��͵���ҩ��ĵ�ŷ���ͬһ��
				Egg_Box_Held++;
				
				if(xTaskGetTickCount() - arm_time_mark > 500)
				{
					arm_move_i++;		
				}
				
			}break;
			case ARM_UNPINCH_CMD:
			{
				ARM_UNPINCH;
				Egg_Box_Held --;
				
				if(Arm_OperateMode == Arm_Auto_Get_I_Egg)
				{
						if(xTaskGetTickCount() - arm_time_mark > 0)
						{
							arm_move_i++;		
						}
				}
				else
				{
					if(xTaskGetTickCount() - arm_time_mark > 200)
						{
							arm_move_i++;		
						}
					}
			}break;
			
			
			
			case ARM_AHEAD_CMD://����ǰ��ʱ�ٶȽ�������Ҫ�ڽ�����һ������֮ǰ��һ����ʱ
			{
				ARM_AHEAD;
				
				if(xTaskGetTickCount() - arm_time_mark > 800)//�����������ʹ��
				{
					arm_move_i++;		
				}

				
			}break;
			case ARM_BACK_CMD:
			{
				ARM_BACK;
				
				if(xTaskGetTickCount() - arm_time_mark > 800)
				{
					arm_move_i++;		
				}
				

		
				
			}break;
			case ARM_RAISE_CMD:
			{
				CM_AngleMark =(abs(Chassis_Motor1_Measure.ecd_angle) + abs(Chassis_Motor2_Measure.ecd_angle) +abs(Chassis_Motor3_Measure.ecd_angle)+abs(Chassis_Motor4_Measure.ecd_angle))/4;
				ARM_LiftMotorRefAngle = LIFT_ANGLE;
				if(Arm_OperateMode == Arm_Auto_Get_I_Eggs||Arm_OperateMode == Arm_Auto_Pull_Eggs)//Arm_OperateMode == Arm_Auto_Get_I_Egg||
				{
					ARM_LiftMotorRefAngle += LIFT_ANGLE_DELTA;
				}
				
				if(Arm_OperateMode == Arm_Auto_Get_I_Egg)
				{
					ARM_LiftMotorRefAngle += LIFT_ANGLE_DELTA1;
				}
				if(arm_average_angle >ARM_LiftMotorRefAngle - 250)//��һ��ֵ��Ϊ�����������һ����������һ�������ܼ�������
				{
					arm_move_i++;
				}	
			}break;
			case ARM_HRAISE_CMD:
			{
				ARM_LiftMotorRefAngle =LIFT_H_ANGLE;
				if(arm_average_angle >LIFT_H_ANGLE -300)//
				{
				//	if(xTaskGetTickCount() - arm_time_mark > 200)
			//		{
						arm_move_i++;		
			//		}
				
				}	
			}break;
			case ARM_I_LOW_CMD://��е�����غ󷭳�ʱ�ѽǶ�����һ���ڼ�ȡ
			{
				ARM_LiftMotorRefAngle = LIFT_ANGLE - 200;
				if(arm_average_angle <LIFT_ANGLE - 100)
				{
					arm_move_i++;
				}	
			}break;
			case ARM_LFALL_CMD:
			{
				ARM_LiftMotorRefAngle = 0;
				if(arm_average_angle <300)
				{
					arm_move_i++;
				}	
			}break;
			case ARM_FALL_CMD:
			{
				ARM_LiftMotorRefAngle  = LIFT_ANGLE;
				if(arm_average_angle <LIFT_ANGLE+300)
				{
					arm_move_i++;	
				}	
			}break;
			case ARM_LEFT_CMD:
			{
//				ARM_TransMotorRefAngle  = 0;//�����Ϊ0ֵ���Ժ�д��flash���򿪻��Լ�
//				if(MoveArm_Motor_Measure.ecd_angle <2000)
//				{
//					arm_move_i++;
//				}	
				
			}break;
			case ARM_MIDDLE_CMD:
			{
				ARM_TransMotorRefAngle = MOVE_ANGLE_M;
				if(MoveArm_Motor_Measure.ecd_angle >MOVE_ANGLE_M - 200)//����߹�����дС��//MoveArm_Motor_Measure.ecd_angle <MOVE_ANGLE_M + 200)//����ͣ��
				{
					arm_move_i++;
				}	
				
			}break;
			case ARM_RIGHT_CMD:
			{
//				ARM_TransMotorRefAngle = MOVE_ANGLE_R;
//				if(MoveArm_Motor_Measure.ecd_angle > MOVE_ANGLE_R - 400)
//				{
//					arm_move_i++;
//				}	
//				
			}break;
			case ARM_ROTATE_I_CMD:
			{
				
				ARM_RotateMotorRefAngle = 0;
					
				
				if(FlipArm_Motor_Measure.ecd_angle < 300)
				{
					arm_move_i++;
				}	

			}break;
				case ARM_ROTATE_O_CMD:
			{
				ARM_RotateMotorRefAngle = ROTATE_ANGLE;
				
				if(Arm_OperateMode == Arm_Auto_Get_I_Egg||Arm_OperateMode == Arm_Auto_Get_I_Eggs)
				{
					ARM_RotateMotorRefAngle += ROTATE_ANGLE_DELTA;
				}
				

					if(FlipArm_Motor_Measure.ecd_angle > ARM_RotateMotorRefAngle-100)
					{
						if(xTaskGetTickCount() - arm_time_mark > 600)
						{
							arm_move_i++;		
						}
					}	
				
	
					
			}break;
			
			case ARM_ROTATE_AWAY_CMD:
			{						
				ARM_RotateMotorRefAngle = ROTATE_ANGLE_AWAY;
				
		
				
//				if(Egg_Box_Held == 1)//��һ����Ϊ�˽���ҩ��˦��ȥ//��ȫ��������ʱд
//				{
//					if(FlipArm_Motor_Measure.ecd_angle > ROTATE_ANGLE-500)//û���Ŀ��
//					{
//							arm_move_i++;		
//					}	
//				}
//				else 
//				{
					if(FlipArm_Motor_Measure.ecd_angle > ROTATE_ANGLE_AWAY-50)
					{
						if(xTaskGetTickCount() - arm_time_mark > 400)
						{
							arm_move_i++;		
						}
					}	
//				}
				
	
					
			}break;
			
					case ARM_ROTATE_MAX_CMD:
			{
				ARM_RotateMotorRefAngle = ROTATE_ANGLE_MAX;
				
//				if(Arm_OperateMode == Arm_Auto_Get_I_Egg||Arm_OperateMode == Arm_Auto_Get_I_Eggs)
//				{
//					ARM_RotateMotorRefAngle += ROTATE_ANGLE_DELTA;
//				}
				

					if(FlipArm_Motor_Measure.ecd_angle > ROTATE_ANGLE_MAX-50)
					{
						if(xTaskGetTickCount() - arm_time_mark > 600)
						{
							arm_move_i++;		
						}
					}	

					
			}break;
			
			case ARM_ROTATE_M_CMD:
			{
				ARM_RotateMotorRefAngle = ROTATE_M_ANGLE;
				if(Arm_OperateMode == Arm_Auto_Get_I_Egg||Arm_OperateMode == Arm_Auto_Get_I_Eggs)
				{
					ARM_RotateMotorRefAngle += ROTATE_M_ANGLE_DELTA;
				}
				if(FlipArm_Motor_Measure.ecd_angle < ROTATE_M_ANGLE+100)
				{
		
					if(xTaskGetTickCount() - arm_time_mark > 500)
					{
						arm_move_i++;		
					}
				}	
			}break;
			case ARM_GIVE_CMD:
			{
				//����Ƕȿ���
				//���Ŀǰ����Ҫ����ʱ�������Ҫд������һ�����������

				
			}break;
			case ARM_MOVE_END_CMD:
			{
					arm_move_i = 0;
					AutoMovement = Auto_NoMovement;//����δ����
					Arm_OperateMode = Arm_KeyMouseMode;
			}break;
			case ARM_DELAY_CMD:
			{

				//�����ĸ���ʱ�õ�,Ŀǰ�Ǹ�������
				if(xTaskGetTickCount() - arm_time_mark > 450)//
				{
					arm_move_i++;		
				}
				
			}break;
			case ARM_WAIT_CM_CMD:
			{
				//�ȴ����̱�����
			}break;
			default:
			{			
			}
			break;
		}
			if(last_arm_move_i != arm_move_i)//ÿһ��״̬�仯,��¼ʱ��ڵ�,����Ҫ��ʱ�Ķ����н���ʱ��
			{
				arm_time_mark = xTaskGetTickCount();
			}
	
		last_arm_move_i = arm_move_i;

	}

}

float chm_kp = 22;//60

extern PID_Regulator_t LCM5PositionPID;
extern PID_Regulator_t LCM6PositionPID;
extern PID_Regulator_t AMRotatePositionPID;
extern PID_Regulator_t AMMovePositionPID;


float pos_kp = 10;



float arm_output_c =  1;

float rotate_pos_kp[5] = {4,8,5,5,10};
float rotate_speed_kp[5] = {2.5,30,20,10,5};
void Arm_Motor_Get_PID_Para(void)//�Ժ��������������;λ�õ���������Ҫ������ʲô��������
{
	//������û�����⶯��ʱ��PID����
	
	LCM5SpeedPID.kp = LCM6SpeedPID.kp =chm_kp;//22;//chm_kp;
	LCM5PositionPID.kp = LCM6PositionPID.kp = 8;//pos_kp;������

	
	
	AMMovePositionPID.kp = rotate_pos_kp[2];
	AMMovePID.kp = rotate_speed_kp[2];//

	AMRotatePID.kp = 2;
	AMRotatePositionPID.kp = 8;//30;

	arm_output_c = 1;
	if(Arm_OperateMode == Arm_Auto_Get_Egg||Arm_OperateMode == Arm_Auto_Get_Eggs)
	{
		if(Arm_Move[arm_move_i] == ARM_ROTATE_I_CMD && Egg_Box_Held > 0)
		{
			AMRotatePID.kp = rotate_speed_kp[1];
			AMRotatePositionPID.kp = rotate_pos_kp[1];//30;
			arm_output_c = 2;
		}
		else if(Arm_Move[arm_move_i] == ARM_ROTATE_I_CMD && Egg_Box_Held == 0)
		{
			AMRotatePID.kp = 5;//
			AMRotatePositionPID.kp = 12;
			arm_output_c = 1;
		
		}
		if(Arm_Move[arm_move_i] == ARM_ROTATE_AWAY_CMD)//ץȡ�󷭻�ʱ���ٶȱ���������Ҫ��
		{
			
			AMRotatePID.kp = 5;
			arm_output_c = 1;
			AMRotatePositionPID.kp = 8;
		}
		
		if(Arm_Move[arm_move_i] == ARM_ROTATE_H_CMD&&Egg_Box_Held == 1)
		{
			AMRotatePID.kp = 5;//
			AMRotatePositionPID.kp = 14;
			arm_output_c = 2;
		}
		if(Arm_Move[arm_move_i] == ARM_ROTATE_M_CMD)//ץȡ�󷭻�ʱ���ٶȱ���������Ҫ��
		{
			
			AMRotatePID.kp = rotate_speed_kp[3];
			AMRotatePositionPID.kp = rotate_pos_kp[3];
			arm_output_c = 2;
		}
		
		if(Arm_Move[arm_move_i] == ARM_ROTATE_O_CMD &&Egg_Box_Held == 0)//��ȥ��ʱҪ��
		{
			AMRotatePID.kp = rotate_speed_kp[0];
			AMRotatePositionPID.kp = rotate_pos_kp[0];//�Ƕȴ���Ŀ��ֵʱ,�п�����kp���󳬵�
			arm_output_c = 1;
		}
		if(Arm_Move[arm_move_i] == ARM_ROTATE_O_CMD&&Egg_Box_Held > 0)//��ȥ��ʱҪ����
		{
			AMRotatePID.kp = 5;
			arm_output_c = 1;
			AMRotatePositionPID.kp = 8;
		}
		
	}
	else if(Arm_OperateMode == Arm_Auto_Get_I_Egg)//��δ��
	{
		//����Ϊ20��ʱpid�Ȳ���
		if(Arm_Move[arm_move_i] == ARM_ROTATE_I_CMD && Egg_Box_Held > 0)
		{
			AMRotatePID.kp = rotate_speed_kp[1];
			AMRotatePositionPID.kp = rotate_pos_kp[1];//30;
			arm_output_c = 2;
		}
		else if(Arm_Move[arm_move_i] == ARM_ROTATE_I_CMD && Egg_Box_Held == 0)
		{
			AMRotatePID.kp = 5;//
			AMRotatePositionPID.kp = 12;
			arm_output_c = 1;
		}
		if(Arm_Move[arm_move_i] == ARM_ROTATE_M_CMD)//ץȡ�󷭻�ʱ���ٶȱ���������Ҫ��
		{
			
			AMRotatePID.kp = 8;//18;
			AMRotatePositionPID.kp = 20;//40;
			arm_output_c = 9;
			
		}
		if(Arm_Move[arm_move_i] == ARM_ROTATE_AWAY_CMD)//ץȡ�󷭻�ʱ���ٶȱ���������Ҫ��
		{
			
			AMRotatePID.kp = 5;
			arm_output_c = 1;
			AMRotatePositionPID.kp = 8;
		}
		if(Arm_Move[arm_move_i] == ARM_ROTATE_O_CMD &&Egg_Box_Held == 0)//��ȥ��ʱҪ��
		{
			AMRotatePID.kp = rotate_speed_kp[0];
			AMRotatePositionPID.kp = rotate_pos_kp[0];//�Ƕȴ���Ŀ��ֵʱ,�п�����kp���󳬵�
			arm_output_c = 1;
		}
		if(Arm_Move[arm_move_i] == ARM_ROTATE_O_CMD&&Egg_Box_Held > 0)//��ȥ��ʱҪ����
		{
			AMRotatePID.kp = 7;
			arm_output_c = 2;
			AMRotatePositionPID.kp = 15;
		}
	
	
	}
	
	if((Arm_Move[arm_move_i] == ARM_ROTATE_O_CMD || Arm_Move[arm_move_i] == ARM_ROTATE_MAX_CMD) &&Egg_Box_Held == 0)
	{
		AMRotatePID.kp = rotate_speed_kp[0];
		AMRotatePositionPID.kp = rotate_pos_kp[0];//�Ƕȴ���Ŀ��ֵʱ,�п�����kp���󳬵�
		arm_output_c = 1;
	
	}
	
	
	AMMovePID.ki = LCM5SpeedPID.ki = LCM6SpeedPID.ki =AMRotatePID.ki = 0;
	AMMovePID.kd = LCM5SpeedPID.kd = LCM6SpeedPID.kd = AMRotatePID.kd = 0;
}



uint32_t  ref_nnn = 0;

void Arm_Cali_Output(void)
{
	if(Arm_OperateMode == Arm_KeyMouseMode)
	{
		PID_Task(&LCM5PositionPID,-ARM_LiftMotorRefAngle,LiftChain_Motor5_Measure.ecd_angle);//
		PID_Task(&LCM5SpeedPID,LCM5PositionPID.output,LiftChain_Motor5_Measure.speed_rpm/3);//��ʵ���ﶼӦ�����Լ��ٱȵ�,����������̫�õ�PID

		PID_Task(&LCM6PositionPID, ARM_LiftMotorRefAngle,LiftChain_Motor6_Measure.ecd_angle);//
		PID_Task(&LCM6SpeedPID,LCM6PositionPID.output,LiftChain_Motor6_Measure.speed_rpm/3);//
		
		PID_Task(&AMRotatePositionPID,0,FlipArm_Motor_Measure.ecd_angle);//��Ϊ0
		PID_Task(&AMRotatePID,AMRotatePositionPID.output,FlipArm_Motor_Measure.speed_rpm/3);//
		
		PID_Task(&AMMovePositionPID,0,MoveArm_Motor_Measure.ecd_angle);//Ӧ����������0��λ��,����д��֪�᲻�����
		PID_Task(&AMMovePID,AMMovePositionPID.output,MoveArm_Motor_Measure.speed_rpm/3);//

	

	
	
	}
	else if(Arm_OperateMode == Arm_NormalRCMode)//��ʱ�ٶȻ�,Ŀ��ֱֵ��Ϊ�ٶ�
	{
		PID_Task(&LCM5SpeedPID,ARM_LiftMotorRefSpeed,LiftChain_Motor5_Measure.speed_rpm/3);//

		PID_Task(&LCM6SpeedPID,-ARM_LiftMotorRefSpeed,LiftChain_Motor6_Measure.speed_rpm/3);
		
		PID_Task(&AMMovePID,ARM_MoveMotorRefSpeed,MoveArm_Motor_Measure.speed_rpm/3);//

		PID_Task(&AMRotatePID,0,FlipArm_Motor_Measure.speed_rpm/10);//��ʱ���Ϊ��

		
		
		
	}
	else 
	{
		
		PID_Task(&LCM5PositionPID,-ARM_LiftMotorRefAngle,LiftChain_Motor5_Measure.ecd_angle);//
		PID_Task(&LCM5SpeedPID,LCM5PositionPID.output,LiftChain_Motor5_Measure.speed_rpm/3);//

		PID_Task(&LCM6PositionPID, ARM_LiftMotorRefAngle,LiftChain_Motor6_Measure.ecd_angle);//
		PID_Task(&LCM6SpeedPID,LCM6PositionPID.output,LiftChain_Motor6_Measure.speed_rpm/3);
		
		
		PID_Task(&AMRotatePositionPID,ARM_RotateMotorRefAngle,FlipArm_Motor_Measure.ecd_angle);//
		PID_Task(&AMRotatePID,AMRotatePositionPID.output,FlipArm_Motor_Measure.speed_rpm/3);//
		
		PID_Task(&AMMovePositionPID,ARM_TransMotorRefAngle,MoveArm_Motor_Measure.ecd_angle);//
		PID_Task(&AMMovePID,AMMovePositionPID.output,MoveArm_Motor_Measure.speed_rpm/3);//
	}

	

}


float arm_output = 0;
void Arm_Set_Output(void)
{
	arm_output = AMRotatePID.output*arm_output_c;
	if(arm_output>15000)//���Ƶ������������ô���16384������ʱ�᲻����
	{
		arm_output = 15000;
	
	}
	else if(arm_output < -15000)
	{
		arm_output = -15000;
	
	}
	if(WorkState == STOP_STATE||WorkState == PREPARE_STATE||OperateMode == Stop_Mode || Arm_OperateMode== Arm_Locked)
	{
		CAN2_Send_LM(0,0,0,0);
	}
	else 
		CAN2_Send_LM(LCM5SpeedPID.output,LCM6SpeedPID.output,arm_output,AMMovePID.output);//AMRotatePID.output*2,AMMovePID.output);
}

uint32_t channel=0;

void Manipulator_Control(void)
{
	Arm_Movement_Split();
	ArmPart_Get_Movement();

	Arm_Motor_Get_PID_Para();//��pid��ֵ���ڻ�������֮��
	Arm_Cali_Output();
	Arm_Set_Output();
//����е�� ��speed.kp pos.kp �� pos.output speed.output

}


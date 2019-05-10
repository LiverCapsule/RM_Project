#include "Driver_LiftMechanism.h"
#include "StatusMachine.h"
#include "DriverLib_PID.h"
#include "CanBusTask.h"
#include "Driver_Remote.h"
#include "Driver_Sensor.h"

LiftMechanismMode_e LiftMechanismMode;

int16_t BeltMotorSpeedRef[4] = {0,0,0,0};


extern PID_Regulator_t LCM1PositionPID;
extern PID_Regulator_t LCM2PositionPID;
extern PID_Regulator_t LCM3PositionPID;
extern PID_Regulator_t LCM4PositionPID;
uint16_t LiftAngleRef = 0;
extern AutoMovement_e AutoMovement;
float LM_SPEED_C = 1;

void BeltMotorSpeedSet(int16_t speed)
{
//	BeltMotorSpeedRef[0] = speed;
//	BeltMotorSpeedRef[1] = -speed;
//	BeltMotorSpeedRef[2] = speed;
//	BeltMotorSpeedRef[3] = -speed;
	
	BeltMotorSpeedRef[0] = BeltMotorSpeedRef[1] = speed;//ֻ������б�Խǵ��ʱ
	
	
}


uint8_t lmkp[4] = {60,60,60,60};


void LM_Get_PID(void)
{
	LCM1SpeedPID.kp = lmkp[0];//60
	LCM1SpeedPID.ki = LCM2SpeedPID.ki = LCM3SpeedPID.ki = LCM4SpeedPID.ki = 0;
	LCM1SpeedPID.kd = LCM2SpeedPID.kd = LCM3SpeedPID.kd = LCM4SpeedPID.kd = 0;
	
	
	LCM2SpeedPID.kp = lmkp[1];//60
	LCM3SpeedPID.kp = lmkp[2];//60
	LCM3SpeedPID.kp = lmkp[3];//60

	
//	LCM1PositionPID.kp = LCM1PositionPID.kp = 2;

}


//#define UP 1
//#define DOWN 0
extern uint8_t lift_flag_again;
extern uint8_t lift_flag_again1;

//int32_t ecd_angle_stamp;
int8_t flag_gcm = 0;//���ֺ͵��̵��ǰ������flag
float angle_average = 0;




void LM_Get_SpeedRef(void)
{
	switch(LiftMechanismMode)
	{
		case Lift_Locked:
		{
				BeltMotorSpeedSet(0);
		}break;
		case Lift_NormalRCMode:
		{
			BeltMotorSpeedSet(RC_CtrlData.rc.ch3/2*LM_SPEED_C);
		}break;
		case Lift_KeyMouseMode:
		{
			if(Remote_CheckJumpKey(KEY_W) == 1 && RC_CtrlData.mouse.press_l == 1)
			{
				BeltMotorSpeedSet(-200);
			}
			else if(Remote_CheckJumpKey(KEY_S) == 1 && RC_CtrlData.mouse.press_l == 1)
			{
				BeltMotorSpeedSet(200);
			}
			else
			{
				BeltMotorSpeedSet(0);
			}

		}break;
		case Lift_Auto_UpIsland:
		{
			//��̧��������������ʹ�������ֵ����ĳ��ֵ(���ָ���̨�ף�,ʹ������ǰ,����Ҳ��ǰͬʱ����,��ȡ�󲿺�������̧�������ص�ԭλ��,���ּ���һ����(����������һ��̨��)
			//�ظ�����
//			angle_average = (abs(LiftChain_Motor1_Measure.ecd_angle)\
//								  +abs(LiftChain_Motor2_Measure.ecd_angle) \
//									+abs(LiftChain_Motor3_Measure.ecd_angle) \
//									+abs(LiftChain_Motor4_Measure.ecd_angle))/4; //����Ƕȵ�һ�¸�
			angle_average = abs(LiftChain_Motor1_Measure.ecd_angle);
			
			if(Remote_CheckJumpKey(KEY_Z)||(lift_flag_again))//��һ���е���Ծ�����ҿ���������
			{
				LiftAngleRef =  13000;
			}
			
			if(angle_average > 12000||angle_average<200)//����14000,���̾�ǰ��
			{
				flag_gcm = 1;//������ڵ���ǰ��һ��ʱ�������
			}
			else
			{
				flag_gcm = 0;
			}
			lift_flag_again1 =lift_flag_again; 
			
			if(flag_gcm == 1)
			{
				if(InfraredState_back == 0 && angle_average >12000)
				{
					lift_flag_again = 0;
					LiftAngleRef =  0;
				}
			}
	
		}break;
		case Lift_Auto_DownIsland:
		{
			
//			angle_average = (abs(LiftChain_Motor1_Measure.ecd_angle)\
//											+abs(LiftChain_Motor2_Measure.ecd_angle)\
//											+abs(LiftChain_Motor3_Measure.ecd_angle)\
//											+abs(LiftChain_Motor4_Measure.ecd_angle))/4; 
				angle_average = abs(LiftChain_Motor1_Measure.ecd_angle);

				if(InfraredState_back == 1&&angle_average <200)//�����⵽���գ�����δ����
				{
					LiftAngleRef =  12000;
				}
				
				if(angle_average >11800||angle_average < 200)
				{
					flag_gcm = -1;
				}
				else 
				{
					flag_gcm = 0;//����������������״̬��������
				}
				
				if(InfraredState_back == 1&&InfraredState_front == 1&&angle_average > 11000)
				{
					LiftAngleRef =  0;
				}
				
				
		}break;
		default:
		{
		
		}break;
	
	
	
	}


	
	
}



void LM_Calc_Output(void)
{
	
	if(AutoMovement == Auto_NoMovement)
	{
		PID_Task(&LCM2SpeedPID,BeltMotorSpeedRef[1],LiftChain_Motor2_Measure.speed_rpm/10.0);
		PID_Task(&LCM1SpeedPID,BeltMotorSpeedRef[0],LiftChain_Motor1_Measure.speed_rpm/10.0);
		PID_Task(&LCM3SpeedPID,BeltMotorSpeedRef[2],LiftChain_Motor3_Measure.speed_rpm/10.0);
		PID_Task(&LCM4SpeedPID,BeltMotorSpeedRef[3],LiftChain_Motor4_Measure.speed_rpm/10.0);

		if(LCM2SpeedPID.output>-800 && LCM2SpeedPID.output<800 ) LCM2SpeedPID.output=0;
		if(LCM1SpeedPID.output>-800 && LCM1SpeedPID.output<800 ) LCM1SpeedPID.output=0;
		if(LCM3SpeedPID.output>-800 && LCM3SpeedPID.output<800 ) LCM3SpeedPID.output=0;
		if(LCM4SpeedPID.output>-800 && LCM4SpeedPID.output<800 ) LCM4SpeedPID.output=0;
	}
	else
	{//�Զ��£�ʹ��˫�����ƣ�����λ�û�
		PID_Task(&LCM1PositionPID,LiftAngleRef,LiftChain_Motor1_Measure.ecd_angle);//
		PID_Task(&LCM1SpeedPID,LCM1PositionPID.output,LiftChain_Motor1_Measure.speed_rpm/3);//

		PID_Task(&LCM2PositionPID,LiftAngleRef,LiftChain_Motor2_Measure.ecd_angle);//
		PID_Task(&LCM2SpeedPID,LCM2PositionPID.output,LiftChain_Motor2_Measure.speed_rpm/3);//
	
//		PID_Task(&LCM3PositionPID,LiftAngleRef,LiftChain_Motor3_Measure.ecd_angle);//
//		PID_Task(&LCM3SpeedPID,LCM3PositionPID.output,LiftChain_Motor3_Measure.speed_rpm/3);//

//		PID_Task(&LCM4PositionPID, -LiftAngleRef,LiftChain_Motor4_Measure.ecd_angle);//
//		PID_Task(&LCM4SpeedPID,LCM4PositionPID.output,LiftChain_Motor4_Measure.speed_rpm/3);//
	}
	
}


void LM_Set_Current(void)
{
	if(WorkState == STOP_STATE||WorkState == PREPARE_STATE||OperateMode == Stop_Mode ||LiftMechanismMode == Lift_Locked)
		{
			CAN1_Send_LM(0,0);
		}
		else
		{
			CAN1_Send_LM(LCM1SpeedPID.output*1.5,LCM2SpeedPID.output*1.5);//LCM3SpeedPID.output*1.5,LCM4SpeedPID.output*1.5);//
		}

}

void LiftMachanism_Control(void)
{
	LM_Get_PID();
	LM_Get_SpeedRef();
	LM_Calc_Output();
	LM_Set_Current();

}




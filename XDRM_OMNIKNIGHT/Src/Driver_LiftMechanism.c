#include "Driver_LiftMechanism.h"
#include "StatusMachine.h"
#include "DriverLib_PID.h"
#include "CanBusTask.h"
#include "Driver_Remote.h"
#include "Driver_Sensor.h"
#include "imu.h"

LiftMechanismMode_e LiftMechanismMode;

int16_t BeltMotorSpeedRef[4] = {0,0,0,0};


extern PID_Regulator_t LCM1PositionPID;
extern PID_Regulator_t LCM2PositionPID;
extern PID_Regulator_t LCM3PositionPID;
extern PID_Regulator_t LCM4PositionPID;
int16_t LiftAngleRef = 0;
extern AutoMovement_e AutoMovement;
float LM_SPEED_C = 1;
extern float angle_save ;

void BeltMotorSpeedSet(int16_t speed)
{

	
	BeltMotorSpeedRef[0] = BeltMotorSpeedRef[1] = speed;//只有两个斜对角电机时
	
	
}


uint8_t lmkp[4] = {40,40,15,15};


void LM_Get_PID(void)
{
	LCM1SpeedPID.kp = lmkp[0];//60
	LCM1SpeedPID.ki = LCM2SpeedPID.ki = LCM3SpeedPID.ki = LCM4SpeedPID.ki = 0;
	LCM1SpeedPID.kd = LCM2SpeedPID.kd = LCM3SpeedPID.kd = LCM4SpeedPID.kd = 0;
	
	
	LCM2SpeedPID.kp = lmkp[1];//60
	LCM3SpeedPID.kp = lmkp[2];//60
	LCM4SpeedPID.kp = lmkp[3];//60

	
//	LCM1PositionPID.kp = LCM1PositionPID.kp = 2;

}


//#define UP 1
//#define DOWN 0
extern uint8_t lift_flag_again;
extern uint8_t lift_flag_again1;

//int32_t ecd_angle_stamp;
int8_t flag_gcm = 0;//导轮和底盘电机前进后退flag
float angle_average = 0;
float temp = 0;



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
			if(abs(BeltMotorSpeedRef[0]) < 50)
			{
				BeltMotorSpeedRef[0] = BeltMotorSpeedRef[1] = 50;//保持抬升机构向上的力,不能让后面的导轮触地
			
			}				
		}break;
		case Lift_KeyMouseMode:
		{
			if(Remote_CheckJumpKey(KEY_W) == 1 && RC_CtrlData.mouse.press_l == 1&&Remote_CheckJumpKey(KEY_R)==0)
			{
				BeltMotorSpeedSet(-350);
			}
			else if(Remote_CheckJumpKey(KEY_S) == 1 && RC_CtrlData.mouse.press_l == 1&&Remote_CheckJumpKey(KEY_R)==0)
			{
				BeltMotorSpeedSet(350);
			}
			else
			{
				BeltMotorSpeedSet(50);
			}

		}break;
		case Lift_Auto_UpIsland:
		{
			//先抬升机构做动作，使其编码器值大于某个值(麦轮高于台阶）,使导轮向前,麦轮也向前同时工作,读取后部红外收起抬升机构回到原位置,麦轮计数一定后(导轮爬上下一级台阶)
			//重复动作
			//在平地启动时，抬升机构将车抬起到最高时的编码器值在10600左右，不上岛默认给50的速度时这个值约为2100
//			angle_average = (abs(LiftChain_Motor1_Measure.ecd_angle)+abs(LiftChain_Motor2_Measure.ecd_angle))/2;
			angle_average = abs(LiftChain_Motor1_Measure.ecd_angle);
			if(Remote_CheckJumpKey(KEY_Z)||(lift_flag_again))//这一句有点跳跃，而且可能有问题
			{
				LiftAngleRef =  13300;
				temp = angle_average;
				angle_save = pitch_angle;
			}
			
			if(angle_average > 10800 || angle_average < 1200)//大于14000,底盘就前进
			{
				flag_gcm = 1;//这个会在底盘前进一段时间后清零
			}
			else
			{
				flag_gcm = 0;
			}
			lift_flag_again1 =lift_flag_again; 
			
			if(flag_gcm == 1)
			{
				if(InfraredState_back == 0 && angle_average >11000)
				{
					lift_flag_again = 0;
					LiftAngleRef =  -500;//可能这里有问题

				}
			}
	
		}break;
		case Lift_Auto_DownIsland:
		{
				angle_average = abs(LiftChain_Motor1_Measure.ecd_angle);

				if(Remote_CheckJumpKey(KEY_X))//这一句有点跳跃，而且可能有问题
			{
				temp = angle_average;
			}
				if(InfraredState_back == 1&&angle_average <2200)//后面检测到悬空，并且未升下
				{
					LiftAngleRef =  12000;
				}
				
				if(angle_average >10000||angle_average < 1200)//11200 200
				{
					flag_gcm = -1;
				}
				else 
				{
					flag_gcm = 0;//除了链条处于两种状态，都不动
				}
				
				if(InfraredState_back == 1&&InfraredState_front == 1&&angle_average > 10000)
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
	else if(AutoMovement == Auto_Up_Island)
	{//以前的问题是双环,目标值太大,最后导致PID参数过大时电机最后会抖动,参数过小时电机力不足
		//5.20.2:08我怀疑下岛时速度太快导致最后着地时有自由落体的感觉冲击力太大，导致编码器值发生变化
		//所以上岛和下岛时速度用不同的两套
		
		if(-(-LiftAngleRef/19 - LiftChain_Motor1_Measure.ecd_angle/19)>2000/19)
		{
			PID_Task(&LCM2SpeedPID,-500,LiftChain_Motor2_Measure.speed_rpm/10.0);
			PID_Task(&LCM1SpeedPID,-500,LiftChain_Motor1_Measure.speed_rpm/10.0);
			PID_Task(&LCM3SpeedPID,BeltMotorSpeedRef[2],LiftChain_Motor3_Measure.speed_rpm/10.0);
			PID_Task(&LCM4SpeedPID,BeltMotorSpeedRef[3],LiftChain_Motor4_Measure.speed_rpm/10.0);

			if(LCM2SpeedPID.output>-800 && LCM2SpeedPID.output<800 ) LCM2SpeedPID.output=0;	
			if(LCM1SpeedPID.output>-800 && LCM1SpeedPID.output<800 ) LCM1SpeedPID.output=0;
			if(LCM3SpeedPID.output>-800 && LCM3SpeedPID.output<800 ) LCM3SpeedPID.output=0;
			if(LCM4SpeedPID.output>-800 && LCM4SpeedPID.output<800 ) LCM4SpeedPID.output=0;

		
		}else if(-LiftAngleRef/19 - LiftChain_Motor1_Measure.ecd_angle/19 > 2000/19)
		{
			PID_Task(&LCM2SpeedPID,500,LiftChain_Motor2_Measure.speed_rpm/10.0);
			PID_Task(&LCM1SpeedPID,500,LiftChain_Motor1_Measure.speed_rpm/10.0);
			PID_Task(&LCM3SpeedPID,BeltMotorSpeedRef[2],LiftChain_Motor3_Measure.speed_rpm/10.0);
			PID_Task(&LCM4SpeedPID,BeltMotorSpeedRef[3],LiftChain_Motor4_Measure.speed_rpm/10.0);

			if(LCM2SpeedPID.output>-800 && LCM2SpeedPID.output<800 ) LCM2SpeedPID.output=0;	
			if(LCM1SpeedPID.output>-800 && LCM1SpeedPID.output<800 ) LCM1SpeedPID.output=0;
			if(LCM3SpeedPID.output>-800 && LCM3SpeedPID.output<800 ) LCM3SpeedPID.output=0;
			if(LCM4SpeedPID.output>-800 && LCM4SpeedPID.output<800 ) LCM4SpeedPID.output=0;
		}
		else
		{		
			LCM1SpeedPID.kp = 8;//60
	
	
		LCM2SpeedPID.kp = 8;//60
		LCM3SpeedPID.kp = 8;//60
		LCM4SpeedPID.kp = 8;//60

		
		PID_Task(&LCM1PositionPID,-LiftAngleRef/19,LiftChain_Motor1_Measure.ecd_angle/19);//
		
		PID_Task(&LCM2PositionPID,-LiftAngleRef/19,LiftChain_Motor2_Measure.ecd_angle/19);//
		PID_Task(&LCM1SpeedPID,LCM1PositionPID.output,LiftChain_Motor1_Measure.speed_rpm/19);	
		PID_Task(&LCM2SpeedPID,LCM2PositionPID.output,LiftChain_Motor2_Measure.speed_rpm/19);//
		}
	}
	else if(AutoMovement == Auto_Down_Island)
	{
		if(-(-LiftAngleRef/19 - LiftChain_Motor1_Measure.ecd_angle/19)>1000/19)
		{
			PID_Task(&LCM2SpeedPID,-300,LiftChain_Motor2_Measure.speed_rpm/10.0);
			PID_Task(&LCM1SpeedPID,-300,LiftChain_Motor1_Measure.speed_rpm/10.0);
			PID_Task(&LCM3SpeedPID,BeltMotorSpeedRef[2],LiftChain_Motor3_Measure.speed_rpm/10.0);
			PID_Task(&LCM4SpeedPID,BeltMotorSpeedRef[3],LiftChain_Motor4_Measure.speed_rpm/10.0);

			if(LCM2SpeedPID.output>-800 && LCM2SpeedPID.output<800 ) LCM2SpeedPID.output=0;	
			if(LCM1SpeedPID.output>-800 && LCM1SpeedPID.output<800 ) LCM1SpeedPID.output=0;
			if(LCM3SpeedPID.output>-800 && LCM3SpeedPID.output<800 ) LCM3SpeedPID.output=0;
			if(LCM4SpeedPID.output>-800 && LCM4SpeedPID.output<800 ) LCM4SpeedPID.output=0;

		
		}else if(-LiftAngleRef/19 - LiftChain_Motor1_Measure.ecd_angle/19 > 1000/19)
		{
			PID_Task(&LCM2SpeedPID,300,LiftChain_Motor2_Measure.speed_rpm/10.0);
			PID_Task(&LCM1SpeedPID,300,LiftChain_Motor1_Measure.speed_rpm/10.0);
			PID_Task(&LCM3SpeedPID,BeltMotorSpeedRef[2],LiftChain_Motor3_Measure.speed_rpm/10.0);
			PID_Task(&LCM4SpeedPID,BeltMotorSpeedRef[3],LiftChain_Motor4_Measure.speed_rpm/10.0);

			if(LCM2SpeedPID.output>-800 && LCM2SpeedPID.output<800 ) LCM2SpeedPID.output=0;	
			if(LCM1SpeedPID.output>-800 && LCM1SpeedPID.output<800 ) LCM1SpeedPID.output=0;
			if(LCM3SpeedPID.output>-800 && LCM3SpeedPID.output<800 ) LCM3SpeedPID.output=0;
			if(LCM4SpeedPID.output>-800 && LCM4SpeedPID.output<800 ) LCM4SpeedPID.output=0;
		}
		else
		{		
			LCM1SpeedPID.kp = 7;//60
		  LCM2SpeedPID.kp = 7;//60
		  LCM3SpeedPID.kp = 7;//60
		  LCM4SpeedPID.kp = 7;//60

		
		PID_Task(&LCM1PositionPID,-LiftAngleRef/19,LiftChain_Motor1_Measure.ecd_angle/19);//
		
		PID_Task(&LCM2PositionPID,-LiftAngleRef/19,LiftChain_Motor2_Measure.ecd_angle/19);//
		PID_Task(&LCM1SpeedPID,LCM1PositionPID.output,LiftChain_Motor1_Measure.speed_rpm/19);	
		PID_Task(&LCM2SpeedPID,LCM2PositionPID.output,LiftChain_Motor2_Measure.speed_rpm/19);//
	}
	
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
			CAN1_Send_LM((LCM1SpeedPID.output*1.5),(LCM2SpeedPID.output*1.5));//LCM3SpeedPID.output*1.5,LCM4SpeedPID.output*1.5);//
		}

}

void LiftMachanism_Control(void)
{
	LM_Get_PID();
	LM_Get_SpeedRef();
	LM_Calc_Output();
	LM_Set_Current();

}




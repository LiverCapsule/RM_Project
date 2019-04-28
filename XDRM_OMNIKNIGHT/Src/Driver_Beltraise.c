#include "Driver_Beltraise.h"
#include "StatusMachine.h"
#include "DriverLib_PID.h"
#include "CanBusTask.h"
#include "Driver_Remote.h"
#include "Driver_Sensor.h"

#define raise_speed 200
#define drop_speed  -200
#define support_speed -100
int16_t BeltMotorSpeedRef[4] = {0,0,0,0};
BeltModeTypeDef BeltMode;

int bmkp = 60;
uint8_t steps = 0;
uint8_t steps_down = 0;
uint8_t BM_AngleGet = 0;
extern uint8_t S_switch;
extern uint8_t Foward_G_flag;
extern uint8_t Foward_C_flag;
extern uint8_t Back_C_flag;
extern uint8_t Back_GW_flag;


extern uint8_t GWTimeGet;


//总共只需要给一个电机算速度值
//其他电机只需要取他的相反数或者等值


void BeltMotorSpeedSet(int16_t speed)
{
	BeltMotorSpeedRef[0] = speed;
	BeltMotorSpeedRef[1] = -speed;
	BeltMotorSpeedRef[2] = speed;
	BeltMotorSpeedRef[3] = -speed;
}

void Belt_Move_Up(void)
{
	static int32_t LBM1_Angle = 0;
	static int32_t LBM2_Angle = 0;
	static int32_t RBM1_Angle = 0;
	static int32_t RBM2_Angle = 0;
	
	BeltMotorSpeedSet(raise_speed);
	if(BM_AngleGet == 1)
	{
		LBM1_Angle = LiftChain_Motor1_Measure.ecd_angle;
		RBM1_Angle = LiftChain_Motor2_Measure.ecd_angle;
		RBM2_Angle = LiftChain_Motor3_Measure.ecd_angle;
		LBM2_Angle = LiftChain_Motor4_Measure.ecd_angle;
		BM_AngleGet = 0;
	}
	if(abs(LiftChain_Motor1_Measure.ecd_angle - LBM1_Angle) > THRESHOLD)//9000
	{
		BeltMotorSpeedSet(0);
		if(UpIslandState == Up_Island_BeltUp_First) 
		{
			UpIslandState = Up_Island_ChassisAdvance_First;
		}
		else if(UpIslandState == Up_Island_BeltUp_Twice) 
		{
			UpIslandState = Up_Island_Stop;
		}
		else if(DownIslandState == Down_Island_BeltUp_First)
		{
			DownIslandState = Down_Island_ChassisBack_Twice;
		}
		else if(DownIslandState == Down_Island_BeltUp_Twice)
		{
			DownIslandState = Down_Island_stop; 
		}
	}
}


uint16_t aaa = 0;
int32_t LBM1_Angle = 0;

void Belt_Move_Down(void)
{
//	static int32_t LBM2_Angle = 0;
//	static int32_t RBM1_Angle = 0;
//	static int32_t RBM2_Angle = 0;
	
	BeltMotorSpeedSet(drop_speed);
	if(BM_AngleGet == 1)
	{
		LBM1_Angle = LiftChain_Motor1_Measure.ecd_angle;

		BM_AngleGet = 0;
	}
	
	aaa = abs(LiftChain_Motor1_Measure.ecd_angle - LBM1_Angle);
	if(aaa > THRESHOLD)//9000
	{
		BeltMotorSpeedSet(support_speed);
		if (UpIslandState == Up_Island_BeltDown_First) 
		{
			UpIslandState = Up_Island_GuideWheelAdvance_First;
			GWTimeGet = 1;
		}
		else if(UpIslandState == Up_Island_BeltDown_Twice)
		{
			UpIslandState = Up_Island_GuideWheelAdvance_Twice;
			GWTimeGet = 1;

		}
		else if(DownIslandState == Down_Island_BeltDown_First)
		{
			DownIslandState = Down_Island_GuideWheelBack_First;
		}
		else if(DownIslandState == Down_Island_BeltDown_Twice)
		{
			DownIslandState = Down_Island_GuideWheelBack_Twice;
		}
		
	}
}




void 	BM_Calc_Output(void)
{
	PID_Task(&LCM2SpeedPID,BeltMotorSpeedRef[1],LiftChain_Motor2_Measure.speed_rpm/10.0);//float /10.0
	PID_Task(&LCM1SpeedPID,BeltMotorSpeedRef[0],LiftChain_Motor1_Measure.speed_rpm/10.0);//序号存疑
	PID_Task(&LCM3SpeedPID,BeltMotorSpeedRef[2],LiftChain_Motor3_Measure.speed_rpm/10.0);//float /10.0
	PID_Task(&LCM4SpeedPID,BeltMotorSpeedRef[3],LiftChain_Motor4_Measure.speed_rpm/10.0);

	if(LCM2SpeedPID.output>-800 && LCM2SpeedPID.output<800 ) LCM2SpeedPID.output=0;
	if(LCM1SpeedPID.output>-800 && LCM1SpeedPID.output<800 ) LCM1SpeedPID.output=0;
	if(LCM3SpeedPID.output>-800 && LCM3SpeedPID.output<800 ) LCM3SpeedPID.output=0;
	if(LCM4SpeedPID.output>-800 && LCM4SpeedPID.output<800 ) LCM4SpeedPID.output=0;

}

UpIslandStateTypeDef LastUpState = 0;
DownIslandStateTypeDef LastDownState = 0;

void BM_Get_SpeedRef(void)
{
	LastUpState = UpIslandState;
	LastDownState = DownIslandState;
	
	switch (BeltMode)
	{
		case Normal_RC_BeltMove:
		{
			BeltMotorSpeedSet(RC_CtrlData.rc.ch3/2);
		}break;
	
		case Normal_Key_BeltMove:
		{
			if(Remote_CheckJumpKey(KEY_W) == 1 && RC_CtrlData.mouse.press_l == 1)
			{
				BeltMotorSpeedSet(200);
			}
			else if(Remote_CheckJumpKey(KEY_S) == 1 && RC_CtrlData.mouse.press_l == 1)
			{
				BeltMotorSpeedSet(-200);
			}
			else
			{
				BeltMotorSpeedSet(0);
			}
		}break;

		case Belt_Up:
		{
			Belt_Move_Up();
		}break;

		case Belt_Down:
		{
			Belt_Move_Down();
		}break;

		case BeltMove_Stop:
		{
			BeltMotorSpeedSet(0);
		}break;

		default:
		{
			BeltMotorSpeedSet(0);
		}break;
	}
	
	if(UpIslandState!= LastUpState && (UpIslandState == Up_Island_BeltDown_First|| UpIslandState == Up_Island_BeltDown_Twice||UpIslandState == Up_Island_BeltUp_Twice||UpIslandState == Up_Island_BeltUp_First))
	{
		BM_AngleGet = 1;
	}
	if((DownIslandState!= LastDownState)&&(UpIslandState ==Down_Island_BeltDown_First||UpIslandState ==Down_Island_BeltDown_Twice||UpIslandState ==Down_Island_BeltUp_Twice||UpIslandState ==Down_Island_BeltUp_First))
	{
		BM_AngleGet = 1;
	}
//	if(UpIslandState!= LastUpState && (UpIslandState == Up_Island_GuideWheelAdvance_First ||UpIslandState == Up_Island_GuideWheelAdvance_Twice ))
//	{
//		GWTimeGet = 1;
//	}
}


void BM_Get_PID(void)
{

	LCM1SpeedPID.kp = bmkp;//60
	LCM1SpeedPID.ki = 0;
	LCM1SpeedPID.kd = 0;//
	
	
	LCM2SpeedPID.kp = bmkp;//60
	LCM2SpeedPID.ki = 0;
	LCM2SpeedPID.kd = 0;//
	
	LCM3SpeedPID.kp = bmkp;//60
	LCM3SpeedPID.ki = 0;
	LCM3SpeedPID.kd = 0;//
	
	LCM3SpeedPID.kp = bmkp;//60
	LCM3SpeedPID.ki = 0;
	LCM3SpeedPID.kd = 0;//

}


void BM_Set_Current(void)
{
		if(RC_CtrlData.rc.s2 == 2)
		{
			CAN2_Send_LM(0,0,0,0);
		}
		else
		{
    	CAN2_Send_LM(LCM1SpeedPID.output*1.5,LCM2SpeedPID.output*1.5,LCM3SpeedPID.output*1.5,LCM4SpeedPID.output*1.5);//不要在这里output加负号
	//		Can_Send_BM(-LCM2SpeedPID.output,LCM1SpeedPID.output,0,0);//像这样，会有问题，数据类型什么的搞出问题了
		}



}


void Belt_Control(void)
{
	BM_Get_PID();
	BM_Get_SpeedRef();
	BM_Calc_Output();
	BM_Set_Current();

}




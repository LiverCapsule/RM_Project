#include "Driver_GuideWheel.h"
#include "BSP_TIM.h"
#include "Driver_Remote.h"
#include "Driver_Manipulator.h"
#include "Driver_Sensor.h"
#include "math.h"
#include "StatusMachine.h"
#include "CanBusTask.h"
#include "DriverLib_PID.h"

#define OneCarDistance 3000


extern uint8_t Foward_C_flag;
extern uint8_t Back_C_flag;


uint8_t Foward_G_flag = 0;
uint8_t Back_GW_flag = 0;
uint8_t GW_direction;
uint16_t GW_Speed;
int16_t GWSpeedRef[2] = {0,0};
uint8_t GWTimeGet = 0;
uint32_t GWRunTime = 0;

GuideWheelMode_e GuideWheelMode;


void 	GM_Calc_Output(void)
{
	PID_Task(&GM1SpeedPID,GWSpeedRef[0],Guide_Motor1_Measure.speed_rpm/96.0);
	PID_Task(&GM2SpeedPID,GWSpeedRef[1],Guide_Motor2_Measure.speed_rpm/96.0);

	if(GM1SpeedPID.output>-400 && GM1SpeedPID.output<400 ) GM1SpeedPID.output=0;
	if(GM2SpeedPID.output>-400 && GM2SpeedPID.output<400 ) GM2SpeedPID.output=0;
}



uint8_t gmkp = 50;

void GM_Get_PID(void)
{
	GM1SpeedPID.kp = GM2SpeedPID.kp = gmkp;
	GM1SpeedPID.ki = GM2SpeedPID.ki = 0;
	GM1SpeedPID.kd = GM2SpeedPID.kd = 0;

}




int8_t GuideWheelSpeedRef = 0;


extern int8_t flag_gcm;//导轮和底盘电机前进后退flag



void GM_Get_SpeedRef(void)
{
	switch (GuideWheelMode)
	{
		case GuideWheel_NormalRCMode:
		{
			GWSpeedRef[0] = RC_CtrlData.rc.ch1/2;
			GWSpeedRef[1] =  -RC_CtrlData.rc.ch1/2;
			
			
		}break;
		case GuideWheel_KeyMouseMode:
		{
			if(Remote_CheckJumpKey(KEY_F) == 1 && RC_CtrlData.mouse.press_l == 1)
			{
				GWSpeedRef[0] = 200;
				GWSpeedRef[1] = -200;
			}
			else if(Remote_CheckJumpKey(KEY_V) == 1 && RC_CtrlData.mouse.press_l == 1)
			{
				GWSpeedRef[0] = -200;
				GWSpeedRef[1] = 200;
			}
			else
			{
				GWSpeedRef[0] = 0;
				GWSpeedRef[1] = 0;
			}
			
		}break;
		
		
		case GuideWheel_Auto_UpIsland:
		{
			//上岛时导轮只需向前，等待的是抬升机构编码器差值,只需存下最初上岛编码器值，大于麦轮上岛编码器值时开启，小于一定值时关闭
			if(flag_gcm == 1)//导轮和底盘电机前进后退flag
			{
				GWSpeedRef[0] = 120;
				GWSpeedRef[1] = -120;
			}
			else if(flag_gcm == -1)
			{
				GWSpeedRef[0] = -120;
				GWSpeedRef[1] = 120;
			}
			else
			{
			
				GWSpeedRef[0] = 0;
				GWSpeedRef[1] = 0;
			}
			

			
		}break;

		case GuideWheel_Auto_DownIsland:
		{
			if(flag_gcm == 1)//导轮和底盘电机前进后退flag
			{
				GWSpeedRef[0] = 70;
				GWSpeedRef[1] = -70;
			}
			else if(flag_gcm == -1)
			{
				GWSpeedRef[0] = -70;
				GWSpeedRef[1] = 70;
			}
			else
			{
			
				GWSpeedRef[0] = 0;
				GWSpeedRef[1] = 0;
			}

		}break;

		case GuideWheel_Locked:
		{
			GWSpeedRef[0] = 0;
			GWSpeedRef[1] = 0;		
		}break;

		default:
		{
			GWSpeedRef[0] = 0;
			GWSpeedRef[1] = 0;		
		}break;
	}
	
	
}



void GM_Set_Current(void)
{
	if(WorkState == STOP_STATE||WorkState == PREPARE_STATE||OperateMode == Stop_Mode ||GuideWheelMode == GuideWheel_Locked)
	{
		CAN1_Send_GM(0,0);
	}
	else
		CAN1_Send_GM(GM1SpeedPID.output*1.5,GM2SpeedPID.output*1.5);

}



uint16_t ref_n = 400;
void GuideWheel_Control(void)
{
	GM_Get_SpeedRef();
	GM_Get_PID();
	GM_Calc_Output();
	GM_Set_Current();
	
}


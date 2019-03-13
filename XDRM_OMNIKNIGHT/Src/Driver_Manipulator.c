#include "Driver_Manipulator.h"
#include "CanBusTask.h"
#include "Driver_Remote.h"




#include "DriverLib_PID.h"



float ArmMotorAngleRef = 0;

void ARM_Get_PID(void)
{
	AMRotatePID.kp =60;
	AMRotatePID.ki =0;
	AMRotatePID.kd =0;

	
	

}
uint8_t AM_Stall_Flag = 0;
uint32_t	time_tick_arm = 0;
void ARM_Get_AngleRef(void)
{
	
	if(RC_CtrlData.rc.s2 == 1)
	{
		if(Action_stick == REMOTE_SWITCH_CHANGE_1TO3TO1TO3)//1-3-1-3为取弹一套//刚开始机械臂需要抬起来
		{
//			if(AM_Stall_Flag==0&&(xTaskGetTickCount() - time_tick_arm>3000)
//			{
//				ARM_PINCH;//等一下把他弄成加括号的
//				static uint16_t aaaab = 0;
//				aaaab++;
//				if(aaaab > 500)
//				{
//					ARM_RAISE;
//					ArmMotorAngleRef = TurntableEncoder.ecd_angle/19 - 120;
//					
//				}
//			
//			}
//			ArmMotorAngleRef = TurntableEncoder.ecd_angle/19 + 120;
//			time_tick_arm = xTaskGetTickCount();
			
		
		}
		else if(Action_stick == REMOTE_SWITCH_CHANGE_2TO3TO2TO3)//2-3-2-3为电机的来回翻转
		{		
			static int sw_count = 0;
			if(sw_count++%2 == 0)//这个也不太合适，最好加上转轴角度处理
			{
				ArmMotorAngleRef = TurntableEncoder.ecd_angle/19 - 120;//这个angle到时候统一改掉
				time_tick_arm = xTaskGetTickCount();
				AM_Stall_Flag = 1;
			}
			else 
			{
				ArmMotorAngleRef = TurntableEncoder.ecd_angle/19 + 120;
				time_tick_arm = xTaskGetTickCount();
				AM_Stall_Flag = 1;

			}
				Action_stick = 0;
				stick_store = RC_CtrlData.rc.s1;
		}
		else if(Action_stick == REMOTE_SWITCH_CHANGE_1TO3TO2TO3)//1-3-2-3为抬升气缸的上下移动
		{
		
		}
		else if(Action_stick == REMOTE_SWITCH_CHANGE_3TO2TO3TO2)//3-2-3-2为夹取气缸的夹和放
		{
		
		
		}
		
		
		if(xTaskGetTickCount() - time_tick_arm > 2000)//我这个用的是时间，其实用转轴角度似乎更合适
		{
			ArmMotorAngleRef = TurntableEncoder.ecd_angle/19;	//也就是等于0，也就是output为零
			AM_Stall_Flag = 0;
		}//防止堵转
	}

}
void ARM_Calc_Output(void)
{
	PID_Task(&AMRotatePID,ArmMotorAngleRef,TurntableEncoder.ecd_angle/19);//这里减速比
	if(AMRotatePID.output>-300 && AMRotatePID.output<300 ) AMRotatePID.output=0;
	

}

void ARM_Set_Current(void)
{
		if(RC_CtrlData.rc.s2 == 2)
		{
		 AMRotatePID.output = 0;
		}
		else
		{
		//这个发送在同步带的can发送里面顺便发掉
		}


}



void ARM_Gas_Operate(void)
{
		if(Action_stick == REMOTE_SWITCH_CHANGE_1TO3TO1TO3)//1-3-1-3为取弹一套
		{
		
		}
		else if(Action_stick == REMOTE_SWITCH_CHANGE_2TO3TO2TO3)//2-3-2-3为电机的来回翻转
		{
		
		
		}
		else if(Action_stick == REMOTE_SWITCH_CHANGE_1TO3TO2TO3)//1-3-2-3为抬升气缸的上下移动
		{
			if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_10)==RESET)
			{
				ARM_RAISE;
				
			}
			else
				ARM_DESCEND;

			Action_stick = 0;
			stick_store = RC_CtrlData.rc.s1;
		
		}
		else if(Action_stick == REMOTE_SWITCH_CHANGE_3TO2TO3TO2)//3-2-3-2为夹取气缸的夹和放
		{
		
			if(HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_9)==RESET)
			{
				ARM_PINCH;
			}
			else 
			{
				ARM_UNPINCH;
			}
				
			Action_stick = 0;
			stick_store = RC_CtrlData.rc.s1;

		}

	



}



void ARM_Electric_Operate(void)
{
	ARM_Get_PID();
	ARM_Get_AngleRef();
	ARM_Calc_Output();
	ARM_Set_Current();
}



void Manipulator_Control(void)
{

	ARM_Gas_Operate();
	ARM_Electric_Operate();

}








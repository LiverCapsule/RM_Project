#include "Driver_GuideWheel.h"
#include "BSP_TIM.h"
#include "Driver_Remote.h"
#include "Driver_Beltraise.h"
#include "Driver_Sensor.h"
#include "math.h"
extern uint32_t count1;
extern uint32_t count2;

extern uint8_t Foward_C_flag;
extern uint8_t Back_C_flag;
uint8_t Foward_G_flag = 0;
uint8_t Back_GW_flag = 0;
uint8_t GW_direction;
uint16_t GW_Speed;
GuideWheelModeTypeDef GuideWheelMode;
void MotorInit(void)
{
	MotorStart();//关闭刹车停机
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);//使能正向
}
void MotorBackInit(void)
{
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);//使能反向
}
extern uint8_t steps;
extern uint8_t steps_down;
extern uint8_t BM_AngelGet;
void MotorSpeedSet(void)
{
	if(RC_CtrlData.rc.s2 == 3)
	{
		MotorStart();
		if(RC_CtrlData.rc.ch1 > 0)
		{
			MotorInit();
		}
		else if(RC_CtrlData.rc.ch1 < 0)
		{
			MotorBackInit();
		}
		GW_Speed = 100-(abs(RC_CtrlData.rc.ch1)/7.33);//占空比10-100，0时导轮停止660/90=7.33
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,GW_Speed);
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,GW_Speed);
	}
		 
	if(Foward_G_flag == 1)
		{
			MotorStart();
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,50);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,50);
			if((count1+count2)>2000)//抬腿时全用导轮
			{//19*6*2*n圈 = 2*信号数*距离(单位信号 = 8000)
				count1 = count2 = 0;
				MotorStop();
				Foward_G_flag =0;
				steps++;
			}//一个车的距离
		}
			else if(Back_GW_flag == 1)
			{
				MotorStart();
				MotorBackInit();
				__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,50);
				__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,50);
				if(InfraredState_back == 1 && InfraredState_front == 1)//后方传感器检测到悬空
				{
					Back_GW_flag = 0;
					MotorStop();
					BM_AngelGet = 1;
					steps_down++;
				}
			}
			else
			{
				MotorStop();
			}
		
}

void MotorSpeedSet_SM(void)
{
	switch (GuideWheelMode)
	{
		case Normal_Rc_GuideWheelMove:
		{
			MotorStart();
			if(RC_CtrlData.rc.ch1 > 0)
			{
				MotorInit();
			}
			else if(RC_CtrlData.rc.ch1 < 0)
			{
				MotorBackInit();
			}
			GW_Speed = 100-(abs(RC_CtrlData.rc.ch1)/7.33);//占空比10-100，0时导轮停止660/90=7.33
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,GW_Speed);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,GW_Speed);
		}break;//当时遥控器遥控导轮的bug还没有改过来

		case Auto_Up_Island_GuideWheelMove:
		{
			MotorStart();
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,50);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,50);
			if((count1+count2)>2000)//抬腿时全用导轮
			{//19*6*2*n圈 = 2*信号数*距离(单位信号 = 8000)
				count1 = count2 = 0;
				MotorStop();
				GuideWheelMode = GuideWheelMove_Stop;
			}//一个车的距离
		}break;

		case Auto_Down_Island_GuideWheelMove:
		{
			MotorStart();
			MotorBackInit();
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,50);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,50);
			if(InfraredState_back == 1 && InfraredState_front == 1)//后方传感器检测到悬空
			{
				MotorStop();
				//BM_AngelGet = 1;
				GuideWheelMode = GuideWheelMove_Stop;
			}
		}break;

		case GuideWheelMove_Stop:
		{
			MotorStop();
		}break;

		default:
		{
			MotorStop();
		}break;
	}
}
void GuideWheel_Control(void)
{
	MotorSpeedSet();
}

	


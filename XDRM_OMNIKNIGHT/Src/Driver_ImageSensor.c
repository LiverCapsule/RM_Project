#include "Driver_ImageSensor.h"
#include "Driver_Remote.h"
#include "BSP_TIM.h"

#include "DriverLib_Func.h"
#include "Driver_Chassis.h"


extern WorkState_e 	WorkState;//先这样写了
extern OperateMode_e OperateMode;

uint32_t mod3 = 0;

//图传舵机和倒车雷达继电器

int32_t SteerAngleRef = 0;

uint8_t image_anglekey = 0;
uint8_t image_anglekeylast = 0;

void SteerAngleSet(void)
{

	if(OperateMode == KeyMouse_Mode)
	{
		image_anglekey = Remote_CheckJumpKey(KEY_C);
		if(image_anglekey != image_anglekeylast && image_anglekey == 1)
		{
			mod3++;
			
		}
		if(mod3%3 == 0)//看向前方为2020
		{
			TIM8->CCR3 = 2000;
			TIM8->CCR4 = 600;
		}
		else if(mod3%3 == 1)//看向倒车雷达为1700
		{
			
			TIM8->CCR3 = 1300;
			TIM8->CCR4 = 600;
			
		}	
		else if(mod3%3 == 2)//看向车后为750
		{
			TIM8->CCR3 = 2100;

			TIM8->CCR4 = 1740;
			
		}
		image_anglekeylast = image_anglekey;
	}
	else 
	{
		TIM8->CCR3 = 2000;
		TIM8->CCR4 = 600;
	}
	
	


}

void CameraRadar_Switch(void)
{
//	if()//倒车雷达视角右键切换
//	{
//		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
//	}
}


void ImageSensor_Control(void)//这个就基本不写自己单独的模式了，首先他几乎是一直工作的，其次不需要其他的工作模式
{
//	CameraRadar_Switch();
	SteerAngleSet();

}













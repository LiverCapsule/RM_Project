#include "Driver_ImageSensor.h"
#include "Driver_Remote.h"
#include "BSP_TIM.h"

#include "DriverLib_Func.h"


extern WorkState_e 	WorkState;//先这样写了
extern OperateMode_e OperateMode;



//图传舵机和倒车雷达继电器

int32_t SteerAngleRef = 0;


void SteerAngleSet(void)
{
	if(WorkState == STOP_STATE ||WorkState == PREPARE_STATE ||OperateMode == Stop_Mode)
	{
		SteerAngleRef = 0;
	}
	else if(OperateMode == NormalRC_Mode)
	{
//		Remote_CheckJumpKey(KEY_C);
	}
	else 
	{
		SteerAngleRef += RC_CtrlData.mouse.y;//这个和上面那个都
	}
	
	
	//600 -2500
	SteerAngleRef += 1200;//这个值是舵机的中间值
	VAL_LIMIT(SteerAngleRef,600,2500);
	TIM8->CCR2 = SteerAngleRef;

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













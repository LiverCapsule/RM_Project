#include "Driver_ImageSensor.h"
#include "Driver_Remote.h"
#include "BSP_TIM.h"

#include "DriverLib_Func.h"
#include "Driver_Chassis.h"


extern WorkState_e 	WorkState;//������д��
extern OperateMode_e OperateMode;

uint32_t mod3 = 0;

//ͼ������͵����״�̵���

int32_t SteerAngleRef = 0;

uint8_t image_anglekey = 0;
uint8_t image_anglekeylast = 0;

void SteerAngleSet(void)
{
//	if(WorkState == STOP_STATE ||WorkState == PREPARE_STATE ||OperateMode == Stop_Mode)
//	{
////		SteerAngleRef = 0;
//		TIM8->CCR4 = 
//	}
//	else if(OperateMode == NormalRC_Mode)
//	{
//SteerAngleRef = 0;
//	}
//	else 
//	{
//		SteerAngleRef += RC_CtrlData.mouse.y;//����������Ǹ���
//	}
	
	if(OperateMode == KeyMouse_Mode)
	{
		image_anglekey = Remote_CheckJumpKey(KEY_C);
		if(image_anglekey != image_anglekeylast && image_anglekey == 1)
		{
			mod3++;
			
		}
		if(mod3%3 == 0)//����ǰ��Ϊ2020
		{
			TIM8->CCR4 = 2020;
		}
		else if(mod3%3 == 1)//���򵹳��״�Ϊ1700
		{
			TIM8->CCR4 = 1700;
			
		}	
		else if(mod3%3 == 2)//���򳵺�Ϊ750
		{
			TIM8->CCR4 = 750;
			
		}
		image_anglekeylast = image_anglekey;
	}
	else 
	{
		TIM8->CCR4 = 2020;
	}
	
	


}

void CameraRadar_Switch(void)
{
//	if()//�����״��ӽ��Ҽ��л�
//	{
//		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
//	}
}


void ImageSensor_Control(void)//����ͻ�����д�Լ�������ģʽ�ˣ�������������һֱ�����ģ���β���Ҫ�����Ĺ���ģʽ
{
//	CameraRadar_Switch();
	SteerAngleSet();

}













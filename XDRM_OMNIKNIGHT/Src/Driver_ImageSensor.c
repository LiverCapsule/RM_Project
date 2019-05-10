#include "Driver_ImageSensor.h"
#include "Driver_Remote.h"
#include "BSP_TIM.h"

#include "DriverLib_Func.h"


extern WorkState_e 	WorkState;//������д��
extern OperateMode_e OperateMode;



//ͼ������͵����״�̵���

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
		SteerAngleRef += RC_CtrlData.mouse.y;//����������Ǹ���
	}
	
	
	//600 -2500
	SteerAngleRef += 1200;//���ֵ�Ƕ�����м�ֵ
	VAL_LIMIT(SteerAngleRef,600,2500);
	TIM8->CCR2 = SteerAngleRef;

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













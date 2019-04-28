#ifndef __CONFIG_H
#define __CONFIG_H


#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "stm32f4xx_hal.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "stdint.h"
#include <stdlib.h>
#define M_PI 3.1415927

typedef enum
{
    PREPARE_STATE,          //�ϵ���ʼ״̬
    NORMAL_RC_STATE,        //ң��������״̬
    KEYBOARD_RC_STATE,      //���̿���״̬
    STOP_STATE,             //ֹͣ״̬
}WorkState_e;



typedef enum
{
    Stop_Mode,
    NormalRC_Mode,//RC--Remote Control
    KeyMouse_Mode,
    Auto_Mode,//�Զ�ģʽָ�����Զ�����ĳһ�˶�,���ϵ�,ȡ����
}OperateMode_e;

typedef enum
{
	Auto_NoMovement,
	Auto_Get_Egg,//������һ��
	Auto_Get_Eggs,
	Auto_Pull_Eggs,//����ȡ�����ֲ��ԣ�һ�����Ȱ�ȫ����������һ������������ȡ
	Auto_Get_I_Egg,	
	Auto_Get_I_Eggs,
	Auto_Give_Egg,
	Auto_Up_Island,
	Auto_Down_Island,
	Auto_Up_Step,
	Auto_Cali_For_Egg,
}AutoMovement_e;

extern WorkState_e 	WorkState;
extern OperateMode_e OperateMode;

/*
* @ breif config document
*/

//�Ƿ�ѽṹ�嶨��������ﻹ��Ҫ����
//����Ϊ�����У��ѽṹ��д�ڶ�Ӧ��bsp.h����

#define NEW_BELIEF //��������



#endif



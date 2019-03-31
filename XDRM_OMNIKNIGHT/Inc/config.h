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

//���Ǿ��ò�����driver������include task��
//����������������������Ķ���
typedef enum
{
    Stop_Mode,//����ң�������»��Զ��˳�ֹͣģʽ
		Test_Mode,//
    Auto_Mode,//�Զ�ģʽ
		Manual_Mode,//�ֶ�ģʽ�����еĻ���driverд����
}OperateMode_e;


typedef enum
{
	REMOTE_INPUT = 3,
	KEY_MOUSE_INPUT = 1,
	STOP =2,
}InputMode_e;//����ĳ�remoteΪ3

typedef enum
{
    PREPARE_STATE,          //�ϵ���ʼ״̬
    NORMAL_RC_STATE,        //ң��������״̬
    KEY_MOUSE_STATE,      	//���̿���״̬
    STOP_STATE,             //ֹͣ״̬
}WorkState_e;


#define PREPARE_TIME_TICK_MS 4000






/*
* @ breif config document
*/

//�Ƿ�ѽṹ�嶨��������ﻹ��Ҫ����
//����Ϊ�����У��ѽṹ��д�ڶ�Ӧ��bsp.h����

#define NEW_BELIEF //��������



#endif



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
    PREPARE_STATE,          //上电后初始状态
    NORMAL_RC_STATE,        //遥控器控制状态
    KEYBOARD_RC_STATE,      //键盘控制状态
    STOP_STATE,             //停止状态
}WorkState_e;



typedef enum
{
    Stop_Mode,
    NormalRC_Mode,//RC--Remote Control
    KeyMouse_Mode,
    Auto_Mode,//自动模式指的是自动进行某一运动,如上岛,取弹等
}OperateMode_e;

typedef enum
{
	Auto_NoMovement,
	Auto_Get_Egg,//下面是一类
	Auto_Get_Eggs,
	Auto_Pull_Eggs,//岛上取弹两种策略，一种是先把全部拉回来，一种是拉回来就取
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

//是否把结构体定义放在这里还需要考虑
//我认为不可行，把结构体写在对应的bsp.h里面

#define NEW_BELIEF //新信仰板



#endif



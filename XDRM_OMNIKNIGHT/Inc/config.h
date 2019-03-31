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

//就是觉得不能在driver层里面include task层
//而且这个东西就是整个车的东西
typedef enum
{
    Stop_Mode,//在有遥控输入下会自动退出停止模式
		Test_Mode,//
    Auto_Mode,//自动模式
		Manual_Mode,//手动模式把所有的基本driver写出来
}OperateMode_e;


typedef enum
{
	REMOTE_INPUT = 3,
	KEY_MOUSE_INPUT = 1,
	STOP =2,
}InputMode_e;//这里改成remote为3

typedef enum
{
    PREPARE_STATE,          //上电后初始状态
    NORMAL_RC_STATE,        //遥控器控制状态
    KEY_MOUSE_STATE,      	//键盘控制状态
    STOP_STATE,             //停止状态
}WorkState_e;


#define PREPARE_TIME_TICK_MS 4000






/*
* @ breif config document
*/

//是否把结构体定义放在这里还需要考虑
//我认为不可行，把结构体写在对应的bsp.h里面

#define NEW_BELIEF //新信仰板



#endif



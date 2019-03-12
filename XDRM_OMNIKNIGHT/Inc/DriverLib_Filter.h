#ifndef __DRIVERLIB_FILTER_H
#define __DRIVERLIB_FILTER_H


#include "config.h"


typedef __packed struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num;       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;


#endif




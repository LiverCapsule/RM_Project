#include "DriverLib_Func.h"




//���ú��� 
//���η�ת����
//uint8_t EdgeDetect(int32_t value_n,int32_t value_l)
//{
//	
//                                                                       
//}






//ѭ���޷�����
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}



#ifndef __DRIVERLIB_FUNC_H
#define __DRIVERLIB_FUNC_H

#include "config.h"






//�޷�����
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\


#endif



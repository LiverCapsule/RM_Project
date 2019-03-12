/* Includes ------------------------------------------------------------------*/
#include "DriverLib_Ramp.h"



RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   //mouse×óÓÒÒÆ¶¯Ğ±ÆÂ
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;   //mouseÇ°ºóÒÆ¶¯Ğ±ÆÂ
RampGen_t MBSpeedRamp = RAMP_GEN_DAFAULT;   //mouseÃ¨²½ÒÆ¶¯Ğ±ÆÂ 

RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t GMYawRamp = RAMP_GEN_DAFAULT;
void RampInit(RampGen_t *ramp, int32_t XSCALE)
{
}

float RampCalc(RampGen_t *ramp)
{
	ramp->count++;
	if(ramp->count < ramp->XSCALE)
	{
	 return (float) ramp->count / ramp->XSCALE;
	}
	else
	{
		return 1.0f;
	}
}

void RampSetCounter(struct RampGen_t *ramp, int32_t count)
{
	ramp->count=count;
}

void RampResetCounter(struct RampGen_t *ramp)
{
	ramp->count=0;
}

void RampSetScale(struct RampGen_t *ramp, int32_t scale)
{
	ramp->XSCALE=scale;
}

uint8_t RampIsOverflow(struct RampGen_t *ramp)
{
	if(ramp->count < ramp->XSCALE) return 0;
	else return 1;
}

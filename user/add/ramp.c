#include "ramp.h"

__weak uint32_t Get_TimerTick() {
    return HAL_GetTick();
}

float Slope(Ramp_Typedef *Ramp) {
    if (!Ramp->flag) {
        Ramp->StartTick = Get_TimerTick();
        Ramp->flag = 1;
    }
    if (Get_TimerTick() > (Ramp->StartTick + Ramp->RampTime))return 1.0f;
    return ((Get_TimerTick() - Ramp->StartTick) / (float) Ramp->RampTime);
}


void ResetSlope(Ramp_Typedef *Ramp) {
    Ramp->flag = 0;
}

int16_t Ramp_Step(int16_t Expected, int16_t Current, int16_t Step)
{
    float buffer = Expected - Current;
	if (buffer > 0)
	{
		if (buffer > Step)  
			Current += Step;  
		else
			Current += buffer;
	}		
	else
	{
		if (buffer < -Step)
			Current += -Step;
		else
			Current += buffer;
	}
	return Current;
}

float fRamp_Stepf(float Expected, float Current, float Step)
{
    float buffer = Expected - Current;
	if (buffer > 0)
	{
		if (buffer > Step)  
			Current += Step;  
		else
			Current += buffer;
	}		
	else
	{
		if (buffer < -Step)
			Current += -Step;
		else
			Current += buffer;
	}
	return Current;
}



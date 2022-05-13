#include "PID.h"

void pid_init(struct pid *pid)
{
    pid->aim = 0;
    pid->prev_error = 0;
    pid->prev_value = 0;
    pid->differentiator = 0;
    pid->integrator = 0;
}

float pid_compute(struct pid *pid, float value)
{
    float e;
    float proportional;
    float out;

    //error computation
    e = pid->aim - value;

    //proportional
    proportional = pid->Kp * e;

    //integrator
    pid->integrator = 0.5f * pid->Ki * pid->T * (e + pid->prev_error) + pid->integrator;

    float limMinInt=0,limMaxInt=0;

    /* Compute integrator limits */
	if(pid->limMax > proportional)
    {
		limMaxInt = pid->limMax - proportional;
	}
    else
    {
		limMaxInt = 0.0f;
	}

	if(pid->limMin < proportional)
    {
		limMinInt = pid->limMin - proportional;
	}
    else 
    {
		limMinInt = 0.0f;
	}

    /* Clamp integrator */
    if(pid->integrator > limMaxInt)
    {
        pid->integrator = limMaxInt;
    }
    else if(pid->integrator < limMinInt)
    {
        pid->integrator = limMinInt;
    }

    /* Note: derivative on measurement, therefore minus sign in front of equation! */
    pid->differentiator = -(2.0f * pid->Kd * (value - pid->prev_value)	
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);

    out = proportional + pid->integrator + pid->differentiator;

    pid->prev_value = value;
    pid->prev_error = e;

    if(out > pid->limMax)
    {
        out = pid->limMax;
    }
    else if(out < pid->limMin)
    {
        out = pid->limMin;
    }

    return out;
}


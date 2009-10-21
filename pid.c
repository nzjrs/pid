/*
 * pid.c
 *
 *  Created on: Sep 11, 2009
 *      Author: pixhawk
 */

#include <stdio.h>
#include "pid.h"

#ifndef ABS
#define ABS(a)	   (((a) < 0) ? -(a) : (a))
#endif

#define WINDUP_ON(_pid)         (_pid->features & PID_ENABLE_WINDUP)
#define DEBUG_ON(_pid)          (_pid->features & PID_DEBUG)
#define SAT_MIN_ON(_pid)        (_pid->features & PID_OUTPUT_SAT_MIN)
#define SAT_MAX_ON(_pid)        (_pid->features & PID_OUTPUT_SAT_MAX)
#define HIST_ON(_pid)           (_pid->features & PID_INPUT_HIST)

void pid_enable_feature(PID_t *pid, unsigned int feature, float value)
{
    pid->features |= feature;

    switch (feature) {
        case PID_ENABLE_WINDUP:
            /* integral windup is in absolute output units, so scale to input units */
            pid->intmax = ABS(value / pid->ki);
            break;
        case PID_DEBUG:
            break;
        case PID_OUTPUT_SAT_MIN:
            pid->sat_min = value;
            break;
        case PID_OUTPUT_SAT_MAX:
            pid->sat_max = value;
            break;
        case PID_INPUT_HIST:
            break;
    }
}

/**
 *
 * @param pid
 * @param kp
 * @param ki
 * @param kd
 */
void pid_init(PID_t *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->sp = 0;
	pid->error_previous = 0;
	pid->integral = 0;

    pid->features = 0;

    if (DEBUG_ON(pid))
        printf("P,I,D,error,i_total,int_windup\n");
}

void pid_set(PID_t *pid, float sp)
{
	pid->sp = sp;
	pid->error_previous = 0;
	pid->integral = 0;
}

/**
 *
 * @param pid
 * @param val
 * @param dt
 * @return
 */
float pid_calculate(PID_t *pid, float val, float dt)
{
	float i,d, error, total;

	error = pid->sp - val;
	i = pid->integral + (error * dt);
	d = (error - pid->error_previous) / dt;

    total = (error * pid->kp) + (i * pid->ki) + (d * pid->kd);

    if (DEBUG_ON(pid))
        printf("%.1f,%.1f,%.1f,%.1f,%.1f,%d\n", 
                (error * pid->kp), (i * pid->ki), (d * pid->kd),
                error, pid->integral, ABS(i) == pid->intmax);

    if ( WINDUP_ON(pid) ) {
        printf("\t%.1f v %.1f\n", i, pid->intmax);
        if ( i < 0 )
            i = ( i < -pid->intmax ? -pid->intmax : i );
        else
   		    i = ( i < pid->intmax ? i : pid->intmax );
    }
    pid->integral = i;

    if ( SAT_MIN_ON(pid) && (total < pid->sat_min) )
        return pid->sat_min;
    if ( SAT_MAX_ON(pid) && (total > pid->sat_max) )
        return pid->sat_max;

	pid->error_previous = error;
	return total;
}

/*
 * pid.h
 *
 *  Created on: Sep 11, 2009
 *      Author: pixhawk
 */

#ifndef PID_H_
#define PID_H_

typedef struct {
		float kp;
		float ki;
		float kd;

		float sp;
		float integral;
		float error_previous;

        unsigned int features;
		float intmax;
        float sat_max;
        float sat_min;
} PID_t;

typedef enum {
    PID_ENABLE_WINDUP   =   (1<<0),
    PID_DEBUG           =   (1<<1),
    PID_OUTPUT_SAT_MIN  =   (1<<2),
    PID_OUTPUT_SAT_MAX  =   (1<<3),
    PID_INPUT_HIST      =   (1<<4)
} PIDFeatures_t;

void    pid_init                                    (PID_t          *pid, 
                                                     float          kp, 
                                                     float          ki, 
                                                     float          kd);
void    pid_enable_feature                          (PID_t          *pid, 
                                                     unsigned int   feature,
                                                     float          value);
void    pid_set                                     (PID_t          *pid,
                                                     float          sp);
float   pid_calculate                               (PID_t          *pid,
                                                     float          val,
                                                     float          dt);

#endif /* PID_H_ */

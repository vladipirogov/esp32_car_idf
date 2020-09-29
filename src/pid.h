#ifndef COMPONENTS_INCLUDE_PID_H_
#define COMPONENTS_INCLUDE_PID_H_

#include <stdint.h>
#include "stdbool.h"
#include <esp_timer.h>
#include <stdio.h>

#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1
#define REFERENCE "ref"
#define PARAM "param"
#define MODE "mode"

	 typedef struct {
	 	 float kp;
	 	 float ki;
	 	 float kd;
	 	 float min_limit;
	 	 float max_limit;
	 	 unsigned long sample_time;
	 	 int direction;
	 	 int pOn;
	 	} Parameters;

	 typedef struct {
	 	float input;
	 	float output;
	 	float setpoint;
	    unsigned long last_time;
	    float output_sum, last_error;
	    bool in_auto;
	 } Reference;


    void init_pid(Reference * reference, const Parameters *source);

    void pid_set_mode(Reference *reference, Parameters *parameters, int Mode);

    float pid_compute(float input, Reference *reference, const Parameters *parameters);

    void pid_set_output_limits(uint32_t, uint32_t);

    void pid_set_tunings(Parameters *parameters, float, float, float);

    void pid_set_sample_time(Parameters *parameters, int);

    void pid_setpoint(Reference *reference, float* setpoint);

    int pid_get_mode(void);



#endif /* COMPONENTS_INCLUDE_PID_H_ */

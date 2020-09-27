/*
 * pid.c
 *
 *  Created on: 8 ����. 2020 �.
 *      Author: admin
 */
#include "pid.h"

void init_pid(Reference *reference, const Parameters *parameters) {
    reference->in_auto = false;
    reference->last_time = esp_timer_get_time()/1000 - parameters->sample_time;
}


bool pid_compute(Reference *reference, const Parameters *parameters) {
   if(!reference->in_auto) {
	   printf("Pid setpoint = %f\n", reference->setpoint.float_val);
	   reference->output = reference->setpoint.float_val;
	   return false;
   }

   unsigned long now = esp_timer_get_time()/1000;
   unsigned long time_change = (now - reference->last_time);

   if(time_change>=parameters->sample_time) {

      float input = reference->input;
      float error = reference->setpoint.float_val - input;
      float dInput = (error - reference->last_error)/time_change;
      reference->output_sum+= (parameters->ki.float_val * error);
      if(reference->output_sum > parameters->max_limit) reference->output_sum= parameters->max_limit;
      else if(reference->output_sum < parameters->min_limit) reference->output_sum= parameters->min_limit;

      printf("Pid setpoint = %f\n", reference->setpoint.float_val);
      printf("Input: %f\n", input);
      printf("Error: %f\n", error);
      printf("Diff input = %f\n", dInput);
      printf("Integral sum = %f\n", reference->output_sum);
      printf("parameters: KP =  %f, KI = %f, KD = %f\n", parameters->kp.float_val, parameters->ki.float_val, parameters->kd.float_val);
      printf("max limit = %f, min limit = %f\n", parameters->max_limit, parameters->min_limit);
      printf("PID mode is %d\n", reference->in_auto);

	  float output = parameters->kp.float_val*error + reference->output_sum + parameters->kd.float_val*dInput;

	  if(output >  parameters->max_limit) output =  parameters->max_limit;
      else if(output < parameters->min_limit) output = parameters->min_limit;
	    reference->output = output;

		printf("Output: %f\n", reference->output);
		printf("------------------");
		printf("------------------\n");

      reference->last_error = error;
      reference->last_time = now;
	    return true;
   }
   else return false;
}




void pid_set_sample_time(Parameters *parameters, int new_sample_time) {
   if (new_sample_time > 0)
   {
      float ratio  = (float)new_sample_time
                      / (float)parameters->sample_time;
      parameters->ki.float_val *= ratio;
      parameters->kd.float_val /= ratio;
      parameters->sample_time = (unsigned long)new_sample_time;
   }
}


void initialize(Reference *reference, Parameters *parameters) {
   reference->output_sum = reference->output;
   //last_input = reference->input;
   if(reference->output_sum > parameters->max_limit) reference->output_sum = parameters->max_limit;
   else if(reference->output_sum < parameters->min_limit) reference->output_sum = parameters->min_limit;
}


void pid_set_mode(Reference *reference, Parameters *parameters, int mode) {
    bool in_auto = (mode == AUTOMATIC);
    if(in_auto && !in_auto) {
        initialize(reference, parameters);
    }
    in_auto = in_auto;
}


void pid_setpoint(Reference *reference, float* setpoint) {
	reference->setpoint.float_val = *setpoint;
}
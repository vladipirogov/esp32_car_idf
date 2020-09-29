#include "pid.h"

void init_pid(Reference *reference, const Parameters *parameters) {
    reference->in_auto = false;
    reference->last_time = esp_timer_get_time()/1000 - parameters->sample_time;
}


float pid_compute(float input, Reference *reference, const Parameters *parameters) {
   reference->input = input;
   if(!reference->in_auto) {
	   reference->output = reference->setpoint;
	   return reference->setpoint;
   }

   unsigned long now = esp_timer_get_time()/1000;
   unsigned long time_change = (now - reference->last_time);

   if(time_change>=parameters->sample_time) {

      float input = reference->input;
      float error = reference->setpoint - input;
      float dInput = (error - reference->last_error) / time_change;
      reference->output_sum += (parameters->ki * error);
      if (reference->output_sum > parameters->max_limit)
         reference->output_sum = parameters->max_limit;
      else if (reference->output_sum < parameters->min_limit)
         reference->output_sum = parameters->min_limit;

      float output = parameters->kp * error + reference->output_sum + parameters->kd * dInput;

      if (output > parameters->max_limit)
         output = parameters->max_limit;
      else if (output < parameters->min_limit)
         output = parameters->min_limit;
      reference->output = output;

      reference->last_error = error;
      reference->last_time = now;
      return output;
   }
   else return 0.0;
}




void pid_set_sample_time(Parameters *parameters, int new_sample_time) {
   if (new_sample_time > 0)
   {
      float ratio  = (float)new_sample_time
                      / (float)parameters->sample_time;
      parameters->ki *= ratio;
      parameters->kd /= ratio;
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
	reference->setpoint = *setpoint;
}
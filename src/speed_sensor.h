#ifndef INCLUDE_SPEEDSENSOR_H_
#define INCLUDE_SPEEDSENSOR_H_

#include "common.h"
#include "driver/pcnt.h"

#define PULSE_PER_TURN 20

typedef struct {
      uint16_t delay; //delay im ms
      int pin;
      int ctrl_pin;
      pcnt_channel_t channel;
      pcnt_unit_t unit;
      int16_t count;
} speed_sensor_params_t;

esp_err_t init_speed_sensor(speed_sensor_params_t* params);
int32_t calculateRpm(speed_sensor_params_t* params);

#endif
#ifndef MAIN_INCLUDE_CAR_H_
#define MAIN_INCLUDE_CAR_H_

#include "common.h"
#include "motor.h"
#include "string.h"

#define DUTY_BIAS 5
#define SPEED_TRS 95
#define CAR_FORWARD "esp32/car/forward"
#define CAR_BACKWARD "esp32/car/backward"
#define CAR_LEFT "esp32/car/left"
#define CAR_RIGHT "esp32/car/right"
#define CAR_STOP "esp32/car/stop"
#define CAR_SPEED "esp32/car/speed"

void init_car();
/**@brief handling car */
void handle_car(const command_t *command);
void handle_sensor(const sensor_data_t *data);

typedef struct
{
    int32_t speed;
    int32_t optical_1;
    int32_t optical_2;
    int32_t optical_diff;
} motor_t;

typedef enum {
    CAR_STOPPED = 0,  
    CAR_FORW_RUN,      
    CAR_BACK_RUN,    
} car_status_t;


void forward(int32_t value, int32_t * speed);
void backward(int32_t value, int32_t * speed);
void stop(int32_t value, int32_t * speed);
void turn_left(int32_t value);
void turn_right(int32_t value);
void obstacle_control(int32_t value);
void change_forw_speed(int32_t value);

#endif
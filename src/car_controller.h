#ifndef MAIN_INCLUDE_CAR_H_
#define MAIN_INCLUDE_CAR_H_

#include "common.h"
#include "motor.h"
#include "string.h"

#define DUTY_BIAS 15
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
    uint32_t speed;
} motor_t;

motor_t motor;

typedef enum {
    CAR_STOPPED = 0,  
    CAR_FORW_RUN,      
    CAR_BACK_RUN,    
} car_status_t;


void forward(uint32_t value, uint32_t * speed);
void backward(uint32_t value, uint32_t * speed);
void stop(uint32_t value, uint32_t * speed);
void turn_left(uint32_t value);
void turn_right(uint32_t value);
void obstacle_control(int32_t value);
void change_forw_speed(uint32_t value);

#endif
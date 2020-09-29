#include "car_controller.h"
#include "freertos/task.h"
#include "pid.h"


static car_status_t car_status; 
static Reference reference_1;
static Reference reference_2;
static Parameters parameters_1;
static Parameters parameters_2;
static motor_t motor;

void init_car() {
    //1. mcpwm initialization
  mcpwm_example_gpio_initialize();
  car_status = CAR_STOPPED;
    parameters_1.min_limit = 0.0; parameters_1.max_limit = 250; parameters_1.sample_time = 100; parameters_1.direction = DIRECT;
    parameters_2.min_limit = 0.0; parameters_2.max_limit = 250; parameters_2.sample_time = 100; parameters_2.direction = DIRECT;
    parameters_1.kp = 2; parameters_1.ki = 0.01; parameters_1.kd = 0.01;
    parameters_2.kp = 2; parameters_2.ki = 0.01; parameters_2.kd = 0.01;
  init_pid(&reference_1, &parameters_1);
  init_pid(&reference_2, &parameters_2);
}

void handle_car(const command_t *command) {
    int32_t value = atoi(command->message);
    printf("car topic: %s\n", command->topic);
    printf("car value = %d\n", value);
    
    if (!strcmp(command->topic, CAR_FORWARD)) forward(value, &motor.speed);
    else if (!strcmp(command->topic, CAR_BACKWARD)) backward(value, &motor.speed);
    else if (!strcmp(command->topic, CAR_STOP)) stop(value, &motor.speed);
    else if (!strcmp(command->topic, CAR_LEFT)) turn_left(value);
    else if (!strcmp(command->topic, CAR_RIGHT)) turn_right(value);
    else if (!strcmp(command->topic, CAR_SPEED)) change_forw_speed(value);
}

/** */
void handle_sensor(const sensor_data_t *data) {
    float output = 0;
  if(car_status == CAR_FORW_RUN) {
    switch (data->sensor) {
    case ULTRASONIC_SENSOR:
        obstacle_control(data->value);
      break;

    case OPTICAL_SENSOR_1: 
	    output = pid_compute(data->value, &reference_1, &parameters_1);
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, output);
        motor.speed = output;
      break;

    case OPTICAL_SENSOR_2: 
	    output = pid_compute(data->value, &reference_2, &parameters_2);
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, output);
        motor.speed = output;
        break;
    
    case MAX_SENSORS:
        break;
    }
 }
}

/** */
void change_forw_speed(int32_t value) {
    if(car_status == CAR_FORW_RUN) {
        printf("Speed changing\n");
        printf("value is: %d \n", value);
        reference_1.setpoint = value;
        reference_2.setpoint = value;
    }
}

/** */
void forward(int32_t value, int32_t * speed) {
    printf("Car forward\n");
    car_status = CAR_FORW_RUN;
    printf("value is: %d \n", value);
    reference_1.setpoint = value < SPEED_TRS ? value + DUTY_BIAS : value;
    reference_2.setpoint = value;
    //brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, value);
    //brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, value + DUTY_BIAS);
    *speed = value;
}

/** */
void backward(int32_t value, int32_t * speed) {
    printf("Car backward: ");
    car_status = CAR_BACK_RUN;
    printf("value is: %d \n", value);
    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, value);
    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, value);
    *speed = 0;
    vTaskDelay(300 / portTICK_RATE_MS);
    stop(0, &motor.speed);
}

/** */
void stop(int32_t value, int32_t * speed) {
    printf("Car stop \n");
    car_status = CAR_STOPPED;
    reference_1.setpoint = 0;
    reference_2.setpoint = 0;
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
    *speed = 0;

}

/** */
void turn_left(int32_t value) {
    printf("Car left \n");
    printf("value is: %d \n", value);
    int32_t speed_b = motor.speed + value;
    if (speed_b > 255)
    {
        speed_b = 255;
    }
    if(car_status == CAR_FORW_RUN)
        reference_1.setpoint = speed_b;
    else
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, speed_b); //
    vTaskDelay(100 / portTICK_RATE_MS);
    if(car_status == CAR_FORW_RUN)
        reference_1.setpoint = value;
    else
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, motor.speed);
}

/** */
void turn_right(int32_t value) {
    printf("Car right \n");
    printf("value is: %d \n", value);
    int32_t speed_b = motor.speed + value;

    if (speed_b > 255)
    {
        speed_b = 255;
    }
    if(car_status == CAR_FORW_RUN)
        reference_2.setpoint = speed_b;
    else
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, speed_b); //
    vTaskDelay(100 / portTICK_RATE_MS);
    if(car_status == CAR_FORW_RUN)
        reference_2.setpoint = value;
    else
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, motor.speed);
}

/** */
void obstacle_control(int32_t value) {
    switch (car_status) {
        case CAR_STOPPED:
         //
         break;
        case CAR_BACK_RUN:
            //
        break;

        case CAR_FORW_RUN:
        if(value < OBSTACLE_DISTANCE){
            stop(0, &motor.speed);
        }
        break;
    }
}
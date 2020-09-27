#ifndef MAIN_INCLUDE_COMMON_H_
#define MAIN_INCLUDE_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "driver/pcnt.h"

#define SERVO_CAM_HOR_PIN GPIO_NUM_15
#define SERVO_CAM_VER_PIN GPIO_NUM_2
#define FLASH_PIN GPIO_NUM_21
#define GPIO_PWM0A_OUT GPIO_NUM_16   
#define GPIO_PWM0B_OUT GPIO_NUM_17   
#define GPIO_PWM1A_OUT GPIO_NUM_5   
#define GPIO_PWM1B_OUT GPIO_NUM_18   
#define GPIO_SYNC0_IN GPIO_NUM_26 
#define ENCODER_1_PIN GPIO_NUM_34
#define WAKEUP_PIN GPIO_NUM_13
#define TRIGGER_PIN GPIO_NUM_33
#define ECHO_PIN GPIO_NUM_26
#define ENCODER_2_PIN GPIO_NUM_35

#define ULTRASONIC "esp32/car/ultrasonic"
#define STATUS_TOPIC "esp32/status"
#define ULTRASONIC_MQTT_TOPIC "esp32/status/ultrasonic"
#define OPTICAL_1_MQTT_TOPIC "esp32/status/optical_1"
#define OPTICAL_2_MQTT_TOPIC "esp32/status/optical_2"
#define OBSTACLE_DISTANCE 20
#define NUMBER_OF_TOPICS_TO_SEND 3
#define CAMERA_HANDLE "camera"
#define CAR_HANDLE "car"

// Mqtt command struct
typedef struct {
  char* topic;
  char* message;
} command_t;

typedef enum {
  ULTRASONIC_SENSOR = 0,
  OPTICAL_SENSOR_1,
  OPTICAL_SENSOR_2,
  MAX_SENSORS
} sensor_t;

typedef struct {
  sensor_t sensor;
  int32_t value;
} sensor_data_t;


#ifdef __cplusplus
}
#endif

#endif 
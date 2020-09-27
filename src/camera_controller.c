#include "camera_controller.h"


void init_camera() {
  gpio_set_direction(FLASH_PIN, GPIO_MODE_OUTPUT);
  servo_init();
}

static void horisontal(const command_t *command) {
  printf("Camera horisontal \n");
  int32_t value = atoi(command->message);
  printf("value is %d: \n", value);
  servo_control_horisontal(value);
}

static void vertical(const command_t *command) {
  printf("Camera vertical\n");
  int32_t value = atoi(command->message);
  printf("value is %d: \n", value);
  servo_control_vertical(value);
}

static void flash(const command_t *command) {
  printf("Camera flash\n");
  int32_t value = atoi(command->message);
  printf("value is %d: \n", value);
  if (value == 0)
    gpio_set_level(FLASH_PIN, 0);
  if (value > 0)
    gpio_set_level(FLASH_PIN, 1);
}

/**
 * @brief handling camera
 */
void handle_camera(const command_t *command) {
    if(!strcmp(command->topic, CAMERA_HORISONTAL)) horisontal(command);
    else if(!strcmp(command->topic, CAMERA_VERTICAL)) vertical(command);
    else if(!strcmp(command->topic, CAMERA_FLASH)) flash(command);
}
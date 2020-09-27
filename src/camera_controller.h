#ifndef MAIN_INCLUDE_CAMERA_H_
#define MAIN_INCLUDE_CAMERA_H_

#include "common.h"
#include "string.h"
#include "servo_control.h"

#define CAMERA_HORISONTAL "esp32/camera/horisontal"
#define CAMERA_VERTICAL "esp32/camera/vertical"
#define CAMERA_FLASH "esp32/camera/flash"

void init_camera();
/** @brief handling camera*/
void handle_camera(const command_t *command);


#endif
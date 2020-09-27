/*
 * ServoService.h
 *
 *  Created on: Feb 18, 2020
 *      Author: Vladyslav_Pyrohov
 */

#ifndef MAIN_INCLUDE_SERVOCONTROL_H_
#define MAIN_INCLUDE_SERVOCONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_system.h"

void servo_init(void);

void servo_control_horisontal(uint32_t reference);
void servo_control_vertical(uint32_t reference);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_INCLUDE_SERVOSERVICE_H_ */

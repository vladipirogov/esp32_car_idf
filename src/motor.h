#ifndef MAIN_INCLUDE_MOTOR_H_
#define MAIN_INCLUDE_MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/mcpwm.h"
#include "mcpwm_struct.h"
#include "common.h"


void mcpwm_example_gpio_initialize(void);

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle);

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle);

/**
 * @brief motor stop
 */
void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num);

#ifdef __cplusplus
}
#endif

#endif 
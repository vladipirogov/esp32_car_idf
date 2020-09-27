#include "servo_control.h"
#include <stdio.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "common.h"

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 microsecond
#define SERVO_MIN_PULSEWIDTH 544 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2400 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate


/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
uint32_t servo_per_degree_init(uint32_t degree_of_rotation) {
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}


void servo_init(void) {
	//1. mcpwm gpio initialization
	printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, SERVO_CAM_VER_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, SERVO_CAM_HOR_PIN);

	//2. initial mcpwm configuration
	printf("Configuring Initial Parameters of mcpwm......\n");
	mcpwm_config_t pwm_config;
			pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
			pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
			pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
			pwm_config.counter_mode = MCPWM_UP_COUNTER;
			pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

	mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
	mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
}

/** */
static void servo_control(uint32_t reference, mcpwm_timer_t timer) {
	if(reference > SERVO_MAX_DEGREE)
				reference = SERVO_MAX_DEGREE;
	//printf("Angle of rotation: %d\n", reference);
	uint32_t angle = servo_per_degree_init(reference);
	//printf("pulse width: %dus\n", angle);
	mcpwm_set_duty_in_us(MCPWM_UNIT_1, timer, MCPWM_OPR_A, angle);
}

/** */
void servo_control_vertical(uint32_t reference) {
    servo_control(reference, MCPWM_TIMER_0);
}

/** */
void servo_control_horisontal(uint32_t reference) {
     servo_control(reference, MCPWM_TIMER_1);
}
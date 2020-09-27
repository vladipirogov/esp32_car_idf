#include "speed_sensor.h"


esp_err_t init_speed_sensor(speed_sensor_params_t* params) {
      /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config;
    // Set PCNT input signal and control GPIOs
    pcnt_config.pulse_gpio_num = params->pin;
    pcnt_config.ctrl_gpio_num = params->ctrl_pin;
    pcnt_config.channel = params->channel;
    pcnt_config.unit = params->unit;
    // What to do on the positive / negative edge of pulse input?
    pcnt_config.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
    pcnt_config.neg_mode = PCNT_COUNT_DIS;   // Keep the counter value on the negative edge
    pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; // Reverse counting direction if low
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
    pcnt_config.counter_h_lim = INT16_MAX;
    pcnt_config.counter_l_lim = - INT16_MAX;

     /* Initialize PCNT unit */
    esp_err_t err = pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(params->unit, 100);
    pcnt_filter_enable(params->unit);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(params->unit);
    pcnt_counter_clear(params->unit);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(params->unit);
    return err;
}


int32_t calculateRpm(speed_sensor_params_t* params) {
    pcnt_get_counter_value(params->unit, &(params->count));
    int32_t rpm = 60*(1000/params->delay)*params->count/PULSE_PER_TURN;
    pcnt_counter_clear(params->unit);
    return rpm;
}
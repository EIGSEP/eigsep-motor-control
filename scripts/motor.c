// motor.c
#include "motor.h"
#include "pico/stdlib.h"

void stepper_init(Stepper *m,
                  uint dir_pin, uint pulse_pin,
                  uint8_t cw_val, uint8_t ccw_val,
                  uint enable_pin) {
    m->direction_pin = dir_pin;
    m->pulse_pin     = pulse_pin;
    m->enable_pin    = enable_pin;
    m->cw_val        = cw_val;
    m->ccw_val       = ccw_val;
    m->delay_us      = 0;
    m->position      = 0;
    m->dir           = 1;

    gpio_init(dir_pin);
    gpio_set_dir(dir_pin, GPIO_OUT);

    gpio_init(pulse_pin);
    gpio_set_dir(pulse_pin, GPIO_OUT);

    gpio_init(enable_pin);
    gpio_set_dir(enable_pin, GPIO_OUT);

    gpio_put(enable_pin, 1);
    gpio_put(pulse_pin, 0);
}

void stepper_move(Stepper *m) {
    if (m->dir > 0) {
        gpio_put(m->direction_pin, m->cw_val);
        m->position++;
    } else {
        gpio_put(m->direction_pin, m->ccw_val);
        m->position--;
    }
    gpio_put(m->enable_pin, 0);
    gpio_put(m->pulse_pin, 1);
    sleep_us(m->delay_us);
    gpio_put(m->pulse_pin, 0);
    sleep_us(m->delay_us);
}

void stepper_close(Stepper *m) {
    gpio_put(m->pulse_pin, 0);
    gpio_put(m->enable_pin, 1);
}


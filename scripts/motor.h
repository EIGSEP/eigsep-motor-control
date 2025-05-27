// motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "hardware/gpio.h"

typedef struct {
    uint    direction_pin;
    uint    pulse_pin;    
    uint    enable_pin;   
    uint8_t cw_val;       
    uint8_t ccw_val;      
    uint32_t delay_us;    
    int32_t position;     
    int8_t  dir;          
} Stepper;

void stepper_init(Stepper *m,
                  uint dir_pin, uint pulse_pin,
                  uint8_t cw_val, uint8_t ccw_val,
                  uint enable_pin);

void stepper_move(Stepper *m);

void stepper_close(Stepper *m);

#endif // MOTOR_H


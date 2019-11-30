#ifndef __A4988_H__
#define __A4988_H__

#include <Arduino.h>

#include <stdint.h>

#include "Stepper.h"

class A4988 : public Stepper
{
private:
    uint8_t dir_pin;
    uint8_t step_pin;

public:
    A4988(uint8_t _dir_pin, uint8_t _step_pin);
    ~A4988();

    void go_to_degree(float angle);
    void go_to_radian(float radian);
    void step(uint64_t steps, stepper_direction direction);

    float get_degrees();
    float get_radians();
};

#endif
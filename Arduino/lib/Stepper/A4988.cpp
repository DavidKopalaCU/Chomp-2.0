#include "A4988.h"

#define A4988_STEPS_PER_REV     (200.0)

A4988::A4988(uint8_t _dir_pin, uint8_t _step_pin)
{
    dir_pin = _dir_pin;
    step_pin = _step_pin;

    step_count = 0;

    pinMode(dir_pin, OUTPUT);
    pinMode(step_pin, OUTPUT);
}

A4988::~A4988() {}


void A4988::go_to_degree(float angle)
{
    float diff = angle - this->get_degrees();
    uint64_t steps = (fabs(diff) / 360.0) * A4988_STEPS_PER_REV;

    if (diff < 0) {
        this->step(steps, CCW);
    } else {
        this->step(steps, CW);
    }
}

void A4988::go_to_radian(float radian)
{
    float diff = radian - this->get_radians();
    uint64_t steps = (fabs(diff) / (2 * PI)) * A4988_STEPS_PER_REV;

    if (diff < 0) {
        this->step(steps, CCW);
    } else {
        this->step(steps, CW);
    }
}

void A4988::step(uint64_t steps, stepper_direction direction)
{
    if (direction == CW) {
        digitalWrite(dir_pin, LOW);
        step_count += steps;
    } else {
        digitalWrite(dir_pin, HIGH);
        step_count -= steps;
    }

    for (uint64_t i = 0; i < steps; i++) {
        digitalWrite(step_pin, HIGH);
        delay(1);
        digitalWrite(step_pin, LOW);
        delay(1);
    }
}

float A4988::get_degrees()
{
    return (step_count / A4988_STEPS_PER_REV) * 360.0;
}
float A4988::get_radians()
{
    return (step_count / A4988_STEPS_PER_REV) * 2 * PI;
}
#ifndef __MOTORENCODER_H__
#define __MOTORENCODER_H__

#include <Arduino.h>

#include <stdint.h>


#define PULSES_PER_REVOLUTION   (180.81)

class MotorEncoder
{
private:

public:
    MotorEncoder(uint8_t a, uint8_t b);
    ~MotorEncoder();

    uint8_t enc_a_pin;
    uint8_t enc_b_pin;

    volatile int64_t pulse_count = 0;
    float getAngle();
};

static MotorEncoder **encoders = nullptr;
static uint8_t encoder_count = 0;

#endif // __MOTORENCODER_H__
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <Arduino.h>
#include <stdint.h>

#include <MotorEncoder.h>

enum motor_direction {
    FORWARD,
    REVERSE,
    STOP
};

class Motor
{
private:
    void common_init(uint8_t _pwm_pin, uint8_t _dir_pin);

    uint8_t pwm_pin;
    uint8_t dir_pin;

    float power;
    motor_direction direction;

    MotorEncoder *encoder;

public:
    Motor(uint8_t _pwm_pin, uint8_t _dir_pin);
    Motor(uint8_t _pwm_pin, uint8_t _dir_pin, uint8_t enc_a, uint8_t enc_b);
    ~Motor();

    void setDirection(motor_direction _dir);
    motor_direction getDirection();

    void setPower(float _power);
    float getPower();

    MotorEncoder* getEncoder();
};

#endif // __MOTOR_H_
/* Designed for the TB6612FNG chip. We're using a the TB6612FNG breakout from Sparkfun
 * (https://www.sparkfun.com/products/14451), and is also available from Pololu. We are also
 * "paralleling" the two channels of the chip so that we have a hiher current limit
 * 
 * More information about how the chip functions is available online, like the Sparkfun
 * hookup guide: https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide?_ga=2.195405868.1664816146.1575054800-1624815254.1571192113
 * 
 * STBY MUST BE PULLED HIGH TO ENABLE THE CHIP
 */

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
    void common_init(uint8_t _pwm_pin, uint8_t _dir_a_pin, uint8_t _dir_b_pin);

    uint8_t pwm_pin;
    uint8_t dir_a_pin;
    uint8_t dir_b_pin;

    float power;
    motor_direction direction;

    MotorEncoder *encoder;

public:
    Motor(uint8_t _pwm_pin, uint8_t _dir_a_pin, uint8_t _dir_b_pin);
    Motor(uint8_t _pwm_pin, uint8_t _dir_a_pin, uint8_t _dir_b_pin, uint8_t enc_a, uint8_t enc_b);
    ~Motor();

    void setDirection(motor_direction _dir);
    motor_direction getDirection();

    void setPower(float _power);
    float getPower();

    MotorEncoder* getEncoder();
};

#endif // __MOTOR_H_
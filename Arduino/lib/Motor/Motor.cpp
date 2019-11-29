#include "Motor.h"

Motor::Motor(uint8_t _pwm_pin, uint8_t _dir_pin)
{
    common_init(_pwm_pin, _dir_pin);

    encoder = nullptr;
}

Motor::Motor(uint8_t _pwm_pin, uint8_t _dir_pin, uint8_t enc_a, uint8_t enc_b)
{
    common_init(_pwm_pin, _dir_pin);

    encoder = new MotorEncoder(enc_a, enc_b);
}

void Motor::common_init(uint8_t _pwm_pin, uint8_t _dir_pin)
{
    pwm_pin = _pwm_pin;
    dir_pin = _dir_pin;

    pinMode(pwm_pin, OUTPUT);
    digitalWrite(pwm_pin, LOW);

    pinMode(dir_pin, OUTPUT);
    digitalWrite(dir_pin, LOW);
}

Motor::~Motor()
{
}

// TODO
void Motor::setDirection(motor_direction _dir)
{
    direction = _dir;
}
motor_direction Motor::getDirection()
{
    return direction;
}

void Motor::setPower(float _power)
{
    power = _power;
}
float Motor::getPower()
{
    return power;
}

MotorEncoder* Motor::getEncoder()
{
    return encoder;
}
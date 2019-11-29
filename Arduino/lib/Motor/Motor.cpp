#include "Motor.h"

Motor::Motor(uint8_t _pwm_pin, uint8_t _dir_a_pin, uint8_t _dir_b_pin)
{
    common_init(_pwm_pin, _dir_a_pin, _dir_b_pin);

    encoder = nullptr;
}

Motor::Motor(uint8_t _pwm_pin, uint8_t _dir_a_pin, uint8_t _dir_b_pin, uint8_t enc_a, uint8_t enc_b)
{
    common_init(_pwm_pin, _dir_a_pin, _dir_b_pin);

    encoder = new MotorEncoder(enc_a, enc_b);
}

void Motor::common_init(uint8_t _pwm_pin, uint8_t _dir_a_pin, uint8_t _dir_b_pin)
{
    pwm_pin = _pwm_pin;
    dir_a_pin = _dir_a_pin;
    dir_b_pin = _dir_b_pin;

    pinMode(pwm_pin, OUTPUT);
    digitalWrite(pwm_pin, LOW);

    pinMode(dir_a_pin, OUTPUT);
    pinMode(dir_b_pin, OUTPUT);
    digitalWrite(dir_a_pin, LOW);
    digitalWrite(dir_b_pin, LOW);
}

Motor::~Motor()
{
    analogWrite(pwm_pin, 0);
}

// TODO
void Motor::setDirection(motor_direction _dir)
{
    direction = _dir;
    switch (direction)
    {
    case FORWARD:
        digitalWrite(dir_a_pin, LOW);
        digitalWrite(dir_b_pin, HIGH);
        break;
    
    case REVERSE:
        digitalWrite(dir_a_pin, HIGH);
        digitalWrite(dir_b_pin, LOW);
        break;

    case STOP:
        this->setPower(0);
        digitalWrite(dir_a_pin, LOW);
        digitalWrite(dir_b_pin, LOW);
        break;

    default:
        break;
    }
}
motor_direction Motor::getDirection()
{
    return direction;
}

void Motor::setPower(float _power)
{
    if (direction == STOP && _power != 0) { return; }

    power = _power;
    analogWrite(pwm_pin, power * 255);
}
float Motor::getPower()
{
    return power;
}

MotorEncoder* Motor::getEncoder()
{
    return encoder;
}
#include "MotorEncoder.h"

void encoder_interrupt_pin_2() {
    MotorEncoder *enc = encoders[0];
    if (digitalRead(enc->enc_b_pin) == LOW) {
        enc->pulse_count = enc->pulse_count + 1; 
    } else {
        enc->pulse_count = enc->pulse_count - 1; 
    }
}

void encoder_interrupt_pin_3() {
    MotorEncoder *enc = encoders[1];
    if (digitalRead(enc->enc_b_pin) == LOW) {
        enc->pulse_count = enc->pulse_count + 1; 
    } else {
        enc->pulse_count = enc->pulse_count - 1; 
    }
}

MotorEncoder::MotorEncoder(uint8_t a, uint8_t b)
{
    if (encoders == nullptr) {
        encoders = (MotorEncoder **) malloc(2 * sizeof(MotorEncoder*));
    }

    enc_a_pin = a;
    enc_b_pin = b;

    pinMode(enc_a_pin, INPUT);
    pinMode(enc_b_pin, INPUT);

    if (a == 2) {
        encoders[0] = this;
        attachInterrupt(digitalPinToInterrupt(a), encoder_interrupt_pin_2, RISING);
    } else {
        encoders[1] = this;
        attachInterrupt(digitalPinToInterrupt(b), encoder_interrupt_pin_3, RISING);
    }
}

MotorEncoder::~MotorEncoder()
{
}

float MotorEncoder::getAngle()
{
    return 2 * PI * (pulse_count / PULSES_PER_REVOLUTION);
}
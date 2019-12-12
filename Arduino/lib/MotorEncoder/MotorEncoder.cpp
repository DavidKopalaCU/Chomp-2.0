#include "MotorEncoder.h"

static MotorEncoder *encoder_2 = nullptr;
static MotorEncoder *encoder_3 = nullptr;

void encoder_interrupt_pin_2() {
    MotorEncoder *enc = encoder_2;
    if (digitalRead(enc->enc_b_pin) == LOW && !enc->flip) {
        enc->pulse_count = enc->pulse_count + 1; 
    } else {
        enc->pulse_count = enc->pulse_count - 1; 
    }
}

void encoder_interrupt_pin_3() {
    MotorEncoder *enc = encoder_3;
    if (digitalRead(enc->enc_b_pin) == LOW && !enc->flip) {
        enc->pulse_count = enc->pulse_count + 1; 
    } else {
        enc->pulse_count = enc->pulse_count - 1; 
    }
}

MotorEncoder::MotorEncoder(uint8_t a, uint8_t b, bool _flip)
{
    // if (encoders == nullptr) {
    //     encoders = (MotorEncoder **) malloc(2 * sizeof(MotorEncoder*));
    // }

    enc_a_pin = a;
    enc_b_pin = b;

    flip = _flip;

    pinMode(enc_a_pin, INPUT);
    pinMode(enc_b_pin, INPUT);

    if (a == 2) {
        encoder_2 = this;
        attachInterrupt(digitalPinToInterrupt(a), encoder_interrupt_pin_2, RISING);
    } else {
        encoder_3 = this;
        attachInterrupt(digitalPinToInterrupt(a), encoder_interrupt_pin_3, RISING);
    }
}

MotorEncoder::~MotorEncoder()
{
}

float MotorEncoder::getAngle()
{
    return 2 * PI * (pulse_count / PULSES_PER_REVOLUTION);
}
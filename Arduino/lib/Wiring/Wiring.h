#ifndef __WIRING_H__
#define __WIRING_H__

#define GPIO_LEFT_MOTOR_DIR_A   (4)
#define GPIO_LEFT_MOTOR_DIR_B   (10)
#define GPIO_LEFT_MOTOR_PWM     (5) // PIN MUST BE PWM (~) CAPABLE
#define GPIO_LEFT_MOTOR_ENC_A   (2) // PIN MUST BE INTERRUPT-ABLE
#define GPIO_LEFT_MOTOR_ENC_B   (7)

#define GPIO_RIGHT_MOTOR_DIR_A  (8)
#define GPIO_RIGHT_MOTOR_DIR_B  (11)
#define GPIO_RIGHT_MOTOR_PWM    (6) // PIN MUST BE PWM (~) CAPABLE
#define GPIO_RIGHT_MOTOR_ENC_A  (3) // PIN MUST BE INTERRUPT-ABLE
#define GPIO_RIGHT_MOTOR_ENC_B  (9)

#define GPIO_ONBOARD_LED        (13)

#endif // __WIRING_H__
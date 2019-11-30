#ifndef __STEPPER_H__
#define __STEPPER_H__

#include <stdint.h>

/* Defines a standard interface for working with stepper motors
 * Originally developed for the A4988 driver
 */

enum stepper_direction {
    CW,
    CCW
};

class Stepper
{
protected:
    int64_t step_count;

public:
    Stepper(/* args */);
    ~Stepper();

    virtual void go_to_degree(float angle);
    virtual void go_to_radian(float radian);
    virtual void step(uint64_t steps, stepper_direction direction);

    virtual float get_degrees();
    virtual float get_radians();
};


#endif
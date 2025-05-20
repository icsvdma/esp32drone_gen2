#ifndef MOTOR_H
#define MOTOR_H

#include "driver/ledc.h"


class Motor {
private:
    uint8_t pin;
    uint8_t pwm_channel;

public:
    Motor(uint8_t pin, uint8_t pwm_channel)
        : pin(pin), pwm_channel(pwm_channel) {}

    void init();
    void set_speed(uint16_t duty);
};

#endif // MOTOR_H

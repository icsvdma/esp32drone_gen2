#include "Arduino.h"
#include "motor.h"

void Motor::init() {
	ledcSetup(pwm_channel, 16000, 10); // 20kHz, 10bit PWM
	ledcAttachPin(pin, pwm_channel);
}

void Motor::set_speed(uint16_t duty) {
	ledcWrite(pwm_channel, duty);
}

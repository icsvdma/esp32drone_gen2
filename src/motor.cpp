#include "Arduino.h"
#include "motor.h"

void Motor::init() {
	ledcSetup(pwm_channel, 16000, 10); // 20kHz, 10bit PWM
	ledcAttachPin(pin, pwm_channel);
}

void Motor::set_speed(uint16_t duty) {
	ledcWrite(pwm_channel, duty);
}

//モーター出力をクリップ
int16_t Motor::clip_param(int16_t pwm_val){
	if		(pwm_val < 0)		return 0;
	else if	(pwm_val >= 1023)	return 1023;
	else						return pwm_val;
} 
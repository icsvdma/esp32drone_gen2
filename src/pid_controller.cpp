#include <stdio.h>
#include "pid_controller.h"

PIDController::PIDController() {}

float PIDController::compute(int16_t setpoint, float measured, float dt) {
    // Prevent derivative spikes when dt is too small
    if (dt < MIN_DT) {
        dt = MIN_DT;
    }
    
    float error = (float)setpoint - measured;
    
    // Update integral with anti-windup
    integral += error * dt;
    if (integral > INTEGRAL_MAX) {
        integral = INTEGRAL_MAX;
    } else if (integral < INTEGRAL_MIN) {
        integral = INTEGRAL_MIN;
    }
    
    // Calculate derivative
    float derivative = (error - prev_error) / dt;
    prev_error = error;
    
    return user_kp * error + user_ki * integral + user_kd * derivative;
}

void PIDController::set_kp(int dir){
	if(dir != 0)		user_kp = user_kp + 1.0;
	else				user_kp = user_kp - 1.0;
	printf("kp is : %f\n",user_kp);
	
}
void PIDController::set_ki(int dir){
	if(dir != 0)		user_ki = user_ki + 0.01;
	else				user_ki = user_ki - 0.01;
	printf("ki is : %f\n",user_ki);
	
}
void PIDController::set_kd(int dir){
	if(dir != 0)		user_kd = user_kd + 0.01;
	else				user_kd = user_kd - 0.01;
	printf("kd is : %f\n",user_kd);

}
float PIDController::get_kp(){
	return user_kp;
}
float PIDController::get_ki(){
	return user_ki;
}
float PIDController::get_kd(){
	return user_kd;
}
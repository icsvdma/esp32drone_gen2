#include <stdio.h>
#include "pid_controller.h"

PIDController::PIDController() {}

float PIDController::compute(int16_t setpoint, float measured, float dt) {
    float error = (float)setpoint - measured;
    integral += error * dt;
    float derivative = (error - prev_error) / dt;
    prev_error = error;
    //return kp * error + ki * integral + kd * derivative;
    return user_kp * error + user_ki * integral + user_kd * derivative;
}

void PIDController::set_kp(int dir){
	if(dir != 0)		user_kp = user_kp + 0.1;
	else				user_kp = user_kp - 0.1;
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
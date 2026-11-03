#include <stdio.h>
#include "pid_controller.h"

PIDController::PIDController() {}

PIDController::PIDController(float i_lim, float out_lim, float d_cutoff)
    : i_limit(i_lim), out_limit(out_lim), d_cutoff_hz(d_cutoff) {}

static float clampf(float v, float lim) {
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

float PIDController::compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    //I項: ki を掛けてから積分し、上限でクリップ（ワインドアップ抑制）
    integral += user_ki * error * dt;
    if (i_limit > 0) integral = clampf(integral, i_limit);

    //D項: 測定値の微分（目標値が変わっても D キックが出ない）＋ 1次LPF
    if (first) {
        prev_meas = measured;
        first = false;
    }
    float d_raw = (measured - prev_meas) / dt;
    prev_meas = measured;
    if (d_cutoff_hz > 0) {
        float rc = 1.0f / (2.0f * 3.14159265f * d_cutoff_hz);
        float alpha = dt / (rc + dt);
        d_filt += alpha * (d_raw - d_filt);
    } else {
        d_filt = d_raw;
    }

    float out = user_kp * error + integral - user_kd * d_filt;
    if (out_limit > 0) out = clampf(out, out_limit);
    return out;
}

void PIDController::reset() {
    integral = 0;
    d_filt = 0;
    first = true;
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
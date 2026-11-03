#include <stdio.h>
#include "pid_controller.h"
#include "flight_config.h"

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

void PIDController::setGains(float kp, float ki, float kd) {
    user_kp = kp;
    user_ki = ki;
    user_kd = kd;
}

//現在値の GAIN_STEP_RATIO 倍（最小 min_step）だけ増減し、0〜max_v に制限する
//（負のゲインは正帰還になって発散するので禁止）
static float stepGain(float v, int dir, float min_step, float max_v) {
    float step = v * GAIN_STEP_RATIO;
    if (step < min_step) step = min_step;
    v += (dir > 0) ? step : -step;
    if (v < 0.0f)  v = 0.0f;
    if (v > max_v) v = max_v;
    return v;
}

//dir > 0 で増加、それ以外で減少
//制御タスクから呼ぶこと（compute() と同じタスクで書き換えるため排他制御は不要）
void PIDController::set_kp(int dir){
	user_kp = stepGain(user_kp, dir, KP_STEP_MIN, KP_MAX);
}
void PIDController::set_ki(int dir){
	user_ki = stepGain(user_ki, dir, KI_STEP_MIN, KI_MAX);
}
void PIDController::set_kd(int dir){
	user_kd = stepGain(user_kd, dir, KD_STEP_MIN, KD_MAX);
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
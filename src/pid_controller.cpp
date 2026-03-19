#include <stdio.h>
#include "pid_controller.h"

PIDController::PIDController() {}

/**
 * PID制御計算の実装
 * Implementation of PID control calculation
 * 
 * PID式: output = Kp×error + Ki×∫error(t)dt + Kd×d(error)/dt
 * 
 * 積分ワインドアップ対策として、積分項を±100.0に制限しています。
 * これにより、以下の問題を防ぎます：
 * - 長時間の大きな誤差による過度な積分蓄積
 * - モーター飽和時の制御不能
 * - 急激な目標値変更時のオーバーシュート
 * 
 * As a countermeasure against integral windup, the integral term is 
 * limited to ±100.0. This prevents:
 * - Excessive integral accumulation from prolonged large errors
 * - Loss of control during motor saturation
 * - Overshoot during sudden setpoint changes
 */
float PIDController::compute(int16_t setpoint, float measured, float dt) {
    // 微分スパイク防止：dtが小さすぎる場合は最小値を使用
    // Prevent derivative spikes: use minimum value if dt is too small
    if (dt < MIN_DT) {
        dt = MIN_DT;
    }
    
    // 誤差計算 / Calculate error
    float error = (float)setpoint - measured;
    
    // 積分項の更新（アンチワインドアップ付き）
    // Update integral term (with anti-windup)
    integral += error * dt;
    
    // 積分ワインドアップ防止：積分項を制限範囲内にクランプ
    // Prevent integral windup: clamp integral term within limits
    // 
    // 例 / Example:
    // - 誤差が10度で1秒間継続した場合、integral = 10
    //   If error is 10° for 1 second, integral = 10
    // - しかし、無制限に増加させると制御が不安定になる
    //   However, unlimited growth causes control instability
    // - そのため±100.0で制限する
    //   Therefore, we limit to ±100.0
    if (integral > INTEGRAL_MAX) {
        integral = INTEGRAL_MAX;
    } else if (integral < INTEGRAL_MIN) {
        integral = INTEGRAL_MIN;
    }
    
    // 微分項の計算 / Calculate derivative term
    float derivative = (error - prev_error) / dt;
    prev_error = error;
    
    // PID出力 = P項 + I項 + D項
    // PID output = P-term + I-term + D-term
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
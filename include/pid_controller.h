#ifndef PID_CTRL_H
#define PID_CTRL_H

// PID制御器
// - I項: 積分値に ki を掛けた状態で保持し、上限でクリップする（ゲイン変更時に出力が跳ねない）
// - D項: 誤差ではなく測定値を微分し（目標値変化による D キックを防ぐ）、1次LPFを通す
// - 出力: 上限でクリップする
class PIDController {
private:
    float user_kp = 0;
    float user_ki = 0;
    float user_kd = 0;

    float i_limit       = 0;    // I項の上限（出力の単位）
    float out_limit     = 0;    // 出力の上限
    float d_cutoff_hz   = 0;    // D項LPFのカットオフ周波数[Hz]（0以下でフィルタなし）

    float integral      = 0;    // ki * ∫error dt
    float prev_meas     = 0;
    float d_filt        = 0;
    bool  first         = true;

public:
    PIDController();
    PIDController(float i_lim, float out_lim, float d_cutoff);
    float compute(float setpoint, float measured, float dt);
    void reset();

	void set_kp(int dir);
	void set_ki(int dir);
	void set_kd(int dir);
	float get_kp();
	float get_ki();
	float get_kd();
};

#endif

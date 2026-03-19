#ifndef PID_CTRL_H
#define PID_CTRL_H

/**
 * PIDController - ドローンの姿勢制御用PIDコントローラ
 * 
 * このクラスは、積分ワインドアップ防止機能を備えたPID制御を実装します。
 * This class implements PID control with integral anti-windup protection.
 * 
 * Design Note (設計メモ):
 * PIDゲインはデフォルト値で初期化されますが、PS3コントローラーの
 * ボタン操作により飛行中にリアルタイムで調整可能です。
 * 
 * PID gains are initialized with default values, but can be adjusted
 * in real-time during flight using PS3 controller buttons.
 * 
 * - Left/Right: パラメータ選択 (P, I, D) / Select parameter (P, I, D)
 * - Up: 値を増加 / Increase value
 * - Down: 値を減少 / Decrease value
 */
class PIDController {
private:
    // PID gains (PIDゲイン)
    float user_kp = 2.0;    // Proportional gain / 比例ゲイン
    float user_ki = 0.05;   // Integral gain / 積分ゲイン
    float user_kd = 0.5;    // Derivative gain / 微分ゲイン

    float prev_error = 0;   // Previous error for derivative calculation / 微分計算用の前回誤差
    float integral = 0;     // Accumulated integral term / 積分項の累積値
    
    /**
     * 積分ワインドアップ防止用の制限値
     * Anti-windup limits to prevent integral term from growing unbounded
     * 
     * 積分項が±INTEGRAL_MAXを超えないように制限することで、
     * 長時間の誤差やモーター飽和時の不安定性を防ぎます。
     * 
     * By limiting the integral term to ±INTEGRAL_MAX, we prevent
     * instability caused by prolonged errors or motor saturation.
     */
    static constexpr float INTEGRAL_MAX = 100.0;
    static constexpr float INTEGRAL_MIN = -100.0;
    
    /**
     * 微分スパイク防止用の最小dt値
     * Minimum dt to prevent derivative spikes
     * 
     * 非常に小さいdtによるゼロ除算や極端な微分値を防ぎます。
     * Prevents division by near-zero or extreme derivative values.
     */
    static constexpr float MIN_DT = 0.001;

public:
    PIDController();
    
    /**
     * PID制御値を計算
     * Compute PID control output
     * 
     * @param setpoint 目標値（度） / Target value in degrees
     * @param measured 現在値（度） / Current measured value in degrees  
     * @param dt サンプリング周期（秒） / Sampling period in seconds
     * @return 制御出力 / Control output
     * 
     * この関数は以下の安全機能を含みます：
     * This function includes the following safety features:
     * 1. 積分ワインドアップ防止（±100.0制限）
     *    Integral anti-windup (±100.0 limits)
     * 2. 微分スパイク防止（最小dt = 0.001s）
     *    Derivative spike protection (minimum dt = 0.001s)
     */
    float compute(int16_t setpoint, float measured, float dt);

    // PID gain adjustment methods / PIDゲイン調整メソッド
	void set_kp(int dir);  // dir != 0: increase, dir == 0: decrease
	void set_ki(int dir);
	void set_kd(int dir);
	
	// PID gain getter methods / PIDゲイン取得メソッド
	float get_kp();
	float get_ki();
	float get_kd();

	void updatePID();
};

#endif
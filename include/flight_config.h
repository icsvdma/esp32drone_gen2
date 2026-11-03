#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

//==========================================================
// 飛行制御の設定値
//==========================================================

// 制御ループ周期（センサ読み → 姿勢推定 → PID → モータ出力）
#define CONTROL_HZ				250
#define CONTROL_PERIOD_MS		(1000 / CONTROL_HZ)
#define CONTROL_DT_NOMINAL		(1.0f / CONTROL_HZ)

// 制御タスクの設定（Bluetooth はコア0で動くので、制御はコア1に固定）
#define CONTROL_TASK_CORE		1
#define CONTROL_TASK_PRIORITY	(configMAX_PRIORITIES - 2)
#define CONTROL_TASK_STACK		8192

// LED更新・デバッグ出力など、制御以外の処理の周期
#define SERVICE_PERIOD_MS		50

// スティック入力
#define STICK_DEADBAND			6		// 中立付近の不感帯（生値, 最大127）
#define MAX_ANGLE_DEG			30.0f	// Roll/Pitch の最大目標角度[deg]

// スティック → 目標角の符号（符号はここだけで決める）
// 目標角の正の向きは Madgwick の Roll/Pitch の正の向きと同じ
// 従来の動作（右スティック右/手前で正の目標角）と同じになるよう +1 にしている
#define ROLL_STICK_DIR			(+1.0f)
#define PITCH_STICK_DIR			(+1.0f)
// 左スティック右(lx>0)で上から見て時計回り = ジャイロZが負（Z軸上向きの場合）
#define YAW_STICK_DIR			(-1.0f)

// カスケード制御
// 外側ループ: 角度誤差[deg] → 目標角速度[deg/s]（P制御）
#define ANGLE_KP				4.0f	// 未チューニングの仮値
// 目標角速度の上限。ジャイロのフルスケール（現在 ±250dps）より小さくすること
#define MAX_RATE_DPS			200.0f
// 内側ループ（角速度PID）の制限
#define RATE_I_LIMIT			100.0f	// I項の上限（PWMカウント）
#define RATE_OUT_LIMIT			400.0f	// 出力の上限（PWMカウント）
#define RATE_D_CUTOFF_HZ		30.0f	// D項LPFのカットオフ周波数

// Roll/Pitch 角速度PIDのゲイン初期値（NVSに保存値がなければこれを使う）
// 単位: 出力[PWMカウント] / 角速度誤差[deg/s]。未チューニングの仮値
#define RATE_KP_DEFAULT			0.5f
#define RATE_KI_DEFAULT			0.5f
#define RATE_KD_DEFAULT			0.005f

// 十字キーでのゲイン調整
// 1回の増減幅 = 現在値 × GAIN_STEP_RATIO（ただし最小 *_STEP_MIN）。0〜*_MAX に制限
#define GAIN_STEP_RATIO			0.1f
#define KP_STEP_MIN				0.05f
#define KI_STEP_MIN				0.05f
#define KD_STEP_MIN				0.0005f
#define KP_MAX					5.0f
#define KI_MAX					5.0f
#define KD_MAX					0.1f

// ヨー制御（6軸IMUではヨー角の絶対値は分からないので、角速度だけを制御する）
// スティック中立で「回転速度 0」を保つ
#define MAX_YAW_RATE_DPS		120.0f	// 左スティック左右いっぱいでの目標ヨー角速度
#define YAW_RATE_KP				1.0f	// 未チューニングの仮値
#define YAW_RATE_KI				0.5f	// 未チューニングの仮値
#define YAW_RATE_KD				0.0f
#define YAW_I_LIMIT				100.0f
#define YAW_OUT_LIMIT			200.0f	// Roll/Pitch の制御余裕を食いすぎないよう小さめ

// モータ出力（LEDC 10bit）
#define MOTOR_PWM_MIN			0
#define MOTOR_PWM_MAX			1023

// モータミキシング表 {roll, pitch, yaw} の係数
// 並びは MOT_FR, MOT_FL, MOT_RR, MOT_RL（define_system.h のピン定義と同じ順）
// 符号はセンサの取り付け向きで決まるので、実機で必ず確認すること（docs/stability_review.md 4章）
#define MIX_FR_ROLL				(-1.0f)
#define MIX_FR_PITCH			(-1.0f)
#define MIX_FL_ROLL				(+1.0f)
#define MIX_FL_PITCH			(-1.0f)
#define MIX_RR_ROLL				(-1.0f)
#define MIX_RR_PITCH			(+1.0f)
#define MIX_RL_ROLL				(+1.0f)
#define MIX_RL_PITCH			(+1.0f)
// ヨーの係数は「プロペラの回転方向（上から見て）が FR:反時計 / FL:時計 / RR:時計 / RL:反時計、
// MPU6050 のZ軸が上向き」を前提にしている（前提が違う場合は符号を反転すること）
// ジャイロZを正（上から見て反時計回り）にしたいときは、時計回りのプロペラ（FL, RR）の
// 回転を上げると反トルクで機体が反時計回りに回る
#define MIX_FR_YAW				(-1.0f)
#define MIX_FL_YAW				(+1.0f)
#define MIX_RR_YAW				(+1.0f)
#define MIX_RL_YAW				(-1.0f)

#endif // FLIGHT_CONFIG_H

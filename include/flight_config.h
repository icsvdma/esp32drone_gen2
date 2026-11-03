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

// モータ出力（LEDC 10bit）
#define MOTOR_PWM_MIN			0
#define MOTOR_PWM_MAX			1023

// モータミキシング表 {roll, pitch} の係数
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

#endif // FLIGHT_CONFIG_H

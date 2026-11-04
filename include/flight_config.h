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

// スロットル（増減式）
// 左スティック上下の倒し量で「スロットルを変える速さ」を決める。中央（不感帯）なら今の値を保つ
// 中央付近で細かく調整できるよう、倒し量は2乗してから使う
#define THROTTLE_RATE			600.0f	// スティックいっぱいでの変化の速さ[PWMカウント/秒]（仮の値）
#define THROTTLE_FAST_DESCEND_MUL	3.0f	// 十字キー下（飛行モード）で下げるときの速さの倍率
#define THROTTLE_MAX			870		// スロットルの上限（MOTOR_PWM_MAX の約85%。姿勢制御の余裕を残す。#4）

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
#define ANGLE_KP_DEFAULT		4.0f	// 初期値（未チューニングの仮値）。調整モードで変更できる
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

// ゲイン調整モード（十字キーでの調整）
// 1回の増減幅 = 現在値 × GAIN_STEP_RATIO（ただし最小 *_STEP_MIN）。0〜*_MAX に制限
#define GAIN_STEP_RATIO			0.1f
#define KP_STEP_MIN				0.05f
#define KI_STEP_MIN				0.05f
#define KD_STEP_MIN				0.0005f
#define KP_MAX					5.0f
#define KI_MAX					5.0f
#define KD_MAX					0.1f
#define ANGLE_KP_STEP_MIN		0.1f
#define ANGLE_KP_MAX			15.0f

// ボタン
#define BUTTON_LONG_MS			1000	// 長押しと判定する時間

// コントローラへの通知（振動の強さは 0〜100）
#define FB_RUMBLE_INTENSITY		100		// 通知の振動の強さ
#define FB_HOLD_RUMBLE_INTENSITY	30		// START を押し続けている間の弱い振動
#define FB_SHORT_MS				150		// 短い振動
#define FB_LONG_MS				600		// 長い振動
#define FB_GAP_MS				150		// 振動と振動の間
#define FB_TICK_MS				100		// ゲイン変更などの軽い合図

// ヨー制御（6軸IMUではヨー角の絶対値は分からないので、角速度だけを制御する）
// スティック中立で「回転速度 0」を保つ
#define MAX_YAW_RATE_DPS		120.0f	// 左スティック左右いっぱいでの目標ヨー角速度
#define YAW_RATE_KP_DEFAULT		1.0f	// 初期値（未チューニングの仮値）。調整モードで変更できる
#define YAW_RATE_KI_DEFAULT		0.5f	// 初期値（未チューニングの仮値）。調整モードで変更できる
#define YAW_RATE_KD				0.0f	// 固定
#define YAW_I_LIMIT				100.0f
#define YAW_OUT_LIMIT			200.0f	// Roll/Pitch の制御余裕を食いすぎないよう小さめ

// 安全機能（アーム/ディスアーム・フェイルセーフ）
// アーム: START 長押し（BUTTON_LONG_MS）。スティックを上に倒していない・通信正常・
//         キャリブレーション成功・機体がほぼ水平のときだけ
// ディスアーム: スロットルが THROTTLE_MIN_CONTROL 未満なら START 短押し、飛行中は START 長押し
// 緊急停止: L1 + R1 同時押し（どの状態でもすぐ止める）
// 自動ディスアーム: 下記の条件
#define LINK_TIMEOUT_MS			500		// この時間パケットが届かなければ通信断としてディスアーム
#define THROTTLE_MIN_CONTROL	30		// これ未満のスロットルではモータ停止＆PIDリセット（地上でI項を溜めない）
#define ARM_MAX_TILT_DEG		25.0f	// アーム時に許容する傾き
#define TILT_CUTOFF_DEG			70.0f	// 飛行中にこれ以上傾いたら（転倒）ディスアーム
#define AUTO_DISARM_MS			10000	// スロットル0のままこの時間経過したらディスアーム
#define IMU_FAIL_MAX			25		// センサ読み出しがこの回数連続で失敗したらディスアーム（250Hzで100ms）

// 着陸の自動検出: アーム後に一度飛んだ（スロットルが THROTTLE_MIN_CONTROL 以上になった）あと、
// モータが止まっていて・傾きが小さく・角速度が小さい状態が続いたらディスアームする
#define LAND_DETECT_MS			2000
#define LAND_MAX_TILT_DEG		20.0f
#define LAND_MAX_RATE_DPS		15.0f

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

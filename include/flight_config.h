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

#endif // FLIGHT_CONFIG_H

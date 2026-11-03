#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "drone_controller.h"

//Droneクラス初期化	
void DroneController::init() {
	//モーター制御の初期化を行う
	m1.init();
	m2.init();
	m3.init();
	m4.init();

	//MPU6050の初期化を実施、オフセットの値を設定する
	mpu_user.init();
	//wakeupを送信
	mpu_user.mpu_write_data();

	//センサのフィルタを使用するために初期化を行う（制御周期と一致させる）
	madgwick_user.init(CONTROL_HZ);

	//コントローラの初期化
	ps3ctrl.init();
	
	//シリアルLEDの初期化	
	ledctrl.init();
	//シリアルLED全消灯
	ledctrl.allClear();

	//ジャイロ/加速度のオフセットを測定する（この間は機体を水平に置いて動かさないこと）
	//キャリブレーション中は黄色、失敗したら赤で知らせる
	ledctrl.setRGBcolor(20,20,0);
	vTaskDelay(pdMS_TO_TICKS(100));	//wakeup後の安定待ち
	if (mpu_user.calibrate() == ESP_OK) {
		//初期化処理完了をLEDでお知らせ
		ledctrl.setRGBcolor(0,10,20);
	} else {
		ledctrl.setRGBcolor(30,0,0);
	}

	//battery ADCの初期化
	batt_adc.init();

	//バッテリ電圧監視タスクの起動
	batt_adc.start();
}

//制御タスクを起動する
void DroneController::start() {
	xTaskCreatePinnedToCore(DroneController::controlTask, "ctrl_task",
							CONTROL_TASK_STACK, this,
							CONTROL_TASK_PRIORITY, NULL, CONTROL_TASK_CORE);
}

//制御タスク本体
//vTaskDelayUntil で周期を固定し、dt は実測値を使う
void DroneController::controlTask(void *param) {
	DroneController* self = static_cast<DroneController*>(param);

	TickType_t last_wake = xTaskGetTickCount();
	int64_t prev_us = esp_timer_get_time();

	while (1) {
		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));

		int64_t now_us = esp_timer_get_time();
		float dt = (now_us - prev_us) * 1e-6f;
		prev_us = now_us;
		//異常値（初回やデバッガ停止後など）は公称値に置き換える
		if (dt <= 0.0f || dt > CONTROL_DT_NOMINAL * 4) {
			dt = CONTROL_DT_NOMINAL;
		}
		self->step(dt);
	}
}

//制御タスク以外（loop）から呼ぶ。LED表示・デバッグ出力
void DroneController::service() {
	//PIDパラメータの選択状態を取得してLEDに反映する（変化したときだけ更新）
	int mode = ps3ctrl.getPidAly();
	if (mode != last_pid_mode) {
		ledctrl.updatePidMode(mode);
		last_pid_mode = mode;
	}

#ifdef DEBUG_MODE
	//mpu_user.debug_sensor();
	//madgwick_user.debug_madrpy();
	//debug_rpy();
	//debug_base_rp();
	//debug_angle();
	//debug_rate();
	//ps3ctrl.debug_raw_param();
	//ps3ctrl.debug_target_param();
	debug_motor_param();
#endif
}

static float rateLimit(float rate) {
	if (rate >  MAX_RATE_DPS) return  MAX_RATE_DPS;
	if (rate < -MAX_RATE_DPS) return -MAX_RATE_DPS;
	return rate;
}

//1制御周期ぶんの処理（センサ読み → 姿勢推定 → PID → モータ出力）
void DroneController::step(float dt) {
	last_dt = dt;

	//センサを読んで姿勢推定を更新する
	if (mpu_user.read(imu) == ESP_OK) {
		madgwick_user.update_mad(imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz);
	} else {
		imu_err_count++;
	}

	//コントローラの制御状態を更新する
	ps3ctrl.update();

	//コントローラから推力を取得して制御する
	base_thrust	= ps3ctrl.getThrust();
	//コントローラから目標の角度を取得して制御する（符号は flight_config.h で決める）
	base_roll	= ps3ctrl.getRoll();
	base_pitch	= ps3ctrl.getPitch();

	//外側ループ: 角度誤差 → 目標角速度（P制御）
	roll_rate_sp	= rateLimit(ANGLE_KP * (base_roll  - madgwick_user.getRoll()));
	pitch_rate_sp	= rateLimit(ANGLE_KP * (base_pitch - madgwick_user.getPitch()));

	//内側ループ: 目標角速度とジャイロの角速度でPID
	//（Madgwick の Roll はジャイロX軸、Pitch はジャイロY軸まわりの回転で増える）
	roll_output		= pid_roll_rate.compute(roll_rate_sp, imu.gx, dt);
	pitch_output	= pid_pitch_rate.compute(pitch_rate_sp, imu.gy, dt);

	mix(base_thrust, roll_output, pitch_output);
	outputMotors();
}

//推力と各軸の制御量から各モータの出力を計算する
//実機出力・デバッグ表示ともにこの結果（motor_out）を使う
void DroneController::mix(float thrust, float roll, float pitch) {
	static const float MIX[MOT_NUM][2] = {
		{ MIX_FR_ROLL, MIX_FR_PITCH },	// front-right	MOT_FR_OUT
		{ MIX_FL_ROLL, MIX_FL_PITCH },	// front-left	MOT_FL_OUT
		{ MIX_RR_ROLL, MIX_RR_PITCH },	// back-right	MOT_RR_OUT
		{ MIX_RL_ROLL, MIX_RL_PITCH },	// back-left	MOT_RL_OUT
	};

	for (int i = 0; i < MOT_NUM; i++) {
		float v = thrust + MIX[i][0] * roll + MIX[i][1] * pitch;
		//float のまま範囲を制限してから整数にする（範囲外の float→int 変換は未定義動作）
		if (v < MOTOR_PWM_MIN) v = MOTOR_PWM_MIN;
		if (v > MOTOR_PWM_MAX) v = MOTOR_PWM_MAX;
		motor_out[i] = (uint16_t)(v + 0.5f);
	}
}

void DroneController::outputMotors() {
#ifndef DEBUG_MODE
	m1.set_speed(motor_out[MOT_FR]);
	m2.set_speed(motor_out[MOT_FL]);
	m3.set_speed(motor_out[MOT_RR]);
	m4.set_speed(motor_out[MOT_RL]);
#endif
}


//==========================================================
//debug用関数群（シリアル出力）
//==========================================================

void DroneController::debug_rpy(){
	printf("%d,",base_thrust);
	printf("%.1f,",roll_output);
	printf("%.1f,",pitch_output);
	printf("\n");
}

void DroneController::debug_base_rp(){
	printf("%d\t,",base_thrust);
	printf("%.2f\t,",base_roll);
	printf("%.2f\t,",base_pitch);
	printf("\n");
}

void DroneController::debug_angle(){
	printf("%f\t,",madgwick_user.getRoll());
	printf("%f\t,",madgwick_user.getPitch());
	printf("dt=%.4f\t,i2c_err=%lu",last_dt,(unsigned long)imu_err_count);
	printf("\n");
}

void DroneController::debug_rate(){
	printf("roll sp:%.1f gx:%.1f\t,",roll_rate_sp,imu.gx);
	printf("pitch sp:%.1f gy:%.1f",pitch_rate_sp,imu.gy);
	printf("\n");
}

void DroneController::debug_motor_param(){
	//実機に出力する値（motor_out）をそのまま表示する
	printf("FR:%u\t,FL:%u\t,RR:%u\t,RL:%u",
		motor_out[MOT_FR], motor_out[MOT_FL], motor_out[MOT_RR], motor_out[MOT_RL]);
	printf("\n");
}

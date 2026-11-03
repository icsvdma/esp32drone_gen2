#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "Arduino.h"
#include <math.h>
#include <Preferences.h>
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

	//Roll/Pitch 角速度PIDのゲインを読み込む（NVSに保存値がなければ初期値）
	loadGains();

	//ヨーのゲインは固定値（十字キーでの調整対象外）
	pid_yaw_rate.setGains(YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD);

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
	calib_ok = (mpu_user.calibrate() == ESP_OK);
	if (calib_ok) {
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
	//ゲインが変わったら表示する（printf は制御タスクでは行わない）
	if (gains_changed) {
		gains_changed = false;
		printGains();
	}

	//ゲインの保存要求。NVS書き込み中はフラッシュアクセスで両コアが止まることがあるため
	//ディスアーム中だけ実行する
	if (save_requested) {
		save_requested = false;
		if (!armed) {
			saveGains();
		} else {
			printf("[gain] save rejected: disarm first\n");
		}
	}

	//アーム状態の表示
	uint8_t ev = arm_event;
	if (ev != EV_NONE) {
		arm_event = EV_NONE;
		printf("[arm] %s\n", armEventStr(ev));
	}
	bool is_armed = armed;
	if (is_armed != last_armed_led) {
		ledctrl.setArmIndicator(is_armed);
		last_armed_led = is_armed;
	}

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

//==========================================================
//PIDゲイン管理
//==========================================================

//PS3のボタン操作をまとめて処理する（制御タスクから呼ぶ）
void DroneController::handleCommands() {
	Ps3Command cmd;
	while (ps3ctrl.popCommand(cmd)) {
		if (cmd.type == PS3_CMD_SAVE_GAINS) {
			save_requested = true;
		} else if (cmd.type == PS3_CMD_ARM_TOGGLE) {
			arm_toggle_req = true;
		} else {
			applyGainCommand(cmd);
			gains_changed = true;
		}
	}
}

//Roll と Pitch は同じゲインで調整する
void DroneController::applyGainCommand(const Ps3Command &cmd) {
	int dir = (cmd.type == PS3_CMD_GAIN_UP) ? 1 : -1;
	switch (cmd.term) {
		case 0:	pid_roll_rate.set_kp(dir);	pid_pitch_rate.set_kp(dir);	break;
		case 1:	pid_roll_rate.set_ki(dir);	pid_pitch_rate.set_ki(dir);	break;
		case 2:	pid_roll_rate.set_kd(dir);	pid_pitch_rate.set_kd(dir);	break;
		default: break;
	}
}

//保存値が壊れている（NaN・負・上限超え）ときは初期値を使う
static float validGain(float v, float max_v, float def) {
	if (isnan(v) || v < 0.0f || v > max_v) return def;
	return v;
}

void DroneController::loadGains() {
	Preferences prefs;
	float kp = RATE_KP_DEFAULT, ki = RATE_KI_DEFAULT, kd = RATE_KD_DEFAULT;
	if (prefs.begin("pid", true)) {		//読み取り専用
		kp = validGain(prefs.getFloat("rp_kp", RATE_KP_DEFAULT), KP_MAX, RATE_KP_DEFAULT);
		ki = validGain(prefs.getFloat("rp_ki", RATE_KI_DEFAULT), KI_MAX, RATE_KI_DEFAULT);
		kd = validGain(prefs.getFloat("rp_kd", RATE_KD_DEFAULT), KD_MAX, RATE_KD_DEFAULT);
		prefs.end();
	}
	pid_roll_rate.setGains(kp, ki, kd);
	pid_pitch_rate.setGains(kp, ki, kd);
	printGains();
}

void DroneController::saveGains() {
	Preferences prefs;
	if (!prefs.begin("pid", false)) {
		printf("[gain] save failed: NVS open error\n");
		return;
	}
	prefs.putFloat("rp_kp", pid_roll_rate.get_kp());
	prefs.putFloat("rp_ki", pid_roll_rate.get_ki());
	prefs.putFloat("rp_kd", pid_roll_rate.get_kd());
	prefs.end();
	printf("[gain] saved\n");
	printGains();
}

void DroneController::printGains() {
	printf("[gain] rate kp:%.3f ki:%.3f kd:%.4f\n",
		pid_roll_rate.get_kp(), pid_roll_rate.get_ki(), pid_roll_rate.get_kd());
}

//==========================================================
//安全機能（アーム/ディスアーム・フェイルセーフ）
//==========================================================

//制御タスクから毎周期呼ぶ
void DroneController::updateArmState(uint32_t now_ms) {
	bool link_ok = ps3ctrl.isLinkAlive();

	if (armed) {
		float roll  = madgwick_user.getRoll();
		float pitch = madgwick_user.getPitch();

		if (!link_ok) {
			disarm(EV_DISARM_LINK);							//フェイルセーフ: 通信断
		} else if (imu_fail_streak >= IMU_FAIL_MAX) {
			disarm(EV_DISARM_IMU);							//センサ異常
		} else if (fabsf(roll) > TILT_CUTOFF_DEG || fabsf(pitch) > TILT_CUTOFF_DEG) {
			disarm(EV_DISARM_TILT);							//転倒
		} else if (base_thrust == 0) {
			if ((uint32_t)(now_ms - zero_thr_since_ms) > AUTO_DISARM_MS) {
				disarm(EV_DISARM_AUTO);						//スロットル0のまま放置
			}
		} else {
			zero_thr_since_ms = now_ms;						//最後にスロットルが入っていた時刻
		}
	}

	if (arm_toggle_req) {
		arm_toggle_req = false;
		if (armed) {
			disarm(EV_DISARM_BUTTON);
		} else {
			tryArm(now_ms);
		}
	}
}

//アームできる条件をすべて満たしたときだけアームする
void DroneController::tryArm(uint32_t now_ms) {
	float roll  = madgwick_user.getRoll();
	float pitch = madgwick_user.getPitch();

	if (!calib_ok) {
		arm_event = EV_REJECT_CALIB;
	} else if (!ps3ctrl.isLinkAlive()) {
		arm_event = EV_REJECT_LINK;
	} else if (base_thrust != 0) {
		arm_event = EV_REJECT_THROTTLE;						//急にモータが回らないようスロットル0のときだけ
	} else if (fabsf(roll) > ARM_MAX_TILT_DEG || fabsf(pitch) > ARM_MAX_TILT_DEG) {
		arm_event = EV_REJECT_TILT;
	} else {
		resetControllers();
		zero_thr_since_ms = now_ms;		//スロットル0放置タイマーの起点
		armed = true;
		arm_event = EV_ARMED;
	}
}

void DroneController::disarm(ArmEvent reason) {
	armed = false;
	resetControllers();
	stopMotors();
	arm_event = reason;
}

void DroneController::stopMotors() {
	for (int i = 0; i < MOT_NUM; i++) {
		motor_out[i] = 0;
	}
	roll_output = pitch_output = yaw_output = 0;
	outputMotors();
}

void DroneController::resetControllers() {
	pid_roll_rate.reset();
	pid_pitch_rate.reset();
	pid_yaw_rate.reset();
}

const char* DroneController::armEventStr(uint8_t ev) {
	switch (ev) {
		case EV_ARMED:				return "ARMED";
		case EV_DISARM_BUTTON:		return "disarmed (button)";
		case EV_DISARM_LINK:		return "disarmed (link lost)";
		case EV_DISARM_TILT:		return "disarmed (tilt over limit)";
		case EV_DISARM_IMU:			return "disarmed (IMU error)";
		case EV_DISARM_AUTO:		return "disarmed (idle timeout)";
		case EV_REJECT_LINK:		return "arm rejected: no controller link";
		case EV_REJECT_THROTTLE:	return "arm rejected: throttle not zero";
		case EV_REJECT_TILT:		return "arm rejected: not level";
		case EV_REJECT_CALIB:		return "arm rejected: calibration failed";
		default:					return "?";
	}
}

static float rateLimit(float rate) {
	if (rate >  MAX_RATE_DPS) return  MAX_RATE_DPS;
	if (rate < -MAX_RATE_DPS) return -MAX_RATE_DPS;
	return rate;
}

//1制御周期ぶんの処理（センサ読み → 姿勢推定 → PID → モータ出力）
void DroneController::step(float dt) {
	last_dt = dt;

	uint32_t now_ms = millis();

	//センサを読んで姿勢推定を更新する（ディスアーム中も姿勢推定は続ける）
	if (mpu_user.read(imu) == ESP_OK) {
		madgwick_user.update_mad(imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz);
		imu_fail_streak = 0;
	} else {
		imu_err_count++;
		imu_fail_streak++;
	}

	//コントローラの制御状態を更新する（通信断なら入力はすべて0になる）
	ps3ctrl.update(now_ms);
	//ボタン操作（ゲイン変更・アームなど）を反映する
	handleCommands();

	//コントローラから推力を取得して制御する
	base_thrust	= ps3ctrl.getThrust();

	//アーム/ディスアームの判定（フェイルセーフを含む）
	updateArmState(now_ms);

	//ディスアーム中、またはスロットルが低いときはモータを止めてPIDをリセットする
	//（地上でI項が溜まって、離陸の瞬間に傾くのを防ぐ）
	if (!armed || base_thrust < THROTTLE_MIN_CONTROL) {
		resetControllers();
		stopMotors();
		return;
	}

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

	//ヨー: スティックの目標角速度とジャイロZでPID（中立なら回転を止める）
	yaw_rate_sp		= ps3ctrl.getYawRate();
	yaw_output		= pid_yaw_rate.compute(yaw_rate_sp, imu.gz, dt);

	mix(base_thrust, roll_output, pitch_output, yaw_output);
	outputMotors();
}

//推力と各軸の制御量から各モータの出力を計算する
//実機出力・デバッグ表示ともにこの結果（motor_out）を使う
void DroneController::mix(float thrust, float roll, float pitch, float yaw) {
	static const float MIX[MOT_NUM][3] = {
		{ MIX_FR_ROLL, MIX_FR_PITCH, MIX_FR_YAW },	// front-right	MOT_FR_OUT
		{ MIX_FL_ROLL, MIX_FL_PITCH, MIX_FL_YAW },	// front-left	MOT_FL_OUT
		{ MIX_RR_ROLL, MIX_RR_PITCH, MIX_RR_YAW },	// back-right	MOT_RR_OUT
		{ MIX_RL_ROLL, MIX_RL_PITCH, MIX_RL_YAW },	// back-left	MOT_RL_OUT
	};

	for (int i = 0; i < MOT_NUM; i++) {
		float v = thrust + MIX[i][0] * roll + MIX[i][1] * pitch + MIX[i][2] * yaw;
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
	printf("pitch sp:%.1f gy:%.1f\t,",pitch_rate_sp,imu.gy);
	printf("yaw sp:%.1f gz:%.1f",yaw_rate_sp,imu.gz);
	printf("\n");
}

void DroneController::debug_motor_param(){
	//実機に出力する値（motor_out）をそのまま表示する
	printf("FR:%u\t,FL:%u\t,RR:%u\t,RL:%u",
		motor_out[MOT_FR], motor_out[MOT_FL], motor_out[MOT_RR], motor_out[MOT_RL]);
	printf("\n");
}

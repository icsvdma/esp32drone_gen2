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

	//ゲインと操作の設定を読み込む（NVSに保存値がなければ初期値）
	loadGains();
	loadUiSettings();

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
	uint32_t now_ms = millis();

	//コントローラの電池が少なくなったら知らせる（少なくなった瞬間に1回）
	bool ctrl_dying = ps3ctrl.isControllerBatteryDying();
	if (ctrl_dying && !last_ctrl_batt_dying) {
		ui_events.push(EV_CTRL_BATT_DYING);
	}
	last_ctrl_batt_dying = ctrl_dying;

	//制御タスクからの通知を処理する（printf や NVS への書き込みは制御タスクでは行わない）
	uint8_t ev;
	while (ui_events.pop(ev)) {
		feedbackForEvent(ev, now_ms);
		switch (ev) {
			case EV_TUNE_CHANGED:
				printGains();
				break;
			case EV_MODE_BEGINNER:
			case EV_MODE_NORMAL:
				printf("[ui] %s\n", uiEventStr(ev));
				saveUiSettings();		//ディスアーム中だけ切り替わるので、ここで保存してよい
				break;
			case EV_SAVE_REQUEST:
				//NVS書き込み中はフラッシュアクセスで両コアが止まることがあるため、ディスアーム中だけ実行する
				if (!armed) {
					saveGains();
				} else {
					printf("[gain] %s\n", uiEventStr(EV_SAVE_REJECT_ARMED));
				}
				break;
			default:
				printf("[ui] %s\n", uiEventStr(ev));
				break;
		}
	}

	bool is_armed = armed;
	if (is_armed != last_armed_led) {
		ledctrl.setArmIndicator(is_armed);
		last_armed_led = is_armed;
	}

	//コントローラへの通知を進める
	//  START を押している間は弱く振動（長押しの合図）
	//  プレイヤーLED: 飛行モードは 通常=1 / 初心者=2、調整モードは選んでいる項目の番号（1〜6）
	int player = tune_mode ? (int)tune_item + 1 : (beginner ? 2 : 1);
	ps3ctrl.serviceFeedback(now_ms, ps3ctrl.btnHeld(BTN_START), player);

	//調整モードで選んでいる項目をLEDに反映する（変化したときだけ更新）
	int mode = tune_mode ? (int)tune_item : -1;
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

//出来事に応じてコントローラを振動させる（#8 の 4.）
//  1回短く: アームした・調整モードの切り替え
//  1回長く: ディスアームした
//  2回短く: 断られた・上限/下限
//  3回短く: コントローラの電池が少ない
//  軽く1回: ゲインの変更・項目の選択
void DroneController::feedbackForEvent(uint8_t ev, uint32_t now_ms) {
	switch (ev) {
		case EV_ARMED:
		case EV_MODE_BEGINNER:
		case EV_MODE_NORMAL:
		case EV_TUNE_ON:
		case EV_TUNE_OFF:
		case EV_TUNE_RESET:
			ps3ctrl.playRumble(1, FB_SHORT_MS, FB_GAP_MS, FB_RUMBLE_INTENSITY, now_ms);
			break;
		case EV_DISARM_BUTTON:
		case EV_DISARM_TILT:
		case EV_DISARM_IMU:
		case EV_DISARM_AUTO:
		case EV_DISARM_KILL:
		case EV_DISARM_LANDED:
			ps3ctrl.playRumble(1, FB_LONG_MS, FB_GAP_MS, FB_RUMBLE_INTENSITY, now_ms);
			break;
		case EV_REJECT_LINK:
		case EV_REJECT_THROTTLE:
		case EV_REJECT_TILT:
		case EV_REJECT_CALIB:
		case EV_REJECT_CTRL_BATT:
		case EV_TUNE_REJECT_ARMED:
		case EV_TUNE_LIMIT:
		case EV_SAVE_REJECT_ARMED:
		case EV_MODE_REJECT_ARMED:
		case EV_HINT_HOLD_ARM:
		case EV_HINT_HOLD_DISARM:
			ps3ctrl.playRumble(2, FB_SHORT_MS, FB_GAP_MS, FB_RUMBLE_INTENSITY, now_ms);
			break;
		case EV_CTRL_BATT_DYING:
			ps3ctrl.playRumble(3, FB_SHORT_MS, FB_GAP_MS, FB_RUMBLE_INTENSITY, now_ms);
			break;
		case EV_TUNE_CHANGED:
			ps3ctrl.playRumble(1, FB_TICK_MS, FB_GAP_MS, FB_RUMBLE_INTENSITY / 2, now_ms);
			break;
		default:
			//通信断で止まったときは、コントローラに届かないので振動しない
			//TODO: バッテリー低下の振動は、電圧測定の修正（#5）のあとで追加する
			break;
	}
}

//==========================================================
//PIDゲイン管理
//==========================================================

//調整項目の表（名前、NVSのキー、初期値、最小の増減幅、上限）
struct TuneSpec {
	const char* name;
	const char* nvs_key;
	float def;
	float step_min;
	float max;
};
static const TuneSpec TUNE_SPECS[] = {
	{ "rate P",  "rp_kp",  RATE_KP_DEFAULT,     KP_STEP_MIN,       KP_MAX       },
	{ "rate I",  "rp_ki",  RATE_KI_DEFAULT,     KI_STEP_MIN,       KI_MAX       },
	{ "rate D",  "rp_kd",  RATE_KD_DEFAULT,     KD_STEP_MIN,       KD_MAX       },
	{ "angle P", "ang_kp", ANGLE_KP_DEFAULT,    ANGLE_KP_STEP_MIN, ANGLE_KP_MAX },
	{ "yaw P",   "yaw_kp", YAW_RATE_KP_DEFAULT, KP_STEP_MIN,       KP_MAX       },
	{ "yaw I",   "yaw_ki", YAW_RATE_KI_DEFAULT, KI_STEP_MIN,       KI_MAX       },
};
static_assert(sizeof(TUNE_SPECS) / sizeof(TUNE_SPECS[0]) == 6, "TUNE_SPECS と TuneItem の数を合わせること");

//ボタン操作を処理する（制御タスクから毎周期呼ぶ）
void DroneController::handleButtons(uint32_t now_ms) {
	(void)now_ms;

	//緊急停止: L1 + R1 同時押しで、どの状態でもすぐ止める
	bool kill_combo = ps3ctrl.btnHeld(BTN_L1) && ps3ctrl.btnHeld(BTN_R1);
	if (kill_combo && !kill_combo_prev) {
		arm_req = ARM_REQ_KILL;
	}
	kill_combo_prev = kill_combo;

	//START
	//  長押し: ディスアーム中ならアーム、アーム中ならディスアーム（飛行中でも可）
	//  短押し: スロットルが低い（モータが止まっている）ときだけディスアーム
	//          飛行中に1回押しただけでは止まらない（誤操作での墜落を防ぐ）
	uint8_t start_ev = ps3ctrl.btnEvent(BTN_START);
	if (arm_req != ARM_REQ_KILL) {
		if (start_ev & ButtonTracker::LONG) {
			arm_req = armed ? ARM_REQ_DISARM : ARM_REQ_ARM;
		} else if (start_ev & ButtonTracker::SHORT) {
			if (!armed) {
				ui_events.push(EV_HINT_HOLD_ARM);
			} else if (throttle < THROTTLE_MIN_CONTROL) {
				arm_req = ARM_REQ_DISARM;
			} else {
				ui_events.push(EV_HINT_HOLD_DISARM);
			}
		}
	}

	//SELECT 長押し: 調整モードの切り替え（入るのはディスアーム中だけ。出るのはいつでも）
	if (ps3ctrl.btnEvent(BTN_SELECT) & ButtonTracker::LONG) {
		if (tune_mode) {
			tune_mode = false;
			ui_events.push(EV_TUNE_OFF);
		} else if (!armed) {
			tune_mode = true;
			ui_events.push(EV_TUNE_ON);
			ui_events.push(EV_TUNE_CHANGED);	//今の値を表示する
		} else {
			ui_events.push(EV_TUNE_REJECT_ARMED);
		}
	}

	//飛行モードでは、ここから下のボタン（十字キーなど）は何もしない（誤操作の防止）
	if (!tune_mode) {
		//△ 短押し: 初心者モード／通常モードの切り替え（ディスアーム中だけ）
		if (ps3ctrl.btnEvent(BTN_TRIANGLE) & ButtonTracker::SHORT) {
			if (armed) {
				ui_events.push(EV_MODE_REJECT_ARMED);
			} else {
				beginner = !beginner;
				ui_events.push(beginner ? EV_MODE_BEGINNER : EV_MODE_NORMAL);
			}
		}
		return;
	}

	//SELECT 短押し: 保存（ディスアーム中だけ）
	if (ps3ctrl.btnEvent(BTN_SELECT) & ButtonTracker::SHORT) {
		ui_events.push(armed ? EV_SAVE_REJECT_ARMED : EV_SAVE_REQUEST);
	}
	//十字キー左右: 項目を選ぶ
	if (ps3ctrl.btnEvent(BTN_RIGHT) & ButtonTracker::PRESSED) {
		tune_item = (uint8_t)((tune_item + 1) % TUNE_NUM);
		ui_events.push(EV_TUNE_CHANGED);
	}
	if (ps3ctrl.btnEvent(BTN_LEFT) & ButtonTracker::PRESSED) {
		tune_item = (uint8_t)((tune_item + TUNE_NUM - 1) % TUNE_NUM);
		ui_events.push(EV_TUNE_CHANGED);
	}
	//十字キー上下: 増減
	if (ps3ctrl.btnEvent(BTN_UP) & ButtonTracker::PRESSED) {
		stepTune(+1);
	}
	if (ps3ctrl.btnEvent(BTN_DOWN) & ButtonTracker::PRESSED) {
		stepTune(-1);
	}
	//△ 長押し: 選んでいる項目を初期値に戻す
	if (ps3ctrl.btnEvent(BTN_TRIANGLE) & ButtonTracker::LONG) {
		resetTuneItem();
	}
}

//選んでいる項目を、現在値の GAIN_STEP_RATIO 倍（最小 step_min）だけ増減する
//0〜上限に制限する（負のゲインは正帰還になって発散するので禁止）
void DroneController::stepTune(int dir) {
	const TuneSpec &sp = TUNE_SPECS[tune_item];
	float v = tune_val[tune_item];
	float step = v * GAIN_STEP_RATIO;
	if (step < sp.step_min) step = sp.step_min;
	float nv = v + ((dir > 0) ? step : -step);
	if (nv < 0.0f)   nv = 0.0f;
	if (nv > sp.max) nv = sp.max;

	if (nv == v) {
		ui_events.push(EV_TUNE_LIMIT);		//上限・下限でこれ以上変わらない
		return;
	}
	tune_val[tune_item] = nv;
	applyTuneValues();
	ui_events.push(EV_TUNE_CHANGED);
}

void DroneController::resetTuneItem() {
	tune_val[tune_item] = TUNE_SPECS[tune_item].def;
	applyTuneValues();
	ui_events.push(EV_TUNE_RESET);
	ui_events.push(EV_TUNE_CHANGED);
}

//調整値を各制御器に反映する（制御タスクか、制御タスク起動前の init() から呼ぶ）
void DroneController::applyTuneValues() {
	pid_roll_rate.setGains(tune_val[TUNE_RATE_KP], tune_val[TUNE_RATE_KI], tune_val[TUNE_RATE_KD]);
	pid_pitch_rate.setGains(tune_val[TUNE_RATE_KP], tune_val[TUNE_RATE_KI], tune_val[TUNE_RATE_KD]);
	angle_kp = tune_val[TUNE_ANGLE_KP];
	pid_yaw_rate.setGains(tune_val[TUNE_YAW_KP], tune_val[TUNE_YAW_KI], YAW_RATE_KD);
}

//==========================================================
//飛行モードの設定（通常／初心者）
//==========================================================

const DroneController::FlightProfile& DroneController::profile() const {
	static const FlightProfile NORMAL   = { MAX_ANGLE_DEG,          MAX_YAW_RATE_DPS,          THROTTLE_MAX,          THROTTLE_RATE };
	static const FlightProfile BEGINNER = { BEGINNER_MAX_ANGLE_DEG, BEGINNER_MAX_YAW_RATE_DPS, BEGINNER_THROTTLE_MAX, BEGINNER_THROTTLE_RATE };
	return beginner ? BEGINNER : NORMAL;
}

void DroneController::loadUiSettings() {
	Preferences prefs;
	if (prefs.begin("ui", true)) {
		beginner = prefs.getBool("beginner", false);
		prefs.end();
	}
	printf("[ui] %s\n", uiEventStr(beginner ? EV_MODE_BEGINNER : EV_MODE_NORMAL));
}

void DroneController::saveUiSettings() {
	Preferences prefs;
	if (!prefs.begin("ui", false)) {
		printf("[ui] save failed: NVS open error\n");
		return;
	}
	prefs.putBool("beginner", beginner);
	prefs.end();
}

//保存値が壊れている（NaN・負・上限超え）ときは初期値を使う
static float validGain(float v, float max_v, float def) {
	if (isnan(v) || v < 0.0f || v > max_v) return def;
	return v;
}

void DroneController::loadGains() {
	Preferences prefs;
	bool opened = prefs.begin("pid", true);		//読み取り専用。保存値がなければ失敗する
	for (int i = 0; i < TUNE_NUM; i++) {
		const TuneSpec &sp = TUNE_SPECS[i];
		tune_val[i] = opened ? validGain(prefs.getFloat(sp.nvs_key, sp.def), sp.max, sp.def) : sp.def;
	}
	if (opened) prefs.end();
	applyTuneValues();
	printGains();
}

void DroneController::saveGains() {
	Preferences prefs;
	if (!prefs.begin("pid", false)) {
		printf("[gain] save failed: NVS open error\n");
		return;
	}
	for (int i = 0; i < TUNE_NUM; i++) {
		prefs.putFloat(TUNE_SPECS[i].nvs_key, tune_val[i]);
	}
	prefs.end();
	printf("[gain] saved\n");
	printGains();
}

void DroneController::printGains() {
	printf("[gain] rate P:%.3f I:%.3f D:%.4f / angle P:%.2f / yaw P:%.3f I:%.3f",
		tune_val[TUNE_RATE_KP], tune_val[TUNE_RATE_KI], tune_val[TUNE_RATE_KD],
		tune_val[TUNE_ANGLE_KP], tune_val[TUNE_YAW_KP], tune_val[TUNE_YAW_KI]);
	if (tune_mode) {
		printf("  [selected: %s]", TUNE_SPECS[tune_item].name);
	}
	printf("\n");
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
		} else if (checkLanded(now_ms)) {
			disarm(EV_DISARM_LANDED);						//着陸した
		} else if (base_thrust <= 0.0f) {
			if ((uint32_t)(now_ms - zero_thr_since_ms) > AUTO_DISARM_MS) {
				disarm(EV_DISARM_AUTO);						//スロットル0のまま放置
			}
		} else {
			zero_thr_since_ms = now_ms;						//最後にスロットルが入っていた時刻
		}
	}

	switch (arm_req) {
		case ARM_REQ_KILL:
			if (armed) disarm(EV_DISARM_KILL);
			break;
		case ARM_REQ_DISARM:
			if (armed) disarm(EV_DISARM_BUTTON);
			break;
		case ARM_REQ_ARM:
			if (!armed) tryArm(now_ms);
			break;
		default:
			break;
	}
	arm_req = ARM_REQ_NONE;
}

//着陸の自動検出（アーム中に毎周期呼ぶ）
//一度飛んだあと、モータが止まっていて・傾きが小さく・角速度が小さい状態が
//LAND_DETECT_MS 続いたら true。アーム直後（まだ飛んでいない）は判定しない
bool DroneController::checkLanded(uint32_t now_ms) {
	if (base_thrust >= THROTTLE_MIN_CONTROL) {
		has_flown = true;
	}

	bool still = has_flown
			&& base_thrust < THROTTLE_MIN_CONTROL
			&& fabsf(madgwick_user.getRoll())  < LAND_MAX_TILT_DEG
			&& fabsf(madgwick_user.getPitch()) < LAND_MAX_TILT_DEG
			&& fabsf(imu.gx) < LAND_MAX_RATE_DPS
			&& fabsf(imu.gy) < LAND_MAX_RATE_DPS
			&& fabsf(imu.gz) < LAND_MAX_RATE_DPS;

	if (!still) {
		landed_since_ms = 0;
		return false;
	}
	if (landed_since_ms == 0) {
		landed_since_ms = now_ms;
		return false;
	}
	return (uint32_t)(now_ms - landed_since_ms) >= LAND_DETECT_MS;
}

//アームできる条件をすべて満たしたときだけアームする
void DroneController::tryArm(uint32_t now_ms) {
	float roll  = madgwick_user.getRoll();
	float pitch = madgwick_user.getPitch();

	if (!calib_ok) {
		ui_events.push(EV_REJECT_CALIB);
	} else if (!ps3ctrl.isLinkAlive()) {
		ui_events.push(EV_REJECT_LINK);
	} else if (ps3ctrl.isControllerBatteryDying()) {
		ui_events.push(EV_REJECT_CTRL_BATT);				//コントローラの電池が少ないと通信が切れやすい
	} else if (ps3ctrl.getThrottleStick() > 0.0f) {
		ui_events.push(EV_REJECT_THROTTLE);					//アームした瞬間にスロットルが上がり始めないよう、スティックを上に倒していないときだけ
	} else if (fabsf(roll) > ARM_MAX_TILT_DEG || fabsf(pitch) > ARM_MAX_TILT_DEG) {
		ui_events.push(EV_REJECT_TILT);
	} else {
		resetControllers();
		zero_thr_since_ms = now_ms;		//スロットル0放置タイマーの起点
		has_flown = false;
		landed_since_ms = 0;
		armed = true;
		ui_events.push(EV_ARMED);
	}
}

void DroneController::disarm(UiEvent reason) {
	armed = false;
	throttle = 0;
	resetControllers();
	stopMotors();
	ui_events.push(reason);
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

const char* DroneController::uiEventStr(uint8_t ev) {
	switch (ev) {
		case EV_ARMED:				return "ARMED";
		case EV_DISARM_BUTTON:		return "disarmed (button)";
		case EV_DISARM_LINK:		return "disarmed (link lost)";
		case EV_DISARM_TILT:		return "disarmed (tilt over limit)";
		case EV_DISARM_IMU:			return "disarmed (IMU error)";
		case EV_DISARM_AUTO:		return "disarmed (idle timeout)";
		case EV_DISARM_KILL:		return "disarmed (EMERGENCY STOP)";
		case EV_DISARM_LANDED:		return "disarmed (landed)";
		case EV_REJECT_LINK:		return "arm rejected: no controller link";
		case EV_REJECT_THROTTLE:	return "arm rejected: throttle stick is pushed up";
		case EV_REJECT_TILT:		return "arm rejected: not level";
		case EV_REJECT_CALIB:		return "arm rejected: calibration failed";
		case EV_REJECT_CTRL_BATT:	return "arm rejected: controller battery is low";
		case EV_CTRL_BATT_DYING:	return "controller battery is low - charge it";
		case EV_HINT_HOLD_ARM:		return "hold START for 1s to arm";
		case EV_HINT_HOLD_DISARM:	return "flying: hold START for 1s to disarm (L1+R1 = emergency stop)";
		case EV_TUNE_ON:			return "tuning mode ON";
		case EV_TUNE_OFF:			return "tuning mode OFF (flight mode)";
		case EV_TUNE_REJECT_ARMED:	return "tuning mode rejected: disarm first";
		case EV_TUNE_LIMIT:			return "gain is at its limit";
		case EV_TUNE_RESET:			return "gain reset to default";
		case EV_SAVE_REJECT_ARMED:	return "save rejected: disarm first";
		case EV_MODE_BEGINNER:		return "BEGINNER mode (gentle: smaller angles, slower throttle)";
		case EV_MODE_NORMAL:		return "NORMAL mode";
		case EV_MODE_REJECT_ARMED:	return "mode change rejected: disarm first";
		default:					return "?";
	}
}

//増減式スロットル（#8）
//左スティック上下の倒し量で変化の速さを決め、中央なら今の値を保つ
//飛行モード中に十字キー下を押している間は、速く下げる
//ディスアーム中は常に0
void DroneController::updateThrottle(float dt) {
	if (!armed) {
		throttle = 0;
		return;
	}
	const FlightProfile &prof = profile();
	float in = ps3ctrl.getThrottleStick();
	float rate = in * fabsf(in) * prof.throttle_rate;	//2乗カーブ（符号は保つ）
	if (!tune_mode && ps3ctrl.btnHeld(BTN_DOWN)) {
		rate = -prof.throttle_rate * THROTTLE_FAST_DESCEND_MUL;
	}
	throttle += rate * dt;
	if (throttle < 0.0f)              throttle = 0.0f;
	if (throttle > prof.throttle_max) throttle = prof.throttle_max;
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
	//ボタン操作（アーム・調整モードなど）を反映する
	handleButtons(now_ms);

	//スロットルを更新する（増減式）
	updateThrottle(dt);
	base_thrust	= throttle;

	//アーム/ディスアームの判定（フェイルセーフを含む）。ディスアームするとスロットルは0に戻る
	updateArmState(now_ms);
	base_thrust	= throttle;

	//ディスアーム中、またはスロットルが低いときはモータを止めてPIDをリセットする
	//（地上でI項が溜まって、離陸の瞬間に傾くのを防ぐ）
	if (!armed || base_thrust < THROTTLE_MIN_CONTROL) {
		resetControllers();
		stopMotors();
		return;
	}

	//コントローラから目標の角度を取得して制御する（符号は flight_config.h、最大角はモードで決める）
	const FlightProfile &prof = profile();
	base_roll	= ps3ctrl.getRollStick()  * prof.max_angle_deg;
	base_pitch	= ps3ctrl.getPitchStick() * prof.max_angle_deg;

	//外側ループ: 角度誤差 → 目標角速度（P制御）
	roll_rate_sp	= rateLimit(angle_kp * (base_roll  - madgwick_user.getRoll()));
	pitch_rate_sp	= rateLimit(angle_kp * (base_pitch - madgwick_user.getPitch()));

	//内側ループ: 目標角速度とジャイロの角速度でPID
	//（Madgwick の Roll はジャイロX軸、Pitch はジャイロY軸まわりの回転で増える）
	roll_output		= pid_roll_rate.compute(roll_rate_sp, imu.gx, dt);
	pitch_output	= pid_pitch_rate.compute(pitch_rate_sp, imu.gy, dt);

	//ヨー: スティックの目標角速度とジャイロZでPID（中立なら回転を止める）
	yaw_rate_sp		= ps3ctrl.getYawStick() * prof.max_yaw_rate_dps;
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
	printf("%.0f,",base_thrust);
	printf("%.1f,",roll_output);
	printf("%.1f,",pitch_output);
	printf("\n");
}

void DroneController::debug_base_rp(){
	printf("%.0f\t,",base_thrust);
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

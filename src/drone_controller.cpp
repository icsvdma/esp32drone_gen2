#include "motor.h"
#include "mpu6050.h"
#include "pid_controller.h"
#include "drone_controller.h"
#include "madgwick_filter.h"
#include "define_system.h"

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

	//センサのフィルタを使用するために初期化を行う
	madgwick_user.init();

	//コントローラの初期化
	ps3ctrl.init();
	
	//シリアルLEDの初期化	
	ledctrl.init();
	//シリアルLED全消灯
	ledctrl.allClear();
	//初期化処理完了をLEDでお知らせ
	ledctrl.setRGBcolor(0,10,20);

	//battery ADCの初期化
	batt_adc.init();

	//バッテリ電圧監視タスクの起動
	batt_adc.start();

	//センサ値取得タスクの起動
	mpu_user.start();
}

//Droneクラス実行処理
void DroneController::update(float dt) {
	
	//フィルタの処理を行う
	// madgwick_user.update();
	
	//TODO:割り込み処理で優先度マックスで動作させる
	//コントローラの制御状態を更新する
	ps3ctrl.update();

	//PIDパラメータの選択状態を取得してLEDに反映する
	ledctrl.updatePidMode(ps3ctrl.getPidAly());

	//コントローラから推力を取得して制御する
	base_thrust	= ps3ctrl.getThrust();
	//コントローラから目標の角度を取得して制御する
	base_roll	= -1 * ps3ctrl.getRoll();
	base_pitch	= -1 * ps3ctrl.getPitch();

	//引数:目標値、現在角度、取得周期
	roll_output = (int16_t)pid_roll.compute(base_roll, madgwick_user.getRoll(), dt);
	pitch_output = (int16_t)pid_pitch.compute(base_pitch, madgwick_user.getPitch(), dt);

#ifdef DEBUG_MODE
	//mpu_user.debug_sensor();
	//madgwick.debug_madrpy();
	//debug_rpy();
	//debug_base_rp();
	//debug_angle();
	//ps3ctrl.debug_raw_param();
	//ps3ctrl.debug_target_param();
	debug_motor_param();
#else
	m1.set_speed(m1.clip_param(base_thrust - roll_output - pitch_output));  // front-right		MOT_FR_OUT
	m2.set_speed(m2.clip_param(base_thrust + roll_output - pitch_output));  // front-left		MOT_FL_OUT
	m3.set_speed(m3.clip_param(base_thrust - roll_output + pitch_output));  // back-right		MOT_RR_OUT
	m4.set_speed(m4.clip_param(base_thrust + roll_output + pitch_output));  // back-left		MOT_RL_OUT
#endif
}


//==========================================================
//debug用関数群（シリアル出力）
//==========================================================

void DroneController::debug_rpy(){
	printf("%d,",base_thrust);
	printf("%d,",roll_output);
	printf("%d,",pitch_output);
	printf("\n");
}

void DroneController::debug_base_rp(){
	printf("%d\t,",base_thrust);
	printf("%d\t,",base_roll);
	printf("%d\t,",base_pitch);
	printf("\n");
}

void DroneController::debug_angle(){
	printf("%f\t,",angle_x);
	printf("%f\t,",angle_y);
	printf("\n");
}

void DroneController::debug_motor_param(){
	int16_t m1_param = (m1.clip_param(base_thrust + roll_output + pitch_output));
	int16_t m2_param = (m2.clip_param(base_thrust - roll_output + pitch_output));
	int16_t m3_param = (m3.clip_param(base_thrust + roll_output - pitch_output));
	int16_t m4_param = (m4.clip_param(base_thrust - roll_output - pitch_output));
	
	printf("%d\t,",m1_param);
	printf("%d\t,",m2_param);
	printf("%d\t,",m3_param);
	printf("%d\t",m4_param);
	printf("\n");
} 
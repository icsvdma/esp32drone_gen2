
# include "Arduino.h"
# include "Ps3Controller.h"
# include "ps3_ctrl.h"
# include "flight_config.h"

//PS3Controller usr_ps3ctrl;

//TODO:ソース、ヘッダ共に名称は「flightManage.xxxx」に変更する

// static メンバの初期化
PS3Controller* PS3Controller::instance = nullptr;

void PS3Controller::init(){
	instance = this;
	cmd_queue = xQueueCreate(16, sizeof(Ps3Command));

	Ps3.attach((void (*)())PS3Controller::staticNotify);
	//Ps3.attachOnConnect(PS3Controller::staticOnConnect);
	
	//esp32 bluetooth address
	Ps3.begin("30:AE:A4:9C:2E:76");	
}

void PS3Controller::onConnect() {
	printf("PS3 Controller Connected!");
}	

//通信状態を判定して入力を更新する（制御タスクから呼ぶ）
//Ps3.isConnected() は電波が途切れても false にならないことがあるため、
//最後にパケットを受信してからの経過時間でも判定する（コールバックは受信パケットごとに呼ばれる）
void PS3Controller::update(uint32_t now_ms){
	uint32_t last = last_packet_ms;
	link_alive = Ps3.isConnected() && (last != 0) && ((uint32_t)(now_ms - last) < LINK_TIMEOUT_MS);

	if (link_alive) {

		ly = Ps3.data.analog.stick.ly;
		throttle_target = mapTarget_th((-1 * ly), 0, 128, 0, 1024); // 仮のスロットル範囲 1024は制御マックス値なのでマージンが必要

		// 右スティック上下 → ピッチ
		ry = Ps3.data.analog.stick.ry;
		pitch_target = PITCH_STICK_DIR * stickNorm(ry) * MAX_ANGLE_DEG; // -30度～+30度

		// 右スティック左右 → ロール
		rx = Ps3.data.analog.stick.rx;
		roll_target = ROLL_STICK_DIR * stickNorm(rx) * MAX_ANGLE_DEG; // -30度～+30度

		// 左スティック左右 → ヨー角速度
		lx = Ps3.data.analog.stick.lx;
		yaw_rate_target = YAW_STICK_DIR * stickNorm(lx) * MAX_YAW_RATE_DPS;
	}
	else{
		//通信断: 最後の入力を保持せず、すべて0にする
		throttle_target	= 0;
		pitch_target	= 0;
		roll_target		= 0;
		yaw_rate_target	= 0;
	}
}

bool PS3Controller::isLinkAlive(){
	return link_alive;
}

uint16_t PS3Controller::mapTarget_th(int x , int in_min , int in_max , int out_min , int out_max){
	static uint16_t target_data;

	if(x < 0){
		return 0;
	}
	else {
		target_data = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
		return target_data;
	}
}

//スティックの生値(-128〜127)を -1.0〜+1.0 に正規化する
//中立付近(STICK_DEADBAND以内)は厳密に0にし、不感帯の外側から0→1へ連続的に変化させる
//（以前の整数マッピングは中立で -1 になり、常に-1度の指令が出ていた）
float PS3Controller::stickNorm(int8_t raw){
	int v = raw;
	int mag = (v < 0) ? -v : v;
	if (mag <= STICK_DEADBAND) {
		return 0.0f;
	}
	float s = (float)(mag - STICK_DEADBAND) / (127.0f - STICK_DEADBAND);
	if (s > 1.0f) s = 1.0f;		//-128 のとき
	return (v < 0) ? -s : s;
}

uint16_t PS3Controller::getThrust(){
	return throttle_target;
}
float PS3Controller::getRoll(){
	return roll_target;
}
float PS3Controller::getPitch(){
	return 	pitch_target;
}

float PS3Controller::getYawRate(){
	return yaw_rate_target;
}

void PS3Controller::debug_raw_param(){
	printf("lx_raw : %d \t",lx);
	printf("ly_raw : %d \t",ly);
	printf("ry_raw : %d \t",ry);
	printf("rx_raw : %d \n",rx);
}

void PS3Controller::debug_target_param(){
	printf("target_th : %d \t",throttle_target);
	printf("target_roll : %.2f \t",roll_target);
	printf("target_pitch : %.2f \t",pitch_target);
	printf("target_yaw_rate : %.1f \n",yaw_rate_target);
}

void PS3Controller::notify() {
	//受信時刻を記録（通信断の検出に使う）
	last_packet_ms = millis();

	if(Ps3.event.button_down.right){
		//pid_aly += 1;
		pid_aly = (pid_aly + 1) % 3;
	}
	if(Ps3.event.button_down.left){
		//pid_aly -= 1;
		pid_aly = (pid_aly + 2) % 3;	//0 + 2 % 3 = 2
		
	}
	//ゲインの変更は制御タスクで行う（ここではコマンドを送るだけ）
	if(Ps3.event.button_down.up){
		pushCommand(PS3_CMD_GAIN_UP);
	}
	if(Ps3.event.button_down.down){
		pushCommand(PS3_CMD_GAIN_DOWN);
	}
	//SELECT: 現在のゲインをNVSに保存（ディスアーム中だけ実行される）
	if(Ps3.event.button_down.select){
		pushCommand(PS3_CMD_SAVE_GAINS);
	}
	//START: アーム/ディスアームの切り替え
	if(Ps3.event.button_down.start){
		pushCommand(PS3_CMD_ARM_TOGGLE);
	}
}

void PS3Controller::pushCommand(Ps3CmdType type){
	if (cmd_queue == NULL) return;
	Ps3Command cmd = { type, pid_aly };
	xQueueSend(cmd_queue, &cmd, 0);	//満杯なら捨てる（待たない）
}

//制御タスクから呼ぶ。コマンドがあれば true
bool PS3Controller::popCommand(Ps3Command &cmd){
	if (cmd_queue == NULL) return false;
	return xQueueReceive(cmd_queue, &cmd, 0) == pdTRUE;
}

int PS3Controller::getPidAly(){
	return pid_aly;
}

// static な中継関数（割り込みハンドラに登録される）
void PS3Controller::staticNotify() {
	if (instance) instance->notify();
}
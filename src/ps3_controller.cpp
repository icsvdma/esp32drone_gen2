
# include "Arduino.h"
# include "Ps3Controller.h"
# include "ps3_ctrl.h"
# include "flight_config.h"

// static メンバの初期化
PS3Controller* PS3Controller::instance = nullptr;

void PS3Controller::init(){
	instance = this;

	Ps3.attach((void (*)())PS3Controller::staticNotify);
	//Ps3.attachOnConnect(PS3Controller::staticOnConnect);
	
	//esp32 bluetooth address
	Ps3.begin("30:AE:A4:9C:2E:76");	
}

//通信状態を判定して入力を更新する（制御タスクから呼ぶ）
//Ps3.isConnected() は電波が途切れても false にならないことがあるため、
//最後にパケットを受信してからの経過時間でも判定する（コールバックは受信パケットごとに呼ばれる）
void PS3Controller::update(uint32_t now_ms){
	uint32_t last = last_packet_ms;
	link_alive = Ps3.isConnected() && (last != 0) && ((uint32_t)(now_ms - last) < LINK_TIMEOUT_MS);

	if (link_alive) {

		// 左スティック上下 → スロットルを変える速さ（ly は上に倒すと負になるので符号を反転）
		ly = Ps3.data.analog.stick.ly;
		throttle_stick = -stickNorm(ly);

		// 右スティック上下 → ピッチ
		ry = Ps3.data.analog.stick.ry;
		pitch_stick = PITCH_STICK_DIR * stickNorm(ry);

		// 右スティック左右 → ロール
		rx = Ps3.data.analog.stick.rx;
		roll_stick = ROLL_STICK_DIR * stickNorm(rx);

		// 左スティック左右 → ヨー角速度
		lx = Ps3.data.analog.stick.lx;
		yaw_stick = YAW_STICK_DIR * stickNorm(lx);

		ctrl_battery = (uint8_t)Ps3.data.status.battery;

		updateButtons(now_ms);
	}
	else{
		//通信断: 最後の入力を保持せず、すべて0にする
		throttle_stick	= 0;
		pitch_stick		= 0;
		roll_stick		= 0;
		yaw_stick		= 0;
		//ボタンも「離した」扱いにする（離したことによる短押しイベントは出さない）
		clearButtons();
	}
}

//ボタンの押下状態を読んでイベントを判定する
//Ps3.data は Bluetooth タスクがまるごとコピーで更新する。読み取り中に更新されても、
//各ビットは前後どちらかの受信データの値なので、存在しない押下が出ることはない
void PS3Controller::updateButtons(uint32_t now_ms){
	const bool held[BTN_NUM] = {
		(bool)Ps3.data.button.start,
		(bool)Ps3.data.button.select,
		(bool)Ps3.data.button.triangle,
		(bool)Ps3.data.button.circle,
		(bool)Ps3.data.button.up,
		(bool)Ps3.data.button.down,
		(bool)Ps3.data.button.left,
		(bool)Ps3.data.button.right,
		(bool)Ps3.data.button.l1,
		(bool)Ps3.data.button.r1,
	};
	for (int i = 0; i < BTN_NUM; i++) {
		btn_ev[i] = btn[i].update(held[i], now_ms, BUTTON_LONG_MS);
	}
}

void PS3Controller::clearButtons(){
	for (int i = 0; i < BTN_NUM; i++) {
		btn[i].reset();
		btn_ev[i] = ButtonTracker::NONE;
	}
}

bool PS3Controller::isLinkAlive(){
	return link_alive;
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

float PS3Controller::getThrottleStick(){
	return throttle_stick;
}
float PS3Controller::getRollStick(){
	return roll_stick;
}
float PS3Controller::getPitchStick(){
	return pitch_stick;
}
float PS3Controller::getYawStick(){
	return yaw_stick;
}

void PS3Controller::debug_raw_param(){
	printf("lx_raw : %d \t",lx);
	printf("ly_raw : %d \t",ly);
	printf("ry_raw : %d \t",ry);
	printf("rx_raw : %d \n",rx);
}

void PS3Controller::debug_target_param(){
	printf("throttle_stick : %.2f \t",throttle_stick);
	printf("roll_stick : %.2f \t",roll_stick);
	printf("pitch_stick : %.2f \t",pitch_stick);
	printf("yaw_stick : %.2f \n",yaw_stick);
}

bool PS3Controller::isControllerBatteryDying(){
	uint8_t b = ctrl_battery;
	return b == ps3_status_battery_shutdown || b == ps3_status_battery_dying;
}

//==========================================================
//コントローラへの通知（振動・プレイヤーLED）
//esp32-ps3 の仕様（ライブラリのソースで確認）
//  setRumble(強さ 0〜100, 時間 0〜5000 / -1 で止めるまで)
//  setPlayer(番号): 0 = 全消灯、1〜4 = LED1〜4、5〜10 = 組み合わせ。接続時にライブラリが 1 にする
//  setPlayer() は振動を 0 にした命令を送るので、振動中には送らない
//==========================================================

//振動のパターンを鳴らす（短い振動を pulses 回）
//時間の単位がライブラリで明記されていないため、止めるときも強さ0の命令を明示的に送る
void PS3Controller::playRumble(uint8_t pulses, uint16_t on_ms, uint16_t off_ms, uint8_t intensity, uint32_t now_ms){
	rumble_pat.pulses_left	= pulses;
	rumble_pat.on_ms		= on_ms;
	rumble_pat.off_ms		= off_ms;
	rumble_pat.intensity	= intensity;
	rumble_pat.on			= false;
	rumble_pat.next_ms		= now_ms;
	hold_rumble_on			= false;	//押し続けている間の弱い振動は、パターンが終わってから出し直す
}

//service() から周期的に呼ぶ
//  hold_rumble : true の間は弱く振動し続ける（START 長押し中の合図）
//  player      : 表示したいプレイヤー番号
void PS3Controller::serviceFeedback(uint32_t now_ms, bool hold_rumble, int player){
	if (!link_alive) {
		//切断中は何も送らない。再接続したら送り直す
		rumble_pat.pulses_left = 0;
		rumble_pat.on = false;
		hold_rumble_on = false;
		cur_player = -1;
		return;
	}

	//振動パターン（他の命令より優先。パターン中は他の命令を送らない）
	if (rumble_pat.pulses_left > 0 || rumble_pat.on) {
		if ((int32_t)(now_ms - rumble_pat.next_ms) >= 0) {
			if (rumble_pat.on) {
				Ps3.setRumble(0, 0);
				rumble_pat.on = false;
				rumble_pat.next_ms = now_ms + rumble_pat.off_ms;
			} else {
				Ps3.setRumble(rumble_pat.intensity, rumble_pat.on_ms);
				rumble_pat.on = true;
				rumble_pat.pulses_left--;
				rumble_pat.next_ms = now_ms + rumble_pat.on_ms;
			}
		}
		return;
	}

	//押し続けている間の弱い振動
	if (hold_rumble != hold_rumble_on) {
		if (hold_rumble) {
			Ps3.setRumble(FB_HOLD_RUMBLE_INTENSITY, -1);
		} else {
			Ps3.setRumble(0, 0);
		}
		hold_rumble_on = hold_rumble;
		return;
	}

	//プレイヤーLED（振動していないときだけ。変わったときだけ送る）
	if (!hold_rumble_on && player != cur_player) {
		Ps3.setPlayer(player);
		cur_player = player;
	}
}

//Bluetooth タスクから受信パケットごとに呼ばれる
//受信時刻を記録するだけ（通信断の検出に使う）。ボタンの処理は制御タスクの update() で行う
void PS3Controller::notify() {
	last_packet_ms = millis();
}

// static な中継関数（割り込みハンドラに登録される）
void PS3Controller::staticNotify() {
	if (instance) instance->notify();
}
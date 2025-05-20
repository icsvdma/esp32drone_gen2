
# include "Arduino.h"
# include "Ps3Controller.h"
# include "ps3_ctrl.h"
# include "pid_controller.h"

//PS3Controller usr_ps3ctrl;

//TODO:ソース、ヘッダ共に名称は「flightManage.xxxx」に変更する

// static メンバの初期化
PS3Controller* PS3Controller::instance = nullptr;

void PS3Controller::init(){
	instance = this;

	Ps3.attach((void (*)())PS3Controller::staticNotify);
	//Ps3.attachOnConnect(PS3Controller::staticOnConnect);
	
	//esp32 bluetooth address
	Ps3.begin("30:AE:A4:9C:2E:76");	
}

void PS3Controller::onConnect() {
	printf("PS3 Controller Connected!");
}	

void PS3Controller::update(){
	if (Ps3.isConnected()) {

		ly = Ps3.data.analog.stick.ly;
		throttle_target = mapTarget_th((-1 * ly), 0, 128, 0, 1024); // 仮のスロットル範囲 1024は制御マックス値なのでマージンが必要

		// 右スティック上下 → ピッチ
		ry = Ps3.data.analog.stick.ry;
		pitch_target = mapTarget((-1 * ry), -127, 128, -30, 30); // -30度～+30度

		// 右スティック左右 → ロール
		rx = Ps3.data.analog.stick.rx;
		roll_target = mapTarget((-1 * rx), -127, 128, -30, 30); // -30度～+30度
	}
	else{
		//NOP
	}
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

int16_t PS3Controller::mapTarget(int x , int in_min , int in_max , int out_min , int out_max){
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t PS3Controller::getThrust(){
	return throttle_target;
}
int16_t PS3Controller::getRoll(){
	return roll_target;
}
int16_t PS3Controller::getPitch(){
	return 	pitch_target;
}

void PS3Controller::debug_raw_param(){
	printf("ly_raw : %d \t",ly);
	printf("ry_raw : %d \t",ry);
	printf("rx_raw : %d \n",rx);
}

void PS3Controller::debug_target_param(){
	printf("target_th : %d \t",throttle_target);
	printf("target_roll : %d \t",roll_target);
	printf("target_pitch : %d \n",pitch_target);
}

void PS3Controller::notify() {
	if(Ps3.event.button_down.right){
		//pid_aly += 1;
		pid_aly = (pid_aly + 1) % 3;
	}
	if(Ps3.event.button_down.left){
		//pid_aly -= 1;
		pid_aly = (pid_aly + 2) % 3;	//0 + 2 % 3 = 2
		
	}
	if(Ps3.event.button_down.up){
		switch(pid_aly){
			case 0	:	pid_roll->set_kp(1);
						pid_pitch->set_kp(1);
						break;
			case 1	:	pid_roll->set_ki(1);
						pid_pitch->set_ki(1);
						break;
			case 2	:	pid_roll->set_kd(1);
						pid_pitch->set_kd(1);
						break;
			default :	break;
		}
	}
	if(Ps3.event.button_down.down){
		switch(pid_aly){
			case 0	:	pid_roll->set_kp(0);
						pid_pitch->set_kp(0);
						break;
			case 1	:	pid_roll->set_ki(0);
						pid_pitch->set_ki(0);
						break;
			case 2	:	pid_roll->set_kd(0);
						pid_pitch->set_kd(0);
						break;
			default :	break;
		}
	}
}

int PS3Controller::getPidAly(){
	return pid_aly;
}

// static な中継関数（割り込みハンドラに登録される）
void PS3Controller::staticNotify() {
	if (instance) instance->notify();
}
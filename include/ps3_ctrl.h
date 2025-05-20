#ifndef PS3CTRL_H
#define PS3CTRL_H

#include "pid_controller.h"

class PS3Controller {
	private:
		PIDController* pid_roll;
		PIDController* pid_pitch;

		uint16_t throttle_target= 0;
		int16_t pitch_target	= 0;
		int16_t roll_target		= 0;
	
		int8_t ly 				= 0;
		int8_t ry 				= 0;
		int8_t rx 				= 0;

		uint8_t pid_aly			= 0;

		
	public:
		//cppで割り込みを使用する場合に必要みたい
		//割り込み関数（ISR）は通常、**特定の関数シグネチャ（戻り値・引数なし）**を要求されます。
		//たとえば、void isr_handler() のような形。
		//C++の非staticメンバ関数は暗黙的にthisポインタを取るため、void MyClass::myFunc()は実際にはvoid(MyClass* this)という形になり、ISRとしては渡せません。
		static PS3Controller* instance;  // グローバルに1インスタンスのみ対応

		//drone_controllerクラスから継承する
		PS3Controller(PIDController* inst_roll , PIDController* inst_pitch) : pid_roll(inst_roll) , pid_pitch(inst_pitch) {}

		void init();
		void onConnect();
		void update();
		void debug_raw_param();
		void debug_target_param();
		uint16_t getThrust();
		int16_t getRoll();
		int16_t getPitch();
		int getPidAly();
		uint16_t mapTarget_th(int x , int in_min , int in_max , int out_min , int out_max);
		int16_t mapTarget(int x , int in_min , int in_max , int out_min , int out_max);
		void notify();

		static void staticNotify();
};

#endif // PS3CTRL_H
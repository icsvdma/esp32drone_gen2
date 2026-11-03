#ifndef PS3CTRL_H
#define PS3CTRL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// コントローラのボタン操作を制御タスクへ渡すためのコマンド
// （ボタンのコールバックは Bluetooth タスクで動くので、PIDゲインを直接書き換えない）
enum Ps3CmdType : uint8_t {
	PS3_CMD_GAIN_UP,
	PS3_CMD_GAIN_DOWN,
	PS3_CMD_SAVE_GAINS,
	PS3_CMD_ARM_TOGGLE,
};
struct Ps3Command {
	Ps3CmdType	type;
	uint8_t		term;	// 0:P 1:I 2:D
};

class PS3Controller {
	private:
		QueueHandle_t cmd_queue = NULL;
		volatile uint32_t last_packet_ms = 0;	//最後にパケットを受信した時刻（Bluetoothタスクが更新）
		bool link_alive = false;
		void pushCommand(Ps3CmdType type);

		uint16_t throttle_target= 0;
		float pitch_target		= 0;
		float roll_target		= 0;
		float yaw_rate_target	= 0;
	
		int8_t lx 				= 0;
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

		PS3Controller() {}

		void init();
		void onConnect();
		void update(uint32_t now_ms);
		bool isLinkAlive();
		void debug_raw_param();
		void debug_target_param();
		uint16_t getThrust();
		float getRoll();
		float getPitch();
		float getYawRate();
		int getPidAly();
		bool popCommand(Ps3Command &cmd);
		uint16_t mapTarget_th(int x , int in_min , int in_max , int out_min , int out_max);
		static float stickNorm(int8_t raw);
		void notify();

		static void staticNotify();
};

#endif // PS3CTRL_H
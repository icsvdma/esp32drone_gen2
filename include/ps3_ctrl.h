#ifndef PS3CTRL_H
#define PS3CTRL_H

#include <stdint.h>
#include "button_tracker.h"

// 判定に使うボタン
enum Ps3Btn : uint8_t {
	BTN_START = 0,
	BTN_SELECT,
	BTN_TRIANGLE,
	BTN_CIRCLE,
	BTN_UP,
	BTN_DOWN,
	BTN_LEFT,
	BTN_RIGHT,
	BTN_L1,
	BTN_R1,
	BTN_NUM
};

// コントローラの入力
// ボタンは制御タスクの周期ごとに押下状態を読み、ButtonTracker で
// 押した瞬間・短押し・長押しを判定する（長押しや同時押しを扱うため）
// Bluetooth のコールバックでは受信時刻の記録だけを行う
class PS3Controller {
	private:
		volatile uint32_t last_packet_ms = 0;	//最後にパケットを受信した時刻（Bluetoothタスクが更新）
		bool link_alive = false;

		float throttle_stick	= 0;	//左スティック上下（上が正、-1.0〜+1.0）
		float pitch_target		= 0;
		float roll_target		= 0;
		float yaw_rate_target	= 0;
	
		int8_t lx 				= 0;
		int8_t ly 				= 0;
		int8_t ry 				= 0;
		int8_t rx 				= 0;

		ButtonTracker btn[BTN_NUM];
		uint8_t btn_ev[BTN_NUM]	= {0};

		void updateButtons(uint32_t now_ms);
		void clearButtons();
		
	public:
		static PS3Controller* instance;  // グローバルに1インスタンスのみ対応

		PS3Controller() {}

		void init();
		void update(uint32_t now_ms);
		bool isLinkAlive();
		void debug_raw_param();
		void debug_target_param();
		float getThrottleStick();
		float getRoll();
		float getPitch();
		float getYawRate();
		static float stickNorm(int8_t raw);

		//ボタンのイベント（ButtonTracker::PRESSED / SHORT / LONG の組み合わせ）
		//update() を呼んだ周期の間だけ有効
		uint8_t btnEvent(Ps3Btn b) const { return btn_ev[b]; }
		bool btnHeld(Ps3Btn b) const { return btn[b].isHeld(); }
		uint32_t btnHeldMs(Ps3Btn b, uint32_t now_ms) const { return btn[b].heldMs(now_ms); }

		void notify();
		static void staticNotify();
};

#endif // PS3CTRL_H

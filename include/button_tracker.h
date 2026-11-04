#ifndef BUTTON_TRACKER_H
#define BUTTON_TRACKER_H

#include <stdint.h>

// ボタン1個の押下状態（押しているか）から、押した瞬間・短押し・長押しを判定する
// 制御タスクの周期ごとに update() を呼ぶ
//   PRESSED : 押した瞬間
//   SHORT   : 長押しになる前に離した瞬間
//   LONG    : 押し続けて long_ms たった瞬間（1回だけ。そのあと離しても SHORT は出ない）
class ButtonTracker {
public:
	enum : uint8_t { NONE = 0, PRESSED = 1, SHORT = 2, LONG = 4 };

	uint8_t update(bool held, uint32_t now_ms, uint32_t long_ms) {
		uint8_t ev = NONE;
		if (held && !prev) {
			press_ms = now_ms;
			long_fired = false;
			ev |= PRESSED;
		} else if (held && !long_fired && (uint32_t)(now_ms - press_ms) >= long_ms) {
			long_fired = true;
			ev |= LONG;
		} else if (!held && prev && !long_fired) {
			ev |= SHORT;
		}
		prev = held;
		return ev;
	}

	bool isHeld() const { return prev; }

	//押し続けている時間（押していなければ0）
	uint32_t heldMs(uint32_t now_ms) const {
		return prev ? (uint32_t)(now_ms - press_ms) : 0;
	}

	//イベントを出さずに「離した状態」に戻す（通信断のときに使う）
	void reset() {
		prev = false;
		long_fired = false;
	}

private:
	bool prev = false;
	bool long_fired = false;
	uint32_t press_ms = 0;
};

#endif // BUTTON_TRACKER_H

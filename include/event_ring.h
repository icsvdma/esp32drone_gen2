#ifndef EVENT_RING_H
#define EVENT_RING_H

#include <stdint.h>

// 1つのタスクが push し、別の1つのタスクが pop する、ロックなしのリングバッファ
// 制御タスク → service()（loop）への表示・通知イベントの受け渡しに使う
// 満杯のときは新しいイベントを捨てる（制御タスクを待たせない）
template <uint8_t N>
class EventRing {
public:
	void push(uint8_t ev) {
		uint8_t h = head;
		uint8_t next = (uint8_t)((h + 1) % N);
		if (next == tail) return;	//満杯
		buf[h] = ev;
		__sync_synchronize();		//データを書いてから head を進める
		head = next;
	}

	bool pop(uint8_t &ev) {
		uint8_t t = tail;
		if (t == head) return false;	//空
		ev = buf[t];
		__sync_synchronize();
		tail = (uint8_t)((t + 1) % N);
		return true;
	}

private:
	volatile uint8_t buf[N] = {0};
	volatile uint8_t head = 0;
	volatile uint8_t tail = 0;
};

#endif // EVENT_RING_H

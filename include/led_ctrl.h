#ifndef LED_CTRL_H
#define LED_CTRL_H

class LEDController {
	private:

	public:
		void init();
		void allClear();
		void setRGBcolor(uint8_t r_param,uint8_t g_param,uint8_t b_param);
		void updatePidMode(uint8_t mode);
};

#endif
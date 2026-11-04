# include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include <fastLED.h>
#include <Adafruit_NeoPixel.h>

#include "led_ctrl.h"
#include "define_system.h"

#define NUM_LEDS				2
#define BRIGHTNESS				64
#define LED_TYPE				WS2812
#define COLOR_ORDER				GRB
#define RGBLED_OUT				17

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, RGBLED_OUT, NEO_GRB + NEO_KHZ800);

void LEDController::init(){
	gpio_set_direction((gpio_num_t)WLED_OUT,GPIO_MODE_OUTPUT);
	pixels.begin();
}

void LEDController::allClear(){
	pixels.clear();
	pixels.show();
}

void LEDController::setRGBcolor(uint8_t r_param,uint8_t g_param,uint8_t b_param){
	pixels.setPixelColor(0, pixels.Color(r_param, g_param, b_param));
	pixels.setPixelColor(1, pixels.Color(r_param, g_param, b_param));
	pixels.show();
}

//調整モードで選んでいる項目を1個目のLEDに表示する（-1: 調整モードではない → 消灯）
//※ 暫定の表示。色と点滅パターンによる表示は issue #6 で作り直す
void LEDController::updatePidMode(int mode){
	static const uint8_t COLORS[][3] = {
		{30,  0,  0},	//0: rate P
		{ 0, 30,  0},	//1: rate I
		{ 0,  0, 30},	//2: rate D
		{30, 30,  0},	//3: angle P
		{30,  0, 30},	//4: yaw P
		{ 0, 30, 30},	//5: yaw I
	};
	if (mode >= 0 && mode < (int)(sizeof(COLORS) / sizeof(COLORS[0]))) {
		pixels.setPixelColor(0, pixels.Color(COLORS[mode][0], COLORS[mode][1], COLORS[mode][2]));
	} else {
		pixels.setPixelColor(0, pixels.Color(0, 0, 0));
	}
	pixels.show();
}

void LEDController::taskSetBattLowIndicator(){
	while(1){
		for(int blkloop = 0;blkloop < 3 ; blkloop++){
			pixels.setPixelColor(0,pixels.Color(120, 0, 0));
			pixels.show();
			vTaskDelay(pdMS_TO_TICKS(250));
			
			pixels.setPixelColor(0,pixels.Color(0, 0, 0));
			pixels.show();
			vTaskDelay(pdMS_TO_TICKS(250));
		}
		vTaskDelay(pdMS_TO_TICKS(3000));
	}
}

//アーム中は白色LEDを点灯する
void LEDController::setArmIndicator(bool armed){
	gpio_set_level((gpio_num_t)WLED_OUT, armed ? 1 : 0);
}

void LEDController::taskFlyMode(){
	while(1){
		for(int blkloop = 0;blkloop < 3 ; blkloop++){
			gpio_set_level((gpio_num_t)WLED_OUT,1);
			vTaskDelay(pdMS_TO_TICKS(50));
			
			gpio_set_level((gpio_num_t)WLED_OUT,0);
			vTaskDelay(pdMS_TO_TICKS(50));
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
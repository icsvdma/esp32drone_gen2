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

void LEDController::updatePidMode(uint8_t mode){
	if(mode == 0){
		pixels.setPixelColor(0, pixels.Color(30, 0, 0));
	}
	else if(mode == 1){
		pixels.setPixelColor(0, pixels.Color(0, 30, 0));
	}
	else if(mode == 2){
		pixels.setPixelColor(0, pixels.Color(0, 0, 30));
	}
	else{
		pixels.setPixelColor(0, pixels.Color(0, 0, 0));
	}
	pixels.show();

}
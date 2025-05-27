#ifndef BATT_ADC_H
#define BATT_ADC_H

#define DEFAULT_VREF    1100
#define NUM_OF_SAMPLE   64

#include "driver/adc.h"
#include "esp_adc_cal.h"

class Battery_ADC {
	private:
        uint32_t vref         = 0;
        uint32_t rawvolt      = 0;
        uint32_t divide_data  = 0;
        double battvolt       = 0;

        //使用するADCチャンネルに合わせる esp32S2の場合は使用するパラメータが変化するので注意
        static const adc1_channel_t channel1    = ADC1_CHANNEL_0; //GPIO36
        static const adc_bits_width_t width     = ADC_WIDTH_BIT_12; //0~4095
        static const adc_atten_t atten          = ADC_ATTEN_11db;
        static const adc_unit_t unit            = ADC_UNIT_1;

		
		
	public:
        esp_adc_cal_characteristics_t *adc_chars;

		void init();
        void update();
        uint32_t taskGetRawData();
        uint32_t calcDivideVolt(uint32_t adc_data);
        double calcBattVolt(uint32_t divide_data);

		static void taskUpdate(void *param);
		void start();

        //getter
        uint32_t getRawVolt();
        uint32_t getDivideVolt();
        uint32_t getBattVolt();

        void check_efuse();
};

#endif
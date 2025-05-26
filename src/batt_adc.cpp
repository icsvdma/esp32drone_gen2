
#include "batt_adc.h"

void Battery_ADC::init(){
    check_efuse();

    adc1_config_width(width);
    adc1_config_channel_atten(channel1 ,atten);

    //構造体のメモリ確保
    adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    // 情報を取得する
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    vref = adc_chars->vref;
}
void Battery_ADC::update(){
    
}

uint32_t Battery_ADC::getRawData(){
    for(int loop =0 ; loop <= NUM_OF_SAMPLE ; loop++){
        rawvolt += adc1_get_raw((adc1_channel_t)channel1);
    }

    rawvolt = rawvolt / NUM_OF_SAMPLE;
    return rawvolt;
}

uint32_t Battery_ADC::calcDivideVolt(uint32_t adc_data){
    uint32_t divide_data;
    // divide_data = esp_adc_cal_raw_to_voltage(adc_data, adc_chars);
    divide_data = adc_data * vref / 4095;
}

uint32_t Battery_ADC::calcBattVolt(uint32_t divide_data){
    
    //TODO:分圧抵抗の比率府を入力する 47k ×2の分圧でやる
    battvolt = 2 * divide_data;
    
    return battvolt;

}

uint32_t Battery_ADC::getRawVolt(){
    return rawvolt;
}

uint32_t Battery_ADC::getDivideVolt(){
    return divide_data;
}

uint32_t Battery_ADC::getBattVolt(){
    return battvolt;
}




void Battery_ADC::check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
    #error "This example is configured for ESP32/ESP32S2."
#endif
}
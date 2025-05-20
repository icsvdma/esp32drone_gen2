//ESP32 ピンアサイン定義
#define MOT_FR_OUT				26
#define MOT_FL_OUT				25
#define MOT_RR_OUT				27
#define MOT_RL_OUT				14
#define MPU6050_INT_IN			21
#define BATT_MON_IN				36
#define RGBLED_OUT				17
#define WLED_OUT				4
#define BUZZER_OUT				16

//PWM出力のチャネル定義
#define PWMCH0_MOT_FR			0
#define PWMCH1_MOT_FL			1
#define PWMCH2_MOT_RR			2
#define PWMCH3_MOT_RL			3
#define PWMCH4_BUZZER			4

#define NUM_LEDS    2
#define BRIGHTNESS  50
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
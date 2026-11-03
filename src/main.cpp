
#include "drone_controller.h"

DroneController drone;

void setup(void){
	drone.init();
	//制御タスク（CONTROL_HZ 周期、コア1固定）を起動
	drone.start();
}


//制御そのものは制御タスクで行う。loop() ではLED表示やデバッグ出力など
//周期が多少ずれても問題ない処理だけを行う
void loop(){
	drone.service();
	vTaskDelay(pdMS_TO_TICKS(SERVICE_PERIOD_MS));
}

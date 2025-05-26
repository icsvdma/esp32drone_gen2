
#include "motor.h"
#include "mpu6050.h"
#include "pid_controller.h"
#include "drone_controller.h"
#include "ps3_ctrl.h"

DroneController drone;

void setup(void){
    drone.init();
}


void loop(){
	const TickType_t delay = pdMS_TO_TICKS(10);  // 100Hz

	drone.update(0.01);  // 10ms
	vTaskDelay(delay);
}


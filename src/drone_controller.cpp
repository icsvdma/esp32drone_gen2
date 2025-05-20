#include "motor.h"
#include "mpu6050.h"
#include "pid_controller.h"
#include "drone_controller.h"
#include "madgwick_filter.h"
#include "define_system.h"

void DroneController::init() {
	m1.init();
	m2.init();
	m3.init();
	m4.init();

	mpu.init_mpu();
	//mpu.mpu_write_data();

	madgwick.init();

	ps3ctrl.init();
	
	ledctrl.init();
	ledctrl.allClear();
	ledctrl.setRGBcolor(0,10,20);
}

void DroneController::update(float dt) {
	//mpu.read_accel_gyro(angle_x, angle_y);
	mpu.read_accel_gyro();
	madgwick.update();
	
	ps3ctrl.update();

	ledctrl.updatePidMode(ps3ctrl.getPidAly());

	base_thrust	= ps3ctrl.getThrust();
	base_roll	= -1 * ps3ctrl.getRoll();
	base_pitch	= -1 * ps3ctrl.getPitch();

	//引数:目標値、現在角度、取得周期
	roll_output = (int16_t)pid_roll.compute(base_roll, madgwick.getRoll(), dt);
	pitch_output = (int16_t)pid_pitch.compute(base_pitch, madgwick.getPitch(), dt);

#ifdef DEBUG_MODE
	madgwick.debug_madrpy();
	//debug_rpy();
	//debug_base_rp();
	//debug_angle();
	//ps3ctrl.debug_raw_param();
	//ps3ctrl.debug_target_param();
	//debug_motor_param();
#else
	m1.set_speed(clip_param(base_thrust - roll_output - pitch_output));  // front-right		MOT_FR_OUT
	m2.set_speed(clip_param(base_thrust + roll_output - pitch_output));  // front-left		MOT_FL_OUT
	m3.set_speed(clip_param(base_thrust - roll_output + pitch_output));  // back-right		MOT_RR_OUT
	m4.set_speed(clip_param(base_thrust + roll_output + pitch_output));  // back-left		MOT_RL_OUT
#endif
}

void DroneController::debug_rpy(){
	printf("%d,",base_thrust);
	printf("%d,",roll_output);
	printf("%d,",pitch_output);
	printf("\n");
}

void DroneController::debug_base_rp(){
	printf("%d\t,",base_thrust);
	printf("%d\t,",base_roll);
	printf("%d\t,",base_pitch);
}

void DroneController::debug_angle(){
	printf("%f\t,",angle_x);
	printf("%f\t,",angle_y);
	printf("\n");
}

void DroneController::debug_motor_param(){
	int16_t m1_param = (base_thrust + roll_output + pitch_output);
	int16_t m2_param = (base_thrust - roll_output + pitch_output);
	int16_t m3_param = (base_thrust + roll_output - pitch_output);
	int16_t m4_param = (base_thrust - roll_output - pitch_output);
	
	printf("%d,",m1_param);
	printf("%d,",m2_param);
	printf("%d,",m3_param);
	printf("%d\n",m4_param);
} 

int16_t DroneController::clip_param(int16_t pwm_val){
	if(pwm_val < 0){
		return 0;
	}
	else if(pwm_val >= 1023){
		return 1023;
	}
	else{
		return pwm_val;
	}
} 
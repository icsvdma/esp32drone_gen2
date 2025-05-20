#ifndef DRONE_CTRL_H
#define DRONE_CTRL_H

#include "motor.h"
#include "mpu6050.h"
#include "pid_controller.h"
#include "ps3_ctrl.h"
#include "led_ctrl.h"
#include "define_system.h"
#include "madgwick_filter.h"

class DroneController {
	private:
		float angle_x = 0;
		float angle_y = 0;
	
		int16_t roll_output;
		int16_t pitch_output;

		uint16_t base_thrust	= 0;
		int16_t base_roll		= 0;
		int16_t base_pitch		= 0;
	public:
		Motor m1, m2, m3, m4;
		MPU6050_user mpu;
		PIDController pid_roll, pid_pitch;
		PS3Controller ps3ctrl;
		LEDController ledctrl;
		madgwick_filter madgwick;

		DroneController() :
				m1(MOT_FR_OUT, LEDC_CHANNEL_0),
				m2(MOT_FL_OUT, LEDC_CHANNEL_1),
				m3(MOT_RR_OUT, LEDC_CHANNEL_2),
				m4(MOT_RL_OUT, LEDC_CHANNEL_3),
				mpu(I2C_NUM_0),
				pid_roll(),
				pid_pitch(),
				ledctrl(),
				ps3ctrl(&pid_roll , &pid_pitch),
				madgwick(&mpu) {}

		void init();
		void update(float dt);
		void debug_rpy();
		void debug_base_rp();
		void debug_angle();
		void debug_motor_param();
		int16_t clip_param(int16_t pwm_val);
};

#endif
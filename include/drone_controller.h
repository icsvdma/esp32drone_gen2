#ifndef DRONE_CTRL_H
#define DRONE_CTRL_H

#include "motor.h"
#include "sensor.h"
#include "pid_controller.h"
#include "ps3_ctrl.h"
#include "led_ctrl.h"
#include "define_system.h"
#include "flight_config.h"
#include "madgwick_filter.h"
#include "batt_adc.h"

class DroneController {
	private:
		Motor m1, m2, m3, m4;
		madgwick_filter madgwick_user;
		MPU6050_user mpu_user;
		PIDController pid_roll, pid_pitch;
		PS3Controller ps3ctrl;
		LEDController ledctrl;
		Battery_ADC batt_adc;

		ImuSample imu				= {};
		uint32_t imu_err_count		= 0;
		float last_dt				= CONTROL_DT_NOMINAL;
	
		int16_t roll_output			= 0;
		int16_t pitch_output		= 0;

		uint16_t base_thrust	= 0;
		int16_t base_roll		= 0;
		int16_t base_pitch		= 0;

		int last_pid_mode		= -1;

		static void controlTask(void *param);
		void step(float dt);
	public:
		DroneController() :
				m1(MOT_FR_OUT, LEDC_CHANNEL_0),
				m2(MOT_FL_OUT, LEDC_CHANNEL_1),
				m3(MOT_RR_OUT, LEDC_CHANNEL_2),
				m4(MOT_RL_OUT, LEDC_CHANNEL_3),
				
				madgwick_user(0, 0, 0),
				mpu_user(&madgwick_user,I2C_NUM_0),
				pid_roll(),
				pid_pitch(),
				ps3ctrl(&pid_roll , &pid_pitch),
				ledctrl()								//TODO:	コンストラクタの初期化によってGPIOポート番号を指定するようにする
		{}

		void init();
		void start();
		void service();
		void debug_rpy();
		void debug_base_rp();
		void debug_angle();
		void debug_motor_param();
};

#endif

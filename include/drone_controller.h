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
		PIDController pid_roll_rate, pid_pitch_rate;	//内側ループ（角速度PID）
		PIDController pid_yaw_rate;						//ヨーは角速度のみ制御
		PS3Controller ps3ctrl;
		LEDController ledctrl;
		Battery_ADC batt_adc;

		ImuSample imu				= {};
		uint32_t imu_err_count		= 0;
		float last_dt				= CONTROL_DT_NOMINAL;
	
		float roll_rate_sp			= 0;	//外側ループの出力 = 内側ループの目標角速度[deg/s]
		float pitch_rate_sp			= 0;
		float roll_output			= 0;
		float pitch_output			= 0;
		float yaw_rate_sp			= 0;
		float yaw_output			= 0;

		// モータ出力値（FR, FL, RR, RL）。実機出力とデバッグ表示で同じ値を使う
		enum { MOT_FR = 0, MOT_FL, MOT_RR, MOT_RL, MOT_NUM };
		uint16_t motor_out[MOT_NUM]	= {0};

		uint16_t base_thrust	= 0;
		float base_roll			= 0;
		float base_pitch		= 0;

		int last_pid_mode		= -1;

		// アーム状態（制御タスクだけが書き換える）
		enum ArmEvent : uint8_t {
			EV_NONE = 0,
			EV_ARMED,
			EV_DISARM_BUTTON,
			EV_DISARM_LINK,
			EV_DISARM_TILT,
			EV_DISARM_IMU,
			EV_DISARM_AUTO,
			EV_REJECT_LINK,
			EV_REJECT_THROTTLE,
			EV_REJECT_TILT,
			EV_REJECT_CALIB,
		};
		volatile bool armed				= false;
		bool arm_toggle_req				= false;
		bool calib_ok					= false;
		uint32_t imu_fail_streak		= 0;
		uint32_t zero_thr_since_ms		= 0;
		volatile uint8_t arm_event		= EV_NONE;	//制御タスク → service() への通知（表示用）
		bool last_armed_led				= false;

		void updateArmState(uint32_t now_ms);
		void tryArm(uint32_t now_ms);
		void disarm(ArmEvent reason);
		void stopMotors();
		void resetControllers();
		static const char* armEventStr(uint8_t ev);

		volatile bool gains_changed		= false;	//制御タスク → service() への通知
		volatile bool save_requested	= false;

		void handleCommands();
		void applyGainCommand(const Ps3Command &cmd);
		void loadGains();
		void saveGains();
		void printGains();

		static void controlTask(void *param);
		void step(float dt);
		void mix(float thrust, float roll, float pitch, float yaw);
		void outputMotors();
	public:
		DroneController() :
				m1(MOT_FR_OUT, LEDC_CHANNEL_0),
				m2(MOT_FL_OUT, LEDC_CHANNEL_1),
				m3(MOT_RR_OUT, LEDC_CHANNEL_2),
				m4(MOT_RL_OUT, LEDC_CHANNEL_3),
				
				madgwick_user(0, 0, 0),
				mpu_user(&madgwick_user,I2C_NUM_0),
				pid_roll_rate(RATE_I_LIMIT, RATE_OUT_LIMIT, RATE_D_CUTOFF_HZ),
				pid_pitch_rate(RATE_I_LIMIT, RATE_OUT_LIMIT, RATE_D_CUTOFF_HZ),
				pid_yaw_rate(YAW_I_LIMIT, YAW_OUT_LIMIT, 0),
				ps3ctrl(),
				ledctrl()								//TODO:	コンストラクタの初期化によってGPIOポート番号を指定するようにする
		{}

		void init();
		void start();
		void service();
		void debug_rpy();
		void debug_base_rp();
		void debug_angle();
		void debug_rate();
		void debug_motor_param();
};

#endif

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
#include "event_ring.h"

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

		float throttle			= 0;	//増減式スロットルの現在値（制御タスクだけが書き換える）
		float base_thrust		= 0;
		float base_roll			= 0;
		float base_pitch		= 0;

		int last_pid_mode		= -2;

		// 制御タスク → service()（loop）への通知。表示・保存などに使う
		enum UiEvent : uint8_t {
			EV_NONE = 0,
			EV_ARMED,
			EV_DISARM_BUTTON,
			EV_DISARM_LINK,
			EV_DISARM_TILT,
			EV_DISARM_IMU,
			EV_DISARM_AUTO,
			EV_DISARM_KILL,
			EV_DISARM_LANDED,
			EV_REJECT_LINK,
			EV_REJECT_THROTTLE,
			EV_REJECT_TILT,
			EV_REJECT_CALIB,
			EV_HINT_HOLD_ARM,
			EV_HINT_HOLD_DISARM,
			EV_TUNE_ON,
			EV_TUNE_OFF,
			EV_TUNE_REJECT_ARMED,
			EV_TUNE_CHANGED,
			EV_TUNE_LIMIT,
			EV_TUNE_RESET,
			EV_SAVE_REQUEST,
			EV_SAVE_REJECT_ARMED,
		};
		EventRing<16> ui_events;
		static const char* uiEventStr(uint8_t ev);

		// アーム状態（制御タスクだけが書き換える）
		volatile bool armed				= false;
		enum ArmRequest : uint8_t { ARM_REQ_NONE = 0, ARM_REQ_ARM, ARM_REQ_DISARM, ARM_REQ_KILL };
		ArmRequest arm_req				= ARM_REQ_NONE;
		bool kill_combo_prev			= false;
		bool calib_ok					= false;
		uint32_t imu_fail_streak		= 0;
		uint32_t zero_thr_since_ms		= 0;
		bool has_flown					= false;	//アーム後に一度でもモータが回るスロットルにしたか
		uint32_t landed_since_ms		= 0;		//着陸状態が始まった時刻（0: 着陸状態ではない）
		bool last_armed_led				= false;

		void updateThrottle(float dt);
		void updateArmState(uint32_t now_ms);
		bool checkLanded(uint32_t now_ms);
		void tryArm(uint32_t now_ms);
		void disarm(UiEvent reason);
		void stopMotors();
		void resetControllers();

		// ゲイン調整モード（制御タスクだけが書き換える）
		// 調整できる項目。Roll と Pitch の角速度PIDは同じ値を使う
		enum TuneItem : uint8_t {
			TUNE_RATE_KP = 0,
			TUNE_RATE_KI,
			TUNE_RATE_KD,
			TUNE_ANGLE_KP,
			TUNE_YAW_KP,
			TUNE_YAW_KI,
			TUNE_NUM
		};
		volatile bool tune_mode			= false;
		volatile uint8_t tune_item		= TUNE_RATE_KP;
		float tune_val[TUNE_NUM]		= {0};
		float angle_kp					= ANGLE_KP_DEFAULT;

		void handleButtons(uint32_t now_ms);
		void stepTune(int dir);
		void resetTuneItem();
		void applyTuneValues();
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

#ifndef PID_CTRL_H
#define PID_CTRL_H

class PIDController {
private:
    float user_kp = 2.0;    // Default P gain for angle control
    float user_ki = 0.05;   // Default I gain for angle control
    float user_kd = 0.5;    // Default D gain for angle control

    float prev_error = 0;
    float integral = 0;
    
    // Anti-windup limits
    static constexpr float INTEGRAL_MAX = 100.0;
    static constexpr float INTEGRAL_MIN = -100.0;
    static constexpr float MIN_DT = 0.001;  // Minimum dt to prevent derivative spikes

public:
    //PIDController(float p, float i, float d);
    PIDController();
    float compute(int16_t setpoint, float measured, float dt);

	void set_kp(int dir);
	void set_ki(int dir);
	void set_kd(int dir);
	float get_kp();
	float get_ki();
	float get_kd();

	void updatePID();
};

#endif
#ifndef PID_CTRL_H
#define PID_CTRL_H

class PIDController {
private:
    float kp;
    float ki;
    float kd;
    float user_kp;
    float user_ki;
    float user_kd;

    float prev_error = 0;
    float integral = 0;

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
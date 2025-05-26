#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include "sensor.h"
#include <MadgwickAHRS.h>


class madgwick_filter {
	private:
	float mad_roll;
	float mad_pitch;
	float mad_yaw;

	//int16_t ax, ay, az;
	//int16_t gx, gy, gz;	
	
	float q0, q1, q2, q3;
	
	public:
		Madgwick madgwick;
		MPU6050_user* mpu_user;
		madgwick_filter(MPU6050_user* inst_mpu) : mpu_user(inst_mpu) {}

		void init();
		void update();

		float getRoll();
		float getPitch();
		float getYaw();
		float getQ0();
		float getQ1();
		float getQ2();
		float getQ3();

		void debug_madrpy();


};

#endif // MADGWICK_FILTER_H
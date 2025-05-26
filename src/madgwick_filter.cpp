
#include "madgwick_filter.h"
#include "mpu6050.h"


void madgwick_filter::init() {
	madgwick.begin(100);
}

void madgwick_filter::update() {
	float ax = mpu_user->getAx();
	float ay = mpu_user->getAy();
	float az = mpu_user->getAz();
	float gx = mpu_user->getGx();
	float gy = mpu_user->getGy();
	float gz = mpu_user->getGz();
	
	madgwick.updateIMU(gx/131.0, gy/131.0, gz/131.0, ax/16384.0, ay/16384.0, az/16384.0);

	mad_roll		= madgwick.getRoll();
	mad_pitch		= madgwick.getPitch();
	mad_yaw			= madgwick.getYaw();

	debug_madrpy();
	//q0				= madgwick.getQ0();
	//q1				= madgwick.getQ1();
	//q2				= madgwick.getQ2();
	//q3				= madgwick.getQ3();
}

	float madgwick_filter::getRoll() {
		return mad_roll;
	}
	float madgwick_filter::getPitch() {
		return mad_pitch;
	}
	float madgwick_filter::getYaw() {
		return mad_yaw;
	}
	float madgwick_filter::getQ0() {
		return q0;
	}
	float madgwick_filter::getQ1() {
		return q1;
	}
	float madgwick_filter::getQ2() {
		return q2;
	}
	float madgwick_filter::getQ3() {
		return q3;
	}

	void madgwick_filter::debug_madrpy(){
		printf("mad roll : %f\t,",mad_roll);
		printf("mad pitch : %f\t,",mad_pitch);
		printf("madg yaw : %f\n",mad_yaw);
	}
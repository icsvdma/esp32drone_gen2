
#include "madgwick_filter.h"
#include "sensor.h"


void madgwick_filter::init() {
	madgwick.begin(100);
}

void madgwick_filter::update_mad(float ax,float ay,float az,float gx,float gy,float gz) {
	//ax				= ;
	//ay				= ;
	//az				= ;
	//gx				= ;
	//gy				= ;
	//gz				= ;

	ax				= ax / ACCEL_LSB_PER_G;
	ay				= ay / ACCEL_LSB_PER_G;
	az				= az / ACCEL_LSB_PER_G;
	gx				= gx / GYRO_LSB_PER_DPS;
	gy				= gy / GYRO_LSB_PER_DPS;
	gz				= gz / GYRO_LSB_PER_DPS;
	
	madgwick.updateIMU(gx, gy, gz, ax, ay, az);

	mad_roll		= madgwick.getRoll();
	mad_pitch		= madgwick.getPitch();
	mad_yaw			= madgwick.getYaw();

	//debug_madrpy();
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
		printf("mad roll : %.2f\t,",mad_roll);
		printf("mad pitch : %.2f\t,",mad_pitch);
		printf("madg yaw : %.2f\n",mad_yaw);
	}
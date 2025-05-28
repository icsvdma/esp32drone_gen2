#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <MadgwickAHRS.h>


class madgwick_filter {
	private:
		float		mad_roll;
		float		mad_pitch;
		float		mad_yaw;

		int16_t 	ax = 0, ay = 0, az = 0;
		int16_t 	gx = 0, gy = 0, gz = 0;
	
		float		q0 = 0, q1 = 0, q2 = 0, q3 = 0;
	
	public:
		Madgwick madgwick;
		// MPU6050_user* mpu_user;
		madgwick_filter(float init_mad_roll, float init_mad_pitch, float init_mad_yaw)
			: mad_roll(init_mad_roll),
			  mad_pitch(init_mad_pitch),
			  mad_yaw(init_mad_yaw) {}

		void init();
		void update_mad(float ax,float ay,float az,float gx,float gy,float gz);

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
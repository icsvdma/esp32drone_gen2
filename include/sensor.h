#ifndef SENSOR_H
#define SENSOR_H

#include "driver/i2c.h"

#include <MPU6050.h>
#include "madgwick_filter.h"

#define I2C_MASTER_SCL_IO           18		// I2C master SCK
#define I2C_MASTER_SDA_IO           19      // I2C master SDA
#define I2C_MASTER_NUM              0		// I2C module Number
#define I2C_MASTER_FREQ_HZ          400000	// I2C master freq
#define I2C_MASTER_TX_BUF_DISABLE   0		// I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0		// I2C master doesn't need buffer

#define ACK_CHECK_EN				0x1		// I2C master will check ack from slave
#define ACK_CHECK_DIS				0x0		// I2C master will not check ack from slave
#define ACK_VAL						0x0		// I2C ack value
#define NACK_VAL					0x1		// I2C nack value

#define MPU6050_PWR_MGMT_1_REG		0x6B
#define MPU6050_ACCEL_XOUT_H_REG	0x3B

#define OFS_Gyro_X					78
#define OFS_Gyro_Y					66
#define OFS_Gyro_Z					132
#define OFS_Accel_X					-1473
#define OFS_Accel_Y					1185
#define OFS_Accel_Z					2226

class madgwick_filter;

class MPU6050_user {
	
	private:
		const char *TAG = "MPU6050";
		i2c_port_t		i2c_port;
		uint8_t			dev_addr = 0x68;
		
		uint8_t 		data[14] = {0x00};
		int16_t 		ax = 0, ay = 0, az = 0;
		int16_t 		gx = 0, gy = 0, gz = 0;
	
	public:
		MPU6050 mpu_host;
		madgwick_filter* mad_user;

		MPU6050_user(madgwick_filter* inst_madgwick_user, i2c_port_t port)
			: mad_user(inst_madgwick_user),
			  i2c_port(port) {}


		esp_err_t		init();
		esp_err_t		read_accel_gyro(float &angle_x, float &angle_y);
		unsigned char	mpu_read(int addr);
		void			mpu_write_data();
		esp_err_t		read_raw_data(uint8_t reg, int16_t &value);
		esp_err_t		read_bytes(uint8_t reg_addr, uint8_t *data, size_t length);

		int16_t 		getAx();
		int16_t 		getAy();
		int16_t 		getAz();
		int16_t 		getGx();
		int16_t 		getGy();
		int16_t 		getGz();

		esp_err_t 		init_mpu();

		void 			debug_sensor();
		//esp_err_t read_accel_gyro();

		static void		taskUpdate(void *param);
		void			start();
};

#endif
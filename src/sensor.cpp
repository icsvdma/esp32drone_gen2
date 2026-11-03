#include "sensor.h"

esp_err_t MPU6050_user::init() {
	uint8_t wake = 0;

	int i2c_master_port = I2C_MASTER_NUM;

	// i2c_config_t conf = {
	// 	.mode				= I2C_MODE_MASTER,
	// 	.sda_io_num			= I2C_MASTER_SDA_IO,
	// 	.scl_io_num			= I2C_MASTER_SCL_IO,
	// 	.sda_pullup_en		= GPIO_PULLUP_DISABLE,
	// 	.scl_pullup_en		= GPIO_PULLUP_DISABLE,
	// 	.master.clk_speed	= I2C_MASTER_FREQ_HZ
	// };

	i2c_config_t conf = {};
	conf.mode				= I2C_MODE_MASTER;
	conf.sda_io_num			= I2C_MASTER_SDA_IO;
	conf.scl_io_num			= I2C_MASTER_SCL_IO;
	conf.sda_pullup_en		= GPIO_PULLUP_DISABLE;
	conf.scl_pullup_en		= GPIO_PULLUP_DISABLE;
	conf.master.clk_speed	= I2C_MASTER_FREQ_HZ;

	esp_err_t err = i2c_param_config(i2c_master_port, &conf);
	if(err != ESP_OK) {
		return err;
	}

	return i2c_driver_install(	i2c_master_port,
								conf.mode,
								I2C_MASTER_RX_BUF_DISABLE,
								I2C_MASTER_TX_BUF_DISABLE,
								0);
}

// esp_err_t MPU6050_user::read_raw_data(uint8_t reg, int16_t &value) {
// 	uint8_t data[2];
// 	i2c_master_write_read_device(i2c_port, dev_addr, &reg, 1, data, 2, 1000 / portTICK_PERIOD_MS);
// 	value = ((int16_t)data[0] << 8) | data[1];
// 	return ESP_OK;
// }

//wakeupのみ行う
void MPU6050_user::mpu_write_data(){
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1_REG, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, 0x00, I2C_MASTER_ACK);
	i2c_master_stop(cmd);
 
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
 }

esp_err_t MPU6050_user::read_bytes(uint8_t reg_addr, uint8_t *data, size_t length){
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
	if (length > 1) {
		i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(3));	//制御周期内に収まるよう短めにする
	i2c_cmd_link_delete(cmd);
	return ret;
}

esp_err_t MPU6050_user::read_accel_gyro(float &angle_x, float &angle_y) {
	if (read_bytes(MPU6050_ACCEL_XOUT_H_REG, data, 14) == ESP_OK) {
		ax = (data[0] << 8) | data[1];
		ay = (data[2] << 8) | data[3];
		az = (data[4] << 8) | data[5];

		gx = (data[8] << 8) | data[9];
		gy = (data[10] << 8) | data[11];
		gz = (data[12] << 8) | data[13];
		
		ESP_LOGI(TAG, "Accel: X=%d Y=%d Z=%d ",ax, ay, az);
	} else {
		ESP_LOGE(TAG, "Failed to read sensor data");
	}

	angle_x = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
	angle_y = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

	return ESP_OK;
}

int16_t MPU6050_user::getAx() {
	return ax;	
}

int16_t MPU6050_user::getAy() {
	return ay;	
}

int16_t MPU6050_user::getAz() {
	return az;	
}

int16_t MPU6050_user::getGx() {
	return gx;	
}

int16_t MPU6050_user::getGy() {
	return gy;	
}

int16_t MPU6050_user::getGz() {
	return gz;	
}

//14バイトを読み出して生値(ax〜gz)を更新する
esp_err_t MPU6050_user::read_raw() {
	esp_err_t ret = read_bytes(MPU6050_ACCEL_XOUT_H_REG, data, 14);
	if (ret == ESP_OK) {
		ax = (data[0] << 8) | data[1];
		ay = (data[2] << 8) | data[3];
		az = (data[4] << 8) | data[5];

		gx = (data[8] << 8) | data[9];
		gy = (data[10] << 8) | data[11];
		gz = (data[12] << 8) | data[13];
	}
	return ret;
}

//起動時キャリブレーション
//機体を静止（CALIB_ACCEL_LEVEL=1 の場合は水平にも）させた状態で呼ぶこと
//ジャイロの最大-最小が大きい（動いている）場合はやり直す
esp_err_t MPU6050_user::calibrate() {
	for (int retry = 0; retry < CALIB_MAX_RETRY; retry++) {
		int32_t sum[6] = {0};
		int16_t g_min[3] = {INT16_MAX, INT16_MAX, INT16_MAX};
		int16_t g_max[3] = {INT16_MIN, INT16_MIN, INT16_MIN};
		int valid = 0;

		for (int i = 0; i < CALIB_SAMPLES; i++) {
			if (read_raw() == ESP_OK) {
				const int16_t g[3] = {gx, gy, gz};
				sum[0] += ax; sum[1] += ay; sum[2] += az;
				sum[3] += gx; sum[4] += gy; sum[5] += gz;
				for (int k = 0; k < 3; k++) {
					if (g[k] < g_min[k]) g_min[k] = g[k];
					if (g[k] > g_max[k]) g_max[k] = g[k];
				}
				valid++;
			}
			vTaskDelay(pdMS_TO_TICKS(2));
		}

		if (valid < CALIB_SAMPLES / 2) {
			printf("[calib] sensor read failed (%d/%d)\n", valid, CALIB_SAMPLES);
			continue;
		}

		bool moving = false;
		for (int k = 0; k < 3; k++) {
			if ((g_max[k] - g_min[k]) > CALIB_GYRO_MAX_P2P) moving = true;
		}
		if (moving) {
			printf("[calib] motion detected, retry %d\n", retry + 1);
			continue;
		}

		gx_ofs = sum[3] / valid;
		gy_ofs = sum[4] / valid;
		gz_ofs = sum[5] / valid;
#if CALIB_ACCEL_LEVEL
		ax_ofs = sum[0] / valid;
		ay_ofs = sum[1] / valid;
		az_ofs = sum[2] / valid - ACCEL_LSB_PER_G;	// Z軸は1gが正しい値
#endif
		printf("[calib] gyro ofs: %ld %ld %ld / accel ofs: %ld %ld %ld\n",
			(long)gx_ofs, (long)gy_ofs, (long)gz_ofs,
			(long)ax_ofs, (long)ay_ofs, (long)az_ofs);
		return ESP_OK;
	}

	printf("[calib] FAILED: offsets are not applied\n");
	return ESP_FAIL;
}

//センサを読み、オフセット補正と単位換算をして返す（制御タスクから呼ぶ）
esp_err_t MPU6050_user::read(ImuSample &out) {
	esp_err_t ret = read_raw();
	if (ret != ESP_OK) {
		return ret;
	}
	out.ax = (float)(ax - ax_ofs) / ACCEL_LSB_PER_G;
	out.ay = (float)(ay - ay_ofs) / ACCEL_LSB_PER_G;
	out.az = (float)(az - az_ofs) / ACCEL_LSB_PER_G;
	out.gx = (float)(gx - gx_ofs) / GYRO_LSB_PER_DPS;
	out.gy = (float)(gy - gy_ofs) / GYRO_LSB_PER_DPS;
	out.gz = (float)(gz - gz_ofs) / GYRO_LSB_PER_DPS;
	return ESP_OK;
}

void MPU6050_user::debug_sensor(){
	printf("ax: %d\t ay: %d\t az: %d\t", ax, ay, az);
	printf("gx: %d\t gy: %d\t gz: %d\n", gx, gy, gz);
}
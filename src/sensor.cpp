#include "sensor.h"

MPU6050_user::MPU6050_user(i2c_port_t port) : i2c_port(port) {}

esp_err_t MPU6050_user::init() {
    uint8_t wake = 0;

	int i2c_master_port = I2C_MASTER_NUM;

    //i2c_config_t conf = {
    //    .mode				= I2C_MODE_MASTER,
    //    .sda_io_num			= I2C_MASTER_SDA_IO,
    //    .scl_io_num			= I2C_MASTER_SCL_IO,
    //    .sda_pullup_en		= GPIO_PULLUP_DISABLE,
    //    .scl_pullup_en		= GPIO_PULLUP_DISABLE,
    //    .master.clk_speed	= I2C_MASTER_FREQ_HZ
    //};

	i2c_config_t conf = {};

	conf.mode				= I2C_MODE_MASTER;
	conf.sda_io_num			= I2C_MASTER_SDA_IO;
	conf.scl_io_num			= I2C_MASTER_SCL_IO;
	conf.sda_pullup_en		= GPIO_PULLUP_DISABLE;
	conf.scl_pullup_en		= GPIO_PULLUP_DISABLE;
	conf.master.clk_speed	= 100000;

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

esp_err_t MPU6050_user::read_raw_data(uint8_t reg, int16_t &value) {
    uint8_t data[2];
    i2c_master_write_read_device(i2c_port, dev_addr, &reg, 1, data, 2, 1000 / portTICK_PERIOD_MS);
    value = ((int16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

//wakeupのみ行う
void MPU6050_user::mpu_write_data(){
   i2c_port_t port = 0;
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
 
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
   i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1_REG, I2C_MASTER_ACK);
   i2c_master_write_byte(cmd, 0x00, I2C_MASTER_ACK);
   i2c_master_stop(cmd);
 
   i2c_master_cmd_begin(port, cmd, 1 / portTICK_RATE_MS);
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
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t MPU6050_user::read_accel_gyro(float &angle_x, float &angle_y) {
    //read_raw_data(0x3B, ax);
    //read_raw_data(0x3D, ay);
    //read_raw_data(0x3F, az);

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

esp_err_t MPU6050_user::init_mpu() {
	mpu_host.initialize();
	if (mpu_host.testConnection() != true) {
		ESP_LOGE(TAG, "MPU disconection");
		return ESP_FAIL;
	}
	//if (mpu.dmpInitialize() != 0) {
	//	ESP_LOGE(TAG, "MPU break");
	//	return ESP_FAIL;
	//}
	mpu_host.setXGyroOffset(Gyro_X);
	mpu_host.setYGyroOffset(Gyro_Y);
	mpu_host.setZGyroOffset(Gyro_Z);
	mpu_host.setXAccelOffset(Accel_X);
	mpu_host.setYAccelOffset(Accel_Y);
	mpu_host.setZAccelOffset(Accel_Z);
	//mpu.setDMPEnabled(true);
	//mpuIntStatus = mpu.getIntStatus();
	//dmpReady = true;
	//packetSize = mpu.dmpGetFIFOPacketSize();

	return ESP_OK;
}

void MPU6050_user::debug_sensor(){
	printf("ax: %d\t ay: %d\t az: %d\t", ax, ay, az);
	printf("gx: %d\t gy: %d\t gz: %d\n", gx, gy, gz);
	//ESP_LOGI(TAG, "q0: %f q1: %f q2: %f q3: %f", q0, q1, q2, q3);
}


void MPU6050_user::taskUpdate(void *param) {
	MPU6050_user* self = static_cast<MPU6050_user*>(param);

	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms周期

	while (1) {
		if (self->read_bytes(MPU6050_ACCEL_XOUT_H_REG, self->data, 14) == ESP_OK) {
			self->ax = (self->data[0] << 8) | self->data[1];
			self->ay = (self->data[2] << 8) | self->data[3];
			self->az = (self->data[4] << 8) | self->data[5];

			self->gx = (self->data[8] << 8) | self->data[9];
			self->gy = (self->data[10] << 8) | self->data[11];
			self->gz = (self->data[12] << 8) | self->data[13];
			
			//ESP_LOGI(TAG, "Accel: X=%d Y=%d Z=%d ",ax, ay, az);
			//printf("Accel: X=%d\t Y=%d\t Z=%d\n ",self->ax, self->ay, self->az);
			//printf("Gyro: X=%d\t Y=%d\t Z=%d\n ",self->gx, self->gy, self->gz);
		}
		else {
		}
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

extern "C" void MPU6050_user::start() {
		xTaskCreate(MPU6050_user::taskUpdate, "mpu_taskUpdate", 4096, this, 5, NULL);
    }
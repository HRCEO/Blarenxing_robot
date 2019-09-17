/*
 * mpu6050_i2c.c
 *
 * Created: 2019-06-10 오후 4:08:16
 *  Author: embedded
 */ 

 #include <avr/io.h>
 #include <util/twi.h>
 #include <util/delay.h>

 #include "i2c.h"
 #include "mpu6050_i2c.h"

 static int write_a_byte(uint8_t addr, uint8_t data);
 static int read_n_bytes(uint8_t addr, uint8_t *data, int n);

 int MPU6050I2CInit(int32_t i2c_clk)
 {
	i2c_init(i2c_clk);
	// Reset MPU6050 (wake-up, reset and wait 100msec)
	write_a_byte(MPU6050_PWR_MGMT_1, 0);
	write_a_byte(MPU6050_PWR_MGMT_2, 0);
	_delay_ms(10);_delay_ms(10);_delay_ms(10);
	_delay_ms(10);_delay_ms(10);_delay_ms(10);
	_delay_ms(10);

	// Reset signal path for I2C Interface
	write_a_byte(MPU6050_SIGNAL_PATH_RESET, 7);
	_delay_ms(10);_delay_ms(10);_delay_ms(10);
	_delay_ms(10);_delay_ms(10);_delay_ms(10);
	_delay_ms(10);
	write_a_byte(MPU6050_SIGNAL_PATH_RESET, 0);

	_delay_ms(10);

	return 0;
 }

 static float acc_scale = 9.8/16384.;
 static float gyro_sacle = ((3.1415926/180.) / 131.);
 int MPU6050I2CSetAccRange(uint8_t range)
 {
	if(write_a_byte(MPU6050_ACCEL_CONFIG, range <<3) < 0) return -1;

	acc_scale = pow(2, range) * (9.8/16384.);

	return 0;
 }
 int MPU6050I2CSetGyroRange(uint8_t range)
 {
	if(write_a_byte(MPU6050_GYRO_CONFIG, range<<3) < 0) return -1;
	gyro_sacle = pow(2, range) * ((3.1415926/180.) / 131.);
	return 0;
 }//Read IMU Data
 int MPU6050I2CReadIMU(int16_t acc[], int16_t gyro[])
 {
	int i;
	unsigned char buf[14];
	unsigned char *ptr_acc, *ptr_gyro;
	if(read_n_bytes(MPU6050_ACCEL_XOUT_H, buf, 14) < 0) return -1;

	ptr_acc = (unsigned char *) acc;
	ptr_gyro = (unsigned char *) gyro;

	for(i = 0; i < 6 ; i+=2)
	{
		ptr_acc[i] = buf[i+1];
		ptr_acc[i+1] = buf[i];
		ptr_gyro[i] = buf[i+8+1];
		ptr_gyro[i+1] = buf[i+8];
	}
	return 0;
 }
 //Read IMU Data float
 int MPU6050I2CReadIMU_f(float acc_f[], float gyro_f[])
 {
	int i;
	unsigned char buf[14];
	unsigned char *ptr_acc, *ptr_gyro;
	int16_t acc[3], gyro[3];

	if(read_n_bytes(MPU6050_ACCEL_XOUT_H, buf, 14) < 0) return -1;

	ptr_acc = (unsigned char *) acc;
	ptr_gyro = (unsigned char *) gyro;

	for(i = 0 ; i < 6 ; i+=2)
	{
		ptr_acc[i] = buf[i+1];  //    0 1  2 3   4 5
		ptr_acc[i+1] = buf[i];  //    1 0  3 2   5 4
		ptr_gyro[i] = buf[i+1+8]; //  0 9  2 11  4 13
		ptr_gyro[i+1] = buf[i+8]; //  1 8  3 10  5 12  
	}
	acc_f[0] = acc_scale * (acc[0]);
	acc_f[1] = acc_scale * (acc[1]);
	acc_f[2] = acc_scale * (acc[2]);
	gyro_f[0] = gyro_sacle * (gyro[0]);
	gyro_f[1] = gyro_sacle * (gyro[1]);
	gyro_f[2] = gyro_sacle * (gyro[2]);

	return 0;
 }
 // Write single byte
 static int write_a_byte(uint8_t addr, uint8_t data)
 {
	uint8_t buffer[2];

	buffer[0] = addr;
	buffer[1] = data;
	return(i2c_write_n_bytes(MPU6050_I2C_ADDRESS, buffer, 2, 1));
 }
 // read n byte
 static int read_n_bytes(uint8_t addr, uint8_t *data, int n)
 {
	//send addr without STOP condition
	if(i2c_write_a_byte(MPU6050_I2C_ADDRESS, addr | 0x80, 0) < 0) return -1;
	return(i2c_read_n_bytes(MPU6050_I2C_ADDRESS, data, n));
	
 }
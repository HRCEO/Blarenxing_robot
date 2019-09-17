/*
 * i2c.h
 *
 * Created: 2019-06-10 오후 3:21:16
 *  Author: embedded
 */ 


#ifndef I2C_H_
#define I2C_H_

void i2c_init(int32_t hz);
int i2c_write_a_byte(uint8_t i2c_addr, uint8_t buffer, int with_stop);
int i2c_write_n_bytes(uint8_t i2c_addr, uint8_t *buffer, int n, int with_stop);
int i2c_read_n_bytes(uint8_t i2c_addr, uint8_t *data, int n);

#endif /* I2C_H_ */
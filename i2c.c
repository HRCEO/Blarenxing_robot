/*
 * i2c.c
 *
 * Created: 2019-06-10 오후 3:23:01
 *  Author: embedded
 */ 
#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>

#include "i2c.h"

//Initialize I2C bus
// input: hz - data transfer rate in Hz
void i2c_init(int32_t hz)
{
	TWSR = 1<<TWPS0;	//비트율 프리스케일러 값 1
	TWBR = (uint8_t) ((F_CPU / hz - 16L) / (2L*1L));
}

#define TIME_OUT_CNT	10

//write a byte on I2C.
// i2c_addr, data, with_stop, return 0: success negative: fail to write
int i2c_write_a_byte(uint8_t i2c_addr, uint8_t data, int with_stop)
{
	int time_out = TIME_OUT_CNT;
	while(1)
	{

		//send Start Condition
		TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
		// Wait until Start condition is sent.
		while(!(TWCR & (1<<TWINT)));

		// Check if START condition is sent normally.
		if((TWSR & TW_STATUS_MASK) != TW_START) return -1;

		TWDR = (i2c_addr<<1);		//SLA_W
		TWCR = (1<<TWINT) | (1<<TWEN);	//send SLA_W
		// Wait until SLA_W is sent.
		while(!(TWCR & (1<<TWINT)));

		// check if slave returns acknowledgment
		if((TWSR & TW_STATUS_MASK) != TW_MT_SLA_ACK)
		{
			// send Stop condition
			TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
			//wait until STOP condition is sent.
			while(TWCR & (1<<TWSTO));
			time_out--;
			if(time_out <= 0)  return -2;
			_delay_us(10);
		}
		else break;
	}

	//Write data to slave
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);	//send data
	while(!(TWCR & (1<<TWINT)));	//wait until data is send

	//check if ACK is received from after sending data
	if((TWSR & TW_STATUS_MASK) != TW_MT_DATA_ACK) return -3;

	if(with_stop)
	{
		TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
		//wait until STOP condition is sent and release bus
		while((TWCR & (1<<TWSTO)));
	}
	return 0;
}

// write n bytes on I2C.
// return : 0 - success
//          negative - fail to write
int i2c_write_n_bytes(uint8_t i2c_addr, uint8_t *buffer, int n, int with_stop)
{
	int i, time_out = TIME_OUT_CNT;

	while(1)
	{
		//send START condition
		TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

		// Wait until START condition is sent.
		while(!(TWCR & (1<<TWINT)));
		//check if START condition is sent normally.
		if((TWSR & TW_STATUS_MASK) != TW_START) return -1;

		TWDR = (i2c_addr<<1); //SLA_W
		TWCR = (1<<TWINT) | (1<<TWEN);	//send SLA_W
		while(!(TWCR & (1<<TWINT)));

		//check if slave returns acknowledgment
		if((TWSR & TW_STATUS_MASK) != TW_MT_SLA_ACK)
		{
			//send STOP condition.
			TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
			//wait until STOP condition is sent.
			while(TWCR & (1<<TWSTO));

			time_out--;
			if(time_out<=0) return -2;
			_delay_us(10);
		}
		else break;
	}
	for(i = 0 ; i<n; i++)
	{
		//write data
		//write i-th data to slave
		TWDR = buffer[i];
		TWCR = (1<<TWINT) | (1<<TWEN);	//send data
		while(!(TWCR & (1<<TWINT)));
		//check if ACK is received after sending data
		if((TWSR & TW_STATUS_MASK) != TW_MT_DATA_ACK) return -3;
	}

	if(with_stop)
	{
		//send STOP condition
		TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
		//wait until STOP condition is sent.
		while((TWCR & (1<<TWSTO)));
	}

	return 0;
}


//read n bytes on I2C
// return 0 - success
//        negative - fail
int i2c_read_n_bytes(uint8_t i2c_addr, uint8_t *data, int n)
{
	int i, time_out = TIME_OUT_CNT;

	while(1)
	{
		//send START or REPEATED START condition
		TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
		//wait until START condition is sent.
		while(!(TWCR & (1<<TWINT)));
		//check if START condition is sent normally.
		if(!((TWSR & TW_STATUS_MASK) == TW_START ||
		     (TWSR & TW_STATUS_MASK) == TW_REP_START)) return -1;
		TWDR = (i2c_addr << 1) | 0x01;	//SLA_R
		TWCR = (1<<TWINT) | (1<<TWEN);	// send SLA_R
		while(!(TWCR & (1<<TWINT)));
		if((TWSR & TW_STATUS_MASK) != TW_MR_SLA_ACK)
		{
			//send STOP conditioin
			TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
			while(TWCR & (1<<TWSTO));
			time_out--;
			if(time_out <=0 ) return -2;
			_delay_us(10);
		}
		else break;
	}

	for(i = 0 ; i<n-1 ; i++)
	{

		//send ACK after receiving data
		TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
		while(!(TWCR & (1<<TWINT)));
		if((TWSR & TW_STATUS_MASK) != TW_MR_DATA_ACK) return -6;
		data[i] = TWDR;		//Read data
	}

	//if read the last byte, send NACK.
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

	if((TWSR & TW_STATUS_MASK) != TW_MR_DATA_NACK) return -7;

	data[i] = TWDR;		//read the last data
	//send STOP condition
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
	while(TWCR & (1<<TWSTO));

	return 0;

}
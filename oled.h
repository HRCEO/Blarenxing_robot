/*
 * oled.h
 *
 * Created: 2019-06-28 오전 9:41:00
 *  Author: kw898
 */ 


#ifndef OLED_H_
#define OLED_H_

#define i2c_address 0x3c
#define i2c_opcode 0x00
#define i2c_datacode 0x40

void i2c_command(uint8_t data);
void i2c_data(uint8_t data);
void oled_init(void);
void draw_picture(char *ch);
void Start_Draw();
#endif /* OLED_H_ */
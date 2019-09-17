/*
 * Blarenxing_robot.c
 *
 * Created: 2019-06-23 오후 8:47:50
 *  Author: kw898
 
 * CW   : INT1  HIGH INT2 LOW INT3 HIGH INT4 LOW
 * CWW  : INT1  LOW INT2 HIGH INT3 LOW INT4 HIGH
 * STOP : INT1  LOW INT2 LOW INT3 LOW INT4 LOW
 
 * PORTB 1(INT1) , 2(INT2)
 * PORTB 3(INT3) , 3(INT4)
 * PORTB 5(PWM L), 6(PWM R)
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "Blarenxing_robot.h"

static volatile unsigned int on_time=0;

ISR(TIMER3_CAPT_vect)
{
	static unsigned int check_point=0;
	if(TCCR3B & (1<<ICES3))
	{
		check_point=ICR3;
		TCCR3B &=~(1<<ICES3);
	}
	else
	{
		on_time=ICR3-check_point;
		TCCR3B|= (1<<ICES3);
	}
	ETIFR=(1<<ICF3);
}
void MOTor_SevO_Micowave_set_up(unsigned int DC_Sevo_Motor_time)
{
	// MOTor Sevo SetUP Fast PWM, ICRx TOP mode, 8clk
	DDRB|= (1<<PORTB1) | (1<<PORTB2) | (1<<PORTB3) | (1<<PORTB4) | (1<<PORTB5) | (1<<PORTB6) | (1<<PORTB7); // 1,2,3,4 INT1~4 5,6 DC MOTOR / 7 SERVO
	TCCR1A|= (1<<COM1A1) | (1<<COM1B1) | (1<<COM1C1) | (1<<WGM11);
	TCCR1B|= (1<<WGM13) | (1<<WGM12) | (1<<CS11);  // 8clk-> 16Mhz MCU / 0.5us
	ICR1=DC_Sevo_Motor_time;
	
	//Ultrasonic SetUP NomalMode, 8clk
	DDRE|= (1<<PORTE6) | (0<<PORTE7); // 6 Ultrasonic / 7 Ultrasonic IN
	TCCR3B|= (1<<ICNC3) | (1<<ICES3) | (1<<CS31); // Noise Canceler, UP_Edge, 8clk 0.5us
	ETIMSK|= (1<<TICIE3);
	
	sei();
}
void Ultrasonic_control()
{
	PORTE=(1<<PORTE6);
	_delay_us(10);
	PORTE=(0<<PORTE6); 
	TCNT3=0x00; // 타이머 초기화
}
void Ultrasonic_comparison_Moter(unsigned char Initialization, unsigned char limit, int PID_Val) // Ultrasonic on time, Use setting comparison   (Time scaled distance, error bound +_10%)
{
	unsigned char Distance=(on_time/116); // 0.5us 초음파센서 DATASHEE cm/58us -> 58*2
	if(Distance>limit) Distance=limit;
	
	if(PID_Val>2)//||(Initialization*1.2<Distance))//CW
	{
		OCR1A=Voltage_ON+(PID_Val*2);
 		OCR1B=Voltage_ON+(PID_Val*2);
			
		PORTB=(ON_Signal<<PORTB1) | (OFF_Signal<<PORTB2) | (ON_Signal<<PORTB3) | (OFF_Signal<<PORTB4);
	}
	else if(PID_Val<-2)//||(Initialization*0.8>Distance)) //CCW
	{
		OCR1A=Voltage_ON+(PID_Val);
		OCR1B=Voltage_ON+(PID_Val);
		
		PORTB=(OFF_Signal<<PORTB1) | (ON_Signal<<PORTB2) | (OFF_Signal<<PORTB3) | (ON_Signal<<PORTB4);
	}
 	else
 	{
 		OCR1A=Voltage_ON;
		OCR1B=Voltage_ON;
 			
	    PORTB=(ON_Signal<<PORTB1) | (OFF_Signal<<PORTB2) | (ON_Signal<<PORTB3) | (OFF_Signal<<PORTB4);
 	}
}
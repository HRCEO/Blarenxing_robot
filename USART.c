/*
 * USART.c
 *
 * Created: 2019-08-08 오후 8:38:21
 *  Author: kw898
 */ 

#include <avr/interrupt.h>

void UART_Init(void)
{
	UCSR0B = 0x18;
	UCSR0C = 0x06;
	UBRR0L = 103;
	
	UCSR1B = 0x18;
	UCSR1C = 0x06;
	UBRR1L = 103;
}

void UART0_Putch(char ch)
{
	while(!(UCSR1A & 0x20));
	UDR0 = ch;
}
char UART0_Getch(void)
{
	while(!(UCSR0A & 0x80));
	return UDR0;
}
void UART0_Puts(char *str)
{
	int i=0;
	while(str[i] != 0)
	UART0_Putch(str[i++]);
}

void UART1_Putch(char ch)
{
	while(!(UCSR1A & 0x20));
	UDR1 = ch;
}
char UART1_Getch(void)
{
	while(!(UCSR1A & 0x80));
	return UDR1;
}
void UART1_Puts(char *str)
{
	int i=0;
	while(str[i] != 0)
	UART1_Putch(str[i++]);
}
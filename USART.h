/*
 * USART.h
 *
 * Created: 2019-08-08 오후 8:39:25
 *  Author: kw898
 */ 


#ifndef USART_H_
#define USART_H_

void UART_Init(void);

void UART0_Putch(char ch);
char UART0_Getch(void);
void UART0_Puts(char *str);

void UART1_Putch(char ch);
char UART1_Getch(void);
void UART1_Puts(char *str);


#endif /* USART_H_ */
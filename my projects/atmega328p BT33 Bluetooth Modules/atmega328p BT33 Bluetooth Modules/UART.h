#include <stdio.h>
/*
* Perform UART startup initialization.
*/
void uart_init(void);
/*
* Send one character to the UART.
*/
void USART_Transmit( char data );
/*
* Receive one character from the UART
*/

void USART_Transmit_Joystick(void);
void USART_Transmit_String(char string[]);
char USART_Receive( void );
/* CPU frequency */
#define F_CPU 16000000UL
#define UART_BAUD 115200
#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include "uart.h"
#include "ADC.h"
/*
* Initialize the UART to 9600:Enable Tx & Rx
*/
void uart_init()
{
	UBRR0 = (F_CPU / (8UL * UART_BAUD)) - 1;
	UCSR0B |= (1<<TXEN0) | (1<<RXEN0)|(1<<RXCIE0);
	UCSR0C = (1<<USBS0)|(1<<UCSZ00)|(1<<UCSZ01);
	UCSR0A |= (1<<U2X0);
}
void USART_Transmit( char data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data
	*/
	UDR0 = data;
}

char USART_Receive()
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) );
	/* Get and return received data from
	buffer */
	return UDR0;
}

void USART_Transmit_String(char string[]){
	for(int i = 0; i <= sizeof(string) + 1 ; i++){
		USART_Transmit(string[i]);
	}
}

void USART_Transmit_Joystick(void){
	//format the data packet, 32 bits is transmitted over BT USART, 16 for the U/D and 16 for the L/R
	uint32_t tx = ADC_Read(ADC_UD_CHAN);
	tx = tx << 16 | (ADC_Read(ADC_LR_CHAN));
	//transmit the data packet in groups of 8 bits(8 bits for easy hex viewing).
	for(int i = 3; i>=0; i--){
		USART_Transmit(tx >> (i*8));
	}
}
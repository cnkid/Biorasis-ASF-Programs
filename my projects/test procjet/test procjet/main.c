#define F_CPU 16000000UL
#define r_buff_size 50

#include <avr/io.h>
#include "SPI.h"
#include <inttypes.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include "mcp_2515.h"
#include "uart.h"
#include <string.h>
#include <util/delay.h>


FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

volatile int r_index;
volatile uint8_t r_buffer[r_buff_size];
volatile uint8_t r_ready;

struct can_message receiveMessage;

// Non Blocking UART from 3411

ISR(USART_RX_vect)
{
	char r_char = UDR0;												// Read in data from UART Data Register
	UDR0 = r_char;													
	if (r_char != '\r')												// Checks for return char
	{
		if (r_char == 127)											// Checks for backspace char and deletes if entered
		{
			putchar(' ');											
			putchar('\b');
			--r_index;												
		}else
		{
			r_buffer[r_index] = r_char;								// Store chars in r_buffer
			if (r_index < r_buff_size)								// Compares index to buffer size
			{
				r_index++;											
			}else
			{
				r_index = 0;										// Reset if index > buffer size
			}
		}
	}else
	{
		putchar('\n');
		r_buffer[r_index] = 0;										
		r_ready = 1;
		UCSR0B ^= (1 << RXCIE0);									// Turn off interrupt until waiting for another string
	}
}

// Resets the ready flag, index and allows interrupts to shift in data
// Also clears buffer
void uart_getstr(void){
	memset(&r_buffer, 0, r_buff_size);
	r_ready = 0;
	r_index = 0;
	UCSR0B |= (1 << RXCIE0);
}


void initialize(void){
	
	// Enable external interrupt on PD3	
	EICRA |= (1 << ISC11);
	EIMSK |= (1 << INT1);
	sei();
	
	// Initialize SPI
	SPI_init();
	
	//Initialize UART
	uart_init();
	stdout = stdin = stderr = &uart_str;
	
	// Initialize CAN
	can_init();
}


ISR(INT1_vect){
	// Upon external interrupt receive the message and print out the data
	// Clears the interrupt flag once the message is received
	EIMSK &= ~(1<<INT1);
	can_message_receive(&receiveMessage);
	printf("Received: \n");
	for(int i = 0; i < receiveMessage.length; i++){
					
		printf("%c", *(receiveMessage.data + i));
	}
	printf("\n");
	printf("Received idH: %d\n", receiveMessage.idH);
	printf("Received idL: %d\n", receiveMessage.idL);
	printf("Received length: %d\n", receiveMessage.length);	
	mcp2515_bitModify(MCP_CANINTF, 0x01, 0x00);
}

int main(void){
	// Initialize a can_messsage struct with arbitrary values for testing
	struct can_message sendMessage;
	sendMessage.idH = 0x00;
	sendMessage.idL = 0x20;
	sendMessage.length = 8;
	
	initialize();
	printf("Connected\n");
	uart_getstr();
	
	while(1){
		_delay_ms(1000);
		EIMSK |= (1 << INT1);
		if(r_ready == 1) {
			sendMessage.data = &r_buffer;									// Set the send message data to be whatever characters are typed into the UART
			printf("Sent: \n");
			for(int i = 0; i < sendMessage.length; i++){
				
				printf("%c", *(sendMessage.data + i));
			}
			
			printf("\n");
			/*
			printf("Sent idH: %d\n", sendMessage.idH);
			printf("Sent idL: %d\n", sendMessage.idL);
			printf("Sent length: %d\n", sendMessage.length);						
			*/
			can_message_send(&sendMessage);									// Send the test message
			uart_getstr();
		}
		//printf("%d\n", mcp2515_readStatus());
		//_delay_ms(100);
	}
	return 0;
}
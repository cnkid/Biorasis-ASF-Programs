/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include "asf.h"
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "SRAM.h" //custom SRAM library
#include "SPI.h"
#include "uart.h"
#include "MCP2515_defines.h"
#define BUFF_LENGTH  8
uint8_t wd_buffer[BUFF_LENGTH] = {0x02,0x0c,0xaa,0,0,0,0,0};
uint8_t rd_buffer[BUFF_LENGTH];
char a = 0xaa;
uint8_t test;
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
char rec[50]; // Declare a character buffer
uint16_t address_counter = 0x0000;
char c = 0x00;
struct can_message{
	uint8_t idH;
	uint8_t idL;
	uint8_t *data;
	uint8_t length;
	};
void mcp2515_write(uint8_t data, uint8_t address){
	SPI_Slave_Select(1);
	SPI_TransmitByte(MCP_WRITE);
	SPI_TransmitByte(address);
	SPI_TransmitByte(data);
	SPI_Slave_Select(0);
}
uint8_t mcp2515_read(uint8_t address){
	uint8_t c_data;
	SPI_Slave_Select(1);
	SPI_TransmitByte(MCP_READ);
	SPI_TransmitByte(address);
	c_data = SPI_SlaveRead();
	SPI_Slave_Select(0);
	return c_data;
}
void mcp2515_bit_modify(uint8_t address, uint8_t mask, uint8_t data){
	SPI_Slave_Select(1);
	SPI_TransmitByte(MCP_BITMOD);
	SPI_TransmitByte(address);
	SPI_TransmitByte(mask);
	SPI_TransmitByte(data);
	SPI_Slave_Select(0);
}
void mcp2515_reset(void){
	SPI_Slave_Select(1);
	SPI_TransmitByte(MCP_RESET);
	SPI_Slave_Select(0);
}
uint8_t mcp2515_read_status(void){
	uint8_t c_data;
	SPI_Slave_Select(1);
	SPI_TransmitByte(MCP_READ_STATUS);
	c_data = SPI_SlaveRead();
	SPI_Slave_Select(0);
	return c_data;
}
void mcp_can_init(void){
	mcp2515_reset();
	mcp2515_bit_modify(MCP_CANINTE, 0x03, 0x01);
	mcp2515_bit_modify(MCP_RXB0CTRL, 0x64, 0x60);
	mcp2515_bit_modify(MCP_CANINTE, 0x04, 0x00);
	mcp2515_bit_modify(0x2A,0xff,0x00);
	mcp2515_bit_modify(0x29,0xff,0xf0);
	mcp2515_bit_modify(0x28,0xff,0x86);
	//mcp2515_bit_modify(MCP_CANCTRL,0xe0, MODE_LOOPBACK);
	mcp2515_bit_modify(MCP_CANCTRL,0xe0, MODE_NORMAL);
}
void port_init(){
	DDRC = 0xff; //turn all into outputs
	DDRB = 0xff;
	PORTC = 0x00;//turn all gpio to off state (default state)
	PORTD = 0x00;
	PORTB = 0x00;
	
}
void Uart_write_string_to_sram(char string[]){
	int i = 0;
	while(rec[i] != 0){
		_delay_ms(20);
		sram_write(address_counter,rec[i]);
		address_counter++;
		i++;
	}
	sram_write(address_counter,0x20); //write SPACE char at end of string 
	address_counter++;
	UCSR0B = (1<<TXEN0) | (1<<RXEN0); //re-enable uart. 
}
void Uart_dump_sram_to_console(void){
	char temp[50];
	for (int i = 0; i < address_counter; i++){
		_delay_ms(20);
		c = sram_read(i);
		temp[i] = c;
	}
	
	UCSR0B = (1<<TXEN0) | (1<<RXEN0);
	address_counter = 0;
	fprintf(stdout, "%s", temp);
}
void mcp2515_writeArray(uint8_t address, uint8_t *data, uint8_t length){
	uint8_t i;
	
	SPI_Slave_Select(1);
	SPI_TransmitByte(MCP_WRITE);
	SPI_TransmitByte(address);
	for(i=0; i < length; i++){
		SPI_TransmitByte(data[i]);
	}
	
	SPI_Slave_Select(0);
}
uint8_t* mcp2515_readArray(uint8_t address, uint8_t length){
	uint8_t i;
	uint8_t *data_out = malloc(length);
	
	SPI_Slave_Select(1);
	SPI_TransmitByte(MCP_READ);
	SPI_TransmitByte(address);
	for(i=0; i < length; i++){
		data_out[i] = SPI_SlaveRead();
	}
	SPI_Slave_Select(0);
	
	return data_out;
}
void mcp_can_send_message(struct can_message *message){
	uint8_t message_length = (0x0F & message->length);
	
	mcp2515_bit_modify(MCP_TXB0SIDH, 0xFF, message->idH);
	mcp2515_bit_modify(0x32, 0xE0, message->idL); //MCP_TXB0SIDL
	mcp2515_bit_modify(0x35, 0x40, 0x40); //TXB0DLC
	mcp2515_bit_modify(0x35, 0x0F, message_length); //TXB0DLC
	mcp2515_writeArray(0x36, message->data, message_length); //TXB0D0
	mcp2515_bit_modify(MCP_TXB0CTRL, 0x0B, 0x0B);
}
void mcp_can_message_receive(struct can_message *received_message){
	received_message->length = mcp2515_read(0x65) & 0x0F; //rxb0dlc
	received_message->data = mcp2515_readArray(0x65, received_message->length); //RXB0D0
	received_message->idH = mcp2515_read(MCP_RXB0SIDH);
	received_message->idL = mcp2515_read(0x62); //RXB0SIDL
	
}
int main (void)
{
	board_init();
	/*port_init();
	sram_write(0x0000,0xaa);
	sram_write(0x0001,0xbb);
	sram_write(0x0002,0xcc);
	test = sram_read(0x0000); 
	test = sram_read(0x0001);
	test = sram_read(0x0002);
	*/ //uncomment for sram
	SPI_MasterInit();
	PORTB |= (1<<SPI_SS); // Pull Slave Select High
	/*
	uart_init();
	stdout = stdin = stderr = &uart_str; // Set File outputs to point to UART stream 
	fprintf(stdout, "Hello! \n");
	*/
	struct can_message mess;
	uint8_t d_data= 0xaa;
	mess.idH = 0x00;
	mess.idL = 0x40;
	mess.length = 2;
	mess.data = &d_data;
	mcp_can_init();
	mcp_can_send_message(&mess);
	mcp_can_send_message(&mess);
	mcp_can_send_message(&mess);
	
	while(1)
	{
		/*
		fscanf(stdin,"%s",rec);
		if(rec[0] == 0x44 && rec[1] == 0x53 && rec[2] == 0x52){
			UCSR0B &= ~(1<<TXEN0) & ~(1<<RXEN0);
			Uart_dump_sram_to_console();
		}
		if(rec[0] != 0){
			UCSR0B &= ~(1<<TXEN0) & ~(1<<RXEN0); //disable uart rx tx since sram shares the rx tx lines
			Uart_write_string_to_sram(rec);
		}
		*/
	}
}

/*
 * SPI.c
 *
 * Created: 8/11/2018 11:41:14 AM
 *  Author: Alexander Valdes
 */ 
#include "SPI.h"

void SPI_MasterInit(void)
{
	/* Set MOSI,SCK, AND CS output, all others input */
	DDRB = (1<<DDB3)|(1<<DDB5)|(1 << DDB2);
	/* Enable SPI, Master, set clock rate fck/16 */
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPR1);

}
void SPI_SlaveInit(void){
	DDRB = (1<<DDB4); //set miso as output all other input 
	SPCR = (1<<SPE); //enable spi
}
void SPI_Slave_Select(uint8_t cond){
	if(cond){
		PORTB &= ~(1<<SPI_SS); // Pull Slave_Select low
	}
	if(!cond){
		PORTB |= (1<<SPI_SS); // Pull Slave Select High
	}
}
void SPI_TransmitByte(uint8_t data)
{
	SPDR = data; // Start transmission
	while( !(SPSR & (1<<SPIF)) ); // Wait for transmission complete
}
void SPI_TransmitBuffer(uint8_t write_buffer[],uint8_t read_buffer[], int size)
{
	PORTB &= ~(1<<SPI_SS); // Pull Slave_Select low
	for(int i = 0; i <= size-1; i++){
		SPDR = write_buffer[i];
		while( !(SPSR & (1<<SPIF)) ); // Wait for transmission complete
		read_buffer[i] = SPDR;
	}
	PORTB |= (1<<SPI_SS); // Pull Slave Select High
	
}
uint8_t SPI_SlaveRead(void){
	SPDR = 0x00;
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}
void SPI_SlaveReadBuffer(uint8_t read_buffer[], int size){
	for(int i = 0; i <= size-1; i++){
		while( !(SPSR & (1<<SPIF)) ); // Wait for transmission complete
		read_buffer[i] = SPDR;
	}
}
/*
 * SPI.c
 *
 * Created: 7/4/2018 2:55:09 PM
 * Author : Matt
 */ 

#include "SPI.h"

void SPI_init(void){
// SPI Register Initializations

DDRB |= (1 << PINB2);					// SS
DDRB |= (1 << PINB3);					// MOSI AS OUTPUT
DDRB &= ~(1 << PINB4);					// MISO AS INPUT
DDRB |= (1 << PINB5);					// SCK

PORTB |= (1 << PINB2);					// SET SS HIGH TO BEGIN

SPCR |= (1 << SPR0) | (1 << SPR1);		// Clock / 16
SPCR |= (1 << MSTR);					// Set Master Mode
SPCR |= (1 << SPE);						// SPI Enable

}

void SPI_SendByte(uint8_t s_data){
	
	SPDR = s_data;						// Place byte to be sent into data register
	
	while(!(SPSR & (1 << SPIF)));		// Wait until transmission is complete
	
}

uint8_t SPI_ReadByte(void){
	
	SPDR = 0x00;						// Send Dummy byte

	while(!(SPSR & (1 << SPIF)));
	
	return (SPDR);						// Return the result of the shifted receive buffer
}
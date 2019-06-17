/*
 * SRAM.c
 *
 * Created: 8/10/2018 9:56:27 PM
 *  Author: Alex Valdes
 */ 
#include "SRAM.h"
#include "uart.h"
void sram_write(uint16_t address, uint8_t data){
	DDRD = 0xff; //set DDRD to output
	PORTD = 0x00; //clear reg of old values
	uint8_t address_lower = address & 0x00ff;
	uint8_t address_upper = address >> 8;
	PORTC |= (1<<SRAM_LE_LOW) | (1<<SRAM_LE_HIGH); //set latches to mirror
	PORTC |= (1<<SRAM_CE) | (1<<SRAM_WE); //set chip enable and WE to standby
	PORTD = address_lower;
	PORTC &= ~(1<<SRAM_LE_LOW);
	PORTD = address_upper;
	PORTC &= ~(1<<SRAM_LE_HIGH);
	PORTC &= ~(1<<SRAM_CE) & ~(1<<SRAM_WE);
	_delay_ms(1);
	PORTD = data;
	PORTC |= (1<<SRAM_CE) | (1<<SRAM_WE);
}
uint8_t sram_read(uint16_t address){
	uint8_t data;
	DDRD = 0xff; //set DDRD to output
	PORTD = 0x00; // clear reg of old values
	uint8_t address_lower = address & 0x00ff;
	uint8_t address_upper = address >> 8;
	PORTC |= (1<<SRAM_LE_LOW) | (1<<SRAM_LE_HIGH); //set latches to mirror
	PORTC |= (1<<SRAM_CE) | (1<<SRAM_WE); //set chip enable and WE to standby
	PORTD = address_lower;
	PORTC &= ~(1<<SRAM_LE_LOW);
	PORTD = address_upper;
	PORTC &= ~(1<<SRAM_LE_HIGH);
	DDRD = 0x00; //turn port into input
	PORTC &= ~(1<<SRAM_CE);
	_delay_ms(1);
	data = PIND;
	PORTC |= (1<<SRAM_CE);
	return data;
}

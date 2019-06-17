/*
 * SRAM.h
 *
 * Created: 8/10/2018 9:56:40 PM
 *  Author: Alex Valdes
 */ 

#include <avr/io.h>
#include <util/delay.h>
#ifndef SRAM_H_
#define SRAM_H_
#define SRAM_CONTROL_PORT_DDR        DDRD
#define SRAM_CONTROL_PORT            PORTD
#define SRAM_LE_LOW                  PORTC0 //when high mirror input, when low hold adress
#define SRAM_LE_HIGH				 PORTC3
#define SRAM_CE                      PORTC1
#define SRAM_WE						 PORTC2
void sram_write(uint16_t address, uint8_t data);
uint8_t sram_read(uint16_t address);
#endif /* SRAM_H_ */
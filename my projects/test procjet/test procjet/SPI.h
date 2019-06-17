/*
 * SPI.h
 *
 * Created: 7/4/2018 2:55:09 PM
 * Author : Matt
 */

#ifndef SPI
#define SPI

#include <avr/io.h>
#include <inttypes.h>

// SS Shortcuts
#define SLAVE_SELECT	PORTB &= ~(1 << PINB2);
#define SLAVE_DESELECT	PORTB |= (1 << PINB2);

// Initialize SPI with standard settings
void SPI_init(void);

// Send a byte over the SPI bus
void SPI_SendByte(uint8_t s_data);

// Read a byte from the SPI bus
uint8_t SPI_ReadByte(void);

#endif






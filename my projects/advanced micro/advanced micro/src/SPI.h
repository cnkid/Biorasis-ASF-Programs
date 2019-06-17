/*
 * SPI.h
 *
 * Created: 8/11/2018 11:41:25 AM
 *  Author: Alexander Valdes
 */ 


#ifndef SPI_H_
#define SPI_H_
#include <avr/io.h>
#include <util/delay.h>
#define SPI_SS PINB2

void SPI_MasterInit(void);
void SPI_SlaveInit(void);
void SPI_TransmitByte(uint8_t data);
void SPI_TransmitBuffer(uint8_t write_buffer[],uint8_t read_buffer[], int size);
uint8_t SPI_SlaveRead(void);
void SPI_SlaveReadBuffer(uint8_t read_buffer[], int size);
void SPI_Slave_Select(uint8_t cond);
#endif /* SPI_H_ */
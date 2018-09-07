/*
 * SPIMOD.c
 *
 * Created: 7/17/2018 11:19:00 AM
 *  Author: Alex Valdes
 * Contains functions for writing and reading to WINBOND W25Q Flash memories using ASF. Uses a standard SPI command structure. 
 *
 */ 
#include "SPIMOD.h"
#include <asf.h>
uint8_t command_buffer[260];
uint8_t busy_bit[1];
enum status_code status_out;	
enum status_code spi_erase_chip(struct packet datain){
	spi_enable_write(datain);
	command_buffer[0] = FLASH_ERASE_CHIP;
	spi_select_slave(&datain.master, &datain.slave, true);
	spi_write_buffer_wait(&datain.master, command_buffer,1);
	spi_select_slave(&datain.master, &datain.slave, false);
	delay_s(6);
	return status_out;
};
enum status_code spi_enable_write(struct packet datain){
	command_buffer[0] = FLASH_ENABLE_WRITE;
	spi_select_slave(&datain.master, &datain.slave, true);
	while(spi_write_buffer_wait(&datain.master,command_buffer,1) != STATUS_OK){
		
	}
	status_out = STATUS_OK;
	spi_select_slave(&datain.master, &datain.slave, false);
	delay_ms(15);
	return status_out;
}; 
enum status_code spi_status_register(struct packet datain){
	command_buffer[0] = FLASH_READ_STATUS_REGISTER;
	spi_select_slave(&datain.master, &datain.slave, true);
	status_out = spi_transceive_buffer_wait(&datain.master,command_buffer,datain.data,2);
	spi_select_slave(&datain.master, &datain.slave, false);
	delay_us(1);
	return status_out;
};
enum status_code spi_read_256(struct packet datain){
	command_buffer[0] = FLASH_FAST_READ;
	command_buffer[1] = (datain.address & 0x00ff0000) >> 16;
	command_buffer[2] = (datain.address & 0x00ff0000) >> 8;
	command_buffer[3] = (datain.address & 0x00ff0000);
	spi_select_slave(&datain.master, &datain.slave, true);
	spi_transceive_buffer_wait(&datain.master, command_buffer ,datain.data,261);
	spi_select_slave(&datain.master, &datain.slave, false);
	delay_us(1);
	return status_out;
};
enum status_code spi_write_page(struct packet datain){
	spi_enable_write(datain);
	command_buffer[0] = FLASH_PAGE_PROGRAM;
	command_buffer[1] = (datain.address & 0x00ff0000) >> 16;
	command_buffer[2] = (datain.address & 0x00ff0000) >> 8;
	command_buffer[3] = (datain.address & 0x00ff0000);
	memcpy(command_buffer + 4, datain.data,256);
	spi_select_slave(&datain.master, &datain.slave, true);
	spi_write_buffer_wait(&datain.master,command_buffer,260);
	spi_select_slave(&datain.master, &datain.slave, false);
	delay_ms(3);
	return status_out;
};
enum status_code spi_overwrite_page(struct packet datain){
	spi_enable_write(datain);
	command_buffer[0] = FLASH_PAGE_PROGRAM;
	command_buffer[1] = (datain.address & 0x00ff0000) >> 16;
	command_buffer[2] = (datain.address & 0x00ff0000) >> 8;
	command_buffer[3] = (datain.address & 0x00ff0000);
	memcpy(command_buffer + 4, datain.data,256);
	spi_select_slave(&datain.master, &datain.slave, true);
	spi_write_buffer_wait(&datain.master,command_buffer,260);
	spi_select_slave(&datain.master, &datain.slave, false);
	return status_out;
};

enum status_code spi_erase_sector_4KB(struct packet datain){
	spi_enable_write(datain);
	command_buffer[0] = FLASH_ERASE_SECTOR_4KB;
	command_buffer[1] = (datain.address & 0x00ff0000) >> 16;
	command_buffer[2] = (datain.address & 0x00ff0000) >> 8;
	command_buffer[3] = (datain.address & 0x00ff0000);
	spi_select_slave(&datain.master, &datain.slave, true);
	spi_write_buffer_wait(&datain.master,command_buffer,260);
	spi_select_slave(&datain.master, &datain.slave, false);
	delay_ms(100);
	return status_out;
};
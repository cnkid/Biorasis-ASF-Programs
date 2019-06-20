/*
 * SPIMOD.h
 *
 * Created: 7/17/2018 11:19:12 AM
 *  Author: Alex Valdes
 */ 


#ifndef SPIMOD_H_
#define SPIMOD_H_
#include <asf.h>
#define FLASH_PAGE_PROGRAM 0x02
#define FLASH_ENABLE_WRITE 0x06
#define FLASH_FAST_READ 0x0b
#define FLASH_READ 0x03
#define FLASH_READ_STATUS_REGISTER 0x05
#define FLASH_ERASE_CHIP 0xc7
#define FLASH_ERASE_SECTOR_4KB 0x20
typedef struct packet{
	struct spi_module master;
	struct spi_slave_inst slave;
	uint32_t address;
	uint8_t *data;
};
enum status_code spi_erase_chip(struct packet datain);
enum status_code spi_erase_sector_4KB(struct packet datain);
enum status_code spi_enable_write(struct packet datain);
enum status_code spi_status_register(struct packet datain);
enum status_code spi_write_page(struct packet datain);
enum status_code spi_read_256(struct packet datain);
enum status_code spi_write_test(struct packet datain);
enum status_code spi_overwrite_page(struct packet datain);




#endif /* SPIMOD_H_ */
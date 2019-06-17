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
 * This is a bare minimum user application template.
 *
 * For documentation of the board, go \ref group_common_boards "here" for a link
 * to the board-specific documentation.
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# Basic usage of on-board LED and button
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
#include <asf.h>

#define BUF_LENGTH 20
static uint8_t rd_buffer[BUF_LENGTH];
static uint8_t buffer_rx[BUF_LENGTH] = {};
static uint8_t wr_buffer[BUF_LENGTH] = {
};
static uint8_t WREN[1] = {0x06};
#define SLAVE_SELECT_PIN		CONF_MASTER_SS_PIN
#define CONF_MASTER_SPI_MODULE  EXT2_SPI_MODULE
#define CONF_MASTER_SS_PIN      EXT2_PIN_SPI_SS_0
#define CONF_MASTER_MUX_SETTING EXT2_SPI_SERCOM_MUX_SETTING
#define CONF_MASTER_PINMUX_PAD0 EXT2_SPI_SERCOM_PINMUX_PAD0
#define CONF_MASTER_PINMUX_PAD1 PINMUX_UNUSED
#define CONF_MASTER_PINMUX_PAD2 EXT2_SPI_SERCOM_PINMUX_PAD2
#define CONF_MASTER_PINMUX_PAD3 EXT2_SPI_SERCOM_PINMUX_PAD3
#define CONF_SLAVE_SPI_MODULE  EXT1_SPI_MODULE
#define CONF_SLAVE_MUX_SETTING EXT1_SPI_SERCOM_MUX_SETTING
#define CONF_SLAVE_PINMUX_PAD0 EXT1_SPI_SERCOM_PINMUX_PAD0
#define CONF_SLAVE_PINMUX_PAD1 EXT1_SPI_SERCOM_PINMUX_PAD1
#define CONF_SLAVE_PINMUX_PAD2 EXT1_SPI_SERCOM_PINMUX_PAD2
#define CONF_SLAVE_PINMUX_PAD3 EXT1_SPI_SERCOM_PINMUX_PAD3
#define CONF_PERIPHERAL_TRIGGER_TX   SERCOM1_DMAC_ID_TX
#define CONF_PERIPHERAL_TRIGGER_RX   SERCOM0_DMAC_ID_RX

struct spi_module spi_master_instance;
struct spi_slave_inst slave;
int result = 0;
volatile bool transrev_complete_spi_master = false;
enum status_code status;
int statd;

void configure_pins(void){
	    struct port_config config_port_pin;
	    port_get_config_defaults(&config_port_pin);
	    config_port_pin.direction  = PORT_PIN_DIR_INPUT;
	    config_port_pin.input_pull = PORT_PIN_PULL_UP;
	    port_pin_set_config(PIN_PA16, &config_port_pin);
	    
}
void configure_spi_master(void)
{
	struct spi_config config_spi_master;
	struct spi_slave_inst_config slave_dev_config;
	/* Configure and initialize software device instance of peripheral slave */
	spi_slave_inst_get_config_defaults(&slave_dev_config);
	slave_dev_config.ss_pin = PIN_PA10;
	spi_attach_slave(&slave, &slave_dev_config);
	/* Configure, initialize and enable SERCOM SPI module */
	spi_get_config_defaults(&config_spi_master);
	config_spi_master.mux_setting = CONF_MASTER_MUX_SETTING;
	config_spi_master.transfer_mode = SPI_TRANSFER_MODE_0;
	config_spi_master.pinmux_pad0 = CONF_MASTER_PINMUX_PAD0;
	config_spi_master.pinmux_pad1 = CONF_MASTER_PINMUX_PAD1;
	config_spi_master.pinmux_pad2 = CONF_MASTER_PINMUX_PAD2;
	config_spi_master.pinmux_pad3 = CONF_MASTER_PINMUX_PAD3;
	spi_init(&spi_master_instance, CONF_MASTER_SPI_MODULE, &config_spi_master);
	spi_enable(&spi_master_instance);
}

static void callback_spi_master( struct spi_module *const module)
{
	transrev_complete_spi_master = true;
}
void configure_spi_master_callbacks(void)
{
	spi_register_callback(&spi_master_instance, callback_spi_master, SPI_CALLBACK_BUFFER_TRANSCEIVED);
	spi_enable_callback(&spi_master_instance, SPI_CALLBACK_BUFFER_TRANSCEIVED);
}
void spi_command(uint8_t command_buffer[BUF_LENGTH]){
	if (1) {
		spi_select_slave(&spi_master_instance, &slave, true);
		spi_transceive_buffer_job(&spi_master_instance, command_buffer,rd_buffer,BUF_LENGTH);
		while (!transrev_complete_spi_master) {
			
		}
		transrev_complete_spi_master = false;
		spi_select_slave(&spi_master_instance, &slave, false);
	}
}
int main (void)
{
	    system_init();
		configure_pins();
	    configure_spi_master();
	    configure_spi_master_callbacks();
		
		
		    while (true) {
				memcpy(wr_buffer, (uint8_t[]){0x06}, 1);
				spi_select_slave(&spi_master_instance, &slave, true);
				statd = spi_write_buffer_wait(&spi_master_instance, wr_buffer,1);
				spi_select_slave(&spi_master_instance, &slave, false);
				
				memcpy(wr_buffer, (uint8_t[]){0x02,0x0F,0x00,0x00,0xaa,0xaa,0xaa}, 7);	
				spi_select_slave(&spi_master_instance, &slave, true);
				statd = spi_write_buffer_wait(&spi_master_instance, wr_buffer,7);
				spi_select_slave(&spi_master_instance, &slave, false);
				
				memcpy(wr_buffer, (uint8_t[]){0x0b,0x0F,0x00,0x00}, 4);	
				spi_select_slave(&spi_master_instance, &slave, true);
				statd = spi_transceive_buffer_wait(&spi_master_instance, wr_buffer ,rd_buffer,BUF_LENGTH);
				spi_select_slave(&spi_master_instance, &slave, false);
				
				 
				memcpy(wr_buffer, (uint8_t[]){0x06}, 1);
				spi_write_buffer_job(&spi_master_instance, wr_buffer,1);
		    }
}

//HOW TO WRITE
//if you want a single command (SUCH AS WREN) select slave, spi_write out command, disable slave
//use transcieve to get data back
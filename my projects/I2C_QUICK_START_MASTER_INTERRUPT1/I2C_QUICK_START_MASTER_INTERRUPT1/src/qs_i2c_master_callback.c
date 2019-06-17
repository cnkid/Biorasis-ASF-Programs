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
#include <system_interrupt.h>
#define DATA_LENGTH 3
static uint8_t write_buffer[DATA_LENGTH] = {
	0x00,0x1,0xda
};
static uint8_t write_buffer_random[2] = {
	0x00,0x00
};
uint8_t tempstatus =0x00; 
#define MAX_RX_BUFFER_LENGTH   5
volatile uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];
static uint8_t read_buffer[4];
#define SLAVE_ADDRESS 0x57
#define TIMEOUT 3000
char snum[5];

void configure_port_pins(void)
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction  = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(BUTTON_0_PIN, &config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED_0_PIN, &config_port_pin);
}
struct i2c_master_module i2c_master_instance;
void configure_i2c_master(void)
{
	/* Initialize config structure and software module. */
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);
	/* Change buffer timeout to something longer. */
	config_i2c_master.buffer_timeout = 10000;
	config_i2c_master.pinmux_pad0 = EXT2_I2C_SERCOM_PINMUX_PAD0;
	config_i2c_master.pinmux_pad1 = EXT2_I2C_SERCOM_PINMUX_PAD1;
	/* Initialize and enable device with config. */
	i2c_master_init(&i2c_master_instance, CONF_I2C_MASTER_MODULE, &config_i2c_master);
	i2c_master_enable(&i2c_master_instance);
}
struct usart_module usart_instance;
void usart_read_callback(struct usart_module *const usart_module)
{
	usart_write_buffer_job(&usart_instance,
	(uint8_t *)rx_buffer, MAX_RX_BUFFER_LENGTH);
}
void usart_write_callback(struct usart_module *const usart_module)
{
	
}
void configure_usart(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate    = 9600;
	config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	while (usart_init(&usart_instance,
	EDBG_CDC_MODULE, &config_usart) != STATUS_OK) {
	}
	usart_enable(&usart_instance);
}
void configure_usart_callbacks(void)
{
	usart_register_callback(&usart_instance,
	usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_instance,
	usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}


int main (void)
{
	system_init();
	configure_i2c_master();
	configure_port_pins();
	delay_init();
	configure_usart();
	configure_usart_callbacks();
	system_interrupt_enable_global();
	uint8_t string[] = "Hello World!\r\n";
	usart_write_buffer_wait(&usart_instance, string, sizeof(string));
	uint16_t timeout = 0;
    struct i2c_master_packet packet = {
		.address     = SLAVE_ADDRESS,
		.data_length = DATA_LENGTH,
		.data        = write_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};
	struct i2c_master_packet packet_random = {
		.address     = SLAVE_ADDRESS,
		.data_length = 3,
		.data        = write_buffer_random,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	while (1) {
		/*
			tempstatus = i2c_master_write_packet_wait(&i2c_master_instance, &packet);
		    while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK) {
			    if (timeout++ == TIMEOUT) {
					port_pin_set_output_level(LED_0_PIN, false);
				    break;
			    }
		    }
			
			write_buffer[0] = 0x00;
			write_buffer[1] = 0x02;
			write_buffer[2] = 0xdb;
			delay_s(1);
			tempstatus = i2c_master_write_packet_wait(&i2c_master_instance, &packet);
			while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK) {
				if (timeout++ == TIMEOUT) {
					port_pin_set_output_level(LED_0_PIN, false);
					break;
				}
			}
			write_buffer[0] = 0x00;
			write_buffer[1] = 0x03;
			write_buffer[2] = 0xdd;
			delay_s(1);
			tempstatus = i2c_master_write_packet_wait(&i2c_master_instance, &packet);
			while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK) {
				if (timeout++ == TIMEOUT) {
					port_pin_set_output_level(LED_0_PIN, false);
					break;
				}
			}
			delay_s(1);
			write_buffer[0] = 0x00;
			write_buffer[1] = 0x04;
			write_buffer[2] = 0xde;
			delay_s(1);
			tempstatus = i2c_master_write_packet_wait(&i2c_master_instance, &packet);
			while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK) {
				if (timeout++ == TIMEOUT) {
					port_pin_set_output_level(LED_0_PIN, false);
					break;
				}
			}
			delay_s(1);
			
			*/
			
			while(1){
			tempstatus = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &packet_random);
			while (i2c_master_write_packet_wait(&i2c_master_instance, &packet_random) != STATUS_OK) {
				if (timeout++ == TIMEOUT) {
					port_pin_set_output_level(LED_0_PIN, false);
					break;
				}
			}
			delay_ms(1);
			
		    packet_random.data = read_buffer;
			tempstatus = i2c_master_read_packet_wait(&i2c_master_instance, &packet_random);
		    while (tempstatus != STATUS_OK) {
			    if (timeout++ == 10000) {
					port_pin_set_output_level(LED_0_PIN, false);
				    break;
			    }
		    }

			itoa(tempstatus,snum,16);
			strcat(snum,"\n");
			usart_write_buffer_wait(&usart_instance, snum, sizeof(snum));
			}
	}
}

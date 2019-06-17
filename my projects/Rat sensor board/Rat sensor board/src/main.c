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
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
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
#include "SPIMOD.h"
#include <stdint.h>
#define DEBUG_PIN PIN_PA02
#define LED_0 PIN_PA04
#define PWM_MODULE      EXT1_PWM_MODULE
#define PWM_OUT_PIN     EXT1_PWM_0_PIN
#define PWM_OUT_MUX     EXT1_PWM_0_MUX
#define FREQ_PIN		PIN_PA18 //set to freq in pin
#define FREQ_MUX		MUX_PA18A_EIC_EXTINT2 //set to pins interrupt mask
#define BOARD_PHOTODIODE_GEN EVSYS_ID_GEN_EIC_EXTINT_2 //make this the same EXTINT mask
#define INPUT_BUFFER_SIZE 11
#define PULSE_TIME 100
#define OFF_TIME 6000
#define FLASH_PWR PIN_PA25
#define SENSOR_PWR PIN_PA16
#define STATUS_LED PIN_PA07
#define READ_MODE 1
enum sleepmgr_mode delta = SLEEPMGR_ACTIVE;
struct usart_module usart_terminal;
struct spi_module spi_master_instance;
struct spi_slave_inst slave;
struct rtc_module rtc_instance;
struct extint_events eic_events;
struct tc_events events_tc;
struct rtc_module rtc_instance;
//struct rtc_calendar_alarm_time alarm;
struct usart_module usart_instance;
//struct rtc_calendar_time time;
struct tc_module capture_instance;
enum status_code status_out;	
enum sleepmgr_mode modus;
struct packet send_data;
struct packet get_data;
const int Captureclock = 48000000;
uint64_t test;
uint32_t time_stamp = 0;
bool count_overflow = false;
uint32_t rtc_counter = 0;
uint32_t rollover = 0;
uint32_t frequency = 0;
uint16_t counter = 0;
uint8_t measure_count = 0;
char input_buffer[15];
static uint8_t command_buffer[260];
static uint8_t rd_buffer[261];
static uint8_t out_buffer[256];
static uint8_t wd_buffer[256];
static uint8_t overflow_buffer[200];
static uint64_t to_be_sent[32];
bool sleepflag = false;
bool controlflag = false;
uint64_t timeout_timer = 0;
uint32_t Number = 520000; // number to convert	
uint8_t page_data[EEPROM_PAGE_SIZE];
bool cond = true; // condition to see if spi is done writting (false == done)
char testbuf[2];
char Result[32]; // string which will contain the number



void configure_eeprom(void)
{
	enum status_code error_code = eeprom_emulator_init();
	if (error_code == STATUS_ERR_NO_MEMORY) {
		while (true) {
			/* No EEPROM section has been set in the device's fuses */
		}
	}
	else if (error_code != STATUS_OK) {
		eeprom_emulator_erase_memory();
		eeprom_emulator_init();
	}
}
#if (SAMD || SAMR21)
void SYSCTRL_Handler(void)
{
	if (SYSCTRL->INTFLAG.reg & SYSCTRL_INTFLAG_BOD33DET) {
		SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33DET;
		eeprom_emulator_commit_page_buffer();
	}
}
#endif
static void configure_bod(void)
{
	#if (SAMD || SAMR21)
	struct bod_config config_bod33;
	bod_get_config_defaults(&config_bod33);
	config_bod33.action = BOD_ACTION_INTERRUPT;
	/* BOD33 threshold level is about 3.2V */
	config_bod33.level = 48;
	bod_set_config(BOD_BOD33, &config_bod33);
	bod_enable(BOD_BOD33);

	SYSCTRL->INTENSET.reg = SYSCTRL_INTENCLR_BOD33DET;
	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_SYSCTRL);
	#endif
}
void configure_terminal(void)
{
	struct usart_config config_terminal;
	usart_get_config_defaults(&config_terminal);
	config_terminal.baudrate    = 256000;
	config_terminal.mux_setting = USART_RX_1_TX_0_XCK_1;
	config_terminal.pinmux_pad0 = PINMUX_PA04D_SERCOM0_PAD0;
	config_terminal.pinmux_pad1 = PINMUX_PA05D_SERCOM0_PAD1;
	config_terminal.pinmux_pad2 = PINMUX_UNUSED;
	config_terminal.pinmux_pad3 = PINMUX_UNUSED;
	while (usart_init(&usart_terminal,SERCOM0, &config_terminal) != STATUS_OK) {
		
	}
	usart_enable(&usart_terminal);
}
void configure_port_pins(void)
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction  = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(DEBUG_PIN,&config_port_pin);
	port_pin_set_config(PIN_PA18, &config_port_pin);
	config_port_pin.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(SENSOR_PWR, &config_port_pin);
	port_pin_set_config(STATUS_LED, &config_port_pin);
	port_pin_set_config(PIN_PA15,&config_port_pin);
	port_pin_set_config(FLASH_PWR,&config_port_pin);
	port_pin_set_output_level(SENSOR_PWR,0);
}
void configure_pinmux(void){
	struct system_pinmux_config config_pinmux;
	system_pinmux_get_config_defaults(&config_pinmux);
	config_pinmux.mux_position = PINMUX_PA22H_GCLK_IO6;
	config_pinmux.direction= SYSTEM_PINMUX_PIN_DIR_OUTPUT;
	system_pinmux_pin_set_config(PIN_PA22H_GCLK_IO6, &config_pinmux);
}
void configure_spi_master(void)
{
	struct spi_config config_spi_master;
	struct spi_slave_inst_config slave_dev_config;
	/* Configure and initialize software device instance of peripheral slave */
	spi_slave_inst_get_config_defaults(&slave_dev_config);
	slave_dev_config.ss_pin = PIN_PA09;
	spi_attach_slave(&slave, &slave_dev_config);
	/* Configure, initialize and enable SERCOM SPI module */
	spi_get_config_defaults(&config_spi_master);
	config_spi_master.mux_setting = SPI_SIGNAL_MUX_SETTING_E;
	config_spi_master.transfer_mode = SPI_TRANSFER_MODE_0;
	config_spi_master.pinmux_pad0 = PINMUX_PA08D_SERCOM2_PAD0;
	config_spi_master.pinmux_pad1 = PINMUX_UNUSED;
	config_spi_master.pinmux_pad2 = PINMUX_PA10D_SERCOM2_PAD2;
	config_spi_master.pinmux_pad3 = PINMUX_PA11D_SERCOM2_PAD3;
	spi_init(&spi_master_instance, SERCOM2, &config_spi_master);
	spi_enable(&spi_master_instance);
}
void configure_tc(void)
{
	struct tc_config config_tc;
	tc_get_config_defaults(&config_tc);
	config_tc.counter_size = TC_COUNTER_SIZE_16BIT;
	config_tc.enable_capture_on_channel[0] = 1;
	config_tc.enable_capture_on_channel[1] = 1;
	config_tc.clock_source = GCLK_GENERATOR_3;
	tc_init(&capture_instance, TC4, &config_tc);
}
void capture_event_callback(void){
	uint64_t holder = 0;
	frequency = Captureclock/(TC4->COUNT16.CC[0].bit.CC*2); //grab frequency from chip
	time_stamp = rtc_count_get_count(&rtc_instance) + (rtc_counter*OFF_TIME) + rollover;
// 	if(measure_count == 0 & rtc_counter != 0){
// 		time_stamp = time_stamp - OFF_TIME;
// 	}
// 	time_stamp = time_stamp * 3.5;
	holder = 0x00000000FFFFFFFF& time_stamp;
	holder = holder<<32;
	holder = holder | frequency;
	if(!count_overflow && measure_count != 0){
		for(int i = 7; i >= 0; i--){
			wd_buffer[(8*counter)+i] =  0xFF & holder;
			holder = holder >> 8;
		}
		counter++;
	}
	if(count_overflow && measure_count != 0){
		for(int i = 7; i >= 0; i--){
			overflow_buffer[(8*(counter-32)+i)] =  0xFF & holder;
			holder = holder >> 8;
		}
		counter++;
	}
	if(counter > 31){
		count_overflow = true;
	}
	measure_count++;
	if(measure_count < 5){
		delay_ms(PULSE_TIME/5);
		extint_enable_events(&eic_events);
	}
	else{
		port_pin_set_output_level(SENSOR_PWR,0);
		
		extint_disable_events(&eic_events);
		//turn off chip on this line
		measure_count = 0;
		if(count_overflow){
			port_pin_set_output_level(FLASH_PWR,1);
			counter = counter - 31;
			send_data.data = wd_buffer;
			spi_write_page(send_data);
			
			spi_read_256(get_data);
			for(int i = 5; i<261; i++){
				out_buffer[i-5] = rd_buffer[i];
				if(out_buffer[i] == 0xFF && out_buffer[i+1] == 0xFF && out_buffer[i+2] == 0xFF){
					test = 121;
				}
			}
			usart_write_buffer_wait(&usart_terminal, out_buffer, sizeof(out_buffer));
			
			
			send_data.address = send_data.address + 256;
			get_data.address = get_data.address + 256;
			port_pin_set_output_level(FLASH_PWR,0);
		
			page_data[1] = send_data.address>>16 & 0x0000FF;
			page_data[2] = send_data.address>>8 & 0x0000FF;
			page_data[3] = send_data.address & 0x0000FF;
			page_data[4] = time_stamp >>24;
			page_data[5] = (time_stamp & 0x00FF0000) >> 16;
			page_data[6] = (time_stamp & 0x0000FF00) >> 8;
			page_data[7] = (time_stamp & 0x000000FF);
			eeprom_emulator_write_page(0, page_data);
			eeprom_emulator_commit_page_buffer();
// 			eeprom_emulator_read_page(0,page_data);
// 			usart_write_buffer_wait(&usart_terminal,page_data,sizeof(page_data));
			for(int i = 0; i <= counter*8; i++){
				wd_buffer[i] = overflow_buffer[i];
			}
			count_overflow = false;
		}
		//usart_write_buffer_wait(&usart_terminal, "test", sizeof("test"));
		sleepflag = true;
	}

}
void configure_rtc_count(void){
	struct rtc_count_config config_rtc_count;
	rtc_count_get_config_defaults(&config_rtc_count);
	config_rtc_count.prescaler           = RTC_COUNT_PRESCALER_DIV_1;
	config_rtc_count.mode                = RTC_COUNT_MODE_16BIT;
	config_rtc_count.continuously_update = true;
	rtc_count_init(&rtc_instance, RTC, &config_rtc_count);
	//rtc_count_enable(&rtc_instance);
}
void rtc_overflow_callback(void)
{
	rtc_counter++;
	port_pin_set_output_level(SENSOR_PWR,1);
	port_pin_set_output_level(STATUS_LED,1);
	extint_enable_events(&eic_events); //enable event controller and jump to capture callback.
	controlflag = true;
	timeout_timer = rtc_count_get_count(&rtc_instance);
	port_pin_set_output_level(STATUS_LED,0);
	
}
void configure_rtc_callbacks(void)
{
	rtc_count_register_callback(&rtc_instance, rtc_overflow_callback, RTC_COUNT_CALLBACK_OVERFLOW);
	rtc_count_enable_callback(&rtc_instance, RTC_COUNT_CALLBACK_OVERFLOW);
}
void configure_event_controller(void){
	// PPW: T captured in CC0, tp captured in CC1
	// f = 1/T, dutyCycle = tp / T
	events_tc.event_action = TC_EVENT_ACTION_PWP;
	// Enable the event action
	events_tc.on_event_perform_action = 1;
	tc_enable_events(&capture_instance, &events_tc);
	tc_register_callback(&capture_instance, capture_event_callback, TC_CALLBACK_CC_CHANNEL0);
	tc_enable_callback(&capture_instance, TC_CALLBACK_CC_CHANNEL0);

	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults(&config_extint_chan);

	config_extint_chan.gpio_pin           = FREQ_PIN;
	config_extint_chan.gpio_pin_mux       = FREQ_MUX;
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_DOWN;
	config_extint_chan.detection_criteria = EXTINT_DETECT_HIGH;
	config_extint_chan.wake_if_sleeping   = 0;
	extint_chan_set_config(2, &config_extint_chan); //change first parameter to number of EXTINT channel
	eic_events.generate_event_on_detect[2] = 1; //change [2] to number of EXTINT channel
	struct events_resource capture_event;
	struct events_config config_evt;
	events_get_config_defaults(&config_evt);
	config_evt.generator      = BOARD_PHOTODIODE_GEN;
	config_evt.edge_detect    = EVENTS_EDGE_DETECT_NONE;
	config_evt.path           = EVENTS_PATH_ASYNCHRONOUS;
	events_allocate(&capture_event, &config_evt);
	events_attach_user(&capture_event, EVSYS_ID_USER_TC4_EVU);
	//extint_enable_events(&eic_events);
	tc_enable(&capture_instance);
} 

int main (void)
{
	system_init();
	board_init();
	configure_eeprom();
	configure_bod();
	configure_terminal();
	configure_pinmux();
	configure_port_pins();
	configure_spi_master();
	delay_init();
	configure_tc();
    configure_event_controller();
	configure_rtc_count();
	configure_rtc_callbacks();
	rtc_count_set_period(&rtc_instance, OFF_TIME);
	sleepmgr_init();
	system_interrupt_enable_global();
	send_data.master = spi_master_instance;
	send_data.slave = slave;
	send_data.address = 0x000000;
	send_data.data = wd_buffer;
	get_data.master = spi_master_instance;
	get_data.slave = slave;
	get_data.address = 0x000000;
	get_data.data = rd_buffer;
	port_pin_set_output_level(FLASH_PWR,1);
	system_interrupt_enable_global();
	//usart_write_buffer_wait(&usart_terminal, "CHIP ERASED, STARTING PROGRAM\r\n", sizeof("CHIP ERASED, STARTING PROGRAM\r\n"));
	eeprom_emulator_read_page(0, page_data); //first time memory protection, page 0 is erase protect, page 1-3 is current addr, 4-7 is last time stamp.
	
	delay_s(10);
	if(page_data[0] == 255 && (READ_MODE != 1)) {
		//page_data[0] = 0;
		//page_data[1] = 0;
		//page_data[2] = 0;
		//page_data[3] = 0;
		//eeprom_emulator_write_page(0, page_data);
		//eeprom_emulator_commit_page_buffer();
		//spi_erase_chip(send_data);
		usart_write_buffer_wait(&usart_terminal, "CHIP ERASED, STARTING PROGRAM\r\n", sizeof("CHIP ERASED, STARTING PROGRAM\r\n"));
	}
	else{
		send_data.address = page_data[1] | send_data.address;
		send_data.address = send_data.address << 8;
		send_data.address = page_data[2] | send_data.address;
		send_data.address = send_data.address << 8;
		send_data.address = page_data[3] | send_data.address;
		get_data.address = send_data.address;
		rollover = (page_data[4] | rollover) << 8;
		rollover = (page_data[5] | rollover) << 8;
		rollover = (page_data[6] | rollover) << 8;
		rollover = (page_data[7] | rollover);
	}
	sleepmgr_lock_mode(SLEEPMGR_STANDBY);
	sleepmgr_lock_mode(SLEEPMGR_STANDBY);
	sleepmgr_unlock_mode(SLEEPMGR_STANDBY);
	delta = sleepmgr_get_sleep_mode();
	delta = sleepmgr_get_sleep_mode();
	port_pin_set_output_level(FLASH_PWR,0);
    rtc_count_enable(&rtc_instance); //enable rtc and sleepmgr to start sleep clock
	
    while(1){
// 		delay_s(1);
// 		page_data[0] = page_data[0] + 1;
// 		eeprom_emulator_write_page(0, page_data);
// 		eeprom_emulator_read_page(0, page_data);
// 		if(page_data[0] >= 4){
// 			usart_write_buffer_wait(&usart_terminal, "win", sizeof("win"));
// 		}


// 		if(controlflag){
// 			controlflag = false;
// 			extint_enable_events(&eic_events); //enable event controller and jump to capture callback.
// 		}
// 		
// 		if(sleepflag == true){
// 			sleepflag = false;
// 			sleepmgr_enter_sleep();
// 		}
// 
// 
// 		if(rtc_count_get_count(&rtc_instance) - timeout_timer > OFF_TIME){
// 			sleepflag = true;
// 			extint_disable_events(&eic_events);
// 			//port_pin_set_output_level(FLASH_PWR,0);
// 			port_pin_set_output_level(SENSOR_PWR,0);
// 		}


		port_pin_set_output_level(FLASH_PWR,1);
		spi_read_256(get_data);
		for(int i = 5; i<261; i++){
				out_buffer[i-5] = rd_buffer[i];
				if(out_buffer[i] == 0xFF && out_buffer[i+1] == 0xFF && out_buffer[i+2] == 0xFF){
					test = 121;
				}
		}
  	    usart_write_buffer_wait(&usart_terminal, out_buffer, sizeof(out_buffer));
	    get_data.address = get_data.address + 256;
		if(test == 121){
			test = 0;
		}
		
		
	}
}

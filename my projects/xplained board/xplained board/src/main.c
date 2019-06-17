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
#include <string.h>
struct usart_module usart_instance;
struct tc_module capture_instance;
struct extint_events eic_events;
struct tc_events events_tc;
#define MAX_RX_BUFFER_LENGTH   5
volatile uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];
//tc defines
#define PWM_MODULE      EXT1_PWM_MODULE
#define PWM_OUT_PIN     EXT1_PWM_0_PIN
#define PWM_OUT_MUX     EXT1_PWM_0_MUX
#define FREQ_PIN		PIN_PA18 //set to freq in pin
#define FREQ_MUX		MUX_PA18A_EIC_EXTINT2 //set to pins interrupt mask
#define BOARD_PHOTODIODE_GEN EVSYS_ID_GEN_EIC_EXTINT_2 //make this the same EXTINT mask
const int Captureclock = 47215500;
char newline[16] = "\r\n";
char clearterm[] = "\033[2J";
float Number = 520000; // number to convert
char Result[32]; // string which will contain the number
int test;
//pin config
void configure_port_pins(void)
{
	  struct system_pinmux_config config_pinmux;
    system_pinmux_get_config_defaults(&config_pinmux);
    config_pinmux.mux_position = PINMUX_PA17H_GCLK_IO3;
    config_pinmux.direction    = SYSTEM_PINMUX_PIN_DIR_OUTPUT;
    system_pinmux_pin_set_config(PIN_PA17H_GCLK_IO3, &config_pinmux);
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);

	config_port_pin.direction  = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(BUTTON_0_PIN, &config_port_pin);
	port_pin_set_config(PIN_PB00, &config_port_pin);

	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED_0_PIN, &config_port_pin);
}
//uart config
void usart_read_callback(struct usart_module *const usart_module)
{
	usart_write_buffer_job(&usart_instance,
	(uint8_t *)rx_buffer, MAX_RX_BUFFER_LENGTH);
}
void usart_write_callback(struct usart_module *const usart_module)
{
	port_pin_toggle_output_level(LED_0_PIN);
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
//interrupt config


//timer for freq measure
void configure_tc(void)
{
	struct tc_config config_tc;
	tc_get_config_defaults(&config_tc);
	config_tc.counter_size = TC_COUNTER_SIZE_32BIT;
	config_tc.enable_capture_on_channel[0] = 1;
	config_tc.enable_capture_on_channel[1] = 1;
	config_tc.clock_source = GCLK_GENERATOR_3;
	tc_init(&capture_instance, TC4, &config_tc);	
}

void capture_event_callback(void){
	Number = (Captureclock/(TC4->COUNT32.CC[0].bit.CC)) - (Number * 0.00248);
	sprintf ( Result, "%d", (int)Number ); // %d makes the result be a decimal integer
	strcat(Result,"\n\r");
	usart_write_buffer_wait(&usart_instance, clearterm, sizeof(clearterm));
	usart_write_buffer_wait(&usart_instance, Result, sizeof(Result));
}
void configure_event_controller(void){
	// PPW: T captured in CC0, tp captured in CC1
	// f = 1/T, dutyCycle = tp / T
	events_tc.event_action = TC_EVENT_ACTION_PPW;
	// Enable the event action
	events_tc.on_event_perform_action = 1;
	tc_enable_events(&capture_instance, &events_tc);
	tc_register_callback(&capture_instance, capture_event_callback, TC_CALLBACK_CC_CHANNEL0);
	tc_enable_callback(&capture_instance, TC_CALLBACK_CC_CHANNEL0);

	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults(&config_extint_chan);

	config_extint_chan.gpio_pin           = FREQ_PIN;
	config_extint_chan.gpio_pin_mux       = FREQ_MUX;
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_NONE;
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


// main
int main (void)
{
	
	
	system_init();
	delay_init();
	//board_init();
	configure_port_pins();
	configure_usart();
	configure_usart_callbacks();
	configure_tc();
	configure_event_controller();
	system_interrupt_enable_global();
	usart_write_buffer_wait(&usart_instance, "test", sizeof("test"));
	
	
	

	/* This skeleton code simply sets the LED to the state of the button. */
	while (1) {
		test = port_pin_get_output_level(BUTTON_0_PIN);
		if(port_pin_get_input_level(BUTTON_0_PIN) == 0){
			extint_enable_events(&eic_events);
		}
		else{
			extint_disable_events(&eic_events);
		}
		delay_s(1);
		usart_write_buffer_wait(&usart_instance, "test", sizeof("test"));
	}
	
}

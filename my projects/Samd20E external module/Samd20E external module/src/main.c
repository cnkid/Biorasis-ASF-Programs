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
#include <string.h>
#define PWM_MODULE      EXT1_PWM_MODULE
#define PWM_OUT_PIN     EXT1_PWM_0_PIN
#define PWM_OUT_MUX     EXT1_PWM_0_MUX
#define FREQ_PIN		PIN_PA01A_EIC_EXTINT1
#define FREQ_MUX		MUX_PA17A_EIC_EXTINT1
#define BOARD_PHOTODIODE_GEN EVSYS_ID_GEN_EIC_EXTINT_1
#define MAX_RX_BUFFER_LENGTH   3
struct usart_module usart_terminal;
struct usart_module usart_rf;
struct tc_module capture_instance;
const int Captureclock = 8000000;
char newline[] = "\r\n";
int Number = 520000; // number to convert
char Result[32]; // string which will contain the number
volatile uint8_t terminal_rx_buffer[MAX_RX_BUFFER_LENGTH];
volatile uint8_t terminal_tx_buffer[MAX_RX_BUFFER_LENGTH];
volatile uint8_t rf_rx_buffer[MAX_RX_BUFFER_LENGTH];
volatile uint8_t rf_tx_buffer[MAX_RX_BUFFER_LENGTH];


void configure_port_pins(void)
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);

	config_port_pin.direction  = PORT_PIN_DIR_OUTPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(PIN_PA25, &config_port_pin);
	port_pin_set_config(PIN_PA27, &config_port_pin);
	
	
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
	Number = Captureclock/(TC4->COUNT16.CC[0].bit.CC);
	sprintf ( Result, "%d", Number ); // %d makes the result be a decimal integer
	strcat(Result,"\n\r");
	usart_write_buffer_job(&usart_rf,Result, sizeof(Result));

}
void configure_event_controller(void){
	struct extint_events eic_events;
	struct tc_events events_tc;
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
	extint_chan_set_config(1, &config_extint_chan);
	eic_events.generate_event_on_detect[1] = 1;
	extint_enable_events(&eic_events);
	struct events_resource capture_event;
	struct events_config config_evt;
	events_get_config_defaults(&config_evt);
	config_evt.generator      = BOARD_PHOTODIODE_GEN;
	config_evt.edge_detect    = EVENTS_EDGE_DETECT_NONE;
	config_evt.path           = EVENTS_PATH_ASYNCHRONOUS;
	events_allocate(&capture_event, &config_evt);
	events_attach_user(&capture_event, EVSYS_ID_USER_TC4_EVU);
	tc_enable(&capture_instance);
}
void configure_usart(void)
{
	
	struct usart_config config_terminal;
	struct usart_config config_rf;
	usart_get_config_defaults(&config_terminal);
	usart_get_config_defaults(&config_rf);
	config_terminal.baudrate    = 115200;
	config_rf.baudrate    = 115200;
	config_terminal.mux_setting = USART_RX_3_TX_2_XCK_3;
	config_rf.mux_setting = USART_RX_1_TX_0_XCK_1;
	config_terminal.pinmux_pad0 = PINMUX_UNUSED;
	config_terminal.pinmux_pad1 = PINMUX_UNUSED;
	config_terminal.pinmux_pad2 = PINMUX_PA14C_SERCOM2_PAD2;
	config_terminal.pinmux_pad3 = PINMUX_PA15C_SERCOM2_PAD3;
	config_rf.pinmux_pad0 = PINMUX_PA04D_SERCOM0_PAD0;
	config_rf.pinmux_pad1 = PINMUX_PA05D_SERCOM0_PAD1;
	config_rf.pinmux_pad2 = PINMUX_UNUSED;
	config_rf.pinmux_pad3 = PINMUX_UNUSED;
	while (usart_init(&usart_terminal,
	SERCOM2, &config_terminal) != STATUS_OK) {
	}
	while (usart_init(&usart_rf,
	SERCOM0, &config_rf) != STATUS_OK) {
	}
	usart_enable(&usart_terminal);
	usart_enable(&usart_rf);
}
void usart_readrf_callback(struct usart_module *const usart_module)
{
	usart_write_buffer_job(&usart_terminal,(uint8_t *)rf_rx_buffer, MAX_RX_BUFFER_LENGTH);
}
void usart_writerf_callback(struct usart_module *const usart_module)
{
}
void usart_readterminal_callback(struct usart_module *const usart_module)
{
	usart_write_buffer_job(&usart_rf,(uint8_t *)terminal_rx_buffer, MAX_RX_BUFFER_LENGTH);
}
void usart_writeterminal_callback(struct usart_module *const usart_module)
{

}
void configure_usart_callbacks(void)
{
	usart_register_callback(&usart_rf, usart_readrf_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_register_callback(&usart_rf, usart_writerf_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_terminal, usart_readterminal_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_register_callback(&usart_rf, usart_writeterminal_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_rf, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_rf, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_terminal, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_terminal, USART_CALLBACK_BUFFER_TRANSMITTED);
}



int main (void)
{
	system_init();
	delay_init();
	configure_port_pins();
    //configure_tc();
	//configure_event_controller();
    configure_usart();
	configure_usart_callbacks();
	system_interrupt_enable_global();
	uint8_t string[] = "Hello World!\r\n";
	port_pin_set_output_level(PIN_PA25, true);
	while (true) {
		usart_read_buffer_job(&usart_rf, (uint8_t *)rf_rx_buffer, MAX_RX_BUFFER_LENGTH);
		usart_read_buffer_job(&usart_terminal, (uint8_t *)terminal_rx_buffer, MAX_RX_BUFFER_LENGTH);
	}
	
}

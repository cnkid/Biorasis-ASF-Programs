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
const int Captureclock = 8000000;
char newline[16] = "\r\n";
char clearterm[] = "\033[2J";
int Number = 520000; // number to convert
char Result[32]; // string which will contain the number
int test;
//pin config
void configure_port_pins(void)
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);

	config_port_pin.direction  = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_UP;

	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
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
	Number = Captureclock/(TC4->COUNT16.CC[1].bit.CC);
	sprintf ( Result, "%d", Number ); // %d makes the result be a decimal integer
	strcat(Result,"\n\r");
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
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;
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
	extint_enable_events(&eic_events);
	tc_enable(&capture_instance);
}


// main
int main (void)
{
	
	
	system_init();
	delay_init();
	//board_init();
	configure_port_pins();
	configure_tc();
	configure_event_controller();
	system_interrupt_enable_global();
	
	
	
	

	/* This skeleton code simply sets the LED to the state of the button. */
	while (1) {
	}
	
}

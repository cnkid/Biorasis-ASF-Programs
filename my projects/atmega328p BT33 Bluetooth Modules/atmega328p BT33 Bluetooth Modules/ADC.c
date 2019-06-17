/*
 * ADC.c
 *
 * Created: 11/19/2018 12:26:57 PM
 *  Author: Alex Valdes
 */ 
#include "ADC.h"
#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>

void ADC_init(void){
	//select reference to be AVcc and select channel 0 (internal ~5v)
	ADMUX = (1<<REFS0);
	//Enable ADC, and prescale by 64
	ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);
}
uint16_t ADC_Read(uint8_t ADC_Channel){
	//set admux to requested channel
	ADMUX = (1<<REFS0) | (0x0f & ADC_Channel);
	//start conversion and wait for it to finish
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC)){}
	//return adc value
	return ADCW;
	
}
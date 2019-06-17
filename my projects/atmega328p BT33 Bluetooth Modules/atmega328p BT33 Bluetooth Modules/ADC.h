/*
 * ADC.h
 *
 * Created: 11/19/2018 12:27:13 PM
 *  Author: Alex Valdes
 */ 
#include <stdio.h>
#define ADC_LR_CHAN 6
#define ADC_UD_CHAN 7

#ifndef ADC_H_
#define ADC_H_

void ADC_init(void);
uint16_t ADC_Read(uint8_t ADC_Channel);



#endif /* ADC_H_ */
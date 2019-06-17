#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include "lcd_lib.h"

void Backspace(void){
	LCDcursorLeft(1);
	LCDstring(" ", 1);
	LCDcursorLeft(1);
}

int main(void){
	uint8_t x = 0;
	uint8_t y = 0;
	uint8_t lock = 0;
	uint8_t reverse = 0;
	initialize_LCD();
	LCDstring("LCD INITIALIZED",sizeof("LCD INITIALIZED"));
	_delay_ms(2000);
	LcdCommandWrite(0x01);
	while(1){
		_delay_ms(10);
		if(!(PINB & (1 << PINB7)) & !lock){
			if(x==16){
				y = !y;
				x = 0;
				if(!y){
					reverse = !reverse;
				}
				else{
					LCDGotoXY(x,y);
				}
				if(reverse & y){
					LCDGotoXY(16,0);
				}
			}
			if(reverse){
				Backspace();
			}
			else{
				LCDstring("*",1);
			}
			lock = 1;
			x++;
		}
		else if (PINB & (1 << PINB7)){
			lock = 0;
		}
	}
}

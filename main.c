/*
 * main.c - Atmel main loop for LED matrix driving
 *
 * Copyright (c) 2015 Raspberry Pi Foundation
 *
 * Author: Serge Schneider <serge@raspberrypi.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *	* Neither the name of Raspberry Pi nor the
 *	  names of its contributors may be used to endorse or promote products
 *	  derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/twi.h>
#include <stdio.h>
#include <stdlib.h>

#define EE_WP			(1 << PB0)
#define KEYS_INT		(1 << PB6)
#define FRAME_INT		(1 << PB7)

#define LED_SDO			(1 << PC0)
#define LED_CLKR		(1 << PC1)
#define LED_LE			(1 << PC2)
#define LED_SDI			(1 << PC3)
#define LED_OE_N		(1 << PC7)

#define DIAMETER 28.
#define PI 3.1415926535
#define POLES 8.
#define TICKSPERSECOND 10.
#define INCHPERMETER 40.
#define MPS2KPH 3.6

#define set(port,x) port |= (x)
#define clr(port,x) port &= ~(x)

extern void draw_loop();
extern void clear_gain(void);
extern void delay(uint8_t ticks);
extern void write_data(uint32_t data, char type);

enum REG_ADDR {
	REG_ID = 192,
	REG_CFG_LOW,
	REG_CFG_HIGH,
};

typedef enum {
	DAT_LATCH = 22,
	CONF_WRITE = 20,
	CONF_READ  = 18,
	GAIN_WRITE = 16,
	GAIN_READ = 14,
	DET_OPEN = 13,
	DET_SHORT = 12,
	DET_OPEN_SHORT = 11,
	THERM_READ = 10,
} le_key;

volatile uint8_t pixels[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	0x00, 0x00, 0x00, 0x1f, 0x1f, 0x1f, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	0x00, 0x1f, 0x1f, 0x00, 0x1f, 0x1f, 0x1f, 0x00,
	0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00,
	0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00,

	0x1f, 0x1f, 0x1f, 0x1f, 0x00, 0x1f, 0x1f, 0x1f,
	0x1f, 0x00, 0x1f, 0x00, 0x00, 0x1f, 0x00, 0x1f,
	0x1f, 0x00, 0x1f, 0x00, 0x00, 0x1f, 0x00, 0x1f,

	0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00,
	0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00,
	0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00,

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

};

volatile char keys;
volatile char i2c_reg = 0xff;
volatile int ticksPerRound[]={0,0,0};

void clear(){
	int i;
	for(i=0;i<192;i++){
		pixels[i]=0x00;
	}
}

void setPixel(char x,char y){
	pixels[24*y+x+16]=0x1f;	
}

void printDigit(char digit,char offX,char offY){

	setPixel(offX+2,offY);
	setPixel(offX+2,offY+2);
	setPixel(offX+2,offY+4);

	switch(digit){
	case 8:
		setPixel(offX+0,offY+3);
	case 9:
		setPixel(offX+0,offY+1);
	case 3:
		setPixel(offX+0,offY+2);
		setPixel(offX+1,offY+2);
		setPixel(offX+0,offY+4);
		setPixel(offX+1,offY+4);
	case 7:
		setPixel(offX+0,offY);
		setPixel(offX+1,offY);
	case 1:
		setPixel(offX+2,offY+1);
		setPixel(offX+2,offY+3);
		break;
	case 6:
		setPixel(offX,offY+3);
	case 5:
		setPixel(offX,offY+1);
		setPixel(offX+2,offY+3);
	case 2:
		setPixel(offX+1,offY+2);
	case 0:
		setPixel(offX+1,offY);
		setPixel(offX,offY+4);
		setPixel(offX+1,offY+4);
	case 4:
		setPixel(offX,offY);
		setPixel(offX,offY+2);
	default:
		break;
	}

	switch(digit){
	case 0:
		setPixel(offX,offY+1);
		setPixel(offX+2,offY+3);
	case 2:
		setPixel(offX+2,offY+1);
		setPixel(offX,offY+3);
		break;
	case 4:
		setPixel(offX,offY+1);
		setPixel(offX+2,offY+1);
		setPixel(offX+1,offY+2);
		setPixel(offX+2,offY+3);
	default:
		break;
	}
}

void print(unsigned char number){
	char digit1=number/10;
	char digit2=number%10;
	if(digit1)printDigit(digit1,0,3);
	printDigit(digit2,4,3);
}

char calcSpeed(){
	float medTicks=(float)(ticksPerRound[0]+ticksPerRound[1]+ticksPerRound[2])/3.;
	float rps=1./(POLES*medTicks*TICKSPERSECOND);
	return PI*DIAMETER*MPS2KPH*rps;
}

int main(void)
{
	PORTA = 0;
	PORTB = 1;
	PORTC = 0;
	PORTD = 0xFF;
	DDRB = EE_WP | FRAME_INT | KEYS_INT;
	DDRC = LED_SDI | LED_CLKR | LED_LE | LED_OE_N;
	DDRD = 0xFF;

	TCCR0A = (1<<CS12);
	TWBR = 0xff;
	TWAR = 0x46 << 1;
	TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT) | (1 << TWIE);
	
	clear();
	print(90);
	//clear_gain();
	sei();
	draw_loop();
	for(;;);
}

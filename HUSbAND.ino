//-------------------------------------------------------------------------------
//
// HUSbAND 	  High freqUency StimulAtioN Device
// Copyright (c) 2015 Christian Garbers.
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the Simplified BSD License
// which accompanies this distribution
//
// Contributors:
//     Christian Garbers - initial API and implementation
//-------------------------------------------------------------------------------

//The setup function is called once at startup of the sketch

#include <Arduino.h>
#include "wiring_private.h"
//#include <math.h>

const int RED_PIN = 10;
const int GREEN_PIN = 5;
const int BLUE_PIN = 9;
const int STIM_TRIG_PIN = 4;
const int FREQ_PIN = 6;
const double CLOCK = 16000000;
const int LED_TRIG_PIN = 13;

volatile uint16_t *GREENR = &OCR3A;
volatile uint16_t *REDR = &OCR1B;
volatile uint16_t *BLUER = &OCR1A;

int RED = 100;
int GREEN = 100;
int BLUE = 100;
double INTENSITY = 1;
char BUFFER[10];
double FREQ=100;
char str_buf[10];
const double CARRIERFREQ = 16000000/256;
volatile double FREQCOUNTER = 0;
volatile double MAXFREQCOUNT = 50000;
unsigned char sreg;

uint8_t FREQ_BIT;
uint8_t FREQ_PORT;
volatile uint8_t *FREQ_OUT;

uint8_t LEDT_BIT;
uint8_t LEDT_PORT;
volatile uint8_t *LEDT_OUT;

ISR(TIMER1_COMPB_vect){
	  FREQCOUNTER++;
	  if(FREQCOUNTER>MAXFREQCOUNT){
		  *FREQ_OUT ^= FREQ_BIT;
		  *LEDT_OUT ^= LEDT_BIT;
		  FREQCOUNTER=0;
	  }
}
ISR(TIMER1_COMPA_vect){
	  FREQCOUNTER++;
	  if(FREQCOUNTER>MAXFREQCOUNT){
		  *FREQ_OUT ^= FREQ_BIT;
		  *LEDT_OUT ^= LEDT_BIT;
		  FREQCOUNTER=0;
	  }
}
ISR(TIMER3_COMPA_vect){
	  FREQCOUNTER++;
	  if(FREQCOUNTER>MAXFREQCOUNT){
		  *FREQ_OUT ^= FREQ_BIT;
		  *LEDT_OUT ^= LEDT_BIT;
		  FREQCOUNTER=0;
	  }
}

int executecommand(char *buffer){
  //Serial.write(buffer);
	switch(buffer[0]){
		case 'R':
			RED = atoi(++buffer);
			if(RED == 0){
				cbi(TCCR1A, COM1B1);
				Serial.write("Red 0");
			}
			else{
				sbi(TCCR1A, COM1B1);
			}
			if(RED>BLUE and RED>GREEN) {
				TIMSK1 = _BV(OCIE1B);
				TIMSK3 = 0 ;
			}
			*REDR = RED * INTENSITY;
			return 1;
		case 'G':
			GREEN = atoi(++buffer);
			if(GREEN == 0){
				Serial.write("Green 0");
				cbi(TCCR3A, COM3A1);
			}
			else{
				sbi(TCCR3A, COM3A1);
			}
			if(GREEN>BLUE> and GREEN>RED) {
				TIMSK3 = _BV(OCIE3A);
				TIMSK1 = 0 ;
			}
			*GREENR = GREEN * INTENSITY ;
			return 1;
		case 'B':
			BLUE = atoi(++buffer);
			if(BLUE == 0){
				Serial.write("Blue 0");
				cbi(TCCR1A, COM1A1);
			}
			else{
				sbi(TCCR1A, COM1A1);
			}
			if(BLUE>GREEN and BLUE>RED) {
				TIMSK1 = _BV(OCIE1A);
				TIMSK3 = 0 ;
			}
			*BLUER = BLUE * INTENSITY;
			return 1;
		case 'I':
			INTENSITY = atof(++buffer);
			*GREENR = GREEN * INTENSITY ;
			*REDR = RED * INTENSITY;
			*BLUER = BLUE * INTENSITY;
			digitalWrite(STIM_TRIG_PIN, INTENSITY>0);
			return 1;
		case 'F':
			FREQ = (double)atoi(++buffer);
			MAXFREQCOUNT = (CARRIERFREQ/FREQ)/2;
			dtostrf(MAXFREQCOUNT,5,1,str_buf);
			Serial.write(str_buf);
			return 1;
	}
	return 0;
}

int readline(int readch, char *buffer, int len)
{
  static int pos = 0;
  int rpos;
  if (readch > 0) {
    switch (readch) {
      case '\n': \
        break;
      case '\r':
        rpos = pos;
        pos = 0;
        return executecommand(buffer);
      default:
        if (pos < len-1) {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
    }
  }
  return -1;
}

void setup()
{
	  pinMode(RED_PIN, OUTPUT);
	  pinMode(GREEN_PIN, OUTPUT);
	  pinMode (BLUE_PIN, OUTPUT);

	  //Set counters 1 and 3 to Fast PWM 8 bit -> 62Khz and connect the OCR to pins
	  TCCR1A,TCCR1B,TCCR3A,TCCR3B  = 0;
	  TCCR1A = _BV (WGM10)|_BV(COM1A1)|_BV(COM1B1);
	  TCCR1B = _BV(CS10) | _BV(WGM12);
	  TCCR3A = _BV (WGM30)|_BV(COM3A1);
	  TCCR3B = _BV(CS30) | _BV(WGM32);
	  //Activate compare match Interupt for OCR1B (Red)
	  TIMSK1 = _BV(OCIE1B);

	  //Setup Counter4
	  pinMode(13, OUTPUT);
	  pinMode(6, OUTPUT);

	  sei();

	  Serial.begin(115200);

	  *GREENR = GREEN*INTENSITY ;
	  *REDR = RED*INTENSITY;
	  *BLUER = BLUE*INTENSITY;

	  FREQ_BIT = digitalPinToBitMask(FREQ_PIN);
	  FREQ_PORT = digitalPinToPort(FREQ_PIN);
	  FREQ_OUT = portOutputRegister(FREQ_PORT);

	  LEDT_BIT = digitalPinToBitMask(LED_TRIG_PIN);
	  LEDT_PORT = digitalPinToPort(LED_TRIG_PIN);
	  LEDT_OUT = portOutputRegister(LEDT_PORT);

	  }

void loop(){
	while(Serial.available()) readline(Serial.read(), BUFFER, 6);
	}

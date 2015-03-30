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
const int INT_PIN = 6;

uint16_t *GREENR = &OCR3C;
uint16_t *REDR = &OCR1B;
uint16_t *BLUER = &OCR1A;

int RED = 100;
int GREEN = 100;
int BLUE = 100;
double INTENSITY = 1;
char BUFFER[6];
double FREQ;
char time_buf[10];
double STEPSIZE = 1./(15000000./2048.);
int FREQ_COUNT = 255;
unsigned char sreg;

int executecommand(char *buffer){
  Serial.write(buffer);
	switch(buffer[0]){
		case 'R':
			RED = atoi(++buffer);
			*REDR = RED*INTENSITY;
			//OCR1B =  RED*INTENSITY;
			return 1;
		case 'G':
			*GREENR = GREEN*INTENSITY ;
			//OCR3C = GREEN*INTENSITY ;
			return 1;
		case 'B':
			BLUE = atoi(++buffer);
			*BLUER = BLUE*INTENSITY;
			//OCR1A = BLUE*INTENSITY;
			return 1;
		case 'I':
			INTENSITY = atof(++buffer);
			*GREENR = GREEN*INTENSITY ;
			*REDR = RED*INTENSITY;
			*BLUER = BLUE*INTENSITY;
			digitalWrite(STIM_TRIG_PIN, INTENSITY>0);
			return 1;
		case 'F':
			FREQ = (double)atoi(++buffer);
			FREQ_COUNT = (.5/FREQ)/STEPSIZE;
			sreg = SREG;
			cli();
			TC4H = FREQ_COUNT>>8;
			OCR4C = (unsigned char)FREQ_COUNT;
			SREG = sreg;
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

	  //Set counters 1 to Fast PWM 8 bit -> 64Khz pwm
	  TCCR1A,TCCR1B,TCCR3A,TCCR3B  = 0;
	  TCCR1A = _BV (WGM10)|_BV (WGM11)|_BV(COM1A1)|_BV(COM1B1);
	  TCCR1B = _BV(CS10) | _BV(WGM12);
	  TCCR3A = _BV (WGM30)|_BV (WGM31)|_BV(COM3C1);
	  TCCR3B = _BV(CS30) | _BV(WGM32);


	  //Setup Counter4
	  TCCR4A,TCCR4B,TCCR4C,TCCR4D = 0;
	  TCCR4C = _BV(COM4D0);//connect to output pin (arduino dp 6)
	  TCCR4A = _BV(COM4A0);//connect to output pin (arduino dp 13)
	  TCCR4B = _BV(CS43)|_BV(CS42);//set prescaler to 2048

	  //OCR4C determines how high the counter counts until reset
	  sreg = SREG;
	  cli();
	  TC4H = FREQ_COUNT>>8;
	  OCR4C = (unsigned char)FREQ_COUNT;
	  SREG = sreg
	  OCR4D = 0;
	  OCR4A = 0;

	  Serial.begin(19200);

	  *GREENR = GREEN*INTENSITY ;
	  *REDR = RED*INTENSITY;
	  *BLUER = BLUE*INTENSITY;
	  }

void loop(){
	while(Serial.available()) readline(Serial.read(), BUFFER, 5);
	}

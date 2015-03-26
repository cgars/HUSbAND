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

const int RED_PIN = 5;
const int GREEN_PIN = 9;
const int BLUE_PIN = 10;
const int INT_PIN = 11;
const int STIM_TRIG_PIN = 4;
const int LED_TRIG_PIN = 13;


int RED;
int GREEN;
int BLUE;
double INTENSITY = 1;
char BUFFER[5];
int DELAY_MS=100;
int DELAY_US=100;
double DELAY_MS_D;
double DELAY_US_D;
double FREQ;
char time_buf[10];

uint8_t LED_TRIG_PIN_TIMER;
uint8_t LED_TRIG_PIN_BIT;
uint8_t LED_TRIG_PIN_PORT;
volatile uint8_t *LED_TRIG_PIN_OUT;

int executecommand(char *buffer){
  Serial.write(buffer);
	switch(buffer[0]){
		case 'R':
			RED = atoi(++buffer);
			analogWrite(RED_PIN, RED*INTENSITY);
			return 1;
		case 'G':
			GREEN = atoi(++buffer);
			analogWrite(GREEN_PIN, GREEN*INTENSITY);
			return 1;
		case 'B':
			BLUE = atoi(++buffer);
			analogWrite(BLUE_PIN, BLUE*INTENSITY);
			return 1;
		case 'I':
			INTENSITY = atof(++buffer);
			analogWrite(GREEN_PIN, GREEN*INTENSITY);
			analogWrite(RED_PIN, RED*INTENSITY);
			analogWrite(BLUE_PIN, BLUE*INTENSITY);
			return 1;
		case 'F':
			FREQ = (double)atoi(++buffer);
			DELAY_US_D = modf(500/FREQ, &DELAY_MS_D)*1000;
			DELAY_US = DELAY_US_D;
			DELAY_MS = DELAY_MS_D;
			if(!DELAY_US)DELAY_US++;
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
	  pinMode(INT_PIN, OUTPUT);

	  //Set 16 Bit counters to Fast PWM 8 bit -> 64Khz pwm
	  TCCR1A,TCCR1B,TCCR3A,TCCR4B  = 0;
	  TCCR1A = _BV (WGM10) ;
	  TCCR1B = _BV(CS10) | _BV(WGM12) ;
	  TCCR3A = _BV (WGM10) ;
	  TCCR3B = _BV(CS10) | _BV(WGM12) ;


	  Serial.begin(9600);
	  }

void loop(){
	while(Serial.available()) readline(Serial.read(), BUFFER, 5);
	digitalWrite(INT_PIN, HIGH);
	digitalWrite(LED_TRIG_PIN, HIGH);
	delay(DELAY_MS);
	delayMicroseconds(DELAY_US);
	digitalWrite(INT_PIN, LOW);
	digitalWrite(LED_TRIG_PIN, LOW);
	delay(DELAY_MS);
	delayMicroseconds(DELAY_US);
	}

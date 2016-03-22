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

int RED = 255;
int GREEN = 120;
int BLUE = 71;
double INTENSITY = 1;
char BUFFER[10];
double FREQ=100;
char str_buf[10];
const double CARRIERFREQ = 16000000/256;
volatile double FREQCOUNTER = 0;
volatile double MAXFREQCOUNTS[2] = {1562, 31};//{31,62};
volatile int MAXFREQINDEX = 0;
volatile int MAXFREQCOUNT = 31;
volatile int STIM_COUNTER = 0;
volatile int F_ONE_MAX = 125000;
int ISI;
int NEWMAXFREQCOUNT;

unsigned char sreg;

uint8_t FREQ_BIT;
uint8_t FREQ_PORT;
volatile uint8_t *FREQ_OUT;

uint8_t LEDT_BIT;
uint8_t LEDT_PORT;
volatile uint8_t *LEDT_OUT;


/* Interrupt Service Routines
 * The ISR of the timer with the highest pwm duty cycle is executed
 * each time a compare match interrupt has been triggered by the responsible
 * Timer/Counter (see above for which T/C is which led). In case we increase
 * the FREQCOUNTER by one.
 *   If FREQCOUNTER is above THE MAXFRECCOUNT we need to toggle the output
 * (switch LEDs on or off as well as the trigger).
 *   After the toggle (and only then) we can also change the stimulation
 * frequency if a change is needed by updating the MAXFREQCOUNTER
 */
ISR(TIMER1_COMPB_vect){
	  FREQCOUNTER++;
	  STIM_COUNTER++;
	  if(FREQCOUNTER>MAXFREQCOUNT){
		  *FREQ_OUT ^= FREQ_BIT;
		  *LEDT_OUT ^= LEDT_BIT;
		  FREQCOUNTER=0;
		  if(STIM_COUNTER>F_ONE_MAX){
			  MAXFREQINDEX=1;
			  MAXFREQCOUNT = MAXFREQCOUNTS[MAXFREQINDEX];
			  STIM_COUNTER=0;
		  }
	  }
}
ISR(TIMER1_COMPA_vect){
	  FREQCOUNTER++;
	  STIM_COUNTER++;
	  if(FREQCOUNTER>MAXFREQCOUNT){
		  *FREQ_OUT ^= FREQ_BIT;
		  *LEDT_OUT ^= LEDT_BIT;
		  FREQCOUNTER=0;
		  if(STIM_COUNTER>F_ONE_MAX){
			  MAXFREQINDEX=1;
			  MAXFREQCOUNT = MAXFREQCOUNTS[MAXFREQINDEX];
			  STIM_COUNTER = 0;
		  }
	  }
}
ISR(TIMER3_COMPA_vect){
	  FREQCOUNTER++;
	  STIM_COUNTER++;
	  if(FREQCOUNTER>MAXFREQCOUNT){
		  *FREQ_OUT ^= FREQ_BIT;
		  *LEDT_OUT ^= LEDT_BIT;
		  FREQCOUNTER=0;
		  if(STIM_COUNTER>F_ONE_MAX){
			  MAXFREQINDEX=1;
			  MAXFREQCOUNT = MAXFREQCOUNTS[MAXFREQINDEX];
			  STIM_COUNTER = 0;
		  }
	  }
}


/**
 * This function changes register values according to the command given
 * in the buffer.
 * buffer[0] is expected to be either R,G,B,I,F followed by numerical values
 * that are to be written to the registers.
 */
int executecommand(char *buffer){
  //Serial.write(buffer);
	switch(buffer[0]){
		case 'R':
			RED = atoi(++buffer);
			if(RED == 0){
				cbi(TCCR1A, COM1B1);
				Serial.write("Red 0");
			}
			if(RED>BLUE and RED>GREEN) {
				/**
				* In case red is the maximum value this ensures that the
				* Interrupt Mask Register of the "RED" counter (TIMSK1) is
				* modified such that the Compare B Match interrupt is enabled.
				* This ensures that frequency changes to happen only after a
				* duty cycle is complete ensuring that frequency changes do not
				* interrupt duty cycles and thereby lead to a
				* hypothetical brightness changes.
				*/
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

			if(GREEN>BLUE and GREEN>RED) {
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
			return 1;
		case 'F':
			FREQ = (double)atoi(++buffer);
			NEWMAXFREQCOUNT = (CARRIERFREQ/FREQ)/2;
			digitalWrite(STIM_TRIG_PIN, !digitalRead(STIM_TRIG_PIN));
			//dtostrf(MAXFREQCOUNT,5,1,str_buf);
			//Serial.write(str_buf);
			MAXFREQCOUNTS[0] = FREQ ;
			return 1;
		case 'H':
			FREQ = (double)atoi(++buffer);
			NEWMAXFREQCOUNT = (CARRIERFREQ/FREQ)/2;
			digitalWrite(STIM_TRIG_PIN, !digitalRead(STIM_TRIG_PIN));
			//dtostrf(MAXFREQCOUNT,5,1,str_buf);
			//Serial.write(str_buf);
			MAXFREQCOUNTS[1] = FREQ ;
			return 1;
		case 'S':
			sbi(TCCR1A, COM1A1);
			sbi(TCCR3A, COM3A1);
			sbi(TCCR1A, COM1B1);
			STIM_COUNTER = 0;
			MAXFREQCOUNT = MAXFREQCOUNTS[0];
			MAXFREQINDEX = 0;
			return 1;
		case 'X':
			cbi(TCCR1A, COM1A1);
			cbi(TCCR3A, COM3A1);
			cbi(TCCR1A, COM1B1);
			return 1;
		case 'T':
			ISI = atoi(++buffer);
			F_ONE_MAX = ISI*(CARRIERFREQ/1000.);
	}
	return 0;
}

/*
 * Read in data from Serial char by char
 * \n is ignored
 * \r triggers execution
 * Other characters are inserted into the buffer
 */
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

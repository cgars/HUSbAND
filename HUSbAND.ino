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
//#include <math.h>

const int RED_PIN = 5;
const int GREEN_PIN = 9;
const int BLUE_PIN = 10;
const int INT_PIN = 6;
const int STIM_TRIG_PIN = 4;
const int LED_TRIG_PIN = 13;

int RED;
int GREEN;
int BLUE;
int INTENSITY;
char BUFFER[5];
double DELAY_MS=100;
double DELAY_US=100;
double FREQ;
char time_buf[10];

int executecommand(char *buffer){
  Serial.write(buffer);
	switch(buffer[0]){
		case 'R':
			RED = atoi(++buffer);
			analogWrite(RED_PIN, RED);
			return 1;
		case 'G':
			GREEN = atoi(++buffer);
			analogWrite(GREEN_PIN, GREEN);
			return 1;
		case 'B':
			BLUE = atoi(++buffer);
			analogWrite(BLUE_PIN, BLUE);
			return 1;
		case 'I':
			INTENSITY = atoi(++buffer);
			digitalWrite(STIM_TRIG_PIN, INTENSITY>0);
			analogWrite(INT_PIN, INTENSITY);
			return 1;
		case 'F':
			FREQ = atoi(++buffer);
			DELAY_MS = modf(FREQ, &DELAY_US);
			printf(time_buf,"ms:%d,us%d",DELAY_MS,DELAY_US);
			Serial.write(time_buf);
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

	  //Set 16 Bit counters to Fast PWM 8 bit -> 32Khz pwm
	  TCCR1A = _BV (WGM10) ;
	  TCCR1B = _BV(CS10) | _BV(WGM12) ;
	  TCCR3A = _BV (WGM10) ;
	  TCCR3B = _BV(CS10) | _BV(WGM12) ;

	  //Set 10 Bit counter to use no clock divisor
	  TCCR4B = _BV(CS40);

	  Serial.begin(9600);
	  }

// The loop function is called in an endless loop
void loop(){
	while(Serial.available()) readline(Serial.read(), BUFFER, 5);
	analogWrite(INT_PIN, INTENSITY);
	digitalWrite(LED_TRIG_PIN, HIGH);
	delay(DELAY_MS);
	delayMicroseconds(DELAY_US);
	analogWrite(INT_PIN, 0);
	digitalWrite(LED_TRIG_PIN, LOW);
	delay(DELAY_MS);
	delayMicroseconds(DELAY_US);
	}

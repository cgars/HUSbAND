[![Build Status](https://travis-ci.org/cgars/HUSbAND.svg?branch=master)](https://travis-ci.org/cgars/HUSbAND)

--
HUSbAND -  High freqUency StimulAtioN Device
-------------

Is a Ardoino micro based stimulation eyglass. It is used for high precission stimulation with light flickering at different frequencies.
This repsoitory contains (or will contain)
* The firmware for the arduino 
* The schemata for the eypieces and the controll board
* PCB Designs for the eypieces and the controll board

--
 Usage
-------------
The device can be used by wrtiting a command strings to the serial port emulated by the arduinos ftdi usb chip. A command consists of a upper case letter (R,G,B,I,F,H,S,X,T) and sometimes a number (R,G,B,I,F,H,T) that is either 8 bit unsigned integer (R,G,B,I) or 32 bit unsigned integer (F,H,T). Due to the way the device uses counters the Frequency the device produces will not match the frequency set exactly but it tries to come as close as possible.

Command Letter | Values | Function
------------ | -------------|-------------
R|0-1023|Red leds
G|0-1023|Green leds
B|0-1023|Blue leds
F|0-15000HZ|Frequency1
H|0-15000Hz|Frequency2
T|0-99999|Time in ms between onset of freq1 and freq2
S||Start the Device
X||Stop the Device



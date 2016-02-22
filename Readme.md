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
The device can be used by wrtiting a command strings to the serial port emulated by the arduinos ftdi usb chip. A command consists of a upper case letter (R,G,B,I,F) and a number that is either 8 bit unsigjned integer (R,G,B,I) or 32 bit unsigned integer (F). Due to the way the device uses counters the Frequency the device produces will not match the frequency set exactly but it tries to come as close as possible.

Command Letter | Values | Function
------------ | -------------|-------------
R|0-254|Red leds
G|0-254|Green leds
B|0-254|Blue leds
I|0-254|Overall intensity
F|2**32|Frequency



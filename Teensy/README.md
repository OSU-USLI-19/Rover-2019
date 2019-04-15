# 2019 OSU USLI Rover Teensy Controller

In order to run this code, you must install arduino along with teensyduino in order to support flashing to a teensy 3.6 micro controller.

Specific libraries must be downloaded as will be prompted when trying to flash the code such as:



Regexp.h
XBee.h (Xbee arduino library)
SD.h
SPI.h
Wire.h

In order to add these, you must install arduino (I use 1.8.8)
-Go to the sketch drop down, select include library. 
-Then click on manage libraries
-A window will pop up where you can search for the libraries. 
*Some libraries will not be named by their .h and some will not even need to be downloaded based on your arduino configuration.

teensythreads.h
https://github.com/ftrias/TeensyThreads 
TeensyThreads library is not automatically available using the sketch downloader. This link will explain how this library can be installed. 
This method is very similar, however, you must manually download this library

These will allow full functionality of the rover teensy code. 
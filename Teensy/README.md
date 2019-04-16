# 2019 OSU USLI Rover Teensy Controller

These instructions are meant for a windows computer.

In order to run this code, you must install arduino (I use 1.8.8, this is necessary for compatability) along with teensyduino in order to support 
flashing to a teensy 3.6 micro controller.

In order to install teensyduino, go to this link and set it up. There is a guide on the site.
https://www.pjrc.com/teensy/teensyduino.html

After you have teensydruino installed, you must go to tools and change the board to a teensy 3.6.

Specific libraries must be downloaded as will be prompted when trying to flash the code such as:

Regexp.h
SD.h
SPI.h
Wire.h

In order to add these, you must install arduino (I use 1.8.8)
-Go to the sketch drop down, select include library. 
-Then click on manage libraries
-A window will pop up where you can search for the libraries. 
*Some libraries will not be named by their .h and some will not even need to be downloaded based on your arduino configuration.


teensythreads.h

TeensyThreads library is not automatically available using the sketch downloader. I have included a .zip folder that contains the library called 
teensythreads-master. This file is to be added by going to sketch, then include libarary and add zip library option. After you have done this, things should 
be functional and able to compile. If you have any questions call 503-871-3841. I will be on my phone on friday.


These will allow full functionality of the rover teensy code. 

THE END
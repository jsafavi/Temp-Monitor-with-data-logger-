# Temp Monitor with Data Logger

This repo contains the necessary Arduino code to run a temperature monitoring project using Adafruit hardware and IoT interface.


# Hardware

This project uses hardware used mostly from Adafruit.com.

-Assembled Feather HUZZAH w/ ESP8266 WiFi With Stacking Headers
[https://www.adafruit.com/product/3213]

-Adalogger FeatherWing - RTC + SD Add-on For All Feather Boards
[https://www.adafruit.com/product/2922]

-MCP9808 High Accuracy I2C Temperature Sensor Breakout Board
[https://www.adafruit.com/product/1782]


<img src="https://github.com/jsafavi/Temp-Monitor-with-data-logger-/blob/readme-edit/unnamed.jpg" width="1000">


# Setup 

As it can be seen in the image above, the sensor is connected to default I2C pins of Feather board (pins 4 and 5). Power and ground are also provided to the sensor using 3.3v and GND pins of the board. 
The logger/RTC wing is mounted on top of the feather board, with a CR1220 battery for the RTC to work properly.
The temperature sensor can be seen to be inside of a plastic container as a protective measure.

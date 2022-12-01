# weather-station

Sensor and network code for a weather station. Hosted on atmega256rfr2 modules in combination with BitCloud HAL library. The IDE used was AtmelStudio 6.

The BitCloud HAL library and build information can be found here: https://microchipsupport.force.com/s/article/Environment-Setup-to-build-Bitcloud-HAL-libraries

The modules used were the Atmega256rfr2 by Microchip, and the sensor attached to that module was the Sensirion SPS30 to measure particulate matter.

The information gets forwarded to the Host module via a ZigBee network. The host module is connected to the Raspberry Pi via a USB-Bridge.

The file that needs to be uploaded on to the ZigBee module can be found under weather_station/src/app.c

DISCLAIMER: This code was written as part of a group project, the frontend and DB were not written by me. In order to run this, we used a Raspberry Pi 4, which hosted the code and connected to the radio modules. The output from the modules was converted to decimal on the Raspberry Pi, hence the transmitted HEX values in my code.

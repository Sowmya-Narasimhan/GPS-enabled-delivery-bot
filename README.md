# GPS Enabled Delivery Robot 

This project implements a WiFi controlled robot using an ESP32 and a GPS module. A mobile application built using MIT App Inventor communicates with the ESP32 to send movement commands and retrieve GPS location data.

## Features

* GPS based location tracking
* Mobile app control using MIT App Inventor
* WiFi communication between smartphone and ESP32
* Manual robot movement control
* UDP based command communication

## Hardware Used

* ESP32 development board
* GPS module (NEO-6M or similar)
* L298N motor driver
* DC motors
* Robot chassis
* Battery pack

## Software Used

* Arduino IDE
* MIT App Inventor

Required libraries:

```
WiFi.h
TinyGPS++
AsyncUDP
Wire.h
```

## Working

1. ESP32 creates a WiFi network.
2. The mobile application connects to the ESP32 network.
3. Commands are sent from the mobile app to the ESP32.
4. ESP32 processes the commands, reads GPS data, and controls the motors.



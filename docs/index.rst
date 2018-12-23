*********
RoverWing
*********

Introduction
============
RoverWing is a  shield (or "wing", following Adafruit's terminology) for Adafruit's `Feather boards <https://www.adafruit.com/feather>`_. 
This wing provides motor drivers, Inertial Motion Unit (IMU), and connection ports for servos, sonars, GPS, 
and other peripherals commonly used by mobile robots. It also contains a microcontroller preloaded with firmware 
to control these peripherals, which communicates with the Feather board using I2C protocol, thus freeing resources 
of the Feather board for other purposes. 

The RoverWing was heavily influenced by Adafruit's CRICKIT board (in particular, it has exact same dimensions and 
mounting holes as the CRICKIT board). However, unlike CRICKIT, it is intended for use with more powerful 12V motors 
and provides a slightly different set of peripherals. 

Below is the list of key features of the RoverWing:

* Power: it can be powered by 7-14V power source, and contains a voltage regulator providing power to the Feather board

* Only uses 2 pins (SDA and SCL) of the Feather board. 

* Contains on-board microcontroller, which takes care of low-level operations such as counting motor encoder pulses, using preloaded firmware

* Contains on-board 6DOF  Inertial Motion Unit (IMU), based on MPU6050 chip, which can be used for tracking robot orientation in space

* Provides the hardware and firmware support for connecting the following external peripherals

  - Motors: two brushed DC motors, at up to 14V@2.9A per motor
  - Quadrature encoders for each motor
  - Sonars: support for three HC-SR04 or compatible ultrasonic sensors (sonars)  
  - Servos: four servos (5V) (see note on power limit below)
  - Six analog inputs (3.3V)
  - Neopixel smart LED (see note on power limit below)
  - GPS and magnetometer (compass) sensors
  - two additional I2C sensors
  
  
Hardware
========

Below is the description of the RoverBoard hardware. 

 Power
 -----

The board can be powered by a 7-14V DC power supply such as 2 or 3 cell LiPO battery or  a 10-cell NiMH battery. 
The battery port uses JST VH male connector; see ??? for list of compatible cables and adapters. Note that there 
is no reverse  polarity protection, so double-check your connections! We recommend using 18 AWG or larger cable for main power connection. 

The board has a 5V high-efficiency  voltage regulator, which provides power to a plugged in Feather board via the USB bus pin of the Feather board. 
It also provides power to sonars, Neopixel LEDs, servos, and a 3.3V line regulator, which powers the built-in microcontroller and IMU. 

Note that 5V regulator is capable of producing 2.5A output. Some of it is used by on-board electronics, leaving about 2A  available for Neopixels and servos. 


Microcontroller
---------------
The brains of the board is the SAMD21G microcontroller - same MCU used by Arduino ZERO and Adafruit Feather M0 boards. It comes preloaded with firmware, which is described in Firmware section below. Normally there is no need to change it. 


The MCU communicates with the Feather board via I2C 

Inertial Motion Unit
--------------------



Motors and encoders
-------------------
The RoverWing provides connections for two brushed DC motors, at the same voltage as the main power supply (7-14V). Each motor is 
controlled by DRV8871 motor driver by Texas Instruments, which can provide up to 2.9A per motor. The drivers are current limited, 
so the current will not exceed 2.9A even if the motor is stalled, which helps prevent motor burnout. The motor ports use JST VH connectors; 
see ??? for list of compatible cables and adapters.

To avoid overheating the motor drivers, it is recommended to attach  additional heatsinks to them if you intend to run the motors at 
more than 2A continuous. 


In addition, the RoverWing provides two ports for connecting quadrature encoders, one for each motor. The encoder ports use JST PH4 connectors, and pinouts are shown below. These are the same ports and pinouts as used by REV Robotics hubs, so one can use the same encoder cables. 




Servos
------
A


Sonars
------

A
Analog inputs
-------------

Neopixel
--------

GPS and compass
---------------


Additional I2C ports
--------------------








Software
========

Add-ons
=======


License
=======



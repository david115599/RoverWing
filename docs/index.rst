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

Power
-----

Microcontroller
---------------

Inertial Motion Unit
--------------------

Motors and encoders
-------------------

Servos
------

Sonars
------

Analog inputs
-------------

Neopixel
--------

GPS and compass
---------------


Additional I2C ports
--------------------









Firmware
========

Add-ons
=======


License
=======



# Juicyware Overview
Juicyware is a fork of [Smoothieware](https://github.com/Smoothieware/Smoothieware) that is supports the extended functionalities of [Juicyboard](http://plugg.ee), which is a modular robotics hardware platform based on NXP's LPC1769 processor. Compiling Juicyware goes exactly like compiling Smoothieware.

**NOTE: you must use GNU ARM toolchain 4.8 2014q1-20140314 for best compatibility results.**

# Change Notes
These are feature deviations between Smoothieware and Juicyware

* Added R1000A module to execute `mod` commands
* Added R1001 module to execute R1001 motor driver specific commands
* Added R1001_I2C module to handle all I2C communications for R1XXX modules
* Included the ability to disable ISP push button to MRI mode through config, in case the user needs ISP pin (add tag and file link here)

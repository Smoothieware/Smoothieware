# Juicyware Overview
Juicyware is a fork of [Smoothieware](https://github.com/Smoothieware/Smoothieware) that is supports the extended functionalities of [Juicyboard](http://plugg.ee), which is a modular robotics hardware platform based on NXP's LPC1769 processor. Compiling Juicyware goes exactly like compiling Smoothieware.

**NOTE: you must use GNU ARM toolchain 4.8 2014q1-20140314 for best compatibility results.**

# Change Notes
These are feature deviations between Smoothieware and Juicyware

* `src/libs/Kernel.cpp`
  * Added `uart0_disable_checksum` to allow the user to disable UART0 functionality
  * Implemented low level UART0 disable
* `src/libs/SlowTicker.cpp` 
  * Eliminated the ability enter MRI mode using the ISP button, this frees up the pin for the user 
* `src/libs/SlowTicker.h`
  * Removed ispbtn variable;
* `src/main.cpp`
  * Added include and instantiation lines for R1000A and R1001 modules
  * Commented out CurrentControl() module, which is only applicable to smoothieboard hardware
* `src/modules/JuicyBoard/R1000A/R1000A.cpp`
  `src/modules/JuicyBoard/R1000A/R1000A.h`
  * Added R1000A module, includes required core functions for R1000A board and all modules
* `src/modules/JuicyBoard/R1000A_I2C/R1000A_I2C.cpp`
  `src/modules/JuicyBoard/R1000A_I2C/R1000A_I2C.h`
  * Added I2C class for all R1000A and module communications
* `src/modules/JuicyBoard/R1001/R1001.cpp`
  `src/modules/JuicyBoard/R1001/R1001.h`
  * Added R1001 module for all stepper motor extended functions
* `src/modules/robot/Robot.cpp`
  * For axes stepper motors: added the required code to decode step, enable and direction pins for a given slot number
* `src/modules/tools/extruder/Extruder.cpp`
  * For extruder stepper motors: added the required code to decode step, enable and direction pins for a given slot number
* `src/modules/tools/zprobe/DeltaCalibrationStrategy.cpp`
  * Copied c278507 from smoothieware edge
* `src/modules/tools/zprobe/ZProbe.cpp`
  * Copied d53d9df from smoothieware edge
* <del>`src/modules/utils/currentcontrol/CurrentControl.cpp`</del>
  <del>`src/modules/utils/currentcontrol/CurrentControl.h`</del>
  <del>`src/modules/utils/currentcontrol/DigipotBase.h`</del>
  <del>`src/modules/utils/currentcontrol/ad5206.h`</del>
  <del>`src/modules/utils/currentcontrol/mcp4451.h`</del>
  * Removed smoothieboard current control module files
* `src/modules/utils/simpleshell/SimpleShell.cpp`
  `src/modules/utils/simpleshell/SimpleShell.h`
  * Added `mod` command extention for Juicyboard and its modules
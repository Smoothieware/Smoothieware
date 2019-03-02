**NOTE**

Make sure you download the binary file and not the HTML page. 

Click the file you want, then click the View Raw button to get the raw binary file.

Flashing the HTML will mean the leds will not flash :)

the firmware-cnc.bin is a build for CNC. It has a different layout for the optional LCD more suited to CNC, this layout requires a graphic LCD. This build also excludes modules required for 3D printing like temperature control and extruders etc. It has only 3-axis compiled in. It also uses $H to home as G28 is used to park in real GCode.

The regular firmware.bin builds are for 3D printers and exclude CNC moudules such as spindle and drilling cycles. It has 5-axis built in so you can use two extuders out of the box.


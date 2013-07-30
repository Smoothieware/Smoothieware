#ifndef __RRDGLCD_H
#define __RRDGLCD_H

/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

/**
 * Based on st7920.h from http://mbed.org/users/Bas/code/ST7920/
 * Original License: Unknown
 * Modified from original to use SPI instead of parallel by Jim Morris
 * Removed all read commands as SPI does not support read
 */

#define VERSION 1.0

#include <mbed.h>
#include "libs/Kernel.h"
#include "libs/utils.h"
#include <libs/Pin.h>

// Instruction Set 1: (RE=0: Basic Instruction)
#define DISPLAY_CLEAR           0x01 // Fill DDRAM with "20H" and set DDRAM address counter (AC) to "00H"
#define RETURN_HOME             0x02 // Set DDRAM address counter (AC) to "00H", and put cursor
// to origin &#65533;Gthe content of DDRAM are not changed
#define ENTRY_MODE_SET          0x04 // Set cursor position and display shift when doing write or read
// operation
#define DISPLAY_CONTROL         0x08 // D=1: Display ON, C=1: Cursor ON, B=1: Character Blink ON
#define CURSOR_DISPLAY_CONTROL  0x10 // Cursor position and display shift control; the content of
// DDRAM are not changed
#define FUNCTION_SET            0x20 // DL=1 8-bit interface, DL=0 4-bit interface
// RE=1: extended instruction, RE=0: basic instruction
#define SET_CGRAM_ADDRESS       0x40 // Set CGRAM address to address counter (AC)
// Make sure that in extended instruction SR=0
#define SET_DDRAM_ADDRESS       0x80 // Set DDRAM address to address counter (AC)
// AC6 is fixed to 0

// Instruction set 2: (RE=1: extended instruction)
#define STANDBY                 0x01 // Enter standby mode, any other instruction can terminate.
// COM1&#65533;c32 are halted
#define SCROLL_OR_RAM_ADDR_SEL  0x02 // SR=1: enable vertical scroll position
// SR=0: enable CGRAM address (basic instruction)
#define REVERSE_BY_LINE         0x04 // Select 1 out of 4 line (in DDRAM) and decide whether to
// reverse the display by toggling this instruction
// R1,R0 initial value is 0,0
#define EXTENDED_FUNCTION_SET   0x20 // DL=1 :8-bit interface, DL=0 :4-bit interface
// RE=1: extended instruction, RE=0: basic instruction
#define SET_SCROLL_ADDRESS      0x40 // G=1 :graphic display ON, G=0 :graphic display OFF
#define SET_GRAPHIC_RAM_ADDRESS 0x80 // Set GDRAM address to address counter (AC)
// Set the vertical address first and followed the horizontal
// address by consecutive writings
// Vertical address range: AC5&#65533;cAC0, Horizontal address range: AC3&#65533;cAC0

// Parameters regarding Instruction Sets 1 & 2
#define DISPLAY_SHIFT_S         0x01 // Set 1, ENTRY_MODE_SET
#define INCREASE_DECREASE_ID    0x02 // Set 1, ENTRY_MODE_SET
#define CURSOR_BLINK_ON_B       0x01 // Set 1, DISPLAY_CONTROL
#define CURSOR_ON_C             0x02 // Set 1, DISPLAY_CONTROL
#define DISPLAY_ON_D            0x04 // Set 1, DISPLAY_CONTROL
#define SHIFT_RL                0x04 // Set 1, CURSOR_DISPLAY_CONTROL
#define CURSOR_SC               0x08 // Set 1, CURSOR_DISPLAY_CONTROL
#define EXTENDED_INSTRUCTION_RE 0x04 // Set 1, FUNCTION_SET; Set 2, EXTENDED_FUNTION_SET
#define DATA_LENGTH_DL          0x10 // Set 1, FUNCTION_SET; Set 2, EXTENDED_FUNTION_SET
#define REVERSE_BY_LINE_R0      0x01 // Set 2, REVERSE_BY_LINE
#define REVERSE_BY_LINE_R1      0x02 // Set 2, REVERSE_BY_LINE
#define EN_VERTICAL_SCROLL_SR   0x01 // Set 2, SCROLL_OR_RAM_ADDR_SEL
#define GRAPHIC_ON_G            0x02 // Set 2, EXTENDED_FUNTION_SET

/*********************************************************************************/

class RrdGlcd {
public:
    /**
    *@brief Constructor, initializes the lcd on the respective pins.
    *@param mosi mbed pinname for mosi
    *@param sclk mbed name for sclk
    *@param cd Smoothie Pin for cs
    *@return none
    *@  ----> pin PSB @ Gnd for Serial/SPI bus mode.
    */
    RrdGlcd (PinName mosi, PinName sclk, Pin cs);
    virtual ~RrdGlcd();

    void setFrequency(int f);

    /**
     *@brief Display initialization
     *
     *@param none
     *@return none
     *
     */
    void initDisplay(void);


    /**
     *@brief Enable Extended Instructions, RE=1, Graphic on
     *
     *@param none
     *@return none
     *
     */
    void setGraphicsMode(void);
    
    /**
     *@brief Go back to Basic Instructions, RE=0
     *
     *@param none
     *@return none
     *
     */
    void setTextMode(void);

    /**
    *@brief Sets DDRAM address counter (AC) to '00H'
    *@basic function, clear screen
    *@param none
    *@return none
    *
    */
    void clearScreen(void);
    
    void returnHome(void);
    void standby(void);

    /**
    *@brief Places a string on the screen with internal characters from the HCGROM
    *@
    *@param row, column, string
    *@return none
    *
    */
    void displayString(int Row,int Column, const char *ptr,int length);

    /**
    *@brief Places a character on the screen with an internal character from the HCGROM
    *@
    *@param row, column, character
    *@return none
    *
    */    
    void displayChar(int Row, int Column,int inpChr);
    
     /**
    *@brief Fills the screen with the graphics described in a 1024-byte array
    *@
    *@param bitmap 128x64, bytes horizontal
    *@return none
    *
    */
    void fillGDRAM(unsigned char *bitmap);
    
    //same as FILLGDRAM, but turnes all the bytes from vertical to horizontal
    //now pictures for eg KS0108 can be used
    void fillGDRAM_Turned(unsigned char *bitmap);
    
    /**
    *@brief Clears the graphics RAM of the screen
    *@writes all pixels to zero
    *@param none
    *@return none
    *
    */
    void clearGDRAM(void);
    
private:
    Pin cs;
    mbed::SPI* spi;

    /**
    *@brief Write instruction to the controller.
    *@param Command     command to send to the controller
    *@return none
    *
    */
    void  writeInstruction(unsigned int command);

    /**
     *@brief Write data byte to the controller.
     *@param data     data send to the controller chip (DDRAM/CGRAM/GDRAM)
     *@return none
     *
     */
    void writeRAM(unsigned int data);

};
#endif


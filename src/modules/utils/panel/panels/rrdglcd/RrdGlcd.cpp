#include "RrdGlcd.h"

RrdGlcd::RrdGlcd(PinName mosi, PinName sclk, Pin cs) {
    this->spi= new mbed::SPI(mosi, NC, sclk); 
     //chip select
    this->cs= cs;
    this->cs.set(0);

    initDisplay();
}

RrdGlcd::~RrdGlcd() {
    delete this->spi;
}

void RrdGlcd::setFrequency(int freq) {
       this->spi->frequency(freq);
}

void RrdGlcd::writeInstruction(unsigned int command) {
    unsigned int hn, ln;
    hn = command & 0xF0;
    ln = (command & 0x0F) << 4;
    
    this->cs.set(1); // chip select
    //delay_ns(60); // setup time

    this->spi->write(0xF8); // sync, rw=0, rs=0
    this->spi->write(hn); // hi nibble
    this->spi->write(ln); // lo nibble

    this->cs.set(0);

    wait_us(80);
}

void RrdGlcd::writeRAM(unsigned int data) {
   unsigned int hn, ln;
    hn = data & 0xF0;
    ln = (data & 0x0F) << 4;
    
    this->cs.set(1); // chip select
    //delay_ns(60); // setup time

    this->spi->write(0xFA); // sync, rw=0, rs=1
    this->spi->write(hn); // hi nibble
    this->spi->write(ln); // lo nibble

    this->cs.set(0);

    wait_us(80);
}

void RrdGlcd::initDisplay() {
    wait_ms(40); // wait 40ms
    writeInstruction(FUNCTION_SET | DATA_LENGTH_DL); // 8 bits interface, RE=0
    writeInstruction(DISPLAY_CONTROL | DISPLAY_ON_D ); // display on
    writeInstruction(DISPLAY_CLEAR); // clear display
    wait_ms(2); // wait 2ms for clear
    writeInstruction(ENTRY_MODE_SET | INCREASE_DECREASE_ID); // move cursor right
    writeInstruction(RETURN_HOME);
    setGraphicsMode();
}

//************************************************************************************************
//public methodes
void RrdGlcd::setGraphicsMode() {
    writeInstruction(EXTENDED_FUNCTION_SET | DATA_LENGTH_DL);
    writeInstruction(EXTENDED_FUNCTION_SET | DATA_LENGTH_DL | EXTENDED_INSTRUCTION_RE); //RE=1 (Extended funtion set)
    writeInstruction(EXTENDED_FUNCTION_SET | DATA_LENGTH_DL | EXTENDED_INSTRUCTION_RE | GRAPHIC_ON_G);
}

void RrdGlcd::setTextMode() {
    writeInstruction(FUNCTION_SET | DATA_LENGTH_DL); // RE=0 (Basic funtion set)
}

void RrdGlcd::clearScreen() {
    writeInstruction(FUNCTION_SET | DATA_LENGTH_DL); // RE=0 (Basic funtion set)
    writeInstruction(DISPLAY_CLEAR);
}

void RrdGlcd::returnHome() {
    writeInstruction(FUNCTION_SET | DATA_LENGTH_DL); //RE=0 (Basic funtion set)
    writeInstruction(RETURN_HOME);
}

void RrdGlcd::standby() {
    writeInstruction(EXTENDED_FUNCTION_SET | DATA_LENGTH_DL | EXTENDED_INSTRUCTION_RE); //RE=1 (Extended funtion set)
    writeInstruction(STANDBY);
}

//Basic text functions
void RrdGlcd::displayString(int Row,int Column,const char *ptr,int length) {
    int i=0;

    switch (Row) {
        case 0:
            Column|=0x80;
            break;
        case 1:
            Column|=0x90;
            break;
        case 2:
            Column|=0x88;
            break;
        case 3:
            Column|=0x98;
            break;
        default:
            Column=0x80;
            break;
    }

    if (Column%2!=0) {
        Column-=1;
        i=1;
    }
    writeInstruction((unsigned int)Column);

    if (i==1) {
        writeRAM(' ');
    }
    for (i=0; i<length; i++) {
        writeRAM((unsigned int)ptr[i]);
    }
}

void RrdGlcd::displayChar(int Row,int Column,int inpChr) {
    int i=0;

    switch (Row) {
        case 0:
            Column|=0x80; // SET_DDRAM_ADDRESS
            break;
        case 1:
            Column|=0x90;
            break;
        case 2:
            Column|=0x88;
            break;
        case 3:
            Column|=0x98;
            break;
        default:
            Column=0x80;
            break;
    }

    if (Column%2!=0) {
        Column-=1;
        i=1;
    }
    writeInstruction((unsigned int)Column);

    if (i==1) {
        writeRAM(' ');
    }
    writeRAM((unsigned int)inpChr);
}

// Graphic functions
void RrdGlcd::fillGDRAM(unsigned char *bitmap) {
    unsigned char i, j, k ;

    for ( i = 0 ; i < 2 ; i++ ) {
        for ( j = 0 ; j < 32 ; j++ ) {
            writeInstruction(SET_GRAPHIC_RAM_ADDRESS | j) ;
            if ( i == 0 ) {
                writeInstruction(SET_GRAPHIC_RAM_ADDRESS) ;
            } else {
                writeInstruction(SET_GRAPHIC_RAM_ADDRESS | 0x08) ;
            }
            for ( k = 0 ; k < 16 ; k++ ) {
                writeRAM( *bitmap++ ) ;
            }
        }
    }
}

void RrdGlcd::fillGDRAM_Turned(unsigned char *bitmap) {
    int i, j, k, m, offset_row, mask ;
    unsigned char data;

    for ( i = 0 ; i < 2 ; i++ ) { //upper and lower page
        for ( j = 0 ; j < 32 ; j++ ) { //32 lines per page
            writeInstruction(SET_GRAPHIC_RAM_ADDRESS | j) ;
            if ( i == 0 ) {
                writeInstruction(SET_GRAPHIC_RAM_ADDRESS) ;
            } else {
                writeInstruction(SET_GRAPHIC_RAM_ADDRESS | 0x08) ;
            }
            mask=1<<(j%8); // extract bitnumber
            //printf("mask: %d\r\n",mask);
            for ( k = 0 ; k < 16 ; k++ ) { //16 bytes per line
                offset_row=((i*32+j)/8)*128 + k*8; //y coordinate/8 = row 0-7 * 128 = byte offset, read 8 bytes
                data=0;
                for (m = 0 ; m < 8 ; m++) { // read 8 bytes from source

                    if ((bitmap[offset_row+m] & mask)) { //pixel = 1
                        data|=(128>>m);
                    }
                }
                writeRAM(data) ;
            }
        }
    }
}

void RrdGlcd::clearGDRAM() {
    unsigned char i, j, k ;

    for ( i = 0 ; i < 2 ; i++ ) {
        for ( j = 0 ; j < 32 ; j++ ) {
            writeInstruction(SET_GRAPHIC_RAM_ADDRESS | j) ;
            if ( i == 0 ) {
                writeInstruction(SET_GRAPHIC_RAM_ADDRESS) ;
            } else {
                writeInstruction(SET_GRAPHIC_RAM_ADDRESS | 0x08) ;
            }
            for ( k = 0 ; k < 16 ; k++ ) {
                writeRAM(0);
            }
        }
    }
}

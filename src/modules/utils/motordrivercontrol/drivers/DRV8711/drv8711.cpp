#include "drv8711.h"

#include "Kernel.h"
#include "StreamOutput.h"
#include "StreamOutputPool.h"

#define REGWRITE    0x00
#define REGREAD     0x80

DRV8711DRV::DRV8711DRV(std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi) : spi(spi)
{
}

void DRV8711DRV::init (unsigned int current, unsigned int microsteps, unsigned int gain)
{
    // derive torque and gain from current
    float c= current/1000.0F; // current in amps
    // I = (2.75 * Torque) / (256 * GAIN * Rsense) use (5,10,20,40) for gain, 0.05 is RSense Torque is 0-255
    float a= 256 * gain * resistor;
    // I = (2.75 * torque) / a; Ia = 2.75 * torque; a= 2.75*torque/I; a/torque= 2.75/I; torque/a = I/2.75; torque= I*a/2.75
    float t= (c*a) / 2.75F;
    while(t > 255) {
        // reduce gain
        gain= gain/2;
        if(gain < 5) {
            gain= 5;
            t= 255;
            break;
        }
        a= 256 * gain * resistor;
        t= (c*a) / 2.75F;
    }
    while(t < 1.0F) {
        // increase gain
        gain= gain*2;
        if(gain > 40) {
            gain= 40;
            t= 1;
            break;
        }
        a= 256 * gain * resistor;
        t= (c*a) / 2.75F;
    }
    uint32_t torque= t;

    // set the Initial default values
    // CTRL Register
    G_CTRL_REG.Address  = 0x00;
    G_CTRL_REG.DTIME    = 0x03;  //850ns
    switch (gain) {
        case 5:
            G_CTRL_REG.ISGAIN   = 0x0;  //Gain of 10
            break;
        case 10:
            G_CTRL_REG.ISGAIN   = 0x01;  //Gain of 10
            break;
        case 20:
            G_CTRL_REG.ISGAIN   = 0x02;  //Gain of 10
            break;
        case 40:
            G_CTRL_REG.ISGAIN   = 0x03;  //Gain of 10
            break;
    }

    G_CTRL_REG.EXSTALL  = 0x00;  //Internal Stall Detect

    switch (microsteps) {
        case 1:
            G_CTRL_REG.MODE     = 0x0;  // 1/8 Step   0x03=1/8, 0x08=1256, 0x05=1/32
            break;
        case 2:
            G_CTRL_REG.MODE     = 0x01;  // 1/8 Step   0x03=1/8, 0x08=1256, 0x05=1/32
            break;
        case 4:
            G_CTRL_REG.MODE     = 0x02;  // 1/8 Step   0x03=1/8, 0x08=1256, 0x05=1/32
            break;
        case 8:
            G_CTRL_REG.MODE     = 0x03;  // 1/8 Step   0x03=1/8, 0x08=1256, 0x05=1/32
            break;
        case 16:
            G_CTRL_REG.MODE     = 0x04;  // 1/8 Step   0x03=1/8, 0x08=1256, 0x05=1/32
            break;
        case 32:
            G_CTRL_REG.MODE     = 0x05;  // 1/8 Step   0x03=1/8, 0x08=1256, 0x05=1/32
            break;
        case 64:
            G_CTRL_REG.MODE     = 0x06;  // 1/8 Step   0x03=1/8, 0x08=1256, 0x05=1/32
            break;
        case 128:
            G_CTRL_REG.MODE     = 0x07;  // 1/8 Step   0x03=1/8, 0x08=1256, 0x05=1/32
            break;
        case 256:
            G_CTRL_REG.MODE     = 0x08;  // 1/8 Step   0x03=1/8, 0x08=1256, 0x05=1/32
            break;
    }


    //G_CTRL_REG.MODE   = 0x05;  // 1/8 Step   0x03=1/8, 0x08=1256, 0x05=1/32
    G_CTRL_REG.RSTEP    = 0x00;  //No Action
    G_CTRL_REG.RDIR     = 0x00;  //Direction set by DIR Pin
    G_CTRL_REG.ENBL     = 0x01;  //enable motor
    //1000 11 01 0 0011 0 0 1

    /// TORQUE Register
    G_TORQUE_REG.Address = 0x01;
    G_TORQUE_REG.SIMPLTH = 0x00;  //50uS Back EMF Sample Threshold
    G_TORQUE_REG.TORQUE  = torque;
    //1000 0 000  01000110

    // OFF Register
    G_OFF_REG.Address   = 0x02;
    G_OFF_REG.PWMMODE   = 0x00;  //Internal Indexer
    G_OFF_REG.TOFF      = 0x30;  //Default
    //1000 000 0 00110000

    // BLANK Register
    G_BLANK_REG.Address = 0x03;
    G_BLANK_REG.ABT     = 0x01;  //enable adaptive blanking time
    G_BLANK_REG.TBLANK  = 0x08;  //no idea what this should be but the
    //1000 000 1 00001000            //user guide shows it set to this

    // DECAY Register.
    G_DECAY_REG.Address = 0x04;
    G_DECAY_REG.DECMOD  = 0x03;  //mixed decay
    G_DECAY_REG.TDECAY  = 0x10;  //default
    //1000001100010000

    // STALL Register
    G_STALL_REG.Address = 0x05;
    G_STALL_REG.VDIV    = 0x03;  //Back EMF is divided by 4
    G_STALL_REG.SDCNT   = 0x03;  //stalln asserted after 8 steps
    G_STALL_REG.SDTHR   = 0x40;  //default
    //1000111101000000

    // DRIVE Register
    G_DRIVE_REG.Address = 0x06;
    G_DRIVE_REG.IDRIVEP = 0x00;  //High Side 50mA peak (source)
    G_DRIVE_REG.IDRIVEN = 0x00;  //Low Side 100mA peak (sink)
    G_DRIVE_REG.TDRIVEP = 0x01;  //High Side gate drive 500nS
    G_DRIVE_REG.TDRIVEN = 0x01;  //Low Side Gate Drive 500nS
    G_DRIVE_REG.OCPDEG = 0x01;  //OCP Deglitch Time 2uS
    G_DRIVE_REG.OCPTH = 0x01;  //OCP Threshold 500mV
    //1000000001010101

    // STATUS Register
    G_STATUS_REG.Address = 0x07;
    G_STATUS_REG.STDLAT  = 0x00;
    G_STATUS_REG.STD     = 0x00;
    G_STATUS_REG.UVLO    = 0x00;
    G_STATUS_REG.BPDF    = 0x00;
    G_STATUS_REG.APDF    = 0x00;
    G_STATUS_REG.BOCP    = 0x00;
    G_STATUS_REG.AOCP    = 0x00;
    G_STATUS_REG.OTS     = 0x00;

    WriteAllRegisters();
}


void DRV8711DRV::set_enable (bool enable)
{
    // Set Enable
    G_CTRL_REG.ENBL= enable ? 0x01 : 0x00;
    WriteAllRegisters();
}

void DRV8711DRV::dump_status(StreamOutput *stream)
{
    unsigned char dataHi = 0x00;
    const unsigned char dataLo = 0x00;
    unsigned int readData = 0x00;

    // Read CTRL Register
    dataHi = REGREAD | (G_CTRL_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);

    stream->printf("CTRL: %08X\n", readData);

    G_CTRL_REG.DTIME        = ((readData >> 10) & 0x0003);
    G_CTRL_REG.ISGAIN       = ((readData >> 8) & 0x0003);
    G_CTRL_REG.EXSTALL      = ((readData >> 7) & 0x0001);
    G_CTRL_REG.MODE         = ((readData >> 3) & 0x000F);
    G_CTRL_REG.RSTEP        = ((readData >> 2) & 0x0001);
    G_CTRL_REG.RDIR         = ((readData >> 1) & 0x0001);
    G_CTRL_REG.ENBL         = ((readData >> 0) & 0x0001);

    // Read TORQUE Register
    dataHi = REGREAD | (G_TORQUE_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);

    stream->printf("TORQUE: %08X\n", readData);

    G_TORQUE_REG.SIMPLTH    = ((readData >> 8) & 0x0007);
    G_TORQUE_REG.TORQUE     = ((readData >> 0) & 0x00FF);

    // Read OFF Register
    dataHi = REGREAD | (G_OFF_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);

    stream->printf("OFF: %08X\n", readData);

    G_OFF_REG.PWMMODE       = ((readData >> 8) & 0x0001);
    G_OFF_REG.TOFF          = ((readData >> 0) & 0x00FF);

    // Read BLANK Register
    dataHi = REGREAD | (G_BLANK_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);

    stream->printf("BLANK: %08X\n", readData);

    G_BLANK_REG.ABT         = ((readData >> 8) & 0x0001);
    G_BLANK_REG.TBLANK      = ((readData >> 0) & 0x00FF);

    // Read DECAY Register
    dataHi = REGREAD | (G_DECAY_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);

    stream->printf("DECAY: %08X\n", readData);

    G_DECAY_REG.DECMOD      = ((readData >> 8) & 0x0007);
    G_DECAY_REG.TDECAY      = ((readData >> 0) & 0x00FF);

    // Read STALL Register
    dataHi = REGREAD | (G_STALL_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);

    stream->printf("STALL: %08X\n", readData);

    G_STALL_REG.VDIV        = ((readData >> 10) & 0x0003);
    G_STALL_REG.SDCNT       = ((readData >> 8) & 0x0003);
    G_STALL_REG.SDTHR       = ((readData >> 0) & 0x00FF);

    // Read DRIVE Register
    dataHi = REGREAD | (G_DRIVE_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);

    stream->printf("DRIVE: %08X\n", readData);

    G_DRIVE_REG.IDRIVEP     = ((readData >> 10) & 0x0003);
    G_DRIVE_REG.IDRIVEN     = ((readData >> 8) & 0x0003);
    G_DRIVE_REG.TDRIVEP     = ((readData >> 6) & 0x0003);
    G_DRIVE_REG.TDRIVEN     = ((readData >> 4) & 0x0003);
    G_DRIVE_REG.OCPDEG      = ((readData >> 2) & 0x0003);
    G_DRIVE_REG.OCPTH       = ((readData >> 0) & 0x0003);

    // Read STATUS Register
    dataHi = REGREAD | (G_STATUS_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);

    stream->printf("STATUS: %08X\n", readData);

    G_STATUS_REG.STDLAT     = ((readData >> 7) & 0x0001);
    G_STATUS_REG.STD        = ((readData >> 6) & 0x0001);
    G_STATUS_REG.UVLO       = ((readData >> 5) & 0x0001);
    G_STATUS_REG.BPDF       = ((readData >> 4) & 0x0001);
    G_STATUS_REG.APDF       = ((readData >> 3) & 0x0001);
    G_STATUS_REG.BOCP       = ((readData >> 2) & 0x0001);
    G_STATUS_REG.AOCP       = ((readData >> 1) & 0x0001);
    G_STATUS_REG.OTS        = ((readData >> 0) & 0x0001);
}

void DRV8711DRV::WriteAllRegisters()
{
    unsigned char dataHi = 0x00;
    unsigned char dataLo = 0x00;

    // Write CTRL Register
    dataHi = REGWRITE | (G_CTRL_REG.Address << 4) | (G_CTRL_REG.DTIME << 2) | (G_CTRL_REG.ISGAIN);
    dataLo = (G_CTRL_REG.EXSTALL << 7) | (G_CTRL_REG.MODE << 3) | (G_CTRL_REG.RSTEP << 2) | (G_CTRL_REG.RDIR << 1) | (G_CTRL_REG.ENBL);
    SPI_DRV8711_ReadWrite(dataHi, dataLo);

    // Write TORQUE Register
    dataHi = REGWRITE | (G_TORQUE_REG.Address << 4) | (G_TORQUE_REG.SIMPLTH);
    dataLo = G_TORQUE_REG.TORQUE;
    SPI_DRV8711_ReadWrite(dataHi, dataLo);

    // Write OFF Register
    dataHi = REGWRITE | (G_OFF_REG.Address << 4) | (G_OFF_REG.PWMMODE);
    dataLo = G_OFF_REG.TOFF;
    SPI_DRV8711_ReadWrite(dataHi, dataLo);

    // Write BLANK Register
    dataHi = REGWRITE | (G_BLANK_REG.Address << 4) | (G_BLANK_REG.ABT);
    dataLo = G_BLANK_REG.TBLANK;
    SPI_DRV8711_ReadWrite(dataHi, dataLo);

    // Write DECAY Register
    dataHi = REGWRITE | (G_DECAY_REG.Address << 4) | (G_DECAY_REG.DECMOD);
    dataLo = G_DECAY_REG.TDECAY;
    SPI_DRV8711_ReadWrite(dataHi, dataLo);

    // Write STALL Register
    dataHi = REGWRITE | (G_STALL_REG.Address << 4) | (G_STALL_REG.VDIV << 2) | (G_STALL_REG.SDCNT);
    dataLo = G_STALL_REG.SDTHR;
    SPI_DRV8711_ReadWrite(dataHi, dataLo);

    // Write DRIVE Register
    dataHi = REGWRITE | (G_DRIVE_REG.Address << 4) | (G_DRIVE_REG.IDRIVEP << 2) | (G_DRIVE_REG.IDRIVEN);
    dataLo = (G_DRIVE_REG.TDRIVEP << 6) | (G_DRIVE_REG.TDRIVEN << 4) | (G_DRIVE_REG.OCPDEG << 2) | (G_DRIVE_REG.OCPTH);
    SPI_DRV8711_ReadWrite(dataHi, dataLo);

    // Write STATUS Register
    dataHi = REGWRITE | (G_STATUS_REG.Address << 4);
    dataLo = (G_STATUS_REG.STDLAT << 7) | (G_STATUS_REG.STD << 6) | (G_STATUS_REG.UVLO << 5) | (G_STATUS_REG.BPDF << 4) | (G_STATUS_REG.APDF << 3) | (G_STATUS_REG.BOCP << 2) | (G_STATUS_REG.AOCP << 1) | (G_STATUS_REG.OTS);
    SPI_DRV8711_ReadWrite(dataHi, dataLo);
}

unsigned int DRV8711DRV::SPI_DRV8711_ReadWrite(unsigned char dataHi, unsigned char dataLo)
{

    uint8_t buf[2]{dataHi, dataLo};
    uint8_t rbuf[2];

    spi(buf, 2, rbuf);

    unsigned int readData= (rbuf[0]<<8) | rbuf[1];
    return readData;
}

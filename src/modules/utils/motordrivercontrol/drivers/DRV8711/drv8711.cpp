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
    float c = current / 1000.0F; // current in amps
    // I = (2.75 * Torque) / (256 * GAIN * Rsense) use (5,10,20,40) for gain, 0.05 is RSense Torque is 0-255
    // torque= (256*gain*resistor*I)/2.75
    float a = 256.0F * gain * resistor;
    float t = (c * a) / 2.75F;
    while(t > 255) {
        // reduce gain
        gain = gain / 2;
        if(gain < 5) {
            gain = 5;
            t = 255;
            break;
        }
        a = 256.0F * gain * resistor;
        t = (c * a) / 2.75F;
    }
    while(t < 1.0F) {
        // increase gain
        gain = gain * 2;
        if(gain > 40) {
            gain = 40;
            t = 1;
            break;
        }
        a = 256.0F * gain * resistor;
        t = (c * a) / 2.75F;
    }
    uint32_t torque = t;
    THEKERNEL->streams->printf("for current of %fA, torque= %lu, gain= %u\n", c, torque, gain);
    // for current of 1.500000A, torque= 139, gain= 20

    // set the Initial default values
    // CTRL Register
    G_CTRL_REG.Address  = 0x00;
    G_CTRL_REG.DTIME    = 0x03;  //850ns
    switch (gain) {
        case 5:
            G_CTRL_REG.ISGAIN   = 0x0;  //Gain of 5
            break;
        case 10:
            G_CTRL_REG.ISGAIN   = 0x01;  //Gain of 10
            break;
        case 20:
            G_CTRL_REG.ISGAIN   = 0x02;  //Gain of 20
            break;
        case 40:
            G_CTRL_REG.ISGAIN   = 0x03;  //Gain of 40
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
    G_CTRL_REG.ENBL     = 0x00;  //enable motor, start disabled
    //1000 11 01 0 0011 0 0 1

    /// TORQUE Register
    G_TORQUE_REG.Address = 0x01;
    G_TORQUE_REG.SIMPLTH = 0x01;  //100uS Back EMF Sample Threshold
    G_TORQUE_REG.TORQUE  = torque;
    G_TORQUE_REG.Reserved= 0;
    //1000 0 000  01000110

    // OFF Register
    G_OFF_REG.Address   = 0x02;
    G_OFF_REG.PWMMODE   = 0x00;  //Internal Indexer
    G_OFF_REG.TOFF      = 0x32;  //Default
    G_OFF_REG.Reserved= 0;
    //1000 000 0 00110000

    // BLANK Register
    G_BLANK_REG.Address = 0x03;
    G_BLANK_REG.ABT     = 0x01;  //enable adaptive blanking time
    G_BLANK_REG.TBLANK  = 0x00;  //no idea what this should be but the
    G_BLANK_REG.Reserved= 0;
    //1000 000 1 00001000            //user guide shows it set to this

    // DECAY Register.
    G_DECAY_REG.Address = 0x04;
    G_DECAY_REG.DECMOD  = 0x05;  // auto mixed decay
    G_DECAY_REG.TDECAY  = 0x10;  //default
    G_DECAY_REG.Reserved= 0;
    //1000001100010000

    // STALL Register
    G_STALL_REG.Address = 0x05;
    G_STALL_REG.VDIV    = 0x02;  //Back EMF is divided by 8
    G_STALL_REG.SDCNT   = 0x01;  //stalln asserted after 2 steps
    G_STALL_REG.SDTHR   = 0x02;  //recommended
    //1000111101000000

    // DRIVE Register
    G_DRIVE_REG.Address = 0x06;
    G_DRIVE_REG.IDRIVEP = 0x00;  //High Side 50mA peak (source)
    G_DRIVE_REG.IDRIVEN = 0x00;  //Low Side 100mA peak (sink)
    G_DRIVE_REG.TDRIVEP = 0x00;  //High Side gate drive 500nS
    G_DRIVE_REG.TDRIVEN = 0x00;  //Low Side Gate Drive 500nS
    G_DRIVE_REG.OCPDEG =  0x00;  //OCP Deglitch Time 2uS
    G_DRIVE_REG.OCPTH =   0x00;  //OCP Threshold 500mV
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
    G_STATUS_REG.Reserved= 0;
    WriteAllRegisters();
}


void DRV8711DRV::set_enable (bool enable)
{
    // Set Enable
    G_CTRL_REG.ENBL = enable ? 0x01 : 0x00;
    WriteAllRegisters();
}

void DRV8711DRV::dump_status(StreamOutput *stream)
{
    unsigned char dataHi = 0x00;
    const unsigned char dataLo = 0x00;
    unsigned int readData = 0x00;

    CTRL_Register_t    R_CTRL_REG;
    TORQUE_Register_t  R_TORQUE_REG;
    OFF_Register_t     R_OFF_REG;
    BLANK_Register_t   R_BLANK_REG;
    DECAY_Register_t   R_DECAY_REG;
    STALL_Register_t   R_STALL_REG;
    DRIVE_Register_t   R_DRIVE_REG;
    STATUS_Register_t  R_STATUS_REG;

    // Read CTRL Register
    dataHi = REGREAD | (G_CTRL_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);

    stream->printf("CTRL: %04X (%04X)\n", readData & 0x0FFF, G_CTRL_REG.raw & 0x0FFF);
    R_CTRL_REG.raw = readData;

    stream->printf("DTIME: %u, ISGAIN: %u, EXSTALL: %u, MODE: %u, RSTEP: %u, RDIR: %u, ENBL: %u\n",
                   R_CTRL_REG.DTIME, R_CTRL_REG.ISGAIN, R_CTRL_REG.EXSTALL, R_CTRL_REG.MODE, R_CTRL_REG.RSTEP, R_CTRL_REG.RDIR, R_CTRL_REG.ENBL);

    // Read TORQUE Register
    dataHi = REGREAD | (G_TORQUE_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);

    stream->printf("TORQUE: %04X (%04X)\n", readData & 0x0FFF, G_TORQUE_REG.raw & 0x0FFF);
    R_TORQUE_REG.raw = readData;

    stream->printf("SIMPLTH: %u, TORQUE: %u\n",  R_TORQUE_REG.SIMPLTH, R_TORQUE_REG.TORQUE);
    int gain = R_CTRL_REG.ISGAIN == 0 ? 5 : R_CTRL_REG.ISGAIN == 1 ? 10 : R_CTRL_REG.ISGAIN == 2 ? 20 : R_CTRL_REG.ISGAIN == 3 ? 40 : 0;
    stream->printf(" Current: %f\n", (2.75F * R_TORQUE_REG.TORQUE) / (256.0F * gain * resistor));

    // Read OFF Register
    dataHi = REGREAD | (G_OFF_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);

    stream->printf("OFF: %04X (%04X)\n", readData & 0x0FFF, G_OFF_REG.raw & 0x0FFF);
    R_OFF_REG.raw = readData;

    stream->printf("PWMMODE: %u, TOFF: %u\n", R_OFF_REG.PWMMODE, R_OFF_REG.TOFF);

    // Read BLANK Register
    dataHi = REGREAD | (G_BLANK_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);

    stream->printf("BLANK: %04X (%04X)\n", readData & 0x0FFF, G_BLANK_REG.raw & 0x0FFF);
    R_BLANK_REG.raw = readData;

    stream->printf("ABT: %u, TBLANK: %u\n", R_BLANK_REG.ABT, R_BLANK_REG.TBLANK);

    // Read DECAY Register
    dataHi = REGREAD | (G_DECAY_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);

    stream->printf("DECAY: %04X (%04X)\n", readData & 0x0FFF, G_DECAY_REG.raw & 0x0FFF);
    R_DECAY_REG.raw = readData;

    stream->printf("DECMOD: %u, TDECAY: %u\n", R_DECAY_REG.DECMOD, R_DECAY_REG.TDECAY);

    // Read STALL Register
    dataHi = REGREAD | (G_STALL_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);

    stream->printf("STALL: %04X (%04X)\n", readData & 0x0FFF, G_STALL_REG.raw & 0x0FFF);
    R_STALL_REG.raw = readData;

    stream->printf("VDIV: %u, SDCNT: %u, SDTHR: %u\n", R_STALL_REG.VDIV, R_STALL_REG.SDCNT, R_STALL_REG.SDTHR);

    // Read DRIVE Register
    dataHi = REGREAD | (G_DRIVE_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);

    stream->printf("DRIVE: %04X (%04X)\n", readData & 0x0FFF, G_DRIVE_REG.raw & 0x0FFF);
    R_DRIVE_REG.raw = readData;

    stream->printf("IDRIVEP: %u, IDRIVEN: %u, TDRIVEP: %u, TDRIVEN: %u, OCPDEG: %u, OCPTH: %u\n",
                   R_DRIVE_REG.IDRIVEP, R_DRIVE_REG.IDRIVEN, R_DRIVE_REG.TDRIVEP, R_DRIVE_REG.TDRIVEN, R_DRIVE_REG.OCPDEG, R_DRIVE_REG.OCPTH);

    // Read STATUS Register
    dataHi = REGREAD | (G_STATUS_REG.Address << 4);
    readData = SPI_DRV8711_ReadWrite(dataHi, dataLo);

    stream->printf("STATUS: %02X\n", readData & 0x00FF);
    R_STATUS_REG.raw = readData;

    stream->printf("STDLAT: %u, STD: %u, UVLO: %u, BPDF: %u, APDF: %u, BOCP: %u, AOCP: %u, OTS: %u\n",
                   R_STATUS_REG.STDLAT, R_STATUS_REG.STD, R_STATUS_REG.UVLO, R_STATUS_REG.BPDF, R_STATUS_REG.APDF, R_STATUS_REG.BOCP, R_STATUS_REG.AOCP, R_STATUS_REG.OTS);

    stream->printf(" motor_driver_control.xxx.reg %03X,%03X,%03X,%03X,%03X,%03X,%03X\n",
                   G_CTRL_REG.raw & 0x0FFF, G_TORQUE_REG.raw & 0x0FFF, G_OFF_REG.raw & 0x0FFF, G_BLANK_REG.raw & 0x0FFF, G_DECAY_REG.raw & 0x0FFF, G_STALL_REG.raw & 0x0FFF, G_DRIVE_REG.raw & 0x0FFF);
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

// sets a raw register to the value specified, for advanced settings
// register 255 writes them, 0 displays what registers are mapped to what
bool DRV8711DRV::setRawRegister(StreamOutput *stream, uint32_t reg, uint32_t val)
{
    switch(reg) {
        case 255:
            WriteAllRegisters();
            stream->printf("Registers written\n");
            break;

        case 1: G_CTRL_REG.raw &= 0xF000; G_CTRL_REG.raw |= (val&0x0FFF); break;
        case 2: G_TORQUE_REG.raw &= 0xF000; G_TORQUE_REG.raw |= (val&0x0FFF); break;
        case 3: G_OFF_REG.raw &= 0xF000; G_OFF_REG.raw |= (val&0x0FFF); break;
        case 4: G_BLANK_REG.raw &= 0xF000; G_BLANK_REG.raw |= (val&0x0FFF); break;
        case 5: G_DECAY_REG.raw &= 0xF000; G_DECAY_REG.raw |= (val&0x0FFF); break;
        case 6: G_STALL_REG.raw &= 0xF000; G_STALL_REG.raw |= (val&0x0FFF); break;
        case 7: G_DRIVE_REG.raw &= 0xF000; G_DRIVE_REG.raw |= (val&0x0FFF); break;

        default:
            stream->printf("1: CTRL Register\n");
            stream->printf("2: TORQUE Register\n");
            stream->printf("3: OFF Register\n");
            stream->printf("4: BLANK Register\n");
            stream->printf("5: DECAY Register\n");
            stream->printf("6: STALL Register\n");
            stream->printf("7: DRIVE Register\n");
            stream->printf("255: write registers to chip\n");
            break;
    }
    return true;
}

unsigned int DRV8711DRV::SPI_DRV8711_ReadWrite(unsigned char dataHi, unsigned char dataLo)
{
    uint8_t buf[2] {dataHi, dataLo};
    uint8_t rbuf[2];

    spi(buf, 2, rbuf);
    //THEKERNEL->streams->printf("sent: %02X, %02X received:%02X, %02X\n", buf[0], buf[1], rbuf[0], rbuf[1]);
    unsigned int readData = (rbuf[0] << 8) | rbuf[1];
    return readData;
}

#include "drv8711.h"

#include "Kernel.h"
#include "StreamOutput.h"
#include "StreamOutputPool.h"

#define REGWRITE    0x00
#define REGREAD     0x80

DRV8711DRV::DRV8711DRV(std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi) : spi(spi)
{
    error_reported.reset();
}

void DRV8711DRV::init (uint16_t gain)
{
    this->gain= gain;

    // initialize the in memory mirror of the registers

    // CTRL Register
    G_CTRL_REG.raw      = 0x0000;
    G_CTRL_REG.Address  = 0x00;
    G_CTRL_REG.DTIME    = 0x03;  //850ns
    G_CTRL_REG.EXSTALL  = 0x00;  //Internal Stall Detect
    G_CTRL_REG.ISGAIN   = 0x02;  //Gain of 20
    G_CTRL_REG.MODE     = 0x04;  // also set by set_microstep()
    G_CTRL_REG.RSTEP    = 0x00;  //No Action
    G_CTRL_REG.RDIR     = 0x00;  //Direction set by DIR Pin
    G_CTRL_REG.ENBL     = 0x00;  //enable motor, start disabled

    /// TORQUE Register
    G_TORQUE_REG.raw     = 0x0000;
    G_TORQUE_REG.Address = 0x01;
    G_TORQUE_REG.SIMPLTH = 0x01;  //100uS Back EMF Sample Threshold
    G_TORQUE_REG.TORQUE  = 0x01; // low default set by set_current()

    // OFF Register
    G_OFF_REG.raw       = 0x0000;
    G_OFF_REG.Address   = 0x02;
    G_OFF_REG.PWMMODE   = 0x00;  //Internal Indexer
    G_OFF_REG.TOFF      = 0x32;  //Default

    // BLANK Register
    G_BLANK_REG.raw     = 0x0000;
    G_BLANK_REG.Address = 0x03;
    G_BLANK_REG.ABT     = 0x01;  //enable adaptive blanking time
    G_BLANK_REG.TBLANK  = 0x00;  //no idea what this should be but the

    // DECAY Register.
    G_DECAY_REG.raw     = 0x0000;
    G_DECAY_REG.Address = 0x04;
    G_DECAY_REG.DECMOD  = 0x05;  // auto mixed decay
    G_DECAY_REG.TDECAY  = 0x10;  //default

    // STALL Register
    G_STALL_REG.raw     = 0x0000;
    G_STALL_REG.Address = 0x05;
    G_STALL_REG.VDIV    = 0x02;  //Back EMF is divided by 8
    G_STALL_REG.SDCNT   = 0x01;  //stalln asserted after 2 steps
    G_STALL_REG.SDTHR   = 0x02;  //recommended

    // DRIVE Register
    G_DRIVE_REG.raw     = 0x0000;
    G_DRIVE_REG.Address = 0x06;
    G_DRIVE_REG.IDRIVEP = 0x00;  //High Side 50mA peak (source)
    G_DRIVE_REG.IDRIVEN = 0x00;  //Low Side 100mA peak (sink)
    G_DRIVE_REG.TDRIVEP = 0x00;  //High Side gate drive 500nS
    G_DRIVE_REG.TDRIVEN = 0x00;  //Low Side Gate Drive 500nS
    G_DRIVE_REG.OCPDEG =  0x00;  //OCP Deglitch Time 2uS
    G_DRIVE_REG.OCPTH =   0x00;  //OCP Threshold 500mV

    // STATUS Register
    G_STATUS_REG.raw     = 0x0000;
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

void DRV8711DRV::set_current(uint32_t currentma)
{
    // derive torque and gain from current
    float c = currentma / 1000.0F; // current in amps
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

    G_TORQUE_REG.TORQUE = t;

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

    //THEKERNEL->streams->printf("for requested current of %lumA, torque= %u, gain= %u, actual current= %fA\n", currentma, G_TORQUE_REG.TORQUE, gain, (2.75F * t) / (256.0F * gain * resistor));
    // for current of 1.500000A, torque= 139, gain= 20

    // set GAIN
    uint8_t dataHi = REGWRITE | ((G_CTRL_REG.raw >> 8) & 0x7F);
    uint8_t dataLo = (G_CTRL_REG.raw & 0x00FF);
    ReadWriteRegister(dataHi, dataLo);

    // set TORQUE
    dataHi = REGWRITE | ((G_TORQUE_REG.raw >> 8) & 0x7F);
    dataLo = (G_TORQUE_REG.raw & 0x00FF);
    ReadWriteRegister(dataHi, dataLo);
}

int DRV8711DRV::set_microsteps(int number_of_steps)
{
    int microsteps;
    if (number_of_steps >= 256) {
        G_CTRL_REG.MODE     = 0x08;
        microsteps = 256;
    } else if (number_of_steps >= 128) {
        G_CTRL_REG.MODE     = 0x07;
        microsteps = 128;
    } else if (number_of_steps >= 64) {
        G_CTRL_REG.MODE     = 0x06;
        microsteps = 64;
    } else if (number_of_steps >= 32) {
        G_CTRL_REG.MODE     = 0x05;
        microsteps = 32;
    } else if (number_of_steps >= 16) {
        G_CTRL_REG.MODE     = 0x04;
        microsteps = 16;
    } else if (number_of_steps >= 8) {
        G_CTRL_REG.MODE     = 0x03;
        microsteps = 8;
    } else if (number_of_steps >= 4) {
        G_CTRL_REG.MODE     = 0x02;
        microsteps = 4;
    } else if (number_of_steps >= 2) {
        G_CTRL_REG.MODE     = 0x01;
        microsteps = 2;
    } else {
        //1 and 0 lead to full step
        G_CTRL_REG.MODE     = 0x0;
        microsteps = 1;
    }

    uint8_t dataHi = REGWRITE | ((G_CTRL_REG.raw >> 8) & 0x7F);
    uint8_t dataLo = (G_CTRL_REG.raw & 0x00FF);
    ReadWriteRegister(dataHi, dataLo);
    return microsteps;
}

void DRV8711DRV::set_enable (bool enable)
{
    // Set Enable
    G_CTRL_REG.ENBL = enable ? 0x01 : 0x00;
    uint8_t dataHi = REGWRITE | ((G_CTRL_REG.raw >> 8) & 0x7F);
    uint8_t dataLo = (G_CTRL_REG.raw & 0x00FF);
    ReadWriteRegister(dataHi, dataLo);
}

void DRV8711DRV::dump_status(StreamOutput *stream)
{
    CTRL_Register_t    R_CTRL_REG;
    TORQUE_Register_t  R_TORQUE_REG;
    OFF_Register_t     R_OFF_REG;
    BLANK_Register_t   R_BLANK_REG;
    DECAY_Register_t   R_DECAY_REG;
    STALL_Register_t   R_STALL_REG;
    DRIVE_Register_t   R_DRIVE_REG;
    STATUS_Register_t  R_STATUS_REG;

    stream->printf("Register Dump:\n");

    // Read CTRL Register
    R_CTRL_REG.raw= ReadRegister(G_CTRL_REG.Address);
    stream->printf("CTRL: %04X (%04X): ", R_CTRL_REG.raw & 0x0FFF, G_CTRL_REG.raw & 0x0FFF);
    stream->printf("DTIME: %u, ISGAIN: %u, EXSTALL: %u, MODE: %u, RSTEP: %u, RDIR: %u, ENBL: %u - ",
                   R_CTRL_REG.DTIME, R_CTRL_REG.ISGAIN, R_CTRL_REG.EXSTALL, R_CTRL_REG.MODE, R_CTRL_REG.RSTEP, R_CTRL_REG.RDIR, R_CTRL_REG.ENBL);
    stream->printf("(DTIME: %u, ISGAIN: %u, EXSTALL: %u, MODE: %u, RSTEP: %u, RDIR: %u, ENBL: %u)\n",
                   G_CTRL_REG.DTIME, G_CTRL_REG.ISGAIN, G_CTRL_REG.EXSTALL, G_CTRL_REG.MODE, G_CTRL_REG.RSTEP, G_CTRL_REG.RDIR, G_CTRL_REG.ENBL);

    // Read TORQUE Register
    R_TORQUE_REG.raw= ReadRegister(G_TORQUE_REG.Address);
    stream->printf("TORQUE: %04X (%04X):", R_TORQUE_REG.raw & 0x0FFF, G_TORQUE_REG.raw & 0x0FFF);
    stream->printf("SIMPLTH: %u, TORQUE: %u - ",  R_TORQUE_REG.SIMPLTH, R_TORQUE_REG.TORQUE);
    stream->printf("(SIMPLTH: %u, TORQUE: %u)\n",  G_TORQUE_REG.SIMPLTH, G_TORQUE_REG.TORQUE);

    // Read OFF Register
    R_OFF_REG.raw= ReadRegister(G_OFF_REG.Address);
    stream->printf("OFF: %04X (%04X) - ", R_OFF_REG.raw & 0x0FFF, G_OFF_REG.raw & 0x0FFF);
    stream->printf("PWMMODE: %u, TOFF: %u - ", R_OFF_REG.PWMMODE, R_OFF_REG.TOFF);
    stream->printf("(PWMMODE: %u, TOFF: %u)\n", G_OFF_REG.PWMMODE, G_OFF_REG.TOFF);

    // Read BLANK Register
    R_BLANK_REG.raw= ReadRegister(G_BLANK_REG.Address);
    stream->printf("BLANK: %04X (%04X) - ", R_BLANK_REG.raw & 0x0FFF, G_BLANK_REG.raw & 0x0FFF);
    stream->printf("ABT: %u, TBLANK: %u - ", R_BLANK_REG.ABT, R_BLANK_REG.TBLANK);
    stream->printf("(ABT: %u, TBLANK: %u)\n", G_BLANK_REG.ABT, G_BLANK_REG.TBLANK);

    // Read DECAY Register
    R_DECAY_REG.raw= ReadRegister(G_DECAY_REG.Address);
    stream->printf("DECAY: %04X (%04X) - ", R_DECAY_REG.raw & 0x0FFF, G_DECAY_REG.raw & 0x0FFF);
    stream->printf("DECMOD: %u, TDECAY: %u - ", R_DECAY_REG.DECMOD, R_DECAY_REG.TDECAY);
    stream->printf("(DECMOD: %u, TDECAY: %u)\n", G_DECAY_REG.DECMOD, G_DECAY_REG.TDECAY);

    // Read STALL Register
    R_STALL_REG.raw= ReadRegister(G_STALL_REG.Address);
    stream->printf("STALL: %04X (%04X) - ", R_STALL_REG.raw & 0x0FFF, G_STALL_REG.raw & 0x0FFF);
    stream->printf("VDIV: %u, SDCNT: %u, SDTHR: %u - ", R_STALL_REG.VDIV, R_STALL_REG.SDCNT, R_STALL_REG.SDTHR);
    stream->printf("(VDIV: %u, SDCNT: %u, SDTHR: %u)\n", G_STALL_REG.VDIV, G_STALL_REG.SDCNT, G_STALL_REG.SDTHR);

    // Read DRIVE Register
    R_DRIVE_REG.raw= ReadRegister(G_DRIVE_REG.Address);
    stream->printf("DRIVE: %04X (%04X) - ", R_DRIVE_REG.raw & 0x0FFF, G_DRIVE_REG.raw & 0x0FFF);
    stream->printf("IDRIVEP: %u, IDRIVEN: %u, TDRIVEP: %u, TDRIVEN: %u, OCPDEG: %u, OCPTH: %u - ",
                   R_DRIVE_REG.IDRIVEP, R_DRIVE_REG.IDRIVEN, R_DRIVE_REG.TDRIVEP, R_DRIVE_REG.TDRIVEN, R_DRIVE_REG.OCPDEG, R_DRIVE_REG.OCPTH);
    stream->printf("(IDRIVEP: %u, IDRIVEN: %u, TDRIVEP: %u, TDRIVEN: %u, OCPDEG: %u, OCPTH: %u)\n",
                   G_DRIVE_REG.IDRIVEP, G_DRIVE_REG.IDRIVEN, G_DRIVE_REG.TDRIVEP, G_DRIVE_REG.TDRIVEN, G_DRIVE_REG.OCPDEG, G_DRIVE_REG.OCPTH);

    // Read STATUS Register
    R_STATUS_REG.raw= ReadRegister(G_STATUS_REG.Address);
    stream->printf("STATUS: %02X - ", R_STATUS_REG.raw & 0x00FF);
    stream->printf("STDLAT: %u, STD: %u, UVLO: %u, BPDF: %u, APDF: %u, BOCP: %u, AOCP: %u, OTS: %u\n",
                   R_STATUS_REG.STDLAT, R_STATUS_REG.STD, R_STATUS_REG.UVLO, R_STATUS_REG.BPDF, R_STATUS_REG.APDF, R_STATUS_REG.BOCP, R_STATUS_REG.AOCP, R_STATUS_REG.OTS);

    int gain = R_CTRL_REG.ISGAIN == 0 ? 5 : R_CTRL_REG.ISGAIN == 1 ? 10 : R_CTRL_REG.ISGAIN == 2 ? 20 : R_CTRL_REG.ISGAIN == 3 ? 40 : 0;
    stream->printf(" Current: %f\n", (2.75F * R_TORQUE_REG.TORQUE) / (256.0F * gain * resistor));
    stream->printf(" Microsteps: 1/%d\n", R_CTRL_REG.MODE > 0 ? 2 << (R_CTRL_REG.MODE-1) : 1);

    stream->printf(" motor_driver_control.xxx.reg %03X,%03X,%03X,%03X,%03X,%03X,%03X\n",
                   G_CTRL_REG.raw & 0x0FFF, G_TORQUE_REG.raw & 0x0FFF, G_OFF_REG.raw & 0x0FFF, G_BLANK_REG.raw & 0x0FFF, G_DECAY_REG.raw & 0x0FFF, G_STALL_REG.raw & 0x0FFF, G_DRIVE_REG.raw & 0x0FFF);
}

void DRV8711DRV::WriteAllRegisters()
{
    uint8_t dataHi = 0x00;
    uint8_t dataLo = 0x00;

    // Write CTRL Register
    dataHi = REGWRITE | (G_CTRL_REG.Address << 4) | (G_CTRL_REG.DTIME << 2) | (G_CTRL_REG.ISGAIN);
    dataLo = (G_CTRL_REG.EXSTALL << 7) | (G_CTRL_REG.MODE << 3) | (G_CTRL_REG.RSTEP << 2) | (G_CTRL_REG.RDIR << 1) | (G_CTRL_REG.ENBL);
    ReadWriteRegister(dataHi, dataLo);

    // Write TORQUE Register
    dataHi = REGWRITE | (G_TORQUE_REG.Address << 4) | (G_TORQUE_REG.SIMPLTH);
    dataLo = G_TORQUE_REG.TORQUE;
    ReadWriteRegister(dataHi, dataLo);

    // Write OFF Register
    dataHi = REGWRITE | (G_OFF_REG.Address << 4) | (G_OFF_REG.PWMMODE);
    dataLo = G_OFF_REG.TOFF;
    ReadWriteRegister(dataHi, dataLo);

    // Write BLANK Register
    dataHi = REGWRITE | (G_BLANK_REG.Address << 4) | (G_BLANK_REG.ABT);
    dataLo = G_BLANK_REG.TBLANK;
    ReadWriteRegister(dataHi, dataLo);

    // Write DECAY Register
    dataHi = REGWRITE | (G_DECAY_REG.Address << 4) | (G_DECAY_REG.DECMOD);
    dataLo = G_DECAY_REG.TDECAY;
    ReadWriteRegister(dataHi, dataLo);

    // Write STALL Register
    dataHi = REGWRITE | (G_STALL_REG.Address << 4) | (G_STALL_REG.VDIV << 2) | (G_STALL_REG.SDCNT);
    dataLo = G_STALL_REG.SDTHR;
    ReadWriteRegister(dataHi, dataLo);

    // Write DRIVE Register
    dataHi = REGWRITE | (G_DRIVE_REG.Address << 4) | (G_DRIVE_REG.IDRIVEP << 2) | (G_DRIVE_REG.IDRIVEN);
    dataLo = (G_DRIVE_REG.TDRIVEP << 6) | (G_DRIVE_REG.TDRIVEN << 4) | (G_DRIVE_REG.OCPDEG << 2) | (G_DRIVE_REG.OCPTH);
    ReadWriteRegister(dataHi, dataLo);

    // Write STATUS Register
    dataHi = REGWRITE | (G_STATUS_REG.Address << 4);
    dataLo = (G_STATUS_REG.STDLAT << 7) | (G_STATUS_REG.STD << 6) | (G_STATUS_REG.UVLO << 5) | (G_STATUS_REG.BPDF << 4) | (G_STATUS_REG.APDF << 3) | (G_STATUS_REG.BOCP << 2) | (G_STATUS_REG.AOCP << 1) | (G_STATUS_REG.OTS);
    ReadWriteRegister(dataHi, dataLo);
}

bool DRV8711DRV::check_alarm()
{
    bool error= false;
    STATUS_Register_t  R_STATUS_REG;
    // Read STATUS Register
    R_STATUS_REG.raw= ReadRegister(G_STATUS_REG.Address);

    if(R_STATUS_REG.OTS) {
        if(!error_reported.test(0)) THEKERNEL->streams->printf("ERROR: Overtemperature shutdown\n");
        error= true;
        error_reported.set(0);
    }else{
        error_reported.reset(0);
    }


    if(R_STATUS_REG.AOCP) {
        if(!error_reported.test(1)) THEKERNEL->streams->printf("ERROR: Channel A over current shutdown\n");
        error= true;
        error_reported.set(1);
    }else{
        error_reported.reset(1);
    }


    if(R_STATUS_REG.BOCP) {
        if(!error_reported.test(2)) THEKERNEL->streams->printf("ERROR: Channel B over current shutdown\n");
        error= true;
        error_reported.set(2);
    }else{
        error_reported.reset(2);
    }

    if(R_STATUS_REG.APDF) {
        if(!error_reported.test(3)) THEKERNEL->streams->printf("ERROR: Channel A predriver fault\n");
        error= true;
        error_reported.set(3);
    }else{
        error_reported.reset(3);
    }


    if(R_STATUS_REG.BPDF) {
        if(!error_reported.test(4)) THEKERNEL->streams->printf("ERROR: Channel B predriver fault\n");
        error= true;
        error_reported.set(4);
    }else{
        error_reported.reset(4);
    }


    return error;
}


// sets a raw register to the value specified, for advanced settings
// register 255 writes them, 0 displays what registers are mapped to what
bool DRV8711DRV::set_raw_register(StreamOutput *stream, uint32_t reg, uint32_t val)
{
    switch(reg) {
        case 255:
            WriteAllRegisters();
            stream->printf("Registers written\n");
            break;

        case 1: G_CTRL_REG.raw   &= 0xF000; G_CTRL_REG.raw   |= (val & 0x0FFF); break;
        case 2: G_TORQUE_REG.raw &= 0xF000; G_TORQUE_REG.raw |= (val & 0x0FFF); break;
        case 3: G_OFF_REG.raw    &= 0xF000; G_OFF_REG.raw    |= (val & 0x0FFF); break;
        case 4: G_BLANK_REG.raw  &= 0xF000; G_BLANK_REG.raw  |= (val & 0x0FFF); break;
        case 5: G_DECAY_REG.raw  &= 0xF000; G_DECAY_REG.raw  |= (val & 0x0FFF); break;
        case 6: G_STALL_REG.raw  &= 0xF000; G_STALL_REG.raw  |= (val & 0x0FFF); break;
        case 7: G_DRIVE_REG.raw  &= 0xF000; G_DRIVE_REG.raw  |= (val & 0x0FFF); break;

        default:
            stream->printf("1: CTRL Register\n");
            stream->printf("2: TORQUE Register\n");
            stream->printf("3: OFF Register\n");
            stream->printf("4: BLANK Register\n");
            stream->printf("5: DECAY Register\n");
            stream->printf("6: STALL Register\n");
            stream->printf("7: DRIVE Register\n");
            stream->printf("255: write registers to chip\n");
            return false;
    }
    return true;
}

uint16_t DRV8711DRV::ReadRegister(uint8_t addr)
{
    return ReadWriteRegister(REGREAD | (addr << 4), 0);
}

uint16_t DRV8711DRV::ReadWriteRegister(uint8_t dataHi, uint8_t dataLo)
{
    uint8_t buf[2] {dataHi, dataLo};
    uint8_t rbuf[2];

    spi(buf, 2, rbuf);
    //THEKERNEL->streams->printf("sent: %02X, %02X received:%02X, %02X\n", buf[0], buf[1], rbuf[0], rbuf[1]);
    uint16_t readData = (rbuf[0] << 8) | rbuf[1];
    return readData;
}

#if 0
#define HAS(X) (options.find(X) != options.end())
#define GET(X) (options.at(X))
bool DRV8711DRV::set_options(const options_t& options)
{
    bool set = false;
    if(HAS('O') || HAS('Q')) {
        // void TMC26X::setStallGuardThreshold(int8_t stall_guard_threshold, int8_t stall_guard_filter_enabled)
        int8_t o = HAS('O') ? GET('O') : getStallGuardThreshold();
        int8_t q = HAS('Q') ? GET('Q') : getStallGuardFilter();
        setStallGuardThreshold(o, q);
        set = true;
    }

    if(HAS('S')) {
        uint32_t s = GET('S');
        if(s == 0 && HAS('U') && HAS('V') && HAS('W') && HAS('X') && HAS('Y')) {
            //void TMC26X::setConstantOffTimeChopper(int8_t constant_off_time, int8_t blank_time, int8_t fast_decay_time_setting, int8_t sine_wave_offset, uint8_t use_current_comparator)
            setConstantOffTimeChopper(GET('U'), GET('V'), GET('W'), GET('X'), GET('Y'));
            set = true;

        } else if(s == 2 && HAS('Z')) {
            setRandomOffTime(GET('Z'));
            set = true;
        }
    }

    return set;
}
#endif

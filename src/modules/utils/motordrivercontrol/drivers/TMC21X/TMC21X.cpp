/*
 Modified from TMC26X.cpp

 based on the stepper library by Tom Igoe, et. al.

 Copyright (c) 2011, Interactive Matter, Marcus Nowotny

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.

 */

#include "TMC21X.h"
#include "mbed.h"
#include "StreamOutput.h"
#include "Kernel.h"
#include "libs/StreamOutputPool.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "ConfigValue.h"
#include "Config.h"
#include "checksumm.h"
#include "StepTicker.h"

//TMC21X register definitions

/* GENERAL CONFIGURATION REGISTERS (0x00...0x0F) */

//         Register               Address    Type    N Bits   Description
/********************************************************************************/
#define GCONF_REGISTER            0x00   // RW       17       /* Global configuration flags */
/********************************************************************************/
//         Function                Bit     Description
#define GCONF_I_SCALE_ANALOG     1 << 0    /*0: Normal operation, use internal reference voltage
                                            *1: Use voltage supplied to AIN as current reference
                                            */
#define GCONF_INTERNAL_RSENSE    1 << 1    /*0: Normal operation
                                            *1: Internal sense resistors. Use current supplied into AIN as reference for internal sense resistor */
#define GCONF_EN_PWM_MODE        1 << 2    //1: stealthChop voltage PWM mode enabled (depending on velocity thresholds).
                                           //Switch from off to on state while in stand still, only.
#define GCONF_ENC_COMUTATION     1 << 3    //1: Enable commutation by full step encoder (DCIN_CFG5 = ENC_A, DCEN_CFG4 = ENC_B)
#define GCONF_SHAFT              1 << 4    //1: Inverse motor direction
#define GCONF_DIAG0_ERROR        1 << 5    //1: Enable DIAG0 active on driver errors: Over temperature (ot), short to GND (s2g), undervoltage chargepump (uv_cp)
                                           //DIAG0 always shows the reset-status, i.e. is active low during reset condition.
#define GCONF_DIAG0_OTPW         1 << 6    //1: Enable DIAG0 active on driver over temperature prewarning (otpw)
#define GCONF_DIAG0_STALL        1 << 7    //1: Enable DIAG0 active on motor stall (set TCOOLTHRS before using this feature)
#define GCONF_DIAG1_STALL        1 << 8    //1: Enable DIAG1 active on motor stall (set TCOOLTHRS before using this feature)
#define GCONF_DIAG1_INDEX                   1 << 9  //1: Enable DIAG1 active on index position (microstep look up table position 0)
#define GCONF_DIAG1_ONSTATE                 1 << 10 //1: Enable DIAG1 active when chopper is on (for the coil which is in the second half of the fullstep)
#define GCONF_DIAG1_STEPS_SKIPPED           1 << 11 //1: Enable output toggle when steps are skipped in dcStep mode (increment of LOST_STEPS). Do not enable in conjunction with other DIAG1 options.
#define GCONF_DIAG0_INT_PUSHPULL            1 << 12 //0: DIAG0 is open collector output (active low)
                                                    //1: Enable DIAG0 push pull output (active high)
#define GCONF_DIAG1_PUSHPULL                1 << 13 //0: DIAG1 is open collector output (active low)
                                                    //1: Enable DIAG1 push pull output (active high)
#define GCONF_SMALL_HYSTERESIS              1 << 14 //0: Hysteresis for step frequency comparison is 1/16
                                                    //1: Hysteresis for step frequency comparison is 1/32
#define GCONF_STOP_ENABLE                   1 << 15 //0: Normal operation
                                                    //1: Emergency stop: DCIN stops the sequencer when tied high (no steps become executed by the sequencer, motor goes to standstill state).
#define GCONF_DIRECT_MODE                   1 << 16 //0: Normal operation
                                                    //1: Motor coil currents and polarity directly programmed via serial interface:
                                                    //      Register XDIRECT (0x2D) specifies signed coil A current (bits 8..0) and coil B current (bits 24..16).
                                                    //      In this mode, the current is scaled by IHOLD setting.
                                                    //      Velocity based current regulation of stealthChop is not available in this mode.
                                                    //      The automatic stealthChop current regulation will work only for low stepper motor velocities.
#define GCONF_TEST_MODE                    1 << 17  //0: Normal operation
                                                    //1: Enable analog test output on pin DCO. IHOLD[1..0] selects the function of DCO: 0…2: T120, DAC, VDDH
                                                    //Attention: Not for user, set to 0 for normal operation!

//         Register         Address     Type    N Bits    Description
/********************************************************************************/
#define GSTAT_REGISTER       0x01    // R + C      3      Global status flags
/********************************************************************************/
//         Function                       Bit     Description
#define GSTAT_RESET                      1 << 0  //1: Indicates that the IC has been reset since the last read access to GSTAT.
#define GSTAT_DRV_ERR                    1 << 1  //1: Indicates, that the driver has been shut down due to over temperature or short circuit detection since the last read access.
                                                 //Read DRV_STATUS for details. The flag can only be reset when all error conditions are cleared.
#define GSTAT_UV_CP                      1 << 2  //1: Indicates an under voltage on the charge pump. The driver is disabled in this case.


//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define IOIN_REGISTER        0x04     // R     8 + 8     Reads the state of all input pins available
/********************************************************************************/
//         Function                       Bit     Description
#define IOIN_STEP                        1 << 0   //1: STEP pin high state
#define IOIN_DIR                         1 << 1   //1: DIR pin high state
#define IOIN_DCEN_CFG4                   1 << 2   //1: DCEN_CFG4 pin high state
#define IOIN_DCIN_CFG5                   1 << 3   //1: DCIN_CFG5 pin high state
#define IOIN_DRV_ENN_CFG6                1 << 4   //1: DRV_ENN_CFG6 pin high state
#define IOIN_DCO                         1 << 5   //1: DCO pin high state
#define IOIN_VERSION_MASK                0xFF000000ul    //0x11=first version of the IC
                                                         //24 Identical numbers mean full digital compatibilityBits 31..24 are the version of the IC

/* VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0X10...0X1F) */

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define IHOLD_IRUN_REGISTER  0x10      // W    5+5+4  //Driver current control
/********************************************************************************/
//         Function                          Bit      Description
#define REG_IHOLD_IRUN_IHOLD_MASK           0x1Ful    //Bits 4..0: Standstill current (0=1/32…31=32/32)
                                                      //In combination with stealthChop mode, setting IHOLD=0 allows to choose freewheeling or coil short circuit for motor stand still.
#define REG_IHOLD_IRUN_IRUN_MASK            0x1F00ul  //Bits 12..8: Motor run current (0=1/32…31=32/32)
                                                      //
                                                      //Hint: Choose sense resistors in a way, that normal IRUN is 16 to 31 for best microstep performance.
#define REG_IHOLD_IRUN_IHOLDDELAY_MASK      0xF0000ul //Bits 19..16: Controls the number of clock cycles for motor power down after a motion as soon as standstill is detected (stst=1) and TPOWERDOWN has expired.
                                                      //The smooth transition avoids a motor jerk upon power down.
                                                      //
                                                      //0: instant power down
                                                      //1..15: Delay per current reduction step in multiple of 2^18 clocks

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define TPOWERDOWN_REGISTER  0x11      // W      8       //Sets the delay time after stand still (stst) of the motor to motor current power down.
                                                         //Time range is about 0 to 4 seconds. 0…((2^8)-1) * 2^18 tCLK
/********************************************************************************/

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define TSTEP_REGISTER       0x12    // R        20      //Actual measured time between two 1/256 microsteps derived from the step input frequency.
                                                         //In units of 1/fCLK. Measured value is (2^20)-1 in case of overflow or stand still.
                                                         //
                                                         //All TSTEP related thresholds use a hysteresis of 1/16 of the compare value to compensate for jitter in the clock or the step frequency.
                                                         //The flag small_hysteresis modifies the hysteresis to a smaller value of 1/32.
                                                         //(Txxx*15/16)-1 or (Txxx*31/32)-1 is used as a second compare value for each comparison value.
                                                         //This means, that the lower switching velocity equals the calculated setting,
                                                         //but the upper switching velocity is higher as defined by the hysteresis setting.
                                                         //
                                                         //In dcStep mode TSTEP will not show the mean velocity of the motor, but the velocities for each microstep,
                                                         //which may not be stable and thus does not represent the real motor velocity in case it runs slower than the target velocity.
                                                         //
                                                         //NOTE: microstep velocity time reference t for velocities: TSTEP = fCLK / fSTEP
/********************************************************************************/

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define TPWMTHRS_REGISTER    0x13    // W        20      //This is the upper velocity for stealthChop voltage PWM mode.
                                                         //TSTEP ≥ TPWMTHRS
                                                         //  - stealthChop PWM mode is enabled, if configured
                                                         //  - dcStep is disabled
/********************************************************************************/

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define TCOOLTHRS_REGISTER   0x14    // W  //    20      //This is the lower threshold velocity for switching on smart energy coolStep and stallGuard feature.
                                                         //
                                                         //Set this parameter to disable coolStep at low speeds, where it cannot work reliably.
                                                         //The stall detection and stallGuard output signal becomes enabled when exceeding this velocity.
                                                         //In non-dcStep mode, it becomes disabled again once the velocity falls below this threshold.
                                                         //
                                                         //TCOOLTHRS ≥ TSTEP ≥ THIGH:
                                                         //  - coolStep is enabled, if configured
                                                         //  - stealthChop voltage PWM mode is disabled
                                                         //
                                                         //TCOOLTHRS ≥ TSTEP
                                                         //  - Stop on stall and stall output signal is enabled, if configured
/********************************************************************************/

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define THIGH_REGISTER       0x15    // W  //    20      //This velocity setting allows velocity dependent switching into a different chopper mode and fullstepping to maximize torque. (unsigned)
                                                         //The stall detection feature becomes switched off for 2-3 electrical periods whenever passing THIGH threshold to compensate for the effect of switching modes.
                                                         //
                                                         //TSTEP ≤ THIGH:
                                                         //  - coolStep is disabled (motor runs with normal current scale)
                                                         //  - stealthChop voltage PWM mode is disabled
                                                         //  - If vhighchm is set, the chopper switches to chm=1 with TFD=0 (constant off time with slow decay, only).
                                                         //  - chopSync2 is switched off (SYNC=0)
                                                         //  - If vhighfs is set, the motor operates in fullstep mode and the stall detection becomes switched over to dcStep stall detection.
                                                         //NOTE: microstep velocity time reference t for velocities: TSTEP = fCLK / fSTEP
/********************************************************************************/

/* SPI MODE REGISTER (0X2D) */

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define XDIRECT_REGISTER     0x2D    // RW       32      // SPI Mode Register
                                                         //NOTE: This register cannot be used in STEP/DIR mode.
                                                         //
                                                         //  Direct mode operation:
                                                         //0: Normal operation
                                                         //1: Directly SPI driven motor current
                                                         //  XDIRECT specifies Motor coil currents and polarity directly programmed via the serial interface. Use signed, two’s complement numbers.
                                                         //
                                                         //In this mode, the current is scaled by IHOLD setting.
                                                         //Velocity based current regulation of voltage PWM is not available in this mode.
                                                         //The automatic voltage PWM current regulation will work only for low stepper motor velocities.
                                                         //dcStep is not available in this mode. coolStep and stallGuard only can be used, when additionally supplying a STEP signal.
                                                         //This will also enable automatic current scaling.
/********************************************************************************/
//         Function                          Bit         Description
#define XDIRECT_COIL_A                       0x1FFUL     //Coil A current (bits 8..0) (signed)
                                                         //Range: +-248 for normal operation, up to +-255 with stealthChop
#define XDIRECT_COIL_B                       0x1FF0000UL //Coil B current (bits 24..16) (signed)
                                                         //Range: +-248 for normal operation, up to +-255 with stealthChop

/* DCSTEP MINIMUM VELOCITY REGISTER (0x33) */

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define VDCMIN_REGISTER      0x33    // W        23      //dcStep Minimum Velocity Register
                                                         //The automatic commutation dcStep becomes enabled by the external signal DCEN.
                                                         //VDCMIN is used as the minimum step velocity when the motor is heavily loaded.
                                                         //
                                                         //Hint: Also set DCCTRL parameters in order to operate dcStep.
                                                         //
                                                         //NOTE: time reference t for VDCMIN: t = 2^24 / fCLK
/********************************************************************************/

/* MICROSTEPPING CONTROL REGISTER SET (0X60...0X6B) */

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define MSLUT0_REGISTER     0x60       // W       32     Microstep table entries 0*31+0..31.
                                                         //Range: 32 * 0 or 1 , reset default = sine wave table
#define MSLUT1_REGISTER     0x61       // W       32     Microstep table entries 1*31+0..31. Range: 32 * 0 or 1 (reset default = sine wave table)
#define MSLUT2_REGISTER     0x62       // W       32     Microstep table entries 2*31+0..31. Range: 32 * 0 or 1 (reset default = sine wave table)
#define MSLUT3_REGISTER     0x63       // W       32     Microstep table entries 3*31+0..31. Range: 32 * 0 or 1 (reset default = sine wave table)
#define MSLUT4_REGISTER     0x64       // W       32     Microstep table entries 4*31+0..31. Range: 32 * 0 or 1 (reset default = sine wave table)
#define MSLUT5_REGISTER     0x65       // W       32     Microstep table entries 5*31+0..31. Range: 32 * 0 or 1 (reset default = sine wave table)
#define MSLUT6_REGISTER     0x66       // W       32     Microstep table entries 6*31+0..31. Range: 32 * 0 or 1 (reset default = sine wave table)
#define MSLUT7_REGISTER     0x67       // W       32     Microstep table entries 7*31+0..31.
                                                         // Range: 7 * 32 * 0 or 1 , reset default = sine wave table
                                                         //
                                                         //Each bit gives the difference between entry x and entry x+1 when combined with the
                                                         //corresponding MSLUTSEL W bits:
                                                         //0: W= %00: -1
                                                         //      %01: +0
                                                         //      %10: +1
                                                         //      %11: +2
                                                         //      1: W= %00: +0
                                                         //      %01: +1
                                                         //      %10: +2
                                                         //      %11: +3
                                                         //This is the differential coding for the first quarter of a wave.
                                                         //Start values for CUR_A and CUR_B are stored for MSCNT position 0 in START_SIN and START_SIN90.
                                                         //ofs31, ofs30, …, ofs01, ofs00
                                                         //…
                                                         //ofs255, ofs254, …, ofs225, ofs224
/********************************************************************************/

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define MSLUTSEL_REGISTER   0x68       // W      32      //This register defines four segments within each quarter MSLUT wave.
                                                         //Four 2 bit entries determine the meaning of a 0 and a 1 bit in the corresponding segment of MSLUT.
                                                         //Range: 0<X1<X2<X3, reset default = sine wave table.

/********************************************************************************/
//         Function                          Bit         Description
#define MSUTSEL_X3                        0xFF000000ul   //LUT segment 3 start
#define MSUTSEL_X2                        0xFF0000ul     //LUT segment 2 start
#define MSUTSEL_X1                        0xFF00ul       //LUT segment 1 start
                                                         //The sine wave look up table can be divided into up to four segments using an individual step width controlentry Wx.
                                                         //The segment borders are selected by X1, X2 and X3.
                                                         //
                                                         //Segment 0 goes from 0 to X1-1.
                                                         //Segment 1 goes from X1 to X2-1.
                                                         //Segment 2 goes from X2 to X3-1.
                                                         //Segment 3 goes from X3 to 255.
                                                         //
                                                         //For defined response the values shall satisfy: 0<X1<X2<X3

#define MSUTSEL_W3                        (3 << 7)       //LUT width select from ofs(X3) to ofs255
#define MSUTSEL_W2                        (3 << 5)       //LUT width select from ofs(X2) to ofs(X3-1)
#define MSUTSEL_W1                        (3 << 3)       //LUT width select from ofs(X1) to ofs(X2-1)
#define MSUTSEL_W0                        (3 << 1)       //LUT width select from ofs00 to ofs(X1-1)
                                                         //
                                                         //Width control bit coding W0...W3:
                                                         //%00: MSLUT entry 0, 1 select: -1, +0
                                                         //%01: MSLUT entry 0, 1 select: +0, +1
                                                         //%10: MSLUT entry 0, 1 select: +1, +2
                                                         //%11: MSLUT entry 0, 1 select: +2, +3

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define MSLUTSTART_REGISTER 0x69     // W       8+8      //bit 7… 0: START_SIN
                                                         //bit 23… 16: START_SIN90
/********************************************************************************/
//         Function                       Bit            Description
#define MSLUTSTART_START_SIN              0xFFul          //START_SIN gives the absolute current at microstep table entry 0.
                                                         //Range: START_SIN, reset default =0
#define MSLUTSTART_START_SIN90            0xFF00ul        //START_SIN90 gives the absolute current for microstep table entry at positions 256.
                                                         //Range: START_SIN 90, reset default =247
                                                         //
                                                         //Start values are transferred to the microstep registers CUR_A and CUR_B, whenever
                                                         //the reference position MSCNT=0 is passed.
/********************************************************************************/

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define MSCNT_REGISTER      0x6A    // R        10       //Microstep counter.
                                                         //Indicates actual position in the microstep table for CUR_A. CUR_B uses an offset of 256 (2 phase motor).
                                                         //Hint: Move to a position where MSCNT is zero before re-initializing MSLUTSTART or MSLUT and MSLUTSEL.
                                                         //Range: 0..123
/********************************************************************************/

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define MSCURACT_REGISTER   0x6B    // R        9+9      //bit 8… 0: CUR_A (signed):
                                                         //bit 24… 16: CUR_B (signed):
/********************************************************************************/
//         Function                       Bit            Description
#define MSCURACT_CUR_A                   0x1FFul         //Actual microstep current for motor phase A as read from MSLUT (not scaled by current)
                                                         //Range +/-0...255
#define MSCURACT_CUR_B                   0x1FF0000ul     //Actual microstep current for motor phase B as read from MSLUT (not scaled by current)
                                                         //Range +/-0...255

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define CHOPCONF_REGISTER   0x6C     // RW       32      //chopper and driver configuration
/********************************************************************************/
//         Function                       Bit            Description
#define CHOPCONF_DISS2G                (1 << 30)         //short to GND protection disable
                                                         //0: Short to GND protection is on
                                                         //1: Short to GND protection is disabled
#define CHOPCONF_DEDGE                 (1 << 29)         //enable souble edge step pulses
                                                         //1: Enable step impulse at each step edge to reduce step frequency requirement.
                                                         //1: Short to GND protection is disabled
#define CHOPCONF_INTPOL                (1 << 28)         //interpolation to 256 microsteps
                                                         //1: The actual microstep resolution (MRES) becomes extrapolated to 256 microsteps for smoothest motor operation.
#define CHOPCONF_MRES                  (15 << 24)        //micro step resolution
                                                         //%0000: Native 256 microstep setting
                                                         //%0001 ... %1000: 128, 64, 32, 16, 8, 4, 2, FULLSTEP
                                                         //Reduced microstep resolution for STEP/DIR operation.
                                                         //The resolution gives the number of microstep entries per sine quarter wave.
                                                         //The driver automatically uses microstep positions which result in a symmetrical wave, when choosing a lower microstep resolution.
                                                         //step width=2^MRES [microsteps]
#define CHOPCONF_SYNC                  (15 << 20)        //PWM synchronization clock
                                                         //This register allows synchronization of the chopper for both phases of a two phase motor in order to avoid the
                                                         //occurrence of a beat, especially at low motor velocities. It is automatically switched off above VHIGH.
                                                         //%0000: Chopper sync function chopSync off
                                                         //%0001 ... %1111:
                                                         //Synchronization with fSYNC = fCLK/(sync*64)
                                                         //Hint: Set TOFF to a low value, so that the chopper cycle is ended, before the next sync clock pulse occurs.
                                                         //Set for the double desired chopper frequency for chm=0, for the desired base chopper frequency for chm=1.
#define CHOPCONF_VHIGHCHM              (1 << 19)         //high velocity chopper mode
                                                         //This bit enables switching to chm=1 and fd=0, when VHIGH is exceeded. This way, a higher velocity can be achieved.
                                                         //Can be combined with vhighfs=1. If set, the TOFF setting automatically becomes doubled during high velocity
                                                         //operation in order to avoid doubling of the chopper frequency.
#define CHOPCONF_VHIGHFS               (1 << 18)         //high velocity fullstep selection
                                                         //This bit enables switching to fullstep, when VHIGH is exceeded. Switching takes place only at 45° position.
                                                         //The fullstep target current uses the current value from the microstep table at the 45° position.
#define CHOPCONF_VSENSE                (1 << 17)         //sense resistor voltage based current scaling
                                                         //0: Low sensitivity, high sense resistor voltage
                                                         //1: High sensitivity, low sense resistor voltage
#define CHOPCONF_TBL                   (3 << 15)         //blank time select
                                                         //%00 ... %11:
                                                         //Set comparator blank time to 16, 24, 36 or 54 clocks
                                                         //Hint: %01 or %10 is recommended for most applications
#define CHOPCONF_CHM                   (1 << 14)         //chopper mode
                                                         //0: Standard mode (spreadcycle) Low sensitivity, high sense resistor voltage
                                                         //1: Constant off time with fast decay time. Fast decay time is also terminated when the negative nominal current is reached. Fast decay is after on time.
#define CHOPCONF_RNDTF                 (1 << 13)         //random TOFF time
                                                         //0: Chopper off time is fixed as set by TOFF
                                                         //1: Random mode, TOFF is random modulated by dNCLK = -12 ... +3 clocks.
#define CHOPCONF_DISFDCC               (1 << 12)         //fast decay mode
                                                         //chm=1: disfdcc=1 disables current comparator usage for termination of the fast decay cycle.
#define CHOPCONF_FD3                   (1 << 11)         //TFD [3]
                                                         //chm=1: MSB of fast decay time setting TFD.
#define CHOPCONF_HEND                  (15 << 7)         //CHM=0: HEND hysteresis low value, CHM=1 OFFSET sine wave offset
                                                         //
                                                         //CHM=0
                                                         //%0000 ... %1111: Hysteresis is -3, -2, -1, 0, 1, ..., 12 (1/512 of this setting adds to current setting)
                                                         //This is the hysteresis value which becomes used for the hysteresis chopper.
                                                         //
                                                         //CHM=1
                                                         //%0000 ... %1111: Offset is -3, -2, -1, 0, 1, ..., 12
                                                         //This is the sine wave offset and 1/512 of the value becomes added to the absolute value of each sine wave entry.
#define CHOPCONF_HSTART                (7 << 4)          //CHM=0: HSTRT hysteresis start value added to HEND, CHM=1 TFD [2--0] fast decay time setting (MSB: fd3)
                                                         //
                                                         //CHM=0
                                                         //%0000 ... %1111: Add 1, 2, ..., 8 to hysteresis low value HEND (1/512 of this setting adds to current setting)
                                                         //Attention: Effective HEND+HSTRT ≤ 16.
                                                         //Hint: Hysteresis decrement is done each 16 clocks
                                                         //
                                                         //CHM=1
                                                         //%0000 ... %1111: Offset is -3, -2, -1, 0, 1, ..., 12
                                                         //Fast decay time setting TFD with NCLK = 32*TFD (%0000: slow decay only)
#define CHOPCONF_TOFF                  (15 << 0)         //off time and driver enable
                                                         //Off time setting controls duration of slow decay phase
                                                         //NCLK = 12 + 32*TOFF
                                                         //%0000: Driver disable, all bridges off
                                                         //%0001: 1 – use only with TBL ≥ 2
                                                         //%0010 ... %1111: 2 ... 15

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define COOLCONF_REGISTER   0x6D    // W        25       //coolStep smart current control register and stallGuard2 configuration
/********************************************************************************/
//         Function                       Bit            Description
#define COOLCONF_SFILT                 (1 << 24)         //stallGuard2 filter enable
                                                         //0: Standard mode, high time resolution for stallGuard2
                                                         //1: Filtered mode, stallGuard2 signal updated for each four fullsteps (resp. six fullsteps for 3 phase motor) only to compensate for motor pole tolerances
#define COOLCONF_SGT                   (255 << 16)       //stallGuard2 threshold value
                                                         //0: Standard mode, high time resolution for stallGuard2
                                                         //This signed value controls stallGuard2 level for stall output and sets the optimum measurement range for readout.
                                                         //A lower value gives a higher sensitivity. Zero is the starting value working with most motors.
                                                         //-64 to +63: A higher value makes stallGuard2 less sensitive and requires more torque to indicate a stall.
#define COOLCONF_SEIMIN                (1 << 15)         //minimum current for smart current control
                                                         //0: 1/2 of current setting (IRUN)
                                                         //1: 1/4 of current setting (IRUN)
#define COOLCONF_SEDN                  (3 << 13)         //current down step speed
                                                         //%00: For each 32 stallGuard2 values decrease by one
                                                         //%01: For each 8 stallGuard2 values decrease by one
                                                         //%10: For each 2 stallGuard2 values decrease by one
                                                         //%11: For each stallGuard2 value decrease by one
#define COOLCONF_SEMAX                 (15 << 8)         //stallGuard2 hysteresis value for smart current control
                                                         //If the stallGuard2 result is equal to or above (SEMIN+SEMAX+1)*32, the motor current becomes decreased to save energy.
                                                         //%0000 ... %1111: 0 ... 15
#define COOLCONF_SEUP                  (3 << 5)          //current up step width
                                                         //Current increment steps per measured stallGuard2 value
                                                         //%00 ... %11: 1, 2, 4, 8
#define COOLCONF_SEMIN                 (15 << 0)         //minimum stallGuard2 value for smart current control and smart current enable
                                                         //If the stallGuard2 result falls below SEMIN*32, the motor current becomes increased to reduce motor load angle.
                                                         //%0000: smart current control coolStep off
                                                         //%0001 ... %1111: 1 ... 15

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define DCCTRL_REGISTER     0x6E    // W        24       //dcStep (DC) automatic commutation configuration register (enable via pin DCEN or via VDCMIN):
                                                         //bit 9… 0: DC_TIME: Upper PWM on time limit for commutation (DC_TIME * 1/fCLK). Set slightly above effective blank time TBL.
                                                         //bit 23… 16: DC_SG: Max. PWM on time for step loss detection using dcStep stallGuard2 in dcStep mode. (DC_SG * 16/fCLK)
                                                         //Set slightly higher than DC_TIME/16
                                                         //0=disable
                                                         //Attention: Using a higher microstep resolution or interpolated operation, dcStep delivers a better stallGuard signal.
                                                         //DC_SG is also available above VHIGH if vhighfs is activated. For best result also set vhighchm.
/********************************************************************************/

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define DRVSTATUS_REGISTER  0x6F    // R  //    22       //stallGuard2 value and driver error flags
/********************************************************************************/
//         Function                    Bit            Description
#define DRVSTATUS_STST                 (1 << 31)      //standstill indicator
                                                      //This flag indicates motor stand still in each operation mode. This occurs 2^20 clocks after the last step pulse.
#define DRVSTATUS_OLB                  (1 << 30)      //open load indicator phase B
#define DRVSTATUS_OLA                  (1 << 29)      //open load indicator phase A
                                                      //1: Open load detected on phase A or B.
                                                      //Hint: This is just an informative flag. The driver takes no action upon it.
                                                      //False detection may occur in fast motion and standstill. Check during slow motion, only.
#define DRVSTATUS_S2GB                 (1 << 28)      //short to ground indicator phase B
#define DRVSTATUS_S2GA                 (1 << 27)      //short to ground indicator phase A
                                                      //1: Short to GND detected on phase A or B. The driver becomes disabled.
                                                      //The flags stay active, until the driver is disabled by software (TOFF=0) or by the ENN input.
#define DRVSTATUS_OTPW                 (1 << 26)      //overtemperature pre-warning flag
                                                      //1: Overtemperature pre-warning threshold is exceeded.
                                                      //The overtemperature pre-warning flag is common for both bridges.
#define DRVSTATUS_OT                   (1 << 25)      //overtemperature flag
                                                      //1: Overtemperature limit has been reached. Drivers become disabled until otpw is also cleared due to cooling down of the IC.
                                                      //The overtemperature flag is common for both bridges.
#define DRVSTATUS_STALLGUARD           (1 << 24)      //stallGuard2 status
                                                      //1: Motor stall detected (SG_RESULT=0) or dcStep stall in dcStep mode.
#define DRVSTATUS_CS_ACTUAL            (31 << 16)     //actual motor current / smart energy current
                                                      //Actual current control scaling, for monitoring smart energy current scaling controlled via settings in register COOLCONF,
                                                      //or for monitoring the function of the automatic current scaling.
#define DRVSTATUS_FSACTIVE             (1 << 15)      //full step active indicator
                                                      //1: Indicates that the driver has switched to fullstep as defined by chopper mode settings and velocity thresholds.
#define DRVSTATUS_SG_RESULT            (1023 << 0)    //stallGuard2 result respectively PWM on time for coil A in stand still for motor temperature detection
                                                      //Mechanical load measurement: The stallGuard2 result gives a means to measure mechanical motor load.
                                                      //A higher value means lower mechanical load. A value of 0 signals highest load. With optimum SGT setting,
                                                      //this is an indicator for a motor stall. The stall detection compares SG_RESULT to 0 in order to detect a stall.
                                                      //SG_RESULT is used as a base for coolStep operation, by comparing it to a programmable upper and a lower limit. It is not applicable in stealthChop mode
                                                      //
                                                      //SG_RESULT is ALSO applicable when dcStep is active. stallGuard2 works best with microstep operation.
                                                      //
                                                      //Temperature measurement: In standstill, no stallGuard2 result can be obtained. SG_RESULT shows the chopper on-time for motor coil A instead.
                                                      //If the motor is moved to a determined microstep position at a certain current setting, a comparison of the chopper on-time can help to get a rough estimation of motor temperature.
                                                      //As the motor heats up, its coil resistance rises and the chopper on-time increases.

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define PWMCONF_REGISTER    0x70    // W         8       //Voltage PWM mode chopper configuration, reset default= 0x00050480
/********************************************************************************/
//         Function                    Bit            Description

#define PWMCONF_FREEWHEEL              (3 << 20)      //Allows different standstill modes
                                                      //Stand still option when motor current setting is zero (I_HOLD=0).
                                                      //%00: Normal operation
                                                      //%01: Freewheeling
                                                      //%10: Coil shorted using LS drivers
                                                      //%11: Coil shorted using HS drivers
#define PWMCONF_PWM_SYMMETRIC          (1 << 19)      //Force symmetric PWM
                                                      //0: The PWM value may change within each PWM cycle (standard mode)
                                                      //1: A symmetric PWM cycle is enforced
#define PWMCONF_PWM_AUTOSCALE          (1 << 18)      //PWM automatic amplitude scaling
                                                      //0: User defined PWM amplitude. The current settings have no influence.
                                                      //1: Enable automatic current control.
                                                      //Attention: When using a user defined sine wave table, the amplitude of this sine wave table should not be less than 244.
                                                      //Best results are obtained with 247 to 252 as peak values.
#define PWMCONF_PWM_FREQ               (1 << 16)      //PWM frequency selection
                                                      //%00: fPWM =2/1024 fCLK
                                                      //%01: fPWM =2/683 fCLK
                                                      //%10: fPWM =2/512 fCLK
                                                      //%11: fPWM =2/410 fCLK
#define PWMCONF_PWM_GRAD               (255 << 8)     //User defined amplitude (gradient) or regulation loop gradient
                                                      //pwm_autoscale=0: Velocity dependent gradient for PWM amplitude:
                                                      //PWM_GRAD * 256 / TSTEP is added to PWM_AMPL
                                                      //pwm_autoscale=1: User defined maximum PWM amplitude change per half wave (1 to 15)
#define PWMCONF_PWM_AMPL               (255 << 0)     //User defined amplitude (offset)
                                                      //pwm_autoscale=0: User defined PWM amplitude offset (0-255). The resulting amplitude (limited to 0...255) is:
                                                      //PWM_AMPL + PWM_GRAD * 256 / TSTEP
                                                      //pwm_autoscale=1: User defined maximum PWM amplitude when switching back from current chopper mode to voltage PWM mode (switch over velocity defined by TPWMTHRS).
                                                      //Do not set too low values, as the regulation cannot measure the current when the actual PWM value goes below a setting specific value.
                                                      //Settings above 0x40 recommended.

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define PWM_SCALE_REGISTER  0x71    // R         8       //Actual PWM amplitude scaler - (255=max. Voltage)
                                                         //In voltage mode PWM, this value allows to detect a motor stall.
                                                         //Range: 0...255
/********************************************************************************/

//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define ENCM_CTRL_REGISTER  0x72    // W         2       //Encoder mode configuration for a special mode (enc_commutation), not for normal use.
                                                         //The direction in this mode is determined by the shaft bit in GCONF or by the inv bit.
/********************************************************************************/
//         Function                    Bit               Description
#define ENCM_CTRL_INV                  (1 << 0)          //Invert encoder inputs
#define ENCM_CTRL_MAXSPEED             (1 << 1)          //Ignore Step input. If set, the hold current IHOLD determines the motor current, unless a step source is activated.


//         Register         Address    Type    N Bits    Description
/********************************************************************************/
#define LOST_STEPS_REGISTER 0x73    // R        20       //Number of input steps skipped due to higher load in dcStep operation, if step input does not stop when DC_OUT is low.
                                                         //This counter wraps around after 2^20 steps. Counts up or down depending on direction. Only with SDMODE=1.
/********************************************************************************/

/* SPECIAL REGISTER STATUS */

//This comes on every response command

//         Register         Address       Type    N Bits    Description
/********************************************************************************/
#define SPI_STATUS          0xFF000000ul  // R        8       //status flags transmitted with each SPI access in bits 39 to 32
/********************************************************************************/
//         Function                    Bit               Description
#define SPI_STATUS_STANDSTILL          1 << 3            //DRV_STATUS[31] – 1: Signals motor stand still
#define SPI_STATUS_SG2                 1 << 2            //DRV_STATUS[24] – 1: Signals stallguard flag active
#define SPI_STATUS_DRIVER_ERROR        1 << 1            //GSTAT[1]       – 1: Signals driver 1 driver error (clear by reading GSTAT)
#define SPI_STATUS_RESET_FLAG          1 << 0            //GSTAT[0]       – 1: Signals, that a reset has occurred (clear by reading GSTAT)

/*
 * Constructor
 */
TMC21X::TMC21X(std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi, char d) : spi(spi), designator(d)
{
    //we are not started yet
    //started = false;
    //by default cool step is not enabled
    //cool_step_enabled = false;
    //error_reported.reset();
}

/*
 * configure the stepper driver
 * just must be called.
 */
void TMC21X::init(uint16_t cs)
{
    //set the initial values
//    send2130(0x00,0x00000000);
//    send2130(0x80,0x00000000);
//    send2130(0xA0,0x00000000);
//    send2130(0xC0,0x00000000);
//    send2130(0xE0,0x00000000);

    //Reference
//    this->send2130(0x80,0x00000001UL); //voltage on AIN is current reference
//    this->send2130(0x90,0x00001010UL); //IHOLD=0x10, IRUN=0x10
//    this->send2130(0xEC,0x00008008UL); //native 256 microsteps, MRES=0, TBL=1=24, TOFF=8

    //Initialization example
//    this->send2130(0xEC,0x000100C3ul);    //CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
//    this->send2130(0x90,0x00061F0Aul);    //IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6
//    this->send2130(0x91,0x0000000Aul);    //TPOWERDOWN=10: Delay before power down in stand still
//    this->send2130(0x80,0x00000004ul);    //EN_PWM_MODE=1 enables stealthChop (with default PWM_CONF)
//    this->send2130(0x93,0x000001F4ul);    //TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM
//    this->send2130(0xF0,0x000401C8ul);    //PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amplitude limit=200, Grad=1

    //Other example
    send2130(0x6F,0x00000000);
    send2130(0x6F,0x00000000);
    send2130(0xEC,0x00ABCDEF);
    send2130(0xEC,0x00123456);

    //All read status
//    send2130(0x00,0x00000000);
//    send2130(0x01,0x00000000);
//    send2130(0x04,0x00000000);
//    send2130(0x12,0x00000000);
//    send2130(0x2D,0x00000000);
//    send2130(0x6A,0x00000000);
//    send2130(0x6B,0x00000000);
//    send2130(0x6C,0x00000000);
//    send2130(0x6F,0x00000000);
//    send2130(0x71,0x00000000);
//    send2130(0x73,0x00000000);
//    send2130(0x6F,0x00000000);
}

/*
 * send register settings to the stepper driver via SPI
 * returns the current status 40 bit datagram, first 8 bits is the status, the last 32 bits are the register contents
 */
void TMC21X::send2130(uint8_t reg, uint32_t datagram) //TODO Converted, needs testing
{
    uint8_t buf[5];

    //Note: SPI write first to last //TODO check that this is actually working as may have just swapped bytes and not the bit order
    buf[0] = (uint8_t)(reg);
    buf[1] = (uint8_t)(datagram >> 24);
    buf[2] = (uint8_t)(datagram >> 16);
    buf[3] = (uint8_t)(datagram >> 8);
    buf[4] = (uint8_t)(datagram >> 0);

    uint8_t rbuf[5];

    //write/read the values
    spi(buf, 5, rbuf);

    //print sent and received bytes
    THEKERNEL->streams->printf("sent: %02X, %02X, %02X, %02X, %02X received: %02X, %02X, %02X, %02X, %02X \n", buf[4], buf[3], buf[2], buf[1], buf[0], rbuf[4], rbuf[3], rbuf[2], rbuf[1], rbuf[0]);
}

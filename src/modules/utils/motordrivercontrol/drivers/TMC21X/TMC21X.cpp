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

#define motor_driver_control_checksum  CHECKSUM("motor_driver_control")
#define sense_resistor_checksum        CHECKSUM("sense_resistor")
#define chopper_mode_checksum          CHECKSUM("chopper_mode")

//! return value for TMC21X.getOverTemperature() if there is a overtemperature situation in the TMC chip
/*!
 * This warning indicates that the TCM chip is too warm.
 * It is still working but some parameters may be inferior.
 * You should do something against it.
 */
#define TMC21X_OVERTEMPERATURE_PREWARNING 1
//! return value for TMC21X.getOverTemperature() if there is a overtemperature shutdown in the TMC chip
/*!
 * This warning indicates that the TCM chip is too warm to operate and has shut down to prevent damage.
 * It will stop working until it cools down again.
 * If you encounter this situation you must do something against it. Like reducing the current or improving the PCB layout
 * and/or heat management.
 */
#define TMC21X_OVERTEMPERATURE_SHUTDOWN 2

//which values can be read out
/*!
 * Selects to readout the microstep position from the motor.
 *\sa readStatus()
 */
#define TMC21X_READOUT_MICROSTEP 0
/*!
 * Selects to read out the StallGuard value of the motor or the current setting (acc. to CoolStep) from the motor.
 *\sa readStatus()
 */
#define TMC21X_READOUT_STALLGUARD_CURRENT 1

//some default values used in initialization
#define DEFAULT_MICROSTEPPING_VALUE     32
#define ZEROS_DEFAULT_DATA              0x00000000
#define PWMCONF_DEFAULT_DATA            0x00050480

//driver access request (read MSB bit is 0 and write MSB bit is 1)
#define READ  0x00
#define WRITE 0x80

//TMC21X register definitions

//GENERAL CONFIGURATION REGISTERS (0x00...0x0F)

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define GCONF_REGISTER                 0x00      //RW      17        //Global configuration flags
/********************************************************************************/
//Function                             Bit                 Description
#define GCONF_I_SCALE_ANALOG           (1 << 0)            //0: Normal operation, use internal reference voltage
                                                           //1: Use voltage supplied to AIN as current reference
                                                           //
#define GCONF_INTERNAL_RSENSE          (1 << 1)            //0: Normal operation
                                                           //1: Internal sense resistors. Use current supplied into AIN as reference for internal sense resistor
                                                           //
#define GCONF_EN_PWM_MODE              (1 << 2)            //1: stealthChop voltage PWM mode enabled (depending on velocity thresholds). Switch from off to on state while in stand still, only.
                                                           //
#define GCONF_ENC_COMUTATION           (1 << 3)            //1: Enable commutation by full step encoder (DCIN_CFG5 = ENC_A, DCEN_CFG4 = ENC_B)
#define GCONF_SHAFT                    (1 << 4)            //1: Inverse motor direction
#define GCONF_DIAG0_ERROR              (1 << 5)            //1: Enable DIAG0 active on driver errors: Over temperature (ot), short to GND (s2g), undervoltage chargepump (uv_cp)
                                                           //DIAG0 always shows the reset-status, i.e. is active low during reset condition.
#define GCONF_DIAG0_OTPW               (1 << 6)            //1: Enable DIAG0 active on driver over temperature prewarning (otpw)
#define GCONF_DIAG0_STALL              (1 << 7)            //1: Enable DIAG0 active on motor stall (set TCOOLTHRS before using this feature)
#define GCONF_DIAG1_STALL              (1 << 8)            //1: Enable DIAG1 active on motor stall (set TCOOLTHRS before using this feature)
#define GCONF_DIAG1_INDEX              (1 << 9)            //1: Enable DIAG1 active on index position (microstep look up table position 0)
#define GCONF_DIAG1_ONSTATE            (1 << 10)           //1: Enable DIAG1 active when chopper is on (for the coil which is in the second half of the fullstep)
#define GCONF_DIAG1_STEPS_SKIPPED      (1 << 11)           //1: Enable output toggle when steps are skipped in dcStep mode (increment of LOST_STEPS). Do not enable in conjunction with other DIAG1 options.
#define GCONF_DIAG0_INT_PUSHPULL       (1 << 12)           //0: DIAG0 is open collector output (active low)
                                                           //1: Enable DIAG0 push pull output (active high)
#define GCONF_DIAG1_PUSHPULL           (1 << 13)           //0: DIAG1 is open collector output (active low)
                                                           //1: Enable DIAG1 push pull output (active high)
#define GCONF_SMALL_HYSTERESIS         (1 << 14)           //0: Hysteresis for step frequency comparison is 1/16
                                                           //1: Hysteresis for step frequency comparison is 1/32
#define GCONF_STOP_ENABLE              (1 << 15)           //0: Normal operation
                                                           //1: Emergency stop: DCIN stops the sequencer when tied high (no steps become executed by the sequencer, motor goes to standstill state).
#define GCONF_DIRECT_MODE              (1 << 16)           //0: Normal operation
                                                           //1: Motor coil currents and polarity directly programmed via serial interface:
                                                           //      Register XDIRECT (0x2D) specifies signed coil A current (bits 8..0) and coil B current (bits 24..16).
                                                           //      In this mode, the current is scaled by IHOLD setting.
                                                           //      Velocity based current regulation of stealthChop is not available in this mode.
                                                           //      The automatic stealthChop current regulation will work only for low stepper motor velocities.
#define GCONF_TEST_MODE                (1 << 17)           //0: Normal operation
                                                           //1: Enable analog test output on pin DCO. IHOLD[1..0] selects the function of DCO: 0…2: T120, DAC, VDDH
                                                           //Attention: Not for user, set to 0 for normal operation!

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define GSTAT_REGISTER                 0x01      //R+C     3         //Global status flags
/********************************************************************************/
//Function                             Bit                 Description
#define GSTAT_RESET                    (1 << 0)            //1: Indicates that the IC has been reset since the last read access to GSTAT.
#define GSTAT_DRV_ERR                  (1 << 1)            //1: Indicates, that the driver has been shut down due to over temperature or short circuit detection since the last read access.
                                                           //Read DRV_STATUS for details. The flag can only be reset when all error conditions are cleared.
#define GSTAT_UV_CP                    (1 << 2)            //1: Indicates an under voltage on the charge pump. The driver is disabled in this case.


//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define IOIN_REGISTER                  0x04      //R       8+8       //Reads the state of all input pins available
/********************************************************************************/
//Function                             Bit                 Description
#define IOIN_STEP                      (1 << 0)            //1: STEP pin high state
#define IOIN_DIR                       (1 << 1)            //1: DIR pin high state
#define IOIN_DCEN_CFG4                 (1 << 2)            //1: DCEN_CFG4 pin high state
#define IOIN_DCIN_CFG5                 (1 << 3)            //1: DCIN_CFG5 pin high state
#define IOIN_DRV_ENN_CFG6              (1 << 4)            //1: DRV_ENN_CFG6 pin high state
#define IOIN_DCO                       (1 << 5)            //1: DCO pin high state
#define IOIN_VERSION                   (255 << 24)         //0x11=first version of the IC
#define IOIN_VERSION_SHIFT             24                  //24 Identical numbers mean full digital compatibility. Bits 31..24 are the version of the IC

//VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0X10...0X1F)

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define IHOLD_IRUN_REGISTER            0x10      //W       5+5+4     //Driver current control
/********************************************************************************/
//Function                             Bit                 Description
#define IHOLD_IRUN_IHOLD               (31 << 0)           //Bits 4..0: Standstill current (0=1/32…31=32/32)
#define IHOLD_IRUN_IHOLD_SHIFT         0                   //In combination with stealthChop mode, setting IHOLD=0 allows to choose freewheeling or coil short circuit for motor stand still.
#define IHOLD_IRUN_IRUN                (31 << 8)           //Bits 12..8: Motor run current (0=1/32…31=32/32)
#define IHOLD_IRUN_IRUN_SHIFT          8                   //
                                                           //Hint: Choose sense resistors in a way, that normal IRUN is 16 to 31 for best microstep performance.
#define IHOLD_IRUN_IHOLDDELAY          (15 << 16)          //Bits 19..16: Controls the number of clock cycles for motor power down after a motion as soon as standstill is detected (stst=1) and TPOWERDOWN has expired.
#define IHOLD_IRUN_IHOLDDELAY_SHIFT    16                  //The smooth transition avoids a motor jerk upon power down.
                                                           //
                                                           //0: instant power down
                                                           //1..15: Delay per current reduction step in multiple of 2^18 clocks

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define TPOWERDOWN_REGISTER            0x11      //W       8         //Sets the delay time after stand still (stst) of the motor to motor current power down.
                                                                     //Time range is about 0 to 4 seconds. 0…((2^8)-1) * 2^18 tCLK
/********************************************************************************/

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define TSTEP_REGISTER                 0x12      //R       20        //Actual measured time between two 1/256 microsteps derived from the step input frequency.
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

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define TPWMTHRS_REGISTER              0x13      //W       20        //This is the upper velocity for stealthChop voltage PWM mode.
                                                                     //TSTEP ≥ TPWMTHRS
                                                                     //  - stealthChop PWM mode is enabled, if configured
                                                                     //  - dcStep is disabled
/********************************************************************************/

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define TCOOLTHRS_REGISTER             0x14      //W       20        //This is the lower threshold velocity for switching on smart energy coolStep and stallGuard feature.
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

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define THIGH_REGISTER                 0x15      //W       20        //This velocity setting allows velocity dependent switching into a different chopper mode and fullstepping to maximize torque. (unsigned)
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

//SPI MODE REGISTER (0X2D)

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define XDIRECT_REGISTER               0x2D      //RW      32        //SPI Mode Register
                                                                     //NOTE: This register cannot be used in STEP/DIR mode.
                                                                     //
                                                                     //Direct mode operation:
                                                                     //  0: Normal operation
                                                                     //  1: Directly SPI driven motor current
                                                                     //XDIRECT specifies Motor coil currents and polarity directly programmed via the serial interface. Use signed, two’s complement numbers.
                                                                     //
                                                                     //In this mode, the current is scaled by IHOLD setting.
                                                                     //Velocity based current regulation of voltage PWM is not available in this mode.
                                                                     //The automatic voltage PWM current regulation will work only for low stepper motor velocities.
                                                                     //dcStep is not available in this mode. coolStep and stallGuard only can be used, when additionally supplying a STEP signal.
                                                                     //This will also enable automatic current scaling.
/********************************************************************************/
//Function                             Bit                           Description
#define XDIRECT_COIL_A                 (511 << 0)                    //Coil A current (bits 8..0) (signed)
#define XDIRECT_COIL_A_SHIFT           0                             //Range: +-248 for normal operation, up to +-255 with stealthChop
#define XDIRECT_COIL_B                 (511 << 16)                   //Coil B current (bits 24..16) (signed)
#define XDIRECT_COIL_B_SHIFT           16                            //Range: +-248 for normal operation, up to +-255 with stealthChop

//DCSTEP MINIMUM VELOCITY REGISTER (0x33)

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define VDCMIN_REGISTER                0x33      //W       23        //dcStep Minimum Velocity Register
                                                                     //The automatic commutation dcStep becomes enabled by the external signal DCEN.
                                                                     //VDCMIN is used as the minimum step velocity when the motor is heavily loaded.
                                                                     //
                                                                     //Hint: Also set DCCTRL parameters in order to operate dcStep.
                                                                     //
                                                                     //NOTE: time reference t for VDCMIN: t = 2^24 / fCLK
/********************************************************************************/

//MICROSTEPPING CONTROL REGISTER SET (0X60...0X6B)

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define MSLUT0_REGISTER                0x60      //W       32        //Microstep table entries 0*31+0..31.
                                                                     //Range: 32 * 0 or 1 , reset default = sine wave table
#define MSLUT1_REGISTER                0x61      //W       32        //Microstep table entries 1*31+0..31. Range: 32 * 0 or 1 (reset default = sine wave table)
#define MSLUT2_REGISTER                0x62      //W       32        //Microstep table entries 2*31+0..31. Range: 32 * 0 or 1 (reset default = sine wave table)
#define MSLUT3_REGISTER                0x63      //W       32        //Microstep table entries 3*31+0..31. Range: 32 * 0 or 1 (reset default = sine wave table)
#define MSLUT4_REGISTER                0x64      //W       32        //Microstep table entries 4*31+0..31. Range: 32 * 0 or 1 (reset default = sine wave table)
#define MSLUT5_REGISTER                0x65      //W       32        //Microstep table entries 5*31+0..31. Range: 32 * 0 or 1 (reset default = sine wave table)
#define MSLUT6_REGISTER                0x66      //W       32        //Microstep table entries 6*31+0..31. Range: 32 * 0 or 1 (reset default = sine wave table)
#define MSLUT7_REGISTER                0x67      //W       32        //Microstep table entries 7*31+0..31.
                                                                     //Range: 7 * 32 * 0 or 1 , reset default = sine wave table
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

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define MSLUTSEL_REGISTER              0x68      //W       32        //This register defines four segments within each quarter MSLUT wave.
                                                                     //Four 2 bit entries determine the meaning of a 0 and a 1 bit in the corresponding segment of MSLUT.
                                                                     //Range: 0<X1<X2<X3, reset default = sine wave table.
/********************************************************************************/
//Function                             Bit                 Description
#define MSUTSEL_X3                     (255 << 24)         //LUT segment 3 start
#define MSUTSEL_X3_SHIFT               24                  //
#define MSUTSEL_X2                     (255 << 16)         //LUT segment 2 start
#define MSUTSEL_X2_SHIFT               16                  //
#define MSUTSEL_X1                     (255 << 8)          //LUT segment 1 start
#define MSUTSEL_X1_SHIFT               8                   //The sine wave look up table can be divided into up to four segments using an individual step width controlentry Wx.
                                                           //The segment borders are selected by X1, X2 and X3.
                                                           //
                                                           //Segment 0 goes from 0 to X1-1.
                                                           //Segment 1 goes from X1 to X2-1.
                                                           //Segment 2 goes from X2 to X3-1.
                                                           //Segment 3 goes from X3 to 255.
                                                           //
                                                           //For defined response the values shall satisfy: 0<X1<X2<X3
#define MSUTSEL_W3                     (3 << 7)            //LUT width select from ofs(X3) to ofs255
#define MSUTSEL_W3_SHIFT               7                   //
#define MSUTSEL_W2                     (3 << 5)            //LUT width select from ofs(X2) to ofs(X3-1)
#define MSUTSEL_W2_SHIFT               5                   //
#define MSUTSEL_W1                     (3 << 3)            //LUT width select from ofs(X1) to ofs(X2-1)
#define MSUTSEL_W1_SHIFT               3                   //
#define MSUTSEL_W0                     (3 << 1)            //LUT width select from ofs00 to ofs(X1-1)
#define MSUTSEL_W0_SHIFT               1                   //
                                                           //Width control bit coding W0...W3:
                                                           //%00: MSLUT entry 0, 1 select: -1, +0
                                                           //%01: MSLUT entry 0, 1 select: +0, +1
                                                           //%10: MSLUT entry 0, 1 select: +1, +2
                                                           //%11: MSLUT entry 0, 1 select: +2, +3

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define MSLUTSTART_REGISTER            0x69      //W       8+8       //bit 7… 0: START_SIN
                                                                     //bit 23… 16: START_SIN90
/********************************************************************************/
//Function                             Bit                 Description
#define MSLUTSTART_START_SIN           (255 << 0)          //START_SIN gives the absolute current at microstep table entry 0.
#define MSLUTSTART_START_SIN_SHIFT     0                   //Range: START_SIN, reset default =0
#define MSLUTSTART_START_SIN90         (255 << 8)          //START_SIN90 gives the absolute current for microstep table entry at positions 256.
#define MSLUTSTART_START_SIN90_SHIFT   8                   //Range: START_SIN 90, reset default =247
                                                           //
                                                           //Start values are transferred to the microstep registers CUR_A and CUR_B, whenever
                                                           //the reference position MSCNT=0 is passed.
/********************************************************************************/

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define MSCNT_REGISTER                 0x6A      //R       10        //Microstep counter.
                                                                     //Indicates actual position in the microstep table for CUR_A. CUR_B uses an offset of 256 (2 phase motor).
                                                                     //Hint: Move to a position where MSCNT is zero before re-initializing MSLUTSTART or MSLUT and MSLUTSEL.
                                                                     //Range: 0..123
/********************************************************************************/

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define MSCURACT_REGISTER              0x6B      //R       9+9       //bit 8… 0: CUR_A (signed):
                                                                     //bit 24… 16: CUR_B (signed):
/********************************************************************************/
//Function                             Bit                 Description
#define MSCURACT_CUR_A                 (511 << 0)          //Actual microstep current for motor phase A as read from MSLUT (not scaled by current)
#define MSCURACT_CUR_A_SHIFT           0                   //Range +/-0...255
#define MSCURACT_CUR_B                 (511 << 16)         //Actual microstep current for motor phase B as read from MSLUT (not scaled by current)
#define MSCURACT_CUR_B_SHIFT           16                  //Range +/-0...255

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define CHOPCONF_REGISTER              0x6C      //RW      32        //chopper and driver configuration
/********************************************************************************/
//Function                             Bit                 Description
#define CHOPCONF_DISS2G                (1 << 30)           //short to GND protection disable
                                                           //0: Short to GND protection is on
                                                           //1: Short to GND protection is disabled
#define CHOPCONF_DEDGE                 (1 << 29)           //enable souble edge step pulses
                                                           //1: Enable step impulse at each step edge to reduce step frequency requirement.
                                                           //1: Short to GND protection is disabled
#define CHOPCONF_INTPOL                (1 << 28)           //interpolation to 256 microsteps
                                                           //1: The actual microstep resolution (MRES) becomes extrapolated to 256 microsteps for smoothest motor operation.
#define CHOPCONF_MRES                  (15 << 24)          //micro step resolution
#define CHOPCONF_MRES_SHIFT            24                  //
                                                           //%0000: Native 256 microstep setting
                                                           //%0001 ... %1000: 128, 64, 32, 16, 8, 4, 2, FULLSTEP
                                                           //Reduced microstep resolution for STEP/DIR operation.
                                                           //The resolution gives the number of microstep entries per sine quarter wave.
                                                           //The driver automatically uses microstep positions which result in a symmetrical wave, when choosing a lower microstep resolution.
                                                           //step width=2^MRES [microsteps]
#define CHOPCONF_SYNC                  (15 << 20)          //PWM synchronization clock
#define CHOPCONF_SYNC_SHIFT            20                  //
                                                           //This register allows synchronization of the chopper for both phases of a two phase motor in order to avoid the
                                                           //occurrence of a beat, especially at low motor velocities. It is automatically switched off above VHIGH.
                                                           //%0000: Chopper sync function chopSync off
                                                           //%0001 ... %1111:
                                                           //Synchronization with fSYNC = fCLK/(sync*64)
                                                           //Hint: Set TOFF to a low value, so that the chopper cycle is ended, before the next sync clock pulse occurs.
                                                           //Set for the double desired chopper frequency for chm=0, for the desired base chopper frequency for chm=1.
#define CHOPCONF_VHIGHCHM              (1 << 19)           //high velocity chopper mode
                                                           //This bit enables switching to chm=1 and fd=0, when VHIGH is exceeded. This way, a higher velocity can be achieved.
                                                           //Can be combined with vhighfs=1. If set, the TOFF setting automatically becomes doubled during high velocity
                                                           //operation in order to avoid doubling of the chopper frequency.
#define CHOPCONF_VHIGHFS               (1 << 18)           //high velocity fullstep selection
                                                           //This bit enables switching to fullstep, when VHIGH is exceeded. Switching takes place only at 45° position.
                                                           //The fullstep target current uses the current value from the microstep table at the 45° position.
#define CHOPCONF_VSENSE                (1 << 17)           //sense resistor voltage based current scaling
                                                           //0: Low sensitivity, high sense resistor voltage
                                                           //1: High sensitivity, low sense resistor voltage
#define CHOPCONF_TBL                   (3 << 15)           //blank time select
#define CHOPCONF_TBL_SHIFT             15                  //
                                                           //%00 ... %11:
                                                           //Set comparator blank time to 16, 24, 36 or 54 clocks
                                                           //Hint: %01 or %10 is recommended for most applications
#define CHOPCONF_CHM                   (1 << 14)           //chopper mode
                                                           //0: Standard mode (spreadcycle) Low sensitivity, high sense resistor voltage
                                                           //1: Constant off time with fast decay time. Fast decay time is also terminated when the negative nominal current is reached. Fast decay is after on time.
#define CHOPCONF_RNDTF                 (1 << 13)           //random TOFF time
                                                           //0: Chopper off time is fixed as set by TOFF
                                                           //1: Random mode, TOFF is random modulated by dNCLK = -12 ... +3 clocks.
#define CHOPCONF_DISFDCC               (1 << 12)           //fast decay mode
                                                           //chm=1: disfdcc=1 disables current comparator usage for termination of the fast decay cycle.
#define CHOPCONF_FD3                   (1 << 11)           //TFD [3]
#define CHOPCONF_FD3_SHIFT             11                  //
                                                           //chm=1: MSB of fast decay time setting TFD.
#define CHOPCONF_HEND                  (15 << 7)           //CHM=0: HEND hysteresis low value, CHM=1 OFFSET sine wave offset
#define CHOPCONF_HEND_SHIFT            7                   //
                                                           //CHM=0
                                                           //%0000 ... %1111: Hysteresis is -3, -2, -1, 0, 1, ..., 12 (1/512 of this setting adds to current setting)
                                                           //This is the hysteresis value which becomes used for the hysteresis chopper.
                                                           //
                                                           //CHM=1
                                                           //%0000 ... %1111: Offset is -3, -2, -1, 0, 1, ..., 12
                                                           //This is the sine wave offset and 1/512 of the value becomes added to the absolute value of each sine wave entry.
#define CHOPCONF_HSTRT                 (7 << 4)            //CHM=0: HSTRT hysteresis start value added to HEND, CHM=1 TFD [2--0] fast decay time setting (MSB: fd3)
#define CHOPCONF_HSTRT_SHIFT           4                   //
                                                           //CHM=0
                                                           //%0000 ... %1111: Add 1, 2, ..., 8 to hysteresis low value HEND (1/512 of this setting adds to current setting)
                                                           //Attention: Effective HEND+HSTRT ≤ 16.
                                                           //Hint: Hysteresis decrement is done each 16 clocks
                                                           //
                                                           //CHM=1
                                                           //%0000 ... %1111: Offset is -3, -2, -1, 0, 1, ..., 12
                                                           //Fast decay time setting TFD with NCLK = 32*TFD (%0000: slow decay only)
#define CHOPCONF_TOFF                  (15 << 0)           //off time and driver enable
#define CHOPCONF_TOFF_SHIFT            0                   //
                                                           //Off time setting controls duration of slow decay phase
                                                           //NCLK = 12 + 32*TOFF
                                                           //%0000: Driver disable, all bridges off
                                                           //%0001: 1 – use only with TBL ≥ 2
                                                           //%0010 ... %1111: 2 ... 15

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define COOLCONF_REGISTER              0x6D      //W       25        //coolStep smart current control register and stallGuard2 configuration
/********************************************************************************/
//Function                             Bit                 Description
#define COOLCONF_SFILT                 (1 << 24)           //stallGuard2 filter enable
                                                           //0: Standard mode, high time resolution for stallGuard2
                                                           //1: Filtered mode, stallGuard2 signal updated for each four fullsteps (resp. six fullsteps for 3 phase motor) only to compensate for motor pole tolerances
#define COOLCONF_SGT                   (127 << 16)         //stallGuard2 threshold value
#define COOLCONF_SGT_SHIFT             16                  //
                                                           //0: Standard mode, high time resolution for stallGuard2
                                                           //This signed value controls stallGuard2 level for stall output and sets the optimum measurement range for readout.
                                                           //A lower value gives a higher sensitivity. Zero is the starting value working with most motors.
                                                           //-64 to +63: A higher value makes stallGuard2 less sensitive and requires more torque to indicate a stall.
#define COOLCONF_SEIMIN                (1 << 15)           //minimum current for smart current control
#define COOLCONF_SEIMIN_SHIFT          15                  //0: 1/2 of current setting (IRUN)
                                                           //1: 1/4 of current setting (IRUN)
#define COOLCONF_SEDN                  (3 << 13)           //current down step speed
#define COOLCONF_SEDN_SHIFT            13                  //
                                                           //%00: For each 32 stallGuard2 values decrease by one
                                                           //%01: For each 8 stallGuard2 values decrease by one
                                                           //%10: For each 2 stallGuard2 values decrease by one
                                                           //%11: For each stallGuard2 value decrease by one
#define COOLCONF_SEMAX                 (15 << 8)           //stallGuard2 hysteresis value for smart current control
#define COOLCONF_SEMAX_SHIFT           8                   //
                                                           //If the stallGuard2 result is equal to or above (SEMIN+SEMAX+1)*32, the motor current becomes decreased to save energy.
                                                           //%0000 ... %1111: 0 ... 15
#define COOLCONF_SEUP                  (3 << 5)            //current up step width
#define COOLCONF_SEUP_SHIFT            5                   //
                                                           //Current increment steps per measured stallGuard2 value
                                                           //%00 ... %11: 1, 2, 4, 8
#define COOLCONF_SEMIN                 (15 << 0)           //minimum stallGuard2 value for smart current control and smart current enable
#define COOLCONF_SEMIN_SHIFT           0                   //
                                                           //If the stallGuard2 result falls below SEMIN*32, the motor current becomes increased to reduce motor load angle.
                                                           //%0000: smart current control coolStep off
                                                           //%0001 ... %1111: 1 ... 15

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define DCCTRL_REGISTER                0x6E      //W       24        //dcStep (DC) automatic commutation configuration register (enable via pin DCEN or via VDCMIN):
                                                                     //bit 9… 0: DC_TIME: Upper PWM on time limit for commutation (DC_TIME * 1/fCLK). Set slightly above effective blank time TBL.
                                                                     //bit 23… 16: DC_SG: Max. PWM on time for step loss detection using dcStep stallGuard2 in dcStep mode. (DC_SG * 16/fCLK)
                                                                     //Set slightly higher than DC_TIME/16
                                                                     //0=disable
                                                                     //Attention: Using a higher microstep resolution or interpolated operation, dcStep delivers a better stallGuard signal.
                                                                     //DC_SG is also available above VHIGH if vhighfs is activated. For best result also set vhighchm.
/********************************************************************************/

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define DRV_STATUS_REGISTER            0x6F      //R       22        //stallGuard2 value and driver error flags
/********************************************************************************/
//Function                             Bit                 Description
#define DRV_STATUS_STST                (1 << 31)           //standstill indicator
                                                           //This flag indicates motor stand still in each operation mode. This occurs 2^20 clocks after the last step pulse.
#define DRV_STATUS_OLB                 (1 << 30)           //open load indicator phase B
#define DRV_STATUS_OLA                 (1 << 29)           //open load indicator phase A
                                                           //1: Open load detected on phase A or B.
                                                           //Hint: This is just an informative flag. The driver takes no action upon it.
                                                           //False detection may occur in fast motion and standstill. Check during slow motion, only.
#define DRV_STATUS_S2GB                (1 << 28)           //short to ground indicator phase B
#define DRV_STATUS_S2GA                (1 << 27)           //short to ground indicator phase A
                                                           //1: Short to GND detected on phase A or B. The driver becomes disabled.
                                                           //The flags stay active, until the driver is disabled by software (TOFF=0) or by the ENN input.
#define DRV_STATUS_OTPW                (1 << 26)           //overtemperature pre-warning flag
                                                           //1: Overtemperature pre-warning threshold is exceeded.
                                                           //The overtemperature pre-warning flag is common for both bridges.
#define DRV_STATUS_OT                  (1 << 25)           //overtemperature flag
                                                           //1: Overtemperature limit has been reached. Drivers become disabled until otpw is also cleared due to cooling down of the IC.
                                                           //The overtemperature flag is common for both bridges.
#define DRV_STATUS_STALLGUARD          (1 << 24)           //stallGuard2 status
                                                           //1: Motor stall detected (SG_RESULT=0) or dcStep stall in dcStep mode.
#define DRV_STATUS_CS_ACTUAL           (31 << 16)          //actual motor current / smart energy current
#define DRV_STATUS_CS_ACTUAL_SHIFT     16                  //
                                                           //Actual current control scaling, for monitoring smart energy current scaling controlled via settings in register COOLCONF,
                                                           //or for monitoring the function of the automatic current scaling.
#define DRV_STATUS_FSACTIVE            (1 << 15)           //full step active indicator
                                                           //1: Indicates that the driver has switched to fullstep as defined by chopper mode settings and velocity thresholds.
#define DRV_STATUS_SG_RESULT           (1023 << 0)         //stallGuard2 result respectively PWM on time for coil A in stand still for motor temperature detection
#define DRV_STATUS_SG_RESULT_SHIFT     0                   //
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

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define PWMCONF_REGISTER               0x70      //W       8         //Voltage PWM mode chopper configuration, reset default= 0x00050480
/********************************************************************************/
//Function                             Bit                 Description
#define PWMCONF_FREEWHEEL              (3 << 20)           //Allows different standstill modes
#define PWMCONF_FREEWHEEL_SHIFT        20                  //
                                                           //Stand still option when motor current setting is zero (I_HOLD=0).
                                                           //%00: Normal operation
                                                           //%01: Freewheeling
                                                           //%10: Coil shorted using LS drivers
                                                           //%11: Coil shorted using HS drivers
#define PWMCONF_PWM_SYMMETRIC          (1 << 19)           //Force symmetric PWM
                                                           //0: The PWM value may change within each PWM cycle (standard mode)
                                                           //1: A symmetric PWM cycle is enforced
#define PWMCONF_PWM_AUTOSCALE          (1 << 18)           //PWM automatic amplitude scaling
                                                           //0: User defined PWM amplitude. The current settings have no influence.
                                                           //1: Enable automatic current control.
                                                           //Attention: When using a user defined sine wave table, the amplitude of this sine wave table should not be less than 244.
                                                           //Best results are obtained with 247 to 252 as peak values.
#define PWMCONF_PWM_FREQ               (3 << 16)           //PWM frequency selection
#define PWMCONF_PWM_FREQ_SHIFT         16                  //%00: fPWM =2/1024 fCLK
                                                           //%01: fPWM =2/683 fCLK
                                                           //%10: fPWM =2/512 fCLK
                                                           //%11: fPWM =2/410 fCLK
#define PWMCONF_PWM_GRAD               (255 << 8)          //User defined amplitude (gradient) or regulation loop gradient
#define PWMCONF_PWM_GRAD_SHIFT         8                   //
                                                           //pwm_autoscale=0: Velocity dependent gradient for PWM amplitude:
                                                           //PWM_GRAD * 256 / TSTEP is added to PWM_AMPL
                                                           //pwm_autoscale=1: User defined maximum PWM amplitude change per half wave (1 to 15)
#define PWMCONF_PWM_AMPL               (255 << 0)          //User defined amplitude (offset)
#define PWMCONF_PWM_AMPL_SHIFT         0                   //
                                                           //pwm_autoscale=0: User defined PWM amplitude offset (0-255). The resulting amplitude (limited to 0...255) is:
                                                           //PWM_AMPL + PWM_GRAD * 256 / TSTEP
                                                           //pwm_autoscale=1: User defined maximum PWM amplitude when switching back from current chopper mode to voltage PWM mode (switch over velocity defined by TPWMTHRS).
                                                           //Do not set too low values, as the regulation cannot measure the current when the actual PWM value goes below a setting specific value.
                                                           //Settings above 0x40 recommended.

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define PWM_SCALE_REGISTER             0x71      //R       8         //Actual PWM amplitude scaler - (255=max. Voltage)
                                                                     //In voltage mode PWM, this value allows to detect a motor stall.
                                                                     //Range: 0...255
/********************************************************************************/

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define ENCM_CTRL_REGISTER             0x72      //W       2         //Encoder mode configuration for a special mode (enc_commutation), not for normal use.
                                                                     //The direction in this mode is determined by the shaft bit in GCONF or by the inv bit.
/********************************************************************************/
//Function                             Bit                           Description
#define ENCM_CTRL_INV                  (1 << 0)                      //Invert encoder inputs
#define ENCM_CTRL_MAXSPEED             (1 << 1)                      //Ignore Step input. If set, the hold current IHOLD determines the motor current, unless a step source is activated.


//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define LOST_STEPS_REGISTER            0x73      //R       20        //Number of input steps skipped due to higher load in dcStep operation, if step input does not stop when DC_OUT is low.
                                                                     //This counter wraps around after 2^20 steps. Counts up or down depending on direction. Only with SDMODE=1.
/********************************************************************************/

//SPECIAL REGISTER STATUS

//This comes on every response command

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define SPI_STATUS                     (15 << 0) //R       8         //status flags transmitted with each SPI access in bits 39 to 32
#define SPI_STATUS_SHIFT               0
/********************************************************************************/
//Function                             Bit                 Description
#define SPI_STATUS_STANDSTILL          (1 << 3)            //DRV_STATUS[31] – 1: Signals motor stand still
#define SPI_STATUS_SG2                 (1 << 2)            //DRV_STATUS[24] – 1: Signals stallguard flag active
#define SPI_STATUS_DRIVER_ERROR        (1 << 1)            //GSTAT[1]       – 1: Signals driver 1 driver error (clear by reading GSTAT)
#define SPI_STATUS_RESET_FLAG          (1 << 0)            //GSTAT[0]       – 1: Signals, that a reset has occurred (clear by reading GSTAT)

/*
 * Constructor
 */
TMC21X::TMC21X(std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi, char d) : spi(spi), designator(d)
{
	connection_method = StepstickParameters::SPI;
    max_current= 2000; //TMC2130 supports upto 2A.
    //we are not started yet
    started = false;
    //by default cool step is not enabled
    cool_step_enabled = false;
    error_reported.reset();
}

/*
 * configure the stepper driver
 * just must be called.
 */
void TMC21X::init(uint16_t cs)
{
    // read chip specific config entries
    this->resistor= THEKERNEL->config->value(motor_driver_control_checksum, cs, sense_resistor_checksum)->by_default(50)->as_number(); // in milliohms
    this->chopper_mode= THEKERNEL->config->value(motor_driver_control_checksum, cs, chopper_mode_checksum)->by_default(0)->as_int();

    //setting the default register values
    this->gconf_register_value = ZEROS_DEFAULT_DATA;
    this->ihold_irun_register_value = ZEROS_DEFAULT_DATA;
    this->tpowerdown_register_value = ZEROS_DEFAULT_DATA;
    this->tpwmthrs_register_value = ZEROS_DEFAULT_DATA;
    this->tcoolthrs_register_value = ZEROS_DEFAULT_DATA;
    this->thigh_register_value = ZEROS_DEFAULT_DATA;
    this->chopconf_register_value = ZEROS_DEFAULT_DATA;
    this->coolconf_register_value = ZEROS_DEFAULT_DATA;
    this->pwmconf_register_value = PWMCONF_DEFAULT_DATA;

    //set the initial values
    send2130(WRITE|GCONF_REGISTER, this->gconf_register_value);
    send2130(WRITE|IHOLD_IRUN_REGISTER, this->ihold_irun_register_value);
    send2130(WRITE|TPOWERDOWN_REGISTER, this->tpowerdown_register_value);
    send2130(WRITE|TPWMTHRS_REGISTER, this->tpwmthrs_register_value);
    send2130(WRITE|TCOOLTHRS_REGISTER, this->tcoolthrs_register_value);
    send2130(WRITE|THIGH_REGISTER, this->thigh_register_value);
    send2130(WRITE|CHOPCONF_REGISTER, this->chopconf_register_value);
    send2130(WRITE|COOLCONF_REGISTER, this->coolconf_register_value);
    send2130(WRITE|PWMCONF_REGISTER, this->pwmconf_register_value);

    started = true;

    /* set and configure chopper
     * 0 - stealthChop
     * 1 - spreadCycle
     * 2 - traditional constant off-time
     */
    switch(chopper_mode) {
    case 0:
        //enable StealthChop
        setStealthChopEnabled(true);
        
        //arguments order: freewheel, symmetric, autoscale, freq, grad, ampl
        //default StealthChop configuration
        setStealthChop(0,0,1,1,4,128);
        break;
    case 1:
        //enable SpreadCycle by disabling StealthChop function
        setStealthChopEnabled(false);
        
        //arguments order: constant_off_time, blank_time, hysteresis_start, hysteresis_end, hysteresis_decrement
        // openbuilds high torque nema23 3amps (2.8)
        //setSpreadCycleChopper(5, 36, 6, 0, 0);
        // for 1.5amp kysan @ 12v
        setSpreadCycleChopper(5, 54, 5, 0, 0);
        // for 4amp Nema24 @ 12v
        //setSpreadCycleChopper(5, 54, 4, 0, 0);

        //arguments order: stall_guard_threshold, stall_guard_filter_enabled
        // set stallguard to a conservative value so it doesn't trigger immediately
        setStallGuardThreshold(10, 1);
        break;
    case 2:
        //enable SpreadCycle by disabling StealthChop function
        setStealthChopEnabled(false);
        
        //arguments order: constant_off_time, blank_time, fast_decay_time_setting, sine_wave_offset, use_current_comparator
        //set constantofftimechopper to a conservative start value
        setConstantOffTimeChopper(7, 54, 13, 12, 1);

        //arguments order: stall_guard_threshold, stall_guard_filter_enabled
        // set stallguard to a conservative value so it doesn't trigger immediately
        setStallGuardThreshold(10, 1);
        break;
    }

    //start with driver disabled
    set_enable(false);

    //set a nice microstepping value
    set_microsteps(DEFAULT_MICROSTEPPING_VALUE);
}

void TMC21X::setGeneralConfiguration(bool i_scale_analog, bool internal_rsense, bool shaft, bool small_hysteresis)
{
    if (i_scale_analog) {
        //set voltage supplied to AIN as current reference
        gconf_register_value |= GCONF_I_SCALE_ANALOG;
    } else {
        //use internal reference voltage
        gconf_register_value &= ~(GCONF_I_SCALE_ANALOG);
    }

    if (internal_rsense) {
        //use internal sense resistors
        gconf_register_value |= GCONF_INTERNAL_RSENSE;
    } else {
        //use external sense resistors
        gconf_register_value &= ~(GCONF_INTERNAL_RSENSE);
    }

    if (shaft) {
        //invert motor direction
        gconf_register_value |= GCONF_SHAFT;
    } else {
        //normal operation
        gconf_register_value &= ~(GCONF_SHAFT);
    }

    if (small_hysteresis) {
        //hysteresis for step frequency comparison is 1/32
        gconf_register_value |= GCONF_SMALL_HYSTERESIS;
    } else {
        //hysteresis for step frequency comparison is 1/16
        gconf_register_value &= ~(GCONF_SMALL_HYSTERESIS);
    }

    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|GCONF_REGISTER, gconf_register_value);
    }
}

void TMC21X::setDiag0options(bool error, bool otpw, bool stall, bool pushpull)
{
    if (error) {
        //enable DIAG0 active on driver errors
        gconf_register_value |= GCONF_DIAG0_ERROR;
    } else {
        //disable DIAG0 active on driver errors
        gconf_register_value &= ~(GCONF_DIAG0_ERROR);
    }

    if (otpw) {
        //enable DIAG0 active on driver over temperature
        gconf_register_value |= GCONF_DIAG0_OTPW;
    } else {
        //disable DIAG0 active on driver over temperature
        gconf_register_value &= ~(GCONF_DIAG0_OTPW);
    }

    if (stall) {
        //enable DIAG0 active on motor stall
        gconf_register_value |= GCONF_DIAG0_STALL;
    } else {
        //disable DIAG0 active on motor stall
        gconf_register_value &= ~(GCONF_DIAG0_STALL);
    }

    if (pushpull) {
        //set DIAG0 to active high
        gconf_register_value |= GCONF_DIAG0_INT_PUSHPULL;
    } else {
        //set DIAG0 to active low
        gconf_register_value &= ~(GCONF_DIAG0_INT_PUSHPULL);
    }

    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|GCONF_REGISTER, gconf_register_value);
    }
}

void TMC21X::setDiag1options(bool stall, bool index, bool onstate, bool pushpull)
{
    if (stall) {
        //enable DIAG1 active on motor stall
        gconf_register_value |= GCONF_DIAG1_STALL;
    } else {
        //disable DIAG1 active on motor stall
        gconf_register_value &= ~(GCONF_DIAG1_STALL);
    }

    if (index) {
        //enable DIAG1 active on index position
        gconf_register_value |= GCONF_DIAG1_INDEX;
    } else {
        //disable DIAG1 active on index position
        gconf_register_value &= ~(GCONF_DIAG1_INDEX);
    }

    if (onstate) {
        //enable DIAG1 active when chopper is on
        gconf_register_value |= GCONF_DIAG1_ONSTATE;
    } else {
        //disable DIAG1 active when chopper is on
        gconf_register_value &= ~(GCONF_DIAG1_ONSTATE);
    }

    if (pushpull) {
        //set DIAG1 to active high
        gconf_register_value |= GCONF_DIAG1_PUSHPULL;
    } else {
        //disable DIAG1 active when chopper is on
        gconf_register_value &= ~(GCONF_DIAG1_PUSHPULL);
    }

    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|GCONF_REGISTER, gconf_register_value);
    }
}

/*
 * Set the number of microsteps per step.
 * 0,2,4,8,16,32,64,128,256 is supported
 * any value in between will be mapped to the next smaller value
 * 0 and 1 set the motor in full step mode
 */
int TMC21X::set_microsteps(int number_of_steps)
{
    long setting_pattern;
    //poor mans log
    if (number_of_steps >= 256) {
        setting_pattern = 0;
        microsteps = 256;
    } else if (number_of_steps >= 128) {
        setting_pattern = 1;
        microsteps = 128;
    } else if (number_of_steps >= 64) {
        setting_pattern = 2;
        microsteps = 64;
    } else if (number_of_steps >= 32) {
        setting_pattern = 3;
        microsteps = 32;
    } else if (number_of_steps >= 16) {
        setting_pattern = 4;
        microsteps = 16;
    } else if (number_of_steps >= 8) {
        setting_pattern = 5;
        microsteps = 8;
    } else if (number_of_steps >= 4) {
        setting_pattern = 6;
        microsteps = 4;
    } else if (number_of_steps >= 2) {
        setting_pattern = 7;
        microsteps = 2;
        //1 and 0 lead to full step
    } else if (number_of_steps <= 1) {
        setting_pattern = 8;
        microsteps = 1;
    }

    //delete the old value
    this->chopconf_register_value &= ~(CHOPCONF_MRES);
    //set the new value
    this->chopconf_register_value |= (setting_pattern << CHOPCONF_MRES_SHIFT);

    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|CHOPCONF_REGISTER, chopconf_register_value);
    }
    
    return microsteps;
}

/*
 * returns the effective number of microsteps at the moment
 */
int TMC21X::get_microsteps(void)
{
    return microsteps;
}

void TMC21X::setStepInterpolation(int8_t value)
{
    if (value) {
        chopconf_register_value |= CHOPCONF_INTPOL;
    } else {
        chopconf_register_value &= ~(CHOPCONF_INTPOL);
    }
    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|CHOPCONF_REGISTER, chopconf_register_value);
    }
}

void TMC21X::setDoubleEdge(int8_t value)
{
    if (value) {
        chopconf_register_value |= CHOPCONF_DEDGE;
    } else {
        chopconf_register_value &= ~(CHOPCONF_DEDGE);
    }
    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|CHOPCONF_REGISTER, chopconf_register_value);
    }
}

/*
 * constant_off_time: The off time setting controls the minimum chopper frequency.
 * For most applications an off time within the range of 5μs to 20μs will fit.
 *      2...15: off time setting
 *
 * blank_time: Selects the comparator blank time. This time needs to safely cover the switching event and the
 * duration of the ringing on the sense resistor. For
 *      0: min. setting 3: max. setting
 *
 * fast_decay_time_setting: Fast decay time setting. With CHM=1, these bits control the portion of fast decay for each chopper cycle.
 *      0: slow decay only
 *      1...15: duration of fast decay phase
 *
 * sine_wave_offset: Sine wave offset. With CHM=1, these bits control the sine wave offset.
 * A positive offset corrects for zero crossing error.
 *      -3..-1: negative offset 0: no offset 1...12: positive offset
 *
 * use_current_comparator: Selects usage of the current comparator for termination of the fast decay cycle.
 * If current comparator is enabled, it terminates the fast decay cycle in case the current
 * reaches a higher negative value than the actual positive value.
 *      1: enable comparator termination of fast decay cycle
 *      0: end by time only
 */
void TMC21X::setConstantOffTimeChopper(int8_t constant_off_time, int8_t blank_time, int8_t fast_decay_time_setting, int8_t sine_wave_offset, uint8_t use_current_comparator)
{
    //perform some sanity checks
    if (constant_off_time < 2) {
        constant_off_time = 2;
    } else if (constant_off_time > 15) {
        constant_off_time = 15;
    }
    //save the constant off time
    this->constant_off_time = constant_off_time;
    int8_t blank_value;
    //calculate the value acc to the clock cycles
    if (blank_time >= 54) {
        blank_value = 3;
    } else if (blank_time >= 36) {
        blank_value = 2;
    } else if (blank_time >= 24) {
        blank_value = 1;
    } else {
        blank_value = 0;
    }
    this->blank_time = blank_time;

    if (fast_decay_time_setting < 0) {
        fast_decay_time_setting = 0;
    } else if (fast_decay_time_setting > 15) {
        fast_decay_time_setting = 15;
    }
    if (sine_wave_offset < -3) {
        sine_wave_offset = -3;
    } else if (sine_wave_offset > 12) {
        sine_wave_offset = 12;
    }
    //shift the sine_wave_offset
    sine_wave_offset += 3;

    //calculate the register setting
    //first of all delete all the values for this
    chopconf_register_value &= ~(CHOPCONF_RNDTF | CHOPCONF_TBL | CHOPCONF_DISFDCC | CHOPCONF_FD3 | CHOPCONF_HEND | CHOPCONF_HSTRT | CHOPCONF_TOFF);
    //set the constant off pattern
    chopconf_register_value |= CHOPCONF_CHM;
    //set the blank timing value
    chopconf_register_value |= ((unsigned long)blank_value) << CHOPCONF_TBL_SHIFT;
    //setting the constant off time
    chopconf_register_value |= constant_off_time;
    //set the fast decay time
    //set msb
    chopconf_register_value |= (((unsigned long)(fast_decay_time_setting >> 3)) << CHOPCONF_FD3_SHIFT);
    //other bits
    chopconf_register_value |= (((unsigned long)(fast_decay_time_setting & 0x7)) << CHOPCONF_HSTRT_SHIFT);
    //set the sine wave offset
    chopconf_register_value |= (unsigned long)sine_wave_offset << CHOPCONF_HEND_SHIFT;
    //using the current comparator?
    if (!use_current_comparator) {
        chopconf_register_value |= CHOPCONF_DISFDCC;
    }
    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|CHOPCONF_REGISTER,chopconf_register_value);
    }
}

/*
 * constant_off_time: The off time setting controls the minimum chopper frequency.
 * For most applications an off time within the range of 5μs to 20μs will fit.
 *      2...15: off time setting
 *
 * blank_time: Selects the comparator blank time. This time needs to safely cover the switching event and the
 * duration of the ringing on the sense resistor. For
 *      0: min. setting 3: max. setting
 *
 * hysteresis_start: Hysteresis start setting. Please remark, that this value is an offset to the hysteresis end value HEND.
 *      1...8
 *
 * hysteresis_end: Hysteresis end setting. Sets the hysteresis end value after a number of decrements. Decrement interval time is controlled by HDEC.
 * The sum HSTRT+HEND must be <16. At a current setting CS of max. 30 (amplitude reduced to 240), the sum is not limited.
 *      -3..-1: negative HEND 0: zero HEND 1...12: positive HEND
 *
 * hysteresis_decrement: Hysteresis decrement setting. This setting determines the slope of the hysteresis during on time and during fast decay time.
 *      0: fast decrement 3: very slow decrement
 */

void TMC21X::setSpreadCycleChopper(int8_t constant_off_time, int8_t blank_time, int8_t hysteresis_start, int8_t hysteresis_end, int8_t hysteresis_decrement)
{
    h_start = hysteresis_start;
    h_end = hysteresis_end;
    h_decrement = hysteresis_decrement;
    this->blank_time = blank_time;

    //perform some sanity checks
    if (constant_off_time < 2) {
        constant_off_time = 2;
    } else if (constant_off_time > 15) {
        constant_off_time = 15;
    }
    //save the constant off time
    this->constant_off_time = constant_off_time;
    int8_t blank_value;
    //calculate the value acc to the clock cycles
    if (blank_time >= 54) {
        blank_value = 3;
    } else if (blank_time >= 36) {
        blank_value = 2;
    } else if (blank_time >= 24) {
        blank_value = 1;
    } else {
        blank_value = 0;
    }

    if (hysteresis_start < 1) {
        hysteresis_start = 1;
    } else if (hysteresis_start > 8) {
        hysteresis_start = 8;
    }
    hysteresis_start--;

    if (hysteresis_end < -3) {
        hysteresis_end = -3;
    } else if (hysteresis_end > 12) {
        hysteresis_end = 12;
    }
    //shift the hysteresis_end
    hysteresis_end += 3;

    if (hysteresis_decrement < 0) {
        hysteresis_decrement = 0;
    } else if (hysteresis_decrement > 3) {
        hysteresis_decrement = 3;
    }

    //first of all delete all the values for this
    chopconf_register_value &= ~(CHOPCONF_CHM | CHOPCONF_TBL | CHOPCONF_DISFDCC | CHOPCONF_FD3 | CHOPCONF_HEND | CHOPCONF_HSTRT | CHOPCONF_TOFF);
    //set the blank timing value
    chopconf_register_value |= ((unsigned long)blank_value) << CHOPCONF_TBL_SHIFT;
    //setting the constant off time
    chopconf_register_value |= constant_off_time;
    //set the hysteresis_start
    chopconf_register_value |= ((unsigned long)hysteresis_start) << CHOPCONF_HSTRT_SHIFT;
    //set the hysteresis end
    chopconf_register_value |= ((unsigned long)hysteresis_end) << CHOPCONF_HEND_SHIFT;
    //set the hystereis decrement
    chopconf_register_value |= ((unsigned long)hysteresis_decrement) << CHOPCONF_FD3_SHIFT;

    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|CHOPCONF_REGISTER,chopconf_register_value);
    }
}

/*
 * In a constant off time chopper scheme both coil choppers run freely, i.e. are not synchronized.
 * The frequency of each chopper mainly depends on the coil current and the position dependent motor coil inductivity, thus it depends on the microstep position.
 * With some motors a slightly audible beat can occur between the chopper frequencies, especially when they are near to each other. This typically occurs at a
 * few microstep positions within each quarter wave. This effect normally is not audible when compared to mechanical noise generated by ball bearings, etc.
 * Further factors which can cause a similar effect are a poor layout of sense resistor GND connection.
 * Hint: A common factor, which can cause motor noise, is a bad PCB layout causing coupling of both sense resistor voltages
 * (please refer to sense resistor layout hint in chapter 8.1).
 * In order to minimize the effect of a beat between both chopper frequencies, an internal random generator is provided.
 * It modulates the slow decay time setting when switched on by the RNDTF bit. The RNDTF feature further spreads the chopper spectrum,
 * reducing electromagnetic emission on single frequencies.
 */
void TMC21X::setRandomOffTime(int8_t value)
{
    if (value) {
        chopconf_register_value |= CHOPCONF_RNDTF;
    } else {
        chopconf_register_value &= ~(CHOPCONF_RNDTF);
    }
    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|CHOPCONF_REGISTER,chopconf_register_value);
    }
}

void TMC21X::setStealthChopEnabled(bool enable)
{
    if (enable) {
        gconf_register_value |= GCONF_EN_PWM_MODE;
    } else {
        gconf_register_value &= ~(GCONF_EN_PWM_MODE);
    }
    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|GCONF_REGISTER, gconf_register_value);
    }
}

void TMC21X::setStealthChop(uint8_t freewheel, bool symmetric, bool autoscale, uint8_t freq, uint8_t grad, uint8_t ampl)
{
    //perform some sanity checks
    if (freewheel > 3) {
        freewheel = 3;
    }
    if (freq > 3) {
        freq = 3;
    }

    //first of all delete all the values for this register
    pwmconf_register_value = 0;

    if (autoscale) {
        //set PWM automatic amplitude scaling
        pwmconf_register_value |= PWMCONF_PWM_AUTOSCALE;
    }

    if (symmetric) {
        //set PWM automatic gradient adaptation
        pwmconf_register_value |= PWMCONF_PWM_SYMMETRIC;
    }

    //set standstill mode
    pwmconf_register_value |= (freewheel << PWMCONF_FREEWHEEL_SHIFT);
    //set PWM frequency
    pwmconf_register_value |= (freq << PWMCONF_PWM_FREQ_SHIFT);
    //set user defined amplitude gradient
    pwmconf_register_value |= (grad << PWMCONF_PWM_GRAD_SHIFT);
    //set user defined amplitude offset
    pwmconf_register_value |= (ampl << PWMCONF_PWM_AMPL_SHIFT);

    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|PWMCONF_REGISTER,pwmconf_register_value);
    }
}

void TMC21X::setHolddelay(uint8_t value)
{
    //perform some sanity checks
    if (value > 15) {
        value = 15;
    }

    //delete the old value
    this->ihold_irun_register_value &= ~(IHOLD_IRUN_IHOLDDELAY);

    //save the new value
    this->ihold_irun_register_value = value << IHOLD_IRUN_IHOLDDELAY_SHIFT;

    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|IHOLD_IRUN_REGISTER,ihold_irun_register_value);
    }
}

void TMC21X::setPowerDowndelay(uint8_t value)
{
    //save the delay to the register variable
    this->tpowerdown_register_value = value;

    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|TPOWERDOWN_REGISTER,tpowerdown_register_value);
    }
}

void TMC21X::setStealthChopthreshold(uint32_t threshold)
{
    //perform some sanity checks
    if (threshold >= (1 << 20)) {
        threshold = (1 << 20) - 1;
    }

    //save the threshold value
    this->tpwmthrs_register_value = threshold;

    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|TPWMTHRS_REGISTER,tpwmthrs_register_value);
    }
}

void TMC21X::setConstantOffTimethreshold (uint32_t threshold, bool vhighchm, bool vhighfs)
{
    //perform some sanity checks
    if (threshold >= (1 << 20)) {
        threshold = (1 << 20) - 1;
    }

    //save the threshold value
    this->thigh_register_value = threshold;

    if (vhighchm) {
        this->chopconf_register_value |= CHOPCONF_VHIGHCHM;
    } else {
        this->chopconf_register_value &= ~(CHOPCONF_VHIGHCHM);
    }

    if (vhighfs) {
        this->chopconf_register_value |= CHOPCONF_VHIGHFS;
    } else {
        this->chopconf_register_value &= ~(CHOPCONF_VHIGHFS);
    }

    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|THIGH_REGISTER,thigh_register_value);
        send2130(WRITE|CHOPCONF_REGISTER,chopconf_register_value);
    }
}

void TMC21X::setCoolStepthreshold (uint32_t threshold)
{
    //perform some sanity checks
    if (threshold >= (1 << 20)) {
        threshold = (1 << 20) - 1;
    }

    //save the threshold value
    this->tcoolthrs_register_value = threshold;

    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|TCOOLTHRS_REGISTER,tcoolthrs_register_value);
    }
}

void TMC21X::set_current(uint16_t current)
{
    uint8_t current_scaling = 0;
    //calculate the current scaling from the max current setting (in mA)
    double mASetting = (double)current;
    double resistor_value = (double) this->resistor;
    // remove vsense flag
    this->chopconf_register_value &= ~(CHOPCONF_VSENSE);
    //this is derived from I(rms)=(CS+1)/32*(Vsense/Rsense)*1/sqrt(2)
    //for vsense = 0,32V (VSENSE not set)
    //or vsense = 0,18V (VSENSE set)
    // this is for I(rms) - but we use I(peak) everywhere
    //current_scaling = (uint8_t)((5.65685F * (resistor_value + 20) * mASetting / 1000.0F) / (125 * 0.32F) - 1);
    current_scaling = (uint8_t)(((resistor_value + 20) * mASetting * 32.0F / (0.32F * 1000.0F * 1000.0F)) - 0.5F); //theoretically - 1.0 for better rounding it is 0.5
    //THEKERNEL->streams->printf("Current - %d, Final CS - %d",current, (uint16_t) current_scaling);
    //check if the current scaling is too low
    if (current_scaling < 16) {
        //set the Vsense bit to get a use half the sense voltage (to support lower motor currents)
        this->chopconf_register_value |= CHOPCONF_VSENSE;
        //and recalculate the current setting
	// I(rms) calculation
        //current_scaling = (uint8_t)((5.65685F * (resistor_value + 20) * mASetting / 1000.0F) / (125 * 0.18F) - 1);
        current_scaling = (uint8_t)(((resistor_value + 20) * mASetting * 32.0F / (0.18F * 1000.0F * 1000.0F)) - 0.5F); //theoretically - 1.0 for better rounding it is 0.5
    }

    //do some sanity checks
    if (current_scaling > 31) {
        current_scaling = 31;
    }

    //delete the old value
    ihold_irun_register_value &= ~(IHOLD_IRUN_IRUN);
    //set the new current scaling
    ihold_irun_register_value  |= current_scaling << IHOLD_IRUN_IRUN_SHIFT;
    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|CHOPCONF_REGISTER,chopconf_register_value);
        send2130(WRITE|IHOLD_IRUN_REGISTER,ihold_irun_register_value);
    }
}

void TMC21X::setHoldCurrent(uint8_t hold)
{
    //hold current is passed as a percentage of run current. Use the existing current_scale to calcualte new IHOLD value.
    double current_scaling = (double)((ihold_irun_register_value & IHOLD_IRUN_IRUN) >> IHOLD_IRUN_IRUN_SHIFT);
    //delete the old value
    ihold_irun_register_value &= ~(IHOLD_IRUN_IHOLD);
    //set the new current scaling
    ihold_irun_register_value |= (uint8_t)(current_scaling * ((double) hold) / 100.0F) << IHOLD_IRUN_IHOLD_SHIFT;
    if (started) {
        send2130(WRITE|IHOLD_IRUN_REGISTER,ihold_irun_register_value);
    }
}

void TMC21X::setResistor(unsigned int value)
{
    this->resistor = value;
}

unsigned int TMC21X::get_current(void)
{
    //we calculate the current according to the datasheet to be on the safe side
    //this is not the fastest but the most accurate and illustrative way
    double result = (double)((ihold_irun_register_value & IHOLD_IRUN_IRUN) >> IHOLD_IRUN_IRUN_SHIFT);
    double resistor_value = (double)this->resistor;
    double voltage = (chopconf_register_value & CHOPCONF_VSENSE) ? 0.18F : 0.32F;
    // this would return I(rms), but we're working with I(peak)
    //result = (result + 1.0F) / 32.0F * voltage / ((resistor_value + 20)*0.001F) * 707.10678F;
    result = (result + 1.0F) / 32.0F * voltage / (resistor_value + 20) * 1000.0F * 1000.0F;
    return (unsigned int)result;
}

void TMC21X::setStallGuardThreshold(int8_t stall_guard_threshold, int8_t stall_guard_filter_enabled)
{
    if (stall_guard_threshold < -64) {
        stall_guard_threshold = -64;
        //We just have 5 bits
    } else if (stall_guard_threshold > 63) {
        stall_guard_threshold = 63;
    }
    //add trim down to 7 bits
    stall_guard_threshold &= 0x7f;
    //delete old stall guard settings
    coolconf_register_value &= ~(COOLCONF_SGT);
    if (stall_guard_filter_enabled) {
        coolconf_register_value |= COOLCONF_SFILT;
    }
    //Set the new stall guard threshold
    coolconf_register_value |= (((unsigned long)stall_guard_threshold << COOLCONF_SGT_SHIFT) & COOLCONF_SGT);

    //if started we directly send it to the motor
    if (started) {
        send2130(WRITE|COOLCONF_REGISTER,coolconf_register_value);
    }
}

int8_t TMC21X::getStallGuardThreshold(void)
{
    unsigned long stall_guard_threshold = coolconf_register_value & COOLCONF_SGT;
    //shift it down to bit 0
    stall_guard_threshold >>= COOLCONF_SGT_SHIFT;
    //convert the value to an int to correctly handle the negative numbers
    int8_t result = stall_guard_threshold;
    //check if it is negative and fill it up with leading 1 for proper negative number representation
    if (result & (1 << 6)) {
        result |= 0xC0;
    }
    return result;
}

int8_t TMC21X::getStallGuardFilter(void)
{
    if (coolconf_register_value & COOLCONF_SFILT) {
        return -1;
    } else {
        return 0;
    }
}

void TMC21X::setCoolStepConfiguration(unsigned int lower_SG_threshold, unsigned int SG_hysteresis, uint8_t current_decrement_step_size,
                                      uint8_t current_increment_step_size, uint8_t lower_current_limit)
{
    //sanitize the input values
    if (lower_SG_threshold > 480) {
        lower_SG_threshold = 480;
    }
    //divide by 32
    lower_SG_threshold >>= 5;
    if (SG_hysteresis > 480) {
        SG_hysteresis = 480;
    }
    //divide by 32
    SG_hysteresis >>= 5;
    if (current_decrement_step_size > 3) {
        current_decrement_step_size = 3;
    }
    if (current_increment_step_size > 3) {
        current_increment_step_size = 3;
    }
    if (lower_current_limit > 1) {
        lower_current_limit = 1;
    }
    //store the lower level in order to enable/disable the cool step
    this->cool_step_lower_threshold = lower_SG_threshold;
    //if cool step is not enabled we delete the lower value to keep it disabled
    if (!this->cool_step_enabled) {
        lower_SG_threshold = 0;
    }
    //the good news is that we can start with a complete new cool step register value
    //and simply set the values in the register
    coolconf_register_value = ((unsigned long)lower_SG_threshold) << COOLCONF_SEMIN_SHIFT | (((unsigned long)SG_hysteresis) << COOLCONF_SEMAX_SHIFT) | (((unsigned long)current_decrement_step_size) << COOLCONF_SEUP_SHIFT)
                               | (((unsigned long)current_increment_step_size) << COOLCONF_SEDN_SHIFT) | (((unsigned long)lower_current_limit) << COOLCONF_SEIMIN_SHIFT);

    if (started) {
        send2130(WRITE|COOLCONF_REGISTER,coolconf_register_value);
    }
}

void TMC21X::setCoolStepEnabled(bool enabled)
{
    //simply delete the lower limit to disable the cool step
    coolconf_register_value &= ~(COOLCONF_SEMIN);
    //and set it to the proper value if cool step is to be enabled
    if (enabled) {
        coolconf_register_value |= this->cool_step_lower_threshold;
    }
    //and save the enabled status
    this->cool_step_enabled = enabled;
    //save the register value
    if (started) {
        send2130(WRITE|COOLCONF_REGISTER,coolconf_register_value);
    }
}

bool TMC21X::isCoolStepEnabled(void)
{
    return this->cool_step_enabled;
}

unsigned int TMC21X::getCoolStepLowerSgThreshold()
{
    //we return our internally stored value - in order to provide the correct setting even if cool step is not enabled
    return ((this->cool_step_lower_threshold << COOLCONF_SEMIN_SHIFT) << 5);
}

unsigned int TMC21X::getCoolStepUpperSgThreshold()
{
    return (uint8_t)((coolconf_register_value & COOLCONF_SEMAX) >> COOLCONF_SEMAX_SHIFT) << 5;
}

uint8_t TMC21X::getCoolStepCurrentIncrementSize()
{
    return (uint8_t)((coolconf_register_value & COOLCONF_SEDN) >> COOLCONF_SEDN_SHIFT);
}

uint8_t TMC21X::getCoolStepNumberOfSGReadings()
{
    return (uint8_t)((coolconf_register_value & COOLCONF_SEUP) >> COOLCONF_SEUP_SHIFT);
}

uint8_t TMC21X::getCoolStepLowerCurrentLimit()
{
    return (uint8_t)((coolconf_register_value & COOLCONF_SEIMIN) >> COOLCONF_SEIMIN_SHIFT);
}

//reads the stall guard setting from last status
//returns -1 if stallguard information is not present
int TMC21X::getCurrentStallGuardReading(void)
{
    //if we don't yet started there cannot be a stall guard value
    if (!started) {
        return -1;
    }
    return ((readStatus(TMC21X_READOUT_STALLGUARD_CURRENT) & DRV_STATUS_SG_RESULT) >> DRV_STATUS_SG_RESULT_SHIFT);
}

uint8_t TMC21X::getCurrentCSReading(void)
{
    //if we don't yet started there cannot be a stall guard value
    if (!started) {
        return 0;
    }
    return ((readStatus(TMC21X_READOUT_STALLGUARD_CURRENT) & DRV_STATUS_CS_ACTUAL) >> DRV_STATUS_CS_ACTUAL_SHIFT);
}

bool TMC21X::isCurrentScalingHalfed()
{
    if (this->chopconf_register_value & CHOPCONF_VSENSE) {
        return true;
    } else {
        return false;
    }
}

unsigned int TMC21X::getCoolstepCurrent(void)
{
    float result = (float)getCurrentCSReading();
    float resistor_value = (float)this->resistor;
    float voltage = (chopconf_register_value & CHOPCONF_VSENSE) ? 0.18F : 0.32F;
    result = (result + 1.0F) / 32.0F * voltage / ((resistor_value + 20)*0.001F) * 707.10678F;
    return (unsigned int)roundf(result);
}

/*
 return true if the stallguard threshold has been reached
*/
bool TMC21X::isStallGuardOverThreshold(void)
{
    if (!this->started) {
        return false;
    }
    return (readStatus(TMC21X_READOUT_STALLGUARD_CURRENT) & DRV_STATUS_STALLGUARD);
}

/*
 returns if there is any over temperature condition:
 OVER_TEMPERATURE_PREWARNING if pre warning level has been reached
 OVER_TEMPERATURE_SHUTDOWN if the temperature is so hot that the driver is shut down
 Any of those levels are not too good.
*/
int8_t TMC21X::getOverTemperature(void)
{
    if (!this->started) {
        return 0;
    }
    if (readStatus(TMC21X_READOUT_STALLGUARD_CURRENT) & DRV_STATUS_OT) {
        return TMC21X_OVERTEMPERATURE_SHUTDOWN;
    }
    if (readStatus(TMC21X_READOUT_STALLGUARD_CURRENT) & DRV_STATUS_OTPW) {
        return TMC21X_OVERTEMPERATURE_PREWARNING;
    }
    return 0;
}

//is motor channel A shorted to ground
bool TMC21X::isShortToGroundA(void)
{
    if (!this->started) {
        return false;
    }
    return (readStatus(TMC21X_READOUT_STALLGUARD_CURRENT) & DRV_STATUS_S2GA);
}

//is motor channel B shorted to ground
bool TMC21X::isShortToGroundB(void)
{
    if (!this->started) {
        return false;
    }
    return (readStatus(TMC21X_READOUT_STALLGUARD_CURRENT) & DRV_STATUS_S2GB);
}

//is motor channel A connected
bool TMC21X::isOpenLoadA(void)
{
    if (!this->started) {
        return false;
    }
    return (readStatus(TMC21X_READOUT_STALLGUARD_CURRENT) & DRV_STATUS_OLA);
}

//is motor channel B connected
bool TMC21X::isOpenLoadB(void)
{
    if (!this->started) {
        return false;
    }
    return (readStatus(TMC21X_READOUT_STALLGUARD_CURRENT) & DRV_STATUS_OLB);
}

//is chopper inactive since 2^20 clock cycles - defaults to ~0,08s
bool TMC21X::isStandStill(void)
{
    if (!this->started) {
        return false;
    }
    return (readStatus(TMC21X_READOUT_STALLGUARD_CURRENT) & DRV_STATUS_STST);
}

//is chopper inactive since 2^20 clock cycles - defaults to ~0,08s
bool TMC21X::isStallGuardReached(void)
{
    if (!this->started) {
        return false;
    }
    return (readStatus(TMC21X_READOUT_STALLGUARD_CURRENT) & DRV_STATUS_STALLGUARD);
}

void TMC21X::set_enable(bool enabled)
{
    int8_t constant_off_time=this->constant_off_time;
    //perform some sanity checks
    if (constant_off_time < 2) {
        constant_off_time = 2;
    } else if (constant_off_time > 15) {
        constant_off_time = 15;
    }

    //save the constant off time
    this->constant_off_time = constant_off_time;

    //delete the t_off in the chopper config to get sure
    chopconf_register_value &= ~(CHOPCONF_TOFF);
    if (enabled) {
        //and set the t_off time
        chopconf_register_value |= this->constant_off_time;
    }
    //if not enabled we don't have to do anything since we already delete t_off from the register
    if (started) {
        send2130(WRITE|CHOPCONF_REGISTER, chopconf_register_value);
    }
}

bool TMC21X::isEnabled()
{
    if (chopconf_register_value & CHOPCONF_TOFF) {
        return true;
    } else {
        return false;
    }
}

//reads a value from the TMC21X status register.
uint32_t TMC21X::readStatus(int8_t read_value)
{
    uint32_t data;
    if(read_value == TMC21X_READOUT_MICROSTEP) {
        //we need to read access twice to obtain the actual read access response
        send2130(READ|MSCNT_REGISTER,ZEROS_DEFAULT_DATA);
        data = send2130(READ|MSCNT_REGISTER,ZEROS_DEFAULT_DATA);
    } else {
        //we need to read access twice to obtain the actual read access response
        send2130(READ|DRV_STATUS_REGISTER,ZEROS_DEFAULT_DATA);
        data = send2130(READ|DRV_STATUS_REGISTER,ZEROS_DEFAULT_DATA);
    }
    return data;
}

void TMC21X::dump_status(StreamOutput *stream)
{
    uint8_t actu= (designator >= 'X' && designator <= 'Z') ? designator-'X' : designator-'A'+3;
    
    stream->printf("designator %c, Chip type TMC21X\n", designator);

    check_error_status_bits(stream);

    if (this->isStallGuardReached()) {
        stream->printf("INFO: Stall Guard level reached!\n");
    }

    if (this->isStandStill()) {
        stream->printf("INFO: Motor is standing still.\n");
    }

    int value = readStatus(TMC21X_READOUT_MICROSTEP);
    stream->printf("Microstep position phase A: %d\n", value);

    value = getCurrentStallGuardReading();
    stream->printf("Stall Guard value: %d\n", value);

    stream->printf("Current setting: %dmA\n", get_current());
    stream->printf("Coolstep current: %dmA\n", getCoolstepCurrent());

    stream->printf("Microsteps: 1/%d\n", microsteps);

    stream->printf("Register dump:\n");
    stream->printf(" gconf register: %08lX (%ld)\n", gconf_register_value, gconf_register_value);
    stream->printf(" ihold_irun register: %08lX (%ld)\n", ihold_irun_register_value, ihold_irun_register_value);
    stream->printf(" tpowerdown register: %08lX (%ld)\n", tpowerdown_register_value, tpowerdown_register_value);
    stream->printf(" tpwmthrs register: %08lX (%ld)\n", tpwmthrs_register_value, tpwmthrs_register_value);
    stream->printf(" tcoolthrs register: %08lX (%ld)\n", tcoolthrs_register_value, tcoolthrs_register_value);
    stream->printf(" thigh register: %08lX (%ld)\n", thigh_register_value, thigh_register_value);
    stream->printf(" chopconf register: %08lX (%ld)\n", chopconf_register_value, chopconf_register_value);
    stream->printf(" coolconf register: %08lX (%ld)\n", coolconf_register_value, coolconf_register_value);
    stream->printf(" pwmconf register: %08lX (%ld)\n", pwmconf_register_value, pwmconf_register_value);
    stream->printf(" motor_driver_control.xxx.reg %05lX,%05lX,%05lX,%05lX,%05lX,%05lX,%05lX,%05lX,%05lX\n", gconf_register_value, ihold_irun_register_value, tpowerdown_register_value,
                                                                                                                      tpwmthrs_register_value, tcoolthrs_register_value, thigh_register_value,
                                                                                                                      chopconf_register_value, coolconf_register_value, pwmconf_register_value);
    
    bool moving = THEROBOT->actuators[actu]->is_moving();
    // dump out in the format that the processing script needs
    if (moving) {
        stream->printf("#sg%d,p%lu,k%u,r,", getCurrentStallGuardReading(), THEROBOT->actuators[actu]->get_current_step(), getCoolstepCurrent());
    } else {
        readStatus(TMC21X_READOUT_MICROSTEP); // get the status bits
        stream->printf("#s,");
    }
    stream->printf("d%d,", THEROBOT->actuators[actu]->which_direction() ? -1 : 1);
    stream->printf("c%u,m%d,", get_current(), get_microsteps());
    // stream->printf('S');
    // stream->printf(tmc21XStepper.getSpeed(), DEC);
    stream->printf("t%d,f%d,", getStallGuardThreshold(), getStallGuardFilter());

    //print out the general cool step config
    if (isCoolStepEnabled()) stream->printf("Ke+,");
    else stream->printf("Ke-,");

    stream->printf("Kl%u,Ku%u,Kn%u,Ki%u,Km%u,",
                   getCoolStepLowerSgThreshold(), getCoolStepUpperSgThreshold(), getCoolStepNumberOfSGReadings(), getCoolStepCurrentIncrementSize(), getCoolStepLowerCurrentLimit());

    //detect the winding status
    if (isOpenLoadA()) {
        stream->printf("ao,");
    } else if(isShortToGroundA()) {
        stream->printf("ag,");
    } else {
        stream->printf("a-,");
    }
    //detect the winding status
    if (isOpenLoadB()) {
        stream->printf("bo,");
    } else if(isShortToGroundB()) {
        stream->printf("bg,");
    } else {
        stream->printf("b-,");
    }

    char temperature = getOverTemperature();
    if (temperature == 0) {
        stream->printf("x-,");
    } else if (temperature == TMC21X_OVERTEMPERATURE_PREWARNING) {
        stream->printf("xw,");
    } else {
        stream->printf("xe,");
    }

    if (isEnabled()) {
        stream->printf("e1,");
    } else {
        stream->printf("e0,");
    }

    //write out the current chopper config
    stream->printf("Cm%d,", (chopconf_register_value & CHOPCONF_CHM) != 0);
    stream->printf("Co%d,Cb%d,", constant_off_time, blank_time);
    if ((chopconf_register_value & CHOPCONF_CHM) == 0) {
        stream->printf("Cs%d,Ce%d,Cd%d,", h_start, h_end, h_decrement);
    }
    stream->printf("\n");

}

// check error bits and report, only report once
bool TMC21X::check_error_status_bits(StreamOutput *stream)
{
    bool error= false;
    readStatus(TMC21X_READOUT_MICROSTEP); // get the status bits

    if (this->getOverTemperature()&TMC21X_OVERTEMPERATURE_PREWARNING) {
        if(!error_reported.test(0)) stream->printf("%c - WARNING: Overtemperature Prewarning!\n", designator);
        error_reported.set(0);
    }else{
        error_reported.reset(0);
    }

    if (this->getOverTemperature()&TMC21X_OVERTEMPERATURE_SHUTDOWN) {
        if(!error_reported.test(1)) stream->printf("%c - ERROR: Overtemperature Shutdown!\n", designator);
        error=true;
        error_reported.set(1);
    }else{
        error_reported.reset(1);
    }

    if (this->isShortToGroundA()) {
        if(!error_reported.test(2)) stream->printf("%c - ERROR: SHORT to ground on channel A!\n", designator);
        error=true;
        error_reported.set(2);
    }else{
        error_reported.reset(2);
    }

    if (this->isShortToGroundB()) {
        if(!error_reported.test(3)) stream->printf("%c - ERROR: SHORT to ground on channel B!\n", designator);
        error=true;
        error_reported.set(3);
    }else{
        error_reported.reset(3);
    }

    // these seem to be triggered when moving so ignore them for now
    if (this->isOpenLoadA()) {
        if(!error_reported.test(4)) stream->printf("%c - ERROR: Channel A seems to be unconnected!\n", designator);
        error=true;
        error_reported.set(4);
    }else{
        error_reported.reset(4);
    }

    if (this->isOpenLoadB()) {
        if(!error_reported.test(5)) stream->printf("%c - ERROR: Channel B seems to be unconnected!\n", designator);
        error=true;
        error_reported.set(5);
    }else{
        error_reported.reset(5);
    }

    return error;
}

bool TMC21X::check_alarm()
{
    return check_error_status_bits(THEKERNEL->streams);
}

// sets a raw register to the value specified, for advanced settings
// register 255 writes them, 0 displays what registers are mapped to what
bool TMC21X::set_raw_register(StreamOutput *stream, uint32_t reg, uint32_t val)
{
    switch(reg) {
        case 255:
            send2130(WRITE|GCONF_REGISTER, this->gconf_register_value);
            send2130(WRITE|IHOLD_IRUN_REGISTER, this->ihold_irun_register_value);
            send2130(WRITE|TPOWERDOWN_REGISTER, this->tpowerdown_register_value);
            send2130(WRITE|TPWMTHRS_REGISTER, this->tpwmthrs_register_value);
            send2130(WRITE|TCOOLTHRS_REGISTER, this->tcoolthrs_register_value);
            send2130(WRITE|THIGH_REGISTER, this->thigh_register_value);
            send2130(WRITE|CHOPCONF_REGISTER, this->chopconf_register_value);
            send2130(WRITE|COOLCONF_REGISTER, this->coolconf_register_value);
            send2130(WRITE|PWMCONF_REGISTER, this->pwmconf_register_value);
            stream->printf("Registers written\n");
            break;


        case 1: this->gconf_register_value = val; stream->printf("gconf register set to %08lX\n", val); break;
        case 2: this->ihold_irun_register_value = val; stream->printf("ihold irun config register set to %08lX\n", val); break;
        case 3: this->tpowerdown_register_value = val; stream->printf("tpowerdown config register set to %08lX\n", val); break;
        case 4: this->tpwmthrs_register_value = val; stream->printf("tpwmthrs config register set to %08lX\n", val); break;
        case 5: this->tcoolthrs_register_value = val; stream->printf("tcoolthrs config register set to %08lX\n", val); break;
        case 6: this->thigh_register_value = val; stream->printf("thigh config register set to %08lX\n", val); break;
        case 7: this->chopconf_register_value = val; stream->printf("chopconf register set to %08lX\n", val); break;
        case 8: this->coolconf_register_value = val; stream->printf("coolconf register set to %08lX\n", val); break;
        case 9: this->pwmconf_register_value = val; stream->printf("pwmconf register set to %08lX\n", val); break;

        default:
            stream->printf("1: gconf register\n");
            stream->printf("2: ihold irun register\n");
            stream->printf("3: tpowerdown_register\n");
            stream->printf("4: tpwmthrs_register\n");
            stream->printf("5: tcoolthrs_register\n");
            stream->printf("6: thigh_register\n");
            stream->printf("7: chopconf register\n");
            stream->printf("8: coolconf register\n");
            stream->printf("9: pwmconf register\n");
            stream->printf("255: update all registers\n");
            return false;
    }
    return true;
}

/*
 * send register settings to the stepper driver via SPI
 * returns last current status 32 bit datagram
 * sends 40bits, the first 8 bits is the register address and the last 32 bits is the register content
 */
uint32_t TMC21X::send2130(uint8_t reg, uint32_t datagram)
{
    uint8_t buf[] {(uint8_t)(reg), (uint8_t)(datagram >> 24), (uint8_t)(datagram >> 16), (uint8_t)(datagram >> 8), (uint8_t)(datagram >> 0)};
    uint8_t rbuf[5];

    //write/read the values
    spi(buf, 5, rbuf);

    //construct reply
    //first 8 bits are the SPI status bits, the last 32 bits are the current status bits
    spi_status_result = rbuf[0];
    uint32_t i_datagram = ((rbuf[1] << 24) | (rbuf[2] << 16) | (rbuf[3] << 8) | (rbuf[4] << 0));

    //THEKERNEL->streams->printf("sent: %02X, %02X, %02X, %02X, %02X received: %02X, %02X, %02X, %02X, %02X \n", buf[4], buf[3], buf[2], buf[1], buf[0], rbuf[4], rbuf[3], rbuf[2], rbuf[1], rbuf[0]);

    return i_datagram;
}

#define HAS(X) (options.find(X) != options.end())
#define GET(X) (options.at(X))
bool TMC21X::set_options(const options_t& options)
{
    bool set = false;
    if(HAS('O') || HAS('Q')) {
        //arguments order: stall_guard_threshold, stall_guard_filter_enabled
        int8_t o = HAS('O') ? GET('O') : getStallGuardThreshold();
        int8_t q = HAS('Q') ? GET('Q') : getStallGuardFilter();
        setStallGuardThreshold(o, q);
        set = true;
    }

    if(HAS('H') && HAS('I') && HAS('J') && HAS('K') && HAS('L')) {
        //arguments order: lower_SG_threshold, SG_hysteresis, current_decrement_step_size, current_increment_step_size, lower_current_limit
        setCoolStepConfiguration(GET('H'), GET('I'), GET('J'), GET('K'), GET('L'));
        set = true;
    }

    if(HAS('S')) {
        uint32_t s = GET('S');
        if(s == 0 && HAS('U') && HAS('V') && HAS('W') && HAS('X') && HAS('Y')) {
            //arguments order: constant_off_time, blank_time, fast_decay_time_setting, sine_wave_offset, use_current_comparator
            setConstantOffTimeChopper(GET('U'), GET('V'), GET('W'), GET('X'), GET('Y'));
            set = true;

        } else if(s == 1 && HAS('U') && HAS('V') && HAS('W') && HAS('X') && HAS('Y')) {
            //arguments order: constant_off_time, blank_time, hysteresis_start, hysteresis_end, hysteresis_decrement
            setSpreadCycleChopper(GET('U'), GET('V'), GET('W'), GET('X'), GET('Y'));
            set = true;

        } else if(s == 2 && HAS('Z')) {
            setRandomOffTime(GET('Z'));
            set = true;

        } else if(s == 3 && HAS('Z')) {
            setDoubleEdge(GET('Z'));
            set = true;

        } else if(s == 4 && HAS('Z')) {
            setStepInterpolation(GET('Z'));
            set = true;

        } else if(s == 5 && HAS('Z')) {
            setCoolStepEnabled(GET('Z') == 1);
            set = true;

        } else if(s == 6 && HAS('Z')) {
            setStealthChopEnabled(GET('Z'));
            set = true;

        } else if(s == 7 && HAS('U') && HAS('V') && HAS('W') && HAS('X') && HAS('Y') && HAS('Z')) {
            //arguments order: freewheel, symmetric, autoscale, freq, grad, ampl
            setStealthChop(GET('U'), GET('V'), GET('W'), GET('X'), GET('Y'), GET('Z'));
            set = true;

        } else if(s == 8 && HAS('Z')) {
            setStealthChopthreshold(GET('Z'));
            set = true;

        } else if(s == 9 && HAS('Z')) {
            setCoolStepthreshold(GET('Z'));
            set = true;

        } else if(s == 10 && HAS('U') && HAS('V') && HAS('W')) {
            //arguments order: threshold, vhighchm, vhighfs
            setConstantOffTimethreshold(GET('U'), GET('V'), GET('W'));
            set = true;

        } else if(s == 11 && HAS('Z')) {
            setResistor(GET('Z'));
            set = true;

        } else if(s == 12 && HAS('Z')) {
            setHoldCurrent(GET('Z'));
            set = true;

        } else if(s == 13 && HAS('Z')) {
            setHolddelay(GET('Z'));
            set = true;

        } else if(s == 14 && HAS('Z')) {
            setPowerDowndelay(GET('Z'));
            set = true;

        } else if(s == 15 && HAS('U') && HAS('V') && HAS('W') && HAS('X')) {
            //arguments order: i_scale_analog, internal_rsense, shaft, small_hysteresis
            setGeneralConfiguration(GET('U'), GET('V'), GET('W'), GET('X'));
            set = true;

        } else if(s == 16 && HAS('U') && HAS('V') && HAS('W') && HAS('X')) {
            //arguments order: error, otpw, stall, pushpull
            setDiag0options(GET('U'), GET('V'), GET('W'), GET('X'));
            set = true;

        } else if(s == 17 && HAS('U') && HAS('V') && HAS('W') && HAS('X')) {
            //arguments order: stall, index, onstate, pushpull
            setDiag1options(GET('U'), GET('V'), GET('W'), GET('X'));
            set = true;
        }
    }

    return set;
}
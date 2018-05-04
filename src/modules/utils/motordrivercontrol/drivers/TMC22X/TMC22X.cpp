/*
 Highly modifed from....

 TMC26X.cpp - - TMC26X Stepper library for Wiring/Arduino

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

#include "TMC22X.h"
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
#define DEFAULT_MICROSTEPPING_VALUE 32
#define DEFAULT_DATA 0x00000000

//driver access request (read MSB bit is 0 and write MSB bit is 1)
#define READ  0x00
#define WRITE 0x80

//sync and address of the target driver
#define SYNC        0x05
#define SLAVEADDR   0x00

//TMC21X register definitions

//GENERAL CONFIGURATION REGISTERS (0x00...0x0F)

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define GCONF_REGISTER                 0x00      //RW      10        //Global configuration flags
/********************************************************************************/
//Function                             Bit                 Description
#define GCONF_I_SCALE_ANALOG           (1 << 0)            //0: Use internal reference derived from 5VOUT
                                                           //1: Use voltage supplied to VREF as current reference
                                                           //Reset default=1
#define GCONF_INTERNAL_RSENSE          (1 << 1)            //0: Operation with external sense resistors
                                                           //1: Internal sense resistors. Use current supplied into VREF as reference for internal sense resistor. VREF pin internally is driven to GND in this mode.
                                                           //Reset default: OTP
#define GCONF_EN_SPREADCYCLE           (1 << 2)            //0: stealthChop PWM mode enabled (depending on velocity thresholds). Initially switch from off to on state while in stand still, only.
                                                           //1: spreadCycle mode enabled
                                                           //A high level on the pin SPREAD (TMC222x, only) inverts this flag to switch between both chopper modes.
                                                           //Reset default: OTP
#define GCONF_SHAFT                    (1 << 3)            //1: Inverse motor direction
#define GCONF_INDEX_OTPW               (1 << 4)            //0: INDEX shows the first microstep position of sequencer.
                                                           //1: INDEX pin outputs overtemperature prewarning flag (otpw) instead
#define GCONF_INDEX_STEP               (1 << 5)            //0: INDEX output as selected by index_otpw
                                                           //1: INDEX output shows step pulses from internal pulse generator (toggle upon each step)
#define GCONF_PDN_DISABLE              (1 << 6)            //0: PDN_UART controls standstill current reduction
                                                           //1: PDN_UART input function disabled. Set this bit, when using the UART interface!
#define GCONF_MSTEP_REG_SELECT         (1 << 7)            //0: Microstep resolution selected by pins MS1, MS2
                                                           //1: Microstep resolution selected by MSTEP register
#define GCONF_MULTISTEP_FILT           (1 << 8)            //0: No filtering of STEP pulses
                                                           //1: Software pulse generator optimization enabled when fullstep frequency > 750Hz (roughly). TSTEP shows filtered step time values when active.
                                                           //Reset default=1
#define GCONF_TEST_MODE                (1 << 9)            //0: Normal operation
                                                           //1: Enable analog test output on pin ENN (pull down resistor off), ENN treated as enabled. IHOLD[1..0] selects the function of DCO: 0…2: T120, DAC, VDDH
                                                           //Attention: Not for user, set to 0 for normal operation!

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define GSTAT_REGISTER                 0x01      //R+WC    3         //Global status flags (Re-Write with ‘1’ bit to clear respective flags)
/********************************************************************************/
//Function                             Bit                 Description
#define GSTAT_RESET                    (1 << 0)            //1: Indicates that the IC has been reset since the last read access to GSTAT.
#define GSTAT_DRV_ERR                  (1 << 1)            //1: Indicates, that the driver has been shut down due to over temperature or short circuit detection since the last read access.
                                                           //Read DRV_STATUS for details. The flag can only be reset when all error conditions are cleared.
#define GSTAT_UV_CP                    (1 << 2)            //1: Indicates an under voltage on the charge pump. The driver is disabled in this case. This flag is not latched and thus does not need to be cleared.

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define IFCNT_REGISTER                 0x02      //R       8         //Interface transmission counter. This register becomes incremented with each successful UART interface write access.
                                                                     //Read out to check the serial transmission for lost data. Read accesses do not change the content. The counter wraps around from 255 to 0.
/********************************************************************************/

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define SLAVECONF_REGISTER             0x03      //W       4         //SENDDELAY for read access (time until reply is sent):
/********************************************************************************/
//Function                             Bit                 Description
#define SLAVECONF_SENDDELAY            (15 << 8)           //0, 1: 8 bit times
#define SLAVECONF_SENDDELAY_SHIFT      8                   //2, 3: 3*8 bit times
                                                           //4, 5: 5*8 bit times
                                                           //6, 7: 7*8 bit times
                                                           //8, 9: 9*8 bit times
                                                           //10, 11: 11*8 bit times
                                                           //12, 13: 13*8 bit times
                                                           //14, 15: 15*8 bit times

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define OTP_PROG_REGISTER              0x04      //W       16       //OTP_PROGRAM – OTP programming
                                                                    //Write access programs OTP memory (one bit at a time),
                                                                    //Read access refreshes read data from OTP after a write
/********************************************************************************/
//Function                             Bit                 Description
#define OTP_PROG_OTPBIT                (7 << 0)            //Selection of OTP bit to be programmed to the selected byte location (n=0..7: programs bit n to a logic 1)
#define OTP_PROG_OTPBIT_SHIFT          0                   //
#define OTP_PROG_OTPBYTE               (3 << 4)            //Selection of OTP programming location (0, 1 or 2)
#define OTP_PROG_OTPBYTE_SHIFT         4                   //
#define OTP_PROG_OTPMAGIC              (255 << 8)          //Set to 0xbd to enable programming. A programming time of minimum 10ms per bit is recommended (check by reading OTP_READ).
#define OTP_PROG_OTPMAGIC_SHIFT        8                   //

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define OTP_READ_REGISTER              0x05      //R       24        //OTP_READ (Access to OTP memory result and update)
                                                                     //7..0: OTP0 byte 0 read data
                                                                     //15..8: OTP1 byte 1 read data
                                                                     //23..16: OTP2 byte 2 read data
/********************************************************************************/
//Function                             Bit                 Description
#define OTP_READ_EN_SPREADCYCLE        (1 << 23)           //otp2.7: This flag determines if the driver defaults to spreadCycle or to stealthChop.
                                                           //0: Default: stealthChop (GCONF.en_spreadCycle=0)
                                                           //   OTP 1.0 to 1.7 and 2.0 used for stealthChop.
                                                           //   spreadCycle settings: HEND=0; HSTART=5; TOFF=3
                                                           //1: Default: spreadCycle (GCONF.en_spreadCycle=1)
                                                           //   OTP 1.0 to 1.7 and 2.0 used for spreadCycle
                                                           //   stealthChop settings: PWM_GRAD=0; TPWM_THRS=0; PWM_OFS=36; pwm_autograd=1
#define OTP_READ_IHOLD                 (3 << 21)           //otp2.6, otp2.5: Reset default for standstill current IHOLD (used only if current reduction enabled, e.g. pin PDN_UART low).
#define OTP_READ_IHOLD_SHIFT           21                  //%00: IHOLD= 16 (53% of IRUN)
                                                           //%01: IHOLD= 2  ( 9% of IRUN)
                                                           //%10: IHOLD= 8  (28% of IRUN)
                                                           //%11: IHOLD= 24 (78% of IRUN)
                                                           //Reset default for run current IRUN=31
#define OTP_READ_IHOLDDELAY            (3 << 19)           //otp2.4, otp2.3: Reset default for IHOLDDELAY
#define OTP_READ_IHOLDDELAY_SHIFT      19                  //%00: IHOLDDELAY= 1
                                                           //%01: IHOLDDELAY= 2
                                                           //%10: IHOLDDELAY= 4
                                                           //%11: IHOLDDELAY= 8
#define OTP_READ_PWM_FREQ              (1 << 18)           //otp2.2: Reset default for PWM_FREQ:
                                                           //0: PWM_FREQ=%01=2/683
                                                           //1: PWM_FREQ=%10=2/512
#define OTP_READ_PWM_REG               (1 << 17)           //otp2.1: Reset default for PWM_REG:
                                                           //0: PWM_REG=%1000: max. 4 increments / cycle
                                                           //1: PWM_REG=%0010: max. 1 increment / cycle
#define OTP_READ_PWM_OFS               (1 << 16)           //otp2.0: Depending on otp_en_spreadCycle:
                                                           //0: otp_en_spreadCycle = 0: PWM_OFS=36
                                                           //   otp_en_spreadCycle = 1: PWM_OFS=00 (no feed forward scaling); pwm_autograd=0
                                                           //1: Reset default for CHOPCONF.8 (hend1)
#define OTP_READ_TPWMTHRS              (7 << 13)           //otp1.7, otp1.6, otp1.5: Depending on otp_en_spreadCycle:
#define OTP_READ_TPWMTHRS_SHIFT        13                  //0: Reset default for TPWM_THRS as defined by (0..7):
                                                           //   otp_en_spreadCycle = 0: TPWM_THRS = 0
                                                           //   otp_en_spreadCycle = 1: TPWM_THRS = 200
                                                           //   otp_en_spreadCycle = 2: TPWM_THRS = 300
                                                           //   otp_en_spreadCycle = 3: TPWM_THRS = 400
                                                           //   otp_en_spreadCycle = 4: TPWM_THRS = 500
                                                           //   otp_en_spreadCycle = 5: TPWM_THRS = 800
                                                           //   otp_en_spreadCycle = 6: TPWM_THRS = 1200
                                                           //   otp_en_spreadCycle = 7: TPWM_THRS = 4000
                                                           //1: Reset default for CHOPCONF.5 to CHOPCONF.7 (hstrt1, hstrt2 and hend0)
#define OTP_READ_PWM_AUTOGRAD          (1 << 12)           //otp1.4: Depending on otp_en_spreadCycle;
                                                           //0: otp_en_spreadCycle = 0: pwm_autograd=1
                                                           //   otp_en_spreadCycle = 1: pwm_autograd=0
                                                           //1: Reset default for CHOPCONF.4 (hstrt0); (pwm_autograd=1)
#define OTP_READ_PWM_GRAD              (15 << 8)           //otp1.3, otp1.2, otp1.1, otp1.0: Depending on otp_en_spreadCycle:
#define OTP_READ_PWM_GRAD_SHIFT        8                   //0: Reset default for PWM_GRAD as defined by (0..15):
                                                           //   otp_en_spreadCycle = 0: PWM_GRAD= 14
                                                           //   otp_en_spreadCycle = 1: PWM_GRAD= 16
                                                           //   otp_en_spreadCycle = 2: PWM_GRAD= 18
                                                           //   otp_en_spreadCycle = 3: PWM_GRAD= 21
                                                           //   otp_en_spreadCycle = 4: PWM_GRAD= 24
                                                           //   otp_en_spreadCycle = 5: PWM_GRAD= 27
                                                           //   otp_en_spreadCycle = 6: PWM_GRAD= 31
                                                           //   otp_en_spreadCycle = 7: PWM_GRAD= 35
                                                           //   otp_en_spreadCycle = 8: PWM_GRAD= 40
                                                           //   otp_en_spreadCycle = 9: PWM_GRAD= 46
                                                           //   otp_en_spreadCycle = 10: PWM_GRAD= 52
                                                           //   otp_en_spreadCycle = 11: PWM_GRAD= 59
                                                           //   otp_en_spreadCycle = 12: PWM_GRAD= 67
                                                           //   otp_en_spreadCycle = 13: PWM_GRAD= 77
                                                           //   otp_en_spreadCycle = 14: PWM_GRAD= 88
                                                           //   otp_en_spreadCycle = 15: PWM_GRAD= 100
                                                           //1: Reset default for CHOPCONF.0 to CHOPCONF.3 (TOFF)
#define OTP_READ_TBL                   (1 << 7)            //otp0.7: Reset default for TBL:
                                                           //0: TBL=%10
                                                           //1: TBL=%01
#define OTP_READ_INTERNALRSENSE        (1 << 6)            //otp0.6: Reset default for GCONF.internal_Rsense
                                                           //0: External sense resistors
                                                           //1: Internal sense resistors
#define OTP_READ_OTTRIM                (1 << 5)            //otp0.5: Reset default for OTTRIM:
                                                           //0: OTTRIM= %00 (143°C)
                                                           //1: OTTRIM= %01 (150°C)
                                                           //internal power stage temperature about 10°C above the sensor temperature limit
#define OTP_READ_FCLKTRIM              (31 << 0)           //otp0.4, otp0.3, otp0.2, otp0.1, otp0.0: Reset default for FCLKTRIM
#define OTP_READ_FCLKTRIM_SHIFT        0                   //0: lowest frequency setting
                                                           //31: highest frequency setting
                                                           //Attention: This value is pre-programmed by factory clock trimming to the default clock frequency of 12MHz and differs between individual ICs! It should not be altered.

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define IOIN_REGISTER                  0x06      //R       10+8      //Reads the state of all input pins available
/********************************************************************************/
//Function                             Bit                 Description
#define IOIN_ENN_TMC220X               (1 << 0)            //1: ENN pin high state for TMC220X driver
#define IOIN_MS1_TMC220X               (1 << 2)            //1: MS1 pin high state for TMC220X driver
#define IOIN_MS2_TMC220X               (1 << 3)            //1: MS2 pin high state for TMC220X driver
#define IOIN_DIAG_TMC220X              (1 << 4)            //1: DIAG pin high state for TMC220X driver
#define IOIN_PDN_UART_TMC220X          (1 << 6)            //1: PDN_UART pin high state for TMC220X driver
#define IOIN_STEP_TMC220X              (1 << 7)            //1: STEP pin high state for TMC220X driver
#define IOIN_DIR_TMC220X               (1 << 9)            //1: DIR pin high state for TMC220X driver
                                                           //
#define IOIN_PDN_UART_TMC222X          (1 << 1)            //1: PDN_UART pin high state for TMC222X driver
#define IOIN_SPREAD_TMC222X            (1 << 2)            //1: SPREAD pin high state for TMC222X driver
#define IOIN_DIR_TMC222X               (1 << 3)            //1: DIR pin high state for TMC222X driver
#define IOIN_ENN_TMC222X               (1 << 4)            //1: ENN pin high state for TMC222X driver
#define IOIN_STEP_TMC222X              (1 << 5)            //1: STEP pin high state for TMC222X driver
#define IOIN_MS1_TMC222X               (1 << 6)            //1: MS1 pin high state for TMC222X driver
#define IOIN_MS2_TMC222X               (1 << 7)            //1: MS2 pin high state for TMC222X driver
                                                           //
#define IOIN_SEL_A                     (1 << 8)            //SEL_A: Driver type
                                                           //1: TMC220x
                                                           //0: TMC222x
#define IOIN_VERSION                   (255 << 24)         //0x20=first version of the IC
#define IOIN_VERSION_SHIFT             24                  //24 Identical numbers mean full digital compatibility. Bits 31..24 are the version of the IC

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define FACTORY_CONF_REGISTER          0x07      //RW      5+2       //Factory configuration
/********************************************************************************/
//Function                             Bit                 Description
#define FACTORY_CONF_FCLKTRIM          (31 << 0)           //0...31: Lowest to highest clock frequency. Check at charge pump output. The frequency span is not guaranteed, but it is tested, that tuning to 12MHz internal clock is possible.
#define FACTORY_CONF_FCLKTRIM_SHIFT    0                   //The devices come preset to 12MHz clock frequency by OTP programming.
                                                           //Reset default: OTP
#define FACTORY_CONF_OTTRIM            (3 << 8)            //%00: OT=143°C, OTPW=120°C
#define FACTORY_CONF_OTTRIM_SHIFT      8                   //%01: OT=150°C, OTPW=120°C
                                                           //%10: OT=150°C, OTPW=143°C
                                                           //%11: OT=157°C, OTPW=143°C
                                                           //Default: OTP

//VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0X10...0X1F)

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define IHOLD_IRUN_REGISTER            0x10      //W       5+5+4     //Driver current control
/********************************************************************************/
//Function                             Bit                 Description
#define IHOLD_IRUN_IHOLD               (31 << 0)           //Standstill current (0=1/32…31=32/32)
#define IHOLD_IRUN_IHOLD_SHIFT         0                   //In combination with stealthChop mode, setting IHOLD=0 allows to choose freewheeling or coil short circuit for motor stand still.
                                                           //Reset default: OTP
#define IHOLD_IRUN_IRUN                (31 << 8)           //Motor run current (0=1/32…31=32/32)
#define IHOLD_IRUN_IRUN_SHIFT          8                   //Hint: Choose sense resistors in a way, that normal IRUN is 16 to 31 for best microstep performance.
                                                           //Reset default=31
#define IHOLD_IRUN_IHOLDDELAY          (15 << 16)          //Controls the number of clock cycles for motor power down after standstill is detected (stst=1) and TPOWERDOWN has expired.
#define IHOLD_IRUN_IHOLDDELAY_SHIFT    16                  //The smooth transition avoids a motor jerk upon power down.
                                                           //0: instant power down
                                                           //1..15: Delay per current reduction step in multiple of 2^18 clocks
                                                           //Reset default: OTP

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define TPOWERDOWN_REGISTER            0x11      //W       8         //Sets the delay time from stand still (stst) detection to motor current power down.
                                                                     //Time range is about 0 to 5.6 seconds. 0…((2^8)-1) * 2^18 tCLK
                                                                     //Attention: A minimum setting of 2 is required to allow automatic tuning of stealthChop PWM_OFFS_AUTO.
/********************************************************************************/

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define TSTEP_REGISTER                 0x12      //R       20        //Actual measured time between two 1/256 microsteps derived from the step input frequency.
                                                                     //In units of 1/fCLK. Measured value is (2^20)-1 in case of overflow or stand still.
                                                                     //
                                                                     //All TSTEP related thresholds use a hysteresis of 1/16 of the compare value to compensate for jitter in the clock or the step frequency.
                                                                     //The flag small_hysteresis modifies the hysteresis to a smaller value of 1/32.
                                                                     //(Txxx*15/16)-1 is the lower compare value for each TSTEP based comparison.
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
#define TPWMTHRS_REGISTER              0x13      //W       20        //Sets the upper velocity for stealthChop voltage PWM mode.
                                                                     //TSTEP ≥ TPWMTHRS
                                                                     //  - stealthChop PWM mode is enabled, if configured
                                                                     //When the velocity exceeds the limit set by TPWMTHRS, the driver switches to spreadCycle.
                                                                     //0: Disabled
/********************************************************************************/

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define VACTUAL_REGISTER               0x22      //W       24        //Microstep counter.
                                                                     //VACTUAL allows moving the motor by UART control. It gives the motor velocity in +-(2^23)-1 [μsteps / t]
                                                                     //0: Normal operation. Driver reacts to STEP input.
                                                                     ///=0: Motor moves with the velocity given by VACTUAL. Step pulses can be monitored via INDEX output. The motor direction is controlled by the sign of VACTUAL.
/********************************************************************************/

//MICROSTEPPING CONTROL REGISTER SET (0X60...0X6B)

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define MSCNT_REGISTER                 0x6A      //R       10        //Microstep counter.
                                                                     //Indicates actual position in the microstep table for CUR_A. CUR_B uses an offset of 256 into the table.
                                                                     //Reading out MSCNT allows determination of the motor position within the electrical wave.
                                                                     //Range: 0..123
/********************************************************************************/

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define MSCURACT_REGISTER              0x6B      //R       9+9       //bit 8… 0: CUR_A (signed):
                                                                     //bit 24… 16: CUR_B (signed):
/********************************************************************************/
//Function                             Bit                 Description
#define MSCURACT_CUR_A                 (511 << 0)          //Actual microstep current for motor phase A as read from the internal sine wave table (not scaled by current)
#define MSCURACT_CUR_A_SHIFT           0                   //Range +/-0...255
#define MSCURACT_CUR_B                 (511 << 16)         //Actual microstep current for motor phase B as read from the internal sine wave table (not scaled by current)
#define MSCURACT_CUR_B_SHIFT           16                  //Range +/-0...255

//DRIVER REGISTER SET (0X6C...0X7F)

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define CHOPCONF_REGISTER              0x6C      //RW      32        //chopper and driver configuration
                                                                     //Reset default = 0x10000053
/********************************************************************************/
//Function                             Bit                 Description
#define CHOPCONF_DISS2VS               (1 << 31)           //Low side short protection disable
                                                           //0: Short protection low side is on
                                                           //1: Short protection low side is disabled
#define CHOPCONF_DISS2G                (1 << 30)           //short to GND protection disable
                                                           //0: Short to GND protection is on
                                                           //1: Short to GND protection is disabled
#define CHOPCONF_DEDGE                 (1 << 29)           //Enable step impulse at each step edge to reduce step frequency requirement. This mode is not compatible with the step filtering function (multistep_filt)
#define CHOPCONF_INTPOL                (1 << 28)           //interpolation to 256 microsteps
                                                           //1: The actual microstep resolution (MRES) becomes extrapolated to 256 microsteps for smoothest motor operation.
                                                           //Default: 1
#define CHOPCONF_MRES                  (15 << 24)          //micro step resolution
#define CHOPCONF_MRES_SHIFT            24                  //
                                                           //%0000: Native 256 microstep setting
                                                           //%0001 ... %1000: 128, 64, 32, 16, 8, 4, 2, FULLSTEP
                                                           //Reduced microstep resolution.
                                                           //The resolution gives the number of microstep entries per sine quarter wave.
                                                           //When choosing a lower microstep resolution, the driver automatically uses microstep positions which result in a symmetrical wave.
                                                           //Number of microsteps per step pulse = 2^MRES
                                                           //Selection by pins unless disabled by GCONF. mstep_reg_select
#define CHOPCONF_VSENSE                (1 << 17)           //sense resistor voltage based current scaling
                                                           //0: Low sensitivity, high sense resistor voltage
                                                           //1: High sensitivity, low sense resistor voltage
#define CHOPCONF_TBL                   (3 << 15)           //blank time select
#define CHOPCONF_TBL_SHIFT             15                  //%00 ... %11:
                                                           //Set comparator blank time to 16, 24, 32 or 40 clocks
                                                           //Hint: %00 or %01 is recommended for most applications
                                                           //Default: OTP
#define CHOPCONF_HEND                  (15 << 7)           //HEND hysteresis low value, OFFSET sine wave offset
#define CHOPCONF_HEND_SHIFT            7                   //
                                                           //CHM=0
                                                           //%0000 ... %1111: Hysteresis is -3, -2, -1, 0, 1, ..., 12 (1/512 of this setting adds to current setting)
                                                           //This is the hysteresis value which becomes used for the hysteresis chopper.
                                                           //Default: OTP, resp. 5 in stealthChop mode
#define CHOPCONF_HSTRT                 (7 << 4)            //HSTRT hysteresis start value added to HEND
#define CHOPCONF_HSTRT_SHIFT           4                   //%0000 ... %1111: Add 1, 2, ..., 8 to hysteresis low value HEND (1/512 of this setting adds to current setting)
                                                           //Attention: Effective HEND+HSTRT ≤ 16.
                                                           //Hint: Hysteresis decrement is done each 16 clocks
#define CHOPCONF_TOFF                  (15 << 0)           //off time and driver enable
#define CHOPCONF_TOFF_SHIFT            0                   //Off time setting controls duration of slow decay phase
                                                           //NCLK = 12 + 32*TOFF
                                                           //%0000: Driver disable, all bridges off
                                                           //%0001: 1 – use only with TBL ≥ 2
                                                           //%0010 ... %1111: 2 ... 15
                                                           //Default: OTP, resp. 3 in stealthChop mode

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define DRV_STATUS_REGISTER            0x6F      //R       32        //Driver status flags and current level read back
/********************************************************************************/
//Function                             Bit                 Description
#define DRV_STATUS_STST                (1 << 31)           //standstill indicator
                                                           //This flag indicates motor stand still in each operation mode. This occurs 2^20 clocks after the last step pulse.
#define DRV_STATUS_STEALTH             (1 << 30)           //stealthChop indicator
                                                           //1: Driver operates in stealthChop mode
                                                           //0: Driver operates in spreadCycle mode
#define DRV_STATUS_CS_ACTUAL           (31 << 16)          //actual motor current / smart energy current
#define DRV_STATUS_CS_ACTUAL_SHIFT     16                  //Actual current control scaling, for monitoring the function of the automatic current scaling.
#define DRV_STATUS_T157                (1 << 11)           //157°C comparator
                                                           //1: Temperature threshold is exceeded
#define DRV_STATUS_T150                (1 << 10)           //150°C comparator
                                                           //1: Temperature threshold is exceeded
#define DRV_STATUS_T143                (1 << 9)            //143°C comparator
                                                           //1: Temperature threshold is exceeded
#define DRV_STATUS_T120                (1 << 8)            //120°C comparator
                                                           //1: Temperature threshold is exceeded
#define DRV_STATUS_OLB                 (1 << 7)            //open load indicator phase B
#define DRV_STATUS_OLA                 (1 << 6)            //open load indicator phase A
                                                           //1: Open load detected on phase A or B.
                                                           //Hint: This is just an informative flag. The driver takes no action upon it.
                                                           //False detection may occur in fast motion and standstill. Check during slow motion, only.
#define DRV_STATUS_S2VSB               (1 << 5)            //short to ground indicator phase B
#define DRV_STATUS_S2VSA               (1 << 4)            //short to ground indicator phase A
                                                           //1: The driver becomes disabled. The flags stay active, until the driver is disabled by software (TOFF=0) or by the ENN input.
                                                           //Flags are separate for both chopper modes.
#define DRV_STATUS_S2GB                (1 << 3)            //short to ground indicator phase B
#define DRV_STATUS_S2GA                (1 << 2)            //short to ground indicator phase A
                                                           //1: The driver becomes disabled. The flags stay active, until the driver is disabled by software (TOFF=0) or by the ENN input.
                                                           //Flags are separate for both chopper modes.
#define DRV_STATUS_OT                  (1 << 1)            //overtemperature flag
                                                           //1: The selected overtemperature limit has been reached. Drivers become disabled until otpw is also cleared due to cooling down of the IC.
                                                           //The overtemperature flag is common for both bridges.
#define DRV_STATUS_OTPW                (1 << 0)            //overtemperature pre-warning flag
                                                           //1: 1: The selected overtemperature pre-warning threshold is exceeded.
                                                           //The overtemperature pre-warning flag is common for both bridges.

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define PWMCONF_REGISTER               0x70      //RW      22        //stealthChop PWM chopper configuration
                                                                     //Reset default= 0xC10D0024
/********************************************************************************/
//Function                             Bit                 Description
#define PWMCONF_PWM_LIM                (15 << 28)          //PWM automatic scale amplitude limit when switching on
#define PWMCONF_PWM_LIM_SHIFT          28                  //Limit for PWM_SCALE_AUTO when switching back from spreadCycle to stealthChop. This value defines the upper limit for bits 7 to 4 of the automatic current control when switching back.
                                                           //It can be set to reduce the current jerk during mode change back to stealthChop. It does not limit PWM_GRAD or PWM_GRAD_AUTO offset.
                                                           //Default = 12
#define PWMCONF_PWM_REG                (15 << 24)          //Regulation loop gradient
#define PWMCONF_PWM_REG_SHIFT          24                  //User defined maximum PWM amplitude change per half wave when using pwm_autoscale=1. (1...15):
                                                           //1: 0.5 increments (slowest regulation)
                                                           //2: 1 increment (default with OTP2.1=1)
                                                           //3: 1.5 increments
                                                           //4: 2 increments
                                                           //...
                                                           //8: 4 increments (default with OTP2.1=0)
                                                           //...
                                                           //15: 7.5 increments (fastest regulation)
#define PWMCONF_FREEWHEEL              (3 << 20)           //Allows different standstill modes
#define PWMCONF_FREEWHEEL_SHIFT        20                  //
                                                           //Stand still option when motor current setting is zero (I_HOLD=0).
                                                           //%00: Normal operation
                                                           //%01: Freewheeling
                                                           //%10: Coil shorted using LS drivers
                                                           //%11: Coil shorted using HS drivers
#define PWMCONF_PWM_AUTOGRAD           (1 << 19)           //PWM automatic gradient adaptation
                                                           //0: Fixed value for PWM_GRAD (PWM_GRAD_AUTO = PWM_GRAD)
                                                           //1: A symmetric PWM cycle is enforced
                                                           //Automatic tuning (only with pwm_autoscale=1)
                                                           //PWM_GRAD_AUTO is initialized with PWM_GRAD and becomes optimized automatically during motion.
                                                           //Preconditions
                                                           //  1. PWM_OFS_AUTO has been automatically initialized. This requires standstill at IRUN for >130ms in order to
                                                           //      a) detect standstill
                                                           //      b) wait > 128 chopper cycles at IRUN
                                                           //      c) regulate PWM_OFS_AUTO so that -1 < PWM_SCALE_AUTO < 1
                                                           //  2. Motor running and 1.5 * PWM_OFS_AUTO < PWM_SCALE_SUM < 4* PWM_OFS_AUTO and PWM_SCALE_SUM < 255.
                                                           //Time required for tuning PWM_GRAD_AUTO
                                                           //About 8 fullsteps per change of +/-1.
#define PWMCONF_PWM_AUTOSCALE          (1 << 18)           //PWM automatic amplitude scaling
                                                           //0: User defined feed forward PWM amplitude. The current settings IRUN and IHOLD have no influence!
                                                           //The resulting PWM amplitude (limited to 0...255) is:
                                                           //PWM_OFS * ((CS_ACTUAL+1) / 32) + PWM_GRAD * 256 / TSTEP
                                                           //1: Enable automatic current control (Reset default)
#define PWMCONF_PWM_FREQ               (3 << 16)           //PWM frequency selection
#define PWMCONF_PWM_FREQ_SHIFT         16                  //%00: fPWM =2/1024 fCLK
                                                           //%01: fPWM =2/683 fCLK
                                                           //%10: fPWM =2/512 fCLK
                                                           //%11: fPWM =2/410 fCLK
#define PWMCONF_PWM_GRAD               (255 << 8)          //User defined amplitude gradient
#define PWMCONF_PWM_GRAD_SHIFT         8                   //Velocity dependent gradient for PWM amplitude:
                                                           //PWM_GRAD * 256 / TSTEP
                                                           //This value is added to PWM_AMPL to compensate for the velocity-dependent motor back-EMF.
                                                           //
                                                           //With automatic scaling (pwm_autoscale=1) the value is used for first initialization, only.
                                                           //Set PWM_GRAD to the application specific value (it can be read out from PWM_GRAD_AUTO) to speed up the automatic tuning process.
                                                           //An approximate value can be stored to OTP by programming OTP_PWM_GRAD.
#define PWMCONF_PWM_OFS                (255 << 0)          //User defined amplitude (offset)
#define PWMCONF_PWM_OFS_SHIFT          0                   //User defined PWM amplitude offset (0-255) related to full motor current (CS_ACTUAL=31) in stand still.
                                                           //Reset default=36
                                                           //
                                                           //When using automatic scaling (pwm_autoscale=1) the value is used for initialization, only.
                                                           //The autoscale function starts with PWM_SCALE_AUTO=PWM_OFS and finds the required offset to yield the target current automatically.
                                                           //
                                                           //PWM_OFS = 0 will disable scaling down motor current below a motor specific lower measurement threshold.
                                                           //This setting should only be used under certain conditions, i.e. when the power supply voltage can vary up and down by a factor of two or more.
                                                           //It prevents the motor going out of regulation, but it also prevents power down below the regulation limit.
                                                           //
                                                           //PWM_OFS > 0 allows automatic scaling to low PWM duty cycles even below the lower regulation threshold.
                                                           //This allows low (standstill) current settings based on the //actual (hold) current scale (register IHOLD_IRUN).

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define PWM_SCALE_REGISTER             0x71      //R       9+8       //Results of stealthChop amplitude regulator. These values can be used to monitor automatic PWM amplitude scaling (255=max. voltage).
/********************************************************************************/
//Function                             Bit                 Description
#define PWM_SCALE_SUM                  (255 << 0)          //Actual PWM duty cycle. This value is used for scaling the values CUR_A and CUR_B read from the sine wave table.
#define PWM_SCALE_SUM_SHIFT            0                   //Range: 0...255
#define PWM_SCALE_AUTO                 (511 << 16)         //9 Bit signed offset added to the calculated PWM duty cycle. This is the result of the automatic amplitude regulation based on current measurement.
#define PWM_SCALE_AUTO_SHIFT           16                  //Range: -255...+255

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define PWM_AUTO_REGISTER              0x72      //R       8+8       //These automatically generated values can be read out in order to determine a default / power up setting for PWM_GRAD and PWM_OFS.
/********************************************************************************/
//Function                             Bit                 Description
#define PWM_AUTO_OFS                   (255 << 0)          //Automatically determined offset value
#define PWM_AUTO_OFS_SHIFT             0                   //Range: 0...255
#define PWM_AUTO_GRAD                  (255 << 16)         //Automatically determined gradient value
#define PWM_AUTO_GRAD_SHIFT            16                  //Range: 0...255

/*
 * Constructor
 */
TMC22X::TMC22X(std::function<int(uint8_t *b, int cnt, uint8_t *r)> uart, char d) : uart(uart), designator(d)
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
void TMC22X::init(uint16_t cs)
{
    // read chip specific config entries
    this->resistor= THEKERNEL->config->value(motor_driver_control_checksum, cs, sense_resistor_checksum)->by_default(50)->as_number(); // in milliohms

    //set the initial values
    send2208(0x80,0x00000040);
    send2208(0x6F,0x00000000);

    started = true;
}

//calculates CRC checksum and stores in last byte of message
void calc_crc(uint8_t *buf, int cnt)
{
    uint8_t *crc = buf + cnt -1; // CRC located in last byte of message
    uint8_t currentByte;

    *crc = 0;
    for (int i = 0; i < cnt-1; i++) {  // Execute for all bytes of a message
        currentByte = buf[i];          // Retrieve a byte to be sent from Array
        for (int j = 0; j < 8; j++) {
            if ((*crc >> 7) ^ (currentByte & 0x01)) {   // update CRC based result of XOR operation
                *crc = (*crc << 1) ^ 0x07;
            } else {
                *crc = (*crc << 1);
            }
            //crc &= 0xff;
            currentByte = currentByte >> 1;
        }   // for CRC bit
    }       // for message byte
}

/*
 * send register settings to the stepper driver via SPI
 * returns the current status
 * sends 20bits, the last 20 bits of the 24bits is taken as the command
 */
bool TMC22X::send2208(uint8_t reg, uint32_t datagram)
{
    uint8_t rbuf[8];
    if(reg & WRITE) {
        uint8_t buf[] {(uint8_t)(SYNC), (uint8_t)(SLAVEADDR), (uint8_t)(reg), (uint8_t)(datagram >> 24), (uint8_t)(datagram >> 16), (uint8_t)(datagram >> 8), (uint8_t)(datagram >> 0), (uint8_t)(0x00)};

        //calculate checksum
        calc_crc(buf, 8);

        //write/read the values
        uart(buf, 8, rbuf);
        THEKERNEL->streams->printf("sent: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X \n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
    } else {
        uint8_t buf[] {(uint8_t)(SYNC), (uint8_t)(SLAVEADDR), (uint8_t)(reg), (uint8_t)(0x00)};

        //calculate checksum
        calc_crc(buf, 4);

        //write/read the values
        uart(buf, 4, rbuf);

        THEKERNEL->streams->printf("sent: %02X, %02X, %02X, %02X received: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X \n", buf[0], buf[1], buf[2], buf[3], rbuf[0], rbuf[1], rbuf[2], rbuf[3], rbuf[4], rbuf[5], rbuf[6], rbuf[7]);
    }
    return true;
}

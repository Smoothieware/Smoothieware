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

#define motor_driver_control_checksum   CHECKSUM("motor_driver_control")
#define sense_resistor_checksum         CHECKSUM("sense_resistor")
#define chopper_mode_checksum           CHECKSUM("chopper_mode")
#define stealthchop_tpwmthrs_checksum   CHECKSUM("stealthchop_tpwmthrs")
#define spreadcycle_toff_checksum       CHECKSUM("spreadcycle_toff")
#define spreadcycle_tbl_checksum        CHECKSUM("spreadcycle_tbl")
#define spreadcycle_hstrt_checksum      CHECKSUM("spreadcycle_hstrt")
#define spreadcycle_hend_checksum       CHECKSUM("spreadcycle_hend")


//! return value for TMC22X.getOverTemperature() if there is a overtemperature situation in the TMC chip
/*!
 * This warning indicates that the TMC chip is too warm.
 * It is still working but some parameters may be inferior.
 * You should do something against it.
 */
#define TMC22X_OVERTEMPERATURE_PREWARNING 1
//! return value for TMC22X.getOverTemperature() if there is a overtemperature shutdown in the TMC chip
/*!
 * This warning indicates that the TMC chip is too warm to operate and has shut down to prevent damage.
 * It will stop working until it cools down again.
 * If you encounter this situation you must do something against it. Like reducing the current or improving the PCB layout
 * and/or heat management.
 */
#define TMC22X_OVERTEMPERATURE_SHUTDOWN 2

//which values can be read out
/*!
 * Selects to readout the microstep position from the motor.
 *\sa readStatus()
 */
#define TMC22X_READOUT_MICROSTEP 0
/*!
 * Selects to read out the current setting from the motor.
 *\sa readStatus()
 */
#define TMC22X_READOUT_CURRENT 1

//some default values used in initialization
#define DEFAULT_MICROSTEPPING_VALUE    32
#define ZEROS_DEFAULT_DATA             0x00000000
#define GCONF_DEFAULT_DATA             0x00000101
#define TPOWERDOWN_DEFAULT_DATA        0x00000014
#define CHOPCONF_DEFAULT_DATA          0x10000053
#define PWMCONF_DEFAULT_DATA           0xC10D0024

//driver access request (read MSB bit is 0 and write MSB bit is 1)
#define READ  0x00
#define WRITE 0x80

//sync and address of the target driver
#define SYNC        0x05
#define SLAVEADDR   0x00

//TMC22X register definitions

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
#define OTP_PROG_REGISTER              0x04      //W       16       //OTP programming
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
#define OTP_READ_REGISTER              0x05      //R       24        //Access to OTP memory result and update
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
                                                                     //Reset default=20
/********************************************************************************/

//Register                             Address   Type      N Bits    Description
/********************************************************************************/
#define TSTEP_REGISTER                 0x12      //R       20        //Actual measured time between two 1/256 microsteps derived from the step input frequency in units of 1/fCLK.
                                                                     //Measured value is (2^20)-1 in case of overflow or stand still.
                                                                     //
                                                                     //The TSTEP related threshold use a hysteresis of 1/16 of the compare value to compensate for jitter in the clock or the step frequency:
                                                                     //(Txxx*15/16)-1 is the lower compare value for each TSTEP based comparison.
                                                                     //This means, that the lower switching velocity equals the calculated setting,
                                                                     //but the upper switching velocity is higher as defined by the hysteresis setting.
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
#define VACTUAL_REGISTER               0x22      //W       24        //VACTUAL allows moving the motor by UART control. It gives the motor velocity in +-(2^23)-1 [μsteps / t]
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
TMC22X::TMC22X(std::function<int(uint8_t *b, int cnt, uint8_t *r)> serial, char d) : serial(serial), designator(d)
{
    //we are not started yet
    started = false;
    error_reported.reset();
}

/*
 * configure the stepper driver
 * just must be called.
 */
void TMC22X::init(uint16_t cs)
{
    // Read chip specific config entries
    this->resistor= THEKERNEL->config->value(motor_driver_control_checksum, cs, sense_resistor_checksum)->by_default(50)->as_number(); // in milliohms
    this->chopper_mode= THEKERNEL->config->value(motor_driver_control_checksum, cs, chopper_mode_checksum)->by_default(0)->as_number();

    // Setting the default register values
    this->gconf_register_value = GCONF_DEFAULT_DATA;
    this->slaveconf_register_value = ZEROS_DEFAULT_DATA;
    this->ihold_irun_register_value = ZEROS_DEFAULT_DATA;
    this->tpowerdown_register_value = TPOWERDOWN_DEFAULT_DATA;
    this->tpwmthrs_register_value = ZEROS_DEFAULT_DATA;
    this->chopconf_register_value = CHOPCONF_DEFAULT_DATA;
    this->pwmconf_register_value = PWMCONF_DEFAULT_DATA;

    // Configure SpreadCycle (also uses in StealthChop when TPWMTHRS is set)
    // (default values come from the Quick Configuration Guide chapter in TMC2208 datasheet)
    int8_t toff  = THEKERNEL->config->value(motor_driver_control_checksum, cs, spreadcycle_toff_checksum)->by_default(5)->as_int();
    int8_t tbl   = THEKERNEL->config->value(motor_driver_control_checksum, cs, spreadcycle_tbl_checksum)->by_default(2)->as_int();
    int8_t hstrt = THEKERNEL->config->value(motor_driver_control_checksum, cs, spreadcycle_hstrt_checksum)->by_default(4)->as_int(); 
    int8_t hend  = THEKERNEL->config->value(motor_driver_control_checksum, cs, spreadcycle_hend_checksum)->by_default(0)->as_int(); 

    setSpreadCycleChopper(toff, tbl, hstrt, hend);

    // Select mode (0 - spreadCycle, 1 - stealthChop)
    if (chopper_mode == 1) {
        // Enable StealthChop by disabling SpreadCycle function
        setSpreadCycleEnabled(false);

        // Arguments order: lim, reg, freewheel, autograd, autoscale, freq, grad, ofs
        // Default stealthChop configuration
        setStealthChop(12,1,0,1,1,1,0,36);

        // Set the upper velocity for stealthChop voltage PWM mode (TPWMTHRS)
        int tpwmthrs = THEKERNEL->config->value(motor_driver_control_checksum, cs, stealthchop_tpwmthrs_checksum)->by_default(0)->as_int();

        if (tpwmthrs > 0) {
            setStealthChopthreshold(tpwmthrs);
        } 
    } else {
        setSpreadCycleEnabled(true);
    }

    // Set microstepping via software and set external sense resistors using internal reference voltage
    setGeneralConfiguration(0,0,0,0,1,1);

    // Set a nice microstepping value
    setMicrosteps(DEFAULT_MICROSTEPPING_VALUE);

    // Set the initial values
    send2208(WRITE|GCONF_REGISTER, this->gconf_register_value);
    send2208(WRITE|SLAVECONF_REGISTER, this->slaveconf_register_value);
    send2208(WRITE|IHOLD_IRUN_REGISTER, this->ihold_irun_register_value);
    send2208(WRITE|TPOWERDOWN_REGISTER, this->tpowerdown_register_value);
    send2208(WRITE|TPWMTHRS_REGISTER, this->tpwmthrs_register_value);
    send2208(WRITE|CHOPCONF_REGISTER, this->chopconf_register_value);
    send2208(WRITE|PWMCONF_REGISTER, this->pwmconf_register_value);

    //started
    started = true;
}

void TMC22X::setGeneralConfiguration(bool i_scale_analog, bool internal_rsense, bool shaft, bool pdn_disable, bool mstep_reg_select, bool multistep_filt)
{
    if (i_scale_analog) {
        //set voltage supplied to VREF as current reference
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

    if (pdn_disable) {
        //disable standstill current reduction via PDN_UART pin
        gconf_register_value |= GCONF_PDN_DISABLE;
    } else {
        //enable standstill current reduction via PDN_UART pin
        gconf_register_value &= ~(GCONF_PDN_DISABLE);
    }

    if (mstep_reg_select) {
        //select microstep resolution by MSTEP register
        gconf_register_value |= GCONF_MSTEP_REG_SELECT;
    } else {
        //select microstep resolution by pins MS1, MS2
        gconf_register_value &= ~(GCONF_MSTEP_REG_SELECT);
    }

    if (multistep_filt) {
        //enable software pulse generator optimization when fullstep frequency > 750 Hz
        gconf_register_value |= GCONF_MULTISTEP_FILT;
    } else {
        //disable STEP pulses filtering
        gconf_register_value &= ~(GCONF_MULTISTEP_FILT);
    }

    //if started we directly send it to the motor
    if (started) {
        send2208(WRITE|GCONF_REGISTER, gconf_register_value);
    }
}

void TMC22X::setIndexoptions(bool otpw, bool step)
{
    if (otpw) {
        //Index pin outputs overtemperature pre-warning flag
        gconf_register_value |= GCONF_INDEX_OTPW;
    } else {
        //Index pin outputs first microstep position of sequencer
        gconf_register_value &= ~(GCONF_INDEX_OTPW);
    }

    if (step) {
        //Index pin outputs step pulses from internal pulse generator
        gconf_register_value |= GCONF_MULTISTEP_FILT;
    } else {
        //Index pin outputs the same as selected by index_otpw
        gconf_register_value &= ~(GCONF_MULTISTEP_FILT);
    }

    //if started we directly send it to the motor
    if (started) {
        send2208(WRITE|GCONF_REGISTER, gconf_register_value);
    }
}

void TMC22X::setSenddelay(uint8_t value)
{
    //perform some sanity checks
    if (value > 15) {
        value = 15;
    }

    //delete the old value
    this->slaveconf_register_value &= ~(SLAVECONF_SENDDELAY);

    //save the new value
    this->slaveconf_register_value = value << SLAVECONF_SENDDELAY_SHIFT;

    //if started we directly send it to the motor
    if (started) {
        send2208(WRITE|SLAVECONF_REGISTER,slaveconf_register_value);
    }
}

/*
 * Set the number of microsteps per step.
 * 0,2,4,8,16,32,64,128,256 is supported
 * any value in between will be mapped to the next smaller value
 * 0 and 1 set the motor in full step mode
 */
void TMC22X::setMicrosteps(int number_of_steps)
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
        send2208(WRITE|CHOPCONF_REGISTER, chopconf_register_value);
    }
}

/*
 * returns the effective number of microsteps at the moment
 */
int TMC22X::getMicrosteps(void)
{
    return microsteps;
}

void TMC22X::setStepInterpolation(int8_t value)
{
    if (value) {
        chopconf_register_value |= CHOPCONF_INTPOL;
    } else {
        chopconf_register_value &= ~(CHOPCONF_INTPOL);
    }
    //if started we directly send it to the motor
    if (started) {
        send2208(WRITE|CHOPCONF_REGISTER, chopconf_register_value);
    }
}

void TMC22X::setDoubleEdge(int8_t value)
{
    if (value) {
        chopconf_register_value |= CHOPCONF_DEDGE;
    } else {
        chopconf_register_value &= ~(CHOPCONF_DEDGE);
    }
    //if started we directly send it to the motor
    if (started) {
        send2208(WRITE|CHOPCONF_REGISTER, chopconf_register_value);
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
 */

void TMC22X::setSpreadCycleChopper(int8_t constant_off_time, int8_t blank_time, int8_t hysteresis_start, int8_t hysteresis_end)
{
    h_start = hysteresis_start;
    h_end = hysteresis_end;
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
    if (blank_time >= 40) {
        blank_value = 3;
    } else if (blank_time >= 32) {
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

    //first of all delete all the values for this
    chopconf_register_value &= ~(CHOPCONF_TBL | CHOPCONF_HEND | CHOPCONF_HSTRT | CHOPCONF_TOFF);
    //set the blank timing value
    chopconf_register_value |= ((unsigned long)blank_value) << CHOPCONF_TBL_SHIFT;
    //setting the constant off time
    chopconf_register_value |= ((unsigned long)constant_off_time) << CHOPCONF_TOFF_SHIFT;
    //set the hysteresis_start
    chopconf_register_value |= ((unsigned long)hysteresis_start) << CHOPCONF_HSTRT_SHIFT;
    //set the hysteresis end
    chopconf_register_value |= ((unsigned long)hysteresis_end) << CHOPCONF_HEND_SHIFT;

    //if started we directly send it to the motor
    if (started) {
        send2208(WRITE|CHOPCONF_REGISTER,chopconf_register_value);
    }
}

void TMC22X::setSpreadCycleEnabled(bool enable)
{
    if (enable) {
        gconf_register_value |= GCONF_EN_SPREADCYCLE;
    } else {
        gconf_register_value &= ~(GCONF_EN_SPREADCYCLE);
    }
    //if started we directly send it to the motor
    if (started) {
        send2208(WRITE|GCONF_REGISTER, gconf_register_value);
    }
}

void TMC22X::setStealthChop(uint8_t lim, uint8_t reg, uint8_t freewheel, bool autograd, bool autoscale, uint8_t freq, uint8_t grad, uint8_t ofs)
{
    //perform some sanity checks
    if (lim > 15) {
        lim = 15;
    }
    if (reg > 15) {
        reg = 15;
    }
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
        if(autograd) {
            //set PWM automatic gradient adaptation
            pwmconf_register_value |= PWMCONF_PWM_AUTOGRAD;
        }
        //set regulation loop gradient
        pwmconf_register_value |= (reg << PWMCONF_PWM_REG_SHIFT);
    }

    //limit for PWM_SCALE_AUTO when switching back from spreadCycle to stealthChop
    pwmconf_register_value |= (lim << PWMCONF_PWM_LIM_SHIFT);
    //set standstill mode
    pwmconf_register_value |= (freewheel << PWMCONF_FREEWHEEL_SHIFT);
    //set PWM frequency
    pwmconf_register_value |= (freq << PWMCONF_PWM_FREQ_SHIFT);
    //set user defined amplitude gradient
    pwmconf_register_value |= (grad << PWMCONF_PWM_GRAD_SHIFT);
    //set user defined amplitude offset
    pwmconf_register_value |= (ofs << PWMCONF_PWM_OFS_SHIFT);

    //if started we directly send it to the motor
    if (started) {
        send2208(WRITE|PWMCONF_REGISTER,pwmconf_register_value);
    }
}

void TMC22X::setHolddelay(uint8_t value)
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
        send2208(WRITE|IHOLD_IRUN_REGISTER,ihold_irun_register_value);
    }
}

void TMC22X::setPowerDowndelay(uint8_t value)
{
    //save the delay to the register variable
    this->tpowerdown_register_value = value;

    //if started we directly send it to the motor
    if (started) {
        send2208(WRITE|TPOWERDOWN_REGISTER,tpowerdown_register_value);
    }
}

void TMC22X::setStealthChopthreshold(uint32_t threshold)
{
    //perform some sanity checks
    if (threshold >= (1 << 20)) {
        threshold = (1 << 20) - 1;
    }

    //save the threshold value
    this->tpwmthrs_register_value = threshold;

    //if started we directly send it to the motor
    if (started) {
        send2208(WRITE|TPWMTHRS_REGISTER,tpwmthrs_register_value);
    }
}

void TMC22X::setCurrent(unsigned int current)
{
    uint8_t current_scaling = 0;
    //calculate the current scaling from the max current setting (in mA)
    double mASetting = (double)current;
    double resistor_value = (double) this->resistor;
    // remove vsense flag
    this->chopconf_register_value &= ~(CHOPCONF_VSENSE);
    //this is derived from I=(CS+1)/32*(Vsense/Rsense)
    //leading to CS = 32*Rsense*I/Vsense
    //with I = 1000 mA (default)
    //with Rsense = 50 milli Ohm (default)
    //for vsense = 0,32V (VSENSE not set)
    //or vsense = 0,18V (VSENSE set)
    current_scaling = (uint8_t)(((resistor_value + 20) * mASetting * 32.0F / (0.32F * 1000.0F * 1000.0F)) - 0.5F); //theoretically - 1.0 for better rounding it is 0.5
    //check if the current scaling is too low
    if (current_scaling < 16) {
        //set the Vsense bit to get a use half the sense voltage (to support lower motor currents)
        this->chopconf_register_value |= CHOPCONF_VSENSE;
        //and recalculate the current setting
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
        send2208(WRITE|CHOPCONF_REGISTER,chopconf_register_value);
        send2208(WRITE|IHOLD_IRUN_REGISTER,ihold_irun_register_value);
    }
}

void TMC22X::setHoldCurrent(uint8_t hold)
{
    double current_scaling = (double)((ihold_irun_register_value & IHOLD_IRUN_IRUN) >> IHOLD_IRUN_IRUN_SHIFT);
    //delete the old value
    ihold_irun_register_value &= ~(IHOLD_IRUN_IHOLD);
    //set the new current scaling
    ihold_irun_register_value |= (uint8_t)(current_scaling * ((double) hold) / 100.0F) << IHOLD_IRUN_IHOLD_SHIFT;
    if (started) {
        send2208(WRITE|IHOLD_IRUN_REGISTER,ihold_irun_register_value);
    }
}

void TMC22X::setResistor(unsigned int value)
{
    this->resistor = value;
}

unsigned int TMC22X::getCurrent(void)
{
    //we calculate the current according to the datasheet to be on the safe side
    //this is not the fastest but the most accurate and illustrative way
    double result = (double)((ihold_irun_register_value & IHOLD_IRUN_IRUN) >> IHOLD_IRUN_IRUN_SHIFT);
    double resistor_value = (double)this->resistor;
    double voltage = (chopconf_register_value & CHOPCONF_VSENSE) ? 0.18F : 0.32F;
    result = (result + 1.0F) / 32.0F * voltage / (resistor_value + 20) * 1000.0F * 1000.0F;
    return (unsigned int)result;
}

unsigned int TMC22X::getCurrentCSReading(void)
{
    //if we don't yet started there cannot be a current value
    if (!started) {
        return 0;
    }
    float result = ((readStatus(TMC22X_READOUT_CURRENT) & DRV_STATUS_CS_ACTUAL) >> DRV_STATUS_CS_ACTUAL_SHIFT);
    float resistor_value = (float)this->resistor;
    float voltage = (chopconf_register_value & CHOPCONF_VSENSE) ? 0.18F : 0.32F;
    result = (result + 1.0F) / 32.0F * voltage / (resistor_value + 20) * 1000.0F * 1000.0F;
    return (unsigned int)roundf(result);
}

bool TMC22X::isCurrentScalingHalfed()
{
    if (this->chopconf_register_value & CHOPCONF_VSENSE) {
        return true;
    } else {
        return false;
    }
}

/*
 returns if there is any over temperature condition:
 OVER_TEMPERATURE_PREWARNING if pre warning level has been reached
 OVER_TEMPERATURE_SHUTDOWN if the temperature is so hot that the driver is shut down
 Any of those levels are not too good.
*/
int8_t TMC22X::getOverTemperature(void)
{
    if (!this->started) {
        return 0;
    }
    if (readStatus(TMC22X_READOUT_CURRENT) & DRV_STATUS_OT) {
        return TMC22X_OVERTEMPERATURE_SHUTDOWN;
    }
    if (readStatus(TMC22X_READOUT_CURRENT) & DRV_STATUS_OTPW) {
        return TMC22X_OVERTEMPERATURE_PREWARNING;
    }
    return 0;
}

//is motor channel A shorted to ground
bool TMC22X::isShortToGroundA(void)
{
    if (!this->started) {
        return false;
    }
    return (readStatus(TMC22X_READOUT_CURRENT) & DRV_STATUS_S2GA);
}

//is motor channel B shorted to ground
bool TMC22X::isShortToGroundB(void)
{
    if (!this->started) {
        return false;
    }
    return (readStatus(TMC22X_READOUT_CURRENT) & DRV_STATUS_S2GB);
}

//is motor channel A connected
bool TMC22X::isOpenLoadA(void)
{
    if (!this->started) {
        return false;
    }
    return (readStatus(TMC22X_READOUT_CURRENT) & DRV_STATUS_OLA);
}

//is motor channel B connected
bool TMC22X::isOpenLoadB(void)
{
    if (!this->started) {
        return false;
    }
    return (readStatus(TMC22X_READOUT_CURRENT) & DRV_STATUS_OLB);
}

//is chopper inactive since 2^20 clock cycles - defaults to ~0,08s
bool TMC22X::isStandStill(void)
{
    if (!this->started) {
        return false;
    }
    return (readStatus(TMC22X_READOUT_CURRENT) & DRV_STATUS_STST);
}

void TMC22X::setEnabled(bool enabled)
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
        chopconf_register_value |= ((unsigned long)this->constant_off_time) << CHOPCONF_TOFF_SHIFT;
    }
    //if not enabled we don't have to do anything since we already delete t_off from the register
    if (started) {
        send2208(WRITE|CHOPCONF_REGISTER, chopconf_register_value);
    }
}

bool TMC22X::isEnabled()
{
    if (chopconf_register_value & CHOPCONF_TOFF) {
        return true;
    } else {
        return false;
    }
}

//reads a value from the TMC22X status register.
unsigned long TMC22X::readStatus(int8_t read_value)
{
    uint32_t data;
    if(read_value == TMC22X_READOUT_MICROSTEP) {
        data = send2208(READ|MSCNT_REGISTER,ZEROS_DEFAULT_DATA);
    } else {
        data = send2208(READ|DRV_STATUS_REGISTER,ZEROS_DEFAULT_DATA);
    }
    return data;
}

void TMC22X::dumpStatus(StreamOutput *stream, bool readable)
{
    if (readable) {
        stream->printf("designator %c, Chip type TMC22X\n", designator);

        check_error_status_bits(stream);

        if (this->isStandStill()) {
            stream->printf("INFO: Motor is standing still.\n");
        }

        int value = readStatus(TMC22X_READOUT_MICROSTEP);
        stream->printf("Microstep position phase A: %d\n", value);

        stream->printf("Current setting: %dmA\n", getCurrent());

        stream->printf("Microsteps: 1/%d\n", microsteps);

        stream->printf("Register dump:\n");
        stream->printf(" gconf register: %08lX(%ld)\n", gconf_register_value, gconf_register_value);
        stream->printf(" slaveconf register: %08lX(%ld)\n", slaveconf_register_value, slaveconf_register_value);
        stream->printf(" ihold_irun register: %08lX(%ld)\n", ihold_irun_register_value, ihold_irun_register_value);
        stream->printf(" tpowerdown register: %08lX(%ld)\n", tpowerdown_register_value, tpowerdown_register_value);
        stream->printf(" tpwmthrs register: %08lX(%ld)\n", tpwmthrs_register_value, tpwmthrs_register_value);
        stream->printf(" chopconf register: %08lX(%ld)\n", chopconf_register_value, chopconf_register_value);
        stream->printf(" pwmconf register: %08lX(%ld)\n", pwmconf_register_value, pwmconf_register_value);
        stream->printf(" motor_driver_control.xxx.reg %05lX,%05lX,%05lX,%05lX,%05lX,%05lX,%05lX\n", gconf_register_value, slaveconf_register_value, ihold_irun_register_value,
                                                                                              tpowerdown_register_value, tpwmthrs_register_value, chopconf_register_value, pwmconf_register_value);

    } else {
        // TODO hardcoded for X need to select ABC as needed
        bool moving = THEROBOT->actuators[0]->is_moving();
        // dump out in the format that the processing script needs
        if (moving) {
            stream->printf("#p%lu,k%u,r,", THEROBOT->actuators[0]->get_current_step(), getCurrentCSReading());
        } else {
            readStatus(TMC22X_READOUT_MICROSTEP); // get the status bits
            stream->printf("#s,");
        }
        stream->printf("d%d,", THEROBOT->actuators[0]->which_direction() ? -1 : 1);
        stream->printf("c%u,m%d,", getCurrent(), getMicrosteps());
        // stream->printf('S');
        // stream->printf(tmc22XStepper.getSpeed(), DEC);

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
        } else if (temperature == TMC22X_OVERTEMPERATURE_PREWARNING) {
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
        stream->printf("Co%d,Cb%d,", constant_off_time, blank_time);
        stream->printf("\n");
    }
}

// check error bits and report, only report once
bool TMC22X::check_error_status_bits(StreamOutput *stream)
{
    bool error= false;
    readStatus(TMC22X_READOUT_MICROSTEP); // get the status bits

    if (this->getOverTemperature()&TMC22X_OVERTEMPERATURE_PREWARNING) {
        if(!error_reported.test(0)) stream->printf("%c - WARNING: Overtemperature Prewarning!\n", designator);
        error_reported.set(0);
    }else{
        error_reported.reset(0);
    }

    if (this->getOverTemperature()&TMC22X_OVERTEMPERATURE_SHUTDOWN) {
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

bool TMC22X::checkAlarm()
{
    return check_error_status_bits(THEKERNEL->streams);
}

// sets a raw register to the value specified, for advanced settings
// register 255 writes them, 0 displays what registers are mapped to what
bool TMC22X::setRawRegister(StreamOutput *stream, uint32_t reg, uint32_t val)
{
    switch(reg) {
        case 255:
            send2208(WRITE|GCONF_REGISTER, this->gconf_register_value);
            send2208(WRITE|SLAVECONF_REGISTER, this->slaveconf_register_value);
            send2208(WRITE|IHOLD_IRUN_REGISTER, this->ihold_irun_register_value);
            send2208(WRITE|TPOWERDOWN_REGISTER, this->tpowerdown_register_value);
            send2208(WRITE|TPWMTHRS_REGISTER, this->tpwmthrs_register_value);
            send2208(WRITE|CHOPCONF_REGISTER, this->chopconf_register_value);
            send2208(WRITE|PWMCONF_REGISTER, this->pwmconf_register_value);
            stream->printf("Registers written\n");
            break;


        case 1: this->gconf_register_value = val; stream->printf("gconf register set to %08lX\n", val); break;
        case 2: this->ihold_irun_register_value = val; stream->printf("ihold irun config register set to %08lX\n", val); break;
        case 3: this->slaveconf_register_value = val; stream->printf("slaveconf config register set to %08lX\n", val); break;
        case 4: this->tpowerdown_register_value = val; stream->printf("tpowerdown register set to %08lX\n", val); break;
        case 5: this->tpwmthrs_register_value = val; stream->printf("tpwmthrs register set to %08lX\n", val); break;
        case 6: this->chopconf_register_value = val; stream->printf("chopconf register set to %08lX\n", val); break;
        case 7: this->pwmconf_register_value = val; stream->printf("pwmconf register set to %08lX\n", val); break;

        default:
            stream->printf("1: gconf register\n");
            stream->printf("2: slaveconf register\n");
            stream->printf("3: ihold irun register\n");
            stream->printf("4: tpowerdown register\n");
            stream->printf("5: tpwmthrs register\n");
            stream->printf("6: chopconf register\n");
            stream->printf("7: pwmconf register\n");
            stream->printf("255: update all registers\n");
            return false;
    }
    return true;
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
 * send register settings to the stepper driver via UART
 * send 64 bits for write request and 32 bit for read request
 * receive 64 bits only for read request
 * returns received data content
 */
uint32_t TMC22X::send2208(uint8_t reg, uint32_t datagram)
{
    uint8_t rbuf[8];
    uint32_t i_datagram = 0;
    if(reg & WRITE) {
        uint8_t buf[] {(uint8_t)(SYNC), (uint8_t)(SLAVEADDR), (uint8_t)(reg), (uint8_t)(datagram >> 24), (uint8_t)(datagram >> 16), (uint8_t)(datagram >> 8), (uint8_t)(datagram >> 0), (uint8_t)(0x00)};

        //calculate checksum
        calc_crc(buf, 8);

        //write/read the values
        serial(buf, 8, rbuf);

        //THEKERNEL->streams->printf("sent: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X \n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
    } else {
        uint8_t buf[] {(uint8_t)(SYNC), (uint8_t)(SLAVEADDR), (uint8_t)(reg), (uint8_t)(0x00)};

        //calculate checksum
        calc_crc(buf, 4);

        //write/read the values
        serial(buf, 4, rbuf);

        //construct reply
        i_datagram = ((rbuf[3] << 24) | (rbuf[4] << 16) | (rbuf[5] << 8) | (rbuf[6] << 0));

        //THEKERNEL->streams->printf("sent: %02X, %02X, %02X, %02X received: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X \n", buf[0], buf[1], buf[2], buf[3], rbuf[0], rbuf[1], rbuf[2], rbuf[3], rbuf[4], rbuf[5], rbuf[6], rbuf[7]);
    }
    return i_datagram;
}

#define HAS(X) (options.find(X) != options.end())
#define GET(X) (options.at(X))
bool TMC22X::set_options(const options_t& options)
{
    bool set = false;

    if(HAS('S')) {
        uint32_t s = GET('S');
        if(s == 0 && HAS('U') && HAS('V') && HAS('W') && HAS('X')) {
            //arguments order: constant_off_time, blank_time, hysteresis_start, hysteresis_end
            setSpreadCycleChopper(GET('U'), GET('V'), GET('W'), GET('X'));
            set = true;

        } else if(s == 1 && HAS('Z')) {
            setSpreadCycleEnabled(GET('Z'));
            set = true;

        } else if(s == 2 && HAS('Z')) {
            setDoubleEdge(GET('Z'));
            set = true;

        } else if(s == 3 && HAS('Z')) {
            setStepInterpolation(GET('Z'));
            set = true;

        } else if(s == 4 && HAS('U') && HAS('V') && HAS('W') && HAS('X') && HAS('Y') && HAS('Z')) {
            //arguments order: lim, reg, freewheel, autograd, autoscale, freq, grad, ofs
            setStealthChop(GET('U'), GET('V'), GET('W'), 1, 1, GET('X'), GET('Y'), GET('Z'));
            set = true;

        } else if(s == 5 && HAS('Z')) {
            setStealthChopthreshold(GET('Z'));
            set = true;

        } else if(s == 6 && HAS('Z')) {
            setResistor(GET('Z'));
            set = true;

        } else if(s == 7 && HAS('Z')) {
            setHoldCurrent(GET('Z'));
            set = true;

        } else if(s == 8 && HAS('Z')) {
            setHolddelay(GET('Z'));
            set = true;

        } else if(s == 9 && HAS('Z')) {
            setPowerDowndelay(GET('Z'));
            set = true;

        } else if(s == 10 && HAS('U') && HAS('V') && HAS('W') && HAS('X') && HAS('Y') && HAS('Z')) {
            //arguments order: i_scale_analog, internal_rsense, shaft, pdn_disable, mstep_reg_select, multistep_filt
            setGeneralConfiguration(GET('U'), GET('V'), GET('W'), GET('X'), GET('Y'), GET('Z'));
            set = true;

        } else if(s == 11 && HAS('U') && HAS('V')) {
            //arguments order: otpw, step
            setIndexoptions(GET('U'), GET('V'));
            set = true;

        } else if(s == 12 && HAS('Z')) {
            setSenddelay(GET('Z'));
            set = true;

        }

    }

    return set;
}


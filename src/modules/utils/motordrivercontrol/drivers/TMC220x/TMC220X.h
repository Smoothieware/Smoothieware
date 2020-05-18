/*
 modified from...
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


// ensure this library description is only included once
#pragma once

#include <functional>
#include <map>
#include <bitset>

#include "../StepperDrv.h"

class StreamOutput;

/*!
 * \class TMC220X
 * \brief Class representing a TMC220X stepper driver
 */
class TMC220X : public StepperDrv 
{
public:
    /*!
     * \brief creates a new represenatation of a stepper motor connected to a TMC220X stepper driver
     *
     * This is the main constructor. If in doubt use this. You must provide all parameters as described below.
     *
     * \param serial send function
     *
     * By default the Constant Off Time chopper is used, see TCM262Stepper.setConstantOffTimeChopper() for details.
     * This should work on most motors (YMMV). You may want to configure and use the Spread Cycle Chopper, see  setSpreadCycleChopper().
     *
     * By default a microstepping of 1/32th is used to provide a smooth motor run, while still giving a good progression per step.
     * You can select a different stepping with setMicrosteps() to aa different value.
     * \sa start(), setMicrosteps()
     */
    TMC220X(std::function<int(uint8_t *b, int cnt, uint8_t *r)> serial, char designator);

    /*!
     * \brief configures the TMC220X stepper driver. Before you called this function the stepper driver is in nonfunctional mode.
     *
     * \param rms_current the maximum current to privide to the motor in mA (!). A value of 200 would send up to 200mA to the motor
     * \param resistor the current sense resistor in milli Ohm, defaults to ,15 Ohm ( or 150 milli Ohm) as in the TMC260 Arduino Shield

     * This routine configures the TMC220X stepper driver for the given values via SPI.
     * Most member functions are non functional if the driver has not been started.
     * Therefore it is best to call this in your Arduino setup() function.
     */
    void init(uint16_t cs);

    /*!
     * \brief Set general parameters
     * \param i_scale_analog sets reference voltage (default = 1)
     * \param internal_rsense sets sense resistors
     * \param shaft inverse motor direction
     * \param pdn_disable controls standstill current reduction
     * \param mstep_reg_select microstep resolution selection via register access
     * \param multistep_filt software pulse generator optimization enabled when fullstep frequency > 750Hz
     *
     * This method sets some general flags of global configuration (GCONF) register
     */
    void setGeneralConfiguration(bool i_scale_analog, bool internal_rsense, bool shaft, bool pdn_disable, bool mstep_reg_select, bool multistep_filt);

    /*!
     * \brief Set Index pin options
     * \param otpw enable INDEX active on driver overtemperature prewarning or on first microstep position of sequencer
     * \param step enable INDEX active on step pulses from internal pulse generator
     *
     *  The index pin supplies a configurable set of different real time information such as driver overtemperature pre-warning, first microstep position
     *  of sequencer or step pulses from internal pulse generator
     */
    void setIndexoptions(bool otpw, bool step);

    /*!
     * \brief Set delay time for read access until a reply is sent
     * \param value each adjacent numbers sets increased odd delay time in multiples of eight bit times
     *
     * In order to ensure a clean bus transition from the master to the slave, the TMC22xx does not
     * immediately send the reply to a read access, but it uses a programmable delay time after which the
     * first reply byte becomes sent following a read request. This delay time can be set in multiples of
     * eight bit times using SENDDELAY time setting (default=8 bit times) according to the needs of the master.
     */
    void setSenddelay(uint8_t value);

    /*!
     * \brief Set the number of microsteps in 2^i values (rounded) up to 256
     *
     * This method set's the number of microsteps per step in 2^i interval.
     * This means you can select 1, 2, 4, 16, 32, 64, 128 or 256 as valid microsteps.
     * If you give any other value it will be rounded to the next smaller number (3 would give a microstepping of 2).
     * You can always check the current microstepping with getMicrosteps().
     */
    int set_microsteps(int number_of_steps);

    /*!
     * \brief returns the effective current number of microsteps selected.
     *
     * This function always returns the effective number of microsteps.
     * This can be a bit different than the micro steps set in setMicrosteps() since it is rounded to 2^i.
     *
     * \sa set_microsteps()
     */
    int get_microsteps(void);

    /*!
     * \brief Enable actual microstep resolution (MRES) to 256 microsteps for smoothest motor operation.
     */
    void setStepInterpolation(int8_t value);

    void setDoubleEdge(int8_t value);

    /*!
     * \brief Configures the driver with spread cycle chopper.
     * \param constant_off_time The off time setting controls the minimum chopper frequency. For most applications an off time within the range of 5μs to 20μs will fit. Setting this parameter to zero completely disables all driver transistors and the motor can free-wheel. 0: chopper off, 1:15: off time setting (1 will work with minimum blank time of 24 clocks)
     * \param blank_time Selects the comparator blank time. This time needs to safely cover the switching event and the duration of the ringing on the sense resistor. For most low current drivers, a setting of 1 or 2 is good. For high current applications with large MOSFETs, a setting of 2 or 3 will be required. 0 (min setting) … (3) amx setting
     * \param hysteresis_start Hysteresis start setting. Please remark, that this value is an offset to the hysteresis end value. 1 … 8
     * \param hysteresis_end Hysteresis end setting. Sets the hysteresis end value after a number of decrements. Decrement interval time is controlled by hysteresis_decrement. The sum hysteresis_start + hysteresis_end must be <16. At a current setting CS of max. 30 (amplitude reduced to 240), the sum is not limited.
     * \param hysteresis_decrement Hysteresis decrement setting. This setting determines the slope of the hysteresis during on time and during fast decay time. 0 (fast decrement) … 3 (slow decrement).
     *
     * The spreadCycle chopper scheme (pat.fil.) is a precise and simple to use chopper principle, which automatically determines
     * the optimum fast decay portion for the motor. Anyhow, a number of settings can be made in order to optimally fit the driver
     * to the motor.
     * Each chopper cycle is comprised of an on-phase, a slow decay phase, a fast decay phase and a second slow decay phase.
     * The slow decay phases limit the maximum chopper frequency and are important for low motor and driver power dissipation.
     * The hysteresis start setting limits the chopper frequency by forcing the driver to introduce a minimum amount of
     * current ripple into the motor coils. The motor inductivity determines the ability to follow a changing motor current.
     * The duration of the on- and fast decay phase needs to cover at least the blank time, because the current comparator is
     * disabled during this time.
     *
     */
    void setSpreadCycleChopper(int8_t constant_off_time, int8_t blank_time, int8_t hysteresis_start, int8_t hysteresis_end);

    /*!
     *\brief enables or disables SpreadCycle mode. If disabled, SpreadCycle is disabled, which means StealthChop is enabled. If enabled, SpreadCycle is enabled and StealthChop not.
     *\param enabled a bool value true if SpreadCycle should be enabled, false otherwise.
     */
    void setSpreadCycleEnabled(bool enable);

    /*!
     * \brief Configures the driver with stealthChop.
     * \param lim Limiting value for limiting the current jerk when switching from spreadCycle to stealthChop. Reduce the value to yield a lower current jerk. 0 ... 15
     * \param reg User defined PWM amplitude (gradient) for velocity based scaling or regulation loop gradient when pwm_autoscale=1. 1 ... 15
     * \param freewheel Stand still option when motor current setting is zero (I_HOLD=0). Only available with stealthChop enabled. The freewheeling option makes the motor easy movable, while both coil short options realize a passive brake. 0 - Normal operation, 1 - Freewheeling, 2 - Coil short via LS drivers, 3 - Coil short via HS drivers
     * \param autograd  Enable automatic tuning of PWM_GRAD_AUTO. 0 - disable, use PWM_GRAD from register instead. 1 - enable
     * \param autoscale Enable automatic current scaling using current measurement or use forward controlled velocity based mode. 0 - Forward controlled mode, 1 - Automatic scaling with current regulator
     * \param freq PWM frequency selection. Use the lowest setting giving good results. The frequency measured at each of the chopper outputs is half of the effective chopper frequency fPWM. 0 - fPWM = 2/1024 fCLK, 1 - fPWM=2/683 fCLK, 2 - fPWM=2/512 fCLK, 3 - fPWM=2/410 fCLK
     * \param grad User defined PWM amplitude (gradient) for velocity based scaling and initialization value for automatic tuning of PWM_GRAD_AUTO.
     * \param ofs User defined PWM amplitude (offset) for velocity based scaling and initialization value for automatic tuning of PWM_OFFS_AUTO.
     *
     * Noiseless stealthChop operates absolutely free of vibration at low velocities. With stealthChop, the motor current is applied by driving a certain effective voltage into the coil, using a voltage mode PWM.
     * Optional configuration allows for tuning the setting in special cases, or for storing initial values for the automatic adaptation algorithm.
     * Use automatic tuning procedure (pwm_autoscale = 1) if motor is not well-known as well as operating conditions.
     */
    void setStealthChop(uint8_t lim, uint8_t reg, uint8_t freewheel, bool autograd, bool autoscale, uint8_t freq, uint8_t grad, uint8_t ofs);

    /*!
     * \brief Configures smooth current reduction time from run current to hold current.
     * \param value Delay before power down in stand still 0=instant power down, 1..15: Current reduction delay per current step in multiple of 2^18 clocks
     *
     * IHOLDDELAY controls the number of clock cycles for motor power down after TPOWERDOWN in increments of 2^18 clocks
     */
    void setHolddelay(uint8_t value);

    /*!
     * \brief Configures delayed standstill current reduction
     * \param value Delay before power down in stand still
     *
     * TPOWERDOWN sets the delay time after stand still (stst) of the motor to motor current power down. Time range is about 0 to 4 seconds (0...((2^8)-1) * 2^18 t CLK).
     */
    void setPowerDowndelay(uint8_t value);

    /*!
     * \brief Configures the stealthChop upper velocity threshold.
     * \param threshold velocity threshold. For most applications an velocity threshold between 30 and 200 will fit. Setting this parameter to zero will not enable SpreadCycle.
     *
     * For applications requiring high velocity motion, spreadCycle may bring more stable operation in the
     * upper velocity range. To combine no-noise operation with highest dynamic performance, the TMC22xx
     * allows combining stealthChop and spreadCycle based on a velocity threshold
     */
    void setStealthChopthreshold(uint32_t threshold);
    
    /*!
     * \brief lower threshold velocity for switching on smart energy CoolStep and StallGuard to DIAG output.
     * \param threshold velocity threshold. For most applications an velocity threshold between 30 and 200 will fit. Setting this parameter to zero will not enable SpreadCycle.
     *
     * Set this parameter to disable CoolStep at low speeds, where it
     * cannot work reliably. The stall output signal become enabled
     * when exceeding this velocity. It becomes disabled again once
     * the velocity falls below this threshold.
     */
    void setCoolThreshold(uint32_t threshold);

    /*!
     * \brief Stall detection threshold
     * \param threshold velocity threshold.
     *
     * A stall is signaled with SG_RESULT ≤ SGTHRS*2
     */
    void setStallguardThreshold(uint32_t threshold);
    
    /*!
     * \brief reads out the current Stallguard threashold.
     *
     */
    uint8_t getStallguardResult(void);

    /*!
     * \brief Stall detection threshold
     * \param seimin minimum current for smart control
     * \param sedn current down step speed
     * \param semax StallGuard hysteresis value. If SG_RESULT is >= (SEMIN+SEMAX+1)*32, then the motor current becomes decreased.
     * \param seup Current increment steps per measured StallGuard value
     * \param semin minimum StallGuard value for smart current control
     *
     * A stall is signaled with SG_RESULT ≤ SGTHRS*2
     */
    void setCoolConf(bool seimin, uint8_t sedn, uint16_t semax, uint8_t seup, uint16_t semin);


    /*!
     * \brief set the maximum motor current in mA (1000 is 1 Amp)
     * Keep in mind this is the maximum peak Current. The RMS current will be 1/sqrt(2) smaller. The actual current can also be smaller
     * by employing CoolStep.
     * \param current the maximum motor current in mA
     * \sa getCurrent(), getCurrentCurrent()
     */
    void set_current(uint16_t current);

    /*!
     * \brief set standstill motor current
     * Keep in mind this is the maximum peak Current. The RMS current will be 1/sqrt(2) smaller.
     * \param hold the standtill motor current relative to maximum motor current in percentage
     */
    void setHoldCurrent(uint8_t hold);

    /*!
     * \brief sets new resistor value
     * This method registers the new resistor value included in current scaling calculations
     *\rparam value sensing resistor value as a reference in milliohms
     */
    void setResistor(unsigned int value);

    /*!
     * \brief readout the motor maximum current in mA (1000 is an Amp)
     * This is the maximum current. to get the current current - which may be affected by CoolStep us getCurrentCurrent()
     *\return the maximum motor current in milli amps
     * \sa getCurrentCurrent()
     */
    unsigned int get_current(void);

    /*!
     *\brief a convenience method to determine if the current scaling uses 0.31V or 0.165V as reference.
     *\return false if 0.13V is the reference voltage, true if 0.165V is used.
     */
    bool isCurrentScalingHalfed();

    /*!
     * \brief Reads the current current setting value as fraction of the maximum current
     * values between 0 and 31, representing 1/32 to 32/32 (=1)
     * Recalculates the absolute current in mA (1A would be 1000).
     * This method calculates the currently used current setting (either by setting or by CoolStep) and reconstructs
     * the current in mA by using the VSENSE and resistor value. This method uses floating point math - so it
     * may not be the fastest.
     * \sa getCurrentCSReading(), getResistor(), isCurrentScalingHalfed(), getCurrent()
     */
    unsigned int getCurrentCSReading(void);

    /*!
     * \brief Return over temperature status of the last status readout
     * return 0 is everything is OK, TMC21X_OVERTEMPERATURE_PREWARNING if status is reached, TMC21X_OVERTEMPERATURE_SHUTDOWN is the chip is shutdown, -1 if the status is unknown.
     * Keep in mind that this method does not enforce a readout but uses the value of the last status readout.
     * You may want to use getMotorPosition() or getCurrentStallGuardReading() to enforce an updated status readout.
     */
    int8_t getOverTemperature(void);

    /*!
     * \brief Is motor channel A shorted to ground detected in the last status readout.
     * \return true is yes, false if not.
     * Keep in mind that this method does not enforce a readout but uses the value of the last status readout.
     * You may want to use getMotorPosition() or getCurrentStallGuardReading() to enforce an updated status readout.
     */
    bool isShortToGroundA(void);

    /*!
     * \brief Is motor channel B shorted to ground detected in the last status readout.
     * \return true is yes, false if not.
     * Keep in mind that this method does not enforce a readout but uses the value of the last status readout.
     * You may want to use getMotorPosition() or getCurrentStallGuardReading() to enforce an updated status readout.
     */
    bool isShortToGroundB(void);
    /*!
     * \brief iIs motor channel A connected according to the last status readout.
     * \return true is yes, false if not.
     * Keep in mind that this method does not enforce a readout but uses the value of the last status readout.
     * You may want to use getMotorPosition() or getCurrentStallGuardReading() to enforce an updated status readout.
     */
    bool isOpenLoadA(void);

    /*!
     * \brief iIs motor channel A connected according to the last status readout.
     * \return true is yes, false if not.
     * Keep in mind that this method does not enforce a readout but uses the value of the last status readout.
     * You may want to use getMotorPosition() or getCurrentStallGuardReading() to enforce an updated status readout.
     */
    bool isOpenLoadB(void);

    /*!
     * \brief Is chopper inactive since 2^20 clock cycles - defaults to ~0,08s
     * \return true is yes, false if not.
     * Keep in mind that this method does not enforce a readout but uses the value of the last status readout.
     * You may want to use getMotorPosition() or getCurrentStallGuardReading() to enforce an updated status readout.
     */
    bool isStandStill(void);

    /*!
     *\brief enables or disables the motor driver bridges. If disabled the motor can run freely. If enabled not.
     *\param enabled a bool value true if the motor should be enabled, false otherwise.
     */
    void set_enable(bool enabled);

    /*!
     *\brief checks if the output bridges are enabled. If the bridges are not enabled the motor can run freely
     *\return true if the bridges and by that the motor driver are enabled, false if not.
     *\sa set_enable()
     */
    bool isEnabled();
    
    /*!
     * \brief Reads a given register
     * \param reg_addr any valid register address
     * \sa TMC220X_[GCONF|GSTAT|IOIN|TSTEP|SG_RESULT|MSCNT|CHOPCONF|DRV_STATUS]_REGISTER
     */
    unsigned long readRegister(int8_t reg_addr);

    /*!
     * \brief Manually read out the status register
     * This function sends a byte to the motor driver in order to get the current readout. The parameter read_value
     * selects which value will get returned. If the read_vlaue changes in respect to the previous readout this method
     * automatically send two bytes to the motor: one to set the readout and one to get the actual readout. So this method
     * may take time to send and read one or two bits - depending on the previous readout.
     * \param read_value selects which value to read out (0..3). You can use the defines TMC21X_READOUT_POSITION, TMC21X_READOUT_STALLGUARD_CURRENT
     * \sa TMC21X_READOUT_POSITION, TMC21X_READOUT_STALLGUARD_CURRENT
     */
    // unsigned long readStatus(int8_t read_value);

    /*!
     * \brief Prints out all the information that can be found in the last status read out - it does not force a status readout.
     * The result is printed via Serial
     */
    void dump_status(StreamOutput *stream);
    
    /*!
     * \brief Returns GCONF and DRV register status. Can be used to validate serial connection.
     * The result is printed via Serial
     */
    void get_debug_info(StreamOutput *stream);
    
    bool set_raw_register(StreamOutput *stream, uint32_t reg, uint32_t val);
    bool check_alarm();

    using options_t= std::map<char,int>;

    bool set_options(const options_t& options);
    
    void set_write_only(bool wo);
    //uint8_t calcCRC(uint8_t *buf, uint8_t len);
    uint8_t calc_crc(uint8_t *buf, uint8_t len);
private:
    bool check_error_status_bits(StreamOutput *stream);

    // UART sender
    inline uint32_t transceive220X(uint8_t reg, uint32_t datagram = 0x00000000);
    std::function<int(uint8_t *b, int cnt, uint8_t *r)> serial;

    unsigned int resistor{50}; // current sense resistor value in milliohm
    uint8_t chopper_mode; // StealthChop or SpreadCycle mode

    //driver control register copies to easily set & modify the registers
    uint32_t gconf_register_value;
    uint32_t slaveconf_register_value;
    uint32_t ihold_irun_register_value;
    uint32_t tpowerdown_register_value;
    uint32_t tpwmthrs_register_value;
    uint32_t tcoolthrs_register_value;
    uint8_t sgthrs_register_value;
    uint32_t coolconf_register_value;
    uint32_t chopconf_register_value;
    uint32_t pwmconf_register_value;

    //status values
    int microsteps; //the current number of microsteps

    std::bitset<8> error_reported;

    // only needed for the tuning app report
    struct {
        int8_t blank_time:8;
        int8_t constant_off_time:5; //we need to remember this value in order to enable and disable the motor
        int8_t h_start:4;
        int8_t h_end:4;
        bool started:1; //if the stepper has been started yet
    };

    char designator;

    // to store response CRC before validating
    uint8_t response_crc;
    bool crc_valid = false;
    
    bool write_only = false;
    
    // slave address for TMC2209
    uint8_t slave_addr = 0;
};


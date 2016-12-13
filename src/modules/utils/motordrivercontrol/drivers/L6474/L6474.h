// ensure this library description is only included once
#pragma once
#include <functional>
#include <map>
#include <bitset>
#include "libs/Pin.h"
#include "l6474_target_config.h"
class StreamOutput;

/// Error while initialising the SPI
#define ERROR_0   (0x8000)
/// Error: Bad SPI transaction
#define ERROR_1   (0x8001)

/// Maximum number of steps
#define MAX_STEPS         (0x7FFFFFFF)

/// Maximum frequency of the PWMs in Hz
#define MAX_PWM_FREQ   (10000)

/// Minimum frequency of the PWMs in Hz
#define MIN_PWM_FREQ   (2)

/// Current FW version
#define FW_VERSION (5)

/// L6474 max number of bytes of command & arguments to set a parameter
#define CMD_ARG_MAX_NB_BYTES              (4)

/// L6474 command + argument bytes number for GET_STATUS command
#define CMD_ARG_NB_BYTES_GET_STATUS       (1)

/// L6474 response bytes number
#define RSP_NB_BYTES_GET_STATUS           (2)

/// L6474 value mask for ABS_POS register
#define ABS_POS_VALUE_MASK    ((uint32_t) 0x003FFFFF)

/// L6474 sign bit mask for ABS_POS register
#define ABS_POS_SIGN_BIT_MASK ((uint32_t) 0x00200000)
///TOFF_FAST values for T_FAST register
typedef enum {
  TOFF_FAST_2us = ((uint8_t) 0x00 << 4),
  TOFF_FAST_4us = ((uint8_t) 0x01 << 4),
  TOFF_FAST_6us = ((uint8_t) 0x02 << 4),
  TOFF_FAST_8us = ((uint8_t) 0x03 << 4),
  TOFF_FAST_10us = ((uint8_t) 0x04 << 4),
  TOFF_FAST_12us = ((uint8_t) 0x05 << 4),
  TOFF_FAST_14us = ((uint8_t) 0x06 << 4),
  TOFF_FAST_16us = ((uint8_t) 0x07 << 4),
  TOFF_FAST_18us = ((uint8_t) 0x08 << 4),
  TOFF_FAST_20us = ((uint8_t) 0x09 << 4),
  TOFF_FAST_22us = ((uint8_t) 0x0A << 4),
  TOFF_FAST_24us = ((uint8_t) 0x0B << 4),
  TOFF_FAST_26us = ((uint8_t) 0x0C << 4),
  TOFF_FAST_28us = ((uint8_t) 0x0D << 4),
  TOFF_FAST_30us = ((uint8_t) 0x0E << 4),
  TOFF_FAST_32us = ((uint8_t) 0x0F << 4)
} TOFF_FAST_t;
///FAST_STEP values for T_FAST register
typedef enum {
  FAST_STEP_2us = ((uint8_t) 0x00),
  FAST_STEP_4us = ((uint8_t) 0x01),
  FAST_STEP_6us = ((uint8_t) 0x02),
  FAST_STEP_8us = ((uint8_t) 0x03),
  FAST_STEP_10us = ((uint8_t) 0x04),
  FAST_STEP_12us = ((uint8_t) 0x05),
  FAST_STEP_14us = ((uint8_t) 0x06),
  FAST_STEP_16us = ((uint8_t) 0x07),
  FAST_STEP_18us = ((uint8_t) 0x08),
  FAST_STEP_20us = ((uint8_t) 0x09),
  FAST_STEP_22us = ((uint8_t) 0x0A),
  FAST_STEP_24us = ((uint8_t) 0x0B),
  FAST_STEP_26us = ((uint8_t) 0x0C),
  FAST_STEP_28us = ((uint8_t) 0x0D),
  FAST_STEP_30us = ((uint8_t) 0x0E),
  FAST_STEP_32us = ((uint8_t) 0x0F)
} FAST_STEP_t;
///OCD_TH register
typedef enum {
  OCD_TH_375mA  = ((uint8_t) 0x00),
  OCD_TH_750mA  = ((uint8_t) 0x01),
  OCD_TH_1125mA = ((uint8_t) 0x02),
  OCD_TH_1500mA = ((uint8_t) 0x03),
  OCD_TH_1875mA = ((uint8_t) 0x04),
  OCD_TH_2250mA = ((uint8_t) 0x05),
  OCD_TH_2625mA = ((uint8_t) 0x06),
  OCD_TH_3000mA = ((uint8_t) 0x07),
  OCD_TH_3375mA = ((uint8_t) 0x08),
  OCD_TH_3750mA = ((uint8_t) 0x09),
  OCD_TH_4125mA = ((uint8_t) 0x0A),
  OCD_TH_4500mA = ((uint8_t) 0x0B),
  OCD_TH_4875mA = ((uint8_t) 0x0C),
  OCD_TH_5250mA = ((uint8_t) 0x0D),
  OCD_TH_5625mA = ((uint8_t) 0x0E),
  OCD_TH_6000mA = ((uint8_t) 0x0F)
} OCD_TH_t;
///STEP_MODE register
typedef enum {
  STEP_MODE_STEP_SEL = ((uint8_t) 0x07),
  STEP_MODE_SYNC_SEL = ((uint8_t) 0x70)
} STEP_MODE_Masks_t;
///STEP_SEL field of STEP_MODE register
typedef enum {
  STEP_SEL_1    = ((uint8_t) 0x08),  //full step
  STEP_SEL_1_2  = ((uint8_t) 0x09),  //half step
  STEP_SEL_1_4  = ((uint8_t) 0x0A),  //1/4 microstep
  STEP_SEL_1_8  = ((uint8_t) 0x0B),  //1/8 microstep
  STEP_SEL_1_16 = ((uint8_t) 0x0C)   //1/16 microstep
} STEP_SEL_t;
///SYNC_SEL field of STEP_MODE register
typedef enum {
  SYNC_SEL_1_2    = ((uint8_t) 0x80),
  SYNC_SEL_1      = ((uint8_t) 0x90),
  SYNC_SEL_2      = ((uint8_t) 0xA0),
  SYNC_SEL_4      = ((uint8_t) 0xB0),
  SYNC_SEL_8      = ((uint8_t) 0xC0),
  SYNC_SEL_UNUSED = ((uint8_t) 0xD0)
} SYNC_SEL_t;
///ALARM_EN register
typedef enum {
  ALARM_EN_OVERCURRENT      = ((uint8_t) 0x01),
  ALARM_EN_THERMAL_SHUTDOWN = ((uint8_t) 0x02),
  ALARM_EN_THERMAL_WARNING  = ((uint8_t) 0x04),
  ALARM_EN_UNDERVOLTAGE     = ((uint8_t) 0x08),
  ALARM_EN_SW_TURN_ON       = ((uint8_t) 0x40),
  ALARM_EN_WRONG_NPERF_CMD  = ((uint8_t) 0x80)
} ALARM_EN_t;
///CONFIG register
typedef enum {
  CONFIG_OSC_SEL  = ((uint16_t) 0x0007),
  CONFIG_EXT_CLK  = ((uint16_t) 0x0008),
  CONFIG_EN_TQREG = ((uint16_t) 0x0020),
  CONFIG_OC_SD    = ((uint16_t) 0x0080),
  CONFIG_POW_SR   = ((uint16_t) 0x0300),
  CONFIG_TOFF      = ((uint16_t) 0x7C00)
} CONFIG_Masks_t;
///Clock source option for CONFIG register
typedef enum {
  CONFIG_INT_16MHZ = ((uint16_t) 0x0000),
  CONFIG_INT_16MHZ_OSCOUT_2MHZ   = ((uint16_t) 0x0008),
  CONFIG_INT_16MHZ_OSCOUT_4MHZ   = ((uint16_t) 0x0009),
  CONFIG_INT_16MHZ_OSCOUT_8MHZ   = ((uint16_t) 0x000A),
  CONFIG_INT_16MHZ_OSCOUT_16MHZ  = ((uint16_t) 0x000B),
  CONFIG_EXT_8MHZ_XTAL_DRIVE     = ((uint16_t) 0x0004),
  CONFIG_EXT_16MHZ_XTAL_DRIVE    = ((uint16_t) 0x0005),
  CONFIG_EXT_24MHZ_XTAL_DRIVE    = ((uint16_t) 0x0006),
  CONFIG_EXT_32MHZ_XTAL_DRIVE    = ((uint16_t) 0x0007),
  CONFIG_EXT_8MHZ_OSCOUT_INVERT  = ((uint16_t) 0x000C),
  CONFIG_EXT_16MHZ_OSCOUT_INVERT = ((uint16_t) 0x000D),
  CONFIG_EXT_24MHZ_OSCOUT_INVERT = ((uint16_t) 0x000E),
  CONFIG_EXT_32MHZ_OSCOUT_INVERT = ((uint16_t) 0x000F)
} CONFIG_OSC_MGMT_t;
///External Torque regulation options for CONFIG register
typedef enum {
  CONFIG_EN_TQREG_TVAL_USED = ((uint16_t) 0x0000),
  CONFIG_EN_TQREG_ADC_OUT = ((uint16_t) 0x0020)
} CONFIG_EN_TQREG_t;
///Over Current Shutdown options for CONFIG register
typedef enum {
  CONFIG_OC_SD_DISABLE = ((uint16_t) 0x0000),
  CONFIG_OC_SD_ENABLE  = ((uint16_t) 0x0080)
} CONFIG_OC_SD_t;
/// POW_SR values for CONFIG register
typedef enum {
  CONFIG_SR_320V_us    =((uint16_t)0x0000),
  CONFIG_SR_075V_us    =((uint16_t)0x0100),
  CONFIG_SR_110V_us    =((uint16_t)0x0200),
  CONFIG_SR_260V_us    =((uint16_t)0x0300)
} CONFIG_POW_SR_t;
/// TOFF values for CONFIG register
typedef enum {
  CONFIG_TOFF_004us   = (((uint16_t) 0x01) << 10),
  CONFIG_TOFF_008us   = (((uint16_t) 0x02) << 10),
  CONFIG_TOFF_012us  = (((uint16_t) 0x03) << 10),
  CONFIG_TOFF_016us  = (((uint16_t) 0x04) << 10),
  CONFIG_TOFF_020us  = (((uint16_t) 0x05) << 10),
  CONFIG_TOFF_024us  = (((uint16_t) 0x06) << 10),
  CONFIG_TOFF_028us  = (((uint16_t) 0x07) << 10),
  CONFIG_TOFF_032us  = (((uint16_t) 0x08) << 10),
  CONFIG_TOFF_036us  = (((uint16_t) 0x09) << 10),
  CONFIG_TOFF_040us  = (((uint16_t) 0x0A) << 10),
  CONFIG_TOFF_044us  = (((uint16_t) 0x0B) << 10),
  CONFIG_TOFF_048us  = (((uint16_t) 0x0C) << 10),
  CONFIG_TOFF_052us  = (((uint16_t) 0x0D) << 10),
  CONFIG_TOFF_056us  = (((uint16_t) 0x0E) << 10),
  CONFIG_TOFF_060us  = (((uint16_t) 0x0F) << 10),
  CONFIG_TOFF_064us  = (((uint16_t) 0x10) << 10),
  CONFIG_TOFF_068us  = (((uint16_t) 0x11) << 10),
  CONFIG_TOFF_072us  = (((uint16_t) 0x12) << 10),
  CONFIG_TOFF_076us  = (((uint16_t) 0x13) << 10),
  CONFIG_TOFF_080us  = (((uint16_t) 0x14) << 10),
  CONFIG_TOFF_084us  = (((uint16_t) 0x15) << 10),
  CONFIG_TOFF_088us  = (((uint16_t) 0x16) << 10),
  CONFIG_TOFF_092us  = (((uint16_t) 0x17) << 10),
  CONFIG_TOFF_096us  = (((uint16_t) 0x18) << 10),
  CONFIG_TOFF_100us = (((uint16_t) 0x19) << 10),
  CONFIG_TOFF_104us = (((uint16_t) 0x1A) << 10),
  CONFIG_TOFF_108us = (((uint16_t) 0x1B) << 10),
  CONFIG_TOFF_112us = (((uint16_t) 0x1C) << 10),
  CONFIG_TOFF_116us = (((uint16_t) 0x1D) << 10),
  CONFIG_TOFF_120us = (((uint16_t) 0x1E) << 10),
  CONFIG_TOFF_124us = (((uint16_t) 0x1F) << 10)
} CONFIG_TOFF_t;
///STATUS Register Bit Masks
typedef enum {
  STATUS_HIZ         = (((uint16_t) 0x0001)),
  STATUS_DIR         = (((uint16_t) 0x0010)),
  STATUS_NOTPERF_CMD = (((uint16_t) 0x0080)),
  STATUS_WRONG_CMD   = (((uint16_t) 0x0100)),
  STATUS_UVLO        = (((uint16_t) 0x0200)),
  STATUS_TH_WRN      = (((uint16_t) 0x0400)),
  STATUS_TH_SD       = (((uint16_t) 0x0800)),
  STATUS_OCD         = (((uint16_t) 0x1000))
} STATUS_Masks_t;
///Diretion field of STATUS register
typedef enum {
  STATUS_DIR_FORWARD = (((uint16_t) 0x0001) << 4),
  STATUS_DIR_REVERSE = (((uint16_t) 0x0000) << 4)
} STATUS_DIR_t;
/// Internal L6474 register addresses
typedef enum {
  ABS_POS        = ((uint8_t) 0x01),
  EL_POS         = ((uint8_t) 0x02),
  MARK           = ((uint8_t) 0x03),
  RESERVED_REG01 = ((uint8_t) 0x04),
  RESERVED_REG02 = ((uint8_t) 0x05),
  RESERVED_REG03 = ((uint8_t) 0x06),
  RESERVED_REG04 = ((uint8_t) 0x07),
  RESERVED_REG05 = ((uint8_t) 0x08),
  RESERVED_REG06 = ((uint8_t) 0x15),
  TVAL           = ((uint8_t) 0x09),
  RESERVED_REG07 = ((uint8_t) 0x0A),
  RESERVED_REG08 = ((uint8_t) 0x0B),
  RESERVED_REG09 = ((uint8_t) 0x0C),
  RESERVED_REG10 = ((uint8_t) 0x0D),
  T_FAST         = ((uint8_t) 0x0E),
  TON_MIN        = ((uint8_t) 0x0F),
  TOFF_MIN       = ((uint8_t) 0x10),
  RESERVED_REG11 = ((uint8_t) 0x11),
  ADC_OUT        = ((uint8_t) 0x12),
  OCD_TH         = ((uint8_t) 0x13),
  RESERVED_REG12 = ((uint8_t) 0x14),
  STEP_MODE      = ((uint8_t) 0x16),
  ALARM_EN       = ((uint8_t) 0x17),
  CONFIG         = ((uint8_t) 0x18),
  STATUS         = ((uint8_t) 0x19),
  RESERVED_REG13 = ((uint8_t) 0x1A),
  RESERVED_REG14 = ((uint8_t) 0x1B),
  INEXISTENT_REG = ((uint8_t) 0x1F)
} Registers_t;
/// L6474 command set
typedef enum {
  NOP           = ((uint8_t) 0x00),
  SET_PARAM     = ((uint8_t) 0x00),
  GET_PARAM     = ((uint8_t) 0x20),
  ENABLE        = ((uint8_t) 0xB8),
  DISABLE       = ((uint8_t) 0xA8),
  GET_STATUS    = ((uint8_t) 0xD0),
  RESERVED_CMD1 = ((uint8_t) 0xEB),
  RESERVED_CMD2 = ((uint8_t) 0xF8)
} Commands_t;
/// Device commands
typedef enum {
  RUN_CMD,
  MOVE_CMD,
  SOFT_STOP_CMD,
  NO_CMD
} deviceCommand_t;

typedef enum {
  STEP_MODE_FULL   = ((uint8_t)0x00),
  STEP_MODE_HALF   = ((uint8_t)0x01),
  STEP_MODE_1_4    = ((uint8_t)0x02),
  STEP_MODE_1_8    = ((uint8_t)0x03),
  STEP_MODE_1_16   = ((uint8_t)0x04),
  STEP_MODE_1_32   = ((uint8_t)0x05),
  STEP_MODE_1_64   = ((uint8_t)0x06),
  STEP_MODE_1_128  = ((uint8_t)0x07),
  STEP_MODE_UNKNOW = ((uint8_t)0xFE),
  STEP_MODE_WAVE   = ((uint8_t)0xFF)
} motorStepMode_t;
class L6474
{
public:
    L6474(std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi, char designator);
    void SelectStepMode(uint8_t deviceId, motorStepMode_t stepMod);
    void CmdEnable(uint8_t deviceId);
    void CmdDisable(uint8_t deviceId);
    void SetHome(uint8_t deviceId);
    void ResetAllDevices(void);
    void HardStop(uint8_t deviceId);
    void Init(uint8_t nbDevices);
    uint8_t Tval_Current_to_Par(double Tval);
    uint8_t Tmin_Time_to_Par(double Tmin);
    void SetCurrent(uint8_t deviceId,double c);
    void SetMaxCurrent(uint8_t deviceId,double c);
    uint16_t CmdGetStatus(uint8_t deviceId);
private:
    /// L6474 Device Paramaters structure
    //deviceParams_t devicePrm[MAX_NUMBER_OF_DEVICES];
    char designator;
    void SetDriver(uint8_t d);
    void UnsetDriver();
    void WriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte);
    void SetDeviceParamsToPredefinedValues(void);
    void SetRegisterToPredefinedValues(uint8_t deviceId);
    void SendCommand(uint8_t deviceId, uint8_t param);
    void Reset(void);
    void ReleaseReset(void);
    uint16_t ReadStatusRegister(uint8_t deviceId);
    void CmdSetParam(uint8_t deviceId,uint32_t param,uint32_t value);
    void CmdNop(uint8_t deviceId);
    std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi;
    uint32_t CmdGetParam(uint8_t deviceId, uint32_t param);
    uint8_t numberOfDevices;
    //bool spiPreemtionByIsr;
    //uint8_t spiTxBursts[CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
    //uint8_t spiRxBursts[CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
	Pin Reset1,Reset2,Reset3,Reset4,Reset5,Reset6;
	Pin A0,A1,A2,A3;
	Pin fault;
};


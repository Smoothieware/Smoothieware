#ifndef __TARGET_CONFIG_H
#define __TARGET_CONFIG_H

/// The maximum number of devices in the daisy chain
#define MAX_NUMBER_OF_DEVICES                 (4)

/************************ Speed Profile  *******************************/

/// Acceleration rate in step/s2 for device 0 (must be greater than 0)
#define CONF_PARAM_ACC_DEVICE_0        (160)
/// Acceleration rate in step/s2 for device 1 (must be greater than 0)
#define CONF_PARAM_ACC_DEVICE_1        (160)
/// Acceleration rate in step/s2 for device 2 (must be greater than 0)
#define CONF_PARAM_ACC_DEVICE_2        (160)
/// Acceleration rate in step/s2 for device 3 (must be greater than 0)
#define CONF_PARAM_ACC_DEVICE_3        (160)
/// Acceleration rate in step/s2 for device 4 (must be greater than 0)
#define CONF_PARAM_ACC_DEVICE_4        (160)
/// Acceleration rate in step/s2 for device 5 (must be greater than 0)
#define CONF_PARAM_ACC_DEVICE_5        (160)
/// Acceleration rate in step/s2 for device 6 (must be greater than 0)
#define CONF_PARAM_ACC_DEVICE_6        (160)

    
/// Deceleration rate in step/s2 for device 0 (must be greater than 0)
#define CONF_PARAM_DEC_DEVICE_0        (160)
/// Deceleration rate in step/s2 for device 1 (must be greater than 0)
#define CONF_PARAM_DEC_DEVICE_1        (160)
/// Deceleration rate in step/s2 for device 2 (must be greater than 0)
#define CONF_PARAM_DEC_DEVICE_2        (160)
/// Deceleration rate in step/s2 for device 3 (must be greater than 0)
#define CONF_PARAM_DEC_DEVICE_3        (160)
/// Deceleration rate in step/s2 for device 4 (must be greater than 0)
#define CONF_PARAM_DEC_DEVICE_4        (160)
/// Deceleration rate in step/s2 for device 5 (must be greater than 0)
#define CONF_PARAM_DEC_DEVICE_5        (160)
/// Deceleration rate in step/s2 for device 6 (must be greater than 0)
#define CONF_PARAM_DEC_DEVICE_6        (160)
    
/// Maximum speed in step/s for device 0 (30 step/s < Maximum speed <= 10 000 step/s )
#define CONF_PARAM_MAX_SPEED_DEVICE_0  (1600)
/// Maximum speed in step/s for device 1 (30 step/s < Maximum speed <= 10 000 step/s )
#define CONF_PARAM_MAX_SPEED_DEVICE_1  (1600)
/// Maximum speed in step/s for device 2 (30 step/s < Maximum speed <= 10 000 step/s )
#define CONF_PARAM_MAX_SPEED_DEVICE_2  (1600)
/// Maximum speed in step/s for device 3 (30 step/s < Maximum speed <= 10 000 step/s )
#define CONF_PARAM_MAX_SPEED_DEVICE_3  (1600)
/// Maximum speed in step/s for device 4 (30 step/s < Maximum speed <= 10 000 step/s )
#define CONF_PARAM_MAX_SPEED_DEVICE_4  (1600)
/// Maximum speed in step/s for device 5 (30 step/s < Maximum speed <= 10 000 step/s )
#define CONF_PARAM_MAX_SPEED_DEVICE_5  (1600)
/// Maximum speed in step/s for device 6 (30 step/s < Maximum speed <= 10 000 step/s )
#define CONF_PARAM_MAX_SPEED_DEVICE_6  (1600)
    
/// Minimum speed in step/s for device 0 (30 step/s <= Minimum speed < 10 000 step/s)
#define CONF_PARAM_MIN_SPEED_DEVICE_0  (800)
/// Minimum speed in step/s for device 1 (30 step/s <= Minimum speed < 10 000 step/s)
#define CONF_PARAM_MIN_SPEED_DEVICE_1  (800)
/// Minimum speed in step/s for device 2 (30 step/s <= Minimum speed < 10 000 step/s)
#define CONF_PARAM_MIN_SPEED_DEVICE_2  (800)
/// Minimum speed in step/s for device 3 (30 step/s <= Minimum speed < 10 000 step/s)
#define CONF_PARAM_MIN_SPEED_DEVICE_3  (800)
/// Minimum speed in step/s for device 4 (30 step/s <= Minimum speed < 10 000 step/s)
#define CONF_PARAM_MIN_SPEED_DEVICE_5  (800)
/// Minimum speed in step/s for device 2 (30 step/s <= Minimum speed < 10 000 step/s)
#define CONF_PARAM_MIN_SPEED_DEVICE_6  (800)
/// Minimum speed in step/s for device 2 (30 step/s <= Minimum speed < 10 000 step/s)
#define CONF_PARAM_MIN_SPEED_DEVICE_6  (800)
    
/************************ Phase Current Control *******************************/

// Current value that is assigned to the torque regulation DAC
/// TVAL register value for device 0 (range 31.25mA to 4000mA)
#define CONF_PARAM_TVAL_DEVICE_0  (750)
/// TVAL register value for device 1 (range 31.25mA to 4000mA)
#define CONF_PARAM_TVAL_DEVICE_1  (875)
/// TVAL register value for device 2 (range 31.25mA to 4000mA)
#define CONF_PARAM_TVAL_DEVICE_2 (750)
/// TVAL register value for device 3 (range 31.25mA to 4000mA)
#define CONF_PARAM_TVAL_DEVICE_3 (750)
/// TVAL register value for device 4 (range 31.25mA to 4000mA)
#define CONF_PARAM_TVAL_DEVICE_4  (750)
/// TVAL register value for device 5 (range 31.25mA to 4000mA)
#define CONF_PARAM_TVAL_DEVICE_5  (750)
/// TVAL register value for device 6 (range 31.25mA to 4000mA)
#define CONF_PARAM_TVAL_DEVICE_6  (750)
    
/// Fall time value (T_FAST field of T_FAST register) for device 0 (range 2us to 32us)
#define CONF_PARAM_FAST_STEP_DEVICE_0  (FAST_STEP_12us)
/// Fall time value (T_FAST field of T_FAST register) for device 1 (range 2us to 32us)
#define CONF_PARAM_FAST_STEP_DEVICE_1  (FAST_STEP_12us)
/// Fall time value (T_FAST field of T_FAST register) for device 2 (range 2us to 32us)
#define CONF_PARAM_FAST_STEP_DEVICE_2  (FAST_STEP_12us)
/// Fall time value (T_FAST field of T_FAST register) for device 3 (range 2us to 32us)
#define CONF_PARAM_FAST_STEP_DEVICE_3  (FAST_STEP_12us)
/// Fall time value (T_FAST field of T_FAST register) for device 4 (range 2us to 32us)
#define CONF_PARAM_FAST_STEP_DEVICE_4  (FAST_STEP_12us)
/// Fall time value (T_FAST field of T_FAST register) for device 5 (range 2us to 32us)
#define CONF_PARAM_FAST_STEP_DEVICE_5  (FAST_STEP_12us)
/// Fall time value (T_FAST field of T_FAST register) for device 6 (range 2us to 32us)
#define CONF_PARAM_FAST_STEP_DEVICE_6  (FAST_STEP_12us)
    
/// Maximum fast decay time (T_OFF field of T_FAST register) for device 0 (range 2us to 32us)
#define CONF_PARAM_TOFF_FAST_DEVICE_0  (TOFF_FAST_8us)
/// Maximum fast decay time (T_OFF field of T_FAST register) for device 1 (range 2us to 32us)
#define CONF_PARAM_TOFF_FAST_DEVICE_1  (TOFF_FAST_8us)
/// Maximum fast decay time (T_OFF field of T_FAST register) for device 2 (range 2us to 32us)
#define CONF_PARAM_TOFF_FAST_DEVICE_2  (TOFF_FAST_8us)
/// Maximum fast decay time (T_OFF field of T_FAST register) for device 3 (range 2us to 32us)
#define CONF_PARAM_TOFF_FAST_DEVICE_3  (TOFF_FAST_8us)
/// Maximum fast decay time (T_OFF field of T_FAST register) for device 4 (range 2us to 32us)
#define CONF_PARAM_TOFF_FAST_DEVICE_4  (TOFF_FAST_8us)
/// Maximum fast decay time (T_OFF field of T_FAST register) for device 5 (range 2us to 32us)
#define CONF_PARAM_TOFF_FAST_DEVICE_5  (TOFF_FAST_8us)
/// Maximum fast decay time (T_OFF field of T_FAST register) for device 6 (range 2us to 32us)
#define CONF_PARAM_TOFF_FAST_DEVICE_6  (TOFF_FAST_8us)
    
/// Minimum ON time (TON_MIN register) for device 0 (range 0.5us to 64us)
#define CONF_PARAM_TON_MIN_DEVICE_0 (3)
/// Minimum ON time (TON_MIN register) for device 1 (range 0.5us to 64us)
#define CONF_PARAM_TON_MIN_DEVICE_1 (3)
/// Minimum ON time (TON_MIN register) for device 2 (range 0.5us to 64us)
#define CONF_PARAM_TON_MIN_DEVICE_2 (3)
/// Minimum ON time (TON_MIN register) for device 3 (range 0.5us to 64us)
#define CONF_PARAM_TON_MIN_DEVICE_3 (3)
/// Minimum ON time (TON_MIN register) for device 4 (range 0.5us to 64us)
#define CONF_PARAM_TON_MIN_DEVICE_4 (3)
/// Minimum ON time (TON_MIN register) for device 5 (range 0.5us to 64us)
#define CONF_PARAM_TON_MIN_DEVICE_5 (3)
/// Minimum ON time (TON_MIN register) for device 6 (range 0.5us to 64us)
#define CONF_PARAM_TON_MIN_DEVICE_6 (3)
    
/// Minimum OFF time (TOFF_MIN register) for device 0 (range 0.5us to 64us)
#define CONF_PARAM_TOFF_MIN_DEVICE_0 (21)
/// Minimum OFF time (TOFF_MIN register) for device 1 (range 0.5us to 64us)
#define CONF_PARAM_TOFF_MIN_DEVICE_1 (21)
/// Minimum OFF time (TOFF_MIN register) for device 2 (range 0.5us to 64us)
#define CONF_PARAM_TOFF_MIN_DEVICE_2 (21)
/// Minimum OFF time (TOFF_MIN register) for device 3 (range 0.5us to 64us)
#define CONF_PARAM_TOFF_MIN_DEVICE_3 (21)
/// Minimum OFF time (TOFF_MIN register) for device 4 (range 0.5us to 64us)
#define CONF_PARAM_TOFF_MIN_DEVICE_4 (21)
/// Minimum OFF time (TOFF_MIN register) for device 5 (range 0.5us to 64us)
#define CONF_PARAM_TOFF_MIN_DEVICE_5 (21)
/// Minimum OFF time (TOFF_MIN register) for device 5 (range 0.5us to 64us)
#define CONF_PARAM_TOFF_MIN_DEVICE_6 (21)

/******************************* Others ***************************************/

/// Overcurrent threshold settings for device 0 (OCD_TH register)
#define CONF_PARAM_OCD_TH_DEVICE_0  (OCD_TH_1125mA)
/// Overcurrent threshold settings for device 1 (OCD_TH register)
#define CONF_PARAM_OCD_TH_DEVICE_1  (OCD_TH_1125mA)
/// Overcurrent threshold settings for device 2 (OCD_TH register)
#define CONF_PARAM_OCD_TH_DEVICE_2  (OCD_TH_1125mA)
/// Overcurrent threshold settings for device 3 (OCD_TH register)
#define CONF_PARAM_OCD_TH_DEVICE_3  (OCD_TH_1125mA)
/// Overcurrent threshold settings for device 4 (OCD_TH register)
#define CONF_PARAM_OCD_TH_DEVICE_4  (OCD_TH_1125mA)
/// Overcurrent threshold settings for device 5 (OCD_TH register)
#define CONF_PARAM_OCD_TH_DEVICE_5  (OCD_TH_1125mA)
/// Overcurrent threshold settings for device 6 (OCD_TH register)
#define CONF_PARAM_OCD_TH_DEVICE_6  (OCD_TH_1125mA)

/// Alarm settings for device 0 (ALARM_EN register)
#define CONF_PARAM_ALARM_EN_DEVICE_0  (ALARM_EN_OVERCURRENT |\
                                             ALARM_EN_THERMAL_SHUTDOWN |\
                                             ALARM_EN_THERMAL_WARNING |\
                                             ALARM_EN_UNDERVOLTAGE |\
                                             ALARM_EN_SW_TURN_ON |\
                                             ALARM_EN_WRONG_NPERF_CMD)

///Alarm settings for device 1 (ALARM_EN register)
#define CONF_PARAM_ALARM_EN_DEVICE_1  (ALARM_EN_OVERCURRENT |\
                                             ALARM_EN_THERMAL_SHUTDOWN |\
                                             ALARM_EN_THERMAL_WARNING |\
                                             ALARM_EN_UNDERVOLTAGE |\
                                             ALARM_EN_SW_TURN_ON |\
                                             ALARM_EN_WRONG_NPERF_CMD)

/// Alarm settings for device 2 (ALARM_EN register)
#define CONF_PARAM_ALARM_EN_DEVICE_2  (ALARM_EN_OVERCURRENT |\
                                             ALARM_EN_THERMAL_SHUTDOWN |\
                                             ALARM_EN_THERMAL_WARNING |\
                                             ALARM_EN_UNDERVOLTAGE |\
                                             ALARM_EN_SW_TURN_ON |\
                                             ALARM_EN_WRONG_NPERF_CMD)
/// Alarm settings for device 3 (ALARM_EN register)
#define CONF_PARAM_ALARM_EN_DEVICE_3  (ALARM_EN_OVERCURRENT |\
                                             ALARM_EN_THERMAL_SHUTDOWN |\
                                             ALARM_EN_THERMAL_WARNING |\
                                             ALARM_EN_UNDERVOLTAGE |\
                                             ALARM_EN_SW_TURN_ON |\
                                             ALARM_EN_WRONG_NPERF_CMD)
/// Alarm settings for device 4 (ALARM_EN register)
#define CONF_PARAM_ALARM_EN_DEVICE_4  (ALARM_EN_OVERCURRENT |\
                                             ALARM_EN_THERMAL_SHUTDOWN |\
                                             ALARM_EN_THERMAL_WARNING |\
                                             ALARM_EN_UNDERVOLTAGE |\
                                             ALARM_EN_SW_TURN_ON |\
                                             ALARM_EN_WRONG_NPERF_CMD)
/// Alarm settings for device 5 (ALARM_EN register)
#define CONF_PARAM_ALARM_EN_DEVICE_5  (ALARM_EN_OVERCURRENT |\
                                             ALARM_EN_THERMAL_SHUTDOWN |\
                                             ALARM_EN_THERMAL_WARNING |\
                                             ALARM_EN_UNDERVOLTAGE |\
                                             ALARM_EN_SW_TURN_ON |\
                                             ALARM_EN_WRONG_NPERF_CMD)
/// Alarm settings for device 6 (ALARM_EN register)
#define CONF_PARAM_ALARM_EN_DEVICE_6  (ALARM_EN_OVERCURRENT |\
                                             ALARM_EN_THERMAL_SHUTDOWN |\
                                             ALARM_EN_THERMAL_WARNING |\
                                             ALARM_EN_UNDERVOLTAGE |\
                                             ALARM_EN_SW_TURN_ON |\
                                             ALARM_EN_WRONG_NPERF_CMD)

/// Step selection settings for device 0 (STEP_SEL field of STEP_MODE register)
#define CONF_PARAM_STEP_SEL_DEVICE_0  (STEP_SEL_1_16)
/// Step selection settings for device 1 (STEP_SEL field of STEP_MODE register)
#define CONF_PARAM_STEP_SEL_DEVICE_1  (STEP_SEL_1_16)
/// Step selection settings for device 2 (STEP_SEL field of STEP_MODE register)
#define CONF_PARAM_STEP_SEL_DEVICE_2  (STEP_SEL_1_16)
/// Step selection settings for device 3 (STEP_SEL field of STEP_MODE register)
#define CONF_PARAM_STEP_SEL_DEVICE_3  (STEP_SEL_1_16)
/// Step selection settings for device 4 (STEP_SEL field of STEP_MODE register)
#define CONF_PARAM_STEP_SEL_DEVICE_4  (STEP_SEL_1_16)
/// Step selection settings for device 5 (STEP_SEL field of STEP_MODE register)
#define CONF_PARAM_STEP_SEL_DEVICE_5  (STEP_SEL_1_16)
/// Step selection settings for device 6 (STEP_SEL field of STEP_MODE register)
#define CONF_PARAM_STEP_SEL_DEVICE_6  (STEP_SEL_1_16)

/// Synch. selection settings for device 0 (SYNC_SEL field of STEP_MODE register)
#define CONF_PARAM_SYNC_SEL_DEVICE_0  (SYNC_SEL_1_2)
/// Synch. selection settings for device 1 (SYNC_SEL field of STEP_MODE register)
#define CONF_PARAM_SYNC_SEL_DEVICE_1  (SYNC_SEL_1_2)
/// Synch. selection settings for device 2 (SYNC_SEL field of STEP_MODE register)
#define CONF_PARAM_SYNC_SEL_DEVICE_2  (SYNC_SEL_1_2)
/// Synch. selection settings for device 3 (SYNC_SEL field of STEP_MODE register)
#define CONF_PARAM_SYNC_SEL_DEVICE_3  (SYNC_SEL_1_2)
/// Synch. selection settings for device 4 (SYNC_SEL field of STEP_MODE register)
#define CONF_PARAM_SYNC_SEL_DEVICE_4  (SYNC_SEL_1_2)
/// Synch. selection settings for device 5 (SYNC_SEL field of STEP_MODE register)
#define CONF_PARAM_SYNC_SEL_DEVICE_5  (SYNC_SEL_1_2)
/// Synch. selection settings for device 6 (SYNC_SEL field of STEP_MODE register)
#define CONF_PARAM_SYNC_SEL_DEVICE_6  (SYNC_SEL_1_2)

/// Target Swicthing Period for device 0 (field TOFF of CONFIG register)
#define CONF_PARAM_TOFF_DEVICE_0  (CONFIG_TOFF_044us)
/// Target Swicthing Period for device 1 (field TOFF of CONFIG register)
#define CONF_PARAM_TOFF_DEVICE_1  (CONFIG_TOFF_044us)
/// Target Swicthing Period for device 2 (field TOFF of CONFIG register)
#define CONF_PARAM_TOFF_DEVICE_2  (CONFIG_TOFF_044us)
/// Target Swicthing Period for device 3 (field TOFF of CONFIG register)
#define CONF_PARAM_TOFF_DEVICE_3  (CONFIG_TOFF_044us)
/// Target Swicthing Period for device 4 (field TOFF of CONFIG register)
#define CONF_PARAM_TOFF_DEVICE_4  (CONFIG_TOFF_044us)
/// Target Swicthing Period for device 5 (field TOFF of CONFIG register)
#define CONF_PARAM_TOFF_DEVICE_5  (CONFIG_TOFF_044us)
/// Target Swicthing Period for device 6 (field TOFF of CONFIG register)
#define CONF_PARAM_TOFF_DEVICE_6  (CONFIG_TOFF_044us)

/// Slew rate for device 0 (POW_SR field of CONFIG register)
#define CONF_PARAM_SR_DEVICE_0  (CONFIG_SR_320V_us)
/// Slew rate for device 1 (POW_SR field of CONFIG register)
#define CONF_PARAM_SR_DEVICE_1  (CONFIG_SR_320V_us)
/// Slew rate for device 2 (POW_SR field of CONFIG register)
#define CONF_PARAM_SR_DEVICE_2  (CONFIG_SR_320V_us)
/// Slew rate for device 3 (POW_SR field of CONFIG register)
#define CONF_PARAM_SR_DEVICE_3  (CONFIG_SR_320V_us)
/// Slew rate for device 4 (POW_SR field of CONFIG register)
#define CONF_PARAM_SR_DEVICE_4  (CONFIG_SR_320V_us)
/// Slew rate for device 5 (POW_SR field of CONFIG register)
#define CONF_PARAM_SR_DEVICE_5  (CONFIG_SR_320V_us)
/// Slew rate for device 6 (POW_SR field of CONFIG register)
#define CONF_PARAM_SR_DEVICE_6  (CONFIG_SR_320V_us)

/// Over current shutwdown enabling for device 0 (OC_SD field of CONFIG register)
#define CONF_PARAM_OC_SD_DEVICE_0  (CONFIG_OC_SD_ENABLE)
/// Over current shutwdown enabling for device 1 (OC_SD field of CONFIG register)
#define CONF_PARAM_OC_SD_DEVICE_1  (CONFIG_OC_SD_ENABLE)
/// Over current shutwdown enabling for device 2 (OC_SD field of CONFIG register)
#define CONF_PARAM_OC_SD_DEVICE_2  (CONFIG_OC_SD_ENABLE)
/// Over current shutwdown enabling for device 3 (OC_SD field of CONFIG register)
#define CONF_PARAM_OC_SD_DEVICE_3  (CONFIG_OC_SD_ENABLE)
/// Over current shutwdown enabling for device 4 (OC_SD field of CONFIG register)
#define CONF_PARAM_OC_SD_DEVICE_4  (CONFIG_OC_SD_ENABLE)
/// Over current shutwdown enabling for device 5 (OC_SD field of CONFIG register)
#define CONF_PARAM_OC_SD_DEVICE_5  (CONFIG_OC_SD_ENABLE)
/// Over current shutwdown enabling for device 6 (OC_SD field of CONFIG register)
#define CONF_PARAM_OC_SD_DEVICE_6  (CONFIG_OC_SD_ENABLE)

/// Torque regulation method for device 0 (EN_TQREG field of CONFIG register)
#define CONF_PARAM_TQ_REG_DEVICE_0  (CONFIG_EN_TQREG_TVAL_USED)
///Torque regulation method for device 1 (EN_TQREG field of CONFIG register)
#define CONF_PARAM_TQ_REG_DEVICE_1  (CONFIG_EN_TQREG_TVAL_USED)
/// Torque regulation method for device 2 (EN_TQREG field of CONFIG register)
#define CONF_PARAM_TQ_REG_DEVICE_2  (CONFIG_EN_TQREG_TVAL_USED)
/// Torque regulation method for device 3 (EN_TQREG field of CONFIG register)
#define CONF_PARAM_TQ_REG_DEVICE_3  (CONFIG_EN_TQREG_TVAL_USED)
/// Torque regulation method for device 4 (EN_TQREG field of CONFIG register)
#define CONF_PARAM_TQ_REG_DEVICE_4  (CONFIG_EN_TQREG_TVAL_USED)
/// Torque regulation method for device 5 (EN_TQREG field of CONFIG register)
#define CONF_PARAM_TQ_REG_DEVICE_5  (CONFIG_EN_TQREG_TVAL_USED)
/// Torque regulation method for device 6 (EN_TQREG field of CONFIG register)
#define CONF_PARAM_TQ_REG_DEVICE_6  (CONFIG_EN_TQREG_TVAL_USED)

/// Clock setting for device 0 (OSC_CLK_SEL field of CONFIG register)
#define CONF_PARAM_CLOCK_SETTING_DEVICE_0  (CONFIG_INT_16MHZ)
/// Clock setting for device 1 (OSC_CLK_SEL field of CONFIG register)
#define CONF_PARAM_CLOCK_SETTING_DEVICE_1  (CONFIG_INT_16MHZ)
/// Clock setting for device 2 (OSC_CLK_SEL field of CONFIG register)
#define CONF_PARAM_CLOCK_SETTING_DEVICE_2  (CONFIG_INT_16MHZ)
/// Clock setting for device 3 (OSC_CLK_SEL field of CONFIG register)
#define CONF_PARAM_CLOCK_SETTING_DEVICE_3  (CONFIG_INT_16MHZ)
/// Clock setting for device 4 (OSC_CLK_SEL field of CONFIG register)
#define CONF_PARAM_CLOCK_SETTING_DEVICE_4  (CONFIG_INT_16MHZ)
/// Clock setting for device 5 (OSC_CLK_SEL field of CONFIG register)
#define CONF_PARAM_CLOCK_SETTING_DEVICE_5  (CONFIG_INT_16MHZ)
/// Clock setting for device 6 (OSC_CLK_SEL field of CONFIG register)
#define CONF_PARAM_CLOCK_SETTING_DEVICE_6  (CONFIG_INT_16MHZ)

#endif /* __TARGET_CONFIG_H */

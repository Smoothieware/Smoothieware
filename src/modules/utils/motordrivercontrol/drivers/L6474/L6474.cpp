#include "L6474.h"
#include "mbed.h"
#include "StreamOutput.h"
#include "Kernel.h"
#include "libs/StreamOutputPool.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "ConfigValue.h"
#include "Config.h"
#include "checksumm.h"
#include "l6474_target_config.h"
#define panel_checksum             CHECKSUM("panel")
#define lcd_cs_pin_checksum     CHECKSUM("lcd_cs_pin")
L6474::L6474(std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi,char d) : spi(spi), designator(d)
{
	A0.from_string("2.13");
	A1.from_string("0.11");
	A2.from_string("0.10");
	A3.from_string("4.29");
	A0.as_output();
	A1.as_output();
	A2.as_output();
	A3.as_output();

	Reset1.from_string("1.18"); //X
	Reset2.from_string("1.25"); //Y
	Reset3.from_string("1.30"); //Z
	Reset4.from_string("1.23"); //E
	Reset5.from_string("1.21"); //B
	Reset6.from_string("0.25"); //C

	Reset1.as_output();
	Reset2.as_output();
	Reset3.as_output();
	Reset4.as_output();
	Reset5.as_output();
	Reset6.as_output();

	fault.from_string("2.12");
	fault.as_input();
	Reset();
	UnsetDriver();
}
void L6474::WriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte)
{
	spi(pByteToTransmit,1,pReceivedByte);
}
void L6474::SetDeviceParamsToPredefinedValues(void)
{
	uint32_t i;
	for (i = 0; i < numberOfDevices; i++)
	{
		SetRegisterToPredefinedValues(i);
	}
}
void L6474::SetRegisterToPredefinedValues(uint8_t deviceId)
{
	CmdSetParam(deviceId,ABS_POS,0);
	CmdSetParam(deviceId,EL_POS,0);
	CmdSetParam(deviceId,MARK,0);
	CmdSetParam(deviceId,TVAL,Tval_Current_to_Par(CONF_PARAM_TVAL_DEVICE_0));
	CmdSetParam(deviceId,T_FAST,(uint8_t)CONF_PARAM_TOFF_FAST_DEVICE_0 |(uint8_t)CONF_PARAM_FAST_STEP_DEVICE_0);
	CmdSetParam(deviceId,TON_MIN,Tmin_Time_to_Par(CONF_PARAM_TON_MIN_DEVICE_0));
	CmdSetParam(deviceId,TOFF_MIN,Tmin_Time_to_Par(CONF_PARAM_TOFF_MIN_DEVICE_0));
	CmdSetParam(deviceId,OCD_TH, CONF_PARAM_OCD_TH_DEVICE_0);
	CmdSetParam(deviceId,STEP_MODE,	(uint8_t)CONF_PARAM_STEP_SEL_DEVICE_0 |	(uint8_t)CONF_PARAM_SYNC_SEL_DEVICE_0);
	CmdSetParam(deviceId,ALARM_EN,CONF_PARAM_ALARM_EN_DEVICE_0);
	CmdSetParam(deviceId,CONFIG,(uint16_t)CONF_PARAM_CLOCK_SETTING_DEVICE_0 |
		(uint16_t)CONF_PARAM_TQ_REG_DEVICE_0 |
		(uint16_t)CONF_PARAM_OC_SD_DEVICE_0 |
		(uint16_t)CONF_PARAM_SR_DEVICE_0 |
		(uint16_t)CONF_PARAM_TOFF_DEVICE_0);

}
void L6474::SendCommand(uint8_t deviceId, uint8_t param)
{

		uint8_t rx;
		SetDriver(deviceId);
		WriteBytes(&param, &rx);
		UnsetDriver();
}
void L6474::SetDriver(uint8_t d)
{
	switch(d)
			{
				case 0:
					A0.set(0);
					A1.set(0);
					A2.set(1);
					A3.set(0);
					//THEKERNEL->streams->printf("Driver X Ativo\n");
				break;
				case 1:
					A0.set(1);
					A1.set(0);
					A2.set(1);
					A3.set(0);
					//THEKERNEL->streams->printf("Driver Y Ativo\n");
				break;
				case 2:
					A0.set(0);
					A1.set(1);
					A2.set(0);
					A3.set(0);
					//THEKERNEL->streams->printf("Driver Z Ativo\n");
				break;
				case 3:
					A0.set(1);
					A1.set(1);
					A2.set(0);
					A3.set(0);
					//THEKERNEL->streams->printf("Driver E Ativo\n");
				break;
				case 4:
					A0.set(1);
					A1.set(0);
					A2.set(0);
					A3.set(0);
					//THEKERNEL->streams->printf("Driver A Ativo\n");
				break;
				case 5:
					A0.set(0);
					A1.set(0);
					A2.set(0);
					A3.set(0);
					//THEKERNEL->streams->printf("Driver B Ativo\n");
				break;
			}
		wait_us(5);
}
void L6474::UnsetDriver()
{
	A0.set(1);
	A1.set(1);
	A2.set(1);
	A3.set(1);
}
void L6474::SelectStepMode(uint8_t deviceId, motorStepMode_t stepMod)
{
	uint8_t stepModeRegister;
	STEP_SEL_t l6474StepMod;

	switch (stepMod)
	{
	case STEP_MODE_FULL:
		l6474StepMod = STEP_SEL_1;
		break;
	case STEP_MODE_HALF:
		l6474StepMod = STEP_SEL_1_2;
		break;
	case STEP_MODE_1_4:
		l6474StepMod = STEP_SEL_1_4;
		break;
	case STEP_MODE_1_8:
		l6474StepMod = STEP_SEL_1_8;
		break;
	case STEP_MODE_1_16:
	default:
		l6474StepMod = STEP_SEL_1_16;
		break;
	}

	/* Eventually deactivate motor */
	//if (devicePrm[deviceId].motionState != INACTIVE)
	//{
	//	HardStop(deviceId);
	//}

	/* Read Step mode register and clear STEP_SEL field */
	stepModeRegister = (uint8_t)(0xF8 & CmdGetParam(deviceId, STEP_MODE));

	/* Apply new step mode */
	CmdSetParam(deviceId, STEP_MODE, stepModeRegister | (uint8_t)l6474StepMod);

	/* Reset abs pos register */
	SetHome(deviceId);
}
void L6474::Reset(void)
{
	Reset1.set(0);
	Reset2.set(0);
	Reset3.set(0);
	Reset4.set(0);
	Reset5.set(0);
	Reset6.set(0);
}
void L6474::ReleaseReset(void)
{
	Reset1.set(1);
	Reset2.set(1);
	Reset3.set(1);
	Reset4.set(1);
	Reset5.set(1);
	Reset6.set(1);
}

void L6474::CmdSetParam(uint8_t deviceId,uint32_t param,uint32_t value)
{
	uint32_t i;
	uint8_t maxArgumentNbBytes = 0;
	uint8_t Tx[CMD_ARG_MAX_NB_BYTES];
	uint8_t Rx[CMD_ARG_MAX_NB_BYTES];
	Tx[0] = NOP;
	Tx[1] = NOP;
	Tx[2] = NOP;
	Tx[3] = NOP;

	switch (param)
	{
		case ABS_POS:;
		case MARK:
			Tx[0] = param;
			Tx[1] = (uint8_t)(value >> 16);
			Tx[2] = (uint8_t)(value >> 8);
			maxArgumentNbBytes = 3;
			break;
		case EL_POS:;
		case CONFIG:
			Tx[1] = param;
			Tx[2] = (uint8_t)(value >> 8);
			maxArgumentNbBytes = 2;
			break;
		default:
			Tx[2] = param;
			maxArgumentNbBytes = 1;
	}
	Tx[3] = (uint8_t)(value);
	for (i = CMD_ARG_MAX_NB_BYTES - 1 - maxArgumentNbBytes;i < CMD_ARG_MAX_NB_BYTES;i++)
	{
		SetDriver(deviceId);
		WriteBytes(&Tx[i], &Rx[i]);
		UnsetDriver();
	}
}
void L6474::CmdNop(uint8_t deviceId)
{
	SendCommand(deviceId, NOP);
}
uint16_t L6474::CmdGetStatus(uint8_t deviceId)
{
	uint32_t i;
	uint16_t status;
	uint8_t Tx[CMD_ARG_MAX_NB_BYTES];
	uint8_t Rx[CMD_ARG_MAX_NB_BYTES];

	Tx[0] = GET_STATUS;
	Tx[1] = NOP;
	Tx[2] = NOP;

	Rx[1] = 0;
	Rx[2] = 0;

	for (i = 0; i < CMD_ARG_NB_BYTES_GET_STATUS + RSP_NB_BYTES_GET_STATUS; i++)
	{
		SetDriver(deviceId);
		WriteBytes(&Tx[i], &Rx[i]);
		UnsetDriver();
	}
	status = (Rx[1] << 8) | (Rx[2]);
	 if ((status & STATUS_HIZ) == STATUS_HIZ)
	  {
			THEKERNEL->streams->printf("Saida Desabilitada\n");
	  }
	  /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
	  /* This often occures when a command is sent to the L6474 */
	  /* while it is in HIZ state */
	  if ((status & STATUS_NOTPERF_CMD) == STATUS_NOTPERF_CMD)
	  {
		  THEKERNEL->streams->printf("Comando não pode ser executado\n");
	  }

	  /* Check WRONG_CMD flag: if set, the command does not exist */
	  if ((status & STATUS_WRONG_CMD) == STATUS_WRONG_CMD)
	  {
		  THEKERNEL->streams->printf("Comando Mal Formatado\n");
	  }

	  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
	  if ((status & STATUS_UVLO) == 0)
	  {
		  THEKERNEL->streams->printf("Baixa Tensao\n");
	  }

	  /* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
	  if ((status & STATUS_TH_WRN) == 0)
	  {
		  THEKERNEL->streams->printf("Aviso de temperatura\n");
	  }

	  /* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
	  if ((status & STATUS_TH_SD) == 0)
	  {
		  THEKERNEL->streams->printf("Desligamento por temperatura\n");
	  }

	  /* Check OCD  flag: if not set, there is an overcurrent detection */
	  if ((status & STATUS_OCD) == 0)
	  {
		  THEKERNEL->streams->printf("Sobrecorrente\n");
	  }
	  THEKERNEL->streams->printf("\n\n");
	return (status);
}
uint32_t L6474::CmdGetParam(uint8_t deviceId, uint32_t param)
{
	uint32_t i;
	uint32_t spiRxData;
	uint8_t maxArgumentNbBytes = 0;
	uint8_t Tx[4];
	uint8_t Rx[4];

	switch (param)
	{
		case ABS_POS:;
		case MARK:
			Tx[0] = ((uint8_t)GET_PARAM) | (param);
			maxArgumentNbBytes = 3;
			break;
		case EL_POS:;
		case CONFIG:;
		case STATUS:
			Tx[1] = ((uint8_t)GET_PARAM) | (param);
			maxArgumentNbBytes = 2;
			break;
		default:
			Tx[2] = ((uint8_t)GET_PARAM) | (param);
			maxArgumentNbBytes = 1;
	}

	for (i = CMD_ARG_MAX_NB_BYTES - 1 - maxArgumentNbBytes;i < CMD_ARG_MAX_NB_BYTES;i++)
	{
		SetDriver(deviceId);
		WriteBytes(&Tx[i],&Rx[i]);
		UnsetDriver();
	}

	spiRxData = ((uint32_t)Rx[1] << 16) |(Rx[2] << 8) |(Rx[3]);

	return (spiRxData);
}
void L6474::CmdEnable(uint8_t deviceId)
{
	//THEKERNEL->streams->printf("Ativando driver %d \n",deviceId);
	//if(!fault.get())
	//{
	//	THEKERNEL->streams->printf("Flag de Erro Ativa!\n");
	//	CmdGetStatus(deviceId);
	//}
	SetHome(deviceId);
	CmdGetStatus(deviceId);
	SendCommand(deviceId, ENABLE);
}
void L6474::CmdDisable(uint8_t deviceId)
{
	//THEKERNEL->streams->printf("Desativando driver %d \n",deviceId);
	//if(!fault.get())
	//{
	//	THEKERNEL->streams->printf("Flag de Erro Ativa!\n");
	//	CmdGetStatus(deviceId);
	//}
	SendCommand(deviceId, DISABLE);
	SetHome(deviceId);
	CmdGetStatus(deviceId);
}
void L6474::SetHome(uint8_t deviceId)
{
	CmdSetParam(deviceId, ABS_POS, 0);
	CmdSetParam(deviceId, EL_POS, 0);
	CmdSetParam(deviceId, MARK, 0);
}
void L6474::ResetAllDevices(void)
{
	uint8_t loop;

	for (loop = 0; loop < numberOfDevices; loop++)
		HardStop(loop);
	Reset();
}
void L6474::HardStop(uint8_t deviceId)
{
	CmdDisable(deviceId);
}
void L6474::Init(uint8_t nbDevices)
{
	uint32_t i;
	numberOfDevices = nbDevices;
	wait_us(50);
	ReleaseReset();
	wait_us(50);
	/* Disable L6474 powerstage */
	for (i = 0; i < nbDevices; i++)
	{
		CmdDisable(i);
		/* Get Status to clear flags after start up */
		CmdGetStatus(i);
		SetHome(i);
	}
}
uint8_t L6474::Tval_Current_to_Par(double Tval)
{
  return ((uint8_t)(((Tval - 31.25)/31.25)+0.5));
}
uint8_t L6474::Tmin_Time_to_Par(double Tmin)
{
  return ((uint8_t)(((Tmin - 0.5)*2)+0.5));
}
void L6474::SetCurrent(uint8_t deviceId,double c)
{
    CmdSetParam(deviceId,TVAL,Tval_Current_to_Par(c));
}
void L6474::SetMaxCurrent(uint8_t deviceId,double c)
{

}

#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



void
initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	//configureSensorINA219(0x1000, menuI2cPullupValue);

	deviceStatePointer->signalType  = (kWarpTypeMaskCurrent); //this is kinda pointless
	return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload, uint16_t menuI2cPullupValue) //need to test write command
{
	uint8_t		payloadByte[2];
	uint8_t 	commandByte[1];
	i2c_status_t	status;

	payloadByte[0] = (payload >> 8) & 0xFF;
	payloadByte[1] = payload & 0xFF;

	// SEGGER_RTT_WriteString(0, payload);

	switch (deviceRegister)
	{
		case 0x00: case 0x05:
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	// uint32_t i;
	// for ( i = 0; i < I2C_BUFSIZE; i++ )
	// {
	// 	I2CMasterBuffer[i] = 0x00;
	// }
	//
	// I2CWriteLength = 4;
	// I2CReadLength = 0;
	// I2CMasterBuffer[0] = INA219_ADDRESS;        // I2C device address
	// I2CMasterBuffer[1] = reg;                   // Register
	// I2CMasterBuffer[2] = value >> 8;            // Upper 8-bits
	// I2CMasterBuffer[3] = value & 0xFF;          // Lower 8-bits

	commandByte[0] = deviceRegister;
	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							2,
							gWarpI2cTimeoutMilliseconds);


	if ((status != kStatus_I2C_Success))
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorINA219(uint8_t payloadCONFIG, uint8_t payloadCTRL_REG1, uint16_t menuI2cPullupValue)
{
	SEGGER_RTT_WriteString(0, "!!!---CONFIGURING INA219---!!!\n");
	WarpStatus	i2cWriteStatus1 = kWarpStatusOK;
	WarpStatus	i2cWriteStatus2 = kWarpStatusOK;

	i2cWriteStatus2 = writeSensorRegisterINA219(kWarpSensorOutputRegisterINA219_CONFIG /* register address CALIB */,
							0x399F /* default value */,
							menuI2cPullupValue);

	i2cWriteStatus2 = writeSensorRegisterINA219(kWarpSensorOutputRegisterINA219_CALIB /* register address CALIB */,
							0x1062 /* I have no idea */,
							menuI2cPullupValue);

	return (i2cWriteStatus1 | i2cWriteStatus2);
}

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{

	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;



	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03:
		case 0x04: case 0x05:
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}


	i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceINA219State.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataINA219(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueMSB;
	uint16_t	readSensorRegisterValueLSB;
	int16_t	readSensorRegisterValueCombined;
	float current;
	WarpStatus	i2cReadStatus;

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_CURRENT, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | readSensorRegisterValueLSB;
	//SEGGER_RTT_printf(0, "!! %d : %d!!,", deviceINA219State.i2cBuffer[0], deviceINA219State.i2cBuffer[1]);

	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " -ND-,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, " 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			current *= 4.096;
			SEGGER_RTT_printf(0, " %d,", current);
		}	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_SHUNT, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0x7F) << 8) | readSensorRegisterValueLSB;
	
	if (readSensorRegisterValueMSB & 0xE0 == 0xE0)
	{
		readSensorRegisterValueCombined = -readSensorRegisterValueCombined;
	}
	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " -ND-");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, " 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined);
		}	}
}

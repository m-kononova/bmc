#include "nn_i2c.h"
#include "dbm_pins.h"

#include <string.h>
#include "cmsis_os.h"

// Devices that installed on board
i2c_dev_t i2c_devices[] =
{
		{.namePCB = "DD29",
		 .name = "ADG715BRUZ",
		 .description = "Analog key. Tunes core power supply",
		 .addr = 0x90,
		 .state = READY_ON_POWERON,
		 .channel = CHANNEL_NO,
		 .resValue = 0
		},
		{.namePCB = "DD10",
		 .name = "TPS65263RHBT",
		 .description = "Power supply 1.8V, 1.5V, VDD_RESERVE",
		 .addr = 0xC0,
		 .state = READY_ON_POWERON,
		 .channel = CHANNEL_NO,
		 .resValue = 0
		},
		{.namePCB = "DA1",
		 .name = "PAC1720",
		 .description = "VDD CORE 0.95V",
		 .addr = 0x98,
		 .state = READY_ON_STANDBY,
		 .channel = CHANNEL_1,
		 .resValue = 2 // Milli Omh
		},
		{.namePCB = "DA1",
		 .name = "PAC1720",
		 .description = "VDD STBY 3.30V",
		 .addr = 0x98,
		 .state = READY_ON_STANDBY,
		 .channel = CHANNEL_2,
		 .resValue = 20 // Milli Omh
		},
		{.namePCB = "DA2",
		 .name = "PAC1720",
		 .description = "VDD 1.50V",
		 .addr = 0x9A,
		 .state = READY_ON_STANDBY,
		 .channel = CHANNEL_1,
		 .resValue = 30 // Milli Omh
		},
		{.namePCB = "DA2",
		 .name = "PAC1720",
		 .description = "VDD 1.80V",
		 .addr = 0x9A,
		 .state = READY_ON_STANDBY,
		 .channel = CHANNEL_2,
		 .resValue = 20 // Milli Omh
		},
		{.namePCB = "DA3",
		 .name = "PAC1720",
		 .description = "VDD CLK 3.30V",
		 .addr = 0x9C,
		 .state = READY_ON_STANDBY,
		 .channel = CHANNEL_1,
		 .resValue = 20 // Milli Omh
		},
		{.namePCB = "DA3",
		 .name = "PAC1720",
		 .description = "VDD PLL 0.95V",
		 .addr = 0x9C,
		 .state = READY_ON_STANDBY,
		 .channel = CHANNEL_2,
		 .resValue = 60 // Milli Omh
		},
		{.namePCB = "DA4",
		 .name = "PAC1720",
		 .description = "VDQ CH2 1.20V",
		 .addr = 0x9E,
		 .state = READY_ON_STANDBY,
		 .channel = CHANNEL_1,
		 .resValue = 3 // Milli Omh
		},
		{.namePCB = "DA4",
		 .name = "PAC1720",
		 .description = "VTT CH2 0.60V",
		 .addr = 0x9E,
		 .state = READY_ON_STANDBY,
		 .channel = CHANNEL_2,
		 .resValue = 39 // Milli Omh
		},
		{.namePCB = "DA5",
		 .name = "PAC1720",
		 .description = "VDQ CH1 1.20V",
		 .addr = 0x92,
		 .state = READY_ON_STANDBY,
		 .channel = CHANNEL_1,
		 .resValue = 3 // Milli Omh
		},
		{.namePCB = "DA5",
		 .name = "PAC1720",
		 .description = "VTT CH1 0.60V",
		 .addr = 0x92,
		 .state = READY_ON_STANDBY,
		 .channel = CHANNEL_2,
		 .resValue = 39 // Milli Omh
		},
		{.namePCB = "DA6",
		 .name = "PAC1720",
		 .description = "VPP CH2 2.50V",
		 .addr = 0x96,
		 .state = READY_ON_STANDBY,
		 .channel = CHANNEL_1,
		 .resValue = 30 // Milli Omh
		},
		{.namePCB = "DA6",
		 .name = "PAC1720",
		 .description = "VPP CH1 2.50V",
		 .addr = 0x96,
		 .state = READY_ON_STANDBY,
		 .channel = CHANNEL_2,
		 .resValue = 30 // Milli Omh
		},
		{.namePCB = "DA7",
		 .name = "MCP9804T",
		 .addr = 0x3C,
		 .state = READY_ON_STANDBY
		},
		{.namePCB = "DD1",
		 .name = "Si52147-A01AGMR",
		 .addr = 0xD6,
		 .state = READY_ON_POWERON
		}
};

size_t i2cDevCount = sizeof(i2c_devices) / sizeof(struct i2c_dev);

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void NN_MX_I2C2_Init(void)
{
	/*
	 * Init i2c_2 pins;
	 */
	__HAL_RCC_GPIOF_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = PIN_SMBUS_CLK_CURSE | PIN_SMBUS_SDA_CURSE;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(PORT_SMBUS_CLK_CURSE, &GPIO_InitStruct);

	__HAL_RCC_I2C2_CLK_ENABLE();

	hI2C2.Instance = I2C2;
	hI2C2.Init.ClockSpeed = 100000;
	hI2C2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hI2C2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hI2C2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hI2C2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hI2C2.Init.OwnAddress1 = 0;
	hI2C2.Init.OwnAddress2 = 0;
	hI2C2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	HAL_I2C_Init(&hI2C2);

}

// ---------- ---------- ---------- ---------- ---------- ----------
const i2c_dev_t *getADG715(void)
{
	for(int16_t i = 0; i < i2cDevCount; i++)
	{
		if (strncmp(i2c_devices[i].name, "ADG715", 6) == 0)
		{
			return (&i2c_devices[i]);
		}
	}
	return NULL;
}

// ---------- ---------- ---------- ---------- ---------- ----------
void setCoreVolatage(adgVoltage value)
{
	const i2c_dev_t* adg = getADG715();

	  uint8_t buff = (uint8_t) value;
	  HAL_I2C_Master_Transmit(&hI2C2, adg->addr, &buff,1, pdMS_TO_TICKS(3));
	  buff = 0x0;
	  HAL_I2C_Master_Receive(&hI2C2, adg->addr, &buff,1, pdMS_TO_TICKS(3));
}

// ---------- ---------- ---------- ---------- ---------- ----------
uint8_t readByteBlock(I2C_HandleTypeDef* i2cp, i2caddr_t addr, uint8_t devReg)
{
	/* S | Sl_W_Addr | A | Dev_Reg | A | S | Sl_R_Addr | A | readByte | NA | P */
	uint8_t buf[1] = {0x00};
	HAL_StatusTypeDef result = HAL_OK;

	result = HAL_I2C_Mem_Read(i2cp, addr, devReg, 1, buf, 1, 10);

	if (result == HAL_TIMEOUT)
	{
		// If HAL_TIMEOUT restart interface
	}
	return buf[0];
}

// ---------- ---------- ---------- ---------- ---------- ----------
uint8_t readBlock(I2C_HandleTypeDef* i2cp, i2caddr_t addr, uint8_t devReg, uint8_t *outBuff, uint8_t outBuffSize)
{
	if (outBuff == NULL)
	{
		return 0;
	}

	HAL_StatusTypeDef result = HAL_OK;
	result = HAL_I2C_Mem_Read(i2cp, addr, devReg, 1, outBuff, outBuffSize, 10);

	if (result == HAL_TIMEOUT)
	{
		// If HAL_TIMEOUT restart interface
	}
	return outBuff[0];
}

// ---------- ---------- ---------- ---------- ---------- ----------
void writeByteBlock(I2C_HandleTypeDef* i2cp, i2caddr_t addr, uint8_t devReg, uint8_t byteData)
{
	/* S | Sl_W_Addr | A | Dev_Reg | A | byteData | A | P */
	uint8_t buf[2] = {0x00, 0x00};
	buf[0] = byteData;
	// buf[0] = devReg;
	// buf[1] = byteData;
	HAL_StatusTypeDef result = HAL_OK;
	result = HAL_I2C_Mem_Write(i2cp, addr, devReg, 1, buf, 1, 10);

	if (result == HAL_TIMEOUT)
	{
		// If HAL_TIMEOUT restart interface
// Old code
//		i2cStop(i2cp);
//		if (i2cp->i2c == I2C2)
//		{
//			i2cStart(i2cp, &i2cfg2);
//		}
//		else if (i2cp->i2c == I2C1)
//		{
//			i2cStart(i2cp, &i2cfg1);
//		}
	}
}

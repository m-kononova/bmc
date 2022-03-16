/*
 * nn_commands.c
 *
 *  Created on: 9 ���. 2017 �.
 *      Author: noname
 */

#include "nn_commands.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "stm32f4xx.h"
#include "nn_serial.h"
#include "dbm_pins.h"
#include "pac1720.h"
#include "nn_i2c.h"
#include "cmsis_os.h"
#include "usbd_dfu.h"
#include "stdlib.h"

extern ADC_HandleTypeDef hadc;
extern I2C_HandleTypeDef hI2C2;
extern TaskHandle_t  systemStartThreadHandle;

extern USBD_HandleTypeDef hUsbDeviceFS;

extern size_t i2cDevCount;
extern i2c_dev_t i2c_devices[];

void printSenseMeasurement(void);

//b2 - is a test buffer
//uint8_t b2[1024];

// \33[33m - set to YELLOW
// \33[0m  - reset settings
// !!! Commands names must be written in upper case
// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
static const CLI_Command_Definition_t xHelloTask =
{
	"HELLO", /* The command string to type. */
	"\33[33mhello:\33[0m\33[20G Greeting user.\r\n",
	helloTaskCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
BaseType_t helloTaskCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	strcpy(pcWriteBuffer, "Hello user.\n\r");

	// Task will be called while it returns pdTRUE
	// if you want to execute code only once return pdFALSE
	return pdFALSE;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
static const CLI_Command_Definition_t xPrintTask =
{
	"PRINT", /* The command string to type. */
	"\33[33mprint:\33[0m\33[20G Just printing simple string.\r\n",
	printTaskCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
BaseType_t printTaskCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{

	USBD_DFU_HandleTypeDef   *hdfu;
	hdfu = (USBD_DFU_HandleTypeDef*) hUsbDeviceFS.pDfuClassData;

	sprintf(pcWriteBuffer, "HDFU Dev State: %#X\r\n", hdfu->dev_state);

	// Task will be called while it returns pdTRUE
	// if you want to execute code only once return pdFALSE
	return pdFALSE;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
static const CLI_Command_Definition_t xGenSeqTask =
{
	"GENSEQ", /* The command string to type. */
	"\33[33mgenseq:\33[0m\33[20G Generate sequence of number rising edges.\r\n",
	genSeqTaskCommand, /* The function to run. */
	1 /* No parameters are expected. */
};

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
BaseType_t genSeqTaskCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{


//	const char *pcParameter;
//	BaseType_t xParameterStringLength;
//	static UBaseType_t uxParameterNumber = 1;
//
//	pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, uxParameterNumber, &xParameterStringLength);

	return pdFALSE;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
BaseType_t getCh1Res( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{


	return pdFALSE;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
static const CLI_Command_Definition_t xGetCh1Task =
{
	"G", /* The command string to type. */
	"\33[33mg:\33[0m\33[20G Get channel1 capture result.\r\n",
	getCh1Res, /* The function to run. */
	0 /* No parameters are expected. */
};


// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
BaseType_t spiTaskCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	// To output data from task print This data to pcWriteBuffer;
	//strcat(pcWriteBuffer, "Reading SPI1.\n\r");
	//strcpy(pcWriteBuffer, pcCommandString);

	if ((pcCommandString[4] == 'S') || (pcCommandString[4] == 's'))
	{
		ENABLE_SECURE_FLASH();
		vTaskDelay(pdMS_TO_TICKS(5));
			readFlashId();
			//strcpy(pcWriteBuffer, "Secure flash info.\n\r");
			vTaskDelay(pdMS_TO_TICKS(5));
		DISABLE_FLASH();

	}
	else
	{
		ENABLE_BOOT_FLASH();
		vTaskDelay(pdMS_TO_TICKS(5));
			readFlashId();
			//strcpy(pcWriteBuffer, "Boot flash info.\n\r");
			vTaskDelay(pdMS_TO_TICKS(5));
		DISABLE_FLASH();

	}

	//ENABLE_BOOT_FLASH();
	//DISABLE_FLASH();
	// Task will be called while it returns pdTRUE
	// if you want to execute code only once return pdFALSE
	return pdFALSE;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
static const CLI_Command_Definition_t xSpiTask =
{
	"SPI", /* The command string to type. */
	"\33[33mspi:\33[0m\33[20G Some test spi functions.\r\n",
	spiTaskCommand, /* The function to run. */
	1 /* No parameters are expected. */
};

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
BaseType_t flashBulkErase( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	// To output data from task print This data to pcWriteBuffer;
	//strcat(pcWriteBuffer, "Reading SPI1.\n\r");


	eraseBulk();
	while(readStatus() == S_BUSY){};

	// Task will be called while it returns pdTRUE
	// if you want to execute code only once return pdFALSE
	return pdFALSE;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
static const CLI_Command_Definition_t xFlashBulkErase =
{
	"B", /* The command string to type. */
	"\33[33mb:\33[0m\33[20G Flash bulk erase.\r\n",
	flashBulkErase, /* The function to run. */
	0 /* No parameters are expected. */
};

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
BaseType_t fwVersion( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	// To output data from task print This data to pcWriteBuffer;
	//strcat(pcWriteBuffer, "Reading SPI1.\n\r");
	sprintf(pcWriteBuffer, "FW VERSION %s: %s - %s\r\n", FW_VERSION, __DATE__, __TIME__);

	// Task will be called while it returns pdTRUE
	// if you want to execute code only once return pdFALSE
	return pdFALSE;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
static const CLI_Command_Definition_t xFwVersion =
{
	"V", /* The command string to type. */
	"\33[33mv:\33[0m\33[20G Firmware version.\r\n",
	fwVersion, /* The function to run. */
	0 /* No parameters are expected. */
};

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
BaseType_t getFanSense( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	// To output data from task print This data to pcWriteBuffer;
	//strcat(pcWriteBuffer, "Reading SPI1.\n\r");
	// HAL_ADC_Start(&hadc);
	// uint32_t val = HAL_ADC_GetValue(&hadc);
	// ---------- ---------- ---------- ---------- ---------- ----------
	uint8_t val = getHighLimitStatus(&hI2C2, 0x98);
	float fl = getCurrentHightLimitCh1(&hI2C2, 0x98, 2);

	BaseType_t paramLen;
	const char* modeParam = NULL;
	modeParam = FreeRTOS_CLIGetParameter(pcCommandString, 1, &paramLen);



	char *pEnd = NULL;
	uint8_t current = strtol(modeParam, &pEnd, 10);
	setCurrentHighLimitCh1(&hI2C2, 0x98, 2, current); // Set CORE current limit to CURRENT

	sprintf(pcWriteBuffer, "High limit status: %#X. Current H limit: %.2fA. New limit: %.2fA\r\n", val, fl, (float)current);
	val = getHighLimitStatus(&hI2C2, 0x98);
	setCurrentHighLimitCh1(&hI2C2, 0x98, 2, current); // Set CORE current limit to CURRENT
	osDelay(220);
	val = getHighLimitStatus(&hI2C2, 0x98);

	// Task will be called while it returns pdTRUE
	// if you want to execute code only once return pdFALSE
	return pdFALSE;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
static const CLI_Command_Definition_t xGetFanSense =
{
	"CC", /* The command string to type. */
	"\33[33mcc:\33[0m\33[20G Core high limit cc [current in A].\r\n",
	getFanSense, /* The function to run. */
	1 /* No parameters are expected. */
};

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
BaseType_t pwrOnOff( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	// To output data from task print This data to pcWriteBuffer;
	//strcat(pcWriteBuffer, "XXXXXXXXXXXXXXXX\n\r");
	BaseType_t paramLen;
	const char* modeParam = NULL;
	modeParam = FreeRTOS_CLIGetParameter(pcCommandString, 1, &paramLen);

	char param[4];
	memset(param, 0, 4);
	paramLen = paramLen >= 3? 3 : 2;
	strncpy(param, modeParam, paramLen);

//  Was for some tests.
//	unsigned long a, b;
//	a = 4294953634;
//	b = 4294953634 - 10;
//
//
//	uint8_t* str = (uint8_t*) FreeRTOS_CLIGetOutputBuffer();
//	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
//	sprintf((char*)str, "b - a = %lu, (long)(b - a) = %ld\n", b-a, (long)b-a);
//
//	vSerialPutString(NULL, ( signed char * ) str, ( unsigned short ) strlen( (char*)str ) );
//	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

	// Send notification to task by int
	// 0 - turn off
	// 1 - turn on
	if ((strncmp(param, "ON", 2) == 0) && (paramLen == 2))
	{
		//xTaskNotifyGive(systemStartThreadHandle);
		// 1 - is ONN
		xTaskNotify(systemStartThreadHandle, (uint8_t) 1, eSetValueWithOverwrite);
	}
	else
	{
		// ZERO - is OFF
		xTaskNotify(systemStartThreadHandle, (uint8_t) 0, eSetValueWithOverwrite);
	}
	// Task will be called while it returns pdTRUE
	// if you want to execute code only once return pdFALSE
	return pdFALSE;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
static const CLI_Command_Definition_t xPwrOnOff =
{
	"PWR", /* The command string to type. */
	"\33[33mpwr [on/off]:\33[0m\33[20G Toogling power on/off.\r\n",
	pwrOnOff, /* The function to run. */
	1 /* No parameters are expected. */
};

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
BaseType_t sensMeasure( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	// To output data from task print This data to pcWriteBuffer;
	//strcat(pcWriteBuffer, "XXXXXXXXXXXXXXXX\n\r");
	BaseType_t paramLen;
	const char* timeParam = NULL;
	timeParam = FreeRTOS_CLIGetParameter(pcCommandString, 1, &paramLen);


	uint32_t miliSec = strtol((char*)timeParam, NULL, 10) * 1000;
	if (miliSec <= 10)
		miliSec = 50;

	uint32_t delayInTick = pdMS_TO_TICKS(miliSec);
	uint32_t startTick = osKernelSysTick();
	while (osKernelSysTick() - startTick < delayInTick)
	{
		printSenseMeasurement();
	}

	uint8_t* str = (uint8_t*) FreeRTOS_CLIGetOutputBuffer();
	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

	// Restore cursor position
	uint8_t locBuffSize = 128;
	uint8_t tmpStr[locBuffSize];

	memset(tmpStr, 0, locBuffSize);
	sprintf((char*)tmpStr, "\r\33[u");
	strcat((char*)str, (char*)tmpStr);
	vSerialPutString(NULL, ( signed char * ) str, ( unsigned short ) strlen( (char*)str ) );
	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

	// Task will be called while it returns pdTRUE
	// if you want to execute code only once return pdFALSE
	return pdFALSE;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
static const CLI_Command_Definition_t xSensMeasure =
{
	"SEN", /* The command string to type. */
	"\33[33msen [TTM]:\33[0m\33[20G Power sense measurement (TimeToMonitor in seconds).\r\n",
	sensMeasure, /* The function to run. */
	1 /* No parameters are expected. */
};

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
BaseType_t setCoreVolt( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	// To output data from task print This data to pcWriteBuffer;
	//strcat(pcWriteBuffer, "Reading SPI1.\n\r");

	BaseType_t paramLen;
	const char* modeParam = NULL;
	modeParam = FreeRTOS_CLIGetParameter(pcCommandString, 1, &paramLen);

	char param[5];
	memset(param, 0, 5);
	paramLen = 1; // Set to 1, only 1 digit value possible
	strncpy(param, modeParam, paramLen);

	adgVoltage voltage;
	if (param[0] == '1')
	{
		voltage = V_0v80;
		strncpy(param, "0.80", 5);
	}
	else if (param[0] == '2')
	{
		voltage = V_1v0;
		strncpy(param, "1.00", 5);
	}
	else if (param[0] == '3')
	{
		voltage = V_0v85;
		strncpy(param, "0.85", 5);
	}
	else if (param[0] == '4')
	{
		voltage = V_1v05;
		strncpy(param, "1.05", 5);
	}
	else if (param[0] == '5')
	{
		voltage = V_0v90;
		strncpy(param, "0.90", 5);
	}
	else if (param[0] == '6')
	{
		voltage = V_1v10;
		strncpy(param, "1.10", 5);
	}
	else if (param[0] == '7')
	{
		voltage = V_0v95;
		strncpy(param, "0.95", 5);
	}
	else if (param[0] == '8')
	{
		voltage = V_0v80;
		strncpy(param, "0.80", 5);
	}
	else
	{
		voltage = V_0v95;
		strncpy(param, "0.95", 5);
	}

	sprintf(pcWriteBuffer, "Set volatge to : %s\r\n", param);
	setCoreVolatage(voltage);
	// Task will be called while it returns pdTRUE
	// if you want to execute code only once return pdFALSE
	return pdFALSE;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
static const CLI_Command_Definition_t xSetCoreVolt =
{
	"CV", /* The command string to type. */
	"\33[33mcv [val]:\33[0m\33[20G Set core voltage.\r\n \
	            1 - 0.80V\r\n \
                   2 - 1.00V\r\n \
	            3 - 0.85V\r\n \
	            4 - 1.05V\r\n \
	            5 - 0.90V\r\n \
	            6 - 1.10V\r\n \
	            7 - 0.95V\r\n \
	            8 - 0.80V\r\n",
	setCoreVolt, /* The function to run. */
	1 /* No parameters are expected. */
};

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void vRegisterCommands( void )
{
	/* Register all the command line commands defined immediately above. */
	FreeRTOS_CLIRegisterCommand(&xGenSeqTask);
	FreeRTOS_CLIRegisterCommand(&xHelloTask);
	FreeRTOS_CLIRegisterCommand(&xPrintTask);
	FreeRTOS_CLIRegisterCommand(&xGetCh1Task);
	FreeRTOS_CLIRegisterCommand(&xSpiTask);
	FreeRTOS_CLIRegisterCommand(&xFlashBulkErase);
	FreeRTOS_CLIRegisterCommand(&xFwVersion);
	FreeRTOS_CLIRegisterCommand(&xGetFanSense);
	FreeRTOS_CLIRegisterCommand(&xPwrOnOff);
	FreeRTOS_CLIRegisterCommand(&xSensMeasure);
	FreeRTOS_CLIRegisterCommand(&xSetCoreVolt);
}

/*
 * Additiona functions
 */
void printSenseMeasurement(void)
{
	uint8_t* str = (uint8_t*) FreeRTOS_CLIGetOutputBuffer();
	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

	uint8_t locBuffSize = 128;
	uint8_t tmpStr[locBuffSize];


//	memset(tmpStr, 0, locBuffSize);
//	sprintf((char*)tmpStr, "\r\33[2J");
//	strcat((char*)str, (char*)tmpStr);
//	vSerialPutString(NULL, ( signed char * ) str, ( unsigned short ) strlen( (char*)str ) );
//	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

	uint8_t outPutLines = 2;

	memset(tmpStr, 0, locBuffSize);
	sprintf((char*)tmpStr, "================================================================================\n\r");
	strcat((char*)str, (char*)tmpStr);
	vSerialPutString(NULL, ( signed char * ) str, ( unsigned short ) strlen( (char*)str ) );
	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
	//char test[] = {"asdfasdfa"};

	float value1 = 0;
	for (uint8_t i = 0; i < i2cDevCount; i++)
	{
		if (strncmp(i2c_devices[i].name, "PAC", 3) != 0)
		{
			continue;
		}

		outPutLines += 1; // Count number of sense data that was printed

		value1 = readVoltageValue(&hI2C2, i2c_devices[i].addr, i2c_devices[i].channel);
		memset(tmpStr, 0, locBuffSize);
		sprintf((char*)tmpStr, "%.4s: %#X ", i2c_devices[i].namePCB, i2c_devices[i].addr);
		strcat((char*)str, (char*)tmpStr);

		memset(tmpStr, 0, locBuffSize);
		sprintf((char*)tmpStr, "CH %.2d ", i2c_devices[i].channel);
		strcat((char*)str, (char*)tmpStr);

		memset(tmpStr, 0, locBuffSize);
		sprintf((char*)tmpStr, "%15s: %5.2fV ", i2c_devices[i].description, value1);
		strcat((char*)str, (char*)tmpStr);

		value1 = readCurrentValue(&hI2C2, i2c_devices[i].addr, i2c_devices[i].channel, i2c_devices[i].resValue);
		memset(tmpStr, 0, locBuffSize);
		sprintf((char*)tmpStr, " %5.2fA ", value1);
		strcat((char*)str, (char*)tmpStr);

		memset(tmpStr, 0, locBuffSize);
		sprintf((char*)tmpStr, " - \n\r");
		strcat((char*)str, (char*)tmpStr);

		vSerialPutString(NULL, ( signed char * ) str, ( unsigned short ) strlen( (char*)str ) );
		memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
	}

	memset(tmpStr, 0, locBuffSize);
	sprintf((char*)tmpStr, "================================================================================\n\r");
	strcat((char*)str, (char*)tmpStr);
	vSerialPutString(NULL, ( signed char * ) str, ( unsigned short ) strlen( (char*)str ) );
	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

	// Save cursor position
	memset(tmpStr, 0, locBuffSize);
	sprintf((char*)tmpStr, "\r\33[s");
	strcat((char*)str, (char*)tmpStr);
	vSerialPutString(NULL, ( signed char * ) str, ( unsigned short ) strlen( (char*)str ) );
	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

	// Moves cursor to the beggining of print position
	memset(tmpStr, 0, locBuffSize);
	sprintf((char*)tmpStr, "\r\33[%dA", outPutLines);
	strcat((char*)str, (char*)tmpStr);
	vSerialPutString(NULL, ( signed char * ) str, ( unsigned short ) strlen( (char*)str ) );

	/*
	 *  CLEAR STR to avoid double output
	 *  Task outpust it buffer after call
	 *  str - points to common ACM buffer
	 */
	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
}

/*
 * dbm_tasks.c
 *
 *  Created on: Nov 21, 2018
 *      Author: root
 */

#include "dbm_tasks.h"
#include "dbm_pins.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "MT25Q.h"

#include "string.h"
#include "semphr.h"

#include "usbd_dfu.h"

#include "nn_i2c.h"
#include "pac1720.h"

extern FLASH_DEVICE_OBJECT fdo;

dbm_pin_t monitorPins[] = {
    {
	    .port = PORT_PE_PRSNT0,
	    .pin = PIN_PE_PRSNT0,
	    .defaultState = GPIO_PIN_RESET,
		.enableState = GPIO_PIN_SET,
		.actionPort = PORT_PE_RST_0,
		.actionPin = PIN_PE_RST_0
    },
	{
		.port = PORT_PE_PRSNT1,
		.pin = PIN_PE_PRSNT1,
		.defaultState = GPIO_PIN_RESET,
		.enableState = GPIO_PIN_SET,
		.actionPort = PORT_PE_RST_1,
		.actionPin = PIN_PE_RST_1
	},
	{
	    .port = PORT_PE_PRSNT2,
	    .pin = PIN_PE_PRSNT2,
	    .defaultState = GPIO_PIN_RESET,
		.enableState = GPIO_PIN_SET,
		.actionPort = PORT_PE_RST_2,
		.actionPin = PIN_PE_RST_2
	},
	{
	    .port = PORT_ALERT_CURSE,
	    .pin = PIN_ALERT_CURSE,
	    .defaultState = GPIO_PIN_RESET,
		.enableState = GPIO_PIN_RESET,
		.actionPort = PORT_LED_RED,
		.actionPin = PIN_LED_RED
	}
};


#define FLASH_PAGE_SIZE      256

SemaphoreHandle_t xSpiWrite = NULL;

TaskHandle_t  flashWriteTaskHandle     = NULL;
TaskHandle_t  flashEraseTaskHandle     = NULL;
TaskHandle_t  flashOperationTaskHandle = NULL;
TaskHandle_t  systemStartThreadHandle  = NULL;

QueueHandle_t flashWriteQueue;
uint32_t      flashAddrDest;

uint8_t flashWriteBusyState;

void vInitFlashWriteThread(void)
{
	xSpiWrite = xSemaphoreCreateBinary();
	flashWriteQueue = xQueueCreate(4096, 1);
	xTaskCreate(flashWriteThread, "FlashWriteThread", configMINIMAL_STACK_SIZE, NULL, 0 + 3, &flashWriteTaskHandle);
	xTaskCreate(flashEraseThread, "FlashEraseThread", configMINIMAL_STACK_SIZE, NULL, 0 + 3, &flashEraseTaskHandle);
	// xTaskCreate(flashOperationThread, "FlashOperationThread", configMINIMAL_STACK_SIZE, NULL, 0 + 2, &flashOperationTaskHandle);

	xTaskCreate(systemStartThread, "SystemStartThread", configMINIMAL_STACK_SIZE, NULL, 0 + 2, &systemStartThreadHandle);
	xTaskCreate(ledGreenThread, "LedGreenThread", configMINIMAL_STACK_SIZE, NULL, 0 + 1, NULL);
	xTaskCreate(inputPinMonitorThread, "inputPinMonitorThread", configMINIMAL_STACK_SIZE, NULL, 0 + 1, NULL);
	// xTaskCreate(ledRedThread, "LedRedThread", configMINIMAL_STACK_SIZE, NULL, 0 + 1, NULL);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void inputPinMonitorThread(void *arg)
{
	for(;;)
	{
		uint16_t pinCount = sizeof(monitorPins) / sizeof(dbm_pin_t);
		for (uint16_t i = 0; i < pinCount;i++)
		{
			if (monitorPins[i].defaultState != HAL_GPIO_ReadPin(monitorPins[i].port, monitorPins[i].pin))
			{
				//HAL_GPIO_WritePin(monitorPins[i].port, monitorPins[i].pin, monitorPins[i].enableState);
				vTaskDelay(pdMS_TO_TICKS(1));
				HAL_GPIO_WritePin(monitorPins[i].actionPort, monitorPins[i].actionPin, monitorPins[i].enableState);
			}
			else
			{
				//HAL_GPIO_WritePin(monitorPins[i].port, monitorPins[i].pin, !monitorPins[i].enableState);
				HAL_GPIO_WritePin(monitorPins[i].actionPort, monitorPins[i].actionPin, !monitorPins[i].enableState);
			}
		}
	}
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void flashOperationThread(void *arg)
{
	for(;;)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		// HAL_GPIO_TogglePin(PORT_LED_RED, PIN_LED_RED);
		// vTaskDelay(pdMS_TO_TICKS(10));
		xSemaphoreGive(xSpiWrite);
	}
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void flashEraseThread(void *arg)
{
	// notificationVal contains sector number for erasing
	// Be careful not address, e.g. sector number 1, 2, 3, etc.
	// address converted to sector earlier
	uint32_t notificationVal = 0;
	USBD_DFU_HandleTypeDef *hdfu = NULL;
	for(;;)
	{
		xTaskNotifyWait(0, 0, &notificationVal, portMAX_DELAY);
		hdfu = (USBD_DFU_HandleTypeDef *) notificationVal;

		if (hdfu->alt_setting == 0)
		{
			ENABLE_BOOT_FLASH();
		}
		else
		{
			ENABLE_SECURE_FLASH();
		}

		vTaskDelay(pdMS_TO_TICKS(1));

		//if (hdfu->data_ptr >= ADDR_15MB)
		//{
			fdo.GenOp.Enter4ByteAddressMode();
			fdo.Desc.NumAddrByte = FLASH_4_BYTE_ADDR_MODE;
		//}

		// 4096 - Is flash subsector size. We send subsector number into erase function;
		// We do sector EraseCommand 0xD8 16Mbyte flash has 256 sectors from 0 to 255
		// The sector size is 65536(bytes) or 64Kbyte
		uint32_t sector = hdfu->data_ptr/65536;
		//fourByteAddressing(AM_ENABLE);
		fdo.GenOp.SectorErase(sector);
		//fourByteAddressing(AM_DISABLE);

//		uint8_t* str = (uint8_t*) FreeRTOS_CLIGetOutputBuffer();
//
//		memset(str, 0, strlen( (char*)str ));
//		sprintf((char*)str, "Erase notification val: %d\n\r", sector);
//		vSerialPutString(NULL, ( signed char * ) str, ( unsigned short ) strlen( (char*)str ) );

		//HAL_GPIO_TogglePin(PORT_LED_RED, PIN_LED_RED);
		//vTaskDelay(pdMS_TO_TICKS(30));
		//if (hdfu->data_ptr >= ADDR_15MB)
		//{
			fdo.GenOp.Exit4ByteAddressMode();
			fdo.Desc.NumAddrByte = FLASH_3_BYTE_ADDR_MODE;
		//}

		hdfu->processedCmd = 0;

		vTaskDelay(pdMS_TO_TICKS(1));
		DISABLE_FLASH();
		xSemaphoreGive(xSpiWrite);
	}
}

//static uint8_t taskEnters      = 0;
//static uint8_t taskEntersAfter = 0;
// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void flashWriteThread(void *arg)
{
	uint8_t srcBuf[FLASH_PAGE_SIZE];
	// uint8_t cmpBuf[FLASH_PAGE_SIZE];

	uint16_t bytesToRead   = 0;
	uint32_t flashAddrDest = 0;
	static uint32_t tstCount = 0;

	ParameterType para;
	USBD_DFU_HandleTypeDef *hdfu = NULL;

	uint32_t notificationVal = 0;
	for(;;)
	{
		xTaskNotifyWait(0, 0, &notificationVal, portMAX_DELAY);
		hdfu = (USBD_DFU_HandleTypeDef *) notificationVal;
		hdfu->processedCmd = 4;

		if (hdfu->alt_setting == 0)
		{
			ENABLE_BOOT_FLASH();
		}
		else
		{
			ENABLE_SECURE_FLASH();
		}

		vTaskDelay(pdMS_TO_TICKS(1));

		//flashAddrDest = hdfu->data_ptr;
		//if (flashAddrDest >= ADDR_15MB)
		//{
			fdo.GenOp.Enter4ByteAddressMode();
			fdo.Desc.NumAddrByte = FLASH_4_BYTE_ADDR_MODE;
		//}

		uint16_t dataLength = hdfu->dataLength;
		for (uint16_t i = 0; i < dataLength;)
		{
			if ((dataLength - i) >= FLASH_PAGE_SIZE)
			{
				bytesToRead = FLASH_PAGE_SIZE;
			}
			else
			{
				bytesToRead = dataLength - i;
			}

			flashAddrDest = hdfu->data_ptr + i;
			memcpy(srcBuf, hdfu->buffer.d8 + i, bytesToRead);


			// PROGRAMM BLOCK OF RECEIVED DATA
			para.PageProgram.udAddr = (uint32_t)flashAddrDest; // Flash addr
			para.PageProgram.pArray = srcBuf; // Set source address
			para.PageProgram.udNrOfElementsInArray = bytesToRead;
			fdo.GenOp.DataProgram(PageProgram, &para);

//			uint8_t result = 0;
//			fdo.GenOp.ReadFlagStatusRegister(&result);
//			fdo.GenOp.ReadStatusRegister(&result);

			// READ PROGRAMMED BLOCK FOR CHECK
//			para.Read.udAddr = (uint32_t)flashAddrDest;
//			para.Read.pArray = cmpBuf;
//			para.Read.udNrOfElementsToRead = bytesToRead;
//			fdo.GenOp.DataRead(Read, &para);

			// READ STATUS !!! CHECK USABILITY MAY BE IT COULD BE DELETED
//			fdo.GenOp.ReadFlagStatusRegister(&result);
//			fdo.GenOp.ReadStatusRegister(&result);

			//flashAddrDest += bytesToRead;
			// COMPARE WRITEN AND READ DATA
//			if (memcmp(srcBuf, cmpBuf, bytesToRead) != 0)
//			{
//				HAL_GPIO_WritePin(PORT_LED_RED, PIN_LED_RED, GPIO_PIN_SET);
//			}

			i += bytesToRead;
		}
		//HAL_GPIO_TogglePin(PORT_LED_RED, PIN_LED_RED);
		//vTaskDelay(pdMS_TO_TICKS(3));
		//if (flashAddrDest >= ADDR_15MB)
		//{
			fdo.GenOp.Exit4ByteAddressMode();
			fdo.Desc.NumAddrByte = FLASH_3_BYTE_ADDR_MODE;
		//}
		tstCount++;
		vTaskDelay(pdMS_TO_TICKS(1));
		DISABLE_FLASH();
		hdfu->processedCmd = 0;
		xSemaphoreGive(xSpiWrite);
	}
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void switchPinFunction(void)
{
	HAL_GPIO_TogglePin(PORT_PS_ON, PIN_PS_ON);
	vTaskDelay(pdMS_TO_TICKS(150));

	HAL_GPIO_TogglePin(PORT_EN_VDD1V5, PIN_EN_VDD1V5);
	// HAL_GPIO_TogglePin(PORT_EN_VDD1V8, PIN_EN_VDD1V8);
	//HAL_GPIO_TogglePin(PORT_EN_VDD_RESERVE, PIN_EN_VDD_RESERVE);
	HAL_GPIO_TogglePin(PORT_EN_VDD3V3_CLK, PIN_EN_VDD3V3_CLK);
	HAL_GPIO_TogglePin(PORT_EN_VDD_PLL, PIN_EN_VDD_PLL);

	HAL_GPIO_TogglePin(PORT_VDQ_S3_CH1, PIN_VDQ_S3_CH1);
	HAL_GPIO_TogglePin(PORT_VDQ_S5_CH1, PIN_VDQ_S5_CH1);
	HAL_GPIO_TogglePin(PORT_EN_VPP_CH1, PIN_EN_VPP_CH1);

	HAL_GPIO_TogglePin(PORT_VDQ_S3_CH2, PIN_VDQ_S3_CH2);
	HAL_GPIO_TogglePin(PORT_VDQ_S5_CH2, PIN_VDQ_S5_CH2);
	HAL_GPIO_TogglePin(PORT_EN_VPP_CH2, PIN_EN_VPP_CH2);

	/* ENABLE CPU */
	// 1. Set all resets to start state.
	HAL_GPIO_TogglePin(PORT_RESET_SPSTX8, PIN_RESET_SPSTX8);
	HAL_GPIO_TogglePin(PORT_JTAG_RESET, PIN_JTAG_RESET);
	HAL_GPIO_TogglePin(PORT_PJTAG_RESET, PIN_PJTAG_RESET);

	setCoreVolatage(V_0v95);

	HAL_GPIO_TogglePin(PORT_EN_VDD1V8, PIN_EN_VDD1V8);

	// 2. Enable CORE VDD
	HAL_GPIO_TogglePin(PORT_EN_VDD_CORE, PIN_EN_VDD_CORE);
	vTaskDelay(pdMS_TO_TICKS(7.2));

	// 2. Enable Clock 25MHz
	HAL_GPIO_TogglePin(PORT_EN_CLK25MHZ_3V3, PIN_EN_CLK25MHZ_3V3);
	vTaskDelay(pdMS_TO_TICKS(150));

	// 3. Wait >= 16 cycles
	// 1MS is 25 cycles
	vTaskDelay(pdMS_TO_TICKS(1));


	HAL_GPIO_TogglePin(PORT_EN_HDMI_PWR, PIN_EN_HDMI_PWR);

	HAL_GPIO_TogglePin(PORT_EN_CLK1_XG, PIN_EN_CLK1_XG);
	// It is used as LED_GREEN on DISCO
	HAL_GPIO_TogglePin(PORT_EN_CLK0_XG, PIN_EN_CLK0_XG);
	HAL_GPIO_TogglePin(PORT_EN_CLK27MHZ_3V3, PIN_EN_CLK27MHZ_3V3); // THIS Equial LED_RED on DISCO_BOARD
	HAL_GPIO_TogglePin(PORT_EN_LVDS_VPLL_27M, PIN_EN_LVDS_VPLL_27M);

	// 4. Keep out CPU reset
	HAL_GPIO_TogglePin(PORT_CPU_RESET_BMC, PIN_CPU_RESET_BMC);

	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_2);
	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_3);
	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_4);
	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5);
	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_6);
	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_7);
	// Tested pin toggling
	// HAL_GPIO_TogglePin(PORT_SPI1_MUX, PIN_SPI1_MUX);
	// HAL_GPIO_TogglePin(PORT_BOOT_PROG, PIN_BOOT_PROG);
}

// Old version of power up sequence
//void switchPinFunction(void)
//{
//	HAL_GPIO_TogglePin(PORT_PS_ON, PIN_PS_ON);
//	vTaskDelay(pdMS_TO_TICKS(150));
//
//	HAL_GPIO_TogglePin(PORT_EN_VDD1V5, PIN_EN_VDD1V5);
//	// HAL_GPIO_TogglePin(PORT_EN_VDD1V8, PIN_EN_VDD1V8);
//	//HAL_GPIO_TogglePin(PORT_EN_VDD_RESERVE, PIN_EN_VDD_RESERVE);
//	HAL_GPIO_TogglePin(PORT_EN_VDD3V3_CLK, PIN_EN_VDD3V3_CLK);
//	HAL_GPIO_TogglePin(PORT_EN_VDD_PLL, PIN_EN_VDD_PLL);
//
//	HAL_GPIO_TogglePin(PORT_VDQ_S3_CH1, PIN_VDQ_S3_CH1);
//	HAL_GPIO_TogglePin(PORT_VDQ_S5_CH1, PIN_VDQ_S5_CH1);
//	HAL_GPIO_TogglePin(PORT_EN_VPP_CH1, PIN_EN_VPP_CH1);
//
//	HAL_GPIO_TogglePin(PORT_VDQ_S3_CH2, PIN_VDQ_S3_CH2);
//	HAL_GPIO_TogglePin(PORT_VDQ_S5_CH2, PIN_VDQ_S5_CH2);
//	HAL_GPIO_TogglePin(PORT_EN_VPP_CH2, PIN_EN_VPP_CH2);
//
//	/* ENABLE CPU */
//	// 1. Set all resets to start state.
//	HAL_GPIO_TogglePin(PORT_RESET_SPSTX8, PIN_RESET_SPSTX8);
//	HAL_GPIO_TogglePin(PORT_JTAG_RESET, PIN_JTAG_RESET);
//	HAL_GPIO_TogglePin(PORT_PJTAG_RESET, PIN_PJTAG_RESET);
//
//	setCoreVolatage(V_0v95);
//
//	// 2. Enable CORE VDD
//	HAL_GPIO_TogglePin(PORT_EN_VDD_CORE, PIN_EN_VDD_CORE);
//
//	vTaskDelay(pdMS_TO_TICKS(7.2));
//	HAL_GPIO_TogglePin(PORT_EN_VDD1V8, PIN_EN_VDD1V8);
//
//	// 2. Enable Clock 25MHz
//	HAL_GPIO_TogglePin(PORT_EN_CLK25MHZ_3V3, PIN_EN_CLK25MHZ_3V3);
//	vTaskDelay(pdMS_TO_TICKS(150));
//
//	// 3. Wait >= 16 cycles
//	// 1MS is 25 cycles
//	vTaskDelay(pdMS_TO_TICKS(1));
//
//	// 4. Keep out CPU reset
//	HAL_GPIO_TogglePin(PORT_CPU_RESET_BMC, PIN_CPU_RESET_BMC);
//
//
//	HAL_GPIO_TogglePin(PORT_EN_HDMI_PWR, PIN_EN_HDMI_PWR);
//
//	HAL_GPIO_TogglePin(PORT_EN_CLK1_XG, PIN_EN_CLK1_XG);
//	// It is used as LED_GREEN on DISCO
//	HAL_GPIO_TogglePin(PORT_EN_CLK0_XG, PIN_EN_CLK0_XG);
//	HAL_GPIO_TogglePin(PORT_EN_CLK27MHZ_3V3, PIN_EN_CLK27MHZ_3V3); // THIS Equial LED_RED on DISCO_BOARD
//	HAL_GPIO_TogglePin(PORT_EN_LVDS_VPLL_27M, PIN_EN_LVDS_VPLL_27M);
//
//	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_2);
//	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_3);
//	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_4);
//	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5);
//	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_6);
//	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_7);
//	// Tested pin toggling
//	// HAL_GPIO_TogglePin(PORT_SPI1_MUX, PIN_SPI1_MUX);
//	// HAL_GPIO_TogglePin(PORT_BOOT_PROG, PIN_BOOT_PROG);
//}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void systemStartThread(void *arg)
{
	for(;;)
	{
		uint32_t notificationVal = 0;
		xTaskNotifyWait(0, 0, &notificationVal, portMAX_DELAY);

		/*
		 * FOR PS_ON - OFF STATE is GPIO_PIN_SET
		 */
		volatile GPIO_PinState curState = HAL_GPIO_ReadPin(PORT_PS_ON, PIN_PS_ON);

		// notificationVal == 3 - hardware button was pressed
		// notificationVal == 0 - pwr off command received
		// notificationVal == 1 - pwr on  command received
		if (notificationVal == 3)
		{
			switchPinFunction();
			HAL_GPIO_WritePin(PORT_LED_RED, PIN_LED_RED, GPIO_PIN_RESET);
		}
		else if ((notificationVal == 0) && (curState == GPIO_PIN_RESET))
		{
			/*
			 * GPIO_PIN_RESET - it means system is ON
			 * notificationVal == 0 - we get command to turn off
			 */
			switchPinFunction();
		}
		else if ((notificationVal == 1) && (curState == GPIO_PIN_SET))
		{
			/*
			 * GPIO_PIN_SET - it means system is OFF
			 * notificationVal == 1 - we get command to turn onb
			 */
			switchPinFunction();
		}


	}
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void ledGreenThread(void *arg)
{
	for(;;)
	{
		static int per = 15;
		static int on = 0;
		static int off = 0;
		off = per - on;
		//		static int seck = 500;
//		if (seck <= 0)
//			seck = 500;
		static int cycle = 0;
		for (cycle = 0; cycle <= 4; cycle++)
		{
			HAL_GPIO_TogglePin(PORT_LED_GREEN, PIN_LED_GREEN);
			vTaskDelay(on);
			HAL_GPIO_TogglePin(PORT_LED_GREEN, PIN_LED_GREEN);
			vTaskDelay(off);
		}
		on++;
		off = per - on;

		if (on >= per)
		{
			on = 0;
			off = per - on;
			HAL_GPIO_WritePin(PORT_LED_GREEN, PIN_LED_GREEN, GPIO_PIN_RESET);
			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void ledRedThread(void *arg)
{
	for(;;)
	{

	}
}



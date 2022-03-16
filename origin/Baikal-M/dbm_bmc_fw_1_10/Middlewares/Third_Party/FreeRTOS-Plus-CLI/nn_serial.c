/*
 * nn_serial.c
 *
 *  Created on: 8 ���. 2017 �.
 *      Author: noname
 */
#include "nn_serial.h"
#include "semphr.h"

SemaphoreHandle_t xRxSemph = NULL;

extern USBD_HandleTypeDef hUsbDeviceFS;
extern SemaphoreHandle_t transmitUsb;

#define MAX_SEND_DELAY    50
//volatile uint16_t curLen = 0;

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void vSerialPutString(xComPortHandle xPort, signed char *  message, uint16_t len)
{
	xSemaphoreTake(transmitUsb, pdMS_TO_TICKS(MAX_SEND_DELAY));
	//curLen = len;
	CDC_Transmit_FS((uint8_t*)message, len);
	//HAL_UART_Transmit_DMA(xPort, (uint8_t*) message, len);
	//while((xPort->State != HAL_UART_STATE_READY) && (xPort->State != HAL_UART_STATE_BUSY_RX));
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
// Remove it seems to me I don't use it.
void vSendNewLine(xComPortHandle xPort)
{
	uint8_t newLine = '\n';
	uint8_t carRet  = '\r';
	HAL_UART_Transmit_DMA(xPort,  &newLine, 1);
	HAL_UART_Transmit_DMA(xPort,  &carRet, 1);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
BaseType_t xSerialGetChar(xComPortHandle port, signed char * symbol, TickType_t delay)
{
	static uint32_t pos = 0;

	if (xSemaphoreTake(xRxSemph, 1) == pdPASS)
	{
	   USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) hUsbDeviceFS.pCdcClassData;

	   //*symbol = *hcdc->RxBuffer;
	   if (pos < hcdc->RxLength)
	   {
		   *symbol = hcdc->RxBuffer[pos];
		   pos++;
		   xSemaphoreGive(xRxSemph);
		   return pdPASS;
	   }
	   else
	   {
		   pos = 0;
		   hcdc->RxLength = 0;
		   xSemaphoreTake(xRxSemph, 0);
		   return pdFAIL;
	   }

	}
	else
	{
		xSemaphoreTake(xRxSemph, 0);
		return pdFAIL;
	}
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void xSerialPutChar(xComPortHandle xPort, signed char symbol,TickType_t delay)
{
	//while((xPort->State != HAL_UART_STATE_READY) && (xPort->State != HAL_UART_STATE_BUSY_RX));
	//HAL_UART_Transmit_DMA(xPort, (uint8_t*) &symbol, 1);
	xSemaphoreTake(transmitUsb, pdMS_TO_TICKS(MAX_SEND_DELAY));
	CDC_Transmit_FS((uint8_t*) &symbol, 1);
}

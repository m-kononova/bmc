#include "nn_spi.h"

#include "stm32f4xx_hal.h"
#include "string.h"

#include "MT25Q.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "FreeRTOS_CLI.h"
#include "nn_serial.h"

#include "dbm_pins.h"

FLASH_DEVICE_OBJECT fdo;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

uint8_t buf2[128];


/* write buffer */
uint8_t wbuffer[16] = {
	0x2F, 0xEF, 0xFE, 0xED, 0xBE, 0xEF, 0xFE, 0xED,
	0xBE, 0xEF, 0xFE, 0xED, 0xBE, 0xEF, 0xFE, 0xED
};

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void NN_MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

   /* DMA interrupt init */
   /* DMA2_Stream0_IRQn interrupt configuration */
   HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
   /* DMA2_Stream3_IRQn interrupt configuration */
   HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void NN_MX_SPI1_Init(void)
{
	/* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    // hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    //hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; //Maximum working speed but hard to debug
    //hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
    	_Error_Handler(__FILE__, __LINE__);
    }

    NN_SpiEnable();
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void SPI1_IRQHandler(void)
{

}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
// Stream 0 is used for RX channel
void DMA2_Stream0_IRQHandler(void)
{
//	hdma_spi1_rx.Instance->CR  &= (~DMA_IT_TC);
//	hdma_spi1_rx.State = HAL_DMA_STATE_READY;
//	hdma_spi1_rx.Instance->CR  &= (~DMA_IT_TE);
//	hdma_spi1_rx.Instance->CR  &= (~DMA_IT_DME);
	HAL_DMA_IRQHandler(&hdma_spi1_rx);
//	testVar++;
//	if (hspi1.State == HAL_SPI_STATE_READY)
//	{
//		HAL_SPI_DeInit(&hspi1);
//		BaseType_t val;
//		//xSemaphoreGiveFromISR(xSpiMutex, &val);
//	}
	//hspi1.State = HAL_SPI_STATE_RESET;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
// Stream 3 is used for TX channel
void DMA2_Stream3_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
ReturnType NN_FlashInit(void)
{
	ReturnType ret;

	// Flash connected through multiplexor
	// it is important enable multiplexor to
	// get access to flash.
	ENABLE_BOOT_FLASH();
    	ret = Driver_Init(&fdo);

    	fdo.GenOp.Exit4ByteAddressMode();
    	fdo.GenOp.ReadFlagStatusRegister(&ret);
    	if ((ret & 0x1) == 0x1)
    		fdo.Desc.NumAddrByte = FLASH_4_BYTE_ADDR_MODE;
    	else
    		fdo.Desc.NumAddrByte = FLASH_3_BYTE_ADDR_MODE;
    DISABLE_FLASH();

	ENABLE_SECURE_FLASH();
    	ret = Driver_Init(&fdo);

    	fdo.GenOp.Exit4ByteAddressMode();
    	fdo.GenOp.ReadFlagStatusRegister(&ret);
    	if ((ret & 0x1) == 0x1)
    		fdo.Desc.NumAddrByte = FLASH_4_BYTE_ADDR_MODE;
    	else
    		fdo.Desc.NumAddrByte = FLASH_3_BYTE_ADDR_MODE;
    DISABLE_FLASH();
    /*
     * If we have problems during Init, returns Flash_WrongType
     */
    return ret;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void NN_SpiEnable(void)
{
	SPI_TypeDef* SPI = hspi1.Instance;
	SPI->CR1 |= SPI_CR1_SPE;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void NN_SpiDisable(void)
{
	SPI_TypeDef* SPI = hspi1.Instance;
	SPI->CR1 &= ~SPI_CR1_SPE;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
//void readFlashId(uint8_t *buf)
void readFlashId(void)
{
	uint8_t* str = (uint8_t*) FreeRTOS_CLIGetOutputBuffer();
	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

	Driver_Init(&fdo);

	sprintf((char*)str, "Id: 0x%X\n\r", (uint)fdo.Desc.FlashId);
	vSerialPutString(NULL, ( signed char * ) str, ( unsigned short ) strlen( (char*)str ) );
	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

	sprintf((char*)str, "Unique Id: 0x");
	char locStr[5];
	for(uint8_t i = 0; i < 17; i++)
	{
		memset(locStr, 0, 5);
		sprintf(locStr, "%x", fdo.Desc.FlashUniqueId[i]);
		strcat((char*)str, locStr);
	}
	strcat((char*)str, "\n\r");
	vSerialPutString(NULL, ( signed char * ) str, ( unsigned short ) strlen( (char*)str ) );
	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

	sprintf((char*)str, "Size: %d bytes\n\r", (uint)fdo.Desc.FlashSize);
	vSerialPutString(NULL, ( signed char * ) str, ( unsigned short ) strlen( (char*)str ) );
	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

	sprintf((char*)str, "Page size: %d bytes\n\r", (uint)fdo.Desc.FlashPageSize);
	vSerialPutString(NULL, ( signed char * ) str, ( unsigned short ) strlen( (char*)str ) );
	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

	sprintf((char*)str, "Page count: %d\n\r", (uint)fdo.Desc.FlashPageCount);
	vSerialPutString(NULL, ( signed char * ) str, ( unsigned short ) strlen( (char*)str ) );
	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

	sprintf((char*)str, "Current address mode: %d bytes\n\r", (uint)fdo.Desc.NumAddrByte);
	vSerialPutString(NULL, ( signed char * ) str, ( unsigned short ) strlen( (char*)str ) );
	memset(str, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void NN_SpiSSEnable(Bool_t useSS)
{
	if (useSS == B_TRUE)
		HAL_GPIO_WritePin(SS_PORT, SS_PIN, GPIO_PIN_RESET);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void NN_SpiSSDisable(Bool_t useSS)
{
	if (useSS == B_TRUE)
		HAL_GPIO_WritePin(SS_PORT, SS_PIN, GPIO_PIN_SET);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void writeData(uint8_t *src, uint16_t srcLen, Bool_t useSS)
{
	if (srcLen == 0)
	{
		return;
	}

	SPI_TypeDef* SPI = hspi1.Instance;

	NN_SpiSSEnable(useSS);

	while (srcLen > 0)
	{
		SPI->DR = (uint32_t)(*src);
		while((SPI->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE) {}

		src += sizeof(uint8_t);
		srcLen--;
	}

	while((SPI->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY) {}

	// This we need. We must clear DR register by reading.
	// It could contain not valid data.
	SPI->DR;

	NN_SpiSSDisable(useSS);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void readData(uint8_t *dest, uint16_t destLen, Bool_t useSS)
{
	if (destLen == 0)
	{
		return;
	}

	SPI_TypeDef* SPI = hspi1.Instance;

	// If SPI->DR cleared earlier by reading we don't need to wait RXNE,
	// because it would ZERO. Waiting next read operation.
	// while((SPI->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE) {}

	// Reset Overrun flag
	while((SPI->SR & SPI_FLAG_OVR) == SPI_FLAG_OVR)
	{
		SPI->DR;
		SPI->SR;
	}

	NN_SpiSSEnable(useSS);

	while(destLen > 0)
	{
		SPI->DR = 0xA5;
		while((SPI->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE) {}

		while((SPI->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE) {}
		(*dest) = SPI->DR;

		dest += sizeof(uint8_t);
		destLen--;
	}
	while((SPI->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY) {}

	NN_SpiSSDisable(useSS);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void writeEnable(void)
{
	uint8_t cmd = FF_WRITE_ENABLE;
	writeData(&cmd, 1, B_TRUE);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
// Depricated
void eraseSector(uint32_t sector)
{
	writeEnable();

	sector *= 0x10000;

//	uint8_t cmd[4];
//	cmd[0] = FF_ERASE_SECTOR;
//	cmd[1] = (uint8_t)(sector >> 16);
//	cmd[2] = (uint8_t)(sector >> 8);
//	cmd[3] = (uint8_t)(sector);

	uint8_t cmd[5];
	cmd[0] = FF_ERASE_SECTOR;
	cmd[1] = (uint8_t)(sector >> 24);
	cmd[2] = (uint8_t)(sector >> 16);
	cmd[3] = (uint8_t)(sector >> 8);
	cmd[4] = (uint8_t)(sector);

	NN_SpiSSEnable(B_TRUE);
		writeData(cmd, 5, B_FALSE);
	NN_SpiSSDisable(B_TRUE);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void eraseBulk(void)
{
	writeEnable();

	uint8_t cmd[1];
	cmd[0] = SPI_FLASH_INS_BE;

	NN_SpiSSEnable(B_TRUE);
		writeData(cmd, 1, B_FALSE);
	NN_SpiSSDisable(B_TRUE);
}
// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
Status_t readStatus(void)
{
	// 0x05h - read status register
	// 0x70h - read flag status register

	// We interesting by 2 bits
	// 0 - Write in progress:  0 - ready; 1 - busy
	// 1 - Write enable latch: 0 - cleared; 1 - set
	// If this two bits 0 flash is ready

	Status_t res = S_BUSY;

	// buf - set to busy state; If we read 0 in bit [0:1]
	// it means flash ready
	uint8_t cmd = 0x05, buf = 0x03;
	NN_SpiSSEnable(B_TRUE);
		writeData(&cmd, 1, B_FALSE);
		readData(&buf, 1, B_FALSE);
	NN_SpiSSDisable(B_TRUE);

	if ((buf & 0x03) == 0x00)
		return S_READY;

	return res;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
uint8_t readStatusFlags(void)
{
	// 0x05h - read status register
	// 0x70h - read flag status register

	// buf - set to busy state; If we read 0 in bit [0:1]
	// it means flash ready
	uint8_t cmd = 0x70, buf = 0xFF;
	NN_SpiSSEnable(B_TRUE);
		writeData(&cmd, 1, B_FALSE);
		readData(&buf, 1, B_FALSE);
	NN_SpiSSDisable(B_TRUE);

	// 0x80 - is OK value; NO FALTS;
	if ((buf & 0x03) == 0x00)
		return buf;

	return buf;
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void flashReadId(uint8_t *dest)
{
	uint8_t cmd = FF_READ_ID;
	NN_SpiSSEnable(B_TRUE);
		writeData(&cmd, 1, B_FALSE);
		readData(dest, 20, B_FALSE);
	NN_SpiSSDisable(B_TRUE);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void flashRead(uint8_t *dest, uint16_t destLen, uint32_t flashAddr, uint8_t cmdCode)
{
//	uint8_t cmd[4];
//	cmd[0] = cmdCode; // For majority of operations it must be FF_READ = 0x03
//	cmd[1] = (uint8_t) (flashAddr >> 16);
//	cmd[2] = (uint8_t) (flashAddr >> 8);
//	cmd[3] = (uint8_t) (flashAddr);

	uint8_t cmd[5];
	cmd[0] = cmdCode; // FOUR BYTE PAGE PROGRAMM
	cmd[1] = (uint8_t) (flashAddr >> 24);
	cmd[2] = (uint8_t) (flashAddr >> 16);
	cmd[3] = (uint8_t) (flashAddr >> 8);
	cmd[4] = (uint8_t) (flashAddr);

	int sendLength = sizeof(cmd);
	if (cmdCode == 0x5A) // 0x5A - read serial discovery parameter
	{
		// 0x5A - accepts ONLY 3 byte address;
		sendLength = sendLength - 1;
	}

	NN_SpiSSEnable(B_TRUE);
		writeData(cmd, sendLength, B_FALSE);
		readData(dest, destLen, B_FALSE);
	NN_SpiSSDisable(B_TRUE);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void fourByteAddressing(AddressingMode_t mode)
{
	uint8_t cmd = 0;
	cmd = (mode == AM_ENABLE) ? 0xB7: 0xE9;

	writeEnable();

	NN_SpiSSEnable(B_TRUE);
		writeData(&cmd, 1, B_FALSE);
	NN_SpiSSDisable(B_TRUE);
}

// ---------- ---------- ---------- ---------- ---------- ---------- ---------- ----------
void flashWrite(uint8_t *src, uint16_t srcLen, uint32_t flashAddr)
{
//	uint8_t cmd[4];
//	cmd[0] = FF_PAGE_PROGRAM;
//	cmd[1] = (uint8_t) (flashAddr >> 16);
//	cmd[2] = (uint8_t) (flashAddr >> 8);
//	cmd[3] = (uint8_t) (flashAddr);

	uint8_t cmd[5];
	cmd[0] = 0x2; // FOUR BYTE PAGE PROGRAMM
	cmd[1] = (uint8_t) (flashAddr >> 24);
	cmd[2] = (uint8_t) (flashAddr >> 16);
	cmd[3] = (uint8_t) (flashAddr >> 8);
	cmd[4] = (uint8_t) (flashAddr);

	// fourByteAddressing(AM_ENABLE);

	writeEnable();

	NN_SpiSSEnable(B_TRUE);
		writeData(cmd, 5, B_FALSE);
		writeData(src, srcLen, B_FALSE);
	NN_SpiSSDisable(B_TRUE);

	// fourByteAddressing(AM_DISABLE);
}

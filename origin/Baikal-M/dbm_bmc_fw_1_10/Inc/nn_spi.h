#ifndef NN_SPI_H
#define NN_SPI_H

#include "stdint.h"
#include "MT25Q.h"

#include "stm32f4xx_hal.h"

#define SS_PIN                         GPIO_PIN_4
#define SS_PORT                        GPIOA

// FLASH COMMANDS CODES
#define FF_WRITE_ENABLE                0x06
#define FF_WRITE_DISABLE               0x04
#define FF_ERASE_SECTOR                0xD8
#define FF_READ                        0x03
#define FF_READ_ID                     0x9F
#define FF_PAGE_PROGRAM                0x02

typedef enum Bool
{
	B_FALSE = 0,
	B_TRUE = 1
} Bool_t;

typedef enum Status
{
	S_READY,
	S_BUSY
} Status_t;

typedef enum AddressingMode
{
	AM_DISABLE = 0,
	AM_ENABLE = 1
}AddressingMode_t;

void NN_MX_DMA_Init(void);
void NN_MX_SPI1_Init(void);
ReturnType NN_FlashInit(void);

void NN_SpiEnable(void);
void NN_SpiDisable(void);

/* Returns Id into buf
 * buf must be allocated before using readFlashId;
 * FOR N25Q128 buf must be at least 20 bytes
 */
void readFlashId(void);

// Functions for RAW data operation in the line
// useSS = B_TRUE - chip select SET in the beginning, RESET in the end of write
// useSS = B_FALSE - just send data;
void writeData(uint8_t *src, uint16_t srcLen, Bool_t useSS);
void readData(uint8_t *dest, uint16_t destLen, Bool_t useSS);

// FLASH OPERATIONS
void writeEnable(void);
void eraseSector(uint32_t sector);
void eraseBulk(void);
Status_t readStatus(void);
uint8_t  readStatusFlags(void);
void flashReadId(uint8_t *dest);
void flashRead(uint8_t *dest, uint16_t destLen, uint32_t flashAddr, uint8_t cmdCode);
void flashWrite(uint8_t *src, uint16_t srcLen, uint32_t flashAddr);
void fourByteAddressing(AddressingMode_t mode);


// Inner functions later move them to private secion
void NN_SpiSSEnable(Bool_t useSS);
void NN_SpiSSDisable(Bool_t useSS);

#endif //NN_SPI_H

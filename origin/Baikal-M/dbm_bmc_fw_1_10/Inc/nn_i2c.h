#ifndef NN_I2C_H
#define NN_I2C_H

#include "stm32f4xx_hal.h"
#include "nn_i2c_types.h"

I2C_HandleTypeDef hI2C2;


void NN_MX_I2C2_Init(void);
const i2c_dev_t *getADG715(void);

void setCoreVolatage(adgVoltage value);

// This function starts with S condition and finishes with P condition
// use this function if you need to read ONLY one byte.
// you can't use it reading block by one byte
uint8_t readByteBlock(I2C_HandleTypeDef* i2cp, i2caddr_t addr, uint8_t devReg);

/* Use this function to read block of bites where outBuffSize is the total size you
 * want to read.
 * Function dosen't allocate out array, you should worry about it
 */
uint8_t readBlock(I2C_HandleTypeDef* i2cp, i2caddr_t addr, uint8_t devReg, uint8_t *outBuff, uint8_t outBuffSize);

// This function starts with S condition and finishes with P condition
// this function sends ONLY one byte to device
void writeByteBlock(I2C_HandleTypeDef* i2cp, i2caddr_t addr, uint8_t devReg, uint8_t byteData);

#endif // NN_I2C_H

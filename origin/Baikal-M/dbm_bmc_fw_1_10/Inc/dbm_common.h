#ifndef DBM_COMMON_H
#define DBM_COMMON_H

#include "stdint.h"
#include "dbm_pins.h"

void nnInitFanSense(void);

struct flashBuffer_t
{
	uint8_t buff[4096]; // Buffer for 4 trasfer by USB dfu
	uint8_t numOfBytes;
};

#define INIT_PIN_AS_OUT(__port__, __pin__, __state__) \
	do \
    { \
		GPIO_InitTypeDef GPIO_InitStruct; \
		GPIO_InitStruct.Pin = __pin__; \
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; \
		GPIO_InitStruct.Pull = GPIO_PULLDOWN; \
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; \
		HAL_GPIO_WritePin(__port__, __pin__, __state__); \
		HAL_GPIO_Init(__port__, &GPIO_InitStruct); \
    } \
    while(0U)

#define INIT_PIN_AS_INPUT(__port__, __pin__) \
	do \
    { \
		GPIO_InitTypeDef GPIO_InitStruct; \
		GPIO_InitStruct.Pin = __pin__; \
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT; \
		GPIO_InitStruct.Pull = GPIO_PULLDOWN; \
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; \
		HAL_GPIO_Init(__port__, &GPIO_InitStruct); \
    } \
    while(0U)

#define INIT_PIN_AS_IT_RISING(__port__, __pin__) \
	do \
    { \
		GPIO_InitTypeDef GPIO_InitStruct; \
		GPIO_InitStruct.Pin = __pin__; \
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; \
		GPIO_InitStruct.Pull = GPIO_PULLDOWN; \
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; \
		HAL_GPIO_Init(__port__, &GPIO_InitStruct); \
    } \
    while(0U)

#endif  // DBM_COMMON_H

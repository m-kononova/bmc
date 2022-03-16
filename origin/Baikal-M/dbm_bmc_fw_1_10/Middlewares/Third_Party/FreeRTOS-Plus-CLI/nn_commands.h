/*
 * nn_commands.h
 *
 *  Created on: 9 ���. 2017 �.
 *      Author: noname
 */

#ifndef NN_COMMANDS_H_
#define NN_COMMANDS_H_

#define LFLASH_KEY1 0x45670123U
#define LFLASH_KEY2 0xCDEF89ABU

#define PAGE127ADR  0x0801FC00U

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "nn_spi.h"

void vRegisterCommands( void );


BaseType_t helloTaskCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t printTaskCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t genSeqTaskCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t getCh1Res( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

#endif /* NN_COMMANDS_H_ */

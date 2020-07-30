#ifndef __BASE_H__
#define __BASE_H__

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"
#include "timers.h"
#include "croutine.h"
#include "stream_buffer.h"

typedef struct ModbusBaseType_t
{
    SemaphoreHandle_t xSemTXTC;
    SemaphoreHandle_t xSemRXRC;
    SemaphoreHandle_t xSemLock;
    void (*pvLock)(struct ModbusBaseType_t *pxModbusBase);
    void (*pvUnLock)(struct ModbusBaseType_t *pxModbusBase);

    UART_HandleTypeDef *huart;
    uint8_t (*pucSend)(struct ModbusBaseType_t *pxModbusBase, uint8_t *pucDat, uint16_t usSize);   
    uint8_t (*pucRead)(struct ModbusBaseType_t *pxModbusBase, uint8_t *pucDat, uint16_t *pusSize); 
    void (*pvIRQ)(struct ModbusBaseType_t *pxModbusBase);

    const uint16_t usTXLen;
    uint8_t *pucTXBuf;
    uint16_t usTXCnt;
    const uint16_t usRXLen;
    uint8_t *pucRXBuf;
    uint16_t usRXCnt;
    uint32_t ulTXDelay;
    uint32_t ulRXDelay;

    uint8_t ucAddr;

    uint16_t usCoilStartAddr;
    uint16_t usCoilSize;
    uint16_t *pusCoilBuf;

    uint16_t usDiscInStartAddr;
    uint16_t usDiscInSize;
    uint16_t *pusDiscInBuf;

    uint16_t usRegInStartAddr;
    uint16_t usRegInSize;
    uint16_t *pusRegInBuf;

    uint16_t usRegHoldStartAddr;
    uint16_t usRegHoldSize;
    uint16_t *pusRegHoldBuf;
} ModbusBaseType_t;

extern uint8_t ucSerialInit(struct ModbusBaseType_t *pxModbusBase);

#endif

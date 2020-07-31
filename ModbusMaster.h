/*
*********************************************************************************************************
*
*	模块名称 : MODEBUS 通信模块 (主机程序）
*	文件名称 : modbus_host.h
*	版    本 : V1.4
*	说    明 : 头文件
*
*	Copyright (C), 2015-2016, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#ifndef __MODBUSMASTER__
#define __MODBUSMASTER__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "ModbusBase.h"

typedef struct ModbusMaster_t
{
	ModbusBaseType_t *pxModbusBase;
	uint8_t (*pucInit)(struct ModbusBaseType_t *pxModbusBase);
	uint16_t usReg01H;
	uint16_t usReg02H;
	uint16_t usReg03H;
	uint16_t usReg04H;
	uint16_t usReg05H;
	uint16_t usReg06H;
	uint16_t usReg10H;
	uint8_t ucRegNum;
	uint8_t ucRXErrNum;
} ModbusMaster_t;

extern void ModbusMasterInit(ModbusMaster_t *pxModbusMaster);
extern uint8_t ModbusMasterReadParam01H(ModbusMaster_t *pxModbusMaster, uint16_t usRegAddr, uint16_t usRegNum);
extern uint8_t ModbusMasterReadParam02H(ModbusMaster_t *pxModbusMaster, uint16_t usRegAddr, uint16_t usRegNum);
extern uint8_t ModbusMasterReadParam03H(ModbusMaster_t *pxModbusMaster, uint16_t usRegAddr, uint16_t usRegNum);
extern uint8_t ModbusMasterReadParam04H(ModbusMaster_t *pxModbusMaster, uint16_t usRegAddr, uint16_t usRegNum);
extern uint8_t ModbusMasterWriteParam05H(ModbusMaster_t *pxModbusMaster, uint16_t usRegAddr, uint16_t usRegValue);
extern uint8_t ModbusMasterWriteParam06H(ModbusMaster_t *pxModbusMaster, uint16_t usRegAddr, uint16_t usRegValue);
extern uint8_t ModbusMasterWriteParam10H(ModbusMaster_t *pxModbusMaster, uint16_t usRegAddr, uint8_t usRegNum, uint16_t *pusRegBuf);

/***************************************************************************************************/
extern uint8_t ModbusMasterRead03H(ModbusMaster_t *pxModbusMaster, uint16_t usAdd, uint16_t *pusBuf, uint16_t usSize);
extern uint8_t ModbusMasterWrite06H(ModbusMaster_t *pxModbusMaster, uint16_t usAdd, uint16_t usDat);
extern uint8_t ModbusMasterWrite10H(ModbusMaster_t *pxModbusMaster, uint16_t usAdd, uint16_t *pusBuf, uint16_t usSize);
/*****************************************************************************************************/
#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/

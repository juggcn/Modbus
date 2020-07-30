#ifndef __MODBUSSLAVE_H__
#define __MODBUSSLAVE_H__

#include "ModbusBase.h"

typedef struct ModbusSlave_t
{
	ModbusBaseType_t *pxModbusBase;
	uint8_t (*pucInit)(struct ModbusBaseType_t *pxModbusBase);
	uint8_t ucRspCode;
	uint8_t (*pucRegInFun)(struct ModbusSlave_t *pxModbusSlave, uint16_t usAddr, uint16_t *pusValue, uint16_t usLen);
	uint8_t (*pucRegHoldFun)(struct ModbusSlave_t *pxModbusSlave, uint16_t usAddr, uint16_t *pusValue, uint16_t usLen, uint8_t ucrw);
} ModbusSlave_t;

extern void ModbusSlaveInit(ModbusSlave_t *pxModbusSlave);
extern void ModbusSlavePoll(ModbusSlave_t *pxModbusSlave);
extern uint8_t ModbusSlavePollCallBack(ModbusSlave_t *pxModbusSlave);  //外部用户使用，为解决一些特殊要求

#endif

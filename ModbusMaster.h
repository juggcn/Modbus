#ifndef __MODBUSMASTER__
#define __MODBUSMASTER__

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

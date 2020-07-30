/*
*********************************************************************************************************
*
*	模块名称 : MODSBUS通信程序 （主机）
*	文件名称 : modbus_host.c
*	版    本 : V1.4
*	说    明 : 无线通信程序。通信协议基于MODBUS
*	修改记录 :
*		版本号  日期        作者    说明
*       V1.4   2015-11-28 修改协议
*
*	Copyright (C), 2015-2016, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#include "ModbusMaster.h"

/* RTU 应答代码 */
#define RSP_OK 0x00				   /* 成功 */
#define RSP_ERR_CMD 0x01		   /* 不支持的功能码 */
#define RSP_ERRusRagAddr_ADDR 0x02 /* 寄存器地址错误 */
#define RSP_ERRusRagValue 0x03	   /* 数据值域错误 */
#define RSP_ERR_WRITE 0x04		   /* 写入失败 */

// CRC 高位字节值表
static const uint8_t s_CRCHi[] = {
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40};
// CRC 低位字节值表
static const uint8_t s_CRCLo[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
	0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
	0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
	0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
	0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
	0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
	0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
	0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
	0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
	0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
	0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
	0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
	0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
	0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
	0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
	0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
	0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
	0x43, 0x83, 0x41, 0x81, 0x80, 0x40};

/*
*********************************************************************************************************
*	函 数 名: CRC16Modbus
*	功能说明: 计算CRC。 用于Modbus协议。
*	形    参: _pBuf : 参与校验的数据
*			  _usLen : 数据长度
*	返 回 值: 16位整数值。 对于Modbus ，此结果高字节先传送，低字节后传送。
*
*   所有可能的CRC值都被预装在两个数组当中，当计算报文内容时可以简单的索引即可；
*   一个数组包含有16位CRC域的所有256个可能的高位字节，另一个数组含有低位字节的值；
*   这种索引访问CRC的方式提供了比对报文缓冲区的每一个新字符都计算新的CRC更快的方法；
*
*  注意：此程序内部执行高/低CRC字节的交换。此函数返回的是已经经过交换的CRC值；也就是说，该函数的返回值可以直接放置
*        于报文用于发送；
*********************************************************************************************************
*/
static uint16_t CRC16Modbus(uint8_t *_pBuf, uint16_t _usLen)
{
	uint8_t ucCRCHi = 0xFF; /* 高CRC字节初始化 */
	uint8_t ucCRCLo = 0xFF; /* 低CRC 字节初始化 */
	uint16_t usIndex;		/* CRC循环中的索引 */

	while (_usLen--)
	{
		usIndex = ucCRCHi ^ *_pBuf++; /* 计算CRC */
		ucCRCHi = ucCRCLo ^ s_CRCHi[usIndex];
		ucCRCLo = s_CRCLo[usIndex];
	}
	return ((uint16_t)ucCRCHi << 8 | ucCRCLo);
}

/*
*********************************************************************************************************
*	函 数 名: BEBufToUint16
*	功能说明: 将2字节数组(大端Big Endian次序，高字节在前)转换为16位整数
*	形    参: _pBuf : 数组
*	返 回 值: 16位整数值
*
*   大端(Big Endian)与小端(Little Endian)
*********************************************************************************************************
*/
static uint16_t BEBufToUint16(uint8_t *_pBuf)
{
	return (((uint16_t)_pBuf[0] << 8) | _pBuf[1]);
}
/*********************************************************************************************/

/* 保存每个从机的计数器值 */

static void ModbusMaster01H(ModbusMaster_t *pxModbusMaster);
static void ModbusMaster02H(ModbusMaster_t *pxModbusMaster);
static void ModbusMaster03H(ModbusMaster_t *pxModbusMaster);
static void ModbusMaster04H(ModbusMaster_t *pxModbusMaster);
static void ModbusMaster05H(ModbusMaster_t *pxModbusMaster);
static void ModbusMaster06H(ModbusMaster_t *pxModbusMaster);
static void ModbusMaster10H(ModbusMaster_t *pxModbusMaster);

void ModbusMasterInit(ModbusMaster_t *pxModbusMaster)
{
	pxModbusMaster->pucInit(pxModbusMaster->pxModbusBase);
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMasterSendAckWithCRC
*	功能说明: 发送应答,自动加CRC.  
*	形    参: 无。发送数据在 pxModbusMaster->ucTXBuf[], [pxModbusMaster->usTXCnt
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusMasterSendAckWithCRC(ModbusMaster_t *pxModbusMaster)
{
	uint16_t crc;
	crc = CRC16Modbus(pxModbusMaster->pxModbusBase->pucTXBuf, pxModbusMaster->pxModbusBase->usTXCnt);
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = crc >> 8;
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = crc;
	pxModbusMaster->pxModbusBase->pucSend(pxModbusMaster->pxModbusBase, pxModbusMaster->pxModbusBase->pucTXBuf, pxModbusMaster->pxModbusBase->usTXCnt);
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMasterAnalyzeApp
*	功能说明: 分析应用层协议。处理应答。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusMasterAnalyzeApp(ModbusMaster_t *pxModbusMaster)
{
	switch (pxModbusMaster->pxModbusBase->pucRXBuf[1]) /* 第2个字节 功能码 */
	{
	case 0x01: /* 读取线圈状态 */
		ModbusMaster01H(pxModbusMaster);
		break;

	case 0x02: /* 读取输入状态 */
		ModbusMaster02H(pxModbusMaster);
		break;

	case 0x03: /* 读取保持寄存器 在一个或多个保持寄存器中取得当前的二进制值 */
		ModbusMaster03H(pxModbusMaster);
		break;

	case 0x04: /* 读取输入寄存器 */
		ModbusMaster04H(pxModbusMaster);
		break;

	case 0x05: /* 强制单线圈 */
		ModbusMaster05H(pxModbusMaster);
		break;

	case 0x06: /* 写单个寄存器 */
		ModbusMaster06H(pxModbusMaster);
		break;

	case 0x10: /* 写多个寄存器 */
		ModbusMaster10H(pxModbusMaster);
		break;

	default:
		break;
	}
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMasterSend01H
*	功能说明: 发送01H指令，查询1个或多个保持寄存器
*	形    参: ucAddr : 从站地址
*			  usRagAddr : 寄存器编号
*			  usRagNum : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusMasterSend01H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagNum)
{
	pxModbusMaster->pxModbusBase->usTXCnt = 0;
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = pxModbusMaster->pxModbusBase->ucAddr; /* 从站地址 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = 0x01;									/* 功能码 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr >> 8;						/* 寄存器编号 高字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr;							/* 寄存器编号 低字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum >> 8;						/* 寄存器个数 高字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum;								/* 寄存器个数 低字节 */

	ModbusMasterSendAckWithCRC(pxModbusMaster); /* 发送数据，自动加CRC */

	pxModbusMaster->ucRegNum = usRagNum;  /* 寄存器个数 */
	pxModbusMaster->usReg01H = usRagAddr; /* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMasterSend02H
*	功能说明: 发送02H指令，读离散输入寄存器
*	形    参: ucAddr : 从站地址
*			  usRagAddr : 寄存器编号
*			  usRagNum : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusMasterSend02H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagNum)
{
	pxModbusMaster->pxModbusBase->usTXCnt = 0;
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = pxModbusMaster->pxModbusBase->ucAddr; /* 从站地址 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = 0x02;									/* 功能码 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr >> 8;						/* 寄存器编号 高字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr;							/* 寄存器编号 低字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum >> 8;						/* 寄存器个数 高字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum;								/* 寄存器个数 低字节 */

	ModbusMasterSendAckWithCRC(pxModbusMaster); /* 发送数据，自动加CRC */

	pxModbusMaster->ucRegNum = usRagNum;  /* 寄存器个数 */
	pxModbusMaster->usReg02H = usRagAddr; /* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMasterSend03H
*	功能说明: 发送03H指令，查询1个或多个保持寄存器
*	形    参: ucAddr : 从站地址
*			  usRagAddr : 寄存器编号
*			  usRagNum : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusMasterSend03H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagNum)
{
	pxModbusMaster->pxModbusBase->usTXCnt = 0;
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = pxModbusMaster->pxModbusBase->ucAddr; /* 从站地址 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = 0x03;									/* 功能码 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr >> 8;						/* 寄存器编号 高字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr;							/* 寄存器编号 低字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum >> 8;						/* 寄存器个数 高字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum;								/* 寄存器个数 低字节 */

	ModbusMasterSendAckWithCRC(pxModbusMaster); /* 发送数据，自动加CRC */

	pxModbusMaster->ucRegNum = usRagNum;  /* 寄存器个数 */
	pxModbusMaster->usReg03H = usRagAddr; /* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMasterSend04H
*	功能说明: 发送04H指令，读输入寄存器
*	形    参: ucAddr : 从站地址
*			  usRagAddr : 寄存器编号
*			  usRagNum : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusMasterSend04H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagNum)
{
	pxModbusMaster->pxModbusBase->usTXCnt = 0;
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = pxModbusMaster->pxModbusBase->ucAddr; /* 从站地址 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = 0x04;									/* 功能码 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr >> 8;						/* 寄存器编号 高字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr;							/* 寄存器编号 低字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum >> 8;						/* 寄存器个数 高字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum;								/* 寄存器个数 低字节 */

	ModbusMasterSendAckWithCRC(pxModbusMaster); /* 发送数据，自动加CRC */

	pxModbusMaster->ucRegNum = usRagNum;  /* 寄存器个数 */
	pxModbusMaster->usReg04H = usRagAddr; /* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMasterSend05H
*	功能说明: 发送05H指令，写强置单线圈
*	形    参: ucAddr : 从站地址
*			  usRagAddr : 寄存器编号
*			  usRagValue : 寄存器值,2字节
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusMasterSend05H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagValue)
{
	pxModbusMaster->pxModbusBase->usTXCnt = 0;
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = pxModbusMaster->pxModbusBase->ucAddr; /* 从站地址 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = 0x05;									/* 功能码 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr >> 8;						/* 寄存器编号 高字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr;							/* 寄存器编号 低字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagValue >> 8;						/* 寄存器值 高字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagValue;							/* 寄存器值 低字节 */

	ModbusMasterSendAckWithCRC(pxModbusMaster); /* 发送数据，自动加CRC */

	pxModbusMaster->ucRegNum = 1;
	pxModbusMaster->usReg05H = usRagAddr;
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMasterSend06H
*	功能说明: 发送06H指令，写1个保持寄存器
*	形    参: ucAddr : 从站地址
*			  usRagAddr : 寄存器编号
*			  usRagValue : 寄存器值,2字节
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusMasterSend06H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagValue)
{
	pxModbusMaster->pxModbusBase->usTXCnt = 0;
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = pxModbusMaster->pxModbusBase->ucAddr; /* 从站地址 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = 0x06;									/* 功能码 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr >> 8;						/* 寄存器编号 高字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr;							/* 寄存器编号 低字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagValue >> 8;						/* 寄存器值 高字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagValue;							/* 寄存器值 低字节 */

	ModbusMasterSendAckWithCRC(pxModbusMaster); /* 发送数据，自动加CRC */

	pxModbusMaster->ucRegNum = 1;
	pxModbusMaster->usReg06H = usRagAddr;
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMasterSend10H
*	功能说明: 发送10H指令，连续写多个保持寄存器. 最多一次支持23个寄存器。
*	形    参: ucAddr : 从站地址
*			  usRagAddr : 寄存器编号
*			  usRagNum : 寄存器个数n (每个寄存器2个字节) 值域
*			  pusRagBuf : n个寄存器的数据。长度 = 2 * n
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusMasterSend10H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint8_t usRagNum, uint16_t *pusRagBuf)
{
	uint16_t i;
	uint8_t *buf = (uint8_t *)pusRagBuf;

	pxModbusMaster->pxModbusBase->usTXCnt = 0;
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = pxModbusMaster->pxModbusBase->ucAddr; /* 从站地址 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = 0x10;									/* 从站地址 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr >> 8;						/* 寄存器编号 高字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr;							/* 寄存器编号 低字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum >> 8;						/* 寄存器个数 高字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum;								/* 寄存器个数 低字节 */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = 2 * usRagNum;							/* 数据字节数 */

	for (i = 0; i < usRagNum; i++)
	{
		if (pxModbusMaster->pxModbusBase->usTXCnt > (pxModbusMaster->pxModbusBase->usTXLen - 2))
		{
			return; /* 数据超过缓冲区超度，直接丢弃不发送 */
		}
		pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = buf[2 * i + 1]; /* 后面的数据长度 */
		pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = buf[2 * i];
	}

	ModbusMasterSendAckWithCRC(pxModbusMaster); /* 发送数据，自动加CRC */

	pxModbusMaster->ucRegNum = usRagNum;
	pxModbusMaster->usReg10H = usRagAddr;
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMasterPoll
*	功能说明: 接收控制器指令. 1ms 响应时间。
*	形    参: 无
*	返 回 值: 0 表示无数据 1表示收到正确命令
*********************************************************************************************************
*/
static void ModbusMasterPoll(ModbusMaster_t *pxModbusMaster)
{
	if (pxModbusMaster->pxModbusBase->pucRXBuf[0] != pxModbusMaster->pxModbusBase->ucAddr || pxModbusMaster->pxModbusBase->usRXCnt < 4)
		goto err_ret;
	/* 计算CRC校验 */
	if (CRC16Modbus(pxModbusMaster->pxModbusBase->pucRXBuf, pxModbusMaster->pxModbusBase->usRXCnt) != 0)
		goto err_ret;
	/* 分析应用层协议 */
	ModbusMasterAnalyzeApp(pxModbusMaster);
err_ret:
	pxModbusMaster->pxModbusBase->usRXCnt = 0; /* 必须清零计数器，方便下次帧同步 */
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMaster01H
*	功能说明: 分析01H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusMaster01H(ModbusMaster_t *pxModbusMaster)
{
	uint8_t *p;
	uint8_t bytes = pxModbusMaster->pxModbusBase->pucRXBuf[2] >> 1; /* 数据长度 字节数 */
	if ((pxModbusMaster->usReg01H >= pxModbusMaster->pxModbusBase->usCoilStartAddr) && ((pxModbusMaster->usReg01H + bytes) <= (pxModbusMaster->pxModbusBase->usCoilStartAddr + pxModbusMaster->pxModbusBase->usCoilSize)))
	{
		uint8_t *p = &pxModbusMaster->pxModbusBase->pucRXBuf[3];
		for (uint8_t i = 0; i < bytes; i++)
		{
			pxModbusMaster->pxModbusBase->pusCoilBuf[pxModbusMaster->usReg01H - pxModbusMaster->pxModbusBase->usCoilStartAddr + i] = BEBufToUint16(p);
			p += 2;
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMaster02H
*	功能说明: 分析02H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusMaster02H(ModbusMaster_t *pxModbusMaster)
{
	uint8_t *p;
	uint8_t bytes = pxModbusMaster->pxModbusBase->pucRXBuf[2] >> 1; /* 数据长度 字节数 */
	if ((pxModbusMaster->usReg02H >= pxModbusMaster->pxModbusBase->usDiscInStartAddr) && ((pxModbusMaster->usReg02H + bytes) <= (pxModbusMaster->pxModbusBase->usDiscInStartAddr + pxModbusMaster->pxModbusBase->usDiscInSize)))
	{
		uint8_t *p = &pxModbusMaster->pxModbusBase->pucRXBuf[3];
		for (uint8_t i = 0; i < bytes; i++)
		{
			pxModbusMaster->pxModbusBase->pusDiscInBuf[pxModbusMaster->usReg02H - pxModbusMaster->pxModbusBase->usDiscInStartAddr + i] = BEBufToUint16(p);
			p += 2;
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMaster03H
*	功能说明: 分析03H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusMaster03H(ModbusMaster_t *pxModbusMaster)
{
	uint8_t *p;
	uint8_t bytes = pxModbusMaster->pxModbusBase->pucRXBuf[2] >> 1; /* 数据长度 字节数 */
	if ((pxModbusMaster->usReg03H >= pxModbusMaster->pxModbusBase->usRegHoldStartAddr) && ((pxModbusMaster->usReg03H + bytes) <= (pxModbusMaster->pxModbusBase->usRegHoldStartAddr + pxModbusMaster->pxModbusBase->usRegHoldSize)))
	{
		uint8_t *p = &pxModbusMaster->pxModbusBase->pucRXBuf[3];
		for (uint8_t i = 0; i < bytes; i++)
		{
			pxModbusMaster->pxModbusBase->pusRegHoldBuf[pxModbusMaster->usReg03H - pxModbusMaster->pxModbusBase->usRegHoldStartAddr + i] = BEBufToUint16(p);
			p += 2;
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMaster04H
*	功能说明: 分析04H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusMaster04H(ModbusMaster_t *pxModbusMaster)
{
	uint8_t *p;
	uint8_t bytes = pxModbusMaster->pxModbusBase->pucRXBuf[2] >> 1; /* 数据长度 字节数 */
	if ((pxModbusMaster->usReg04H >= pxModbusMaster->pxModbusBase->usRegInStartAddr) && ((pxModbusMaster->usReg04H + bytes) <= (pxModbusMaster->pxModbusBase->usRegInStartAddr + pxModbusMaster->pxModbusBase->usRegInSize)))
	{
		uint8_t *p = &pxModbusMaster->pxModbusBase->pucRXBuf[3];
		for (uint8_t i = 0; i < bytes; i++)
		{
			pxModbusMaster->pxModbusBase->pusRegInBuf[pxModbusMaster->usReg04H - pxModbusMaster->pxModbusBase->usRegInStartAddr + i] = BEBufToUint16(p);
			p += 2;
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMaster05H
*	功能说明: 分析05H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusMaster05H(ModbusMaster_t *pxModbusMaster)
{
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMaster06H
*	功能说明: 分析06H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusMaster06H(ModbusMaster_t *pxModbusMaster)
{
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMaster10H
*	功能说明: 分析10H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusMaster10H(ModbusMaster_t *pxModbusMaster)
{
}
/*
*********************************************************************************************************
*	函 数 名: ModbusMasterReadParam01H
*	功能说明: 单个参数. 通过发送01H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t ModbusMasterReadParam01H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagNum)
{
	pxModbusMaster->pxModbusBase->pvLock(pxModbusMaster->pxModbusBase);
	uint8_t i;
	uint8_t res = 0;
	for (i = 0; i < pxModbusMaster->ucRXErrNum; i++)
	{
		ModbusMasterSend01H(pxModbusMaster, usRagAddr, usRagNum); /* 发送命令 */
		if (pxModbusMaster->pxModbusBase->pucRead(pxModbusMaster->pxModbusBase, NULL, NULL) == 1)
		{
			ModbusMasterPoll(pxModbusMaster);
			res = 1;
			break;
		}
	}
	pxModbusMaster->pxModbusBase->pvUnLock(pxModbusMaster->pxModbusBase);
	return res;
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMasterReadParam02H
*	功能说明: 单个参数. 通过发送02H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t ModbusMasterReadParam02H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagNum)
{
	pxModbusMaster->pxModbusBase->pvLock(pxModbusMaster->pxModbusBase);
	uint8_t i;
	uint8_t res = 0;
	for (i = 0; i < pxModbusMaster->ucRXErrNum; i++)
	{
		ModbusMasterSend02H(pxModbusMaster, usRagAddr, usRagNum); /* 发送命令 */
		if (pxModbusMaster->pxModbusBase->pucRead(pxModbusMaster->pxModbusBase, NULL, NULL) == 1)
		{
			ModbusMasterPoll(pxModbusMaster);
			res = 1;
			break;
		}
	}
	pxModbusMaster->pxModbusBase->pvUnLock(pxModbusMaster->pxModbusBase);
	return res;
}
/*
*********************************************************************************************************
*	函 数 名: ModbusMasterReadParam03H
*	功能说明: 单个参数. 通过发送03H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t ModbusMasterReadParam03H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagNum)
{
	pxModbusMaster->pxModbusBase->pvLock(pxModbusMaster->pxModbusBase);
	uint8_t i;
	uint8_t res = 0;
	for (i = 0; i < pxModbusMaster->ucRXErrNum; i++)
	{
		ModbusMasterSend03H(pxModbusMaster, usRagAddr, usRagNum); /* 发送命令 */
		if (pxModbusMaster->pxModbusBase->pucRead(pxModbusMaster->pxModbusBase, NULL, NULL) == 1)
		{
			ModbusMasterPoll(pxModbusMaster);
			res = 1;
			break;
		}
	}
	pxModbusMaster->pxModbusBase->pvUnLock(pxModbusMaster->pxModbusBase);
	return res;
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMasterReadParam04H
*	功能说明: 单个参数. 通过发送04H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t ModbusMasterReadParam04H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagNum)
{
	pxModbusMaster->pxModbusBase->pvLock(pxModbusMaster->pxModbusBase);
	uint8_t i;
	uint8_t res = 0;
	for (i = 0; i < pxModbusMaster->ucRXErrNum; i++)
	{
		ModbusMasterSend04H(pxModbusMaster, usRagAddr, usRagNum); /* 发送命令 */
		if (pxModbusMaster->pxModbusBase->pucRead(pxModbusMaster->pxModbusBase, NULL, NULL) == 1)
		{
			ModbusMasterPoll(pxModbusMaster);
			res = 1;
			break;
		}
	}
	pxModbusMaster->pxModbusBase->pvUnLock(pxModbusMaster->pxModbusBase);
	return res;
}
/*
*********************************************************************************************************
*	函 数 名: ModbusMasterWriteParam05H
*	功能说明: 单个参数. 通过发送05H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t ModbusMasterWriteParam05H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagValue)
{
	pxModbusMaster->pxModbusBase->pvLock(pxModbusMaster->pxModbusBase);
	uint8_t i;
	uint8_t res = 0;
	for (i = 0; i < pxModbusMaster->ucRXErrNum; i++)
	{
		ModbusMasterSend05H(pxModbusMaster, usRagAddr, usRagValue); /* 发送命令 */
		if (pxModbusMaster->pxModbusBase->pucRead(pxModbusMaster->pxModbusBase, NULL, NULL) == 1)
		{
			ModbusMasterPoll(pxModbusMaster);
			res = 1;
			break;
		}
	}
	pxModbusMaster->pxModbusBase->pvUnLock(pxModbusMaster->pxModbusBase);
	return res;
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMasterWriteParam06H
*	功能说明: 单个参数. 通过发送06H指令实现，发送之后，等待从机应答。循环NUM次写命令
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t ModbusMasterWriteParam06H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagValue)
{
	pxModbusMaster->pxModbusBase->pvLock(pxModbusMaster->pxModbusBase);
	uint8_t i;
	uint8_t res = 0;
	for (i = 0; i < pxModbusMaster->ucRXErrNum; i++)
	{
		ModbusMasterSend06H(pxModbusMaster, usRagAddr, usRagValue); /* 发送命令 */
		if (pxModbusMaster->pxModbusBase->pucRead(pxModbusMaster->pxModbusBase, NULL, NULL) == 1)
		{
			ModbusMasterPoll(pxModbusMaster);
			res = 1;
			break;
		}
	}
	pxModbusMaster->pxModbusBase->pvUnLock(pxModbusMaster->pxModbusBase);
	return res;
}

/*
*********************************************************************************************************
*	函 数 名: ModbusMasterWriteParam10H
*	功能说明: 单个参数. 通过发送10H指令实现，发送之后，等待从机应答。循环NUM次写命令
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t ModbusMasterWriteParam10H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint8_t usRagNum, uint16_t *pusRagBuf)
{
	pxModbusMaster->pxModbusBase->pvLock(pxModbusMaster->pxModbusBase);
	uint8_t i;
	uint8_t res = 0;
	for (i = 0; i < pxModbusMaster->ucRXErrNum; i++)
	{
		ModbusMasterSend10H(pxModbusMaster, usRagAddr, usRagNum, pusRagBuf); /* 发送命令 */
		if (pxModbusMaster->pxModbusBase->pucRead(pxModbusMaster->pxModbusBase, NULL, NULL) == 1)
		{
			ModbusMasterPoll(pxModbusMaster);
			res = 1;
			break;
		}
	}
	pxModbusMaster->pxModbusBase->pvUnLock(pxModbusMaster->pxModbusBase);
	return res;
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
/******************************************************************************************************/
uint8_t ModbusMasterRead03H(ModbusMaster_t *pxModbusMaster, uint16_t usAdd, uint16_t *pusBuf, uint16_t usSize)
{
	pxModbusMaster->pxModbusBase->usRegHoldStartAddr = usAdd;
	pxModbusMaster->pxModbusBase->usRegHoldSize = usSize;
	pxModbusMaster->pxModbusBase->pusRegHoldBuf = pusBuf;
	return ModbusMasterReadParam03H(pxModbusMaster, usAdd, usSize);
}

uint8_t ModbusMasterWrite06H(ModbusMaster_t *pxModbusMaster, uint16_t usAdd, uint16_t usDat)
{
	return ModbusMasterWriteParam06H(pxModbusMaster, usAdd, usDat);
}

uint8_t ModbusMasterWrite10H(ModbusMaster_t *pxModbusMaster, uint16_t usAdd, uint16_t *pusBuf, uint16_t usSize)
{
	pxModbusMaster->pxModbusBase->usRegHoldStartAddr = usAdd;
	pxModbusMaster->pxModbusBase->usRegHoldSize = usSize;
	pxModbusMaster->pxModbusBase->pusRegHoldBuf = pusBuf;
	return ModbusMasterWriteParam10H(pxModbusMaster, usAdd, usSize, pusBuf);
}
/*********************************************************************************************************/
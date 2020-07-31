#include "ModbusSlave.h"

/* RTU 应答代码 */
#define RSP_OK 0x00			  /* 成功 */
#define RSP_ERR_CMD 0x01	  /* 不支持的功能码 */
#define RSP_ERR_REG_ADDR 0x02 /* 寄存器地址错误 */
#define RSP_ERR_VALUE 0x03	  /* 数据值域错误 */
#define RSP_ERR_WRITE 0x04	  /* 写入失败 */

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
*	形    参: pucBuf : 参与校验的数据
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
static uint16_t CRC16Modbus(uint8_t *pucBuf, uint16_t _usLen)
{
	uint8_t ucCRCHi = 0xFF; /* 高CRC字节初始化 */
	uint8_t ucCRCLo = 0xFF; /* 低CRC 字节初始化 */
	uint16_t usIndex;		/* CRC循环中的索引 */

	while (_usLen--)
	{
		usIndex = ucCRCHi ^ *pucBuf++; /* 计算CRC */
		ucCRCHi = ucCRCLo ^ s_CRCHi[usIndex];
		ucCRCLo = s_CRCLo[usIndex];
	}
	return ((uint16_t)ucCRCHi << 8 | ucCRCLo);
	/************************时间换空间********************************/
	// uint8_t i;
	// uint16_t crc = 0xffff;
	// if (_usLen == 0)
	// {
	// 	_usLen = 1;
	// }
	// while (_usLen--)
	// {
	// 	crc ^= *pucBuf;
	// 	for (i = 0; i < 8; i++)
	// 	{
	// 		if (crc & 1)
	// 		{
	// 			crc >>= 1;
	// 			crc ^= 0xA001;
	// 		}
	// 		else
	// 		{
	// 			crc >>= 1;
	// 		}
	// 	}
	// 	pucBuf++;
	// }
	// return (crc << 8 | crc >> 8);
}

/*
*********************************************************************************************************
*	函 数 名: BEBufToUint16
*	功能说明: 将2字节数组(大端Big Endian次序，高字节在前)转换为16位整数
*	形    参: pucBuf : 数组
*	返 回 值: 16位整数值
*
*   大端(Big Endian)与小端(Little Endian)
*********************************************************************************************************
*/
static uint16_t BEBufToUint16(uint8_t *pucBuf)
{
	return (((uint16_t)pucBuf[0] << 8) | pucBuf[1]);
}
/********************************************************************************************************/

static void ModbusSlaveSendWithCRC(ModbusSlave_t *pxModbusSlave, uint8_t *pucBuf, uint16_t ucLen);
static void ModbusSlaveSendAckOk(ModbusSlave_t *pxModbusSlave);
static void ModbusSlaveSendAckErr(ModbusSlave_t *pxModbusSlave);
static void ModbusSlaveAnalyzeApp(ModbusSlave_t *pxModbusSlave);
static void ModbusSlave01H(ModbusSlave_t *pxModbusSlave);
static void ModbusSlave02H(ModbusSlave_t *pxModbusSlave);
static void ModbusSlave03H(ModbusSlave_t *pxModbusSlave);
static void ModbusSlave04H(ModbusSlave_t *pxModbusSlave);
static void ModbusSlave05H(ModbusSlave_t *pxModbusSlave);
static void ModbusSlave06H(ModbusSlave_t *pxModbusSlave);
static void ModbusSlave10H(ModbusSlave_t *pxModbusSlave);

static uint8_t ModbusSlave03HCallBack(ModbusSlave_t *pxModbusSlave, uint16_t reg_addr, uint16_t *reg_value, uint16_t usLen);
static uint8_t ModbusSlave06H10HCallBack(ModbusSlave_t *pxModbusSlave, uint16_t reg_addr, uint16_t *reg_value, uint16_t usLen);

void ModbusSlaveInit(ModbusSlave_t *pxModbusSlave)
{
	pxModbusSlave->pucInit(pxModbusSlave->pxModbusBase);
}

__WEAK uint8_t ModbusSlavePollCallBack(ModbusSlave_t *pxModbusSlave)
{
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: ModbusSlavePoll
*	功能说明: 解析数据包. 在主程序中轮流调用。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void ModbusSlavePoll(ModbusSlave_t *pxModbusSlave)
{
	pxModbusSlave->pxModbusBase->pucRead(pxModbusSlave->pxModbusBase, NULL, NULL);
	if (pxModbusSlave->pxModbusBase->pucRXBuf[0] != 0 &&
		pxModbusSlave->pxModbusBase->pucRXBuf[0] != pxModbusSlave->pxModbusBase->ucAddr) /* 判断主机发送的命令地址是否符合 */
		goto err_ret;
	if (pxModbusSlave->pxModbusBase->usRXCnt < 4) /* 判断主机发送数据小于4个字节就认为错误 */
		goto err_ret;
	/* 计算CRC校验和 */
	if (CRC16Modbus(pxModbusSlave->pxModbusBase->pucRXBuf, pxModbusSlave->pxModbusBase->usRXCnt) != 0)
		goto err_ret;
	if (ModbusSlavePollCallBack(pxModbusSlave) != 0) //此处留给用户进行其他的特殊的操作
		goto err_ret;
	/* 分析应用层协议 */
	ModbusSlaveAnalyzeApp(pxModbusSlave);
err_ret:
	pxModbusSlave->pxModbusBase->usRXCnt = 0; /* 必须清零计数器，方便下次帧同步 */
}

/*
*********************************************************************************************************
*	函 数 名: ModbusSlaveSendWithCRC
*	功能说明: 发送一串数据, 自动追加2字节CRC
*	形    参: pucBuf 数据；
*			  ucLen 数据长度（不带CRC）
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusSlaveSendWithCRC(ModbusSlave_t *pxModbusSlave, uint8_t *pucBuf, uint16_t ucLen)
{
	uint16_t usCrc;
	uint8_t ucBuf[256];
	memcpy(ucBuf, pucBuf, ucLen);
	usCrc = CRC16Modbus(pucBuf, ucLen);
	ucBuf[ucLen++] = usCrc >> 8;
	ucBuf[ucLen++] = usCrc;
	pxModbusSlave->pxModbusBase->pucSend(pxModbusSlave->pxModbusBase, ucBuf, ucLen);
}

/*
*********************************************************************************************************
*	函 数 名: ModbusSlaveSendAckErr
*	功能说明: 发送错误应答
*	形    参: _ucErrCode : 错误代码
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusSlaveSendAckErr(ModbusSlave_t *pxModbusSlave)
{
	uint8_t ucTXBuf[3];
	ucTXBuf[0] = pxModbusSlave->pxModbusBase->pucRXBuf[0];		  /* 485地址 */
	ucTXBuf[1] = pxModbusSlave->pxModbusBase->pucRXBuf[1] | 0x80; /* 异常的功能码 */
	ucTXBuf[2] = pxModbusSlave->ucRspCode;						  /* 错误代码(01,02,03,04) */
	ModbusSlaveSendWithCRC(pxModbusSlave, ucTXBuf, 3);
}

/*
*********************************************************************************************************
*	函 数 名: ModbusSlaveSendAckOk
*	功能说明: 发送正确的应答.
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusSlaveSendAckOk(ModbusSlave_t *pxModbusSlave)
{
	uint8_t ucTXBuf[6];
	memcpy(ucTXBuf, pxModbusSlave->pxModbusBase->pucRXBuf, 6);
	ModbusSlaveSendWithCRC(pxModbusSlave, ucTXBuf, 6);
}

/*
*********************************************************************************************************
*	函 数 名: ModbusSlaveAnalyzeApp
*	功能说明: 分析应用层协议
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusSlaveAnalyzeApp(ModbusSlave_t *pxModbusSlave)
{
	switch (pxModbusSlave->pxModbusBase->pucRXBuf[1]) /* 第2个字节 功能码 */
	{
	case 0x01: /* 读取线圈状态（此例程用led代替）*/
		ModbusSlave01H(pxModbusSlave);
		break;
	case 0x02: /* 读取输入状态（按键状态）*/
		ModbusSlave02H(pxModbusSlave);
		break;
	case 0x03: /* 读取保持寄存器（此例程存在g_tVar中）*/
		ModbusSlave03H(pxModbusSlave);
		break;
	case 0x04: /* 读取输入寄存器（ADC的值）*/
		ModbusSlave04H(pxModbusSlave);
		break;
	case 0x05: /* 强制单线圈（设置led）*/
		ModbusSlave05H(pxModbusSlave);
		break;
	case 0x06: /* 写单个保存寄存器（此例程改写g_tVar中的参数）*/
		ModbusSlave06H(pxModbusSlave);
		break;
	case 0x10: /* 写多个保存寄存器（此例程存在g_tVar中的参数）*/
		ModbusSlave10H(pxModbusSlave);
		break;
	default:
		pxModbusSlave->ucRspCode = RSP_ERR_CMD;
		ModbusSlaveSendAckErr(pxModbusSlave); /* 告诉主机命令错误 */
		break;
	}
}

static uint8_t ModbusSlave01HCallBack(ModbusSlave_t *pxModbusSlave, uint16_t usAdd, uint16_t *pusDat)
{
}

static uint8_t ModbusSlave02CallBack(ModbusSlave_t *pxModbusSlave, uint16_t usAdd, uint16_t *pusDat)
{
}
/*
*********************************************************************************************************
*	函 数 名: ModbusSlave03HCallBack
*	功能说明: 读取保持寄存器的值
*	形    参: reg_addr 寄存器地址
*			  reg_value 存放寄存器结果
*	返 回 值: 1表示OK 0表示错误
*********************************************************************************************************
*/
static uint8_t ModbusSlave03HCallBack(ModbusSlave_t *pxModbusSlave, uint16_t reg_addr, uint16_t *reg_value, uint16_t usLen)
{
	uint8_t ucRes = 0;
	if (pxModbusSlave != NULL)
	{
		if (pxModbusSlave->pucRegHoldFun != NULL)
		{
			ucRes = pxModbusSlave->pucRegHoldFun((struct ModbusSlave_t *)pxModbusSlave, reg_addr, reg_value, usLen, 0);
			for (uint16_t i = 0; i < usLen; i++)
				*(reg_value + i) = ((*(reg_value + i) << 8) & 0xff00) | ((*(reg_value + i) >> 8) & 0xff);
		}
	}
	return ucRes;
}

static uint8_t ModbusSlave04HCallBack(ModbusSlave_t *pxModbusSlave, uint16_t reg_addr, uint16_t *reg_value, uint16_t usLen)
{
	uint8_t ucRes = 0;
	if (pxModbusSlave != NULL)
	{
		if (pxModbusSlave->pucRegInFun != NULL)
		{
			ucRes = pxModbusSlave->pucRegInFun((struct ModbusSlave_t *)pxModbusSlave, reg_addr, reg_value, usLen);
			for (uint16_t i = 0; i < usLen; i++)
				*(reg_value + i) = ((*(reg_value + i) << 8) & 0xff00) | ((*(reg_value + i) >> 8) & 0xff);
		}
	}
	return ucRes;
}

/*
*********************************************************************************************************
*	函 数 名: ModbusSlave06H10HCallBack
*	功能说明: 读取保持寄存器的值
*	形    参: reg_addr 寄存器地址
*			  reg_value 寄存器值
*	返 回 值: 1表示OK 0表示错误
*********************************************************************************************************
*/

static uint8_t ModbusSlave06H10HCallBack(ModbusSlave_t *pxModbusSlave, uint16_t reg_addr, uint16_t *reg_value, uint16_t usLen)
{
	uint8_t ucRes = 0;
	if (pxModbusSlave != NULL)
	{
		if (pxModbusSlave->pucRegHoldFun != NULL)
		{
			ucRes = pxModbusSlave->pucRegHoldFun((struct ModbusSlave_t *)pxModbusSlave, reg_addr, reg_value, usLen, 1);
		}
	}
	return ucRes;
}

/*
*********************************************************************************************************
*	函 数 名: ModbusSlave01H
*	功能说明: 读取线圈状态（对应远程开关D01/D02/D03）
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
/* 说明:这里用LED代替继电器,便于观察现象 */
static void ModbusSlave01H(ModbusSlave_t *pxModbusSlave)
{
	/*
	 举例：
		主机发送:
			11 从机地址
			01 功能码
			00 寄存器起始地址高字节
			13 寄存器起始地址低字节
			00 寄存器数量高字节
			25 寄存器数量低字节
			0E CRC校验高字节
			84 CRC校验低字节
		从机应答: 	1代表ON，0代表OFF。若返回的线圈数不为8的倍数，则在最后数据字节未尾使用0代替. BIT0对应第1个
			11 从机地址
			01 功能码
			05 返回字节数
			CD 数据1(线圈0013H-线圈001AH)
			6B 数据2(线圈001BH-线圈0022H)
			B2 数据3(线圈0023H-线圈002AH)
			0E 数据4(线圈0032H-线圈002BH)
			1B 数据5(线圈0037H-线圈0033H)
			45 CRC校验高字节
			E6 CRC校验低字节
		例子:
			01 01 10 01 00 03   29 0B	--- 查询D01开始的3个继电器状态
			01 01 10 03 00 01   09 0A   --- 查询D03继电器的状态
	*/
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint16_t m;
	uint8_t status[10];

	pxModbusSlave->ucRspCode = RSP_OK;

	/* 没有外部继电器，直接应答错误 */
	if (pxModbusSlave->pxModbusBase->usRXCnt != 8)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* 数据值域错误 */
		return;
	}

	reg = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[2]); /* 寄存器号 */
	num = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[4]); /* 寄存器个数 */

	m = (num + 7) / 8;

	if ((reg >= pxModbusSlave->pxModbusBase->usCoilStartAddr) && (num > 0) && (reg + num <= pxModbusSlave->pxModbusBase->usCoilStartAddr + pxModbusSlave->pxModbusBase->usCoilSize))
	{
		memset(status, 0, m);
		for (i = 0; i < num; i++)
		{
			if (pxModbusSlave->pxModbusBase->pusCoilBuf[reg - pxModbusSlave->pxModbusBase->usCoilStartAddr + i])
				status[i / 8] |= (1 << (i % 8));
		}
	}
	else
	{
		pxModbusSlave->ucRspCode = RSP_ERR_REG_ADDR; /* 寄存器地址错误 */
	}

	if (pxModbusSlave->ucRspCode == RSP_OK) /* 正确应答 */
	{
		pxModbusSlave->pxModbusBase->usTXCnt = 0;
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = pxModbusSlave->pxModbusBase->pucRXBuf[0];
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = pxModbusSlave->pxModbusBase->pucRXBuf[1];
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = m; /* 返回字节数 */

		for (i = 0; i < m; i++)
			pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = status[i]; /* 继电器状态 */
		ModbusSlaveSendWithCRC(pxModbusSlave, pxModbusSlave->pxModbusBase->pucTXBuf, pxModbusSlave->pxModbusBase->usTXCnt);
	}
	else
	{
		ModbusSlaveSendAckErr(pxModbusSlave); /* 告诉主机命令错误 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: ModbusSlave02H
*	功能说明: 读取输入状态（对应K01～K03）
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusSlave02H(ModbusSlave_t *pxModbusSlave)
{
	/*
		主机发送:
			11 从机地址
			02 功能码
			00 寄存器地址高字节
			C4 寄存器地址低字节
			00 寄存器数量高字节
			16 寄存器数量低字节
			BA CRC校验高字节
			A9 CRC校验低字节
		从机应答:  响应各离散输入寄存器状态，分别对应数据区中的每位值，1 代表ON；0 代表OFF。
		           第一个数据字节的LSB(最低字节)为查询的寻址地址，其他输入口按顺序在该字节中由低字节
		           向高字节排列，直到填充满8位。下一个字节中的8个输入位也是从低字节到高字节排列。
		           若返回的输入位数不是8的倍数，则在最后的数据字节中的剩余位至该字节的最高位使用0填充。
			11 从机地址
			02 功能码
			03 返回字节数
			AC 数据1(00C4H-00CBH)
			DB 数据2(00CCH-00D3H)
			35 数据3(00D4H-00D9H)
			20 CRC校验高字节
			18 CRC校验低字节
		例子:
		01 02 20 01 00 08  23CC  ---- 读取T01-08的状态
		01 02 20 04 00 02  B3CA  ---- 读取T04-05的状态
		01 02 20 01 00 12  A207   ---- 读 T01-18
	*/
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint16_t m;
	uint8_t status[10];

	pxModbusSlave->ucRspCode = RSP_OK;

	if (pxModbusSlave->pxModbusBase->usRXCnt != 8)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* 数据值域错误 */
		return;
	}

	reg = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[2]); /* 寄存器号 */
	num = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[4]); /* 寄存器个数 */

	m = (num + 7) / 8;
	if ((reg >= pxModbusSlave->pxModbusBase->usDiscInStartAddr) && (num > 0) && (reg + num <= pxModbusSlave->pxModbusBase->usDiscInStartAddr + pxModbusSlave->pxModbusBase->usDiscInSize))
	{
		memset(status, 0, m);
		for (i = 0; i < num; i++)
		{
			if (pxModbusSlave->pxModbusBase->pusDiscInBuf[reg - pxModbusSlave->pxModbusBase->usDiscInStartAddr + i])
				status[i / 8] |= (1 << (i % 8));
		}
	}
	else
	{
		pxModbusSlave->ucRspCode = RSP_ERR_REG_ADDR; /* 寄存器地址错误 */
	}

	if (pxModbusSlave->ucRspCode == RSP_OK) /* 正确应答 */
	{
		pxModbusSlave->pxModbusBase->usTXCnt = 0;
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = pxModbusSlave->pxModbusBase->pucRXBuf[0];
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = pxModbusSlave->pxModbusBase->pucRXBuf[1];
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = m; /* 返回字节数 */

		for (i = 0; i < m; i++)
			pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = status[i]; /* T01-02状态 */
		ModbusSlaveSendWithCRC(pxModbusSlave, pxModbusSlave->pxModbusBase->pucTXBuf, pxModbusSlave->pxModbusBase->usTXCnt);
	}
	else
	{
		ModbusSlaveSendAckErr(pxModbusSlave); /* 告诉主机命令错误 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: ModbusSlave03H
*	功能说明: 读取保持寄存器 在一个或多个保持寄存器中取得当前的二进制值
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusSlave03H(ModbusSlave_t *pxModbusSlave)
{
	/*
		从机地址为11H。保持寄存器的起始地址为006BH，结束地址为006DH。该次查询总共访问3个保持寄存器。
		主机发送:
			11 从机地址
			03 功能码
			00 寄存器地址高字节
			6B 寄存器地址低字节
			00 寄存器数量高字节
			03 寄存器数量低字节
			76 CRC高字节
			87 CRC低字节
		从机应答: 	保持寄存器的长度为2个字节。对于单个保持寄存器而言，寄存器高字节数据先被传输，
					低字节数据后被传输。保持寄存器之间，低地址寄存器先被传输，高地址寄存器后被传输。
			11 从机地址
			03 功能码
			06 字节数
			00 数据1高字节(006BH)
			6B 数据1低字节(006BH)
			00 数据2高字节(006CH)
			13 数据2 低字节(006CH)
			00 数据3高字节(006DH)
			00 数据3低字节(006DH)
			38 CRC高字节
			B9 CRC低字节
		例子:
			01 03 30 06 00 01  6B0B      ---- 读 3006H, 触发电流
			01 03 4000 0010 51C6         ---- 读 4000H 倒数第1条浪涌记录 32字节
			01 03 4001 0010 0006         ---- 读 4001H 倒数第1条浪涌记录 32字节
			01 03 F000 0008 770C         ---- 读 F000H 倒数第1条告警记录 16字节
			01 03 F001 0008 26CC         ---- 读 F001H 倒数第2条告警记录 16字节
			01 03 7000 0020 5ED2         ---- 读 7000H 倒数第1条波形记录第1段 64字节
			01 03 7001 0020 0F12         ---- 读 7001H 倒数第1条波形记录第2段 64字节
			01 03 7040 0020 5F06         ---- 读 7040H 倒数第2条波形记录第1段 64字节
	*/
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint8_t reg_value[256] = {0};

	pxModbusSlave->ucRspCode = RSP_OK;

	if (pxModbusSlave->pxModbusBase->usRXCnt != 8) /* 03H命令必须是8个字节 */
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* 数据值域错误 */
		goto err_ret;
	}

	reg = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[2]); /* 寄存器号 */
	num = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[4]); /* 寄存器个数 */

	if (num > sizeof(reg_value) / 2)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* 数据值域错误 */
		goto err_ret;
	}

	if (ModbusSlave03HCallBack(pxModbusSlave, reg, (uint16_t *)reg_value, num) == 0)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_REG_ADDR;
	}

err_ret:
	if (pxModbusSlave->ucRspCode == RSP_OK) /* 正确应答 */
	{
		pxModbusSlave->pxModbusBase->usTXCnt = 0;
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = pxModbusSlave->pxModbusBase->pucRXBuf[0];
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = pxModbusSlave->pxModbusBase->pucRXBuf[1];
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = num * 2; /* 返回字节数 */

		for (i = 0; i < num; i++)
		{
			pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = reg_value[2 * i];
			pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = reg_value[2 * i + 1];
		}
		ModbusSlaveSendWithCRC(pxModbusSlave, pxModbusSlave->pxModbusBase->pucTXBuf, pxModbusSlave->pxModbusBase->usTXCnt); /* 发送正确应答 */
	}
	else
	{
		ModbusSlaveSendAckErr(pxModbusSlave); /* 发送错误应答 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: ModbusSlave04H
*	功能说明: 读取输入寄存器（对应A01/A02） SMA
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusSlave04H(ModbusSlave_t *pxModbusSlave)
{
	/*
		主机发送:
			11 从机地址
			04 功能码
			00 寄存器起始地址高字节
			08 寄存器起始地址低字节
			00 寄存器个数高字节
			02 寄存器个数低字节
			F2 CRC高字节
			99 CRC低字节
		从机应答:  输入寄存器长度为2个字节。对于单个输入寄存器而言，寄存器高字节数据先被传输，
				低字节数据后被传输。输入寄存器之间，低地址寄存器先被传输，高地址寄存器后被传输。
			11 从机地址
			04 功能码
			04 字节数
			00 数据1高字节(0008H)
			0A 数据1低字节(0008H)
			00 数据2高字节(0009H)
			0B 数据2低字节(0009H)
			8B CRC高字节
			80 CRC低字节
		例子:
			01 04 2201 0006 2BB0  --- 读 2201H A01通道模拟量 开始的6个数据
			01 04 2201 0001 6A72  --- 读 2201H
	*/
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint8_t status[256] = {0};

	pxModbusSlave->ucRspCode = RSP_OK;

	if (pxModbusSlave->pxModbusBase->usRXCnt != 8)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* 数据值域错误 */
		goto err_ret;
	}

	reg = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[2]); /* 寄存器号 */
	num = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[4]); /* 寄存器个数 */

	if (num > sizeof(status) / 2)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* 数据值域错误 */
		goto err_ret;
	}

	if (ModbusSlave04HCallBack(pxModbusSlave, reg, (uint16_t *)status, num) == 0)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_REG_ADDR;
	}

err_ret:
	if (pxModbusSlave->ucRspCode == RSP_OK) /* 正确应答 */
	{
		pxModbusSlave->pxModbusBase->usTXCnt = 0;
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = pxModbusSlave->pxModbusBase->pucRXBuf[0];
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = pxModbusSlave->pxModbusBase->pucRXBuf[1];
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = num * 2; /* 返回字节数 */

		for (i = 0; i < num; i++)
		{
			pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = status[2 * i];
			pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = status[2 * i + 1];
		}
		ModbusSlaveSendWithCRC(pxModbusSlave, pxModbusSlave->pxModbusBase->pucTXBuf, pxModbusSlave->pxModbusBase->usTXCnt);
	}
	else
	{
		ModbusSlaveSendAckErr(pxModbusSlave); /* 告诉主机命令错误 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: ModbusSlave05H
*	功能说明: 强制单线圈（对应D01/D02/D03）
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusSlave05H(ModbusSlave_t *pxModbusSlave)
{
	/*
		主机发送: 写单个线圈寄存器。FF00H值请求线圈处于ON状态，0000H值请求线圈处于OFF状态
		。05H指令设置单个线圈的状态，15H指令可以设置多个线圈的状态。
			11 从机地址
			05 功能码
			00 寄存器地址高字节
			AC 寄存器地址低字节
			FF 数据1高字节
			00 数据2低字节
			4E CRC校验高字节
			8B CRC校验低字节
		从机应答:
			11 从机地址
			05 功能码
			00 寄存器地址高字节
			AC 寄存器地址低字节
			FF 寄存器1高字节
			00 寄存器1低字节
			4E CRC校验高字节
			8B CRC校验低字节
		例子:
		01 05 10 01 FF 00   D93A   -- D01打开
		01 05 10 01 00 00   98CA   -- D01关闭
		01 05 10 02 FF 00   293A   -- D02打开
		01 05 10 02 00 00   68CA   -- D02关闭
		01 05 10 03 FF 00   78FA   -- D03打开
		01 05 10 03 00 00   390A   -- D03关闭
	*/
	uint16_t reg;
	uint16_t value;

	pxModbusSlave->ucRspCode = RSP_OK;

	if (pxModbusSlave->pxModbusBase->usRXCnt != 8)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* 数据值域错误 */
		goto err_ret;
	}

	reg = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[2]);	  /* 寄存器号 */
	value = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[4]); /* 数据 */

	if (value != 0 && value != 1)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* 数据值域错误 */
		goto err_ret;
	}
	if (reg >= pxModbusSlave->pxModbusBase->usCoilStartAddr && reg < (pxModbusSlave->pxModbusBase->usCoilStartAddr + pxModbusSlave->pxModbusBase->usCoilSize))
	{
		pxModbusSlave->pxModbusBase->pusCoilBuf[reg - pxModbusSlave->pxModbusBase->usCoilStartAddr] = value;
	}
	else
	{
		pxModbusSlave->ucRspCode = RSP_ERR_REG_ADDR; /* 寄存器地址错误 */
	}
err_ret:
	if (pxModbusSlave->ucRspCode == RSP_OK) /* 正确应答 */
	{
		ModbusSlaveSendAckOk(pxModbusSlave);
	}
	else
	{
		ModbusSlaveSendAckErr(pxModbusSlave); /* 告诉主机命令错误 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: ModbusSlave06H
*	功能说明: 写单个寄存器
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusSlave06H(ModbusSlave_t *pxModbusSlave)
{

	/*
		写保持寄存器。注意06指令只能操作单个保持寄存器，16指令可以设置单个或多个保持寄存器
		主机发送:
			11 从机地址
			06 功能码
			00 寄存器地址高字节
			01 寄存器地址低字节
			00 数据1高字节
			01 数据1低字节
			9A CRC校验高字节
			9B CRC校验低字节
		从机响应:
			11 从机地址
			06 功能码
			00 寄存器地址高字节
			01 寄存器地址低字节
			00 数据1高字节
			01 数据1低字节
			1B CRC校验高字节
			5A	CRC校验低字节
		例子:
			01 06 30 06 00 25  A710    ---- 触发电流设置为 2.5
			01 06 30 06 00 10  6707    ---- 触发电流设置为 1.0
			01 06 30 1B 00 00  F6CD    ---- SMA 滤波系数 = 0 关闭滤波
			01 06 30 1B 00 01  370D    ---- SMA 滤波系数 = 1
			01 06 30 1B 00 02  770C    ---- SMA 滤波系数 = 2
			01 06 30 1B 00 05  36CE    ---- SMA 滤波系数 = 5
			01 06 30 07 00 01  F6CB    ---- 测试模式修改为 T1
			01 06 30 07 00 02  B6CA    ---- 测试模式修改为 T2
			01 06 31 00 00 00  8736    ---- 擦除浪涌记录区
			01 06 31 01 00 00  D6F6    ---- 擦除告警记录区
*/
	uint16_t reg;
	uint16_t value;

	pxModbusSlave->ucRspCode = RSP_OK;

	if (pxModbusSlave->pxModbusBase->usRXCnt != 8)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* 数据值域错误 */
		goto err_ret;
	}

	reg = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[2]);	  /* 寄存器号 */
	value = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[4]); /* 寄存器值 */

	if (ModbusSlave06H10HCallBack(pxModbusSlave, reg, &value, 1) == 0) /* 该函数会把写入的值存入寄存器 */
	{
		pxModbusSlave->ucRspCode = RSP_ERR_REG_ADDR; /* 寄存器地址错误 */
	}

err_ret:
	if (pxModbusSlave->ucRspCode == RSP_OK) /* 正确应答 */
	{
		ModbusSlaveSendAckOk(pxModbusSlave);
	}
	else
	{
		ModbusSlaveSendAckErr(pxModbusSlave); /* 告诉主机命令错误 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: ModbusSlave10H
*	功能说明: 连续写多个寄存器.  进用于改写时钟
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ModbusSlave10H(ModbusSlave_t *pxModbusSlave)
{
	/*
		从机地址为11H。保持寄存器的其实地址为0001H，寄存器的结束地址为0002H。总共访问2个寄存器。
		保持寄存器0001H的内容为000AH，保持寄存器0002H的内容为0102H。
		主机发送:
			11 从机地址
			10 功能码
			00 寄存器起始地址高字节
			01 寄存器起始地址低字节
			00 寄存器数量高字节
			02 寄存器数量低字节
			04 字节数
			00 数据1高字节
			0A 数据1低字节
			01 数据2高字节
			02 数据2低字节
			C6 CRC校验高字节
			F0 CRC校验低字节
		从机响应:
			11 从机地址
			06 功能码
			00 寄存器地址高字节
			01 寄存器地址低字节
			00 数据1高字节
			01 数据1低字节
			1B CRC校验高字节
			5A	CRC校验低字节
		例子:
			01 10 30 00 00 06 0C  07 DE  00 0A  00 01  00 08  00 0C  00 00     389A    ---- 写时钟 2014-10-01 08:12:00
			01 10 30 00 00 06 0C  07 DF  00 01  00 1F  00 17  00 3B  00 39     5549    ---- 写时钟 2015-01-31 23:59:57
	*/
	uint16_t reg_addr;
	uint16_t reg_num;
	uint8_t byte_num;
	uint16_t value[256];

	pxModbusSlave->ucRspCode = RSP_OK;

	if (pxModbusSlave->pxModbusBase->usRXCnt < 11)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* 数据值域错误 */
		goto err_ret;
	}

	reg_addr = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[2]); /* 寄存器号 */
	reg_num = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[4]);	 /* 寄存器个数 */
	byte_num = pxModbusSlave->pxModbusBase->pucRXBuf[6];				 /* 后面的数据体字节数 */

	if (byte_num != 2 * reg_num)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* 数据值域错误 */
		goto err_ret;
	}
	for (uint8_t i = 0; i < reg_num; i++)
	{
		value[i] = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[7 + 2 * i]);
	}
	if (ModbusSlave06H10HCallBack(pxModbusSlave, reg_addr, value, reg_num) == 0)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_REG_ADDR;
	}
err_ret:
	if (pxModbusSlave->ucRspCode == RSP_OK) /* 正确应答 */
	{
		ModbusSlaveSendAckOk(pxModbusSlave);
	}
	else
	{
		ModbusSlaveSendAckErr(pxModbusSlave); /* 告诉主机命令错误 */
	}
}
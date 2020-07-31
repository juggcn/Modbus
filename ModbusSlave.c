#include "ModbusSlave.h"

/* RTU Ӧ����� */
#define RSP_OK 0x00			  /* �ɹ� */
#define RSP_ERR_CMD 0x01	  /* ��֧�ֵĹ����� */
#define RSP_ERR_REG_ADDR 0x02 /* �Ĵ�����ַ���� */
#define RSP_ERR_VALUE 0x03	  /* ����ֵ����� */
#define RSP_ERR_WRITE 0x04	  /* д��ʧ�� */

// CRC ��λ�ֽ�ֵ��
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
// CRC ��λ�ֽ�ֵ��
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
*	�� �� ��: CRC16Modbus
*	����˵��: ����CRC�� ����ModbusЭ�顣
*	��    ��: pucBuf : ����У�������
*			  _usLen : ���ݳ���
*	�� �� ֵ: 16λ����ֵ�� ����Modbus ���˽�����ֽ��ȴ��ͣ����ֽں��͡�
*
*   ���п��ܵ�CRCֵ����Ԥװ���������鵱�У������㱨������ʱ���Լ򵥵��������ɣ�
*   һ�����������16λCRC�������256�����ܵĸ�λ�ֽڣ���һ�����麬�е�λ�ֽڵ�ֵ��
*   ������������CRC�ķ�ʽ�ṩ�˱ȶԱ��Ļ�������ÿһ�����ַ��������µ�CRC����ķ�����
*
*  ע�⣺�˳����ڲ�ִ�и�/��CRC�ֽڵĽ������˺������ص����Ѿ�����������CRCֵ��Ҳ����˵���ú����ķ���ֵ����ֱ�ӷ���
*        �ڱ������ڷ��ͣ�
*********************************************************************************************************
*/
static uint16_t CRC16Modbus(uint8_t *pucBuf, uint16_t _usLen)
{
	uint8_t ucCRCHi = 0xFF; /* ��CRC�ֽڳ�ʼ�� */
	uint8_t ucCRCLo = 0xFF; /* ��CRC �ֽڳ�ʼ�� */
	uint16_t usIndex;		/* CRCѭ���е����� */

	while (_usLen--)
	{
		usIndex = ucCRCHi ^ *pucBuf++; /* ����CRC */
		ucCRCHi = ucCRCLo ^ s_CRCHi[usIndex];
		ucCRCLo = s_CRCLo[usIndex];
	}
	return ((uint16_t)ucCRCHi << 8 | ucCRCLo);
	/************************ʱ�任�ռ�********************************/
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
*	�� �� ��: BEBufToUint16
*	����˵��: ��2�ֽ�����(���Big Endian���򣬸��ֽ���ǰ)ת��Ϊ16λ����
*	��    ��: pucBuf : ����
*	�� �� ֵ: 16λ����ֵ
*
*   ���(Big Endian)��С��(Little Endian)
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
*	�� �� ��: ModbusSlavePoll
*	����˵��: �������ݰ�. �����������������á�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ModbusSlavePoll(ModbusSlave_t *pxModbusSlave)
{
	pxModbusSlave->pxModbusBase->pucRead(pxModbusSlave->pxModbusBase, NULL, NULL);
	if (pxModbusSlave->pxModbusBase->pucRXBuf[0] != 0 &&
		pxModbusSlave->pxModbusBase->pucRXBuf[0] != pxModbusSlave->pxModbusBase->ucAddr) /* �ж��������͵������ַ�Ƿ���� */
		goto err_ret;
	if (pxModbusSlave->pxModbusBase->usRXCnt < 4) /* �ж�������������С��4���ֽھ���Ϊ���� */
		goto err_ret;
	/* ����CRCУ��� */
	if (CRC16Modbus(pxModbusSlave->pxModbusBase->pucRXBuf, pxModbusSlave->pxModbusBase->usRXCnt) != 0)
		goto err_ret;
	if (ModbusSlavePollCallBack(pxModbusSlave) != 0) //�˴������û���������������Ĳ���
		goto err_ret;
	/* ����Ӧ�ò�Э�� */
	ModbusSlaveAnalyzeApp(pxModbusSlave);
err_ret:
	pxModbusSlave->pxModbusBase->usRXCnt = 0; /* ��������������������´�֡ͬ�� */
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusSlaveSendWithCRC
*	����˵��: ����һ������, �Զ�׷��2�ֽ�CRC
*	��    ��: pucBuf ���ݣ�
*			  ucLen ���ݳ��ȣ�����CRC��
*	�� �� ֵ: ��
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
*	�� �� ��: ModbusSlaveSendAckErr
*	����˵��: ���ʹ���Ӧ��
*	��    ��: _ucErrCode : �������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusSlaveSendAckErr(ModbusSlave_t *pxModbusSlave)
{
	uint8_t ucTXBuf[3];
	ucTXBuf[0] = pxModbusSlave->pxModbusBase->pucRXBuf[0];		  /* 485��ַ */
	ucTXBuf[1] = pxModbusSlave->pxModbusBase->pucRXBuf[1] | 0x80; /* �쳣�Ĺ����� */
	ucTXBuf[2] = pxModbusSlave->ucRspCode;						  /* �������(01,02,03,04) */
	ModbusSlaveSendWithCRC(pxModbusSlave, ucTXBuf, 3);
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusSlaveSendAckOk
*	����˵��: ������ȷ��Ӧ��.
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: ModbusSlaveAnalyzeApp
*	����˵��: ����Ӧ�ò�Э��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusSlaveAnalyzeApp(ModbusSlave_t *pxModbusSlave)
{
	switch (pxModbusSlave->pxModbusBase->pucRXBuf[1]) /* ��2���ֽ� ������ */
	{
	case 0x01: /* ��ȡ��Ȧ״̬����������led���棩*/
		ModbusSlave01H(pxModbusSlave);
		break;
	case 0x02: /* ��ȡ����״̬������״̬��*/
		ModbusSlave02H(pxModbusSlave);
		break;
	case 0x03: /* ��ȡ���ּĴ����������̴���g_tVar�У�*/
		ModbusSlave03H(pxModbusSlave);
		break;
	case 0x04: /* ��ȡ����Ĵ�����ADC��ֵ��*/
		ModbusSlave04H(pxModbusSlave);
		break;
	case 0x05: /* ǿ�Ƶ���Ȧ������led��*/
		ModbusSlave05H(pxModbusSlave);
		break;
	case 0x06: /* д��������Ĵ����������̸�дg_tVar�еĲ�����*/
		ModbusSlave06H(pxModbusSlave);
		break;
	case 0x10: /* д�������Ĵ����������̴���g_tVar�еĲ�����*/
		ModbusSlave10H(pxModbusSlave);
		break;
	default:
		pxModbusSlave->ucRspCode = RSP_ERR_CMD;
		ModbusSlaveSendAckErr(pxModbusSlave); /* ��������������� */
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
*	�� �� ��: ModbusSlave03HCallBack
*	����˵��: ��ȡ���ּĴ�����ֵ
*	��    ��: reg_addr �Ĵ�����ַ
*			  reg_value ��żĴ������
*	�� �� ֵ: 1��ʾOK 0��ʾ����
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
*	�� �� ��: ModbusSlave06H10HCallBack
*	����˵��: ��ȡ���ּĴ�����ֵ
*	��    ��: reg_addr �Ĵ�����ַ
*			  reg_value �Ĵ���ֵ
*	�� �� ֵ: 1��ʾOK 0��ʾ����
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
*	�� �� ��: ModbusSlave01H
*	����˵��: ��ȡ��Ȧ״̬����ӦԶ�̿���D01/D02/D03��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
/* ˵��:������LED����̵���,���ڹ۲����� */
static void ModbusSlave01H(ModbusSlave_t *pxModbusSlave)
{
	/*
	 ������
		��������:
			11 �ӻ���ַ
			01 ������
			00 �Ĵ�����ʼ��ַ���ֽ�
			13 �Ĵ�����ʼ��ַ���ֽ�
			00 �Ĵ����������ֽ�
			25 �Ĵ����������ֽ�
			0E CRCУ����ֽ�
			84 CRCУ����ֽ�
		�ӻ�Ӧ��: 	1����ON��0����OFF�������ص���Ȧ����Ϊ8�ı�����������������ֽ�δβʹ��0����. BIT0��Ӧ��1��
			11 �ӻ���ַ
			01 ������
			05 �����ֽ���
			CD ����1(��Ȧ0013H-��Ȧ001AH)
			6B ����2(��Ȧ001BH-��Ȧ0022H)
			B2 ����3(��Ȧ0023H-��Ȧ002AH)
			0E ����4(��Ȧ0032H-��Ȧ002BH)
			1B ����5(��Ȧ0037H-��Ȧ0033H)
			45 CRCУ����ֽ�
			E6 CRCУ����ֽ�
		����:
			01 01 10 01 00 03   29 0B	--- ��ѯD01��ʼ��3���̵���״̬
			01 01 10 03 00 01   09 0A   --- ��ѯD03�̵�����״̬
	*/
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint16_t m;
	uint8_t status[10];

	pxModbusSlave->ucRspCode = RSP_OK;

	/* û���ⲿ�̵�����ֱ��Ӧ����� */
	if (pxModbusSlave->pxModbusBase->usRXCnt != 8)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* ����ֵ����� */
		return;
	}

	reg = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[2]); /* �Ĵ����� */
	num = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[4]); /* �Ĵ������� */

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
		pxModbusSlave->ucRspCode = RSP_ERR_REG_ADDR; /* �Ĵ�����ַ���� */
	}

	if (pxModbusSlave->ucRspCode == RSP_OK) /* ��ȷӦ�� */
	{
		pxModbusSlave->pxModbusBase->usTXCnt = 0;
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = pxModbusSlave->pxModbusBase->pucRXBuf[0];
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = pxModbusSlave->pxModbusBase->pucRXBuf[1];
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = m; /* �����ֽ��� */

		for (i = 0; i < m; i++)
			pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = status[i]; /* �̵���״̬ */
		ModbusSlaveSendWithCRC(pxModbusSlave, pxModbusSlave->pxModbusBase->pucTXBuf, pxModbusSlave->pxModbusBase->usTXCnt);
	}
	else
	{
		ModbusSlaveSendAckErr(pxModbusSlave); /* ��������������� */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusSlave02H
*	����˵��: ��ȡ����״̬����ӦK01��K03��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusSlave02H(ModbusSlave_t *pxModbusSlave)
{
	/*
		��������:
			11 �ӻ���ַ
			02 ������
			00 �Ĵ�����ַ���ֽ�
			C4 �Ĵ�����ַ���ֽ�
			00 �Ĵ����������ֽ�
			16 �Ĵ����������ֽ�
			BA CRCУ����ֽ�
			A9 CRCУ����ֽ�
		�ӻ�Ӧ��:  ��Ӧ����ɢ����Ĵ���״̬���ֱ��Ӧ�������е�ÿλֵ��1 ����ON��0 ����OFF��
		           ��һ�������ֽڵ�LSB(����ֽ�)Ϊ��ѯ��Ѱַ��ַ����������ڰ�˳���ڸ��ֽ����ɵ��ֽ�
		           ����ֽ����У�ֱ�������8λ����һ���ֽ��е�8������λҲ�Ǵӵ��ֽڵ����ֽ����С�
		           �����ص�����λ������8�ı������������������ֽ��е�ʣ��λ�����ֽڵ����λʹ��0��䡣
			11 �ӻ���ַ
			02 ������
			03 �����ֽ���
			AC ����1(00C4H-00CBH)
			DB ����2(00CCH-00D3H)
			35 ����3(00D4H-00D9H)
			20 CRCУ����ֽ�
			18 CRCУ����ֽ�
		����:
		01 02 20 01 00 08  23CC  ---- ��ȡT01-08��״̬
		01 02 20 04 00 02  B3CA  ---- ��ȡT04-05��״̬
		01 02 20 01 00 12  A207   ---- �� T01-18
	*/
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint16_t m;
	uint8_t status[10];

	pxModbusSlave->ucRspCode = RSP_OK;

	if (pxModbusSlave->pxModbusBase->usRXCnt != 8)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* ����ֵ����� */
		return;
	}

	reg = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[2]); /* �Ĵ����� */
	num = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[4]); /* �Ĵ������� */

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
		pxModbusSlave->ucRspCode = RSP_ERR_REG_ADDR; /* �Ĵ�����ַ���� */
	}

	if (pxModbusSlave->ucRspCode == RSP_OK) /* ��ȷӦ�� */
	{
		pxModbusSlave->pxModbusBase->usTXCnt = 0;
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = pxModbusSlave->pxModbusBase->pucRXBuf[0];
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = pxModbusSlave->pxModbusBase->pucRXBuf[1];
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = m; /* �����ֽ��� */

		for (i = 0; i < m; i++)
			pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = status[i]; /* T01-02״̬ */
		ModbusSlaveSendWithCRC(pxModbusSlave, pxModbusSlave->pxModbusBase->pucTXBuf, pxModbusSlave->pxModbusBase->usTXCnt);
	}
	else
	{
		ModbusSlaveSendAckErr(pxModbusSlave); /* ��������������� */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusSlave03H
*	����˵��: ��ȡ���ּĴ��� ��һ���������ּĴ�����ȡ�õ�ǰ�Ķ�����ֵ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusSlave03H(ModbusSlave_t *pxModbusSlave)
{
	/*
		�ӻ���ַΪ11H�����ּĴ�������ʼ��ַΪ006BH��������ַΪ006DH���ôβ�ѯ�ܹ�����3�����ּĴ�����
		��������:
			11 �ӻ���ַ
			03 ������
			00 �Ĵ�����ַ���ֽ�
			6B �Ĵ�����ַ���ֽ�
			00 �Ĵ����������ֽ�
			03 �Ĵ����������ֽ�
			76 CRC���ֽ�
			87 CRC���ֽ�
		�ӻ�Ӧ��: 	���ּĴ����ĳ���Ϊ2���ֽڡ����ڵ������ּĴ������ԣ��Ĵ������ֽ������ȱ����䣬
					���ֽ����ݺ󱻴��䡣���ּĴ���֮�䣬�͵�ַ�Ĵ����ȱ����䣬�ߵ�ַ�Ĵ����󱻴��䡣
			11 �ӻ���ַ
			03 ������
			06 �ֽ���
			00 ����1���ֽ�(006BH)
			6B ����1���ֽ�(006BH)
			00 ����2���ֽ�(006CH)
			13 ����2 ���ֽ�(006CH)
			00 ����3���ֽ�(006DH)
			00 ����3���ֽ�(006DH)
			38 CRC���ֽ�
			B9 CRC���ֽ�
		����:
			01 03 30 06 00 01  6B0B      ---- �� 3006H, ��������
			01 03 4000 0010 51C6         ---- �� 4000H ������1����ӿ��¼ 32�ֽ�
			01 03 4001 0010 0006         ---- �� 4001H ������1����ӿ��¼ 32�ֽ�
			01 03 F000 0008 770C         ---- �� F000H ������1���澯��¼ 16�ֽ�
			01 03 F001 0008 26CC         ---- �� F001H ������2���澯��¼ 16�ֽ�
			01 03 7000 0020 5ED2         ---- �� 7000H ������1�����μ�¼��1�� 64�ֽ�
			01 03 7001 0020 0F12         ---- �� 7001H ������1�����μ�¼��2�� 64�ֽ�
			01 03 7040 0020 5F06         ---- �� 7040H ������2�����μ�¼��1�� 64�ֽ�
	*/
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint8_t reg_value[256] = {0};

	pxModbusSlave->ucRspCode = RSP_OK;

	if (pxModbusSlave->pxModbusBase->usRXCnt != 8) /* 03H���������8���ֽ� */
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* ����ֵ����� */
		goto err_ret;
	}

	reg = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[2]); /* �Ĵ����� */
	num = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[4]); /* �Ĵ������� */

	if (num > sizeof(reg_value) / 2)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* ����ֵ����� */
		goto err_ret;
	}

	if (ModbusSlave03HCallBack(pxModbusSlave, reg, (uint16_t *)reg_value, num) == 0)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_REG_ADDR;
	}

err_ret:
	if (pxModbusSlave->ucRspCode == RSP_OK) /* ��ȷӦ�� */
	{
		pxModbusSlave->pxModbusBase->usTXCnt = 0;
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = pxModbusSlave->pxModbusBase->pucRXBuf[0];
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = pxModbusSlave->pxModbusBase->pucRXBuf[1];
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = num * 2; /* �����ֽ��� */

		for (i = 0; i < num; i++)
		{
			pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = reg_value[2 * i];
			pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = reg_value[2 * i + 1];
		}
		ModbusSlaveSendWithCRC(pxModbusSlave, pxModbusSlave->pxModbusBase->pucTXBuf, pxModbusSlave->pxModbusBase->usTXCnt); /* ������ȷӦ�� */
	}
	else
	{
		ModbusSlaveSendAckErr(pxModbusSlave); /* ���ʹ���Ӧ�� */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusSlave04H
*	����˵��: ��ȡ����Ĵ�������ӦA01/A02�� SMA
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusSlave04H(ModbusSlave_t *pxModbusSlave)
{
	/*
		��������:
			11 �ӻ���ַ
			04 ������
			00 �Ĵ�����ʼ��ַ���ֽ�
			08 �Ĵ�����ʼ��ַ���ֽ�
			00 �Ĵ����������ֽ�
			02 �Ĵ����������ֽ�
			F2 CRC���ֽ�
			99 CRC���ֽ�
		�ӻ�Ӧ��:  ����Ĵ�������Ϊ2���ֽڡ����ڵ�������Ĵ������ԣ��Ĵ������ֽ������ȱ����䣬
				���ֽ����ݺ󱻴��䡣����Ĵ���֮�䣬�͵�ַ�Ĵ����ȱ����䣬�ߵ�ַ�Ĵ����󱻴��䡣
			11 �ӻ���ַ
			04 ������
			04 �ֽ���
			00 ����1���ֽ�(0008H)
			0A ����1���ֽ�(0008H)
			00 ����2���ֽ�(0009H)
			0B ����2���ֽ�(0009H)
			8B CRC���ֽ�
			80 CRC���ֽ�
		����:
			01 04 2201 0006 2BB0  --- �� 2201H A01ͨ��ģ���� ��ʼ��6������
			01 04 2201 0001 6A72  --- �� 2201H
	*/
	uint16_t reg;
	uint16_t num;
	uint16_t i;
	uint8_t status[256] = {0};

	pxModbusSlave->ucRspCode = RSP_OK;

	if (pxModbusSlave->pxModbusBase->usRXCnt != 8)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* ����ֵ����� */
		goto err_ret;
	}

	reg = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[2]); /* �Ĵ����� */
	num = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[4]); /* �Ĵ������� */

	if (num > sizeof(status) / 2)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* ����ֵ����� */
		goto err_ret;
	}

	if (ModbusSlave04HCallBack(pxModbusSlave, reg, (uint16_t *)status, num) == 0)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_REG_ADDR;
	}

err_ret:
	if (pxModbusSlave->ucRspCode == RSP_OK) /* ��ȷӦ�� */
	{
		pxModbusSlave->pxModbusBase->usTXCnt = 0;
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = pxModbusSlave->pxModbusBase->pucRXBuf[0];
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = pxModbusSlave->pxModbusBase->pucRXBuf[1];
		pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = num * 2; /* �����ֽ��� */

		for (i = 0; i < num; i++)
		{
			pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = status[2 * i];
			pxModbusSlave->pxModbusBase->pucTXBuf[pxModbusSlave->pxModbusBase->usTXCnt++] = status[2 * i + 1];
		}
		ModbusSlaveSendWithCRC(pxModbusSlave, pxModbusSlave->pxModbusBase->pucTXBuf, pxModbusSlave->pxModbusBase->usTXCnt);
	}
	else
	{
		ModbusSlaveSendAckErr(pxModbusSlave); /* ��������������� */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusSlave05H
*	����˵��: ǿ�Ƶ���Ȧ����ӦD01/D02/D03��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusSlave05H(ModbusSlave_t *pxModbusSlave)
{
	/*
		��������: д������Ȧ�Ĵ�����FF00Hֵ������Ȧ����ON״̬��0000Hֵ������Ȧ����OFF״̬
		��05Hָ�����õ�����Ȧ��״̬��15Hָ��������ö����Ȧ��״̬��
			11 �ӻ���ַ
			05 ������
			00 �Ĵ�����ַ���ֽ�
			AC �Ĵ�����ַ���ֽ�
			FF ����1���ֽ�
			00 ����2���ֽ�
			4E CRCУ����ֽ�
			8B CRCУ����ֽ�
		�ӻ�Ӧ��:
			11 �ӻ���ַ
			05 ������
			00 �Ĵ�����ַ���ֽ�
			AC �Ĵ�����ַ���ֽ�
			FF �Ĵ���1���ֽ�
			00 �Ĵ���1���ֽ�
			4E CRCУ����ֽ�
			8B CRCУ����ֽ�
		����:
		01 05 10 01 FF 00   D93A   -- D01��
		01 05 10 01 00 00   98CA   -- D01�ر�
		01 05 10 02 FF 00   293A   -- D02��
		01 05 10 02 00 00   68CA   -- D02�ر�
		01 05 10 03 FF 00   78FA   -- D03��
		01 05 10 03 00 00   390A   -- D03�ر�
	*/
	uint16_t reg;
	uint16_t value;

	pxModbusSlave->ucRspCode = RSP_OK;

	if (pxModbusSlave->pxModbusBase->usRXCnt != 8)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* ����ֵ����� */
		goto err_ret;
	}

	reg = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[2]);	  /* �Ĵ����� */
	value = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[4]); /* ���� */

	if (value != 0 && value != 1)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* ����ֵ����� */
		goto err_ret;
	}
	if (reg >= pxModbusSlave->pxModbusBase->usCoilStartAddr && reg < (pxModbusSlave->pxModbusBase->usCoilStartAddr + pxModbusSlave->pxModbusBase->usCoilSize))
	{
		pxModbusSlave->pxModbusBase->pusCoilBuf[reg - pxModbusSlave->pxModbusBase->usCoilStartAddr] = value;
	}
	else
	{
		pxModbusSlave->ucRspCode = RSP_ERR_REG_ADDR; /* �Ĵ�����ַ���� */
	}
err_ret:
	if (pxModbusSlave->ucRspCode == RSP_OK) /* ��ȷӦ�� */
	{
		ModbusSlaveSendAckOk(pxModbusSlave);
	}
	else
	{
		ModbusSlaveSendAckErr(pxModbusSlave); /* ��������������� */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusSlave06H
*	����˵��: д�����Ĵ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusSlave06H(ModbusSlave_t *pxModbusSlave)
{

	/*
		д���ּĴ�����ע��06ָ��ֻ�ܲ����������ּĴ�����16ָ��������õ����������ּĴ���
		��������:
			11 �ӻ���ַ
			06 ������
			00 �Ĵ�����ַ���ֽ�
			01 �Ĵ�����ַ���ֽ�
			00 ����1���ֽ�
			01 ����1���ֽ�
			9A CRCУ����ֽ�
			9B CRCУ����ֽ�
		�ӻ���Ӧ:
			11 �ӻ���ַ
			06 ������
			00 �Ĵ�����ַ���ֽ�
			01 �Ĵ�����ַ���ֽ�
			00 ����1���ֽ�
			01 ����1���ֽ�
			1B CRCУ����ֽ�
			5A	CRCУ����ֽ�
		����:
			01 06 30 06 00 25  A710    ---- ������������Ϊ 2.5
			01 06 30 06 00 10  6707    ---- ������������Ϊ 1.0
			01 06 30 1B 00 00  F6CD    ---- SMA �˲�ϵ�� = 0 �ر��˲�
			01 06 30 1B 00 01  370D    ---- SMA �˲�ϵ�� = 1
			01 06 30 1B 00 02  770C    ---- SMA �˲�ϵ�� = 2
			01 06 30 1B 00 05  36CE    ---- SMA �˲�ϵ�� = 5
			01 06 30 07 00 01  F6CB    ---- ����ģʽ�޸�Ϊ T1
			01 06 30 07 00 02  B6CA    ---- ����ģʽ�޸�Ϊ T2
			01 06 31 00 00 00  8736    ---- ������ӿ��¼��
			01 06 31 01 00 00  D6F6    ---- �����澯��¼��
*/
	uint16_t reg;
	uint16_t value;

	pxModbusSlave->ucRspCode = RSP_OK;

	if (pxModbusSlave->pxModbusBase->usRXCnt != 8)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* ����ֵ����� */
		goto err_ret;
	}

	reg = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[2]);	  /* �Ĵ����� */
	value = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[4]); /* �Ĵ���ֵ */

	if (ModbusSlave06H10HCallBack(pxModbusSlave, reg, &value, 1) == 0) /* �ú������д���ֵ����Ĵ��� */
	{
		pxModbusSlave->ucRspCode = RSP_ERR_REG_ADDR; /* �Ĵ�����ַ���� */
	}

err_ret:
	if (pxModbusSlave->ucRspCode == RSP_OK) /* ��ȷӦ�� */
	{
		ModbusSlaveSendAckOk(pxModbusSlave);
	}
	else
	{
		ModbusSlaveSendAckErr(pxModbusSlave); /* ��������������� */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusSlave10H
*	����˵��: ����д����Ĵ���.  �����ڸ�дʱ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusSlave10H(ModbusSlave_t *pxModbusSlave)
{
	/*
		�ӻ���ַΪ11H�����ּĴ�������ʵ��ַΪ0001H���Ĵ����Ľ�����ַΪ0002H���ܹ�����2���Ĵ�����
		���ּĴ���0001H������Ϊ000AH�����ּĴ���0002H������Ϊ0102H��
		��������:
			11 �ӻ���ַ
			10 ������
			00 �Ĵ�����ʼ��ַ���ֽ�
			01 �Ĵ�����ʼ��ַ���ֽ�
			00 �Ĵ����������ֽ�
			02 �Ĵ����������ֽ�
			04 �ֽ���
			00 ����1���ֽ�
			0A ����1���ֽ�
			01 ����2���ֽ�
			02 ����2���ֽ�
			C6 CRCУ����ֽ�
			F0 CRCУ����ֽ�
		�ӻ���Ӧ:
			11 �ӻ���ַ
			06 ������
			00 �Ĵ�����ַ���ֽ�
			01 �Ĵ�����ַ���ֽ�
			00 ����1���ֽ�
			01 ����1���ֽ�
			1B CRCУ����ֽ�
			5A	CRCУ����ֽ�
		����:
			01 10 30 00 00 06 0C  07 DE  00 0A  00 01  00 08  00 0C  00 00     389A    ---- дʱ�� 2014-10-01 08:12:00
			01 10 30 00 00 06 0C  07 DF  00 01  00 1F  00 17  00 3B  00 39     5549    ---- дʱ�� 2015-01-31 23:59:57
	*/
	uint16_t reg_addr;
	uint16_t reg_num;
	uint8_t byte_num;
	uint16_t value[256];

	pxModbusSlave->ucRspCode = RSP_OK;

	if (pxModbusSlave->pxModbusBase->usRXCnt < 11)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* ����ֵ����� */
		goto err_ret;
	}

	reg_addr = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[2]); /* �Ĵ����� */
	reg_num = BEBufToUint16(&pxModbusSlave->pxModbusBase->pucRXBuf[4]);	 /* �Ĵ������� */
	byte_num = pxModbusSlave->pxModbusBase->pucRXBuf[6];				 /* ������������ֽ��� */

	if (byte_num != 2 * reg_num)
	{
		pxModbusSlave->ucRspCode = RSP_ERR_VALUE; /* ����ֵ����� */
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
	if (pxModbusSlave->ucRspCode == RSP_OK) /* ��ȷӦ�� */
	{
		ModbusSlaveSendAckOk(pxModbusSlave);
	}
	else
	{
		ModbusSlaveSendAckErr(pxModbusSlave); /* ��������������� */
	}
}
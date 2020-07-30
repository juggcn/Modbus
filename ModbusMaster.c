/*
*********************************************************************************************************
*
*	ģ������ : MODSBUSͨ�ų��� ��������
*	�ļ����� : modbus_host.c
*	��    �� : V1.4
*	˵    �� : ����ͨ�ų���ͨ��Э�����MODBUS
*	�޸ļ�¼ :
*		�汾��  ����        ����    ˵��
*       V1.4   2015-11-28 �޸�Э��
*
*	Copyright (C), 2015-2016, ���������� www.armfly.com
*
*********************************************************************************************************
*/
#include "ModbusMaster.h"

/* RTU Ӧ����� */
#define RSP_OK 0x00				   /* �ɹ� */
#define RSP_ERR_CMD 0x01		   /* ��֧�ֵĹ����� */
#define RSP_ERRusRagAddr_ADDR 0x02 /* �Ĵ�����ַ���� */
#define RSP_ERRusRagValue 0x03	   /* ����ֵ����� */
#define RSP_ERR_WRITE 0x04		   /* д��ʧ�� */

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
*	��    ��: _pBuf : ����У�������
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
static uint16_t CRC16Modbus(uint8_t *_pBuf, uint16_t _usLen)
{
	uint8_t ucCRCHi = 0xFF; /* ��CRC�ֽڳ�ʼ�� */
	uint8_t ucCRCLo = 0xFF; /* ��CRC �ֽڳ�ʼ�� */
	uint16_t usIndex;		/* CRCѭ���е����� */

	while (_usLen--)
	{
		usIndex = ucCRCHi ^ *_pBuf++; /* ����CRC */
		ucCRCHi = ucCRCLo ^ s_CRCHi[usIndex];
		ucCRCLo = s_CRCLo[usIndex];
	}
	return ((uint16_t)ucCRCHi << 8 | ucCRCLo);
}

/*
*********************************************************************************************************
*	�� �� ��: BEBufToUint16
*	����˵��: ��2�ֽ�����(���Big Endian���򣬸��ֽ���ǰ)ת��Ϊ16λ����
*	��    ��: _pBuf : ����
*	�� �� ֵ: 16λ����ֵ
*
*   ���(Big Endian)��С��(Little Endian)
*********************************************************************************************************
*/
static uint16_t BEBufToUint16(uint8_t *_pBuf)
{
	return (((uint16_t)_pBuf[0] << 8) | _pBuf[1]);
}
/*********************************************************************************************/

/* ����ÿ���ӻ��ļ�����ֵ */

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
*	�� �� ��: ModbusMasterSendAckWithCRC
*	����˵��: ����Ӧ��,�Զ���CRC.  
*	��    ��: �ޡ����������� pxModbusMaster->ucTXBuf[], [pxModbusMaster->usTXCnt
*	�� �� ֵ: ��
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
*	�� �� ��: ModbusMasterAnalyzeApp
*	����˵��: ����Ӧ�ò�Э�顣����Ӧ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusMasterAnalyzeApp(ModbusMaster_t *pxModbusMaster)
{
	switch (pxModbusMaster->pxModbusBase->pucRXBuf[1]) /* ��2���ֽ� ������ */
	{
	case 0x01: /* ��ȡ��Ȧ״̬ */
		ModbusMaster01H(pxModbusMaster);
		break;

	case 0x02: /* ��ȡ����״̬ */
		ModbusMaster02H(pxModbusMaster);
		break;

	case 0x03: /* ��ȡ���ּĴ��� ��һ���������ּĴ�����ȡ�õ�ǰ�Ķ�����ֵ */
		ModbusMaster03H(pxModbusMaster);
		break;

	case 0x04: /* ��ȡ����Ĵ��� */
		ModbusMaster04H(pxModbusMaster);
		break;

	case 0x05: /* ǿ�Ƶ���Ȧ */
		ModbusMaster05H(pxModbusMaster);
		break;

	case 0x06: /* д�����Ĵ��� */
		ModbusMaster06H(pxModbusMaster);
		break;

	case 0x10: /* д����Ĵ��� */
		ModbusMaster10H(pxModbusMaster);
		break;

	default:
		break;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusMasterSend01H
*	����˵��: ����01Hָ���ѯ1���������ּĴ���
*	��    ��: ucAddr : ��վ��ַ
*			  usRagAddr : �Ĵ������
*			  usRagNum : �Ĵ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusMasterSend01H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagNum)
{
	pxModbusMaster->pxModbusBase->usTXCnt = 0;
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = pxModbusMaster->pxModbusBase->ucAddr; /* ��վ��ַ */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = 0x01;									/* ������ */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr >> 8;						/* �Ĵ������ ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr;							/* �Ĵ������ ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum >> 8;						/* �Ĵ������� ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum;								/* �Ĵ������� ���ֽ� */

	ModbusMasterSendAckWithCRC(pxModbusMaster); /* �������ݣ��Զ���CRC */

	pxModbusMaster->ucRegNum = usRagNum;  /* �Ĵ������� */
	pxModbusMaster->usReg01H = usRagAddr; /* ����03Hָ���еļĴ�����ַ�������Ӧ�����ݽ��з��� */
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusMasterSend02H
*	����˵��: ����02Hָ�����ɢ����Ĵ���
*	��    ��: ucAddr : ��վ��ַ
*			  usRagAddr : �Ĵ������
*			  usRagNum : �Ĵ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusMasterSend02H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagNum)
{
	pxModbusMaster->pxModbusBase->usTXCnt = 0;
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = pxModbusMaster->pxModbusBase->ucAddr; /* ��վ��ַ */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = 0x02;									/* ������ */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr >> 8;						/* �Ĵ������ ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr;							/* �Ĵ������ ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum >> 8;						/* �Ĵ������� ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum;								/* �Ĵ������� ���ֽ� */

	ModbusMasterSendAckWithCRC(pxModbusMaster); /* �������ݣ��Զ���CRC */

	pxModbusMaster->ucRegNum = usRagNum;  /* �Ĵ������� */
	pxModbusMaster->usReg02H = usRagAddr; /* ����03Hָ���еļĴ�����ַ�������Ӧ�����ݽ��з��� */
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusMasterSend03H
*	����˵��: ����03Hָ���ѯ1���������ּĴ���
*	��    ��: ucAddr : ��վ��ַ
*			  usRagAddr : �Ĵ������
*			  usRagNum : �Ĵ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusMasterSend03H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagNum)
{
	pxModbusMaster->pxModbusBase->usTXCnt = 0;
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = pxModbusMaster->pxModbusBase->ucAddr; /* ��վ��ַ */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = 0x03;									/* ������ */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr >> 8;						/* �Ĵ������ ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr;							/* �Ĵ������ ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum >> 8;						/* �Ĵ������� ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum;								/* �Ĵ������� ���ֽ� */

	ModbusMasterSendAckWithCRC(pxModbusMaster); /* �������ݣ��Զ���CRC */

	pxModbusMaster->ucRegNum = usRagNum;  /* �Ĵ������� */
	pxModbusMaster->usReg03H = usRagAddr; /* ����03Hָ���еļĴ�����ַ�������Ӧ�����ݽ��з��� */
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusMasterSend04H
*	����˵��: ����04Hָ�������Ĵ���
*	��    ��: ucAddr : ��վ��ַ
*			  usRagAddr : �Ĵ������
*			  usRagNum : �Ĵ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusMasterSend04H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagNum)
{
	pxModbusMaster->pxModbusBase->usTXCnt = 0;
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = pxModbusMaster->pxModbusBase->ucAddr; /* ��վ��ַ */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = 0x04;									/* ������ */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr >> 8;						/* �Ĵ������ ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr;							/* �Ĵ������ ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum >> 8;						/* �Ĵ������� ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum;								/* �Ĵ������� ���ֽ� */

	ModbusMasterSendAckWithCRC(pxModbusMaster); /* �������ݣ��Զ���CRC */

	pxModbusMaster->ucRegNum = usRagNum;  /* �Ĵ������� */
	pxModbusMaster->usReg04H = usRagAddr; /* ����03Hָ���еļĴ�����ַ�������Ӧ�����ݽ��з��� */
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusMasterSend05H
*	����˵��: ����05Hָ�дǿ�õ���Ȧ
*	��    ��: ucAddr : ��վ��ַ
*			  usRagAddr : �Ĵ������
*			  usRagValue : �Ĵ���ֵ,2�ֽ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusMasterSend05H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagValue)
{
	pxModbusMaster->pxModbusBase->usTXCnt = 0;
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = pxModbusMaster->pxModbusBase->ucAddr; /* ��վ��ַ */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = 0x05;									/* ������ */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr >> 8;						/* �Ĵ������ ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr;							/* �Ĵ������ ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagValue >> 8;						/* �Ĵ���ֵ ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagValue;							/* �Ĵ���ֵ ���ֽ� */

	ModbusMasterSendAckWithCRC(pxModbusMaster); /* �������ݣ��Զ���CRC */

	pxModbusMaster->ucRegNum = 1;
	pxModbusMaster->usReg05H = usRagAddr;
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusMasterSend06H
*	����˵��: ����06Hָ�д1�����ּĴ���
*	��    ��: ucAddr : ��վ��ַ
*			  usRagAddr : �Ĵ������
*			  usRagValue : �Ĵ���ֵ,2�ֽ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusMasterSend06H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagValue)
{
	pxModbusMaster->pxModbusBase->usTXCnt = 0;
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = pxModbusMaster->pxModbusBase->ucAddr; /* ��վ��ַ */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = 0x06;									/* ������ */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr >> 8;						/* �Ĵ������ ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr;							/* �Ĵ������ ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagValue >> 8;						/* �Ĵ���ֵ ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagValue;							/* �Ĵ���ֵ ���ֽ� */

	ModbusMasterSendAckWithCRC(pxModbusMaster); /* �������ݣ��Զ���CRC */

	pxModbusMaster->ucRegNum = 1;
	pxModbusMaster->usReg06H = usRagAddr;
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusMasterSend10H
*	����˵��: ����10Hָ�����д������ּĴ���. ���һ��֧��23���Ĵ�����
*	��    ��: ucAddr : ��վ��ַ
*			  usRagAddr : �Ĵ������
*			  usRagNum : �Ĵ�������n (ÿ���Ĵ���2���ֽ�) ֵ��
*			  pusRagBuf : n���Ĵ��������ݡ����� = 2 * n
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusMasterSend10H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint8_t usRagNum, uint16_t *pusRagBuf)
{
	uint16_t i;
	uint8_t *buf = (uint8_t *)pusRagBuf;

	pxModbusMaster->pxModbusBase->usTXCnt = 0;
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = pxModbusMaster->pxModbusBase->ucAddr; /* ��վ��ַ */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = 0x10;									/* ��վ��ַ */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr >> 8;						/* �Ĵ������ ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagAddr;							/* �Ĵ������ ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum >> 8;						/* �Ĵ������� ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = usRagNum;								/* �Ĵ������� ���ֽ� */
	pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = 2 * usRagNum;							/* �����ֽ��� */

	for (i = 0; i < usRagNum; i++)
	{
		if (pxModbusMaster->pxModbusBase->usTXCnt > (pxModbusMaster->pxModbusBase->usTXLen - 2))
		{
			return; /* ���ݳ������������ȣ�ֱ�Ӷ��������� */
		}
		pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = buf[2 * i + 1]; /* ��������ݳ��� */
		pxModbusMaster->pxModbusBase->pucTXBuf[pxModbusMaster->pxModbusBase->usTXCnt++] = buf[2 * i];
	}

	ModbusMasterSendAckWithCRC(pxModbusMaster); /* �������ݣ��Զ���CRC */

	pxModbusMaster->ucRegNum = usRagNum;
	pxModbusMaster->usReg10H = usRagAddr;
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusMasterPoll
*	����˵��: ���տ�����ָ��. 1ms ��Ӧʱ�䡣
*	��    ��: ��
*	�� �� ֵ: 0 ��ʾ������ 1��ʾ�յ���ȷ����
*********************************************************************************************************
*/
static void ModbusMasterPoll(ModbusMaster_t *pxModbusMaster)
{
	if (pxModbusMaster->pxModbusBase->pucRXBuf[0] != pxModbusMaster->pxModbusBase->ucAddr || pxModbusMaster->pxModbusBase->usRXCnt < 4)
		goto err_ret;
	/* ����CRCУ�� */
	if (CRC16Modbus(pxModbusMaster->pxModbusBase->pucRXBuf, pxModbusMaster->pxModbusBase->usRXCnt) != 0)
		goto err_ret;
	/* ����Ӧ�ò�Э�� */
	ModbusMasterAnalyzeApp(pxModbusMaster);
err_ret:
	pxModbusMaster->pxModbusBase->usRXCnt = 0; /* ��������������������´�֡ͬ�� */
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusMaster01H
*	����˵��: ����01Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusMaster01H(ModbusMaster_t *pxModbusMaster)
{
	uint8_t *p;
	uint8_t bytes = pxModbusMaster->pxModbusBase->pucRXBuf[2] >> 1; /* ���ݳ��� �ֽ��� */
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
*	�� �� ��: ModbusMaster02H
*	����˵��: ����02Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusMaster02H(ModbusMaster_t *pxModbusMaster)
{
	uint8_t *p;
	uint8_t bytes = pxModbusMaster->pxModbusBase->pucRXBuf[2] >> 1; /* ���ݳ��� �ֽ��� */
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
*	�� �� ��: ModbusMaster03H
*	����˵��: ����03Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusMaster03H(ModbusMaster_t *pxModbusMaster)
{
	uint8_t *p;
	uint8_t bytes = pxModbusMaster->pxModbusBase->pucRXBuf[2] >> 1; /* ���ݳ��� �ֽ��� */
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
*	�� �� ��: ModbusMaster04H
*	����˵��: ����04Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusMaster04H(ModbusMaster_t *pxModbusMaster)
{
	uint8_t *p;
	uint8_t bytes = pxModbusMaster->pxModbusBase->pucRXBuf[2] >> 1; /* ���ݳ��� �ֽ��� */
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
*	�� �� ��: ModbusMaster05H
*	����˵��: ����05Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusMaster05H(ModbusMaster_t *pxModbusMaster)
{
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusMaster06H
*	����˵��: ����06Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusMaster06H(ModbusMaster_t *pxModbusMaster)
{
}

/*
*********************************************************************************************************
*	�� �� ��: ModbusMaster10H
*	����˵��: ����10Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ModbusMaster10H(ModbusMaster_t *pxModbusMaster)
{
}
/*
*********************************************************************************************************
*	�� �� ��: ModbusMasterReadParam01H
*	����˵��: ��������. ͨ������01Hָ��ʵ�֣�����֮�󣬵ȴ��ӻ�Ӧ��
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ�ͨ�ų�ʱ�򱻾ܾ���
*********************************************************************************************************
*/
uint8_t ModbusMasterReadParam01H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagNum)
{
	pxModbusMaster->pxModbusBase->pvLock(pxModbusMaster->pxModbusBase);
	uint8_t i;
	uint8_t res = 0;
	for (i = 0; i < pxModbusMaster->ucRXErrNum; i++)
	{
		ModbusMasterSend01H(pxModbusMaster, usRagAddr, usRagNum); /* �������� */
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
*	�� �� ��: ModbusMasterReadParam02H
*	����˵��: ��������. ͨ������02Hָ��ʵ�֣�����֮�󣬵ȴ��ӻ�Ӧ��
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ�ͨ�ų�ʱ�򱻾ܾ���
*********************************************************************************************************
*/
uint8_t ModbusMasterReadParam02H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagNum)
{
	pxModbusMaster->pxModbusBase->pvLock(pxModbusMaster->pxModbusBase);
	uint8_t i;
	uint8_t res = 0;
	for (i = 0; i < pxModbusMaster->ucRXErrNum; i++)
	{
		ModbusMasterSend02H(pxModbusMaster, usRagAddr, usRagNum); /* �������� */
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
*	�� �� ��: ModbusMasterReadParam03H
*	����˵��: ��������. ͨ������03Hָ��ʵ�֣�����֮�󣬵ȴ��ӻ�Ӧ��
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ�ͨ�ų�ʱ�򱻾ܾ���
*********************************************************************************************************
*/
uint8_t ModbusMasterReadParam03H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagNum)
{
	pxModbusMaster->pxModbusBase->pvLock(pxModbusMaster->pxModbusBase);
	uint8_t i;
	uint8_t res = 0;
	for (i = 0; i < pxModbusMaster->ucRXErrNum; i++)
	{
		ModbusMasterSend03H(pxModbusMaster, usRagAddr, usRagNum); /* �������� */
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
*	�� �� ��: ModbusMasterReadParam04H
*	����˵��: ��������. ͨ������04Hָ��ʵ�֣�����֮�󣬵ȴ��ӻ�Ӧ��
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ�ͨ�ų�ʱ�򱻾ܾ���
*********************************************************************************************************
*/
uint8_t ModbusMasterReadParam04H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagNum)
{
	pxModbusMaster->pxModbusBase->pvLock(pxModbusMaster->pxModbusBase);
	uint8_t i;
	uint8_t res = 0;
	for (i = 0; i < pxModbusMaster->ucRXErrNum; i++)
	{
		ModbusMasterSend04H(pxModbusMaster, usRagAddr, usRagNum); /* �������� */
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
*	�� �� ��: ModbusMasterWriteParam05H
*	����˵��: ��������. ͨ������05Hָ��ʵ�֣�����֮�󣬵ȴ��ӻ�Ӧ��
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ�ͨ�ų�ʱ�򱻾ܾ���
*********************************************************************************************************
*/
uint8_t ModbusMasterWriteParam05H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagValue)
{
	pxModbusMaster->pxModbusBase->pvLock(pxModbusMaster->pxModbusBase);
	uint8_t i;
	uint8_t res = 0;
	for (i = 0; i < pxModbusMaster->ucRXErrNum; i++)
	{
		ModbusMasterSend05H(pxModbusMaster, usRagAddr, usRagValue); /* �������� */
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
*	�� �� ��: ModbusMasterWriteParam06H
*	����˵��: ��������. ͨ������06Hָ��ʵ�֣�����֮�󣬵ȴ��ӻ�Ӧ��ѭ��NUM��д����
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ�ͨ�ų�ʱ�򱻾ܾ���
*********************************************************************************************************
*/
uint8_t ModbusMasterWriteParam06H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint16_t usRagValue)
{
	pxModbusMaster->pxModbusBase->pvLock(pxModbusMaster->pxModbusBase);
	uint8_t i;
	uint8_t res = 0;
	for (i = 0; i < pxModbusMaster->ucRXErrNum; i++)
	{
		ModbusMasterSend06H(pxModbusMaster, usRagAddr, usRagValue); /* �������� */
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
*	�� �� ��: ModbusMasterWriteParam10H
*	����˵��: ��������. ͨ������10Hָ��ʵ�֣�����֮�󣬵ȴ��ӻ�Ӧ��ѭ��NUM��д����
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ�ͨ�ų�ʱ�򱻾ܾ���
*********************************************************************************************************
*/
uint8_t ModbusMasterWriteParam10H(ModbusMaster_t *pxModbusMaster, uint16_t usRagAddr, uint8_t usRagNum, uint16_t *pusRagBuf)
{
	pxModbusMaster->pxModbusBase->pvLock(pxModbusMaster->pxModbusBase);
	uint8_t i;
	uint8_t res = 0;
	for (i = 0; i < pxModbusMaster->ucRXErrNum; i++)
	{
		ModbusMasterSend10H(pxModbusMaster, usRagAddr, usRagNum, pusRagBuf); /* �������� */
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

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
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
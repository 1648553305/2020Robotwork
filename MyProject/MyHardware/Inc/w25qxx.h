#ifndef __W25QXX_H
#define __W25QXX_H
#include "usart.h"

//W25Xϵ��/Qϵ��оƬ�б�	   
//W25Q80  ID  0XEF13
//W25Q16  ID  0XEF14
//W25Q32  ID  0XEF15
//W25Q64  ID  0XEF16	
//W25Q128 ID  0XEF17	
//W25Q256 ID  0XEF18
#define W25Q80 	0XEF13 	
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16
#define W25Q128	0XEF17
#define W25Q256 0XEF18

extern uint16_t W25QXX_TYPE;					//����W25QXXоƬ�ͺ�		   

#define	W25QXX_CS 		PBout(14)  		//W25QXX��Ƭѡ�ź�

////////////////////////////////////////////////////////////////////////////////// 
//ָ���
#define W25X_WriteEnable		0x06 
#define W25X_WriteDisable		0x04 
#define W25X_ReadStatusReg1		0x05 
#define W25X_ReadStatusReg2		0x35 
#define W25X_ReadStatusReg3		0x15 
#define W25X_WriteStatusReg1    0x01 
#define W25X_WriteStatusReg2    0x31 
#define W25X_WriteStatusReg3    0x11 
#define W25X_ReadData			0x03 
#define W25X_FastReadData		0x0B 
#define W25X_FastReadDual		0x3B 
#define W25X_PageProgram		0x02 
#define W25X_BlockErase			0xD8 
#define W25X_SectorErase		0x20 
#define W25X_ChipErase			0xC7 
#define W25X_PowerDown			0xB9 
#define W25X_ReleasePowerDown	0xAB 
#define W25X_DeviceID			0xAB 
#define W25X_ManufactDeviceID	0x90 
#define W25X_JedecDeviceID		0x9F 
#define W25X_Enable4ByteAddr    0xB7
#define W25X_Exit4ByteAddr      0xE9

void W25QXX_Init(void);
uint16_t  W25QXX_ReadID(void);  	    		//��ȡFLASH ID
uint8_t W25QXX_ReadSR(uint8_t regno);             //��ȡ״̬�Ĵ��� 
void W25QXX_4ByteAddr_Enable(void);     //ʹ��4�ֽڵ�ַģʽ
void W25QXX_Write_SR(uint8_t regno,uint8_t sr);   //д״̬�Ĵ���
void W25QXX_Write_Enable(void);  		//дʹ�� 
void W25QXX_Write_Disable(void);		//д����
void W25QXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);
void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);   //��ȡflash
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);//д��flash
void W25QXX_Erase_Chip(void);    	  	//��Ƭ����
void W25QXX_Erase_Sector(uint32_t Dst_Addr);	//��������
void W25QXX_Wait_Busy(void);           	//�ȴ�����
void W25QXX_PowerDown(void);        	//�������ģʽ
void W25QXX_WAKEUP(void);				//����
void W25QXX_Write_Page(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);
void W25QXX_Put_u8Data(uint8_t Data);  // дu8�����ݵ�flash��
void W25QXX_Put_u16Data(uint16_t Data);  // дu16�����ݵ�flash��
void W25QXX_Put_u32Data(uint32_t Data);  // дu32�����ݵ�flash��
void W25QXX_Put_intData(int Data);  // дint�����ݵ�flash��
void W25QXX_Put_floatData(float Data);  // дfloat�����ݵ�flash��
void W25QXX_Put_doubleData(double Data);  // дdouble�����ݵ�flash��
void W25QXX_Put_charData(char Data);  // дchar�����ݵ�flash��
void W25QXX_Write_Dictionary(uint8_t); // д�����ļ���һ���ò�����
void W25QXX_AllData_Initial(void); // ������������
uint8_t W25QXX_Data_u8Read(uint32_t location); // ��ȡu8����
uint16_t W25QXX_Data_u16Read(uint32_t location); // ��ȡu16����
uint32_t W25QXX_Data_u32Read(uint32_t location); // ��ȡu32����
int W25QXX_Data_intRead(uint32_t location); // ��ȡint����
float W25QXX_Data_floatRead(uint32_t location); // ��ȡfloat����
double W25QXX_Data_doubleRead(uint32_t location); // ��ȡdouble����
char W25QXX_Data_charRead(uint32_t location);// ��ȡchar����
void W25QXX_Data_Fixu8(uint32_t location,uint8_t Data); // �޸�u8����
void W25QXX_Data_Fixu16(uint32_t location,uint16_t Data); // �޸�u16����
void W25QXX_Data_Fixu32(uint32_t location,uint32_t Data); // �޸�u32����
void W25QXX_Data_Fixint(uint32_t location,int Data); // �޸�int����
void W25QXX_Data_Fixfloat(uint32_t location,float Data); // �޸�float����
void W25QXX_Data_Fixdlobule(uint32_t location,double Data); // �޸�double����
void W25QXX_Data_Fixchar(uint32_t location,char Data); // �޸�char����
#endif

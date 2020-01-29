#include "w25qxx.h"
#include "spi.h"
#include "usart.h"
#include "stm32f4xx_hal_gpio.h"

#define Typeu8 0
#define Typeu16 1
#define Typeu32 2
#define Typeint 3
#define Typefloat 4
#define Typedouble 5
#define Typechar 6

#define u8StartDress 4096 
#define u16StartDress 65537 
#define u32StartDress 131074 
#define intStartDress 196611 
#define floatStartDress 262148 
#define doubleStartDress 327685 
#define charStartDress 393222

#define u8EndDress 4095 
#define u16EndDress 65536
#define u32EndDress 196610
#define intEndDress 262147 
#define floatEndDress 327684 
#define doubleEndDress 393221 
#define charEndDress 458758

uint16_t W25QXX_TYPE=W25Q128;	//Ĭ����W25Q128

/*�ô洢��һ���洢7���������� u8 u16 u32 int float double char
�洢ʱ�����ݰ�˳�����ָ���������ڹ涨���£�
0-4095Ϊ����λ�ô洢����
4096-65536�ֽ�Ϊu8�洢���� ���Դ�61440������
65537-131073�ֽ�Ϊu16���������Դ�32768������
131074-196610�ֽ�Ϊu32���������Դ�16384������
196611-262147�ֽ�Ϊint�洢�����Դ�16384������
262148-327684�ֽ�Ϊfloat�洢�����Դ�16384������
327685-393221�ֽ�Ϊdouble�洢�����Դ�8192������
393222-458758�ֽ�Ϊchar�洢�����Դ�65536������
*/
//4KbytesΪһ��Sector
//16������Ϊ1��Block
//W25Q256
//����Ϊ32M�ֽ�,����512��Block,8192��Sector 
													 
//��ʼ��SPI FLASH��IO��
void W25QXX_Init(void)
{ 
    uint8_t temp;
		W25QXX_TYPE=W25QXX_ReadID();	        //��ȡFLASH ID.
		if(W25QXX_TYPE==W25Q256)                //SPI FLASHΪW25Q256
			{
					temp=W25QXX_ReadSR(3);              //��ȡ״̬�Ĵ���3���жϵ�ַģʽ
		if((temp&0X01)==0)			        //�������4�ֽڵ�ַģʽ,�����4�ֽڵ�ַģʽ
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);  			        //ѡ��
				SPI1_ReadWriteByte(W25X_Enable4ByteAddr);//���ͽ���4�ֽڵ�ַģʽָ��   
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);       		        //ȡ��Ƭѡ   
			}
    }
}  

//��ȡW25QXX��״̬�Ĵ�����W25QXXһ����3��״̬�Ĵ���
//״̬�Ĵ���1��
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1,æ;0,����)
//Ĭ��:0x00
//״̬�Ĵ���2��
//BIT7  6   5   4   3   2   1   0
//SUS   CMP LB3 LB2 LB1 (R) QE  SRP1
//״̬�Ĵ���3��
//BIT7      6    5    4   3   2   1   0
//HOLD/RST  DRV1 DRV0 (R) (R) WPS ADP ADS
//regno:״̬�Ĵ����ţ���:1~3
//����ֵ:״̬�Ĵ���ֵ
uint8_t W25QXX_ReadSR(uint8_t regno)   
{  
	uint8_t byte=0,command=0; 
    switch(regno)
    {
        case 1:
            command=W25X_ReadStatusReg1;    //��״̬�Ĵ���1ָ��
            break;
        case 2:
            command=W25X_ReadStatusReg2;    //��״̬�Ĵ���2ָ��
            break;
        case 3:
            command=W25X_ReadStatusReg3;    //��״̬�Ĵ���3ָ��
            break;
        default:
            command=W25X_ReadStatusReg1;    
            break;
    }    
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                            //ʹ������   
	SPI1_ReadWriteByte(command);            //���Ͷ�ȡ״̬�Ĵ�������    
	byte=SPI1_ReadWriteByte(0Xff);          //��ȡһ���ֽ�  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                            //ȡ��Ƭѡ     
	return byte;   
} 
//дW25QXX״̬�Ĵ���
void W25QXX_Write_SR(uint8_t regno,uint8_t sr)   
{   
    uint8_t command=0;
    switch(regno)
    {
        case 1:
            command=W25X_WriteStatusReg1;    //д״̬�Ĵ���1ָ��
            break;
        case 2:
            command=W25X_WriteStatusReg2;    //д״̬�Ĵ���2ָ��
            break;
        case 3:
            command=W25X_WriteStatusReg3;    //д״̬�Ĵ���3ָ��
            break;
        default:
            command=W25X_WriteStatusReg1;    
            break;
    }   
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                             //ʹ������   
	SPI1_ReadWriteByte(command);            //����дȡ״̬�Ĵ�������    
	SPI1_ReadWriteByte(sr);                 //д��һ���ֽ�  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                            //ȡ��Ƭѡ     	      
}   
//W25QXXдʹ��	
//��WEL��λ   
void W25QXX_Write_Enable(void)   
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                             //ʹ������   
    SPI1_ReadWriteByte(W25X_WriteEnable);   //����дʹ��  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                            //ȡ��Ƭѡ     	      
} 
//W25QXXд��ֹ	
//��WEL����  
void W25QXX_Write_Disable(void)   
{  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                             //ʹ������   
    SPI1_ReadWriteByte(W25X_WriteDisable);  //����д��ָֹ��    
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                            //ȡ��Ƭѡ     	      
} 

//��ȡоƬID
//����ֵ����:				   
//0XEF13,��ʾоƬ�ͺ�ΪW25Q80  
//0XEF14,��ʾоƬ�ͺ�ΪW25Q16    
//0XEF15,��ʾоƬ�ͺ�ΪW25Q32  
//0XEF16,��ʾоƬ�ͺ�ΪW25Q64 
//0XEF17,��ʾоƬ�ͺ�ΪW25Q128 	  
//0XEF18,��ʾоƬ�ͺ�ΪW25Q256
uint16_t W25QXX_ReadID(void)
{
	uint16_t Temp = 0;	  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); 				    
	SPI1_ReadWriteByte(0x90);//���Ͷ�ȡID����	    
	SPI1_ReadWriteByte(0x00); 	    
	SPI1_ReadWriteByte(0x00); 	    
	SPI1_ReadWriteByte(0x00); 	 			   
	Temp|=SPI1_ReadWriteByte(0xFF)<<8;  
	Temp|=SPI1_ReadWriteByte(0xFF);	 
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);;				    
	return Temp;
}   		    
//��ȡSPI FLASH  
//��ָ����ַ��ʼ��ȡָ�����ȵ�����
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)   
{ 
 	uint16_t i;   										    
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                             //ʹ������   
    SPI1_ReadWriteByte(W25X_ReadData);      //���Ͷ�ȡ����  
    if(W25QXX_TYPE==W25Q256)                //�����W25Q256�Ļ���ַΪ4�ֽڵģ�Ҫ�������8λ
    {
        SPI1_ReadWriteByte((uint8_t)((ReadAddr)>>24));    
    }
    SPI1_ReadWriteByte((uint8_t)((ReadAddr)>>16));   //����24bit��ַ    
    SPI1_ReadWriteByte((uint8_t)((ReadAddr)>>8));   
    SPI1_ReadWriteByte((uint8_t)ReadAddr);   
    for(i=0;i<NumByteToRead;i++)
	{ 
        pBuffer[i]=SPI1_ReadWriteByte(0XFF);    //ѭ������  
    }
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  				    	      
}  
//SPI��һҳ(0~65535)��д������256���ֽڵ�����
//��ָ����ַ��ʼд�����256�ֽڵ�����
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!	 
void W25QXX_Write_Page(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{
 	uint16_t i;  
    W25QXX_Write_Enable();                  //SET WEL 
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                            //ʹ������   
    SPI1_ReadWriteByte(W25X_PageProgram);   //����дҳ����   
    if(W25QXX_TYPE==W25Q256)                //�����W25Q256�Ļ���ַΪ4�ֽڵģ�Ҫ�������8λ
    {
        SPI1_ReadWriteByte((uint8_t)((WriteAddr)>>24)); 
    }
    SPI1_ReadWriteByte((uint8_t)((WriteAddr)>>16)); //����24bit��ַ    
    SPI1_ReadWriteByte((uint8_t)((WriteAddr)>>8));   
    SPI1_ReadWriteByte((uint8_t)WriteAddr);   
    for(i=0;i<NumByteToWrite;i++)SPI1_ReadWriteByte(pBuffer[i]);//ѭ��д��  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                            //ȡ��Ƭѡ 
	W25QXX_Wait_Busy();					   //�ȴ�д�����
} 
//�޼���дSPI FLASH 
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//�����Զ���ҳ���� 
//��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//CHECK OK
void W25QXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)  
{ 			 		 
	uint16_t pageremain;	   
	pageremain=256-WriteAddr%256; //��ҳʣ����ֽ���		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//������256���ֽ�
	while(1)
	{	   
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//д�������
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //��ȥ�Ѿ�д���˵��ֽ���
			if(NumByteToWrite>256)pageremain=256; //һ�ο���д��256���ֽ�
			else pageremain=NumByteToWrite; 	  //����256���ֽ���
		}
	};	    
} 
//дSPI FLASH  
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)						
//NumByteToWrite:Ҫд����ֽ���(���65535)   
uint8_t W25QXX_BUFFER[4096];		 
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)   
{ 
	uint32_t secpos;
	uint16_t secoff;
	uint16_t secremain;	   
 	uint16_t i;    
	uint8_t * W25QXX_BUF;	  
  W25QXX_BUF=W25QXX_BUFFER;	     
 	secpos=WriteAddr/4096;//������ַ  
	secoff=WriteAddr%4096;//�������ڵ�ƫ��
	secremain=4096-secoff;//����ʣ��ռ��С   
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//������
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//������4096���ֽ�
	while(1) 
		{	
			W25QXX_Read(W25QXX_BUF,secpos*4096,4096);//������������������
			for(i=0;i<secremain;i++)//У������
				{
					if(W25QXX_BUF[secoff+i]!=0XFF)
						break;//��Ҫ����  	  
				}
			if(i<secremain)//��Ҫ����
				{
					W25QXX_Erase_Sector(secpos);//�����������
					for(i=0;i<secremain;i++)	   //����
							W25QXX_BUF[i+secoff]=pBuffer[i];	  
					W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);//д����������  
				}
			else W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
			if(NumByteToWrite==secremain)
				break;//д�������
			else//д��δ����
				{
					secpos++;//������ַ��1
					secoff=0;//ƫ��λ��Ϊ0 	 
					pBuffer+=secremain;  //ָ��ƫ��
					WriteAddr+=secremain;//д��ַƫ��	   
					NumByteToWrite-=secremain;				//�ֽ����ݼ�
					if(NumByteToWrite>4096)secremain=4096;	//��һ����������д����
					else secremain=NumByteToWrite;			//��һ����������д����
				}	 
		};	 
}
//��������оƬ		  
//�ȴ�ʱ�䳬��...
void W25QXX_Erase_Chip(void)   
{                                   
    W25QXX_Write_Enable();                  //SET WEL 
    W25QXX_Wait_Busy();   
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                             //ʹ������   
    SPI1_ReadWriteByte(W25X_ChipErase);        //����Ƭ��������  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                            //ȡ��Ƭѡ     	      
	W25QXX_Wait_Busy();   				   //�ȴ�оƬ��������
}   
//����һ������
//Dst_Addr:������ַ ����ʵ����������
//����һ������������ʱ��:150ms
void W25QXX_Erase_Sector(uint32_t Dst_Addr)   
{  
	//����falsh�������,������   
 	//printf("fe:%x\r\n",Dst_Addr);	  
 	Dst_Addr*=4096;
    W25QXX_Write_Enable();                  //SET WEL 	 
    W25QXX_Wait_Busy();   
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                             //ʹ������   
    SPI1_ReadWriteByte(W25X_SectorErase);   //������������ָ�� 
    if(W25QXX_TYPE==W25Q256)                //�����W25Q256�Ļ���ַΪ4�ֽڵģ�Ҫ�������8λ
    {
        SPI1_ReadWriteByte((uint8_t)((Dst_Addr)>>24)); 
    }
    SPI1_ReadWriteByte((uint8_t)((Dst_Addr)>>16));  //����24bit��ַ    
    SPI1_ReadWriteByte((uint8_t)((Dst_Addr)>>8));   
    SPI1_ReadWriteByte((uint8_t)Dst_Addr);  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                            //ȡ��Ƭѡ     	      
    W25QXX_Wait_Busy();   				    //�ȴ��������
}  
//�ȴ�����
void W25QXX_Wait_Busy(void)   
{   
	while((W25QXX_ReadSR(1)&0x01)==0x01);   // �ȴ�BUSYλ���
}  
//�������ģʽ
void W25QXX_PowerDown(void)   
{ 
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                             //ʹ������   
    SPI1_ReadWriteByte(W25X_PowerDown);     //���͵�������  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                            //ȡ��Ƭѡ     	      
    HAL_Delay(1);                           //�ȴ�TPD  
}   
//����
void W25QXX_WAKEUP(void)   
{  
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                                //ʹ������   
    SPI1_ReadWriteByte(W25X_ReleasePowerDown);  //  send W25X_PowerDown command 0xAB    
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                                 //ȡ��Ƭѡ     	      
    HAL_Delay(1);                                //�ȴ�TRES1
}   
//�����ҵ����ݴ洢��ʽ
void W25QXX_Put_u8Data(uint8_t Data)
{
	uint8_t Dict[4];
	uint32_t WriteAddr;
	uint16_t NumByteToWrite=1;
	uint8_t pBuffer[1]={0};
	W25QXX_Read(Dict,0,4);
	((uint8_t *)&WriteAddr)[0]=Dict[0];
	((uint8_t *)&WriteAddr)[1]=Dict[1];
	((uint8_t *)&WriteAddr)[2]=Dict[2];
	((uint8_t *)&WriteAddr)[3]=Dict[3];	
	if(WriteAddr<61440){
		WriteAddr+=u8StartDress;
		W25QXX_Write_Dictionary(Typeu8);
		pBuffer[0] = ((uint8_t *)&Data)[0];
		W25QXX_Write(pBuffer,WriteAddr,NumByteToWrite);	
	}
}
void W25QXX_Put_u16Data(uint16_t Data)
{
	uint8_t Dict[4];
	uint32_t WriteAddr;
	uint16_t NumByteToWrite=2;
	uint8_t pBuffer[2]={0};
	W25QXX_Read(Dict,4,4);
	((uint8_t *)&WriteAddr)[0]=Dict[0];
	((uint8_t *)&WriteAddr)[1]=Dict[1];
	((uint8_t *)&WriteAddr)[2]=Dict[2];
	((uint8_t *)&WriteAddr)[3]=Dict[3];	
	if(WriteAddr<32768){
		WriteAddr =u16StartDress+WriteAddr*2;
		W25QXX_Write_Dictionary(Typeu16);
		pBuffer[0] = ((uint8_t *)&Data)[0];
		pBuffer[1] = ((uint8_t *)&Data)[1];
		W25QXX_Write(pBuffer,WriteAddr,NumByteToWrite);	
	}
}

void W25QXX_Put_u32Data(uint32_t Data)
{
	uint8_t Dict[4];
	uint32_t WriteAddr;
	uint16_t NumByteToWrite=4;
	uint8_t pBuffer[4]={0};
	W25QXX_Read(Dict,8,4);
	((uint8_t *)&WriteAddr)[0]=Dict[0];
	((uint8_t *)&WriteAddr)[1]=Dict[1];
	((uint8_t *)&WriteAddr)[2]=Dict[2];
	((uint8_t *)&WriteAddr)[3]=Dict[3];	
	if(WriteAddr<16384)
		{
			WriteAddr =u32StartDress+WriteAddr*4;
			W25QXX_Write_Dictionary(Typeu32);
			pBuffer[0] = ((uint8_t *)&Data)[0];
			pBuffer[1] = ((uint8_t *)&Data)[1];
			pBuffer[2] = ((uint8_t *)&Data)[2];
			pBuffer[3] = ((uint8_t *)&Data)[3];
			W25QXX_Write(pBuffer,WriteAddr,NumByteToWrite);	
		}
}

void W25QXX_Put_intData(int Data)
{
	uint8_t Dict[4];
	uint32_t WriteAddr;
	uint16_t NumByteToWrite=4;
	uint8_t pBuffer[4]={0};
	W25QXX_Read(Dict,12,4);
	((uint8_t *)&WriteAddr)[0]=Dict[0];
	((uint8_t *)&WriteAddr)[1]=Dict[1];
	((uint8_t *)&WriteAddr)[2]=Dict[2];
	((uint8_t *)&WriteAddr)[3]=Dict[3];	
	if(WriteAddr<16384)
		{
			WriteAddr =intStartDress+WriteAddr*4;
			W25QXX_Write_Dictionary(Typeint);
			pBuffer[0] = ((uint8_t *)&Data)[0];
			pBuffer[1] = ((uint8_t *)&Data)[1];
			pBuffer[2] = ((uint8_t *)&Data)[2];
			pBuffer[3] = ((uint8_t *)&Data)[3];
			W25QXX_Write(pBuffer,WriteAddr,NumByteToWrite);	
		}
}

void W25QXX_Put_floatData(float Data)
{
	uint8_t Dict[4];
	uint32_t WriteAddr;
	uint16_t NumByteToWrite=4;
	uint8_t pBuffer[4]={0};
	W25QXX_Read(Dict,16,4);
	((uint8_t *)&WriteAddr)[0]=Dict[0];
	((uint8_t *)&WriteAddr)[1]=Dict[1];
	((uint8_t *)&WriteAddr)[2]=Dict[2];
	((uint8_t *)&WriteAddr)[3]=Dict[3];	
	if(WriteAddr<16384)
		{
			WriteAddr =floatStartDress+WriteAddr*4;
			W25QXX_Write_Dictionary(Typefloat);
			pBuffer[0] = ((uint8_t *)&Data)[0];
			pBuffer[1] = ((uint8_t *)&Data)[1];
			pBuffer[2] = ((uint8_t *)&Data)[2];
			pBuffer[3] = ((uint8_t *)&Data)[3];
			W25QXX_Write(pBuffer,WriteAddr,NumByteToWrite);	
		}
}

void W25QXX_Put_doubleData(double Data)
{
	uint8_t Dict[4];
	uint32_t WriteAddr=0;
	uint16_t NumByteToWrite=8;
	uint8_t pBuffer[8]={0};
	W25QXX_Read(Dict,20,4);
	((uint8_t *)&WriteAddr)[0]=Dict[0];
	((uint8_t *)&WriteAddr)[1]=Dict[1];
	((uint8_t *)&WriteAddr)[2]=Dict[2];
	((uint8_t *)&WriteAddr)[3]=Dict[3];	
	if(WriteAddr<8192)
		{
			WriteAddr =doubleStartDress+WriteAddr*8;
			W25QXX_Write_Dictionary(Typedouble);
			pBuffer[0] = ((uint8_t *)&Data)[0];
			pBuffer[1] = ((uint8_t *)&Data)[1];
			pBuffer[2] = ((uint8_t *)&Data)[2];
			pBuffer[3] = ((uint8_t *)&Data)[3];
			pBuffer[4] = ((uint8_t *)&Data)[4];
			pBuffer[5] = ((uint8_t *)&Data)[5];
			pBuffer[6] = ((uint8_t *)&Data)[6];
			pBuffer[7] = ((uint8_t *)&Data)[7];
			W25QXX_Write(pBuffer,WriteAddr,NumByteToWrite);	
		}
}

void W25QXX_Put_charData(char Data)
{
	uint8_t Dict[4];
	uint32_t WriteAddr=0;
	uint16_t NumByteToWrite=1;
	uint8_t pBuffer[1]={0};
	W25QXX_Read(Dict,24,4);
	((uint8_t *)&WriteAddr)[0]=Dict[0];
	((uint8_t *)&WriteAddr)[1]=Dict[1];
	((uint8_t *)&WriteAddr)[2]=Dict[2];
	((uint8_t *)&WriteAddr)[3]=Dict[3];	
	if(WriteAddr<32768)
		{
			WriteAddr+=charStartDress;
			W25QXX_Write_Dictionary(Typechar);
			pBuffer[0] = ((uint8_t *)&Data)[0];
			W25QXX_Write(pBuffer,WriteAddr,NumByteToWrite);	
		}
}

void W25QXX_Write_Dictionary(uint8_t type)
{
	uint8_t Dict[4];
	uint32_t count;
	switch(type)
		{
			case Typeu8:
							W25QXX_Read(Dict,0,4);
							((uint8_t *)&count)[0]=Dict[0];
							((uint8_t *)&count)[1]=Dict[1];
							((uint8_t *)&count)[2]=Dict[2];
							((uint8_t *)&count)[3]=Dict[3];							
							count++;
							Dict[0] = ((uint8_t *)&count)[0];
							Dict[1] = ((uint8_t *)&count)[1];
							Dict[2] = ((uint8_t *)&count)[2];
							Dict[3] = ((uint8_t *)&count)[3];
							W25QXX_Write(Dict,0,4);	
							break;
			case Typeu16:
							W25QXX_Read(Dict,4,4);
							((uint8_t *)&count)[0]=Dict[0];
							((uint8_t *)&count)[1]=Dict[1];
							((uint8_t *)&count)[2]=Dict[2];
							((uint8_t *)&count)[3]=Dict[3];							
							count++;
							Dict[0] = ((uint8_t *)&count)[0];
							Dict[1] = ((uint8_t *)&count)[1];
							Dict[2] = ((uint8_t *)&count)[2];
							Dict[3] = ((uint8_t *)&count)[3];
							W25QXX_Write(Dict,4,4);	
							break;
			case Typeu32:
							W25QXX_Read(Dict,8,4);
							((uint8_t *)&count)[0]=Dict[0];
							((uint8_t *)&count)[1]=Dict[1];
							((uint8_t *)&count)[2]=Dict[2];
							((uint8_t *)&count)[3]=Dict[3];							
							count++;
							Dict[0] = ((uint8_t *)&count)[0];
							Dict[1] = ((uint8_t *)&count)[1];
							Dict[2] = ((uint8_t *)&count)[2];
							Dict[3] = ((uint8_t *)&count)[3];
							W25QXX_Write(Dict,8,4);	
							break;
			case Typeint:
							W25QXX_Read(Dict,12,4);
							((uint8_t *)&count)[0]=Dict[0];
							((uint8_t *)&count)[1]=Dict[1];
							((uint8_t *)&count)[2]=Dict[2];
							((uint8_t *)&count)[3]=Dict[3];							
							count++;
							Dict[0] = ((uint8_t *)&count)[0];
							Dict[1] = ((uint8_t *)&count)[1];
							Dict[2] = ((uint8_t *)&count)[2];
							Dict[3] = ((uint8_t *)&count)[3];
							W25QXX_Write(Dict,12,4);	
							break;
			case Typefloat:
							W25QXX_Read(Dict,16,4);
							((uint8_t *)&count)[0]=Dict[0];
							((uint8_t *)&count)[1]=Dict[1];
							((uint8_t *)&count)[2]=Dict[2];
							((uint8_t *)&count)[3]=Dict[3];							
							count++;
							Dict[0] = ((uint8_t *)&count)[0];
							Dict[1] = ((uint8_t *)&count)[1];
							Dict[2] = ((uint8_t *)&count)[2];
							Dict[3] = ((uint8_t *)&count)[3];
							W25QXX_Write(Dict,16,4);	
							break;
			case Typedouble:
							W25QXX_Read(Dict,20,4);
							((uint8_t *)&count)[0]=Dict[0];
							((uint8_t *)&count)[1]=Dict[1];
							((uint8_t *)&count)[2]=Dict[2];
							((uint8_t *)&count)[3]=Dict[3];							
							count++;
							Dict[0] = ((uint8_t *)&count)[0];
							Dict[1] = ((uint8_t *)&count)[1];
							Dict[2] = ((uint8_t *)&count)[2];
							Dict[3] = ((uint8_t *)&count)[3];
							W25QXX_Write(Dict,20,4);	
							break;
			case Typechar:
							W25QXX_Read(Dict,24,4);
							((uint8_t *)&count)[0]=Dict[0];
							((uint8_t *)&count)[1]=Dict[1];
							((uint8_t *)&count)[2]=Dict[2];
							((uint8_t *)&count)[3]=Dict[3];							
							count++;
							Dict[0] = ((uint8_t *)&count)[0];
							Dict[1] = ((uint8_t *)&count)[1];
							Dict[2] = ((uint8_t *)&count)[2];
							Dict[3] = ((uint8_t *)&count)[3];
							W25QXX_Write(Dict,24,4);	
							break;
		}
}

void W25QXX_AllData_Initial(void)
{
	uint8_t Dict[4];
	uint32_t InitialDress=0;
	Dict[0] = ((uint8_t *)&InitialDress)[0];
	Dict[1] = ((uint8_t *)&InitialDress)[1];
	Dict[2] = ((uint8_t *)&InitialDress)[2];
	Dict[3] = ((uint8_t *)&InitialDress)[3];
	W25QXX_Write(Dict,0,4);	
	W25QXX_Write(Dict,4,4);	
	W25QXX_Write(Dict,8,4);	
	W25QXX_Write(Dict,12,4);
	W25QXX_Write(Dict,16,4);
	W25QXX_Write(Dict,20,4);
	W25QXX_Write(Dict,24,4);	
}

uint8_t W25QXX_Data_u8Read(uint32_t location)
{
	uint8_t Dict[4],num;
	uint32_t tem;
	W25QXX_Read(Dict,0,4);
	((uint8_t *)&tem)[0]=Dict[0];
	((uint8_t *)&tem)[1]=Dict[1];
	((uint8_t *)&tem)[2]=Dict[2];
	((uint8_t *)&tem)[3]=Dict[3];		
	if(location<tem)
		{
			W25QXX_Read(Dict,u8StartDress+location,1);
			((uint8_t *)&num)[0]=Dict[0];
		}
		return num;
}

uint16_t W25QXX_Data_u16Read(uint32_t location)
{
	uint8_t Dict[4];
	uint16_t num;
	uint32_t tem;
	W25QXX_Read(Dict,4,4);
	((uint8_t *)&tem)[0]=Dict[0];
	((uint8_t *)&tem)[1]=Dict[1];
	((uint8_t *)&tem)[2]=Dict[2];
	((uint8_t *)&tem)[3]=Dict[3];		
	if(location<tem)
		{
			W25QXX_Read(Dict,u16StartDress+location*2,2);
			((uint8_t *)&num)[0]=Dict[0];
			((uint8_t *)&num)[1]=Dict[1];
		}
		return num;
}

uint32_t W25QXX_Data_u32Read(uint32_t location)
{
	uint8_t Dict[4];
	uint32_t num;
	uint32_t tem;
	W25QXX_Read(Dict,8,4);
	((uint8_t *)&tem)[0]=Dict[0];
	((uint8_t *)&tem)[1]=Dict[1];
	((uint8_t *)&tem)[2]=Dict[2];
	((uint8_t *)&tem)[3]=Dict[3];		
	if(location<tem)
		{
			W25QXX_Read(Dict,u32StartDress+location*4,4);
			((uint8_t *)&num)[0]=Dict[0];
			((uint8_t *)&num)[1]=Dict[1];
			((uint8_t *)&num)[2]=Dict[2];
			((uint8_t *)&num)[3]=Dict[3];
		}
		return num;
}

int W25QXX_Data_intRead(uint32_t location)
{
	uint8_t Dict[4];
	int num;
	uint32_t tem;
	W25QXX_Read(Dict,12,4);
	((uint8_t *)&tem)[0]=Dict[0];
	((uint8_t *)&tem)[1]=Dict[1];
	((uint8_t *)&tem)[2]=Dict[2];
	((uint8_t *)&tem)[3]=Dict[3];		
	if(location<tem)
		{
			W25QXX_Read(Dict,intStartDress+location*4,4);
			((uint8_t *)&num)[0]=Dict[0];
			((uint8_t *)&num)[1]=Dict[1];
			((uint8_t *)&num)[2]=Dict[2];
			((uint8_t *)&num)[3]=Dict[3];
		}
		return num;
}

float W25QXX_Data_floatRead(uint32_t location)
{
	uint8_t Dict[4];
	float num;
	uint32_t tem;
	W25QXX_Read(Dict,16,4);
	((uint8_t *)&tem)[0]=Dict[0];
	((uint8_t *)&tem)[1]=Dict[1];
	((uint8_t *)&tem)[2]=Dict[2];
	((uint8_t *)&tem)[3]=Dict[3];		
	if(location<tem)
		{
			W25QXX_Read(Dict,floatStartDress+location*4,4);
			((uint8_t *)&num)[0]=Dict[0];
			((uint8_t *)&num)[1]=Dict[1];
			((uint8_t *)&num)[2]=Dict[2];
			((uint8_t *)&num)[3]=Dict[3];
		}
		return num;
}

double W25QXX_Data_doubleRead(uint32_t location)
{
	uint8_t Dict[8];
	double num;
	uint32_t tem;
	W25QXX_Read(Dict,20,4);
	((uint8_t *)&tem)[0]=Dict[0];
	((uint8_t *)&tem)[1]=Dict[1];
	((uint8_t *)&tem)[2]=Dict[2];
	((uint8_t *)&tem)[3]=Dict[3];		
	if(location<tem)
		{
			W25QXX_Read(Dict,doubleStartDress+location*8,8);
			((uint8_t *)&num)[0]=Dict[0];
			((uint8_t *)&num)[1]=Dict[1];
			((uint8_t *)&num)[2]=Dict[2];
			((uint8_t *)&num)[3]=Dict[3];
			((uint8_t *)&num)[4]=Dict[4];
			((uint8_t *)&num)[5]=Dict[5];
			((uint8_t *)&num)[6]=Dict[6];
			((uint8_t *)&num)[7]=Dict[7];
		}
		return num;
}

char W25QXX_Data_charRead(uint32_t location)
{
	uint8_t Dict[4];
	char num;
	uint32_t tem;
	W25QXX_Read(Dict,24,4);
	((uint8_t *)&tem)[0]=Dict[0];
	((uint8_t *)&tem)[1]=Dict[1];
	((uint8_t *)&tem)[2]=Dict[2];
	((uint8_t *)&tem)[3]=Dict[3];		
	if(location<tem)
		{
			W25QXX_Read(Dict,charStartDress+location,1);
			((uint8_t *)&num)[0]=Dict[0];
		}
		return num;
}

void W25QXX_Data_Fixu8(uint32_t location,uint8_t Data)
{
	uint8_t temData[1];
	if((location+u8StartDress)<u8EndDress)
		{
			temData[0]=((uint8_t *)&Data)[0];
			W25QXX_Write(temData,u8StartDress+location,1);
		}
}

void W25QXX_Data_Fixu16(uint32_t location,uint16_t Data)
{
	uint8_t temData[2];	
	if((location*2+u16StartDress)<u16EndDress)
		{
			temData[0]=((uint8_t *)&Data)[0];
			temData[1]=((uint8_t *)&Data)[1];
			W25QXX_Write(temData,u16StartDress+location,2);
		}
}

void W25QXX_Data_Fixu32(uint32_t location,uint32_t Data)
{
	uint8_t temData[4];
	if((location*4+u32StartDress)<u32EndDress)
		{
			temData[0]=((uint8_t *)&Data)[0];
			temData[1]=((uint8_t *)&Data)[1];
			temData[2]=((uint8_t *)&Data)[2];
			temData[3]=((uint8_t *)&Data)[3];
			W25QXX_Write(temData,u32StartDress+location,4);
		}
}

void W25QXX_Data_Fixint(uint32_t location,int Data)
{
	uint8_t temData[4];
	if((location*4+intStartDress)<intEndDress)
		{
			temData[0]=((uint8_t *)&Data)[0];
			temData[1]=((uint8_t *)&Data)[1];
			temData[2]=((uint8_t *)&Data)[2];
			temData[3]=((uint8_t *)&Data)[3];
			W25QXX_Write(temData,u32StartDress+location,4);
		}
}

void W25QXX_Data_Fixfloat(uint32_t location,float Data)
{
	uint8_t temData[4];
	if((location*4+floatStartDress)<floatEndDress)
		{
			temData[0]=((uint8_t *)&Data)[0];
			temData[1]=((uint8_t *)&Data)[1];
			temData[2]=((uint8_t *)&Data)[2];
			temData[3]=((uint8_t *)&Data)[3];
			W25QXX_Write(temData,floatStartDress+location,4);
		}
}

void W25QXX_Data_Fixdlobule(uint32_t location,double Data)
{
	uint8_t temData[8];
	if((location*4+doubleStartDress)<doubleEndDress)
		{
			temData[0]=((uint8_t *)&Data)[0];
			temData[1]=((uint8_t *)&Data)[1];
			temData[2]=((uint8_t *)&Data)[2];
			temData[3]=((uint8_t *)&Data)[3];
			temData[4]=((uint8_t *)&Data)[4];
			temData[5]=((uint8_t *)&Data)[5];
			temData[6]=((uint8_t *)&Data)[6];
			temData[7]=((uint8_t *)&Data)[7];
			W25QXX_Write(temData,doubleStartDress+location,8);
		}
}

void W25QXX_Data_Fixchar(uint32_t location,char Data)
{
	uint8_t temData[1];
	if((location*4+charStartDress)<charEndDress)
		{
			temData[0]=((uint8_t *)&Data)[0];
			W25QXX_Write(temData,charStartDress+location,1);
		}
}

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

uint16_t W25QXX_TYPE=W25Q128;	//默认是W25Q128

/*该存储器一个存储7种数据类型 u8 u16 u32 int float double char
存储时把数据按顺序存在指定的区域内规定如下：
0-4095为数据位置存储区域
4096-65536字节为u8存储区域 可以存61440个变量
65537-131073字节为u16储存区可以存32768个变量
131074-196610字节为u32储存区可以存16384个变量
196611-262147字节为int存储区可以存16384个变量
262148-327684字节为float存储区可以存16384个变量
327685-393221字节为double存储区可以存8192个变量
393222-458758字节为char存储区可以存65536个变量
*/
//4Kbytes为一个Sector
//16个扇区为1个Block
//W25Q256
//容量为32M字节,共有512个Block,8192个Sector 
													 
//初始化SPI FLASH的IO口
void W25QXX_Init(void)
{ 
    uint8_t temp;
		W25QXX_TYPE=W25QXX_ReadID();	        //读取FLASH ID.
		if(W25QXX_TYPE==W25Q256)                //SPI FLASH为W25Q256
			{
					temp=W25QXX_ReadSR(3);              //读取状态寄存器3，判断地址模式
		if((temp&0X01)==0)			        //如果不是4字节地址模式,则进入4字节地址模式
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);  			        //选中
				SPI1_ReadWriteByte(W25X_Enable4ByteAddr);//发送进入4字节地址模式指令   
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);       		        //取消片选   
			}
    }
}  

//读取W25QXX的状态寄存器，W25QXX一共有3个状态寄存器
//状态寄存器1：
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:默认0,状态寄存器保护位,配合WP使用
//TB,BP2,BP1,BP0:FLASH区域写保护设置
//WEL:写使能锁定
//BUSY:忙标记位(1,忙;0,空闲)
//默认:0x00
//状态寄存器2：
//BIT7  6   5   4   3   2   1   0
//SUS   CMP LB3 LB2 LB1 (R) QE  SRP1
//状态寄存器3：
//BIT7      6    5    4   3   2   1   0
//HOLD/RST  DRV1 DRV0 (R) (R) WPS ADP ADS
//regno:状态寄存器号，范:1~3
//返回值:状态寄存器值
uint8_t W25QXX_ReadSR(uint8_t regno)   
{  
	uint8_t byte=0,command=0; 
    switch(regno)
    {
        case 1:
            command=W25X_ReadStatusReg1;    //读状态寄存器1指令
            break;
        case 2:
            command=W25X_ReadStatusReg2;    //读状态寄存器2指令
            break;
        case 3:
            command=W25X_ReadStatusReg3;    //读状态寄存器3指令
            break;
        default:
            command=W25X_ReadStatusReg1;    
            break;
    }    
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                            //使能器件   
	SPI1_ReadWriteByte(command);            //发送读取状态寄存器命令    
	byte=SPI1_ReadWriteByte(0Xff);          //读取一个字节  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                            //取消片选     
	return byte;   
} 
//写W25QXX状态寄存器
void W25QXX_Write_SR(uint8_t regno,uint8_t sr)   
{   
    uint8_t command=0;
    switch(regno)
    {
        case 1:
            command=W25X_WriteStatusReg1;    //写状态寄存器1指令
            break;
        case 2:
            command=W25X_WriteStatusReg2;    //写状态寄存器2指令
            break;
        case 3:
            command=W25X_WriteStatusReg3;    //写状态寄存器3指令
            break;
        default:
            command=W25X_WriteStatusReg1;    
            break;
    }   
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                             //使能器件   
	SPI1_ReadWriteByte(command);            //发送写取状态寄存器命令    
	SPI1_ReadWriteByte(sr);                 //写入一个字节  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                            //取消片选     	      
}   
//W25QXX写使能	
//将WEL置位   
void W25QXX_Write_Enable(void)   
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                             //使能器件   
    SPI1_ReadWriteByte(W25X_WriteEnable);   //发送写使能  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                            //取消片选     	      
} 
//W25QXX写禁止	
//将WEL清零  
void W25QXX_Write_Disable(void)   
{  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                             //使能器件   
    SPI1_ReadWriteByte(W25X_WriteDisable);  //发送写禁止指令    
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                            //取消片选     	      
} 

//读取芯片ID
//返回值如下:				   
//0XEF13,表示芯片型号为W25Q80  
//0XEF14,表示芯片型号为W25Q16    
//0XEF15,表示芯片型号为W25Q32  
//0XEF16,表示芯片型号为W25Q64 
//0XEF17,表示芯片型号为W25Q128 	  
//0XEF18,表示芯片型号为W25Q256
uint16_t W25QXX_ReadID(void)
{
	uint16_t Temp = 0;	  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); 				    
	SPI1_ReadWriteByte(0x90);//发送读取ID命令	    
	SPI1_ReadWriteByte(0x00); 	    
	SPI1_ReadWriteByte(0x00); 	    
	SPI1_ReadWriteByte(0x00); 	 			   
	Temp|=SPI1_ReadWriteByte(0xFF)<<8;  
	Temp|=SPI1_ReadWriteByte(0xFF);	 
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);;				    
	return Temp;
}   		    
//读取SPI FLASH  
//在指定地址开始读取指定长度的数据
//pBuffer:数据存储区
//ReadAddr:开始读取的地址(24bit)
//NumByteToRead:要读取的字节数(最大65535)
void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)   
{ 
 	uint16_t i;   										    
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                             //使能器件   
    SPI1_ReadWriteByte(W25X_ReadData);      //发送读取命令  
    if(W25QXX_TYPE==W25Q256)                //如果是W25Q256的话地址为4字节的，要发送最高8位
    {
        SPI1_ReadWriteByte((uint8_t)((ReadAddr)>>24));    
    }
    SPI1_ReadWriteByte((uint8_t)((ReadAddr)>>16));   //发送24bit地址    
    SPI1_ReadWriteByte((uint8_t)((ReadAddr)>>8));   
    SPI1_ReadWriteByte((uint8_t)ReadAddr);   
    for(i=0;i<NumByteToRead;i++)
	{ 
        pBuffer[i]=SPI1_ReadWriteByte(0XFF);    //循环读数  
    }
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  				    	      
}  
//SPI在一页(0~65535)内写入少于256个字节的数据
//在指定地址开始写入最大256字节的数据
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!	 
void W25QXX_Write_Page(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{
 	uint16_t i;  
    W25QXX_Write_Enable();                  //SET WEL 
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                            //使能器件   
    SPI1_ReadWriteByte(W25X_PageProgram);   //发送写页命令   
    if(W25QXX_TYPE==W25Q256)                //如果是W25Q256的话地址为4字节的，要发送最高8位
    {
        SPI1_ReadWriteByte((uint8_t)((WriteAddr)>>24)); 
    }
    SPI1_ReadWriteByte((uint8_t)((WriteAddr)>>16)); //发送24bit地址    
    SPI1_ReadWriteByte((uint8_t)((WriteAddr)>>8));   
    SPI1_ReadWriteByte((uint8_t)WriteAddr);   
    for(i=0;i<NumByteToWrite;i++)SPI1_ReadWriteByte(pBuffer[i]);//循环写数  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                            //取消片选 
	W25QXX_Wait_Busy();					   //等待写入结束
} 
//无检验写SPI FLASH 
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//具有自动换页功能 
//在指定地址开始写入指定长度的数据,但是要确保地址不越界!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
//CHECK OK
void W25QXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)  
{ 			 		 
	uint16_t pageremain;	   
	pageremain=256-WriteAddr%256; //单页剩余的字节数		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//不大于256个字节
	while(1)
	{	   
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//写入结束了
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //减去已经写入了的字节数
			if(NumByteToWrite>256)pageremain=256; //一次可以写入256个字节
			else pageremain=NumByteToWrite; 	  //不够256个字节了
		}
	};	    
} 
//写SPI FLASH  
//在指定地址开始写入指定长度的数据
//该函数带擦除操作!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)						
//NumByteToWrite:要写入的字节数(最大65535)   
uint8_t W25QXX_BUFFER[4096];		 
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)   
{ 
	uint32_t secpos;
	uint16_t secoff;
	uint16_t secremain;	   
 	uint16_t i;    
	uint8_t * W25QXX_BUF;	  
  W25QXX_BUF=W25QXX_BUFFER;	     
 	secpos=WriteAddr/4096;//扇区地址  
	secoff=WriteAddr%4096;//在扇区内的偏移
	secremain=4096-secoff;//扇区剩余空间大小   
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//测试用
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//不大于4096个字节
	while(1) 
		{	
			W25QXX_Read(W25QXX_BUF,secpos*4096,4096);//读出整个扇区的内容
			for(i=0;i<secremain;i++)//校验数据
				{
					if(W25QXX_BUF[secoff+i]!=0XFF)
						break;//需要擦除  	  
				}
			if(i<secremain)//需要擦除
				{
					W25QXX_Erase_Sector(secpos);//擦除这个扇区
					for(i=0;i<secremain;i++)	   //复制
							W25QXX_BUF[i+secoff]=pBuffer[i];	  
					W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);//写入整个扇区  
				}
			else W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
			if(NumByteToWrite==secremain)
				break;//写入结束了
			else//写入未结束
				{
					secpos++;//扇区地址增1
					secoff=0;//偏移位置为0 	 
					pBuffer+=secremain;  //指针偏移
					WriteAddr+=secremain;//写地址偏移	   
					NumByteToWrite-=secremain;				//字节数递减
					if(NumByteToWrite>4096)secremain=4096;	//下一个扇区还是写不完
					else secremain=NumByteToWrite;			//下一个扇区可以写完了
				}	 
		};	 
}
//擦除整个芯片		  
//等待时间超长...
void W25QXX_Erase_Chip(void)   
{                                   
    W25QXX_Write_Enable();                  //SET WEL 
    W25QXX_Wait_Busy();   
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                             //使能器件   
    SPI1_ReadWriteByte(W25X_ChipErase);        //发送片擦除命令  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                            //取消片选     	      
	W25QXX_Wait_Busy();   				   //等待芯片擦除结束
}   
//擦除一个扇区
//Dst_Addr:扇区地址 根据实际容量设置
//擦除一个扇区的最少时间:150ms
void W25QXX_Erase_Sector(uint32_t Dst_Addr)   
{  
	//监视falsh擦除情况,测试用   
 	//printf("fe:%x\r\n",Dst_Addr);	  
 	Dst_Addr*=4096;
    W25QXX_Write_Enable();                  //SET WEL 	 
    W25QXX_Wait_Busy();   
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                             //使能器件   
    SPI1_ReadWriteByte(W25X_SectorErase);   //发送扇区擦除指令 
    if(W25QXX_TYPE==W25Q256)                //如果是W25Q256的话地址为4字节的，要发送最高8位
    {
        SPI1_ReadWriteByte((uint8_t)((Dst_Addr)>>24)); 
    }
    SPI1_ReadWriteByte((uint8_t)((Dst_Addr)>>16));  //发送24bit地址    
    SPI1_ReadWriteByte((uint8_t)((Dst_Addr)>>8));   
    SPI1_ReadWriteByte((uint8_t)Dst_Addr);  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                            //取消片选     	      
    W25QXX_Wait_Busy();   				    //等待擦除完成
}  
//等待空闲
void W25QXX_Wait_Busy(void)   
{   
	while((W25QXX_ReadSR(1)&0x01)==0x01);   // 等待BUSY位清空
}  
//进入掉电模式
void W25QXX_PowerDown(void)   
{ 
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                             //使能器件   
    SPI1_ReadWriteByte(W25X_PowerDown);     //发送掉电命令  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                            //取消片选     	      
    HAL_Delay(1);                           //等待TPD  
}   
//唤醒
void W25QXX_WAKEUP(void)   
{  
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);                                //使能器件   
    SPI1_ReadWriteByte(W25X_ReleasePowerDown);  //  send W25X_PowerDown command 0xAB    
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);                                 //取消片选     	      
    HAL_Delay(1);                                //等待TRES1
}   
//定义我的数据存储格式
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

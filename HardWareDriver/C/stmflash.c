/*    
      ____                      _____                  +---+
     / ___\                     / __ \                 | R |
    / /                        / /_/ /                 +---+
   / /   ________  ____  ___  / ____/___  ____  __   __
  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
 / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
 \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
                                                 / /
                                            ____/ /
                                           /_____/
moto.c file
编写者：小马  (Camel)
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.内部flash初始化，相当于一个片内的模拟EEPROM //Internal flash initialization, the equivalent of an on-chip EEPROM emulation
2.具体操作细节还有些BUG，有些地址读出来的数据有问题，我暂时没找到原因，望各路热血青年一起来解决 //Specific operational details still some BUG, and some address data read out a problem, I do not find the cause, look together to solve the brightest young blood
------------------------------------
*/
#include "stmflash.h"
#include "delay.h"
#include "UART1.h"
#include "stdio.h"
//////////////////////////////////////////////////////////////////////////////////	 
//stm32f103t8u6--->64K Bytes  flash
//小容量stm32的最后一页开始地址为0x08007c00，结束地址为0x08007fff //Address of the last page of a small capacity stm32 0x08007c00, end address 0x08007fff
//每一页大小为1K个字节 //Each page size 1K bytes
//////////////////////////////////////////////////////////////////////////////////

//解锁STM32的FLASH //Unlock the STM32 FLASH
void STMFLASH_Unlock(void)
{
  FLASH->KEYR=FLASH_KEY1;//写入解锁序列. //Written unlock sequence
  FLASH->KEYR=FLASH_KEY2;
  printf("内部FLASH解锁完成...\r\n"); //Unlock complete internal FLASH
}
//flash上锁 //flash lock
void STMFLASH_Lock(void)
{
  FLASH->CR|=1<<7;//上锁 //Locked
}
//得到FLASH状态 //Get FLASH status
u8 STMFLASH_GetStatus(void)
{	
	u32 res;		
	res=FLASH->SR; 
	if(res&(1<<0))return 1;		    //忙 //Busy
	else if(res&(1<<2))return 2;	//编程错误 //Programming error
	else if(res&(1<<4))return 3;	//写保护错误 //Write protect error
	return 0;						//操作完成 //The operation is complete
}
//等待操作完成 //Wait until the operation is completed
//time:要延时的长短 //To the length of delay
//返回值:状态. //Return Value: Status.
u8 STMFLASH_WaitDone(u16 time)
{
	u8 res;
	do
	{
		res=STMFLASH_GetStatus();
		if(res!=1)break;//非忙,无需等待了,直接退出. //Not busy, no wait, exit.
		delay_us(1);
		time--;
	 }while(time);
	 if(time==0)res=0xff;//TIMEOUT
	 return res;
}
//擦除页 //Erase Page
//paddr:页地址 //Page Address
//返回值:执行情况 //Returns: Implementation
u8 STMFLASH_ErasePage(u32 paddr)
{
	u8 res=0;
	res=STMFLASH_WaitDone(0X5FFF);//等待上次操作结束,>20ms //Wait for the end of last operation,> 20ms  
	if(res==0)
	{ 
		FLASH->CR|=1<<1;//页擦除 //Page Erase
		FLASH->AR=paddr;//设置页地址 //Set page address
		FLASH->CR|=1<<6;//开始擦除	 //Begin erasing
		res=STMFLASH_WaitDone(0X5FFF);//等待操作结束,>20ms //Wait until the operation ends,> 20ms 
		if(res!=1)//非忙 //Non-busy
		{
			FLASH->CR&=~(1<<1);//清除页擦除标志. //Clear Page Erase flag
		}
	}
	return res;
}
//在FLASH指定地址写入半字 //Half-word write to the specified address in FLASH
//faddr:指定地址(此地址必须为2的倍数!!) //faddr: specified address (this address must be a multiple of 2 !!)
//dat:要写入的数据 //Data to be written
//返回值:写入的情况 //Returns: case writing
u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat)
{
	u8 res;	   	    
	res=STMFLASH_WaitDone(0XFF);	 
	if(res==0)//OK
	{
		FLASH->CR|=1<<0;//编程使能 //Programming Enable
		*(vu16*)faddr=dat;//写入数据 //Write data
		res=STMFLASH_WaitDone(0XFF);//等待操作完成 //Wait until the operation is completed
		if(res!=1)//操作成功 //Successful operation
		{
			FLASH->CR&=~(1<<0);//清除PG位. //Clear PG bit
		}
	} 
	return res;
} 
//读取指定地址的半字(16位数据) //Read the specified address half-word (16-bit data)
//faddr:读地址 //faddr: read address
//返回值:对应数据. //Returns: the corresponding data.
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
	return *(vu16*)faddr; 
}
#if STM32_FLASH_WREN	//如果使能了写   //If enabled write
//不检查的写入 //Do not check writing
//WriteAddr:起始地址 //WriteAddr: starting address
//pBuffer:数据指针 //pBuffer: Data Pointer
//NumToWrite:半字(16位)数  //NumToWrite: half-word (16-bit) number
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)   
{ 			 		 
	u16 i;
	for(i=0;i<NumToWrite;i++)
	{
		STMFLASH_WriteHalfWord(WriteAddr,pBuffer[i]);
	  WriteAddr+=2;//地址增加2. //Address increase of 2
	}  
} 
//从指定地址开始写入指定长度的数据 //Start writing data specified length from the specified address
//WriteAddr:起始地址(此地址必须为2的倍数!!) //WriteAddr: start address (this address must be a multiple of 2 !!)
//pBuffer:数据指针 //pBuffer: Data Pointer
//NumToWrite:半字(16位)数(就是要写入的16位数据的个数.) //NumToWrite: half-word (16-bit) number (the number is to be written in 16-bit data.)
#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 //字节 //Byte
#else 
#define STM_SECTOR_SIZE	2048
#endif

u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//最多是2K字节 //Most of 2K bytes

void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	
{
	u32 secpos;	   //扇区地址 //Sector address
	u16 secoff;	   //扇区内偏移地址(16位字计算) //Within the sector offset address (16 word calculation)
	u16 secremain; //扇区内剩余地址(16位字计算) //Remaining within the sector address (16 word calculation)	   
 	u16 i;    
	u32 offaddr;   //去掉0X08000000后的地址 //Address removed after 0X08000000
	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//非法地址
	STMFLASH_Unlock();						//解锁 //Unlock
	offaddr=WriteAddr-STM32_FLASH_BASE;		//实际偏移地址. //The actual offset address
	secpos=offaddr/STM_SECTOR_SIZE;			//扇区地址  0~127 for STM32F103RBT6 //Sector address 0 ~ 127 for STM32F103RBT6
	secoff=(offaddr%STM_SECTOR_SIZE)/2;		//在扇区内的偏移(2个字节为基本单位.) //Offset in the sector (2 bytes as a basic unit.)
	secremain=STM_SECTOR_SIZE/2-secoff;		//扇区剩余空间大小 //Remaining space sector  
	if(NumToWrite<=secremain)secremain=NumToWrite;//不大于该扇区范围 //No more than the sector range
	while(1) 
	{	
		STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//读出整个扇区的内容
		for(i=0;i<secremain;i++)//校验数据 //Check data
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//需要擦除 //To be erased	  
		}
		if(i<secremain)//需要擦除 //To be erased
		{
			STMFLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区 //Erase this sector
			for(i=0;i<secremain;i++)//复制 //Copy
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//写入整个扇区  //Write the entire sector
		}else STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//写已经擦除了的,直接写入扇区剩余区间. //Write has erased the direct write sector remaining interval 				   
		if(NumToWrite==secremain)break;//写入结束了 //Writing end of the
		else//写入未结束 //Write is not the end
		{
			secpos++;				//扇区地址增1 //The sector address by 1
			secoff=0;				//偏移位置为0 //Offset position is 0 
		   	pBuffer+=secremain;  	//指针偏移  //Pointer offset
			WriteAddr+=secremain;	//写地址偏移 //Write Address Offset   
		   	NumToWrite-=secremain;	//字节(16位)数递减 //Byte (16) desc
			if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;//下一个扇区还是写不完 //Or could not finish the next sector
			else secremain=NumToWrite;//下一个扇区可以写完了 //You can finish the next sector
		}	 
	};	
	STMFLASH_Lock();//上锁 //Locked
}
#endif

//从指定地址开始读出指定长度的数据 //Began to read out the data specified length from the specified address
//ReadAddr:起始地址 //Starting Address
//pBuffer:数据指针 //Data Pointer
//NumToWrite:半字(16位)数 //Half-word (16-bit) number
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	
{
	u16 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//读取2个字节. //Read 2 bytes
		ReadAddr+=2;//偏移2个字节.	 //Offset 2 bytes
	}
}

//////////////////////////////////////////测试用(Test)///////////////////////////////////////////
//WriteAddr:起始地址 //Starting Address
//WriteData:要写入的数据 //Data to be written
void Test_Write(u32 WriteAddr,u16 WriteData)   	
{
	STMFLASH_Write(WriteAddr,&WriteData,1);//写入一个字 //Write a word
}

















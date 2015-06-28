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
iic.c file
编写者：小马  (Camel)
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.初始化软件IIC协议 //IIC protocol initialization software
2.软件IIC协议的引脚，也是STM32硬件IIC的引脚。只是我没有这个功能 //Pin Software IIC protocol, also STM32 hardware IIC pins. But I do not have this feature
------------------------------------
*/
//STM32模拟IIC协议，STM32的硬件IIC有些BUG //IIC protocol simulation STM32, STM32 hardware IIC some BUG
//细节有空再改 //Details free and then change
//最后修改:2014-03-11


#include "IIC.h"
#include "delay.h"
#include "Led.h"
#include "UART1.h"
#include "stdio.h"
/**************************实现函数********************************************
*函数原型:		void IIC_Init(void)
*功　　能:		初始化I2C对应的接口引脚。 //Initializes I2C corresponding interface pins.
*******************************************************************************/
void IIC_Init(void)
{			
	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);			     
 	//配置PB6 PB7 为开漏输出  刷新频率为10Mhz //PB6 PB7 configured as open-drain output refresh rate of 10Mhz
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //应用配置到GPIOB //Application configuration to GPIOB 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  printf("IIC总线初始化完成...\r\n"); //IIC bus initialization is complete
}

/**************************实现函数********************************************
*函数原型:		void IIC_Start(void)
*功　　能:		产生IIC起始信号 //IIC start signal generation
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出 //sda line output
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 //Vise I2C bus, ready to send or receive data
}

/**************************Implement function********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号 //Stop signal generating IIC
*******************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出 //sda line output
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号 //Send I2C bus completion signal
	delay_us(4);							   	
}

/**************************实现函数********************************************
*函数原型:		u8 IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来 //Waiting for the arrival of the response signal
//返回值：1，接收应答失败 //Returns: 1, receiving response failure
//        0，接收应答成功 //0, receives the response successfully
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入 //SDA is set to enter
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 1;
		}
	  delay_us(1);
	}
	IIC_SCL=0;//时钟输出0 //Clock Output 0	   
	return 0;  
} 

/**************************Implement function********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答 //An ACK
*******************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}
	
/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答 //Generate NACK response
*******************************************************************************/	    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}					 				     

/**************************实现函数********************************************
*函数原型:		void IIC_Send_Byte(u8 txd)
*功　　能:	    IIC发送一个字节 //IIC send a byte
*******************************************************************************/		  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输 //Low clock starts data transmission
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(1);   
		IIC_SCL=1;
		delay_us(1); 
		IIC_SCL=0;	
		delay_us(1);
    }	 
} 	 
   
/**************************实现函数********************************************
*函数原型:		u8 IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK //Read one byte, ack = 1, send ACK, ack = 0, send nACK
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入 //SDA is set to enter
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(1);
				IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (ack)
        IIC_Ack(); //发送ACK //Send ACK
    else
        IIC_NAck();//发送nACK //Send nACK
    return receive;
}

/**************************实现函数********************************************
*函数原型:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*功　　能:	    读取指定设备 指定寄存器的一个值 //Reads a value specifying device register
输入	I2C_Addr  目标设备地址 //Target Device Address
		addr	   寄存器地址 //Register Address
返回   读出来的值 //The value read out
*******************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_Start();	
  

  
  
	IIC_Send_Byte(I2C_Addr);	   //发送写命令 //Send Write Command
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //发送地址 //Send Address
	IIC_Wait_Ack();	  
	//IIC_Stop();//产生一个停止条件	 //Generating a stop condition
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;          //进入接收模式	//Receive mode		   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
    IIC_Stop();//产生一个停止条件  //Generating a stop condition

	return res;
}


/**************************实现函数********************************************
*函数原型:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*功　　能:	    读取指定设备 指定寄存器的 length个值 //Read the specified device specified length value register
输入	dev  目标设备地址 //Target Device Address
		reg	  寄存器地址 //register address
		length 要读的字节数 //The number of bytes to be read
		*data  读出的数据将要存放的指针 //Data read out will be stored pointer
返回   读出来的字节数量 //Read out the number of bytes
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	u8 temp;
	IIC_Start();
	IIC_Send_Byte(dev);	   //发送写命令 //Send Write Command
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //发送地址 //Send Address
    IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte(dev+1);  //进入接收模式 //Receive mode
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=(length-1))
		 	temp = IIC_Read_Byte(1);  //带ACK的读数据 //Reading data with an ACK
		 	else  
			temp = IIC_Read_Byte(0);	 //最后一个字节NACK //NACK last byte

		data[count] = temp;
	}
    IIC_Stop();//产生一个停止条件 //Generating a stop condition
    return count;
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*功　　能:	    将多个字节写入指定设备 指定寄存器 //A plurality of bytes written to the specified device-specific register
输入	dev  目标设备地址 //Target Device Address
		reg	  寄存器地址 //Register Address
		length 要写的字节数 //The number of bytes to write
		*data  将要写的数据的首地址 //The first address data to be written
返回   返回是否成功 //Return success
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   //发送写命令 //Send Write Command
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //发送地址 //Send Address
    IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	 }
	IIC_Stop();//产生一个停止条件 //Generating a stop condition

    return 1; //status == 0;
}

/**************************实现函数********************************************
*函数原型:		u8 IICreadByte(u8 dev, u8 reg, u8 *data)
*功　　能:	    读取指定设备 指定寄存器的一个值 //Reads a value specifying device register
输入	dev  目标设备地址 //Target Device Address
		reg	   寄存器地址 //Register Address
		*data  读出的数据将要存放的地址 //Data read will be stored in address
返回   1
*******************************************************************************/ 
u8 IICreadByte(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************实现函数********************************************
*函数原型:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*功　　能:	    写入指定设备 指定寄存器一个字节 //Writes a byte specifying device register
输入	dev  目标设备地址 //Target Device Address
		reg	   寄存器地址 //Register Address
		data  将要写入的字节 //The bytes to be written
返回   1
*******************************************************************************/ 
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位 //Read-modify-write specifying device registers a plurality of bits in a byte
输入	dev  目标设备地址 //Target Device Address
		reg	   寄存器地址 //Register Address
		bitStart  目标字节的起始位 //Target byte start bit
		length   位长度 //Bit length
		data    存放改变目标字节位的值 //Change the value of the target byte stored bits
返回   成功 为1  //Success 1
 		失败为0 //Failed 0
*******************************************************************************/ 
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (IICreadByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的1个位 //Read-modify-write specifying device registers a byte one bit
输入	dev  目标设备地址 //Target Device Address
		reg	   寄存器地址 //Register Address
		bitNum  要修改目标字节的bitNum位 //To modify the target byte bitNum bit
		data  为0 时，目标位将被清0 否则将被置位 //0, the target position will be cleared or will be set
返回   成功 为1 
 		   失败为0
*******************************************************************************/ 
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    
    return IICwriteByte(dev, reg, b);
}

//------------------End of File----------------------------

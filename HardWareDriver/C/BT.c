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
BT.c file
编写者：小马  (Camel)
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.蓝牙透传模块的电源使能端BT_EN--->PB2 //Bluetooth transparent transmission module power enable end BT_EN--->PB2
2.打开蓝牙电源-->BT_EN=1; //Turn on Bluetooth power-->BT_EN=1;
------------------------------------
*/


#include "BT.h"
#include "delay.h"
#include "UART1.h"
#include "stdio.h"
#include "string.h"
#include "sys_fun.h"
#include "control.h"
#include "Led.h"
	
BTtype CrazepoyBT;//实例化一个蓝牙结构体 //Examples of a Bluetooth structure
float BTstate;//蓝牙是否需要写参数标志 //Whether Bluetooth need to write parameter flag
	
	
/********************************************
              蓝牙电源初始化函数 (Bluetooth power initialization function)
********************************************/
void BT_PowerInit(void)
{
    RCC->APB2ENR|=1<<3;      //使能PORTB时钟	//Enable PORTB clock
    GPIOB->CRL&=0XFFFFF0FF;  //PB2推挽输出 //PB2 push-pull output
    GPIOB->CRL|=0X00000300;
    GPIOB->ODR|=1<<2;        //PB2上拉 //Pull the PB2
    BT_on();   

}


char Cmdreturn[CmdreturnLength];//指令的返回结果缓存 //Command returns the result cache

/********************************************
              往蓝牙写入一个指令包 (Bluetooth is written to a command packet)
********************************************/
void Uart1SendaBTCmd(const char *p)
{
  char i;
	
  for(i=0;i<CmdreturnLength;i++) Cmdreturn[i] = 0;//释放指令接收缓存 //Release command receive buffer
	LedA_on;
	delay_ms(100);//写完一条指令，延时500ms再度接收缓存 //Finished an instruction 500ms delay once again receive buffer
	LedA_off;
  for(i=0;i<strlen(p);i++)
  UART1_Put_Char(*(p+i));  
  delay_ms(100);//写完一条指令，延时500ms再度接收缓存 //Finished an instruction 500ms delay once again receive buffer
	
	
  i=0;
  while(UartBuf_Cnt(&UartRxbuf) != 0)     //当串口缓冲不为空时，将串口缓冲赋值给指令结果缓冲 //When the serial buffer is not empty , the serial buffer is assigned to command the result buffer
  Cmdreturn[i++] = UartBuf_RD(&UartRxbuf);
}

/********************************************
         判断一个指令返回是不是等于设定值 (Analyzing a command returns is not equal to the set value)
         返回值：0-->指令与设定值不匹配 (Return Value : 0 - > command does not match the setpoint)
                 1-->指令与设定值匹配 (Command and setpoint adjustment)
********************************************/
char CmdJudgement(const char *p)
{
  char i;
  for(i=0;i<strlen(p);i++) if(Cmdreturn[i] != *(p+i)) break;
  if(i != strlen(p)) return 0;
  return 1;
}

const char ATcmdAsk[]    =		 {"AT"};
const char ATcmdAnswer[] =     {"OK"};

const char ATcmdNameAsk[] = 	 {"AT+NAME?"};
const char ATcmdNameAnswer[] =  {"OK+NAME:Crazepony"};	//{BT_BAUD_AT};//
const char ATcmdNameSet[] = 	 {"AT+NAMECrazepony"};    //设置蓝牙设备名为：Crazepony，当然可以在这里修改成 what ever you want... //Setting up the Bluetooth device name : Crazepony, of course, where you can revise what ever you want...

const char ATcmdCodeAsk[] = 	 {"AT+PIN?"};
const char ATcmdCodeAnswer[] = {"OK+PIN:1234"};	
const char ATcmdCodeSet[] =		 {"AT+PIN1234"};          //蓝牙配对密码默认为1234 //Bluetooth pairing default password is 1234

const char ATcmdBaudAsk[] =		 {"AT+BAUD?"};
const char ATcmdBaudAnswer[] = {"OK+BAUD:115200"};
const char ATcmdBaudSet[] =    {"AT+BAUD8"};            //修改此处，可以修改蓝牙波特率 //Modify Here, you can modify Bluetooth baud rate
																//baud1--->1200
																//baud2--->2400
																//baud3--->4800
																//baud4--->9600
																//baud5--->19200
																//baud6--->38400
																//baud7--->57600
																//baud8--->115200                                        


/*得到蓝牙透传当前通信波特率,返回当前波特率值*/ //Get the current Bluetooth communication baud rate pass-through, returns the current baud rate
u32 BT_CurBaud_Get(void)
{
	static u32 bandsel[8] = {1200,2400,4800,9600,19200,38400,57600,115200};//蓝牙波特率率表
  u8 i;

		BT_on();        //开蓝牙 //Open Bluetooth
    delay_ms(500); //等待蓝牙稳定 //Wait Bluetooth stable
		/**确定当前蓝牙波特率**/ //Bluetooth determine the current baud rate
			for(i=0;i<8;i++)
			{
				UART1_init(SysClock,bandsel[i]); 
				Uart1SendaBTCmd(ATcmdAsk);
				if(CmdJudgement(ATcmdAnswer) == true)
				{
				  //printf("\r\nHM-06 baud -->%d\r\n",bandsel[i]);
					break;//得到当前波特率为Bandsel[i] //Get the current baud rate Bandsel[i]
				}
			}
	return bandsel[i];
}

extern void SaveParamsToEEPROM(void);

/********************************************
              写蓝牙参数函数 (Write Bluetooth function parameters)
********************************************/
void BT_ATcmdWrite(void)
{
	static	u32 BT_CurBaud;

	BT_CurBaud = BT_CurBaud_Get();
	if((BT_CurBaud == BT_BAUD_Set))  BTstate = BThavewrote;//检测到蓝牙当前的波特率和设定值不同，就写入设定值 //Bluetooth current baud rate detection and setting values are different, the set value is written
	else 				BTstate = BTneedwrite;

				if(BTstate == BTneedwrite)
					{
						LedA_off;LedB_off;LedC_off;LedD_off;
						UART1_init(SysClock,BT_CurBaud);//以当前波特率重新初始化串口 //In the current re-initialize the serial port baud rate
						/*开始配置蓝牙设备名,pin码，波特率*/ //Begin configuring the Bluetooth device name, pin code, baud rate
						Uart1SendaBTCmd(ATcmdAsk);
						//printf("\r\n与蓝牙通信中...\r\n"); //Bluetooth Communication
								if(CmdJudgement(ATcmdAnswer) == true)//有蓝牙返回，才往下写指令 //Bluetooth returned, only to write down instructions
									{
										Uart1SendaBTCmd(ATcmdNameAsk);
											if(CmdJudgement(ATcmdNameAnswer) == false)  {Uart1SendaBTCmd(ATcmdNameSet);LedA_off;LedB_on;LedC_off;LedD_on; }   
												
											else ;
										Uart1SendaBTCmd(ATcmdCodeAsk);
											if(CmdJudgement(ATcmdCodeAnswer) == false) {Uart1SendaBTCmd(ATcmdCodeSet); LedA_on;LedB_off;LedC_on;LedD_off; }
												 
											else ;
										Uart1SendaBTCmd(ATcmdBaudAsk);
											if(CmdJudgement(ATcmdBaudAnswer) == false) {
																																	Uart1SendaBTCmd(ATcmdBaudSet);
																																	LedA_off;LedB_on;LedC_off;LedD_on;												
																																	BTstate = BThavewrote;
																																	SaveParamsToEEPROM();
																																	LedA_on;LedB_on;LedC_on;LedD_on;
																																	delay_ms(1000);
																																	LedA_off;LedB_off;LedC_off;LedD_off;
																																	}//最后修改波特率,并写入EEPROM //Last Modified baud rate, and write EEPROM
														
											else BTstate = BTneedwrite;
									
									}
								else  {BTstate = BTneedwrite; printf("\r\n与蓝牙通信失败\r\n");}  //Bluetooth communication and failure
					}
					else ;
			UART1_init(SysClock,BT_BAUD_Set);
}


#ifndef _ReceiveData_H_
#define _ReceiveData_H_
#include "stm32f10x.h"


//RC遥控 //RC remote control
typedef struct int16_rcget
{
    float ROOL;
    float PITCH;
    float THROTTLE;
    float YAW;
}RC_GETDATA;


extern RC_GETDATA RC_DATA,RC_DATA_RAW;//经过处理的RC数据 //RC data processed
extern uint8_t FLY_ENABLE;//飞行使能端  7/-5    14/15 //Flight Enable

void ReceiveDataFormNRF(void);
void ReceiveDataFormUART(void);
void Send_PIDToPC(void);
void Send_AtitudeToPC(void);
//extern int  Rool_error_init;     //如果飞机起飞朝左偏，Rool_error_init朝正向增大修改;朝右偏，Rool_error_init朝负向增大修改 //If the plane took off to the left bias , Rool_error_init forward towards increasing modified; rightward slant , Rool_error_init modification increases in the negative direction
//extern int  Pitch_error_init;     //如果飞机起飞朝前偏，Pitch_error_init朝负向增大修改;朝吼偏，Pitch_error_init朝正向增大修改 //If takeoff forward biased , Pitch_error_init modification increases in the negative direction ; towards roar partial , Pitch_error_init forward towards increasing modification
void NRFmatching(void);


#endif


#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f10x.h"


//#define Debug  //Debug or not conditional compilation

//待速转 //Pending transfer speed
#define SLOW_THRO 200
//定义飞机最大倾斜角度 //The maximum tilt angle is defined aircraft
#define  Angle_Max  40.0
#define  YAW_RATE_MAX  180.0f/M_PI_F		//deg/s  
//纠正姿态误差，可以用来抵抗重心偏移等带来的初始不平衡 //Correct attitude error can be used to resist gravity offset, initial imbalance brought
//#define  Rool_error_init   7      //如果飞机起飞朝左偏，Rool_error_init朝正向增大修改;朝右偏，Rool_error_init朝负向增大修改 //If the plane took off to the left bias , Rool_error_init forward towards increasing modified; rightward slant , Rool_error_init modification increases in the negative direction
//#define  Pitch_error_init  -5      //如果飞机起飞朝前偏，Pitch_error_init朝负向增大修改;朝后偏，Pitch_error_init朝正向增大修改 //If takeoff forward biased , Pitch_error_init modification increases in the negative direction ; rearward bias , Pitch_error_init forward towards increasing modification
//定高部分 //Given the high part
#define LAND_SPEED						1.2f		//m/s^2
#define ALT_VEL_MAX 					4.0f
#define THR_MIN								0.42f		//min thrust ，根据机重和最小降速而定，用于下降速度过大时，油门过小，导致失衡。再增加fuzzy control ，在油门小时用更大的姿态参数// According to machine weight and minimum deceleration may be, for the rate of decline is too large, the throttle is too small, causing the imbalance. Add fuzzy control, with a larger throttle hour attitude parameters


#define HOVER_THRU	         -0.63  //-0.5f  //悬停 //Hover


enum {CLIMB_RATE=0,MANUAL,LANDING};
extern uint8_t altCtrlMode;
extern float hoverThrust;
extern uint8_t zIntReset;
extern uint8_t offLandFlag;
extern float altLand;
extern uint8_t isAltLimit;
extern float thrustZSp,thrustZInt;

// PID结构体 //PID 구조
typedef struct
{
    float P;
    float I;
    float D;
    float Desired;
    float Error;
    float PreError;
    float PrePreError;
    float Increment;
    float Integ;
		float iLimit;
    float Deriv;
    float Output;
 
}PID_Typedef;


//写入Flash参数结构体 //Write Flash parameter structure
typedef struct
{
  u16 WriteBuf[10];       //写入flash的临时数组 //Flash write temporary array
  u16 ReadBuf[10];        //读取Flash的临时数组 //Read temporary array of Flash
  
}Parameter_Typedef;


void Controler(void);
void PID_INIT(void);
void PID_Calculate(void);

void CtrlAttiAng(void);
void CtrlAttiRate(void);
void CtrlAlti(void);
void CtrlAltiVel(void);
void CtrlMotor(void);
void CtrlTest(void);
void CtrlAttiRateNew(void);
void CtrlAttiNew(void);

void SetHeadFree(uint8_t on);

extern u16 PIDWriteBuf[3];//写入flash的临时数字，由NRF24L01_RXDATA[i]赋值 //Write flash provisional figures from NRF24L01_RXDATA [i] Assignment

extern PID_Typedef pitch_angle_PID;	  //pitch角度环的PID //PID pitch angle ring
extern PID_Typedef pitch_rate_PID;		//pitch角速率环的PID //PID pitch angular velocity loop

extern PID_Typedef roll_angle_PID;    //roll角度环的PID //PID roll angle ring
extern PID_Typedef roll_rate_PID;     //roll角速率环的PID //PID roll angular velocity loop

extern PID_Typedef yaw_angle_PID;     //yaw角度环的PID  //PID yaw angle ring
extern PID_Typedef yaw_rate_PID;      //yaw的角速率环的PID  //PID yaw angular velocity of the ring

extern PID_Typedef	alt_PID;
extern PID_Typedef alt_vel_PID;


extern float gyroxGloble;
extern float gyroyGloble;

extern volatile unsigned char motorLock;
#endif



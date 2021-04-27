#include  <math.h>    //Keil library  
#include  <stdio.h>
#include "sys.h"

GPIO_InitTypeDef GPIO_InitStructure;
ErrorStatus HSEStartUpStatus;

#define   uchar unsigned char
#define   uint unsigned int        

// ??MPU6050????
//****************************************
#define        SMPLRT_DIV                0x19        //??????,???:0x07(125Hz)
#define        CONFIG                        0x1A        //??????,???:0x06(5Hz)
#define        GYRO_CONFIG                0x1B        //??????????,???:0x18(???,2000deg/s)
#define        ACCEL_CONFIG        0x1C        //?????????????????,???:0x01(???,2G,5Hz)
#define        ACCEL_XOUT_H        0x3B
#define        ACCEL_XOUT_L        0x3C
#define        ACCEL_YOUT_H        0x3D
#define        ACCEL_YOUT_L        0x3E
#define        ACCEL_ZOUT_H        0x3F
#define        ACCEL_ZOUT_L        0x40
#define        TEMP_OUT_H                0x41
#define        TEMP_OUT_L                0x42

#define        GYRO_XOUT_H                0x43
#define        GYRO_XOUT_L                0x44        
#define        GYRO_YOUT_H                0x45
#define        GYRO_YOUT_L                0x46
#define        GYRO_ZOUT_H                0x47
#define        GYRO_ZOUT_L                0x48

#define        PWR_MGMT_1                0x6B        //????,???:0x00(????)
#define        WHO_AM_I                0x75        //IIC?????(????0x68,??)


//****************************

#define        MPU6050_Addr   0xD0          //?????IIC???????,??ALT  ADDRESS????????
#define        TRUE   1
#define        FALSE   0

//************************************
/*??IIC????????*/
#define SCL_H         GPIOB->BSRR = GPIO_Pin_6
#define SCL_L         GPIOB->BRR  = GPIO_Pin_6 
   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_7
#define SDA_L         GPIOB->BRR  = GPIO_Pin_7

#define SCL_read      GPIOB->IDR  & GPIO_Pin_6
#define SDA_read      GPIOB->IDR  & GPIO_Pin_7

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART2_Configuration(void);
void WWDG_Configuration(void);
void Delay(u32 nTime);
void Delayms(vu32 m);  
void  USART2_SendData(uchar SendData);
void detection180(u8 mode,int b2,int res);
void I2C_GPIO_Config(void);
void I2C_delay(void);
void delay5ms(void);
int I2C_Start1(void);
void I2C_Stop1(void);
void I2C_Ack1(void);
void I2C_NoAck1(void);
int I2C_WaitAck1(void);
void I2C_SendByte1(u8 SendByte);
unsigned char I2C_RadeByte1(void);
int Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);	
void Init_MPU6050(void);
void READ_MPU6050(void);
int MPU6050_Get_Angle(float res,float resb);



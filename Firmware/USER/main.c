#include "tly.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "myiic.h"
#include "MAX30102.h"
#include "algorithm.h"
#include "stm32f10x.h"

#define MAX_BRIGHTNESS 255
#define ADC1_DR_Address    ((u32)0x4001244C)
__IO u16 ADC_ConvertedValue;  

uint32_t aun_ir_buffer[500]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;

int32_t get_heart(void);
void ADC1_Init(void);
void ADC1_Mode_Config(void);
float Read_ADC_data(ADC_TypeDef* ADCx);

uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
int i;
int32_t n_brightness;
float f_temp;
int32_t rate;
int b2;
unsigned char TX_DATA[4];           //??????
unsigned char BUF[20];       //???????
char  test=0;//IIC??
short T_X,T_Y,T_Z,J_X,J_Y,J_Z,T_T;                 //X,Y,Z?,??
int aaa=1;

 int main(void)
 { 

	float AD_value;	
	int res;
	u8 flag=1,num=0,mode=0;
	RCC_Configuration();                 //??RCC
	GPIO_Configuration();                 //??GPIO
	USART2_Configuration();         //????1
	I2C_GPIO_Config();                 //??IIC????
	Delayms(10);                                 //??
	Init_MPU6050();                     //???MPU6050
	ADC1_Init();
	 
    //uint8_t IIC_Flag=1;//IICÍ¨ÐÅ×´Ì¬£¬0Îª³É¹¦1ÎªÊ§°Ü
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// ÉèÖÃÖÐ¶ÏÓÅÏÈ¼¶·Ö×é2
	delay_init();	    	 //ÑÓÊ±º¯Êý³õÊ¼»¯	  
	uart_init(9600);	 	//´®¿Ú³õÊ¼»¯Îª9600
    IIC_Init();
    
    
    if(maxim_max30102_reset());//¸´Î» MAX30102
        //printf("max30102_reset failed!\r\n");
    if(maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy));//read and clear status register
        //printf("read_reg REG_INTR_STATUS_1 failed!\r\n");
    if(maxim_max30102_init());//³õÊ¼»¯MAX30102
        //printf("max30102_init failed!\r\n");
    
    //printf("Ö¸Ê¾µÆÁÁÁËÂð£¿\r\n");
    
////////    n_brightness=0;
////////    un_min=0x3FFFF;
////////    un_max=0;
////////    
////////    n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps
////////    
////////    //printf("²É¼¯500¸öÑù±¾\r\n");
////////    //read the first 500 samples, and determine the signal range
////////    for(i=0;i<n_ir_buffer_length;i++)
////////    {
////////        while(max30102_INTPin==1);   //µÈ´ýMAX30102ÖÐ¶ÏÒý½ÅÀ­µÍ

////////        maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
////////            
////////        if(un_min>aun_red_buffer[i])
////////            un_min=aun_red_buffer[i];    //update signal min
////////        if(un_max<aun_red_buffer[i])
////////            un_max=aun_red_buffer[i];    //update signal max
////////    }
////////    un_prev_data=aun_red_buffer[i];
////////	maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
	
	Delayms(500);
	ADC1_Init();
	AD_value  = Read_ADC_data(ADC1); 
	   
   //printf("AD value = %f mV  \r\n", AD_value);
    printf("1");
    //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)

    //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
    while(1)
    {
//		rate=get_heart();
//		printf(", HR=%i, ", rate); 
//		ADC1_Init();
//		AD_value  = Read_ADC_data(ADC1); 
//		printf("AD value = %f mV  \r\n", AD_value);
//		Delayms(500);
		READ_MPU6050();                 //??MPU6050??
		b2=MPU6050_Get_Angle(J_Y,J_X);
		if(abs(T_X)+abs(T_Y)>500&&aaa==1)
		{
			if(mode==1)mode=0;
			else mode=1;
			printf("z");
			num=0;
			aaa=0;
		}
		else
		{
			num++;
			if(num>10)
			{
				num=0;
			}
		}
		if(flag==1)
		{
			flag=0;
			res=b2;
		}
		detection180(mode,b2,res);
    }

}

int32_t get_heart(void)
{

	i=0;
	un_min=0x3FFFF;
	un_max=0;
	
	//dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
	for(i=100;i<500;i++)
	{
		aun_red_buffer[i-100]=aun_red_buffer[i];
		aun_ir_buffer[i-100]=aun_ir_buffer[i];
		
		//update the signal min and max
		if(un_min>aun_red_buffer[i])
		un_min=aun_red_buffer[i];
		if(un_max<aun_red_buffer[i])
		un_max=aun_red_buffer[i];
	}
	
	//take 100 sets of samples before calculating the heart rate.
	for(i=400;i<500;i++)
	{
		un_prev_data=aun_red_buffer[i-1];
		while(max30102_INTPin==1);   //µÈ´ýMAX30102ÖÐ¶ÏÒý½ÅÀ­µÍ
		
		maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));
	
		if(aun_red_buffer[i]>un_prev_data)//just to determine the brightness of LED according to the deviation of adjacent two AD data
		{
			f_temp=aun_red_buffer[i]-un_prev_data;
			f_temp/=(un_max-un_min);
			f_temp*=MAX_BRIGHTNESS;
			n_brightness-=(int)f_temp;
			if(n_brightness<0)
				n_brightness=0;
		}
		else
		{
			f_temp=un_prev_data-aun_red_buffer[i];
			f_temp/=(un_max-un_min);
			f_temp*=MAX_BRIGHTNESS;
			n_brightness+=(int)f_temp;
			if(n_brightness>MAX_BRIGHTNESS)
				n_brightness=MAX_BRIGHTNESS;
		}


		//send samples and calculation result to terminal program through UART
//		printf("red=");
//		printf("%i", aun_red_buffer[i]);
//		printf(", ir=");
//		printf("%i", aun_ir_buffer[i]);
//		printf(", HR=%i, ", n_heart_rate); 
//		printf("HRvalid=%i, ", ch_hr_valid);
//		printf("SpO2=%i, ", n_sp02);
//		printf("SPO2Valid=%i\r\n", ch_spo2_valid);
	} 
	maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
	return n_heart_rate;
}




/*ÅäÖÃADC1µÄ¹¤×÷Ä£Ê½ÎªMDAÄ£Ê½  */
 void ADC1_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;                                                    
	//???_ADC??-??   
	
	GPIO_InitTypeDef GPIO_InitStructure;                                                  
	//???_????-??   
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);           
	//????_GPIOA,ADC1
	
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;                                             
	//???_??-PA1
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                     
	//???_??-????_50Mhz
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;                                         
	//???_??-????_????
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);                                                
	//???_??_????
	
	
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                                    
	//???_ADC-???_????
 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;                                         
	//???_ADC-????_???
	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                                   
	//???_ADC-????_??
	
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;                   
	//???_ADC-????_????
	
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                                
	//???_ADC-????_???
	
	ADC_InitStructure.ADC_NbrOfChannel = 1;                                               
	//???_ADC-????_???
	
	ADC_Init(ADC1, &ADC_InitStructure);                                                   
	//???_ADC_????
	
	
	
	//	RCC_ADCCLKConfig(RCC_PCLK2_Div2);                                                 
	//ADC1-??????       PS:?????ADC?????1/2?????
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_1Cycles5);            
	//?????ADC?????????   (ADC?? ,ADC?? ,????-????? ,?????)     
 
 
	
	ADC_Cmd(ADC1,ENABLE);   
	
	ADC_ResetCalibration(ADC1);                                                           
	//???????             ????1
	
	while(ADC_GetResetCalibrationStatus(ADC1));                                           
	//???????????      ????0
 
  ADC_StartCalibration(ADC1);                                                           
	//ADC??                    ????1
	
  while(ADC_GetCalibrationStatus(ADC1)); 

}

/*³õÊ¼»¯ADC1 */
void ADC1_Init(void)
{
	ADC_DeInit(ADC1); 
	ADC1_Mode_Config();
}
float Read_ADC_data(ADC_TypeDef* ADCx)
{
	float kk=0;                                                                                                                                                                                                                                                                                                                                                               
	
	ADC_SoftwareStartConvCmd(ADCx,ENABLE);                                                
	//??_ADC????-??    ??????0                                                                  
				
	while(!ADC_GetFlagStatus(ADCx,ADC_FLAG_EOC));                                         
	//??????             ????1                                                                            
			
	kk=(3.3*(((float)ADC_GetConversionValue(ADCx)/4096)));                                
	//??????????,    ??????????,????????0                                                                                    
	
	return kk;                                                                            
	//?????                                          
	
}
void I2C_GPIO_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
 
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void I2C_delay(void)
{
                
   u8 i=30; //????????        ,??????5????
   while(i) 
   { 
     i--; 
   }  
}

void delay5ms(void)
{
                
   int i=5000;  
   while(i) 
   { 
     i--; 
   }  
}
int I2C_Start1(void)
{
        SDA_H;
        SCL_H;
        I2C_delay();
        if(!SDA_read)return FALSE;        //SDA?????????,??
        SDA_L;
        I2C_delay();
        if(SDA_read) return FALSE;        //SDA??????????,??
        SDA_L;
        I2C_delay();
        return TRUE;
}
void I2C_Stop1(void)
{
        SCL_L;
        I2C_delay();
        SDA_L;
        I2C_delay();
        SCL_H;
        I2C_delay();
        SDA_H;
        I2C_delay();
} 
void I2C_Ack1(void)
{        
        SCL_L;
        I2C_delay();
        SDA_L;
        I2C_delay();
        SCL_H;
        I2C_delay();
        SCL_L;
        I2C_delay();
}   
void I2C_NoAck1(void)
{        
        SCL_L;
        I2C_delay();
        SDA_H;
        I2C_delay();
        SCL_H;
        I2C_delay();
        SCL_L;
        I2C_delay();
} 
int I2C_WaitAck1(void)          //???:=1?ACK,=0?ACK
{
        SCL_L;
        I2C_delay();
        SDA_H;                        
        I2C_delay();
        SCL_H;
        I2C_delay();
        if(SDA_read)
        {
      SCL_L;
          I2C_delay();
      return FALSE;
        }
        SCL_L;
        I2C_delay();
        return TRUE;
}
void I2C_SendByte1(u8 SendByte) //????????//
{
    u8 i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        I2C_delay();
                SCL_H;
        I2C_delay();
    }
    SCL_L;
}  
unsigned char I2C_RadeByte1(void)  //????????//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;                                
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2C_delay();
          SCL_H;
      I2C_delay();        
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;
    return ReceiveByte;
} 

int Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)                     //void
{
          if(!I2C_Start1())return FALSE;
    I2C_SendByte1(SlaveAddress);   //??????+???//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//???????+???? 
    if(!I2C_WaitAck1()){I2C_Stop1(); return FALSE;}
    I2C_SendByte1(REG_Address );   //???????      
    I2C_WaitAck1();        
    I2C_SendByte1(REG_data);
    I2C_WaitAck1();   
    I2C_Stop1(); 
    delay5ms();
    return TRUE;
}
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{   unsigned char REG_data;             
        if(!I2C_Start1())return FALSE;
    I2C_SendByte1(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//???????+???? 
    if(!I2C_WaitAck1()){I2C_Stop1(); return FALSE;}
    I2C_SendByte1((u8) REG_Address);   //???????      
    I2C_WaitAck1();
    I2C_Start1();
    I2C_SendByte1(SlaveAddress+1);
    I2C_WaitAck1();

	REG_data= I2C_RadeByte1();
    I2C_NoAck1();
    I2C_Stop1();
    //return TRUE;
        return REG_data;

}                                                      
void RCC_Configuration(void)
{   
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  } 
   /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG and AFIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB , ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD , ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF , ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO  , ENABLE);  
}
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE  );
   /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                                 //        ????9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                 // ??????
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                 // ??????50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);                                 // ??A??
    
  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;                          //????10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;          //????
  GPIO_Init(GPIOA, &GPIO_InitStructure);                                  //??A??

}
void USART2_Configuration(void)
{

USART_InitTypeDef USART_InitStructure;
USART_ClockInitTypeDef  USART_ClockInitStructure;

RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE  );
RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE  );
USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;                        // ???????
USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;                                // ?????
USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;                                // ?????????????
USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;                // ?????????????SCLK??
/* Configure the USART1 synchronous paramters */
USART_ClockInit(USART2, &USART_ClockInitStructure);                                        // ?????????
                                                                                                                                         
USART_InitStructure.USART_BaudRate = 9600;                                                  // ????:115200
USART_InitStructure.USART_WordLength = USART_WordLength_8b;                          // 8???
USART_InitStructure.USART_StopBits = USART_StopBits_1;                                  // ??????1????
USART_InitStructure.USART_Parity = USART_Parity_No ;                                  // ????
USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;        // ???????

USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                  // ????+????
/* Configure USART1 basic and asynchronous paramters */
USART_Init(USART2, &USART_InitStructure);
    
  /* Enable USART1 */
USART_ClearFlag(USART2, USART_IT_RXNE);                         //???,??????????????
USART_ITConfig(USART2,USART_IT_RXNE, ENABLE);                //??USART1???
USART_Cmd(USART2, ENABLE);                                                        //USART1???:?? 
}
void NVIC_Configuration(void)
{ 
  NVIC_InitTypeDef NVIC_InitStructure;  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); 
 
  NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_Init(&NVIC_InitStructure);

}
 void Delay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}


 void Delayms(vu32 m)
{
  u32 i;
  
  for(; m != 0; m--)        
       for (i=0; i<50000; i++);
}

void WWDG_IRQHandler(void)
{
  /* Update WWDG counter */
  WWDG_SetCounter(0x50);
        
  /* Clear EWI flag */
  WWDG_ClearFlag(); 
}
void  USART2_SendData(uchar SendData)
{
USART_SendData(USART2, SendData);
Delayms(1);
}
void Init_MPU6050(void)
{
/*
   Single_Write(MPU6050_Addr,PWR_M, 0x80);   //
   Single_Write(MPU6050_Addr,SMPL, 0x07);    //
   Single_Write(MPU6050_Addr,DLPF, 0x1E);    //±2000°
   Single_Write(MPU6050_Addr,INT_C, 0x00 );  //
   Single_Write(MPU6050_Addr,PWR_M, 0x00);   //
*/
           Single_Write(MPU6050_Addr,PWR_MGMT_1, 0x00);        //??????
        Single_Write(MPU6050_Addr,SMPLRT_DIV, 0x07);
        Single_Write(MPU6050_Addr,CONFIG, 0x06);
        Single_Write(MPU6050_Addr,GYRO_CONFIG, 0x18);
        Single_Write(MPU6050_Addr,ACCEL_CONFIG, 0x01);
}
        
void READ_MPU6050(void)
{
   BUF[0]=Single_Read(MPU6050_Addr,GYRO_XOUT_L); 
   BUF[1]=Single_Read(MPU6050_Addr,GYRO_XOUT_H);
   T_X=        (BUF[1]<<8)|BUF[0];
   T_X/=16.4;                                                    //????X???

   BUF[2]=Single_Read(MPU6050_Addr,GYRO_YOUT_L);
   BUF[3]=Single_Read(MPU6050_Addr,GYRO_YOUT_H);
   T_Y=        (BUF[3]<<8)|BUF[2];
   T_Y/=16.4;                                                    //????Y???
   BUF[4]=Single_Read(MPU6050_Addr,GYRO_ZOUT_L);
   BUF[5]=Single_Read(MPU6050_Addr,GYRO_ZOUT_H);
   T_Z=        (BUF[5]<<8)|BUF[4];
   T_Z/=16.4;                                                //????Z???
	
   BUF[8]=Single_Read(MPU6050_Addr,ACCEL_XOUT_L); 
   BUF[9]=Single_Read(MPU6050_Addr,ACCEL_XOUT_H);
   J_X=        (BUF[9]<<8)|BUF[8];
   J_X/=16.4;                                                    //????X???

   BUF[10]=Single_Read(MPU6050_Addr,ACCEL_YOUT_L);
   BUF[11]=Single_Read(MPU6050_Addr,ACCEL_YOUT_H);
   J_Y=        (BUF[11]<<8)|BUF[10];
   J_Y/=16.4;                                                    //????Y???
   BUF[12]=Single_Read(MPU6050_Addr,ACCEL_ZOUT_L);
   BUF[13]=Single_Read(MPU6050_Addr,ACCEL_ZOUT_H);
   J_Z=        (BUF[13]<<8)|BUF[12];
   J_Z/=16.4;                                                //????Z???

  // BUF[6]=Single_Read(MPU6050_Addr,TEMP_OUT_L); 
  // BUF[7]=Single_Read(MPU6050_Addr,TEMP_OUT_H); 
  // T_T=(BUF[7]<<8)|BUF[6];
  // T_T = 35+ ((double) (T_T + 13200)) / 280;// ???????
}

//-1017~983
//-960~1030
int MPU6050_Get_Angle(float res,float resb)
{
	int b;
	if(resb>35)
		b=(res+1017)/2000*180;
	else
		b=(res-983)/-2000*180+180;
	return b;
}
 
/*************??***************/
void detection180(u8 mode,int b2,int res)
{
	static u8 flag1=1;
	static u8 num1=0,num2=0;
	static u16 kll=0;
	if(mode==0)
	{
		if(abs(T_Z)<50&&abs(b2-res)<30)
		{
//			if(b2<res)
//				res=b2;
			flag1=1;
		}
		if(b2-res>130&&flag1==1)
		{
			num1++;
			kll+=160;
//			printf("J:%d\nc:%d",num1,kll);
			printf("w");
			flag1=0;
			aaa=1;
			//rate=get_heart();
			//printf("%d",rate);
		}
	}
	else
	{
		if(abs(T_Z)<50&&abs(b2-res)<30)
		{
//			if(b2<res)
//				res=b2;
			flag1=1;
		}
		if(b2-res>70&&flag1==1)
		{
			num2++;
			kll+=80;
//			printf("P:%d\nc:%d",num2,kll);
			printf("p");
			flag1=0;
			aaa=1;
			//rate=get_heart();
			//printf("%d",rate);
		}
	}
	if((num1+num2)%6==0&&(num1+num2)!=0)
	{
	}
}

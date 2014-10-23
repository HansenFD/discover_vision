#include "stm32f4_discovery.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include "SCCB.h"
#include "OV2640.h"
#include "ov2640_regs.h"
#include "stm32f4_discovery_sdio_sd.h"
#include "ff.h"


#define BLOCK_SIZE            512 /* Block Size in Bytes */
FRESULT fr;          /* FatFs function common result code */
GPIO_InitTypeDef  GPIO_InitStructure;
extern volatile unsigned d_m;
uint8_t jpg_flag=0;
void Delayms(unsigned iv);

/*
USART  JPEG  - PA9
CAMERA CLICK - PA0
LED D12-D15
DCMI--

SDA   PB8   PB6
SCL   PB9
RESET PC1		OK
XLCK  PA8		OK
PWDN  PB5		OK

HSYNC PA4		OK		-
PCLK  PA6		OK

VSYNC PB7   OK

DCMI0 PC6 	OK
DCMI1 PC7 	OK 
DCMI2 PE0 	OK  
DCMI3 PE1 	OK  
DCMI4 PE4 	OK
DCMI5 PB6   OK 
DCMI6 PE5 	OK
DCMI7 PE6   OK

B-5,6,7,8,9
C-1,6,7
A-4,6,8
E-0,1,4,5,6,9

PD4 I2SAUDIO RST PULL LOW
*/
#define GPIO_WAKEUP_CLK    RCC_AHB1Periph_GPIOA
#define GPIO_WAKEUP_PORT   GPIOA
#define GPIO_WAKEUP_PIN    GPIO_Pin_0

#define JpegBufferLen (sizeof(JpegBuffer)/sizeof(char))


#define DCMI_DR_ADDRESS     0x50050028
unsigned char JpegBuffer[1024*100];//33KB
void OV2460_IOConfigure(void);
extern void OV2640_JPEGFullInit(void);

SD_Error Status = SD_OK;
unsigned char aBuffer_Block_Tx[512];

FATFS    microSD_fatfs;
DIR      directory;
FIL fil;       /* File object */

void OV2640_DMA_Init(void)
{
  DMA_InitTypeDef  DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Configures the DMA2 to transfer Data from DCMI to the LCD ****************/
  /* Enable DMA2 clock */
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  
  //RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_DMA2, ENABLE);  
  /* DMA2 Stream1 Configuration */  
  DMA_DeInit(DMA2_Stream1);

  DMA_InitStructure.DMA_Channel = DMA_Channel_1;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = DCMI_DR_ADDRESS;	
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)JpegBuffer;	
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 0xFFFF;  
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  //DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  //DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  //DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_Priority =DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;        
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	//DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream1, &DMA_InitStructure); 
	
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;                    
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
}

void DCMI_Config(void)
{
	
  DCMI_InitTypeDef DCMI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable DCMI GPIOs clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |  RCC_AHB1Periph_GPIOC |
                         RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);


  /* Enable DCMI clock */
	
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);

/*
	
HSYNC PA4		OK		-
PCLK  PA6		OK
VSYNC PB7   OK

DCMI0 PC6 	OK
DCMI1 PC7 	OK 
DCMI2 PE0 	OK  
DCMI3 PE1 	OK  
DCMI4 PE4 	OK
DCMI5 PB6   OK 
DCMI6 PE5 	OK
DCMI7 PE6   OK

*/
  /* Connect DCMI pins to AF13 ************************************************/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI); //HSYNC
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI); //PCLK
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI); //VSYNC
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI0
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI); //DCMI1
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_DCMI); //DCMI2
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_DCMI); //DCMI3
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_DCMI); //DCMI4
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI5
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_DCMI); //DCMI6
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI7
	
  
  
  /* DCMI GPIO configuration **************************************************/
  /* D0..D4(PH9/10/11/12/14), HSYNC(PH8) */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | 
//                                GPIO_Pin_12 | GPIO_Pin_14| GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;  
	
//  GPIO_Init(GPIOH, &GPIO_InitStructure);
/*
	
HSYNC PA4		OK		-
PCLK  PA6		OK
VSYNC PB7   OK

DCMI0 PC6 	OK
DCMI1 PC7 	OK 
DCMI2 PE0 	OK  
DCMI3 PE1 	OK  
DCMI4 PE4 	OK
DCMI5 PB6   OK 
DCMI6 PE5 	OK
DCMI7 PE6   OK

*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  //D0 D1
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_4| GPIO_Pin_5 | GPIO_Pin_6;  //D2 D3 D4 D6 D7
  GPIO_Init(GPIOE, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;  //D5
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  //HSYNC
  GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;  //VSYNC
  GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	
//  /* D5..D7(PI4/6/7), VSYNC(PI5) */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_5;
//  GPIO_Init(GPIOI, &GPIO_InitStructure);

  /* PCLK(PA6) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	

		
  /* DCMI configuration *******************************************************/ 
  DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_SnapShot;//DCMI_CaptureMode_Continuous
  DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;//DCMI_SynchroMode_Hardware
  DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;//DCMI_PCKPolarity_Falling
  DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_Low; //DCMI_VSPolarity_High
  DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_Low; //DCMI_HSPolarity_High
  DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;//DCMI_CaptureRate_All_Frame
  DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b; //?
  
  DCMI_Init(&DCMI_InitStructure);

	DCMI_JPEGCmd(ENABLE);
      
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure); 

  /* DCMI Interrupts config ***************************************************/
	 DCMI_ITConfig(DCMI_IT_VSYNC, ENABLE);
	 DCMI_ITConfig(DCMI_IT_LINE, ENABLE);
   DCMI_ITConfig(DCMI_IT_FRAME, ENABLE);
   DCMI_ITConfig(DCMI_IT_ERR, ENABLE);
	 DCMI_ITConfig(DCMI_IT_OVF, ENABLE);
}

void MCO1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_ClockSecuritySystemCmd(ENABLE);

	/* Enable GPIOs clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);
		
	/* Configure MCO (PA8) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  //UP
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_MCO1Config(RCC_MCO1Source_HSI, RCC_MCO1Div_1);// 16MHZ
}
void extclk_config(void)
{
	
}

void USART_Transmit(uint8_t ch_data)
{
	// Loop until the end of transmission 
  while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == 0)
  {
		;
	}
	USART_SendData(USART3, (uint8_t)ch_data);
}


void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
	
  GPIO_InitTypeDef GPIO_InitStructure;
	
  /* USARTx configured as follows:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode =USART_Mode_Tx;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	/* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);

  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(USART3, &USART_InitStructure);
    
  /* Enable USART */
  USART_Cmd(USART3, ENABLE);
	
}

void LEDSet(unsigned char ledno,unsigned value)
{
	if(ledno == 1)
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
}

void ConfigureLED(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}
void OV2460_IOConfigure(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	

	
	/* Enable GPIOs clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);/*PWDN PB5*/
	GPIO_SetBits(GPIOB, GPIO_Pin_5);
	Delayms(10); 
	GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	Delayms(100);
	
	
	 //RESET PC1
	/* Enable GPIOs clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_1);
	GPIO_ResetBits(GPIOC, GPIO_Pin_1);
	Delayms(200); 
	GPIO_SetBits(GPIOC, GPIO_Pin_1);
	Delayms(200);

}
void OV2640_ClickInit(void)
{
	RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_WAKEUP_PIN;
	GPIO_Init(GPIO_WAKEUP_PORT, &GPIO_InitStructure);
}

void PWMConfig(unsigned frequency)
{
	unsigned short p;
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	TIM_OCInitTypeDef TIM_OCStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	/* Enable clock for TIM4 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
/*	
	TIM4 is connected to APB1 bus, which has on F407 device 42MHz clock 				
	But, timer has internal PLL, which double this frequency for timer, up to 84MHz 	
	Remember: Not each timer is connected to APB1, there are also timers connected 	
	on APB2, which works at 84MHz by default, and internal PLL increase 				
	this to up to 168MHz 															
	
	Set timer prescaller 
	Timer count frequency is set with 
	
	timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)		
	
	In our case, we want a max frequency for timer, so we set prescaller to 0 		
	And our timer will have tick frequency		
	
	timer_tick_frequency = 84000000 / (0 + 1) = 84000000 
*/	
	TIM_BaseStruct.TIM_Prescaler = 0;
	/* Count up */
  TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
/*
	Set timer period when it have reset
	First you have to know max value for timer
	In our case it is 16bit = 65535
	To get your frequency for PWM, equation is simple
	
	PWM_frequency = timer_tick_frequency / (TIM_Period + 1)
	
	If you know your PWM frequency you want to have timer period set correct
	
	TIM_Period = timer_tick_frequency / PWM_frequency - 1
	
	In our case, for 10Khz PWM_frequency, set Period to
	
	TIM_Period = 84000000 / 10000 - 1 = 8399
	
	If you get TIM_Period larger than max timer value (in our case 65535),
	you have to choose larger prescaler and slow down timer tick frequency
*/
		//p = 84000000 / frequency - 1;
    //TIM_BaseStruct.TIM_Period = 8399; /* 10kHz PWM */
		TIM_BaseStruct.TIM_Period = 6; /* 10kHz PWM */
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;
	/* Initialize TIM4 */
    TIM_TimeBaseInit(TIM2, &TIM_BaseStruct);
	/* Start count on TIM4 */
    TIM_Cmd(TIM2, ENABLE);
		
	//PWM Init
	
	/* Common settings */
	
	/* PWM mode 2 = Clear on compare match */
	/* PWM mode 1 = Set on compare match */
	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
	
/*
	To get proper duty cycle, you have simple equation
	
	pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
	
	where DutyCycle is in percent, between 0 and 100%
	
	25% duty cycle: 	pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
	50% duty cycle: 	pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
	75% duty cycle: 	pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
	100% duty cycle:	pulse_length = ((8399 + 1) * 100) / 100 - 1 = 8399
	
	Remember: if pulse_length is larger than TIM_Period, you will have output HIGH all the time
*/
// 	TIM_OCStruct.TIM_Pulse = 2099; /* 25% duty cycle */
// 	TIM_OC1Init(TIM4, &TIM_OCStruct);
// 	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_OCStruct.TIM_Pulse = 3; /* 50% duty cycle */
	TIM_OC2Init(TIM2, &TIM_OCStruct);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
// 	TIM_OCStruct.TIM_Pulse = 6299; /* 75% duty cycle */
// 	TIM_OC3Init(TIM4, &TIM_OCStruct);
// 	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
// 	
// 	TIM_OCStruct.TIM_Pulse = 8399; /* 100% duty cycle */
// 	TIM_OC4Init(TIM4, &TIM_OCStruct);
// 	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
//GPIO config

	
	/* Clock for GPIOD */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* Alternating functions for pins */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
// 	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
// 	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
// 	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
// 	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
	
	/* Set pins */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;//GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

}
static void SDNVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = SD_SDIO_DMA_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);  
}

int main(void)
{
	

	
	unsigned char temp,key_flag=0;
	uint32_t i,JpegDataCnt=0;



	SysTick_Config(SystemCoreClock / 1000); //1Ms Tick
  SDNVIC_Configuration();
	

	
// 	Delayms(100);//power stable
	

 	Status = SD_Init();
  	Delayms(100);//power stable
  	SD_ReadBlock(aBuffer_Block_Tx,0,512);
  	Status = SD_WaitReadOperation();
   while(SD_GetStatus() != SD_TRANSFER_OK);

	
	 disk_initialize(0);
	

	if (f_mount(0, &microSD_fatfs)== FR_OK)
	{
		if (f_opendir(&directory, "/") != FR_OK) 
		{
			//while(1);
		}
	}
	ConfigureLED();
	
	USART_Config();
	
	OV2460_IOConfigure(); //Also does a power cycle;
	
	SCCB_GPIO_Config();
	
	Delayms(100); 
	
	PWMConfig(0);
	
	//while(1);
	
	//MCO1_Init();
	//extclk_config(12000000);
	
  Delayms(100); 
	
	OV2640_DMA_Init();
	
	DCMI_Config();
	
	OV2640_ClickInit();
	
	DCMI_SingleRandomWrite(0xFF, 0x01);//SELECT CMOS
	if(DCMI_SingleRandomRead(0x0A,&temp)!=0)//READ PID
		while(1);
	
	//DCMI_SingleRandomWrite(0xFF, 0x00);//SELECT DSP
	///DCMI_SingleRandomRead(0x44,&temp);
	
	OV2640_JPEGFullInit();
	
	jpg_flag=0;
	GPIO_ResetBits(GPIOD, GPIO_Pin_12);
	
	Delayms(5000); 
	
	key_flag = 0;
	
	GPIO_ResetBits(GPIOD, GPIO_Pin_12);
	
	
	DMA_Cmd(DMA2_Stream1, ENABLE);		
	DCMI_Cmd(ENABLE);
	DCMI_CaptureCmd(ENABLE);
  GPIO_SetBits(GPIOD, GPIO_Pin_12);
	fr = f_open(&fil, "f1.jpg",  FA_CREATE_ALWAYS | FA_WRITE);
	Delayms(400);
	
  while (1)
  {
		if(GPIO_ReadInputDataBit(GPIO_WAKEUP_PORT,GPIO_WAKEUP_PIN) && key_flag == 0	)
		{

		  key_flag = 1;
			OV2640_DMA_Init();
			DCMI_Config();
	    DMA_Cmd(DMA2_Stream1, ENABLE);		
			DCMI_Cmd(ENABLE);
			DCMI_CaptureCmd(ENABLE);
			Delayms(400);
			while(GPIO_ReadInputDataBit(GPIO_WAKEUP_PORT,GPIO_WAKEUP_PIN));
		}
		
		if(jpg_flag==1 )//&&key_flag
		{
			key_flag = 0;	
			DCMI_Cmd(DISABLE); 
			DCMI_CaptureCmd(DISABLE);
			DMA_Cmd(DMA2_Stream1, DISABLE);
			JpegDataCnt = 0;
			jpg_flag = 0;
			
			if( (JpegBuffer[0]==0xFF)&&(JpegBuffer[1]==0xD8) )
			{
					while ( !( (JpegBuffer[JpegBufferLen - JpegDataCnt-2]==0xFF) && (JpegBuffer[JpegBufferLen-JpegDataCnt-1]==0xD9) ) )
				{		
					JpegDataCnt++;
				}				
				// for(i = 0; i < (JpegBufferLen - JpegDataCnt); i++)	//sizeof(JpegBuffer)
				//{
				//	USART_Transmit(JpegBuffer[i]);
				//} 
				
				
				i=0;
				GPIO_SetBits(GPIOD, GPIO_Pin_13);
				fr = f_write(&fil, JpegBuffer, JpegBufferLen - JpegDataCnt, &i); 
				GPIO_ResetBits(GPIOD, GPIO_Pin_13);
				f_close(&fil);
				
				
			}
		}
  }
	

}

void Delayms(unsigned iv)
{
	d_m = iv;
	while(d_m);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

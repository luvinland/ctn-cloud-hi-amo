/**************************************************************************//**
 * @file     BoardSetup.c
 * @version  V0.10
 * $Revision: 1 $
 * $Date: 11/03/2017 10:04a $
 * @brief
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "PlatformAPI.h"
#include "BoardSupport.h"
#include "ConfigSysClk.h"
#include "usbd_audio.h"
#include "audioclass.h"

#include "Cyb_support2.h"   //SW
#include "..\NuOne\PlayNuOne.h"		//SW

#define xTEST_LED

#define SAMPLE_RATE             16000
#define INPUT_CHANNEL_COUNT     4
#define OUTPUT_CHANNEL_COUNT    2
#define AWE_FRAME_SIZE          40
#define BUF_COUNT               (INPUT_CHANNEL_COUNT * AWE_FRAME_SIZE)
#define CIRBUFFER_SIZE          PCM_SAMPLE_NUM*2//SW 0807 480

volatile uint32_t g_msTicks = 0;
extern INT32 g_target_baudrate;

DSCT_T     sPDMA_MIC[2];               // Provide PDMA description for ping-pong.
S_BUFCTRL sInBufCtrl;                  // Buffer control handler.
S_CIRBUFCTRL sCirBufCtrl; 

#if 1 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
volatile S_BUFCTRL sOutBufCtrl;
int32_t    ai32OutBuf[AWE_FRAME_SIZE * OUTPUT_CHANNEL_COUNT]; 	  // Buffer array: store audio data ready to send to DPWM

volatile S_BUFCTRL sSpiOutBufCtrl;
int32_t    ai32SpiOutBuf[AWE_FRAME_SIZE * OUTPUT_CHANNEL_COUNT];
#endif

// Function prototype declaration
void AWEProcessing(INT32 * pMicSamples, INT32 * pProcessedSamples);

/************************** Input buffer for BF_NR Algorithm *****************
** Input format: 4 channel, 16K sample rate, channel width 32bit, data width 24bit, Data alignment MSB, block size: 40 samples
**              first sample: Channel 0 32 bit, channel 1 32 bit, channel 2 32 bit, channel 3 32 bit --> 128 bit, 4 UINT32 words, 
**              second sample: Channel 0 32 bit, channel 1 32 bit, channel 2 32 bit, channel 3 32 bit,, and so on
**              one buffer total: 4*40 = 160 UINT32 words. 
** use Ping-Pang buffer: 160*2 buffer size
*******************************************************************/
int32_t    ai32InBuf[INPUT_CHANNEL_COUNT * AWE_FRAME_SIZE * 2]; // Ping-Pang buffer; INPUT_CHANNEL_COUNT: default 4; AWE_FRAME_SIZE = 40;
int32_t    i32recordbufferindex =0;

/************************** Output buffer for BF_NR Algorithm *****************
** if BF_NR Algorithm outputs 2 channel stereo output 
** Output format: 2 channels, 16K sample rate, channel width 32bit, data width 24bit, Data alignment MSB, block size: 40 samples
**              first sample: Channel 0 32 bit, channel 1 32 bit  -->  2 UINT32 words, 
**              second sample: Channel 0 32 bit, channel 1 32 bit,, and so on
**              total: 2*40 = 80 UINT32 words. 
** if BF_NR Algorithm outputs MONO output 
** Output format: 1 channel, 16K sample rate, channel width 32bit, data width 24bit, Data alignment MSB, block size: 32 samples
**              first sample: Channel 0 32 bit  --> 1 UINT32 words, 
**              second sample: Channel 0 32 bit,, and so on
**              total: 1*40 = 40 UINT32 words. 
*******************************************************************/
int32_t g_o32DataBuf[AWE_FRAME_SIZE * OUTPUT_CHANNEL_COUNT];    // AWE_FRAME_SIZE = 40; OUTPUT_CHANNEL_COUNT = 2 (stereo, default) 
  
/*********************  Circular buffer for VR ***************
** Mono 16K SR 16bit audio data will be pumped into this beffer by BF_NR Algorithm PDMA interrupt: 40 samples PDMA interrupt--> about 2ms per interrupt. 
** 1. Software will maintain a Read Index and a Write Index for this buffer via sCirBufCtrl; 
** 2. for each interrupt, it will pump into 1 16bit sample into this circular buffer; then sCirBufCtrl->u16WriteIdx++; 
** 3. after 256 samples received, software moves 256 samples of data into ai16CyberonInputBuffer (for VR processing); 
**    this operation will move sCirBufCtrl->u16ReadIdx forward 256; and singal bdataready --> notify other thread or main to call VR processing
******************************************************************/
int16_t ai16CirBuffer[CIRBUFFER_SIZE];


// ******************* Systick Handler **********************
#define TIMEOUT     0x02
extern void esp_i2c_command(uint8_t cmd);

extern volatile uint8_t g_u8Recognized;
extern volatile uint32_t g_u32FlashCount;
void SysTick_Handler(void)
{
	g_msTicks++; // used for AWE server to count processing MIPS	 
    if (g_u8Recognized == 1)
    {
        if (g_u32FlashCount++ >= 500)
        {
            g_u32FlashCount = 0;
            g_u8Recognized = 0;
            esp_i2c_command(TIMEOUT);
        }
    }
}

void Systick_Init(void)
{
   SystemCoreClockUpdate();
   NVIC_SetPriorityGrouping(3U);
   SysTick_Config(SYSTICK_1MS);
   NVIC_SetPriority(SysTick_IRQn, 1);
}


void I9400_System_Init(void) {
    uint8_t u8Lock; 
	volatile uint32_t i;

    // Initiate system clock.
    SYSCLK_INITIATE();
	u8Lock = SYS_Unlock();
	CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
    // Waiting for HXT clock ready 
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
	SYS_Lock(u8Lock);	

    GPIO_SetMode(PC, BIT3, GPIO_MODE_OUTPUT);  
	PC3 = 1;

	// PDMA Initial.
	{
		// Enable PDMA clock. 
		CLK_EnableModuleClock(PDMA_MODULE);
		// Reset PDMA module
		SYS_ResetModule(PDMA_RST);
		// Enable PDMA's NVIC
		NVIC_EnableIRQ(PDMA_IRQn);
        NVIC_SetPriority(PDMA_IRQn, 2);
	}

	// These defines are from  BUFCTRL.h for buffer control in this samle.
	// Buffer control handler configuration.
	BUFCTRL_CFG((&sInBufCtrl),ai32InBuf,sizeof(ai32InBuf)/sizeof(uint32_t));
    CIRBUFCTRL_CFG((&sCirBufCtrl),ai16CirBuffer,sizeof(ai16CirBuffer)/sizeof(uint16_t));

#if 1 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
	BUFCTRL_CFG((&sOutBufCtrl),ai32OutBuf,sizeof(ai32OutBuf)/sizeof(uint32_t));
	sOutBufCtrl.u16DataCount = sOutBufCtrl.u16BufCount;  

	BUFCTRL_CFG((&sSpiOutBufCtrl),ai32SpiOutBuf,sizeof(ai32SpiOutBuf)/sizeof(uint32_t));
	sSpiOutBufCtrl.u16DataCount = sSpiOutBufCtrl.u16BufCount;  
#endif

	// Initiate microphone.
	MIC_Init((S_BUFCTRL*)&sInBufCtrl);

	// Start microphone.
	MIC_Start();
    
    // Systick  initialization
    Systick_Init();
    
#if 1 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
	SPI_Init((S_BUFCTRL*)&sSpiOutBufCtrl);
	SPI_Start();

	SPK_Init((S_BUFCTRL*)&sOutBufCtrl, SAMPLE_RATE);
	//printf("DPWM sampling rate: %d Hz\r\n", DPWM_GetSampleRate());
	SPK_Start();

	I2C0_Init();
#endif

}

void StopAll(void)
{
	SysTick->CTRL  |= SysTick_CTRL_ENABLE_Msk;  /* Disable SysTick IRQ and SysTick Timer */
}


#if 1 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
S_BUFCTRL* psSPI_BufCtrl = NULL;            // Provide SPI to output data.

void SPI_Init(S_BUFCTRL* psSpiOutBufCtrl)
{
    SYS_UnlockReg(); // Jace. 191118. Improve i2s output perpormance.

	/* Select PCLK0 as the clock source of SPI2 */
	CLK_SetModuleClock(SPI2_MODULE, CLK_CLKSEL2_SPI2SEL_PCLK0, MODULE_NoMsk);

	/* Enable peripheral clock */
	CLK_EnableModuleClock(SPI2_MODULE);
	
	/* Set SPI2 multi-function pins */
	SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC5MFP_Msk | SYS_GPC_MFPL_PC6MFP_Msk | SYS_GPC_MFPL_PC7MFP_Msk)) | 
	(SYS_GPC_MFPL_PC5MFP_SPI2_MOSI | SYS_GPC_MFPL_PC6MFP_SPI2_MISO | SYS_GPC_MFPL_PC7MFP_SPI2_CLK);  
	
	SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC8MFP_Msk)) | (SYS_GPC_MFPH_PC8MFP_SPI2_SS);

	/* Slave mode, 16-bit word width, stereo mode, I2S format. Set TX and RX FIFO threshold to middle value. */
	/* I2S peripheral clock rate is equal to PCLK0 clock rate. */
	SPI_I2SOpen(SPI2, SPI_I2SSLAVE, 0, SPI_I2SDATABIT_16, SPI_I2SSTEREO, SPI_I2SFORMAT_I2S);

	/* Enable TX threshold level interrupt */
	SPI_I2S_SET_TXTH(SPI2, SPI_I2S_FIFO_TX_LEVEL_8);
	SPI_I2SEnableInt(SPI2, SPI_I2S_TXTH_INT_MASK);
	SPI_I2S_RST_TX_FIFO(SPI2);
	SPI_I2S_RST_RX_FIFO(SPI2);
	NVIC_EnableIRQ(SPI2_IRQn);

    SYS_LockReg(); // Jace. 191118. Improve i2s output perpormance.

	psSPI_BufCtrl = psSpiOutBufCtrl;

}

void SPI_Start(void)
{
	if( psSPI_BufCtrl != NULL )
	{
		// Enable I2S.
		SPI_I2SEnableControl(SPI2);
		SPI_I2S_ENABLE_TX(SPI2);
		SPI_I2S_ENABLE_RX(SPI2);
	}
	
}

void SPI_Stop(void)
{
	SPI_I2SDisableInt(SPI2, SPI_I2S_TXTH_INT_MASK);
	NVIC_DisableIRQ(SPI2_IRQn);

	SPI_I2SClose(SPI2);
	CLK_DisableModuleClock(SPI2_MODULE);
}

void SPI2_IRQHandler()
{
	uint32_t u32Tmp, i32Data1, i;

	if( !BUFCTRL_IS_EMPTY(psSPI_BufCtrl) )
	{
	    for(i = 0; i < 2; i++) // Jace. 191118. Improve i2s output perpormance.
        {   
    		BUFCTRL_READ(psSPI_BufCtrl, &u32Tmp);
    		SPI_I2S_WRITE_TX_FIFO(SPI2, u32Tmp);
        }
	}
	else
	{
		//printf("psSPI_BufCtrl empty\n");
	}

	//if( SPI_I2S_IS_RX_EMPTY(SPI2) == FALSE)
	{
        i32Data1 = SPI_I2S_READ_RX_FIFO(SPI2);
        BUFCTRL_WRITE((&sOutBufCtrl), i32Data1);
    }	

}


// Speaker(DPWM) = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
S_BUFCTRL* psSPK_BufCtrl = NULL;            // Provide Speaker to output data.

void SPK_Init(S_BUFCTRL* psOutBufCtrl, uint32_t u32SampleRate)
{
	// (1) Config DPWM to be a speaker output.
	{
        SYS_UnlockReg();

		// Enable DPWM clock. 
		CLK_EnableModuleClock(DPWM_MODULE);
		// Select DPWM CLK source HIRC. 
		CLK_SetModuleClock(DPWM_MODULE, CLK_CLKSEL2_DPWMSEL_HIRC, MODULE_NoMsk);
		//CLK_SetModuleClock(DPWM_MODULE, CLK_CLKSEL2_DPWMSEL_PCLK0, MODULE_NoMsk);

		// DPWM IPReset. 
		SYS_ResetModule(DPWM_RST);
		// HIRC=49.152MHz,Fs=24.576MHz/(128x4)=48kHz. 
		//DPWM_SET_CLKSET(DPWM, DPWM_CLKSET_512FS);
		//DPWM_SetSampleRate(SAMPLE_RATE);
		
		// HIRC=96MHz,Fs=48MHz = 500*16000*6
		DPWM_SET_CLKSET(DPWM, DPWM_CLKSET_500FS);
		DPWM_SetSampleRate(u32SampleRate); //16KHz
		
		// Enable threshold int and Data width 24Bits. 
		//DPWM_SET_FIFODATAWIDTH(DPWM, DPWM_FIFO_DATAWIDTH_MSB24BITS);
		DPWM_SET_FIFODATAWIDTH(DPWM, DPWM_FIFO_DATAWIDTH_16BITS);
		
		DPWM_ENABLE_FIFOTHRESHOLDINT(DPWM, 12); // Jace. 191118. Improve i2s output perpormance.
		// Enable NVIC.
		NVIC_EnableIRQ(DPWM_IRQn);

        SYS_LockReg();
        
		// GPIO multi-function.
		SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC13MFP_Msk|SYS_GPC_MFPH_PC12MFP_Msk|SYS_GPC_MFPH_PC11MFP_Msk|SYS_GPC_MFPH_PC10MFP_Msk))|(SYS_GPC_MFPH_PC13MFP_DPWM_LP|SYS_GPC_MFPH_PC12MFP_DPWM_LN|SYS_GPC_MFPH_PC11MFP_DPWM_RP|SYS_GPC_MFPH_PC10MFP_DPWM_RN);	

	}
	// (2) Config DPWM(Speaker) buffer control 
	{
		psSPK_BufCtrl = psOutBufCtrl;
	}
}

void SPK_Start(void)
{
	if( psSPK_BufCtrl != NULL )
	{
		DPWM_ENABLE_DRIVER(DPWM);
		DPWM_START_PLAY(DPWM);
	}
}

void SPK_Stop(void)
{
	DPWM_STOP_PLAY(DPWM);
	DPWM_DISABLE_DRIVER(DPWM);

	CLK_DisableModuleClock(DPWM_MODULE);
}

/* // Jace. 191118. Playback voice prompt using ESP32.
void SPK_Restart(uint32_t u32SampleRate)
{
    SPK_Stop();
    SPK_Init((S_BUFCTRL*)&sOutBufCtrl, u32SampleRate);
    SPK_Start();
}
*/

void DPWM_IRQHandler(void) 
{
	uint32_t u32Tmp, i;

	if( BUFCTRL_IS_EMPTY(psSPK_BufCtrl) ) 
	{
        //for( i=0; i<8; i++ )
        	DPWM_WRITE_INDATA(DPWM, 0);
	} 
	else 
	{
		for(i = 0; i < 4; i++)
		{
			//if(!DPWM_IS_FIFOFULL(DPWM)) 
			{
				//if( !BUFCTRL_IS_EMPTY(psSPK_BufCtrl) )
				{
					BUFCTRL_READ(psSPK_BufCtrl, &u32Tmp);
					DPWM_WRITE_INDATA(DPWM, u32Tmp);
				}
			}
			//else
				//break;
		}
	}

}

typedef struct {
	uint8_t u8DeviceAddr;
	uint8_t pau8Cmd;
	uint8_t endFlag;
} S_ESP_I2CCTRL;

volatile S_ESP_I2CCTRL s_ESP_I2CCtrl;

void I2C0_Init(void)
{
	/* Enable I2C0 clock */
	CLK_EnableModuleClock(I2C0_MODULE);

	/* Set GPA multi-function pins for I2C0 SDA and SCL */
	SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk)) | (SYS_GPA_MFPH_PA9MFP_I2C0_SCL | SYS_GPA_MFPH_PA10MFP_I2C0_SDA);

	/* Reset module */
	SYS_ResetModule(I2C0_RST);

	/* Open I2C0 and set clock to 100k */
	I2C_Open(I2C0, 100000);

	/* Get I2C0 Bus Clock */
	//printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

	/* Enable I2C interrupt */
	I2C_EnableInt(I2C0);
	NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);
}

void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

	printf("I2C0 u32Status[%x]\n", u32Status);
    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
	    if(u32Status == 0x08)                       /* START has been transmitted */
	    {
	        I2C_SET_DATA(I2C0, s_ESP_I2CCtrl.u8DeviceAddr << 1);    /* Write SLA+W to Register I2CDAT */
	        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
	    }
	    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
	    {
	        I2C_SET_DATA(I2C0, s_ESP_I2CCtrl.pau8Cmd);
	        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
	    }
	    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
	    {
	        I2C_STOP(I2C0);
	        //I2C_START(I2C0);
			s_ESP_I2CCtrl.endFlag = 1;
	    }
	    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
	    {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
			s_ESP_I2CCtrl.endFlag = 1;
	    }
		else
		{
			printf("0x?? return\n");
		}
    }
}

#endif


// Microphone(AMIC + 85L40)= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
typedef struct {
	uint8_t  u8DeviceAddr;
	uint16_t u16Counter;
	uint16_t u16MaxCount;
	uint8_t* pau8Cmd;
} S_MIC_I2CCTRL;

typedef struct {
	uint8_t  u8Reg[2];
	uint8_t  u8Value[2];
} S_MIC_I2CCMD;

#define MIC_I2S_PIN_MASK  (SYS_GPD_MFPL_PD2MFP_Msk|SYS_GPD_MFPL_PD3MFP_Msk|SYS_GPD_MFPL_PD4MFP_Msk|SYS_GPD_MFPL_PD5MFP_Msk|SYS_GPD_MFPL_PD6MFP_Msk)
#define MIC_I2S_PIN       (SYS_GPD_MFPL_PD2MFP_I2S0_MCLK|SYS_GPD_MFPL_PD3MFP_I2S0_LRCK|SYS_GPD_MFPL_PD4MFP_I2S0_DI|SYS_GPD_MFPL_PD5MFP_I2S0_DO|SYS_GPD_MFPL_PD6MFP_I2S0_BCLK)
#define MIC_I2C_PIN_MASK  (SYS_GPD_MFPH_PD14MFP_Msk|SYS_GPD_MFPH_PD15MFP_Msk)
#define MIC_I2C_PIN       (SYS_GPD_MFPH_PD14MFP_I2C1_SCL|SYS_GPD_MFPH_PD15MFP_I2C1_SDA)

#define MIC_PDMA_CH       (2)

S_BUFCTRL* psMIC_BufCtrl = NULL;           // Provide microphone input buffer control.
volatile S_MIC_I2CCTRL s_MIC_I2CCtrl;      // Provide microphone send command to 85L40S_BUFCTRL* psSPK_BufCtrl = NULL;

// Command for 85L40(transfer via I2C1)
S_MIC_I2CCMD const asMIC_Cmd_85L40[] = {
//-------
{	0x00	,	0x00	,	0x00	,	0x01	}	,
//-------
{	0x00	,	0x03	,	0x00	,	0x47	}	,  //MCLK divide by 3 for I94 MCLK limitation
{	0x00	,	0x04	,	0x00	,	0x01	}	,
{	0x00	,	0x05	,	0x31	,	0x26	}	,
{	0x00	,	0x06	,	0x00	,	0x08	}	,
{	0x00	,	0x07	,	0x00	,	0x10	}	,
{	0x00	,	0x08	,	0xC0	,	0x00	}	,
{	0x00	,	0x09	,	0xE0	,	0x00	}	,
{	0x00	,	0x0A	,	0xF1	,	0x3C	}	,
{	0x00	,	0x10	,	0x00	,	0x4F	}	,  //PCMB, 32Bit
{	0x00	,	0x11	,	0x00	,	0x00	}	,
{	0x00	,	0x12	,	0x00	,	0x00	}	,
{	0x00	,	0x13	,	0x00	,	0x00	}	,
{	0x00	,	0x14	,	0xC0	,	0x0F	}	,
{	0x00	,	0x20	,	0x00	,	0x00	}	,
{	0x00	,	0x21	,	0x70	,	0x0B	}	,
{	0x00	,	0x22	,	0x00	,	0x22	}	,
{	0x00	,	0x23	,	0x10	,	0x10	}	,
{	0x00	,	0x24	,	0x10	,	0x10	}	,
{	0x00	,	0x2D	,	0x10	,	0x10	}	,
{	0x00	,	0x2E	,	0x10	,	0x10	}	,
{	0x00	,	0x2F	,	0x00	,	0x00	}	,
{	0x00	,	0x30	,	0x00	,	0x00	}	,
{	0x00	,	0x31	,	0x00	,	0x00	}	,
{	0x00	,	0x32	,	0x00	,	0x00	}	,
{	0x00	,	0x33	,	0x00	,	0x00	}	,
{	0x00	,	0x34	,	0x00	,	0x00	}	,
{	0x00	,	0x35	,	0x00	,	0x00	}	,
{	0x00	,	0x36	,	0x00	,	0x00	}	,
{	0x00	,	0x37	,	0x00	,	0x00	}	,
{	0x00	,	0x38	,	0x00	,	0x00	}	,
{	0x00	,	0x39	,	0x00	,	0x00	}	,
{	0x00	,	0x3A	,	0x40	,	0x62	}	,  //16K Sample Rate, ADC OSR = 128
{	0x00	,	0x40	,	0x04	,	0x00	}	,  //DGAIN = 0dB
{	0x00	,	0x41	,	0x04	,	0x00	}	,  //DGAIN = 0dB
{	0x00	,	0x42	,	0x04	,	0x00	}	,  //DGAIN = 0dB
{	0x00	,	0x43	,	0x04	,	0x00	}	,  //DGAIN = 0dB
{	0x00	,	0x44	,	0x00	,	0xE4	}	,
{	0x00	,	0x48	,	0x00	,	0x00	}	,
{	0x00	,	0x49	,	0x00	,	0x00	}	,
{	0x00	,	0x4A	,	0x00	,	0x00	}	,
{	0x00	,	0x4B	,	0x00	,	0x00	}	,
{	0x00	,	0x4C	,	0x00	,	0x00	}	,
{	0x00	,	0x4D	,	0x00	,	0x00	}	,
{	0x00	,	0x4E	,	0x00	,	0x00	}	,
{	0x00	,	0x4F	,	0x00	,	0x00	}	,
{	0x00	,	0x50	,	0x00	,	0x00	}	,
{	0x00	,	0x51	,	0x00	,	0x00	}	,
{	0x00	,	0x52	,	0xEF	,	0xFF	}	,
{	0x00	,	0x57	,	0x00	,	0x00	}	,
{	0x00	,	0x58	,	0x1C	,	0xF0	}	,
{	0x00	,	0x59	,	0x00	,	0x08	}	,
{	0x00	,	0x60	,	0x00	,	0x60	}	,
{	0x00	,	0x61	,	0x00	,	0x00	}	,
{	0x00	,	0x62	,	0x00	,	0x00	}	,
{	0x00	,	0x63	,	0x00	,	0x00	}	,
{	0x00	,	0x64	,	0x00	,	0x11	}	,
{	0x00	,	0x65	,	0x02	,	0x20	}	,
{	0x00	,	0x66	,	0x00	,	0x0F	}	,
{	0x00	,	0x67	,	0x0D	,	0x04	}	,
{	0x00	,	0x68	,	0x70	,	0x00	}	,
{	0x00	,	0x69	,	0x00	,	0x00	}	,
{	0x00	,	0x6A	,	0x00	,	0x00	}	,
{	0x00	,	0x6B	,	0x21	,	0x21	}	,  //AGAIN = 32dB
{	0x00	,	0x6C	,	0x21	,	0x21	}	,  //AGAIN = 32dB
{	0x00	,	0x6D	,	0xF0	,	0x00	}	,
{	0x00	,	0x01	,	0x00	,	0x0F	}	,
{	0x00	,	0x02	,	0x80	,	0x03	}	
};

void MIC_Init(S_BUFCTRL* psInBufCtrl)
{

    // (1) Config I2C1 for sending command to 85L40
	// (1-1) Initiate I2C1
	{
        // Unlock protected registers
        SYS_UnlockReg();
		// Reset module. 
		SYS_ResetModule(I2C1_RST);
		// Enable I2C0 module clock. 
		CLK_EnableModuleClock(I2C1_MODULE);
		// Open I2C module and set bus clock. 
		I2C_Open(I2C1, 100000);
		// Enable I2C interrupt. 
		I2C_EnableInt(I2C1);
		NVIC_EnableIRQ(I2C1_IRQn);	
        // Lock protected registers
        SYS_LockReg();
		// GPIO multi-function. 
		SYS->GPD_MFPH = (SYS->GPD_MFPH & ~MIC_I2C_PIN_MASK)|MIC_I2C_PIN;
	}		
    
    // (1-2) Send command to 85L40 via I2C1
	{
		uint16_t u16i;
		I2C_SetBusClockFreq(I2C1,100000);
		s_MIC_I2CCtrl.u8DeviceAddr = 0x1C;
		for(u16i=0;u16i<sizeof(asMIC_Cmd_85L40)/sizeof(S_MIC_I2CCMD);u16i++) 
		{
			s_MIC_I2CCtrl.pau8Cmd = (uint8_t*)&asMIC_Cmd_85L40[u16i];
			s_MIC_I2CCtrl.u16Counter = 0;
			s_MIC_I2CCtrl.u16MaxCount = sizeof(S_MIC_I2CCMD);
			I2C_START(I2C1);
			/* Wait for I2C transmit completed. */
			while(s_MIC_I2CCtrl.u16MaxCount>0);
		}		
#if 1 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
		I2C1_Close();
#endif
	}
    
	// (2) Config I2S to ge voice data from 85L40
	{
		/* GPIO multi-function. */
		SYS->GPD_MFPL = (SYS->GPD_MFPL & ~MIC_I2S_PIN_MASK) | MIC_I2S_PIN;
		
		/* Enable I2S clock. */
		CLK_EnableModuleClock(I2S0_MODULE);

		/* Select I2S clock. */
		CLK_SetModuleClock(I2S0_MODULE, CLK_CLKSEL3_I2S0SEL_HXT, NULL);
		
		/* I2S IPReset. */
		SYS_ResetModule(I2S0_RST);
		
		/* Open I2S and enable master clock. */
		I2S_Open(I2S0, I2S_MASTER, SAMPLE_RATE, I2S_DATABIT_24, I2S_TDMCHNUM_4CH, I2S_STEREO, I2S_FORMAT_PCMMSB);
        // SR=16kHz, HXT=12.288MHz as I2S bus clock(I2SCLK), MCLK = 256*16k = 4.096MHz, but I2SCLK / MCLK must be 2's multiple (see TRM p801)
        // So here make MCLK = I2SCLK = 12.288MHz, and then divide by 3 in 85L40
		I2S_EnableMCLK(I2S0, SAMPLE_RATE * 256*3);
		
		/* I2S Configuration. */
		I2S_SET_PCMSYNC(I2S0, I2S_PCMSYNC_BCLK);
		I2S_SET_MONO_RX_CHANNEL(I2S0, I2S_MONO_RX_RIGHT);
        // make 24-bit data LSB alignment
		I2S_SET_STEREOORDER(I2S0, I2S_ORDER_EVENHIGH);
		
		/* Set channel width. */
		I2S_SET_CHWIDTH(I2S0, I2S_CHWIDTH_32);
		/* Set FIFO threshold. */
		I2S_SET_TXTH(I2S0, I2S_FIFO_TX_LEVEL_WORD_8);
		I2S_SET_RXTH(I2S0, I2S_FIFO_RX_LEVEL_WORD_9);
		/* Enable interrupt. */
		//I2S_ENABLE_INT(I2S0, I2S_TXTH_INT_MASK|I2S_RXTH_INT_MASK|I2S_TXOV_INT_MASK|I2S_RXOV_INT_MASK|I2S_TXUF_INT_MASK|I2S_RXUF_INT_MASK);
		/* Clear TX, RX FIFO buffer */
		I2S_CLR_TX_FIFO(I2S0);
		I2S_CLR_RX_FIFO(I2S0);
	}    
    
    // (3) Config PDMA for transfer data.
	{
        // MIC(RX) buffer description
        sPDMA_MIC[0].CTL = ((INPUT_CHANNEL_COUNT * AWE_FRAME_SIZE-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_FIX|PDMA_DAR_INC|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
        sPDMA_MIC[0].SA = (uint32_t)(&I2S0->RXFIFO);
        sPDMA_MIC[0].DA = (uint32_t)&(ai32InBuf[0]);
        sPDMA_MIC[0].NEXT = (uint32_t)&sPDMA_MIC[1] - (PDMA->SCATBA);	
        
        sPDMA_MIC[1].CTL = ((INPUT_CHANNEL_COUNT * AWE_FRAME_SIZE-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_FIX|PDMA_DAR_INC|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
        sPDMA_MIC[1].SA = (uint32_t)(&I2S0->RXFIFO);
        sPDMA_MIC[1].DA = (uint32_t)&(ai32InBuf[INPUT_CHANNEL_COUNT * AWE_FRAME_SIZE]);
        sPDMA_MIC[1].NEXT = (uint32_t)&sPDMA_MIC[0] - (PDMA->SCATBA);
        // Open PDMA channel
        PDMA_Open((1<<MIC_PDMA_CH));
        // Set TransMode
        PDMA_SetTransferMode(MIC_PDMA_CH, PDMA_I2S0_RX, TRUE, (uint32_t)&sPDMA_MIC[0]);
        // Enable interrupt
        PDMA_EnableInt(MIC_PDMA_CH,PDMA_INT_TRANS_DONE);
	}

	// Config DPWM(Speaker) buffer control 
	psMIC_BufCtrl = psInBufCtrl;
}

void MIC_Start(void)
{
	if( psMIC_BufCtrl != NULL )
	{
		I2S_ENABLE(I2S0);
		I2S_ENABLE_RX(I2S0);	
		I2S_ENABLE_RXDMA(I2S0);
	}
}

void MIC_Stop(void)
{
	I2S_DISABLE_RXDMA(I2S0);
	I2S_DISABLE_RX(I2S0);
	I2S_DISABLE(I2S0);
}

void I2C1_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C1);
    NVIC_DisableIRQ(I2C1_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C1);
    CLK_DisableModuleClock(I2C1_MODULE);
}

void I2C1_IRQHandler() 
{
    if(I2C_GET_TIMEOUT_FLAG(I2C1)) 
        I2C_ClearTimeoutFlag(I2C1); 
	else 
	{
        uint8_t u8Temp;
          
		switch(I2C_GET_STATUS(I2C1)) {
			/* START has been transmitted and Write SLA+W to Register I2CDAT. */
			case 0x08:
				I2C_SET_DATA(I2C1, s_MIC_I2CCtrl.u8DeviceAddr << 1);    
				I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);			
				break;
			/* SLA+W has been transmitted and ACK has been received. */
			case 0x18:
                u8Temp = s_MIC_I2CCtrl.u16Counter++;
                u8Temp = s_MIC_I2CCtrl.pau8Cmd[u8Temp];
				I2C_SET_DATA(I2C1, u8Temp);
				I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);		
				break;
			/* SLA+W has been transmitted and NACK has been received. */
			case 0x20:
				I2C_STOP(I2C1);
				I2C_START(I2C1);	
				s_MIC_I2CCtrl.u16MaxCount = 0;
				break;
			/* DATA has been transmitted and ACK has been received. */
			case 0x28:
                u8Temp = s_MIC_I2CCtrl.u16MaxCount;
				if(s_MIC_I2CCtrl.u16Counter < u8Temp) {
                    u8Temp = s_MIC_I2CCtrl.u16Counter++;
					I2C_SET_DATA(I2C1, s_MIC_I2CCtrl.pau8Cmd[u8Temp]);
					I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
				} else {
					I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
					s_MIC_I2CCtrl.u16MaxCount = 0;
				}
                break;
		}
	}
}


// PDMA =========================================================================================================
void PDMA_IRQHandler(void) 
{
    volatile uint32_t i; 
    volatile uint32_t i32MicReadPos;
    uint32_t u32TDStatus = PDMA_GET_TD_STS();

#if 1 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
	int32_t i32SPIOutData;
#endif

	if( u32TDStatus&(1<<MIC_PDMA_CH) )
	{
        PDMA_CLR_TD_FLAG((1<<MIC_PDMA_CH));
        psMIC_BufCtrl->u16DataCount += (psMIC_BufCtrl->u16BufCount/2);

        i32MicReadPos = i32recordbufferindex * AWE_FRAME_SIZE * INPUT_CHANNEL_COUNT; 

		PC3 ^= 1; // detect 200Hz in scope

        // shift all data from 24bit LSB to 24 bit MSB
        for (i = 0; i<BUF_COUNT; i++)
        {
            ai32InBuf[i32MicReadPos+i] <<=8;
        }

        // Call BF_NR Algorithm library for audio processing;
        AWEProcessing(&ai32InBuf[i32MicReadPos],g_o32DataBuf);

        // Make processed data LSB alignment and copy into UAC ring buffer        
        for (i=0;i<AWE_FRAME_SIZE;i++)
        {
#if 1 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
			i32SPIOutData = (g_o32DataBuf[(i*2)] & 0xFFFF0000) | ((g_o32DataBuf[(i*2)] >> 16) & 0x0000FFFF);
			BUFCTRL_WRITE((&sSpiOutBufCtrl),i32SPIOutData);
#endif
        }

        // update MIC input data record index
        i32recordbufferindex++; 
        if (i32recordbufferindex>1) i32recordbufferindex= 0;
		
        // prepare audio data for Cyberon, signal ready if 240 samples available
        PrepareCybData(g_o32DataBuf, &sCirBufCtrl);//SW

	}
}


void PrepareCybData(int32_t * InBuf, S_CIRBUFCTRL * pOutBuf)
{
    int32_t i;
    int16_t i16data;
    
    // write the BF_NR Algorithm output into circular buffer
    for (i = 0; i<AWE_FRAME_SIZE; i++)
    {
      CIRBUFCTRL_WRITE(pOutBuf,(int16_t)(InBuf[i*OUTPUT_CHANNEL_COUNT]>>16));
      

    }
    
    // if there are at least 256 samples of data, then move the data into Cyberon Input buffer & set ready signal.
    if (((pOutBuf->u16WriteIdx >  pOutBuf->u16ReadIdx) &&
        (pOutBuf->u16WriteIdx >= (pOutBuf->u16ReadIdx + PCM_SAMPLE_NUM))) ||
        ((pOutBuf->u16WriteIdx < pOutBuf->u16ReadIdx) &&
        ((pOutBuf->u16WriteIdx+PCM_SAMPLE_NUM) >= pOutBuf->u16ReadIdx)))
    {
      for (i = 0; i<PCM_SAMPLE_NUM; i++)
      {
        CIRBUFCTRL_READ(pOutBuf,&i16data);
        ai16CyberonInputBuffer[i] = i16data;
      }
      
      if (bCybBufReady)
      {
        // last processing is not don yet; raise en error flag here...
        // or return Error flag?
      }
      
      bCybBufReady = TRUE; 
    }
}

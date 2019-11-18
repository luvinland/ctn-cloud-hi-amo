#include "PlayNuOne.h"
//#include "pdma_dpwm.h"
#include "AudioCommon.h"
#include "..\Source\BoardSupport.h"//SW 1120
#include "OverLapRam.h"

//#define SPK_PDMA_CH     (3)


#if defined (__APROM) || defined(__DATAFLASH)
#include "string.h"
#endif


#ifdef SPIFLASH
//	extern S_SPIFLASH_HANDLER g_sSpiFlash;
#include "SPIFlash.h"
#endif


S_ROM_AUDIO_CHUNK_INFO sAudioChunkInfo;

DSCT_T     sPDMA_SPK[2];                    // Provide PDMA description for ping-pong.

BOOL	bPCMPlaying=FALSE;
BOOL 	bBufferEmpty=FALSE;
BOOL	bLastOneBuffer=FALSE;
UINT16 	u16SampleRate;
UINT32 	PDMA1CallBackCount;

//__align(4) UINT32 au32DecodeWorkBuf[(NUONEEX_DECODE_WORK_BUF_SIZE+3)/4];
//__align(4) UINT32 u32NuOneExTempBuf[(NUONEEX_DECODE_TEMP_BUF_SIZE+3)/4];
//INT16 i16OutBuf[2][NUONEEX_DECODE_SAMPLE_PER_FRAME*2];


uint32_t ReadDataCallback(void *pu8Buf, uint32_t u32StartAddr, uint32_t u32Count);
uint32_t UserEvent_ProcessCallback(uint16_t u16EventIndex, uint16_t u16EventSubIndex);
void DPWM_Start(void);

/*SW 1120 move to BoardSetup.c
void PDMA_IRQHandler(void)
{
	if( PDMA_GET_TD_STS()&(1<<SPK_PDMA_CH) )
	{
		PDMA_CLR_TD_FLAG((1<<SPK_PDMA_CH));
		bBufferEmpty=TRUE;
		PDMA1CallBackCount++;                      
	}
}
*/

uint32_t DPWM_SetSampleRate_M(uint32_t u32SampleRate)
{
	uint32_t u32Temp, u32ZOHDiv, u32Ratio, u32DPWMClk, u32ClkDiv, u32Error1, u32Error2;
	u32ClkDiv = 0;
	
	SystemCoreClockUpdate();
	
	/* Get DPWM working clock ratio and clock divisor. */
	u32Ratio = (DPWM->CTL&DPWM_CLKSET_500FS)?125:128;
	/* Get DPWM clock source frequency. */
	u32Temp = CLK->CLKSEL2 & CLK_CLKSEL2_DPWMSEL_Msk;
	switch(u32Temp)
	{
		case CLK_CLKSEL2_DPWMSEL_PCLK0:
			u32DPWMClk = CLK_GetPCLK0Freq();
		break;
		case CLK_CLKSEL2_DPWMSEL_PLL:
			u32DPWMClk = CLK_GetPLLClockFreq();
		break;
		case CLK_CLKSEL2_DPWMSEL_HXT:
			u32DPWMClk = CLK_GetHXTFreq();
		break;
		case CLK_CLKSEL2_DPWMSEL_HIRC:
			u32DPWMClk = __HIRC;
		break;
		default:
			return 0;
	}

	if(u32Temp==CLK_CLKSEL2_DPWMSEL_HIRC && u32DPWMClk==48000000)
	{
		u32Ratio =125;
	}
	
	if(u32SampleRate == 48000 || u32SampleRate == 96000)
	{
		u32ZOHDiv = 4;
		u32ClkDiv = u32DPWMClk / ((u32Ratio * u32ZOHDiv) * u32SampleRate) - 1;
	}
	else
	{
		/* Calculate divider value.(overall divisor = (DPWM's source clock/Fs)/Ratio). */
		u32Temp = (u32DPWMClk/u32SampleRate)/u32Ratio;
		/* Adjust Error */
		u32Error1 = ((u32DPWMClk/u32Ratio)/u32Temp)-u32SampleRate;
		u32Error2 = u32SampleRate- ((u32DPWMClk/u32Ratio)/(u32Temp+1));
		u32Temp = (u32Error1>u32Error2)?(u32Temp+1):u32Temp;
		
		if ((u32Temp >= DPWM_ZOHDIV_MIN))
		{
			if(u32Temp <= DPWM_ZOHDIV_MAX)
			{
				u32ZOHDiv = u32Temp;
				u32ClkDiv = 0;
			}else
			{
				do
				{
					u32ZOHDiv = u32Temp/(++u32ClkDiv+1);
				}
				while(u32ZOHDiv > DPWM_ZOHDIV_MAX);
			}
		}
		else
		{
			u32ZOHDiv = DPWM_ZOHDIV_MIN;
			u32ClkDiv = 0;
		}
	}
	
	

	DPWM_SET_CLOCKDIV(DPWM, u32ClkDiv);
	DPWM_SET_ZOHDIV(DPWM, u32ZOHDiv);
	
	/* Return exact frequency. */
	return ((u32DPWMClk/DPWM_GET_ZOHDIV(DPWM))/u32Ratio);
}

void DPWM_Init(uint32_t u32SampleRate)
{
	/* Unlock protected registers */
    SYS_UnlockReg();
	/* Enable DPWM module clock */
    CLK_EnableModuleClock(DPWM_MODULE);
	/* Set DPWM module clock */
	//CLK_SetModuleClock(DPWM_MODULE, CLK_CLKSEL2_DPWMSEL_PCLK0, MODULE_NoMsk); 
	CLK_SetModuleClock(DPWM_MODULE, CLK_CLKSEL2_DPWMSEL_HIRC, MODULE_NoMsk);
	
	/* Reset IP */
	SYS_ResetModule(DPWM_RST);
	
	/* Set DPWM sampling rate */
	// HIRC=48MHz,Fs=24.576MHz/(128x4)=48kHz. 
	DPWM_SET_CLKSET(DPWM, DPWM_CLKSET_512FS);//SW 0312 ??
	DPWM_SetSampleRate_M(u32SampleRate); //Set sample rate
	/* Set Datawidth */
	DPWM_SET_FIFODATAWIDTH(DPWM, DPWM_FIFO_DATAWIDTH_16BITS);
	
//	DPWM_ENABLE_FIFOTHRESHOLDINT(DPWM,8);//SW 0312 ??
		
	/* Lock protected registers */
    SYS_LockReg();
	/* GPIO multi-function.(GPD0:DPWM1_N,GPD1:DPWM1_P) */
	SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC13MFP_Msk|SYS_GPC_MFPH_PC12MFP_Msk))|(SYS_GPC_MFPH_PC13MFP_DPWM_LP|SYS_GPC_MFPH_PC12MFP_DPWM_LN);
	/* GPIO multi-function.(GPD5:DPWM0_N,GPD6:DPWM0_P) */
	SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC11MFP_Msk|SYS_GPC_MFPH_PC10MFP_Msk))|(SYS_GPC_MFPH_PC11MFP_DPWM_RP|SYS_GPC_MFPH_PC10MFP_DPWM_RN);

	DPWM_Start();

}



void PDMA_DPWM_Init(void/*S_BUFCTRL* psOutBufCtrl*/)
{
	uint32_t g_u32DMAConfig = 0;
//	BUFCTRL_CFG((&sOutBufCtrl),ai32OutBuf,sizeof(ai32OutBuf)/sizeof(uint32_t));//
	
	CLK_EnableModuleClock(PDMA_MODULE);
	SYS_ResetModule(PDMA_RST);

    /* Scatter-Gather descriptor table configuration in SRAM */
    g_u32DMAConfig = \
                     ( (NUONEEX_DECODE_SAMPLE_PER_FRAME*2-1) << PDMA_DSCT_CTL_TXCNT_Pos ) | /* Transfer count is 1 */ \
                     PDMA_WIDTH_16 |  /* Transfer width is 16 bits(one word) */ \
                     PDMA_SAR_INC |   /* Source increment size is fixed(no increment) */ \
                     PDMA_DAR_FIX |   /* Destination increment size is fixed(no increment) */ \
                     PDMA_REQ_SINGLE | /* Transfer type is burst transfer type */ \
                     /*PDMA_BURST_1 |*/   /* Burst size is 128. No effect in single transfer type */ \
                     PDMA_OP_SCATTER; /* Operation mode is scatter-gather mode */
	
	
    // SPK(TX) buffer description
    sPDMA_SPK[0].CTL = g_u32DMAConfig;//(((psOutBufCtrl->u16BufCount/2)-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_INC|PDMA_DAR_FIX|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
    sPDMA_SPK[0].SA = (uint32_t)&overlapMem.NuOneMemPool.i16OutBuf[0][0];//(psOutBufCtrl->pai32Buf[0]);
    sPDMA_SPK[0].DA = (uint32_t)(&DPWM->FIFO);
    sPDMA_SPK[0].NEXT = (uint32_t)&sPDMA_SPK[1] - (PDMA->SCATBA);
    sPDMA_SPK[1].CTL = g_u32DMAConfig;//(((psOutBufCtrl->u16BufCount/2)-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_INC|PDMA_DAR_FIX|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
    sPDMA_SPK[1].SA = (uint32_t)&overlapMem.NuOneMemPool.i16OutBuf[1][0];//(psOutBufCtrl->pai32Buf[psOutBufCtrl->u16BufCount/2]);
    sPDMA_SPK[1].DA = (uint32_t)(&DPWM->FIFO);
    sPDMA_SPK[1].NEXT = (uint32_t)&sPDMA_SPK[0] - (PDMA->SCATBA);	
	// Open PDMA channel
	PDMA_Open((1<<SPK_PDMA_CH));
	// Set TransMode
    PDMA_SetTransferMode(SPK_PDMA_CH, PDMA_DPWM_TX, TRUE, (uint32_t)&sPDMA_SPK[0]);
	// Enable interrupt
	PDMA_EnableInt(SPK_PDMA_CH,PDMA_INT_TRANS_DONE);	
	
	//test
	NVIC_EnableIRQ(PDMA_IRQn);	
 	//Config DPWM(Speaker) buffer control 
	//psSPK_BufCtrl = psOutBufCtrl;

	PDMA_Trigger(SPK_PDMA_CH);
}






S_AUDIO_CALLBACK const g_asAppCallBack[] =
{
	{
		ReadDataCallback,	// Read data callback
		NULL,						// Write data callback
		NULL,//SW 1125 need to check		UserEvent_ProcessCallback,	// User event procss callback
		NULL						// Read MIDI wavetable callback
	}
};



uint32_t ReadDataCallback(void *pu8Buf, uint32_t u32StartAddr, uint32_t u32Count)
{
#if defined (__APROM)
		memcpy(pu8Buf,(uint32_t*)u32StartAddr,u32Count);
#elif defined (__DATAFLASH)
#elif defined (__SPIFLASH)
		SPIFlash_Read(&g_sSPIFlash, u32StartAddr, pu8Buf, u32Count);
#endif
	return u32Count;
}

uint32_t UserEvent_ProcessCallback(uint16_t u16EventIndex, uint16_t u16EventSubIndex)
{
/*SW 1125 need to check	ULTRAIO_EVENTHANDLER(u16EventIndex,u16EventSubIndex);
*/	return 0;
} 

void duplicate(uint8_t u8idx)
{
	uint16_t i;
	for(i=NUONEEX_DECODE_SAMPLE_PER_FRAME-1;i>0;i--)
	{
		overlapMem.NuOneMemPool.i16OutBuf[u8idx][i*2]=overlapMem.NuOneMemPool.i16OutBuf[u8idx][i];
		overlapMem.NuOneMemPool.i16OutBuf[u8idx][i*2+1]=overlapMem.NuOneMemPool.i16OutBuf[u8idx][i];
	}
	overlapMem.NuOneMemPool.i16OutBuf[u8idx][1]=overlapMem.NuOneMemPool.i16OutBuf[u8idx][0];
	
}
	
	


BOOL PlayNuOnebyIdx(UINT8 u8PlayIdx)
{
	AudioRom_GetAudioChunkInfo( ReadDataCallback, AUDIOROM_STORAGE_START_ADDR, u8PlayIdx, &sAudioChunkInfo);

	if((u16SampleRate= NuOneEx_DecodeInitiate(	(UINT8*)overlapMem.NuOneMemPool.au32DecodeWorkBuf, (UINT8*)overlapMem.NuOneMemPool.u32NuOneExTempBuf, sAudioChunkInfo.u32AudioChunkAddr, ReadDataCallback))==0)
	{	
		//printf("NuOne Initiation Failed\n");
		return FALSE;
	}
	
	//printf("u16SampleRate: %d\n", u16SampleRate);

	bPCMPlaying=TRUE;
	PDMA1CallBackCount=0;
	bBufferEmpty=FALSE;

	if(!NuOneEx_DecodeIsEnd((UINT8*)overlapMem.NuOneMemPool.au32DecodeWorkBuf))
	{
		NuOneEx_DecodeProcess((UINT8*)overlapMem.NuOneMemPool.au32DecodeWorkBuf, (UINT8*)overlapMem.NuOneMemPool.u32NuOneExTempBuf,&overlapMem.NuOneMemPool.i16OutBuf[0][0],ReadDataCallback,UserEvent_ProcessCallback);
		duplicate(0);
	}

	DPWM_Init(u16SampleRate);//InitialDPWM(u16SampleRate);
	PDMA_DPWM_Init();//PdmaCtrl_Init();
	
	//PDMA1forDPWM((UINT32)&i16OutBuf[0][0]);

	if(!NuOneEx_DecodeIsEnd((UINT8*)overlapMem.NuOneMemPool.au32DecodeWorkBuf))
	{
		NuOneEx_DecodeProcess((UINT8*)overlapMem.NuOneMemPool.au32DecodeWorkBuf, (UINT8*)overlapMem.NuOneMemPool.u32NuOneExTempBuf,&overlapMem.NuOneMemPool.i16OutBuf[1][0],ReadDataCallback,UserEvent_ProcessCallback);
		duplicate(1);	
	}

	//printf("bPCMPlaying TRUE loop start bBufferEmpty[%d]\n", bBufferEmpty);
	while(bPCMPlaying)
		PlayLoop();
	
	return TRUE;
}


void PlayLoop(void)
{
	uint32_t g_u32DMAConfig = 0;	
	
	if(bBufferEmpty==TRUE)
	{
		if(!NuOneEx_DecodeIsEnd((UINT8*)overlapMem.NuOneMemPool.au32DecodeWorkBuf))
		{	
			if ((PDMA1CallBackCount&0x1)==1)
			{
				NuOneEx_DecodeProcess((UINT8*)overlapMem.NuOneMemPool.au32DecodeWorkBuf, (UINT8*)overlapMem.NuOneMemPool.u32NuOneExTempBuf,&overlapMem.NuOneMemPool.i16OutBuf[0][0],ReadDataCallback,UserEvent_ProcessCallback);		
				duplicate(0);			
			}
			else
			{	
				NuOneEx_DecodeProcess((UINT8*)overlapMem.NuOneMemPool.au32DecodeWorkBuf, (UINT8*)overlapMem.NuOneMemPool.u32NuOneExTempBuf,&overlapMem.NuOneMemPool.i16OutBuf[1][0],ReadDataCallback,UserEvent_ProcessCallback);			
				duplicate(1);			
			}
			bBufferEmpty=FALSE;
		}
		else
		{
			if(bLastOneBuffer==TRUE)
			{
				bPCMPlaying=FALSE;
				PlayClose();
			}
			else
			{	
				bLastOneBuffer=TRUE;
			}
		}		
	}
}




void DPWM_Start(void)
{
//	if( psSPK_BufCtrl != NULL )
//	{
		DPWM_ENABLE_DRIVER(DPWM);
		DPWM_START_PLAY(DPWM);
		DPWM_ENABLE_PDMA(DPWM);
//	}
}

void PDMA_CH_Close(uint32_t u32Mask)
{
  PDMA->CHCTL &= ~u32Mask;
//	NVIC_DisableIRQ(PDMA_IRQn);
}


void DPWMClose(void)
{
	DPWM_DISABLE_PDMA(DPWM);
	DPWM_STOP_PLAY(DPWM);
	DPWM_DISABLE_DRIVER(DPWM);	
}

void PlayClose(void)
{
	PDMA_CH_Close(1<<SPK_PDMA_CH);//PDMA_Close();
	DPWMClose();//DPWM_Close();
}

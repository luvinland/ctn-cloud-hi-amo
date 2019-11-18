
#include <stdio.h>
#include <string.h>
#include "Platform.h"
#include "BUFCTRL.h"

#define PLL_CLOCK           72000000

// Function prototype declaration
void System_Init(void);   
void DPWM_Init(void);                   
void I2C1_Init(void);                
void SPI2I2S_Init(void);                                       
void I2S0_Init(void);     
void SPI1I2S_Init(void);       
extern void Process(S_BUFCTRL* psInBufCtrl,S_BUFCTRL* psOutBufCtrl);
extern PATHMODE path;
void GPIO_Init(void);

S_BUFCTRL* g_psIrq_InBufCtrl = NULL;
S_BUFCTRL* g_psIrq_OutBufCtrl = NULL;

// Initiate Hardware IP(Include system clock).
void Hardware_Init(void) {
    // Initiate system clock.
    System_Init();
	// Initiate DPWM for 83P20
	DPWM_Init();
	// Initiate I2C1 for 88L25 & 85L40.
	I2C1_Init();
	// Initiate SPI2I2S for 88L25
	SPI2I2S_Init();
	// Initiate I2S0 for 85L40
	I2S0_Init();
	// Initiate SPI1I2S for MINIDSP
	SPI1I2S_Init();
	
	GPIO_Init(); 
}
// ===================================== DPWM ===================================
S_BUFCTRL* psDPWM_BufCtrl = NULL;

// DPWM Handler.
void DPWM_IRQHandler(void) {
	// Provide user to process data.
	if( g_psIrq_InBufCtrl != NULL && g_psIrq_OutBufCtrl != NULL ) {
		Process(g_psIrq_InBufCtrl,g_psIrq_OutBufCtrl);
	}
	if( psDPWM_BufCtrl != NULL )
	{
		if( BUFCTRL_IS_EMPTY(psDPWM_BufCtrl) ) {
			while((DPWM->STS&DPWM_STS_FULL_Msk)==0) {
				DPWM->DATA = 0;
			}
		} else {
			if((DPWM->STS&DPWM_STS_FULL_Msk)==0) {
				BUFCTRL_READ(psDPWM_BufCtrl,&(DPWM->DATA));
			}
		}
	}
}
// Initiate DPWM.
void DPWM_Init(void) {
    // Unlock protected registers
    SYS_UnlockReg();
    // Enable DPWM clock
    CLK->APBCLK1 |= CLK_APBCLK1_DPWMCKEN_Msk;	
    // Select DPWM CLK source PCLK	(OSC5)
    CLK->CLKSEL2 = (CLK->CLKSEL2&~(CLK_CLKSEL2_DPWMSEL_Msk)) | (0x2<<12);
	// DPWM IPReset
	SYS->IPRST2 |= (1<<6);//SYS_IPRST2_DPWMRST_Msk;
    SYS->IPRST2 &= ~(1<<6);//SYS_IPRST2_DPWMRST_Msk;	
    //HIRC=48MHz,Fs=24.576MHz/(128x4)=48kHz 
    DPWM->ZOHDIV = (DPWM->ZOHDIV&(~DPWM_ZOHDIV_ZOHDIV_Msk)&(~DPWM_ZOHDIV_DPWM_CLK_DIV_Msk))|(0x2<<DPWM_ZOHDIV_DPWM_CLK_DIV_Pos)|(0x4<<DPWM_ZOHDIV_ZOHDIV_Pos);
    // HPF
    DPWM->COEFF_0 = 0x1d2b9f; //b0_1
    DPWM->COEFF_1 = 0xc5a8c1; //b1_1
    DPWM->COEFF_2 = 0x1d2b9f; //b2_1
    DPWM->COEFF_3 = 0xc5e8ea; //a1_1
    DPWM->COEFF_4 = 0x1a9768; //a2_1	
    DPWM->COEFF_5 = 0x1d2b9f; //b0_2
    DPWM->COEFF_6 = 0xc5a8c1; //b1_2
    DPWM->COEFF_7 = 0x1d2b9f; //b2_2
    DPWM->COEFF_8 = 0xc5e8ea; //a1_2
    DPWM->COEFF_9 = 0x1a9768; //a2_2	
    // BIQ_COEF_CTL set 0
	DPWM->COEFF_CTL &= (~DPWM_COEFF_CTL_PRGCOEFF_Msk) ; 
    // (0x801)Enable threshold int and Data width=PDMA 16Bits
    DPWM->CTL |= ((DPWM_CTL_RXTHIE_Msk) | (0x8<<DPWM_CTL_RXTH_Pos) | (0x3<<DPWM_CTL_FIFOWIDTH_Pos) ); 
/*    DPWM->CTL |= ((DPWM_CTL_RXTHIE_Msk) | (0x8<<DPWM_CTL_RXTH_Pos) );//| (0x3<<DPWM_CTL_FIFOWIDTH_Pos) ); 
		DPWM->CTL &= ~DPWM_CTL_FIFOWIDTH_Msk;
*/		
		
    // Enable NVIC
	NVIC_EnableIRQ(DPWM_IRQn);
	// Lock protected registers 
    SYS_LockReg();
	// GPIO multi-function.(GPD0:DPWM1_N,GPD1:DPWM1_P)
	SYS->GPD_MFPL = (SYS->GPD_MFPL & ~(SYS_GPD_MFPL_PD0MFP_Msk|SYS_GPD_MFPL_PD1MFP_Msk))|(SYS_GPD_MFPL_PD0MFP_DPWM1_N|SYS_GPD_MFPL_PD1MFP_DPWM1_P);
}
// Start DPWM
void DPWM_Start(S_BUFCTRL* psBufCtrl) {
	if( psBufCtrl != NULL ) {
		psDPWM_BufCtrl = psBufCtrl;
		g_psIrq_OutBufCtrl = psBufCtrl;
		DPWM->CTL |= ((DPWM_CTL_DWPMDRVEN_Msk)|(DPWM_CTL_DPWMEN_Msk));
	}
}
// Stop DPWM
void DPWM_Stop(void) {
	DPWM->CTL &= ~((DPWM_CTL_DWPMDRVEN_Msk)|(DPWM_CTL_DPWMEN_Msk));
	psDPWM_BufCtrl = NULL;
}
// ============================== SPI2 I2S ================================
S_BUFCTRL* psSPI2I2S_InBufCtrl = NULL;
S_BUFCTRL* psSPI2I2S_OutBufCtrl = NULL;

// SPI2I2S IRQHandler.
void SPI2_IRQHandler() {
	
//if (SPI2->I2SSTS & SPI_I2SSTS_TXFULL_Msk)	
	
	// SPI2I2S RX process.
		while( (SPI2->I2SSTS&SPI_I2SSTS_RXEMPTY_Msk) == 0 ) {
			if( psSPI2I2S_InBufCtrl != NULL && !BUFCTRL_IS_FULL(psSPI2I2S_InBufCtrl) ) {
				BUFCTRL_WRITE(psSPI2I2S_InBufCtrl,SPI2->RX);
			} else {
				SPI2->RX;
			}
		}

		// Provide user to process data.
//	if( g_psIrq_InBufCtrl != NULL && g_psIrq_OutBufCtrl != NULL ) {
//		Process(g_psIrq_InBufCtrl,g_psIrq_OutBufCtrl);
//	}
		
	// SPI2I2S TX process.
	while( psSPI2I2S_OutBufCtrl != NULL && !BUFCTRL_IS_EMPTY(psSPI2I2S_OutBufCtrl) ) {
		if( (SPI2->I2SSTS&SPI_I2SSTS_TXFULL_Msk) == 0) {
			BUFCTRL_READ(psSPI2I2S_OutBufCtrl,&(SPI2->TX));
		} else {
			break;
		}
	}
}

// Initiate SPI2I2S.
void SPI2I2S_Init(void) {
    // Unlock protected registers
    SYS_UnlockReg();
	// Enable clock module
	CLK_EnableModuleClock(SPI2_MODULE);
	// Set module clock from HXT
	CLK_SetModuleClock(SPI2_MODULE,CLK_CLKSEL2_SPI2SEL_HXT,0);
	// Reset SPI2.
	SYS_ResetModule(SPI2_RST);
    // OSC5=24.576MHz, 48k/32bits , MCLK=12.288MHz , BCLK=3.072MHz	,2channels
//    SPI2->I2SCLK |= ( (0x1<<0) | (0xB<<8));
		SPI2->I2SCLK = ( (0x1<<0) | (0xB<<8));
		// Enable MCLK 
		SPI2->I2SCTL |= (0x1<<15);
			
    // Master mode	
    SPI2->I2SCTL &= ~(0x1<<8);						
		// slave mode
//	    SPI2->I2SCTL |= (0x1<<8);						
    // 32bits data channel	
	  SPI2->I2SCTL |= (0x3<<4);	
    // Stereo
    SPI2->I2SCTL &= ~(0x1<<6);		
    // format to I2S mode.
		SPI2->I2SCTL &= ~(0x3<<SPI_I2SCTL_FORMAT_Pos);	
		SPI2->I2SCTL |= (0x1<<SPI_I2SCTL_FORMAT_Pos);	
    // Tx threshold = 6
    SPI2->FIFOCTL &= ~(0x7<<28);
    SPI2->FIFOCTL |= (0x6<<28);     		
    // Rx threshold = 6	
    SPI2->FIFOCTL &= ~(0x7<<24);
		SPI2->FIFOCTL |= (0x6<<24); 
		// enable both Tx and Rx threshold
//    SPI2->FIFOCTL |= (0x3<<2);     			 	   	 
		SPI2->FIFOCTL &= ~(0x3<<2);     			 	   	    
    SPI2->FIFOCTL |= (0x3<<2);     			 	   	    
    // Enable SPI2I2S control	
    SPI2->I2SCTL |= (1<<0);         
		// Tx and Rx FIFO buffer clear
		SPI2->FIFOCTL |= (0x3<<8);								
		// Enable SPI2's NVIC	
    NVIC_EnableIRQ(SPI2_IRQn);
		//	NVIC_DisableIRQ(SPI2_IRQn);
		// Lock protected registers
    SYS_LockReg();
		// GPIO multi-function.(GPA7:SPI2I2S_DI,GPA8:SPI2I2S_DO)
		SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA7MFP_Msk))|(SYS_GPA_MFPL_PA7MFP_SPI2I2S_DI);
		SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA8MFP_Msk|SYS_GPA_MFPH_PA9MFP_Msk|SYS_GPA_MFPH_PA10MFP_Msk|SYS_GPA_MFPH_PA12MFP_Msk))|(SYS_GPA_MFPH_PA8MFP_SPI2I2S_DO|SYS_GPA_MFPH_PA9MFP_SPI2I2S_LRCK|SYS_GPA_MFPH_PA10MFP_SPI2I2S_BCLK|SYS_GPA_MFPH_PA12MFP_SPI2I2S_MCLK);
}

// Start SPI2I2S.
void SPI2I2S_Start(S_BUFCTRL* psInBufCtrl,S_BUFCTRL* psOutBufCtrl, RXTXMODE RxTx) {
	if( psInBufCtrl != NULL ) {
		psSPI2I2S_InBufCtrl = psInBufCtrl;
		g_psIrq_InBufCtrl = 	psInBufCtrl;
	}
	if( psOutBufCtrl != NULL ) {
		psSPI2I2S_OutBufCtrl = psOutBufCtrl;	
		g_psIrq_OutBufCtrl = psOutBufCtrl;
		while( psSPI2I2S_OutBufCtrl != NULL && (SPI2->I2SSTS&SPI_I2SSTS_TXFULL_Msk) == 0 && !BUFCTRL_IS_EMPTY(psSPI2I2S_OutBufCtrl) ) {
			BUFCTRL_READ(psSPI2I2S_OutBufCtrl,&(SPI2->TX));
		}
	}
 
	SPI2->I2SCTL |= (SPI_I2SCTL_TXEN_Msk|SPI_I2SCTL_RXEN_Msk); 	
//	SPI2->I2SCTL &= ~(SPI_I2SCTL_TXEN_Msk|SPI_I2SCTL_RXEN_Msk);
//	SPI1->I2SCTL |= RxTx <<1;   
}

// Stop SPI2I2S.
void SPI2I2S_Stop(void) {		
    SPI2->I2SCTL &= ~(SPI_I2SCTL_TXEN_Msk|SPI_I2SCTL_RXEN_Msk);   
	psSPI2I2S_InBufCtrl = 0;
	psSPI2I2S_OutBufCtrl = 0;
}

// ==================================    SPI1 I2S ===================================
S_BUFCTRL* psSPI1I2S_InBufCtrl = NULL;
S_BUFCTRL* psSPI1I2S_OutBufCtrl = NULL;

// SPI1I2S IRQHandler.
void SPI1_IRQHandler() {
		uint32_t u32Tmp1,u32Tmp2;
	// SPI1I2S RX process.
	
	if (path == 	PATH_MINIDSP_TO_88L25){
		while( (SPI1->I2SSTS&SPI_I2SSTS_RXEMPTY_Msk) == 0 ) // when RX FIFO is not empty
		{
			if( psSPI1I2S_InBufCtrl != NULL && !BUFCTRL_IS_FULL(psSPI1I2S_InBufCtrl) ) // if receiving FIFO is not full;
			{
				BUFCTRL_WRITE(psSPI1I2S_InBufCtrl,SPI1->RX);
				BUFCTRL_READ(psSPI1I2S_InBufCtrl,&u32Tmp1);
				BUFCTRL_WRITE(g_psIrq_OutBufCtrl,u32Tmp1);
				BUFCTRL_READ(g_psIrq_OutBufCtrl,&(SPI2->TX));
			} else {
				SPI1->RX;
			}
		}
		return; 
	}
	else if (path == 	PATH_MINIDSP_TO_83P20){
		while( (SPI1->I2SSTS&SPI_I2SSTS_RXEMPTY_Msk) == 0 ) // when RX FIFO is not empty
		{
			BUFCTRL_WRITE(psSPI1I2S_InBufCtrl,SPI1->RX);
		}
		while( !BUFCTRL_IS_EMPTY(psSPI1I2S_InBufCtrl) && !BUFCTRL_IS_FULL(g_psIrq_OutBufCtrl) )
		{		
				if (BUFCTRL_GET_COUNT(psSPI1I2S_InBufCtrl)<2 )
					break;
				else
				{
					// 4 channel mixer to 2 channel.
					BUFCTRL_READ(psSPI1I2S_InBufCtrl,&u32Tmp1);
					BUFCTRL_READ(psSPI1I2S_InBufCtrl,&u32Tmp2);
					u32Tmp1 += u32Tmp2;
					u32Tmp1 >>= 3; // 24bits I2S data to 16bits PCM?
					BUFCTRL_WRITE(g_psIrq_OutBufCtrl,u32Tmp1);
				}
			}					
	} else {
		while( (SPI1->I2SSTS&SPI_I2SSTS_RXEMPTY_Msk) == 0 ) // when RX FIFO is not empty
		{
			if( psSPI1I2S_InBufCtrl != NULL && !BUFCTRL_IS_FULL(psSPI1I2S_InBufCtrl) ) // if receiving FIFO is not full;
			{
				BUFCTRL_WRITE(psSPI1I2S_InBufCtrl,SPI1->RX);
			} else {
				SPI1->RX;
			}
		}
		// Provide user to process data.
//		if( g_psIrq_InBufCtrl != NULL && g_psIrq_OutBufCtrl != NULL && path == PATH_MINIDSP_TO_88L25) {
	//		Process(g_psIrq_InBufCtrl,g_psIrq_OutBufCtrl);
//			u32Tmp1 = BUFCTRL_IS_EMPTY(g_psIrq_InBufCtrl);
//			u32Tmp2 = BUFCTRL_IS_FULL(g_psIrq_OutBufCtrl);
//			while( !BUFCTRL_IS_EMPTY(g_psIrq_InBufCtrl) && !BUFCTRL_IS_FULL(g_psIrq_OutBufCtrl) ) {
					
//						BUFCTRL_READ(g_psIrq_InBufCtrl,&u32Tmp1);
						//u32Tmp1 >>=8;
//						BUFCTRL_WRITE(g_psIrq_OutBufCtrl,u32Tmp1);
//			}
//		}

		if (path == PATH_85L40_TO_MINIDSP) {
			while( psSPI1I2S_OutBufCtrl != NULL && !BUFCTRL_IS_EMPTY(psSPI1I2S_OutBufCtrl) ) {
				if( (SPI1->I2SSTS&SPI_I2SSTS_TXFULL_Msk) == 0) { // if TX FIFO is not full 
					BUFCTRL_READ(psSPI1I2S_OutBufCtrl,&(SPI1->TX)); 
				} else {
					break;
				}
			}
		}		
	}
}

// Initiate SPI1I2S.
void SPI1I2S_Init(void) {
    // Unlock protected registers
    SYS_UnlockReg();
	// Enable clock module
	CLK_EnableModuleClock(SPI1_MODULE);
	// Set module clock from HXT
	CLK_SetModuleClock(SPI1_MODULE,CLK_CLKSEL2_SPI1SEL_HXT,0);
	// Reset SPI1.
	SYS_ResetModule(SPI1_RST);
    // OSC5=24.576MHz, 48k/32bits , MCLK=12.288MHz , BCLK=3.072MHz	,2channels
    SPI1->I2SCLK = ( (0x1<<0) | (0xB<<8));
    
		// Enable MCLK
    SPI1->I2SCTL |= (0x1<<15);
		//disable MCLK
		//SPI1->I2SCTL &= ~(0x1<<15);
		
		// Master mode
    SPI1->I2SCTL &= ~(0x1<<8);
		//slave mode
//		SPI1->I2SCTL |= (0x1<<8);

		// 32bits data channel
    SPI1->I2SCTL |= (0x3<<4);
    // Stereo
    SPI1->I2SCTL &= ~(0x1<<6);
    // format to I2S mode.
		SPI1->I2SCTL &= ~(0x3<<SPI_I2SCTL_FORMAT_Pos);
		SPI1->I2SCTL |= (0x1<<SPI_I2SCTL_FORMAT_Pos);
    // Tx threshold = 6
		SPI1->FIFOCTL &= ~(0x7<<28);
    SPI1->FIFOCTL |= (0x6<<28);
    // Rx threshold = 6
		SPI1->FIFOCTL &= ~(0x7<<24);
    SPI1->FIFOCTL |= (0x6<<24);
 
		// enable Rx threshold   			 	   	 
		SPI1->FIFOCTL &= ~(0x3<<2);     			 	   	    
    SPI1->FIFOCTL |= (0x1<<2);   
 
    // Enable SPI1I2S control
    SPI1->I2SCTL |= (1<<0);
		// Tx and Rx FIFO buffer clear
		SPI1->FIFOCTL |= (0x3<<8);
		// Enable SPI1's NVIC
    NVIC_EnableIRQ(SPI1_IRQn);
		// Lock protected registers
    SYS_LockReg();
		// GPIO multi-function.(GPC0:SPI1I2S_DO,GPC1:SPI1I2S_DI,GPC2:SPI1I2S_BCLK,GPC3:SPI1I2S_LRCK,GPC4:SPI1I2S_MCLK)
		SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC0MFP_Msk|SYS_GPC_MFPL_PC1MFP_Msk|SYS_GPC_MFPL_PC2MFP_Msk|SYS_GPC_MFPL_PC3MFP_Msk|SYS_GPC_MFPL_PC4MFP_Msk))|(SYS_GPC_MFPL_PC0MFP_SPI1I2S_DO|SYS_GPC_MFPL_PC1MFP_SPI1I2S_DI|SYS_GPC_MFPL_PC2MFP_SPI1I2S_BCLK|SYS_GPC_MFPL_PC3MFP_SPI1I2S_LRLK|SYS_GPC_MFPL_PC4MFP_SPI1I2S_MCLK);

}
// Start SPI1I2S.
void SPI1I2S_Start(S_BUFCTRL* psInBufCtrl,S_BUFCTRL* psOutBufCtrl, RXTXMODE RxTx) {
	if( psInBufCtrl != NULL ) {
		psSPI1I2S_InBufCtrl = psInBufCtrl;
		g_psIrq_InBufCtrl = psInBufCtrl;
	}
	if( psOutBufCtrl != NULL ) {
		psSPI1I2S_OutBufCtrl = psOutBufCtrl;	
		g_psIrq_OutBufCtrl = psOutBufCtrl;
		while( psSPI1I2S_OutBufCtrl != NULL && (SPI1->I2SSTS&SPI_I2SSTS_TXFULL_Msk) == 0 && !BUFCTRL_IS_EMPTY(psSPI1I2S_OutBufCtrl) ) {
			BUFCTRL_READ(psSPI1I2S_OutBufCtrl,&(SPI1->TX));
		}
	}
	
	SPI1->I2SCTL &= ~(SPI_I2SCTL_TXEN_Msk|SPI_I2SCTL_RXEN_Msk);
	SPI1->I2SCTL |= RxTx <<1;   
}
// Stop SPI1I2S.
void SPI1I2S_Stop(void) {		
    SPI1->I2SCTL &= ~(SPI_I2SCTL_TXEN_Msk|SPI_I2SCTL_RXEN_Msk);   
	psSPI1I2S_InBufCtrl = 0;
	psSPI1I2S_OutBufCtrl = 0;
}

// =================================== I2S0 =====================================
S_BUFCTRL* psI2S0_BufCtrl = NULL;

// I2S0 IRQHandler.
void I2S0_IRQHandler() {
	while( (I2S0->STATUS0&I2S_STATUS0_RXEMPTY_Msk) == 0 ) {
		if( psI2S0_BufCtrl != NULL && !BUFCTRL_IS_FULL(psI2S0_BufCtrl) ) {
			BUFCTRL_WRITE(psI2S0_BufCtrl,I2S0->RXFIFO);
		} else {
			I2S0->RXFIFO;
		}
	}
	// Provide user to process data.
	if( g_psIrq_InBufCtrl != NULL && g_psIrq_OutBufCtrl != NULL ) {
		Process(g_psIrq_InBufCtrl,g_psIrq_OutBufCtrl);
	}
}
// Initiate I2S0.
void I2S0_Init(void) {
    // Unlock protected registers
    SYS_UnlockReg();
    // Enable I2S clock 
    CLK->APBCLK0 |= CLK_APBCLK0_I2S0CKEN_Msk;	
    // Select I2S clock
    CLK->CLKSEL3 = (CLK->CLKSEL3 & ~0x30000) | (0x0<<16);				
    // I2S IPReset
    SYS->IPRST1 |= SYS_IPRST1_I2S0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_I2S0RST_Msk;
    // OSC5=24.576MHz, 48k/24bits , MCLK=12.288MHz , BCLK=6.144MHz	,4channels
	I2S0->CLKDIV |= (0x1<<0) | (0x5<<8);		// 05<<8 makes the FS 16KHz????
//		I2S0->CLKDIV |= (0x1<<0) | (0x01<<8);		
	// Enable MCLK 
    I2S0->CTL0 |= (0x1<<15);								
    // Master mode
    I2S0->CTL0 |= (0x0<<8);					
    // 4 channel TDM
    I2S0->CTL0 |= (0x1<<30);			
    // 32bits audio channel
	I2S0->CTL0 |= (0x3<<28);								
    // 32bits data channel
    I2S0->CTL0 |= (0x3<<4);							
    // PCMSYNC=one bclk 
    I2S0->CTL0 |= (0<<27);
    // PCM-MSB format
    I2S0->CTL0 |= (0x5<<24);
    // Receive Channel1 data in Mono mode
    I2S0->CTL0 |= (0<<23);
    // Order bit, even channel data at high byte(LSB)
    //I2S0->CTL0 |= (0<<7);
    // Stereo
    I2S0->CTL0 |= (0<<6);
    // Tx threshold = 8
    I2S0->CTL1 |= (0x8<<8);
    // Rx threshold = 8
    I2S0->CTL1 |= (0x8<<16);
    // enable TX underflow/threshold/overflow interrupt
    I2S0->IEN  &= ~(0x7<<8);
    // enable RX underflow/threshold/overflow interrupt
    I2S0->IEN  &= ~(0x7<<0);
    // enable I2S control	
    I2S0->CTL0 |= (1<<0);             	 	
    // Tx and Rx FIFO buffer clear
    I2S0->CTL0 |= (0x3<<18);
	// Enable I2S's NVIC
//    NVIC_EnableIRQ(I2S0_IRQn);
    // Lock protected registers
    SYS_LockReg();
	// GPIO multi-function.(GPA7:SPI1I2S_DI,GPA8:SPI2I2S_DO)
	SYS->GPD_MFPL = (SYS->GPD_MFPL & ~(SYS_GPD_MFPL_PD2MFP_Msk|SYS_GPD_MFPL_PD3MFP_Msk|SYS_GPD_MFPL_PD4MFP_Msk|SYS_GPD_MFPL_PD5MFP_Msk|SYS_GPD_MFPL_PD6MFP_Msk)) | (SYS_GPD_MFPL_PD2MFP_I2S0_MCLK|SYS_GPD_MFPL_PD3MFP_I2S0_LRCK|SYS_GPD_MFPL_PD4MFP_I2S0_DI|SYS_GPD_MFPL_PD5MFP_I2S0_DO|SYS_GPD_MFPL_PD6MFP_I2S0_BCLK);
}

// Start I2S0
void I2S0_Start(S_BUFCTRL* psBufCtrl) {
/*	if( psBufCtrl != NULL ) {
		psI2S0_BufCtrl = psBufCtrl;	
		g_psIrq_InBufCtrl = psBufCtrl;
		I2S0->CTL0 |= I2S_CTL0_RXEN_Msk;
	}
*/
		I2S0->CTL0 |= I2S_CTL0_RXEN_Msk;
}

// Stop I2S0
void I2S0_Stop(void) {
    I2S0->CTL0 &= ~I2S_CTL0_RXEN_Msk;
}
// I2C ===============================================================================
typedef struct {
	uint8_t  u8DeviceAddr;
	uint16_t u16Counter;
	uint16_t u16MaxCount;
	uint8_t* pau8Cmd;
} S_I2CCTRL;

typedef struct {
	uint8_t  u8Reg[2];
	uint8_t  u8Value[2];
} S_I2CPCMD;

#define I2C1_DEVICEADDR_88L25   (0x1A)
#define I2C1_DEVICEADDR_85L40   (0x1C)
#define I2C1_FREQ_88L25         (400000)
#define I2C1_FREQ_85L40         (100000)

S_I2CPCMD const asI2C1_Cmd_88L25[] = {
{	0x00	,	0x03	,	0x00	,	0xA0	}	,		// Clock_ADC_SRC, Clock_DAC_SRC
{	0x00	,	0x04	,	0x00	,	0x01	}	,		// FLL Ratio
{	0x00	,	0x05	,	0x31	,	0x26	}	,		// FLL
{	0x00	,	0x06	,	0xF0	,	0x10	}	,		// FLL
{	0x00	,	0x07	,	0x00	,	0x00	}	,		// FLL
{	0x00	,	0x08	,	0x10	,	0x00	}	,		// FLL
{	0x00	,	0x09	,	0x60	,	0x00	}	,		// FLL
{	0x00	,	0x0A	,	0xF1	,	0x3C	}	,		// FLL
{	0x00	,	0x0C	,	0x00	,	0x08	}	,		// HSD_CTRL: Manual Access JKR2
{	0x00	,	0x0D	,	0x00	,	0x00	}	,		// JACK_DET_CTRL
{	0x00	,	0x0F	,	0x00	,	0x00	}	,		// INTRERUPT_MASK
{	0x00	,	0x10	,	0x00	,	0x00	}	,		// IRQ_STATUS
{	0x00	,	0x11	,	0x00	,	0x00	}	,		// INT_CLR_KEY_STATUS
{	0x00	,	0x12	,	0xFF	,	0xFF	}	,		// INT_DIS_CTRL
{	0x00	,	0x13	,	0x00	,	0x15	}	,		// SAR_CTRL
{	0x00	,	0x14	,	0x01	,	0x10	}	,		// KEYDET_CTRL
{	0x00	,	0x15	,	0x00	,	0x00	}	,		// VDET_THRESHOLD_1
{	0x00	,	0x16	,	0x00	,	0x00	}	,		// VDET_THRESHOLD_2
{	0x00	,	0x17	,	0x00	,	0x00	}	,		// VDET_THRESHOLD_3
{	0x00	,	0x18	,	0x00	,	0x00	}	,		// VDET_THRESHOLD_4
{	0x00	,	0x19	,	0x00	,	0x00	}	,		// GPIO34_CTRL
{	0x00	,	0x1A	,	0x00	,	0x00	}	,		// GPIO12_CTRL
{	0x00	,	0x1B	,	0x00	,	0x00	}	,		// TDM_CTRL : disable TDM
{	0x00	,	0x1C	,	0x00	,	0x0A	}	,		// I2S_PCM_CTRL1: 24bit word length, standard I2S format
{	0x00	,	0x1D	,	0x20	,	0x11	}	,		// I2S_PCM_CTRl2: xxxxxxxx 
{	0x00	,	0x1E	,	0x00	,	0x00	}	,		// LEFT_TIME_SLOT
{	0x00	,	0x1F	,	0x00	,	0x00	}	,		// RIGHT_TIME_SLOT
{	0x00	,	0x20	,	0x00	,	0x00	}	,		// BIQ_CTRL
{	0x00	,	0x21	,	0x00	,	0x00	}	,		// BIQ_COF
{	0x00	,	0x22	,	0x00	,	0x00	}	,		// BIQ_COF
{	0x00	,	0x23	,	0x00	,	0x00	}	,		// BIQ_COF
{	0x00	,	0x24	,	0x00	,	0x00	}	,		// BIQ_COF
{	0x00	,	0x25	,	0x00	,	0x00	}	,		// BIQ_COF
{	0x00	,	0x26	,	0x00	,	0x00	}	,		// BIQ_COF
{	0x00	,	0x27	,	0x00	,	0x00	}	,		// BIQ_COF
{	0x00	,	0x28	,	0x00	,	0x00	}	,		// BIQ_COF
{	0x00	,	0x29	,	0x00	,	0x00	}	,		// BIQ_COF
{	0x00	,	0x2A	,	0x00	,	0x00	}	,		// BIQ_COF
{	0x00	,	0x2B	,	0x00	,	0x02	}	,		// ADC_RATE: down 128
{	0x00	,	0x2C	,	0x00	,	0x80	}	,		// DAC_CTRL1 
{	0x00	,	0x2D	,	0x00	,	0x00	}	,		// DAC_CTRL2
{	0x00	,	0x30	,	0x00	,	0xCF	}	,		// ADC_DGAIN_CTRL: 
{	0x00	,	0x31	,	0x08	,	0x00	}	,		// MUTE_CTRL
{	0x00	,	0x32	,	0xC0	,	0x00	}	,		// HSVOL_CTRL
{	0x00	,	0x33	,	0x00	,	0xCF	}	,		// DACL_CTRL
{	0x00	,	0x34	,	0x02	,	0xCF	}	,		// DACR_CTRL
{	0x00	,	0x38	,	0x14	,	0x86	}	,		// ADC_DRC_KNEE_IP12
{	0x00	,	0x39	,	0x0F	,	0x12	}	,		// ADC_DRC_KNEE_IP34
{	0x00	,	0x3A	,	0x25	,	0xFF	}	,		// ADC_DRC_SLOPES
{	0x00	,	0x3B	,	0x34	,	0x57	}	,		// ADC_DRC_ATKDCY
{	0x00	,	0x45	,	0x14	,	0x86	}	,		// DAC_DRC_KNEE_IP12
{	0x00	,	0x46	,	0x0F	,	0x12	}	,		// DAC_DRC_KNEE_IP34
{	0x00	,	0x47	,	0x25	,	0xF9	}	,		// DAC_DRC_SLOPES
{	0x00	,	0x48	,	0x34	,	0x57	}	,		// DAC_DRC_ATKDCY
{	0x00	,	0x4C	,	0x00	,	0x00	}	,		// MODE_CTRL
{	0x00	,	0x4D	,	0x00	,	0x00	}	,		// ????????????? 
{	0x00	,	0x4E	,	0x00	,	0x00	}	,		// ?????????????
{	0x00	,	0x50	,	0x20	,	0x07	}	,		// CLASSG_CTRL
{	0x00	,	0x51	,	0x00	,	0x00	}	,		// OPT_EFUSE_CTRL
{	0x00	,	0x53	,	0xC6	,	0x13	}	,		// ???????????????
{	0x00	,	0x54	,	0x07	,	0x18	}	,		// ???????????????
{	0x00	,	0x55	,	0x00	,	0x00	}	,		// MISC_CTRL
{	0x00	,	0x58	,	0x1A	,	0x14	}	,
{	0x00	,	0x59	,	0x00	,	0xFF	}	,
{	0x00	,	0x66	,	0x00	,	0x60	}	,
{	0x00	,	0x68	,	0x00	,	0x00	}	,
{	0x00	,	0x69	,	0x00	,	0x00	}	,
{	0x00	,	0x6A	,	0x10	,	0x03	}	,
{	0x00	,	0x71	,	0x00	,	0x11	}	,
{	0x00	,	0x72	,	0x00	,	0x60	}	,
{	0x00	,	0x73	,	0x33	,	0x2C	}	,
{	0x00	,	0x74	,	0x41	,	0x06	}	,
{	0x00	,	0x76	,	0x31	,	0x40	}	,
{	0x00	,	0x77	,	0x00	,	0x00	}	,
{	0x00	,	0x7F	,	0x41	,	0x3F	}	,
{	0x00	,	0x80	,	0x64	,	0xA0	}	,
{	0x00	,	0x01	,	0x07	,	0xFF	}	
};
S_I2CPCMD const asI2C1_Cmd_85L40[] = {
{	0x00	,	0x03	,	0x00	,	0x40	}	,
{	0x00	,	0x04	,	0x00	,	0x01	}	,
{	0x00	,	0x05	,	0x31	,	0x26	}	,
{	0x00	,	0x06	,	0x00	,	0x08	}	,
{	0x00	,	0x07	,	0x00	,	0x10	}	,
{	0x00	,	0x08	,	0xC0	,	0x00	}	,
{	0x00	,	0x09	,	0xE0	,	0x00	}	,
{	0x00	,	0x0A	,	0xF1	,	0x3C	}	,
//{	0x00	,	0x10	,	0x00	,	0x0F	}	,	// PCMA, 32 bit daat length
{	0x00	,	0x10	,	0x00	,	0x4F	}	,	// PCMB, 32 bit daat length
{	0x00	,	0x11	,	0x00	,	0x00	}	, // 
{	0x00	,	0x12	,	0x00	,	0x00	}	, // 
{	0x00	,	0x13	,	0x00	,	0x00	}	,
{	0x00	,	0x14	,	0xC0	,	0x0F	}	, // C00F
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
{	0x00	,	0x3A	,	0x40	,	0x62	}	,//4002
{	0x00	,	0x40	,	0x04	,	0x00	}	,
{	0x00	,	0x41	,	0x04	,	0x00	}	,
{	0x00	,	0x42	,	0x04	,	0x00	}	,
{	0x00	,	0x43	,	0x04	,	0x00	}	,
{	0x00	,	0x44	,	0x00	,	0xE4	}	,//00E4
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

{	0x00	,	0x6B	,	0x01	,	0x01	}	,
{	0x00	,	0x6C	,	0x01	,	0x01	}	,
//{	0x00	,	0x6B	,	0x16	,	0x16	}	,
//{	0x00	,	0x6C	,	0x16	,	0x16	}	,

{	0x00	,	0x6D	,	0xF0	,	0x00	}	,
{	0x00	,	0x01	,	0x00	,	0x0F	}	,  //000F
{	0x00	,	0x02	,	0x80	,	0x03	}	
};

S_I2CCTRL sI2C1_Ctrl;

// I2C1 Handler.
void I2C1_IRQHandler() {
    if(I2C_GET_TIMEOUT_FLAG(I2C1)) {
        I2C_ClearTimeoutFlag(I2C1);
    } else {
		switch(I2C_GET_STATUS(I2C1)) {
			// START has been transmitted and Write SLA+W to Register I2CDAT
			case 0x08:
				I2C_SET_DATA(I2C1, sI2C1_Ctrl.u8DeviceAddr << 1);    
				I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);			
				break;
			// SLA+W has been transmitted and ACK has been received
			case 0x18:
				I2C_SET_DATA(I2C1, sI2C1_Ctrl.pau8Cmd[sI2C1_Ctrl.u16Counter++]);
				I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);		
				break;
			// SLA+W has been transmitted and NACK has been received
			case 0x20:
				I2C_STOP(I2C1);
				I2C_START(I2C1);	
				sI2C1_Ctrl.u16MaxCount = 0;
				break;
			// DATA has been transmitted and ACK has been received
			case 0x28:
				if(sI2C1_Ctrl.u16Counter < sI2C1_Ctrl.u16MaxCount) {
					I2C_SET_DATA(I2C1, sI2C1_Ctrl.pau8Cmd[sI2C1_Ctrl.u16Counter++]);
					I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
				} else {
					I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
					sI2C1_Ctrl.u16MaxCount = 0;
				}
				break;
		}
	}
}
// Initiate I2C1.
void I2C1_Init(void) {
    // Unlock protected registers
    SYS_UnlockReg();
    // Reset module
	SYS_ResetModule(I2C1_RST);
    // Enable I2C0 module clock 
    CLK_EnableModuleClock(I2C1_MODULE);
    // Open I2C module and set bus clock 
    I2C_Open(I2C1, I2C1_FREQ_88L25);
    // Enable I2C interrupt
    I2C_EnableInt(I2C1);
    NVIC_EnableIRQ(I2C1_IRQn);
    // Lock protected registers 
    SYS_LockReg();	
	// GPIO multi-function.(GPD14:I2C1_SCL,GPD15:I2C1_SDA)
	SYS->GPD_MFPH = (SYS->GPD_MFPH & ~(SYS_GPD_MFPH_PD14MFP_Msk|SYS_GPD_MFPH_PD15MFP_Msk))|(SYS_GPD_MFPH_PD14MFP_I2C1_SCL|SYS_GPD_MFPH_PD15MFP_I2C1_SDA);
}
// I2C1 & configuration(block-in).
void I2C1_Command(uint8_t u8ToDevice) {
	uint16_t u16i;
	switch(u8ToDevice) {
		case 0: // to 88L25
			I2C_SetBusClockFreq(I2C1,I2C1_FREQ_88L25);
			sI2C1_Ctrl.u8DeviceAddr = I2C1_DEVICEADDR_88L25;
			for(u16i=0;u16i<sizeof(asI2C1_Cmd_88L25)/sizeof(S_I2CPCMD);u16i++) {
				sI2C1_Ctrl.pau8Cmd = (uint8_t*)&asI2C1_Cmd_88L25[u16i];
				sI2C1_Ctrl.u16Counter = 0;
				sI2C1_Ctrl.u16MaxCount = sizeof(S_I2CPCMD);
				I2C_START(I2C1);
				while(sI2C1_Ctrl.u16MaxCount>0);
			}
			break;
		case 1: // to 85L40
			I2C_SetBusClockFreq(I2C1,I2C1_FREQ_85L40);
			sI2C1_Ctrl.u8DeviceAddr = I2C1_DEVICEADDR_85L40;
			for(u16i=0;u16i<sizeof(asI2C1_Cmd_85L40)/sizeof(S_I2CPCMD);u16i++) {
				sI2C1_Ctrl.pau8Cmd = (uint8_t*)&asI2C1_Cmd_85L40[u16i];
				sI2C1_Ctrl.u16Counter = 0;
				sI2C1_Ctrl.u16MaxCount = sizeof(S_I2CPCMD);
				I2C_START(I2C1);
				while(sI2C1_Ctrl.u16MaxCount>0);
			}
			break;
	}
}
// System clock =====================================================================
// Initiate system clock(PLL)
void System_Init(void) {
    // Unlock protected registers
    SYS_UnlockReg();
    // Enable HIRC clock 
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    // Waiting for HIRC clock ready 
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    // Switch HCLK clock source to HIRC 
//    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
		CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));
	
    // Enable HXT clock (external XTAL 12MHz) 
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
    // Wait for HXT clock ready 
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    // Set core clock as PLL_CLOCK from PLL
//    CLK_SetCoreClock(PLL_CLOCK);
    // Update System Core Clock 
    // User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. 
    SystemCoreClockUpdate();
    // Lock protected registers 
    SYS_LockReg();
}


void GPIO_Init(void)
{
	SYS->GPC_MFPH &= 0x00; //~(0x00<<
	PC->MODE |= 0x80000000; //<<22;  // PC15 as open drain output; 
}
	



/**************************************************************************//**
 * @file     Platform.c
 * @version  V0.10
 * $Revision: 1 $
 * $Date: 11/03/2017 10:04a $
 * @brief
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __PLATFORM_H__
#define __PLATFORM_H__

//#include "..\I94100\Include\I94100.h"		        /* H/W SFR definition */
#include "I94100.h"		        /* H/W SFR definition */

#define	__NUVOTON__									/* Nuvoton platform */

#include "NVTTypes.h"				/* Nuvoton type definition */

#define SYSTICK_1MS	 (SystemCoreClock/1000)
#define SYSTICK_0point1MS	 (SystemCoreClock/10000)

#define PDMA_TX_CHANNEL 0
#define PDMA_RX_CHANNEL 1

//PDMA channel associated with an index, defined by I94100
#define PDMA_I2S2_TX     24
#define PDMA_I2S0_RX     45

#define NUM_OF_DATA_BUFFERS 2
#define DATA_BUFFER_SIZE (32 * 4) // one channel 32 bit * 4 channel * 32 block
#define __ASSERT( x ) if( ( x ) == 0 ) { __disable_irq(); for( ;; ); }

#define I2C1_DEVICEADDR_88L25   (0x1A)
#define I2C1_DEVICEADDR_85L40   (0x1C)
#define I2C1_FREQ_88L25         (100000)
#define I2C1_FREQ_85L40         (100000)


//#define SPK_PDMA_CH	3

typedef enum _RXTXMODE{
	SEND_ONLY = 1,
	RECEIVE_ONLY = 2,
	SEND_AND_RECEIVE = 3,
} RXTXMODE;


typedef struct {
	uint16_t  u16DataCount;
	uint16_t  u16BufCount;
	uint16_t  u16WriteIdx;
	uint16_t  u16ReadIdx;
	int32_t*  pai32Buf;
} S_BUFCTRL;


#define BUFCTRL_IS_FULL(psBuf)                ((psBuf->u16DataCount>=psBuf->u16BufCount)?1:0)
#define BUFCTRL_IS_EMPTY(psBuf)               ((psBuf->u16DataCount==0)?1:0)
#define BUFCTRL_GET_COUNT(psBuf)              (psBuf->u16DataCount)
#define BUFCTRL_CFG(psBuf,pi32Data,u16Count)                   \
        psBuf->u16DataCount = 0;                               \
        psBuf->u16WriteIdx = 0;                                \
        psBuf->u16ReadIdx = 0;                                 \
        psBuf->u16BufCount = u16Count;                         \
        psBuf->pai32Buf = pi32Data
#define BUFCTRL_WRITE(psBuf,u32Data)                           \
        if(psBuf->u16DataCount<psBuf->u16BufCount) {           \
            psBuf->pai32Buf[psBuf->u16WriteIdx] = u32Data;     \
            psBuf->u16DataCount++;                             \
			if((psBuf->u16WriteIdx+=1)>=psBuf->u16BufCount) {  \
				psBuf->u16WriteIdx = 0;                        \
			}                                                  \
		}
#define BUFCTRL_READ(psBuf,pu32Data)                           \
        if(psBuf->u16DataCount>0) {                            \
            *pu32Data = psBuf->pai32Buf[psBuf->u16ReadIdx];    \
            psBuf->u16DataCount--;                             \
			if((psBuf->u16ReadIdx+=1)>=psBuf->u16BufCount) {   \
				psBuf->u16ReadIdx = 0;                         \
			}                                                  \
		}

// ******** I2C command interface structure **************
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

typedef struct {
	uint16_t  u16DataCount;
	uint16_t  u16BufCount;
	uint16_t  u16WriteIdx;
	uint16_t  u16ReadIdx;
	int16_t*  pai16Buf;
} S_CIRBUFCTRL;

#define CIRBUFCTRL_IS_FULL(psBuf)                ((psBuf->u16DataCount>=psBuf->u16BufCount)?1:0)
#define CIRBUFCTRL_IS_EMPTY(psBuf)               ((psBuf->u16DataCount==0)?1:0)
#define CIRBUFCTRL_GET_COUNT(psBuf)              (psBuf->u16DataCount)
#define CIRBUFCTRL_CFG(psBuf,pi16Data,u16Count)                 \
        psBuf->u16DataCount = 0;                               \
        psBuf->u16WriteIdx = 0;                                \
        psBuf->u16ReadIdx = 0;                                 \
        psBuf->u16BufCount = u16Count;                         \
        psBuf->pai16Buf = pi16Data
#define CIRBUFCTRL_WRITE(psBuf,u16Data)                         \
            {          \
            psBuf->pai16Buf[psBuf->u16WriteIdx] = u16Data;      \
            psBuf->u16DataCount++;                             \
            if((psBuf->u16WriteIdx+=1)>=psBuf->u16BufCount) {   \
		psBuf->u16WriteIdx = 0;                         \
		}                                               \
            }
#define CIRBUFCTRL_READ(psBuf,pu16Data)                         \
            {          \
            *pu16Data = psBuf->pai16Buf[psBuf->u16ReadIdx];     \
            psBuf->u16DataCount--;                             \
            if((psBuf->u16ReadIdx+=1)>=psBuf->u16BufCount) {    \
		psBuf->u16ReadIdx = 0;                          \
		}                                               \
            }

// preipheral, Core, board initialization
void GPIO_Init(void);
void UART0_Init(int32_t baudRate);
void I9400_System_Init(void);

//SW 1120 void SPK_Init(S_BUFCTRL* psOutBufCtrl);
//SW 1120 void SPK_Start(void);
//SW 1120 void SPK_Stop(void);
void MIC_Init(S_BUFCTRL* psInBufCtrl);
void MIC_Start(void);
void MIC_Stop(void);

#if 1 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
void SPK_Init(S_BUFCTRL* psOutBufCtrl, uint32_t u32SampleRate);
void SPK_Start(void);
void SPK_Stop(void);
//void SPK_Restart(uint32_t u32SampleRate); // Jace. 191118. Playback voice prompt using ESP32.
void SPI_Init(S_BUFCTRL* psSpiOutBufCtrl);
void SPI_Start(void);
void SPI_Stop(void);
void I2C0_Init(void);
void I2C0_Close(void);
void I2C1_Close(void);
#endif

// Demo, test functions
//void DEMO_85L40IN_88L25OUT(void);
void Read1500CharfromUart(void);
void Write1500chartoUart(void);
void Key_In_n_Times(uint32_t n);

void Dummy(void);
void PrepareCybData(int32_t * InBuf, S_CIRBUFCTRL * pOutBuf);
#endif /* __PLATFORM_H__ */

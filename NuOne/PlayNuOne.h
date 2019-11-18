#include <stdio.h>
#include "I94100.h"
#include "AudioRom.h"
#include "NuOneEx.h"
#include "SPIFlash.h"

//#define __APROM
//#define __DATAFLASH
#define __SPIFLASH

#if defined (__APROM)
	extern uint32_t u32audioBegin, u32audioEnd;
	#define AUDIOROM_STORAGE_START_ADDR (uint32_t)&u32audioBegin
#elif defined (__DATAFLASH)
	#define DATAFLASHADDR //0x32000
	#define AUDIOROM_STORAGE_START_ADDR DATAFLASHADDR
#elif defined (__SPIFLASH)
	#define SPIFLASHADDR 0
	#define AUDIOROM_STORAGE_START_ADDR SPIFLASHADDR
#endif

#define SPK_PDMA_CH     (3)


extern BOOL	bPCMPlaying;

extern BOOL 	bBufferEmpty;
extern UINT32 PDMA1CallBackCount;

BOOL PlayNuOnebyIdx(UINT8 u8PlayIdx);
void PlayLoop(void);
void PlayClose(void);
uint32_t ReadDataCallback(void *pu8Buf, uint32_t u32StartAddr, uint32_t u32Count);


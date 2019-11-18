#include <stdint.h>
#include "I94100.h"
#include "AwePlatform.h"
#include "PlatformAPI.h"
#include "BoardSupport.h"

//#define REC

#define PCM_SAMPLE_NUM (256)
#define DATAFLASH 0x64000

#define KEYCODE_0       0x1000
#define KEYCODE_1       0x1001
#define KEYCODE_2	0x1002

extern BOOL bCybBufReady;
extern int16_t ai16CyberonInputBuffer[];

extern AWEInstance g_AWEInstance;

extern uint32_t u32RecordAddr;
extern int16_t ai16CyberonInputBuffer[PCM_SAMPLE_NUM];

extern void EraseDataFlash(uint32_t u32DataFlash);
extern void Record2DF(void);
//extern void PrepareCybData(int32_t * InBuf, S_CIRBUFCTRL * pOutBuf);//void PrepareCybData(int32_t * InBuf);

extern void Test(void);


#include "Cyb_Support2.h"


#define AWE_FRAME_SIZE          40
#define OUTPUT_CHANNEL_COUNT    2

//int16_t i16CybInputBuffer[PCM_SAMPLE_NUM];
uint8_t u8CybBufPrprCnt = 0;
BOOL bCybBufReady = FALSE;

/*********************  Input buffer for Cyberon **************************
**  256 samples, 16 bit, 1 channel, 16K sample rate  
**  Format: total 256 samples, each 16 bit;                         
******************************************************************/
int16_t ai16CyberonInputBuffer[PCM_SAMPLE_NUM];


#ifdef REC
uint32_t u32RecordAddr = DATAFLASH;

void EraseDataFlash(uint32_t u32DataAddr)
{
  SYS_UnlockReg();
  FMC_Open();
  
  while(u32DataAddr<0x80000)
  {
    FMC_Erase(u32DataAddr);
    u32DataAddr += 0x1000;
  }
}

void Record2DF(void)
{
  uint16_t i;
  uint32_t u32temp;

  SYS_UnlockReg();  
  FMC_Open();
  for(i=0;i<PCM_SAMPLE_NUM;i+=2)
  {
    u32temp = ((uint16_t)ai16CyberonInputBuffer[i+1])<<16|(uint16_t)ai16CyberonInputBuffer[i];
    FMC_Write(u32RecordAddr,u32temp);
    u32RecordAddr +=4; 
  }
}
#endif

#if 0
void PrepareCybData(int32_t * InBuf)
{
  uint16_t i;
  
  for(i=0;i<AWE_FRAME_SIZE;i++)
  {
    i16CybInputBuffer[u8CybBufPrprCnt*AWE_FRAME_SIZE + i] = (int16_t)(InBuf[i*OUTPUT_CHANNEL_COUNT]>>16);
  }
  
  u8CybBufPrprCnt++;  
  if(u8CybBufPrprCnt==(PCM_SAMPLE_NUM/AWE_FRAME_SIZE))
  {
    bCybBufReady = TRUE;
    u8CybBufPrprCnt=0;
  }
}
#endif

#if 0
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
#endif
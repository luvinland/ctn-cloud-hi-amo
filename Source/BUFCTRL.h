
#include "Platform.h"

#ifndef __BUFCTRL_H__
#define __BUFCTRL_H__

//typedef enum {MINIDSP_TRANSMITTER, MINIDSP_RECEIVER} MINIDSP_MODE;

typedef enum _PATHMODE {
	PATH_88L25_TO_83P20,
	PATH_88L25_TO_88L25,
	PATH_85L40_TO_83P20,
	PATH_85L40_TO_88L25,
	PATH_APROM_TO_83P20,
	PATH_APROM_TO_88L25,
	PATH_MINIDSP_TO_83P20,
	PATH_MINIDSP_TO_88L25,
	PATH_85L40_TO_MINIDSP,
} PATHMODE;

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

#endif

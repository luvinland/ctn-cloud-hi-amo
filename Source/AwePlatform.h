/*******************************************************************************
*
*               Audio Framework
*               ---------------
*
********************************************************************************
*     AWE_Wrapper.h
********************************************************************************
*
*     Description:  AudioWeaver Wrapper Header File
*
*******************************************************************************/
 #ifndef __PLATFORM_H__
 #define __PLATFORM_H__

#include "PlatformAPI.h"
#include "MathHelper.h"
#include "Errors.h"
#include "TargetInfo.h"
#include "TuningHandler.h"
#include "ControlDriver.h"
#include "Framework.h"

#define INPUT_CHANNEL_COUNT 4
#define OUTPUT_CHANNEL_COUNT 2

#define TOTAL_BUFFERS 2
#define CHANNEL_BLOCK_SIZE_IN_SAMPLES 40
#define STEREO_BLOCK_SIZE_IN_SAMPLES 80

#define AWE_FRAME_SIZE 40
#define AWE_INPUT_BUFFER_SIZE_IN_SAMPLES (AWE_FRAME_SIZE * IN_CHANNEL_COUNT)
#define AWE_OUTPUT_BUFFER_SIZE_IN_SAMPLES (AWE_FRAME_SIZE * OUTPUT_CHANNEL_COUNT)

#define MAX_PUMP_COUNT 100

#define DEFAULT_AUDIO_IN_VOLUME 64

#define SMOOTHING_COEFF 894598

extern UINT32 s_PacketBuffer[];

extern volatile BOOL g_bReadyToSend;

extern AWE_FW_SLOW_ANY_DATA volatile INT32 s_AudioRunning;

extern volatile BOOL g_bBlinkLED4ForBoardAlive;

extern volatile UINT32 g_nPumpCount;

extern DSPC_ASRC USB_ASRC;

extern DWORD g_target_control_flags;

extern AWEInstance g_AWEInstance;

extern void BoardSetup(void);

extern volatile uint32_t g_msTicks;

#define AudioWeaverPump_IRQHandler1 EADC0_IRQHandler
#define AudioWeaverPump_IRQ1 EADC0_IRQn

#define AudioWeaverPump_IRQHandler2 EADC1_IRQHandler
#define AudioWeaverPump_IRQ2 EADC1_IRQn

void CoreInit(void);
void BoardInit(void);
void AudioInit(void);

void UART0_Init(INT32 nBaudRate);
void UART0SendReply(void);

extern volatile BOOL g_bPacketSendComplete;
extern volatile BOOL g_bBaudRateChanged;

INT32 awe_pltAudioStart(void);

INT32 awe_pltAudioStop(void);

void AWEProcessing(INT32 * pMicSamples, INT32 * pProcessedSamples);

void awe_pltGPIOSetPin(UINT32 nPinNo, UINT32 nValue);
void awe_pltGPIOTogglePin(UINT32 nPinNo);

void awe_vecScaleSmoothFract32(
		     fract32 *              src,
		     int                    srcInc,
		     fract32 *              dst,
		     int                    dstInc,
		     fract32 * VEC_RESTRICT currentGainFract,
		     fract32                targetGainFract,
		     int                    postShift,
		     fract32                smoothingCoeff,
		     int                    blockSize);

             extern volatile fract32 g_nNewVolGain;

extern volatile fract32 g_nNewVolGain;

void amo_vent_rx(uint8_t cmd_id); // Jace. 191014. Add control interface using UART0.

#endif

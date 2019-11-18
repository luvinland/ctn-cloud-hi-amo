/*******************************************************************************
*
*               Audio Framework
*               ---------------
*
********************************************************************************
*     AudioDriver.c
********************************************************************************
*
*     Description:  AudioWeaver Audio Driver
*
*******************************************************************************/
#include <string.h>
#include "AwePlatform.h"
#include "I94100.h"
//#include "dspc_asrc.h"		//JW
#include "Framework.h"
//#include "TargetProcessor.h" //JW 

volatile BOOL g_bAudioPump1Active = FALSE;
volatile BOOL g_bAudioPump2Active = FALSE;
volatile BOOL g_bAudioPumpActive = FALSE;

volatile UINT32 g_nPumpCount = 0;

static volatile fract32 nVolCurrentGain = 0;

volatile fract32 g_nNewVolGain = 0;

//-----------------------------------------------------------------------------
// METHOD:  AudioInit
// PURPOSE: Initialize AWE
//-----------------------------------------------------------------------------

AWE_OPTIMIZE_FOR_SPACE	//JW 
AWE_FW_SLOW_CODE			//JW
void AudioInit(void)
{
    // Higher Level AWE Processing Interrupt
    NVIC_SetPriority(AudioWeaverPump_IRQ1, 3);
    NVIC_EnableIRQ(AudioWeaverPump_IRQ1);

    // Lower level AWE Processing Interrupt
    NVIC_SetPriority(AudioWeaverPump_IRQ2, 4);
    NVIC_EnableIRQ(AudioWeaverPump_IRQ2);

}	// End AudioInit


//-----------------------------------------------------------------------------
// METHOD:  AWEProcessing
// PURPOSE: Pass new samples to Audio Weaver and return processed samples
//          Input - 4 channels of interleaved left aligned 24-bit data
//          Output - 2 channels of interleaved left aligned 24-bit data
//-----------------------------------------------------------------------------
void AWEProcessing(INT32 * pMicSamples, INT32 * pProcessedSamples)
{
    UINT32  fwInCount, fwOutCount;
    UINT32  used_chans, chan;
    INT32   pinStride;
    UINT32  layoutMask;
    INT32   nSample;
    INT32 * dest32BitPtr;
    fract32 * src32BitPtr;

    const UINT32 nPinNdx = 0;

    // If no AWE layout wired up then copy input to output
	if (!s_AudioRunning)
    {
        memset(pProcessedSamples, 0, AWE_OUTPUT_BUFFER_SIZE_IN_SAMPLES * sizeof(INT32) );

        // ****************** FOR DEBUG ************************************
        //for (nSample = 0; nSample < AWE_FRAME_SIZE; nSample++)
        //{
        //    // Send USB left channel to output left channel
        //    *pProcessedSamples++ = *pMicSamples++;
        //
        //    // Send USB right channel to output right channel
        //    *pProcessedSamples++ = *pMicSamples++;
        //
        //     // Skip 2 channels
        //     pMicSamples += 2;
        //
        //}
        // ******************************************************************
    }
    else
    {
        // Get Current audio layout number of channels
        awe_fwGetChannelCount(&g_AWEInstance, &fwInCount, &fwOutCount);

        if (fwInCount > 0)
        {
            // Get the input pin channels actually used
            used_chans = (fwInCount < INPUT_CHANNEL_COUNT) ? fwInCount : INPUT_CHANNEL_COUNT;

            // For USB the block size is in units of stereo samples and there can be only two channels
            for (chan = 0; chan < used_chans; chan++)
            {
                // Determine where the Framework wants the input data written
                dest32BitPtr = awe_fwGetInputChannelPtr(&g_AWEInstance, nPinNdx, chan, &pinStride);

                src32BitPtr = pMicSamples + chan;

                for (nSample = 0; nSample < AWE_FRAME_SIZE; nSample++)
                {
                    *dest32BitPtr = *src32BitPtr;

                    dest32BitPtr += pinStride;

                    src32BitPtr += INPUT_CHANNEL_COUNT;
                }
            }

            // Zero any unused layout inputs
            for ( ; chan < fwInCount; chan++)
            {
                // Determine where the Framework wants the input data written
                dest32BitPtr = awe_fwGetInputChannelPtr(&g_AWEInstance, nPinNdx, chan, &pinStride);

                for (nSample = 0; nSample < AWE_FRAME_SIZE; nSample++)
                {
                    *dest32BitPtr = 0;
                     dest32BitPtr += pinStride;
                }
            }
        }

        for (chan = 0; chan < OUTPUT_CHANNEL_COUNT; chan++)
        {
            if (chan < fwOutCount)
            {
                // Determine where the Framework wants the input data written
                src32BitPtr = (fract32 *)awe_fwGetOutputChannelPtr(&g_AWEInstance, nPinNdx, chan, &pinStride);

                // Implement output volume control
                //awe_vecScaleSmoothFract32(src32BitPtr, pinStride, src32BitPtr, pinStride,
//                                          (fract32 *)&nVolCurrentGain, g_nNewVolGain, 0, SMOOTHING_COEFF, AWE_FRAME_SIZE);

//                nVolCurrentGain = g_nNewVolGain;

                dest32BitPtr = pProcessedSamples + chan;

                for (nSample = 0; nSample < AWE_FRAME_SIZE; nSample++)
                {
                    *dest32BitPtr = *src32BitPtr;

                    src32BitPtr += pinStride;

                    // Output samples are interleaved
                    dest32BitPtr += OUTPUT_CHANNEL_COUNT;
                }
            }
            else
            {
				// The layout doesn't have enough channels; fill the remaining one with zeros
                dest32BitPtr = pProcessedSamples + chan;

                for (nSample = 0; nSample < AWE_FRAME_SIZE; nSample++)
                {
                    *dest32BitPtr = 0;

                    // Output samples are interleaved
                    dest32BitPtr += OUTPUT_CHANNEL_COUNT;
                }
            }
        }

        layoutMask = awe_fwAudioDMAComplete(&g_AWEInstance, nPinNdx, AWE_FRAME_SIZE);

        // If higher priority level processing ready pend an interrupt for it
        if (layoutMask & 1)
        {
            if (s_AudioRunning && !g_bAudioPump1Active)
            {
                NVIC_SetPendingIRQ(AudioWeaverPump_IRQ1);
            }
        }

        // If lower priority level processing ready pend an interrupt for it
        if (layoutMask & 2)
        {
            if (s_AudioRunning && !g_bAudioPump2Active)
            {
                NVIC_SetPendingIRQ(AudioWeaverPump_IRQ2);
            }
        }

        // When this higher level priority method returns, processing will resume
        // with AudioWeaverPump_IRQ1 or AudioWeaverPump_IRQ2 if triggered.
        // If both were triggered then processing for AudioWeaverPump_IRQ2 will start
        // only when the processing for AudioWeaverPump_IRQ1 has completed
    }

}	// End AWEProcessing



//-----------------------------------------------------------------------------
// METHOD:  AudioWeaver Pump Interrupt Handler
// PURPOSE: Perform AudioWeaver Processing
//-----------------------------------------------------------------------------
void AudioWeaverPump_IRQHandler1(void)
{
    g_bAudioPump1Active = TRUE;

    // If IDLE loop did not get some CPU time then skip the pump
    // g_nPumpCount gets reset in the idle loop method awe_pltTick
    if (g_nPumpCount++ < MAX_PUMP_COUNT)
    {
        // Perform highest priority AWE processing
        awe_fwPump(&g_AWEInstance, 0);
    }

    g_bAudioPump1Active = FALSE;

    // Manually clear software interrupt
    NVIC_ClearPendingIRQ(AudioWeaverPump_IRQ1);

}   // End AudioWeaverPump_IRQHandler1


//-----------------------------------------------------------------------------
// METHOD:  AudioWeaver Pump Interrupt Handler
// PURPOSE: Perform AudioWeaver Processing
//-----------------------------------------------------------------------------
void AudioWeaverPump_IRQHandler2(void)
{
    g_bAudioPump2Active = TRUE;

    // Perform medium priority AWE processing (FFT, etc)
    awe_fwPump(&g_AWEInstance, 1);

    g_bAudioPump2Active = FALSE;

    // Manually clear software interrupt
    NVIC_ClearPendingIRQ(AudioWeaverPump_IRQ2);

}   // End AudioWeaverPump_IRQHandler2


/****************************************************************************
*
*               Audio Framework
*               ---------------
*
****************************************************************************
*     ModTriggeredPulse.h
****************************************************************************
*
*     Description:  Generates pulse of pulseDuration, in msec, duration, when the input pin is triggered from 0 to 1.
*
*     Copyright:    DSP Concepts, Inc, 2007 - 2017
*                   1800 Wyatt Drive, Suite 14
*                   Santa Clara, CA 95054
*
***************************************************************************/

/**
 * @addtogroup Modules
 * @{
 */

/**
 * @file
 * @brief Generates pulse of pulseDuration, in msec, duration, when the input pin is triggered from 0 to 1.
 */

#ifndef _MOD_TRIGGEREDPULSE_H
#define _MOD_TRIGGEREDPULSE_H

#include "ModCommon.h"
#include "MathHelper.h"

#define MASK_TriggeredPulse_pulseDuration 0x00000100
#define MASK_TriggeredPulse_pulseDurationSamples 0x00000200
#define MASK_TriggeredPulse_sampleCount 0x00000400
#define MASK_TriggeredPulse_extendedCount 0x00000800
#define MASK_TriggeredPulse_prevTrigState 0x00001000
#define MASK_TriggeredPulse_trigger 0x00002000
#define OFFSET_TriggeredPulse_pulseDuration 0x00000008
#define OFFSET_TriggeredPulse_pulseDurationSamples 0x00000009
#define OFFSET_TriggeredPulse_sampleCount 0x0000000A
#define OFFSET_TriggeredPulse_extendedCount 0x0000000B
#define OFFSET_TriggeredPulse_prevTrigState 0x0000000C
#define OFFSET_TriggeredPulse_trigger 0x0000000D

#define CLASSID_TRIGGEREDPULSE (CLASS_ID_MODBASE + 1377)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Overall instance class
// ----------------------------------------------------------------------

typedef struct _awe_modTriggeredPulseInstance
{
    ModuleInstanceDescriptor instance;
    FLOAT32            pulseDuration;       // Width of the output generated pulse.
    UINT32             pulseDurationSamples; // Width of the output generated pulse in samples.
    UINT32             sampleCount;         // Sample counter that increments for each value output.
    UINT32             extendedCount;       // Extended pulse duation count in samples.
    INT32              prevTrigState;       // Holds the previous state of the input trigger pin.
    INT32              trigger;             // Boolean indicates when to trigger the pulse.
    
} awe_modTriggeredPulseInstance;

#if !defined(NOREDEF)
extern const ModClassModule awe_modTriggeredPulseClass;
#endif // #if !defined(NOREDEF)

/* Dynamic instantiation is used by default.  When building for static
** code, define AWE_STATIC_CODE to eliminate the constructor function. */

#ifndef AWE_STATIC_CODE
// This points the constructor for this class to the base constructor
#define awe_modTriggeredPulseConstructor(ARG1, ARG2, ARG3, ARG4, ARG5) ClassModule_Constructor(CLASSID_TRIGGEREDPULSE, ARG1, ARG2, ARG3, ARG4, ARG5)
#endif // #ifndef AWE_STATIC_CODE


void awe_modTriggeredPulseProcess(void *pInstance);

UINT32 awe_modTriggeredPulseSet(void *pInstance, UINT32 mask);
 



#ifdef __cplusplus
}
#endif


#endif // _MOD_TRIGGEREDPULSE_H

/**
 * @}
 *
 * End of file.
 */

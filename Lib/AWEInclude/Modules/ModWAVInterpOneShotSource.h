/****************************************************************************
*
*               Audio Framework
*               ---------------
*
****************************************************************************
*     ModWAVInterpOneShotSource.h
****************************************************************************
*
*     Description:  Periodic one shot WAV playback with linear interpolation
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
 * @brief Periodic one shot WAV playback with linear interpolation
 */

#ifndef _MOD_WAVINTERPONESHOTSOURCE_H
#define _MOD_WAVINTERPONESHOTSOURCE_H

#include "ModCommon.h"
#include "MathHelper.h"

#define MASK_WAVInterpOneShotSource_smoothingTime 0x00000100
#define MASK_WAVInterpOneShotSource_circularBufferSize 0x00000200
#define MASK_WAVInterpOneShotSource_extensionSize 0x00000400
#define MASK_WAVInterpOneShotSource_currentFRatio 0x00000800
#define MASK_WAVInterpOneShotSource_smoothingCoeff 0x00001000
#define MASK_WAVInterpOneShotSource_fIndex 0x00002000
#define MASK_WAVInterpOneShotSource_psObjectID 0x00004000
#define MASK_WAVInterpOneShotSource_psObjectPtr 0x00008000
#define OFFSET_WAVInterpOneShotSource_smoothingTime 0x00000008
#define OFFSET_WAVInterpOneShotSource_circularBufferSize 0x00000009
#define OFFSET_WAVInterpOneShotSource_extensionSize 0x0000000A
#define OFFSET_WAVInterpOneShotSource_currentFRatio 0x0000000B
#define OFFSET_WAVInterpOneShotSource_smoothingCoeff 0x0000000C
#define OFFSET_WAVInterpOneShotSource_fIndex 0x0000000D
#define OFFSET_WAVInterpOneShotSource_psObjectID 0x0000000E
#define OFFSET_WAVInterpOneShotSource_psObjectPtr 0x0000000F

#define CLASSID_WAVINTERPONESHOTSOURCE (CLASS_ID_MODBASE + 1380)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Overall instance class
// ----------------------------------------------------------------------

typedef struct _awe_modWAVInterpOneShotSourceInstance
{
    ModuleInstanceDescriptor instance;
    FLOAT32            smoothingTime;       // Time constant of the smoothing process
    INT32              circularBufferSize;  // Size of the upstream circular buffer.
    INT32              extensionSize;       // Size of the extension region of the circular buffer
    FLOAT32            currentFRatio;       // Specifies rate at which audio is read from the input buffer
    FLOAT32            smoothingCoeff;      // Smoothing coefficient
    FLOAT32            fIndex;              // state index
    INT32              psObjectID;          // Object ID of the upstream PeriodicStreaming module
    ModuleInstanceDescriptor **    psObjectPtr;         // Points to the upstream PeriodicStreaming module
} awe_modWAVInterpOneShotSourceInstance;

#if !defined(NOREDEF)
extern const ModClassModule awe_modWAVInterpOneShotSourceClass;
#endif // #if !defined(NOREDEF)

/* Dynamic instantiation is used by default.  When building for static
** code, define AWE_STATIC_CODE to eliminate the constructor function. */

#ifndef AWE_STATIC_CODE
// This points the constructor for this class to the base constructor
#define awe_modWAVInterpOneShotSourceConstructor(ARG1, ARG2, ARG3, ARG4, ARG5) ClassModule_Constructor(CLASSID_WAVINTERPONESHOTSOURCE, ARG1, ARG2, ARG3, ARG4, ARG5)
#endif // #ifndef AWE_STATIC_CODE


void awe_modWAVInterpOneShotSourceProcess(void *pInstance);

UINT32 awe_modWAVInterpOneShotSourceSet(void *pInstance, UINT32 mask);
 



#ifdef __cplusplus
}
#endif


#endif // _MOD_WAVINTERPONESHOTSOURCE_H

/**
 * @}
 *
 * End of file.
 */

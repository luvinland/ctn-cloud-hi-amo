/****************************************************************************
*
*               Audio Framework
*               ---------------
*
****************************************************************************
*     ModTriggeredFileSink.h
****************************************************************************
*
*     Description:  Writes data to a WAV file when triggered
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
 * @brief Writes data to a WAV file when triggered
 */

#ifndef _MOD_TRIGGEREDFILESINK_H
#define _MOD_TRIGGEREDFILESINK_H

#include "ModCommon.h"
#include "MathHelper.h"
#include "stdio.h"
#include "string.h"

#define MASK_TriggeredFileSink_fileIndex 0x00000100
#define MASK_TriggeredFileSink_samplesStored 0x00000200
#define MASK_TriggeredFileSink_recordingActive 0x00000400
#define MASK_TriggeredFileSink_errorCode 0x00000800
#define MASK_TriggeredFileSink_duration 0x00001000
#define MASK_TriggeredFileSink_durationSamples 0x00002000
#define MASK_TriggeredFileSink_fullFileSize 0x00004000
#define MASK_TriggeredFileSink_dataType 0x00008000
#define MASK_TriggeredFileSink_fullFileName 0x00010000
#define MASK_TriggeredFileSink_state 0x00020000
#define OFFSET_TriggeredFileSink_fileIndex 0x00000008
#define OFFSET_TriggeredFileSink_samplesStored 0x00000009
#define OFFSET_TriggeredFileSink_recordingActive 0x0000000A
#define OFFSET_TriggeredFileSink_errorCode 0x0000000B
#define OFFSET_TriggeredFileSink_duration 0x0000000C
#define OFFSET_TriggeredFileSink_durationSamples 0x0000000D
#define OFFSET_TriggeredFileSink_fullFileSize 0x0000000E
#define OFFSET_TriggeredFileSink_dataType 0x0000000F
#define OFFSET_TriggeredFileSink_fullFileName 0x00000010
#define OFFSET_TriggeredFileSink_state 0x00000011

#define CLASSID_TRIGGEREDFILESINK (CLASS_ID_MODBASE + 1375)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Overall instance class
// ----------------------------------------------------------------------

typedef struct _awe_modTriggeredFileSinkInstance
{
    ModuleInstanceDescriptor instance;
    INT32              fileIndex;           // Index of file to record
    INT32              samplesStored;       // Number of samples stored in the state buffer
    INT32              recordingActive;     // Indicates whether we are actively recording a file
    INT32              errorCode;           // Return error condition
    FLOAT32            duration;            // Length of each recording, in seconds
    INT32              durationSamples;     // Length of each recording, in samples
    INT32              fullFileSize;        // Size of the array to hold the full file path of the output .wav file
    INT32              dataType;            // Data type of the input pin
    INT32*             fullFileName;        // Complete file path of the output .wav file to be written data
    fract32*           state;               // Stores data prior to writing to disk
} awe_modTriggeredFileSinkInstance;

#if !defined(NOREDEF)
extern const ModClassModule awe_modTriggeredFileSinkClass;
#endif // #if !defined(NOREDEF)

/* Dynamic instantiation is used by default.  When building for static
** code, define AWE_STATIC_CODE to eliminate the constructor function. */

#ifndef AWE_STATIC_CODE
ModInstanceDescriptor * awe_modTriggeredFileSinkConstructor(INT32 * FW_RESTRICT retVal, UINT32 nIO, WireInstance ** FW_RESTRICT pWires, size_t argCount, const Sample * FW_RESTRICT args);
#endif // #ifndef AWE_STATIC_CODE


void awe_modTriggeredFileSinkProcess(void *pInstance);

UINT32 awe_modTriggeredFileSinkSet(void *pInstance, UINT32 mask);
 



#ifdef __cplusplus
}
#endif


#endif // _MOD_TRIGGEREDFILESINK_H

/**
 * @}
 *
 * End of file.
 */

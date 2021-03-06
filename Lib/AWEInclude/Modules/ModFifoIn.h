/****************************************************************************
*
*               Audio Framework
*               ---------------
*
****************************************************************************
*     ModFifoIn.h
****************************************************************************
*
*     Description:  First-in first-out buffering
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
 * @brief First-in first-out buffering
 */

#ifndef _MOD_FIFOIN_H
#define _MOD_FIFOIN_H

#include "ModCommon.h"
#include "MathHelper.h"

#define MASK_FifoIn_bufferSize 0x00000100
#define MASK_FifoIn_inBlockCounter 0x00000200
#define MASK_FifoIn_outBlockCounter 0x00000400
#define MASK_FifoIn_writeIndex 0x00000800
#define MASK_FifoIn_readIndex 0x00001000
#define MASK_FifoIn_state 0x00002000
#define OFFSET_FifoIn_bufferSize 0x00000008
#define OFFSET_FifoIn_inBlockCounter 0x00000009
#define OFFSET_FifoIn_outBlockCounter 0x0000000A
#define OFFSET_FifoIn_writeIndex 0x0000000B
#define OFFSET_FifoIn_readIndex 0x0000000C
#define OFFSET_FifoIn_state 0x0000000D

#define CLASSID_FIFOIN (CLASS_ID_MODBASE + 1362)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Overall instance class
// ----------------------------------------------------------------------

typedef struct _awe_modFifoInInstance
{
    ModuleInstanceDescriptor instance;
    INT32              bufferSize;          // Number of samples buffer per channel. The size of the array is BUFFERSIZE*numChannels.
    INT32              inBlockCounter;      // Counts number of input blocks that were written.
    INT32              outBlockCounter;     // Counts number of output blocks that were read.
    INT32              writeIndex;          // Points to where the next input sample will be written in the state buffer.
    INT32              readIndex;           // Points to where the next output sample will be read from.
    FLOAT32*           state;               // State variable array.
} awe_modFifoInInstance;

#if !defined(NOREDEF)
extern const ModClassModule awe_modFifoInClass;
#endif // #if !defined(NOREDEF)

/* Dynamic instantiation is used by default.  When building for static
** code, define AWE_STATIC_CODE to eliminate the constructor function. */

#ifndef AWE_STATIC_CODE
ModInstanceDescriptor * awe_modFifoInConstructor(INT32 * FW_RESTRICT retVal, UINT32 nIO, WireInstance ** FW_RESTRICT pWires, size_t argCount, const Sample * FW_RESTRICT args);
#endif // #ifndef AWE_STATIC_CODE


void awe_modFifoInProcess(void *pInstance);

 



#ifdef __cplusplus
}
#endif


#endif // _MOD_FIFOIN_H

/**
 * @}
 *
 * End of file.
 */

/****************************************************************************
*
*               Audio Framework
*               ---------------
*
****************************************************************************
*     ModMemoryLoading.h
****************************************************************************
*
*     Description:  Allocates memory for stress test
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
 * @brief Allocates memory for stress test
 */

#ifndef _MOD_MEMORYLOADING_H
#define _MOD_MEMORYLOADING_H

#include "ModCommon.h"
#include "MathHelper.h"

#define MASK_MemoryLoading_blockWriteCount 0x00000100
#define MASK_MemoryLoading_memSize 0x00000200
#define MASK_MemoryLoading_memHeap 0x00000400
#define MASK_MemoryLoading_blockCounter 0x00000800
#define MASK_MemoryLoading_memArray 0x00001000
#define OFFSET_MemoryLoading_blockWriteCount 0x00000008
#define OFFSET_MemoryLoading_memSize 0x00000009
#define OFFSET_MemoryLoading_memHeap 0x0000000A
#define OFFSET_MemoryLoading_blockCounter 0x0000000B
#define OFFSET_MemoryLoading_memArray 0x0000000C

#define CLASSID_MEMORYLOADING (CLASS_ID_MODBASE + 1378)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Overall instance class
// ----------------------------------------------------------------------

typedef struct _awe_modMemoryLoadingInstance
{
    ModuleInstanceDescriptor instance;
    INT32              blockWriteCount;     // Number of iterations to repeat the memory block write
    INT32              memSize;             // Memory size of the buffer in words
    INT32              memHeap;             // Heap in which to allocate memory
    INT32              blockCounter;        // Number of cycles to consumer per block process
    INT32*             memArray;            // Internal array allocated in speficied memory heap
} awe_modMemoryLoadingInstance;

#if !defined(NOREDEF)
extern const ModClassModule awe_modMemoryLoadingClass;
#endif // #if !defined(NOREDEF)

/* Dynamic instantiation is used by default.  When building for static
** code, define AWE_STATIC_CODE to eliminate the constructor function. */

#ifndef AWE_STATIC_CODE
ModInstanceDescriptor * awe_modMemoryLoadingConstructor(INT32 * FW_RESTRICT retVal, UINT32 nIO, WireInstance ** FW_RESTRICT pWires, size_t argCount, const Sample * FW_RESTRICT args);
#endif // #ifndef AWE_STATIC_CODE


void awe_modMemoryLoadingProcess(void *pInstance);

 



#ifdef __cplusplus
}
#endif


#endif // _MOD_MEMORYLOADING_H

/**
 * @}
 *
 * End of file.
 */

/****************************************************************************
*
*               Audio Framework
*               ---------------
*
****************************************************************************
*     ModTableLookupV2.h
****************************************************************************
*
*     Description:  Evenly spaced table lookup
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
 * @brief Evenly spaced table lookup
 */

#ifndef _MOD_TABLELOOKUPV2_H
#define _MOD_TABLELOOKUPV2_H

#include "ModCommon.h"
#include "MathHelper.h"

#define MASK_TableLookupV2_L 0x00000100
#define MASK_TableLookupV2_interpolationType 0x00000200
#define MASK_TableLookupV2_minX 0x00000400
#define MASK_TableLookupV2_maxX 0x00000800
#define MASK_TableLookupV2_divisor 0x00001000
#define MASK_TableLookupV2_tableHeap 0x00002000
#define MASK_TableLookupV2_table 0x00004000
#define OFFSET_TableLookupV2_L 0x00000008
#define OFFSET_TableLookupV2_interpolationType 0x00000009
#define OFFSET_TableLookupV2_minX 0x0000000A
#define OFFSET_TableLookupV2_maxX 0x0000000B
#define OFFSET_TableLookupV2_divisor 0x0000000C
#define OFFSET_TableLookupV2_tableHeap 0x0000000D
#define OFFSET_TableLookupV2_table 0x0000000E

#define CLASSID_TABLELOOKUPV2 (CLASS_ID_MODBASE + 221)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Overall instance class
// ----------------------------------------------------------------------

typedef struct _awe_modTableLookupV2Instance
{
    ModuleInstanceDescriptor instance;
    INT32              L;                   // Number of entries in the table.
    INT32              interpolationType;   // Selects the type of interpolation: 0=nearest, 1=linear.
    FLOAT32            minX;                // X value corresponding to the first table entry.
    FLOAT32            maxX;                // X value corresponding to the last table entry.
    FLOAT32            divisor;             // Precomputed constant = (L-1)/(maxX-minX) to eliminate division on the target.
    INT32              tableHeap;           // Heap in which to allocate memory.
    FLOAT32*           table;               // Table of evenly spaced values.
} awe_modTableLookupV2Instance;

#if !defined(NOREDEF)
extern const ModClassModule awe_modTableLookupV2Class;
#endif // #if !defined(NOREDEF)

/* Dynamic instantiation is used by default.  When building for static
** code, define AWE_STATIC_CODE to eliminate the constructor function. */

#ifndef AWE_STATIC_CODE
ModInstanceDescriptor * awe_modTableLookupV2Constructor(INT32 * FW_RESTRICT retVal, UINT32 nIO, WireInstance ** FW_RESTRICT pWires, size_t argCount, const Sample * FW_RESTRICT args);
#endif // #ifndef AWE_STATIC_CODE


void awe_modTableLookupV2Process(void *pInstance);

UINT32 awe_modTableLookupV2Set(void *pInstance, UINT32 mask);
 



#ifdef __cplusplus
}
#endif


#endif // _MOD_TABLELOOKUPV2_H

/**
 * @}
 *
 * End of file.
 */

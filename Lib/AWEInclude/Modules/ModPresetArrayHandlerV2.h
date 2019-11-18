/****************************************************************************
*
*               Audio Framework
*               ---------------
*
****************************************************************************
*     ModPresetArrayHandlerV2.h
****************************************************************************
*
*     Description:  Updates module array from different preset tables.
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
 * @brief Updates module array from different preset tables.
 */

#ifndef _MOD_PRESETARRAYHANDLERV2_H
#define _MOD_PRESETARRAYHANDLERV2_H

#include "ModCommon.h"
#include "MathHelper.h"

#define MASK_PresetArrayHandlerV2_numArrays 0x00000100
#define MASK_PresetArrayHandlerV2_numRows 0x00000200
#define MASK_PresetArrayHandlerV2_numCols 0x00000400
#define MASK_PresetArrayHandlerV2_operation 0x00000800
#define MASK_PresetArrayHandlerV2_initialFlag 0x00001000
#define MASK_PresetArrayHandlerV2_presetsArray 0x00002000
#define MASK_PresetArrayHandlerV2_pg1 0x00004000
#define MASK_PresetArrayHandlerV2_pg2 0x00008000
#define MASK_PresetArrayHandlerV2_pg3 0x00010000
#define MASK_PresetArrayHandlerV2_pg4 0x00020000
#define MASK_PresetArrayHandlerV2_pg5 0x00040000
#define MASK_PresetArrayHandlerV2_pg6 0x00080000
#define MASK_PresetArrayHandlerV2_pg7 0x00100000
#define MASK_PresetArrayHandlerV2_pg8 0x00200000
#define MASK_PresetArrayHandlerV2_pg9 0x00400000
#define MASK_PresetArrayHandlerV2_pg10 0x00800000
#define MASK_PresetArrayHandlerV2_modPtr 0x01000000
#define MASK_PresetArrayHandlerV2_varPtr 0x02000000
#define OFFSET_PresetArrayHandlerV2_numArrays 0x00000008
#define OFFSET_PresetArrayHandlerV2_numRows 0x00000009
#define OFFSET_PresetArrayHandlerV2_numCols 0x0000000A
#define OFFSET_PresetArrayHandlerV2_operation 0x0000000B
#define OFFSET_PresetArrayHandlerV2_initialFlag 0x0000000C
#define OFFSET_PresetArrayHandlerV2_presetsArray 0x0000000D
#define OFFSET_PresetArrayHandlerV2_pg1 0x0000000E
#define OFFSET_PresetArrayHandlerV2_pg2 0x0000000F
#define OFFSET_PresetArrayHandlerV2_pg3 0x00000010
#define OFFSET_PresetArrayHandlerV2_pg4 0x00000011
#define OFFSET_PresetArrayHandlerV2_pg5 0x00000012
#define OFFSET_PresetArrayHandlerV2_pg6 0x00000013
#define OFFSET_PresetArrayHandlerV2_pg7 0x00000014
#define OFFSET_PresetArrayHandlerV2_pg8 0x00000015
#define OFFSET_PresetArrayHandlerV2_pg9 0x00000016
#define OFFSET_PresetArrayHandlerV2_pg10 0x00000017
#define OFFSET_PresetArrayHandlerV2_modPtr 0x00000018
#define OFFSET_PresetArrayHandlerV2_varPtr 0x00000019

#define CLASSID_PRESETARRAYHANDLERV2 (CLASS_ID_MODBASE + 1382)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Overall instance class
// ----------------------------------------------------------------------

typedef struct _awe_modPresetArrayHandlerV2Instance
{
    ModuleInstanceDescriptor instance;
    INT32              numArrays;           // number of presets.
    INT32              numRows;             // the number of vectors.
    INT32              numCols;             // the size of each vector.
    INT32              operation;           // math operation on presets. 0 - multiplication and 1 - addition.
    INT32              initialFlag;         // boolean flag which handles initial setup.
    FLOAT32*           presetsArray;        // 3-D presets array table.
    FLOAT32*           pg1;                 // Points to the 1st preset group elements in the presetsArray.
    FLOAT32*           pg2;                 // Points to the 2nd preset group elements in the presetsArray.
    FLOAT32*           pg3;                 // Points to the 3rd preset group elements in the presetsArray.
    FLOAT32*           pg4;                 // Points to the 4th preset group elements in the presetsArray.
    FLOAT32*           pg5;                 // Points to the 5th preset group elements in the presetsArray.
    FLOAT32*           pg6;                 // Points to the 6th preset group elements in the presetsArray.
    FLOAT32*           pg7;                 // Points to the 7th preset group elements in the presetsArray.
    FLOAT32*           pg8;                 // Points to the 8th preset group elements in the presetsArray.
    FLOAT32*           pg9;                 // Points to the 9th preset group elements in the presetsArray.
    FLOAT32*           pg10;                // Points to the 10th preset group elements in the presetsArray.
    void *             modPtr;              // Points to the module to set.
    void *             varPtr;              // Points to the variable to set within the module instance structure.
} awe_modPresetArrayHandlerV2Instance;

#if !defined(NOREDEF)
extern const ModClassModule awe_modPresetArrayHandlerV2Class;
#endif // #if !defined(NOREDEF)

/* Dynamic instantiation is used by default.  When building for static
** code, define AWE_STATIC_CODE to eliminate the constructor function. */

#ifndef AWE_STATIC_CODE
ModInstanceDescriptor * awe_modPresetArrayHandlerV2Constructor(INT32 * FW_RESTRICT retVal, UINT32 nIO, WireInstance ** FW_RESTRICT pWires, size_t argCount, const Sample * FW_RESTRICT args);
#endif // #ifndef AWE_STATIC_CODE


void awe_modPresetArrayHandlerV2Process(void *pInstance);

UINT32 awe_modPresetArrayHandlerV2Set(void *pInstance, UINT32 mask);
 



#ifdef __cplusplus
}
#endif


#endif // _MOD_PRESETARRAYHANDLERV2_H

/**
 * @}
 *
 * End of file.
 */

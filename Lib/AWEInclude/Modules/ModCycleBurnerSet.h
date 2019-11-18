/****************************************************************************
*
*               Audio Framework
*               ---------------
*
****************************************************************************
*     ModCycleBurnerSet.h
****************************************************************************
*
*     Description:  Consumes processing cycles on the target in Set function
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
 * @brief Consumes processing cycles on the target in Set function
 */

#ifndef _MOD_CYCLEBURNERSET_H
#define _MOD_CYCLEBURNERSET_H

#include "ModCommon.h"
#include "MathHelper.h"

#define MASK_CycleBurnerSet_numCycles 0x00000100
#define MASK_CycleBurnerSet_trigger 0x00000200
#define OFFSET_CycleBurnerSet_numCycles 0x00000008
#define OFFSET_CycleBurnerSet_trigger 0x00000009

#define CLASSID_CYCLEBURNERSET (CLASS_ID_MODBASE + 1379)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Overall instance class
// ----------------------------------------------------------------------

typedef struct _awe_modCycleBurnerSetInstance
{
    ModuleInstanceDescriptor instance;
    INT32              numCycles;           // Number of cycles to consumer per trigger event
    INT32              trigger;             // Used to trigger cycles consume process in Set function.
    
} awe_modCycleBurnerSetInstance;

#if !defined(NOREDEF)
extern const ModClassModule awe_modCycleBurnerSetClass;
#endif // #if !defined(NOREDEF)

/* Dynamic instantiation is used by default.  When building for static
** code, define AWE_STATIC_CODE to eliminate the constructor function. */

#ifndef AWE_STATIC_CODE
// This points the constructor for this class to the base constructor
#define awe_modCycleBurnerSetConstructor(ARG1, ARG2, ARG3, ARG4, ARG5) ClassModule_Constructor(CLASSID_CYCLEBURNERSET, ARG1, ARG2, ARG3, ARG4, ARG5)
#endif // #ifndef AWE_STATIC_CODE


void awe_modCycleBurnerSetProcess(void *pInstance);

UINT32 awe_modCycleBurnerSetSet(void *pInstance, UINT32 mask);
 



#ifdef __cplusplus
}
#endif


#endif // _MOD_CYCLEBURNERSET_H

/**
 * @}
 *
 * End of file.
 */

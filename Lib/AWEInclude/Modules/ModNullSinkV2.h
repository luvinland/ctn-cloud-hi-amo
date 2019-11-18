/****************************************************************************
*
*               Audio Framework
*               ---------------
*
****************************************************************************
*     ModNullSinkV2.h
****************************************************************************
*
*     Description:  Sink module with variable number of input pins which performs no computation and discards the input
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
 * @brief Sink module with variable number of input pins which performs no computation and discards the input
 */

#ifndef _MOD_NULLSINKV2_H
#define _MOD_NULLSINKV2_H

#include "ModCommon.h"
#include "MathHelper.h"


#define CLASSID_NULLSINKV2 (CLASS_ID_MODBASE + 220)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Overall instance class
// ----------------------------------------------------------------------

typedef struct _awe_modNullSinkV2Instance
{
    ModuleInstanceDescriptor instance;
    
    
} awe_modNullSinkV2Instance;

#if !defined(NOREDEF)
extern const ModClassModule awe_modNullSinkV2Class;
#endif // #if !defined(NOREDEF)

/* Dynamic instantiation is used by default.  When building for static
** code, define AWE_STATIC_CODE to eliminate the constructor function. */

#ifndef AWE_STATIC_CODE
// This points the constructor for this class to the base constructor
#define awe_modNullSinkV2Constructor(ARG1, ARG2, ARG3, ARG4, ARG5) ClassModule_Constructor(CLASSID_NULLSINKV2, ARG1, ARG2, ARG3, ARG4, ARG5)
#endif // #ifndef AWE_STATIC_CODE


void awe_modNullSinkV2Process(void *pInstance);

 



#ifdef __cplusplus
}
#endif


#endif // _MOD_NULLSINKV2_H

/**
 * @}
 *
 * End of file.
 */

/****************************************************************************
*
*               Audio Framework
*               ---------------
*
****************************************************************************
*     ModUnwrap.h
****************************************************************************
*
*     Description:  Unwrap phase angle in radians
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
 * @brief Unwrap phase angle in radians
 */

#ifndef _MOD_UNWRAP_H
#define _MOD_UNWRAP_H

#include "ModCommon.h"
#include "MathHelper.h"


#define CLASSID_UNWRAP (CLASS_ID_MODBASE + 1349)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Overall instance class
// ----------------------------------------------------------------------

typedef struct _awe_modUnwrapInstance
{
    ModuleInstanceDescriptor instance;
    
    
} awe_modUnwrapInstance;

#if !defined(NOREDEF)
extern const ModClassModule awe_modUnwrapClass;
#endif // #if !defined(NOREDEF)

/* Dynamic instantiation is used by default.  When building for static
** code, define AWE_STATIC_CODE to eliminate the constructor function. */

#ifndef AWE_STATIC_CODE
// This points the constructor for this class to the base constructor
#define awe_modUnwrapConstructor(ARG1, ARG2, ARG3, ARG4, ARG5) ClassModule_Constructor(CLASSID_UNWRAP, ARG1, ARG2, ARG3, ARG4, ARG5)
#endif // #ifndef AWE_STATIC_CODE


void awe_modUnwrapProcess(void *pInstance);

 



#ifdef __cplusplus
}
#endif


#endif // _MOD_UNWRAP_H

/**
 * @}
 *
 * End of file.
 */

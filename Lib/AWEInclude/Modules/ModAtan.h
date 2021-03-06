/****************************************************************************
*
*               Audio Framework
*               ---------------
*
****************************************************************************
*     ModAtan.h
****************************************************************************
*
*     Description:  Computes the funtion atan(x) using the math library
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
 * @brief Computes the funtion atan(x) using the math library
 */

#ifndef _MOD_ATAN_H
#define _MOD_ATAN_H

#include "ModCommon.h"
#include "MathHelper.h"


#define CLASSID_ATAN (CLASS_ID_MODBASE + 83)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Overall instance class
// ----------------------------------------------------------------------

typedef struct _awe_modAtanInstance
{
    ModuleInstanceDescriptor instance;
    
    
} awe_modAtanInstance;

#if !defined(NOREDEF)
extern const ModClassModule awe_modAtanClass;
#endif // #if !defined(NOREDEF)

/* Dynamic instantiation is used by default.  When building for static
** code, define AWE_STATIC_CODE to eliminate the constructor function. */

#ifndef AWE_STATIC_CODE
// This points the constructor for this class to the base constructor
#define awe_modAtanConstructor(ARG1, ARG2, ARG3, ARG4, ARG5) ClassModule_Constructor(CLASSID_ATAN, ARG1, ARG2, ARG3, ARG4, ARG5)
#endif // #ifndef AWE_STATIC_CODE


void awe_modAtanProcess(void *pInstance);

 



#ifdef __cplusplus
}
#endif


#endif // _MOD_ATAN_H

/**
 * @}
 *
 * End of file.
 */

/****************************************************************************
*
*               Audio Framework
*               ---------------
*
****************************************************************************
*     ModPow10toX.h
****************************************************************************
*
*     Description:  Computes the function y = 10^x using the math library
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
 * @brief Computes the function y = 10^x using the math library
 */

#ifndef _MOD_POW10TOX_H
#define _MOD_POW10TOX_H

#include "ModCommon.h"
#include "MathHelper.h"


#define CLASSID_POW10TOX (CLASS_ID_MODBASE + 59)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Overall instance class
// ----------------------------------------------------------------------

typedef struct _awe_modPow10toXInstance
{
    ModuleInstanceDescriptor instance;
    
    
} awe_modPow10toXInstance;

#if !defined(NOREDEF)
extern const ModClassModule awe_modPow10toXClass;
#endif // #if !defined(NOREDEF)

/* Dynamic instantiation is used by default.  When building for static
** code, define AWE_STATIC_CODE to eliminate the constructor function. */

#ifndef AWE_STATIC_CODE
// This points the constructor for this class to the base constructor
#define awe_modPow10toXConstructor(ARG1, ARG2, ARG3, ARG4, ARG5) ClassModule_Constructor(CLASSID_POW10TOX, ARG1, ARG2, ARG3, ARG4, ARG5)
#endif // #ifndef AWE_STATIC_CODE


void awe_modPow10toXProcess(void *pInstance);

 



#ifdef __cplusplus
}
#endif


#endif // _MOD_POW10TOX_H

/**
 * @}
 *
 * End of file.
 */

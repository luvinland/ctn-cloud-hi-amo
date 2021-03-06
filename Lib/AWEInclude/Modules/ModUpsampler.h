/****************************************************************************
*
*               Audio Framework
*               ---------------
*
****************************************************************************
*     ModUpsampler.h
****************************************************************************
*
*     Description:  Multi-channel upsampler that inserts zeros between samples
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
 * @brief Multi-channel upsampler that inserts zeros between samples
 */

#ifndef _MOD_UPSAMPLER_H
#define _MOD_UPSAMPLER_H

#include "ModCommon.h"
#include "MathHelper.h"

#define MASK_Upsampler_L 0x00000100
#define OFFSET_Upsampler_L 0x00000008

#define CLASSID_UPSAMPLER (CLASS_ID_MODBASE + 1351)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Overall instance class
// ----------------------------------------------------------------------

typedef struct _awe_modUpsamplerInstance
{
    ModuleInstanceDescriptor instance;
    INT32              L;                   // Interpolation factor. L-1 zeros inserted between each sample.
    
} awe_modUpsamplerInstance;

#if !defined(NOREDEF)
extern const ModClassModule awe_modUpsamplerClass;
#endif // #if !defined(NOREDEF)

/* Dynamic instantiation is used by default.  When building for static
** code, define AWE_STATIC_CODE to eliminate the constructor function. */

#ifndef AWE_STATIC_CODE
// This points the constructor for this class to the base constructor
#define awe_modUpsamplerConstructor(ARG1, ARG2, ARG3, ARG4, ARG5) ClassModule_Constructor(CLASSID_UPSAMPLER, ARG1, ARG2, ARG3, ARG4, ARG5)
#endif // #ifndef AWE_STATIC_CODE


void awe_modUpsamplerProcess(void *pInstance);

 



#ifdef __cplusplus
}
#endif


#endif // _MOD_UPSAMPLER_H

/**
 * @}
 *
 * End of file.
 */

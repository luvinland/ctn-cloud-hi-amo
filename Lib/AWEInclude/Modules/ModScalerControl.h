/****************************************************************************
*
*               Audio Framework
*               ---------------
*
****************************************************************************
*     ModScalerControl.h
****************************************************************************
*
*     Description:  Smooth scaler in which the gain (in linear units) is taken from an input pin
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
 * @brief Smooth scaler in which the gain (in linear units) is taken from an input pin
 */

#ifndef _MOD_SCALERCONTROL_H
#define _MOD_SCALERCONTROL_H

#include "ModCommon.h"
#include "MathHelper.h"

#define MASK_ScalerControl_smoothingTime 0x00000100
#define MASK_ScalerControl_currentGain 0x00000200
#define MASK_ScalerControl_smoothingCoeff 0x00000400
#define OFFSET_ScalerControl_smoothingTime 0x00000008
#define OFFSET_ScalerControl_currentGain 0x00000009
#define OFFSET_ScalerControl_smoothingCoeff 0x0000000A

#define CLASSID_SCALERCONTROL (CLASS_ID_MODBASE + 24)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Overall instance class
// ----------------------------------------------------------------------

typedef struct _awe_modScalerControlInstance
{
    ModuleInstanceDescriptor instance;
    FLOAT32            smoothingTime;       // Time constant of the smoothing process
    FLOAT32            currentGain;         // Instantaneous gain applied by the module.
    FLOAT32            smoothingCoeff;      // Smoothing coefficient
    
} awe_modScalerControlInstance;

#if !defined(NOREDEF)
extern const ModClassModule awe_modScalerControlClass;
#endif // #if !defined(NOREDEF)

/* Dynamic instantiation is used by default.  When building for static
** code, define AWE_STATIC_CODE to eliminate the constructor function. */

#ifndef AWE_STATIC_CODE
// This points the constructor for this class to the base constructor
#define awe_modScalerControlConstructor(ARG1, ARG2, ARG3, ARG4, ARG5) ClassModule_Constructor(CLASSID_SCALERCONTROL, ARG1, ARG2, ARG3, ARG4, ARG5)
#endif // #ifndef AWE_STATIC_CODE


void awe_modScalerControlProcess(void *pInstance);

UINT32 awe_modScalerControlSet(void *pInstance, UINT32 mask);
 



#ifdef __cplusplus
}
#endif


#endif // _MOD_SCALERCONTROL_H

/**
 * @}
 *
 * End of file.
 */

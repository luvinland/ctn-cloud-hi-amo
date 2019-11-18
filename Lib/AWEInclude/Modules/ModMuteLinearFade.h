/****************************************************************************
*
*               Audio Framework
*               ---------------
*
****************************************************************************
*     ModMuteLinearFade.h
****************************************************************************
*
*     Description:  Smoothly mutes and unmutes with Linear Fade
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
 * @brief Smoothly mutes and unmutes with Linear Fade
 */

#ifndef _MOD_MUTELINEARFADE_H
#define _MOD_MUTELINEARFADE_H

#include "ModCommon.h"
#include "MathHelper.h"

#define MASK_MuteLinearFade_fadeOut 0x00000100
#define MASK_MuteLinearFade_fadeOutTime 0x00000200
#define MASK_MuteLinearFade_fadeInTime 0x00000400
#define MASK_MuteLinearFade_faderState 0x00000800
#define MASK_MuteLinearFade_currentGain 0x00001000
#define MASK_MuteLinearFade_fadeOutCoeff 0x00002000
#define MASK_MuteLinearFade_fadeInCoeff 0x00004000
#define MASK_MuteLinearFade_gain 0x00008000
#define OFFSET_MuteLinearFade_fadeOut 0x00000008
#define OFFSET_MuteLinearFade_fadeOutTime 0x00000009
#define OFFSET_MuteLinearFade_fadeInTime 0x0000000A
#define OFFSET_MuteLinearFade_faderState 0x0000000B
#define OFFSET_MuteLinearFade_currentGain 0x0000000C
#define OFFSET_MuteLinearFade_fadeOutCoeff 0x0000000D
#define OFFSET_MuteLinearFade_fadeInCoeff 0x0000000E
#define OFFSET_MuteLinearFade_gain 0x0000000F

#define CLASSID_MUTELINEARFADE (CLASS_ID_MODBASE + 1374)

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Overall instance class
// ----------------------------------------------------------------------

typedef struct _awe_modMuteLinearFadeInstance
{
    ModuleInstanceDescriptor instance;
    INT32              fadeOut;             // Boolean that controls muting/unmuting.
    FLOAT32            fadeOutTime;         // Time constant of the fade out (mute) process.
    FLOAT32            fadeInTime;          // Time constant of the fade in (unmute) process.
    INT32              faderState;          // State of the fader ramp. 0 - fader in/out is in progress and 1 - fading is done.
    FLOAT32            currentGain;         // Instantaneous gain applied by the module.  This is also the starting gain of the module.
    FLOAT32            fadeOutCoeff;        // Fade out coefficient.
    FLOAT32            fadeInCoeff;         // Fade in coefficient.
    FLOAT32            gain;                // Target gain.
    
} awe_modMuteLinearFadeInstance;

#if !defined(NOREDEF)
extern const ModClassModule awe_modMuteLinearFadeClass;
#endif // #if !defined(NOREDEF)

/* Dynamic instantiation is used by default.  When building for static
** code, define AWE_STATIC_CODE to eliminate the constructor function. */

#ifndef AWE_STATIC_CODE
// This points the constructor for this class to the base constructor
#define awe_modMuteLinearFadeConstructor(ARG1, ARG2, ARG3, ARG4, ARG5) ClassModule_Constructor(CLASSID_MUTELINEARFADE, ARG1, ARG2, ARG3, ARG4, ARG5)
#endif // #ifndef AWE_STATIC_CODE


void awe_modMuteLinearFadeProcess(void *pInstance);

UINT32 awe_modMuteLinearFadeSet(void *pInstance, UINT32 mask);
 



#ifdef __cplusplus
}
#endif


#endif // _MOD_MUTELINEARFADE_H

/**
 * @}
 *
 * End of file.
 */

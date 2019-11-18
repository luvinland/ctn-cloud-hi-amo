/*******************************************************************************
*
********************************************************************************
*     ModuleList.h
********************************************************************************
*
*     Description:  Module list for the target
*
*     Copyright: (c) 2017 DSP Concepts, Inc. All rights reserved.
*                         1800 Wyatt Drive, Suite 14
*                         Sunnyvale, CA 95054
*
*******************************************************************************/

#include "Framework.h"

// Sensory TrulyHandsFree Modules
extern const ModClassModule awe_modSensoryTHFCortexMClass;
extern const ModClassModule awe_modSensoryTHFCortexMCmdsClass;

// Standard library modules
extern const ModClassModule awe_modAGCCoreClass;
extern const ModClassModule awe_modAGCLimiterCoreClass;
extern const ModClassModule awe_modAGCLimiterCoreFract32Class;
extern const ModClassModule awe_modAGCMultiplierFract32Class;
extern const ModClassModule awe_modAGCNoiseGateCoreClass;
extern const ModClassModule awe_modAbsClass;
extern const ModClassModule awe_modAbsFract32Class;
extern const ModClassModule awe_modAcosClass;
extern const ModClassModule awe_modAdderClass;
extern const ModClassModule awe_modAdderFract32Class;
extern const ModClassModule awe_modAdderInt32Class;
extern const ModClassModule awe_modAsinClass;
extern const ModClassModule awe_modAtanClass;
extern const ModClassModule awe_modAtan2Class;
extern const ModClassModule awe_modBalanceClass;
extern const ModClassModule awe_modBalanceFract32Class;
extern const ModClassModule awe_modBiquadClass;
extern const ModClassModule awe_modBiquadCascadeClass;
extern const ModClassModule awe_modBiquadCascadeFract32Class;
extern const ModClassModule awe_modBiquadFract32Class;
extern const ModClassModule awe_modBiquadNCascadeClass;
extern const ModClassModule awe_modBiquadSmoothedClass;
extern const ModClassModule awe_modBiquadSmoothedFract32Class;
extern const ModClassModule awe_modBiquadV2Fract32Class;
extern const ModClassModule awe_modBitsToIntClass;
extern const ModClassModule awe_modBlockCounterClass;
extern const ModClassModule awe_modBooleanInvertClass;
extern const ModClassModule awe_modBooleanSinkClass;
extern const ModClassModule awe_modBooleanSourceClass;
extern const ModClassModule awe_modButterworthFilterClass;
extern const ModClassModule awe_modButterworthFilterFract32Class;
extern const ModClassModule awe_modCeilClass;
extern const ModClassModule awe_modClipAsymClass;
extern const ModClassModule awe_modClipAsymFract32Class;
extern const ModClassModule awe_modClipIndicatorClass;
extern const ModClassModule awe_modClipIndicatorFract32Class;
extern const ModClassModule awe_modCopierClass;
extern const ModClassModule awe_modCosClass;
extern const ModClassModule awe_modCoshClass;
extern const ModClassModule awe_modCounterClass;
extern const ModClassModule awe_modCrossFaderClass;
extern const ModClassModule awe_modDCSourceIntClass;
extern const ModClassModule awe_modDCSourceV2Class;
extern const ModClassModule awe_modDCSourceV2Fract32Class;
extern const ModClassModule awe_modDb10Class;
extern const ModClassModule awe_modDb10ApproxClass;
extern const ModClassModule awe_modDb20Class;
extern const ModClassModule awe_modDb20ApproxClass;
extern const ModClassModule awe_modDb20Fract32Class;
extern const ModClassModule awe_modDebounceClass;
extern const ModClassModule awe_modDeinterleaveClass;
extern const ModClassModule awe_modDelayClass;
extern const ModClassModule awe_modDelayMsecClass;
extern const ModClassModule awe_modDivideClass;
extern const ModClassModule awe_modExpClass;
extern const ModClassModule awe_modFIRClass;
extern const ModClassModule awe_modFIRFract32Class;
extern const ModClassModule awe_modFloorClass;
extern const ModClassModule awe_modFmodClass;
extern const ModClassModule awe_modFrexpClass;
extern const ModClassModule awe_modGPIOClass;
extern const ModClassModule awe_modGraphicEQBandClass;
extern const ModClassModule awe_modGraphicEQBandFract32Class;
extern const ModClassModule awe_modImpulseMsecSourceClass;
extern const ModClassModule awe_modImpulseMsecSourceFract32Class;
extern const ModClassModule awe_modImpulseSourceFract32Class;
extern const ModClassModule awe_modIntToBitsClass;
extern const ModClassModule awe_modInterleaveClass;
extern const ModClassModule awe_modInvertClass;
extern const ModClassModule awe_modInvertFract32Class;
extern const ModClassModule awe_modLdexpClass;
extern const ModClassModule awe_modLogClass;
extern const ModClassModule awe_modLog10Class;
extern const ModClassModule awe_modLog10Fract32Class;
extern const ModClassModule awe_modLog2Class;
extern const ModClassModule awe_modLog2Fract32Class;
extern const ModClassModule awe_modLogicAllClass;
extern const ModClassModule awe_modLogicAndConstFract32Class;
extern const ModClassModule awe_modLogicAnyClass;
extern const ModClassModule awe_modLogicBinaryOpClass;
extern const ModClassModule awe_modLogicCompareClass;
extern const ModClassModule awe_modLogicCompareConstClass;
extern const ModClassModule awe_modLogicCompareConstFract32Class;
extern const ModClassModule awe_modLogicCompareConstInt32Class;
extern const ModClassModule awe_modLogicCompareFract32Class;
extern const ModClassModule awe_modLogicCompareInt32Class;
extern const ModClassModule awe_modMaxAbsClass;
extern const ModClassModule awe_modMaxAbsFract32Class;
extern const ModClassModule awe_modMeterClass;
extern const ModClassModule awe_modMeterFract32Class;
extern const ModClassModule awe_modMixerDenseFract32Class;
extern const ModClassModule awe_modMixerFract32Class;
extern const ModClassModule awe_modMixerSmoothedFract32Class;
extern const ModClassModule awe_modMixerV3Class;
extern const ModClassModule awe_modMixerV3Fract32Class;
extern const ModClassModule awe_modModfClass;
extern const ModClassModule awe_modMultiplexorClass;
extern const ModClassModule awe_modMultiplexorV2Class;
extern const ModClassModule awe_modMultiplexorV2Fract32Class;
extern const ModClassModule awe_modMultiplierFract32Class;
extern const ModClassModule awe_modMultiplierV2Class;
extern const ModClassModule awe_modMultiplierV2Fract32Class;
extern const ModClassModule awe_modMuteNSmoothedClass;
extern const ModClassModule awe_modMuteNSmoothedFract32Class;
extern const ModClassModule awe_modMuteSmoothedClass;
extern const ModClassModule awe_modMuteSmoothedFract32Class;
extern const ModClassModule awe_modMuteUnmuteClass;
extern const ModClassModule awe_modMuteUnmuteFract32Class;
extern const ModClassModule awe_modNullSinkClass;
extern const ModClassModule awe_modParamGetClass;
extern const ModClassModule awe_modParamSetClass;
extern const ModClassModule awe_modPeriodicFunctionGenClass;
extern const ModClassModule awe_modPeriodicSourceClass;
extern const ModClassModule awe_modPeriodicSourceFract32Class;
extern const ModClassModule awe_modPolynomialClass;
extern const ModClassModule awe_modPolynomialFract32Class;
extern const ModClassModule awe_modPowClass;
extern const ModClassModule awe_modPow10toXClass;
extern const ModClassModule awe_modPow2Fract32Class;
extern const ModClassModule awe_modPowFract32Class;
extern const ModClassModule awe_modPulseGenClass;
extern const ModClassModule awe_modPulseGenFract32Class;
extern const ModClassModule awe_modRMSClass;
extern const ModClassModule awe_modRMSFract32Class;
extern const ModClassModule awe_modRMSNClass;
extern const ModClassModule awe_modRandClass;
extern const ModClassModule awe_modReciprocalClass;
extern const ModClassModule awe_modReciprocalFract32Class;
extern const ModClassModule awe_modRemainderClass;
extern const ModClassModule awe_modRoundClass;
extern const ModClassModule awe_modRouterClass;
extern const ModClassModule awe_modRouterSmoothedClass;
extern const ModClassModule awe_modRouterSmoothedFract32Class;
extern const ModClassModule awe_modSMixer2x1Class;
extern const ModClassModule awe_modSMixer2x1Fract32Class;
extern const ModClassModule awe_modSOFSmoothedCascadeFract32Class;
extern const ModClassModule awe_modSawtoothFract32Class;
extern const ModClassModule awe_modScaleOffsetClass;
extern const ModClassModule awe_modScaleOffsetFract32Class;
extern const ModClassModule awe_modScaleOffsetInt32Class;
extern const ModClassModule awe_modScalerControlClass;
extern const ModClassModule awe_modScalerControlBlockSmoothedFract32Class;
extern const ModClassModule awe_modScalerControlSmoothedFract32Class;
extern const ModClassModule awe_modScalerDBControlClass;
extern const ModClassModule awe_modScalerDBFract32Class;
extern const ModClassModule awe_modScalerDBSmoothedFract32Class;
extern const ModClassModule awe_modScalerFract32Class;
extern const ModClassModule awe_modScalerNDBSmoothedFract32Class;
extern const ModClassModule awe_modScalerNFract32Class;
extern const ModClassModule awe_modScalerNSmoothedFract32Class;
extern const ModClassModule awe_modScalerNV2Class;
extern const ModClassModule awe_modScalerSmoothedFract32Class;
extern const ModClassModule awe_modScalerV2Class;
extern const ModClassModule awe_modSecondOrderFilterSmoothedClass;
extern const ModClassModule awe_modSecondOrderFilterSmoothedCascadeClass;
extern const ModClassModule awe_modSecondOrderFilterSmoothedFract32Class;
extern const ModClassModule awe_modShiftFract32Class;
extern const ModClassModule awe_modSignClass;
extern const ModClassModule awe_modSinClass;
extern const ModClassModule awe_modSineGenControlFract32Class;
extern const ModClassModule awe_modSineGenFract32Class;
extern const ModClassModule awe_modSineSmoothedGenClass;
extern const ModClassModule awe_modSineSmoothedGenFract32Class;
extern const ModClassModule awe_modSinhClass;
extern const ModClassModule awe_modSinkClass;
extern const ModClassModule awe_modSinkFract32Class;
extern const ModClassModule awe_modSinkIntClass;
extern const ModClassModule awe_modSofV2HpFract32Class;
extern const ModClassModule awe_modSoftClipClass;
extern const ModClassModule awe_modSoftClipFract32Class;
extern const ModClassModule awe_modSoloMuteClass;
extern const ModClassModule awe_modSoloMuteFract32Class;
extern const ModClassModule awe_modSourceClass;
extern const ModClassModule awe_modSourceFract32Class;
extern const ModClassModule awe_modSourceIntClass;
extern const ModClassModule awe_modSqrtClass;
extern const ModClassModule awe_modSqrtFract32Class;
extern const ModClassModule awe_modSquareClass;
extern const ModClassModule awe_modSquareAddClass;
extern const ModClassModule awe_modSquareAddFract32Class;
extern const ModClassModule awe_modSquareFract32Class;
extern const ModClassModule awe_modSubtractClass;
extern const ModClassModule awe_modSubtractFract32Class;
extern const ModClassModule awe_modSubtractInt32Class;
extern const ModClassModule awe_modSumDiffClass;
extern const ModClassModule awe_modSumDiffFract32Class;
extern const ModClassModule awe_modSumDiffInt32Class;
extern const ModClassModule awe_modTableInterpClass;
extern const ModClassModule awe_modTableInterpFract32Class;
extern const ModClassModule awe_modTableLookupClass;
extern const ModClassModule awe_modTanClass;
extern const ModClassModule awe_modTanhClass;
extern const ModClassModule awe_modThreeBandToneControlClass;
extern const ModClassModule awe_modThreeBandToneControlFract32Class;
extern const ModClassModule awe_modToggleButtonClass;
extern const ModClassModule awe_modTypeConversionClass;
extern const ModClassModule awe_modUndb10Class;
extern const ModClassModule awe_modUndb10ApproxClass;
extern const ModClassModule awe_modUndb10Fract32Class;
extern const ModClassModule awe_modUndb20Class;
extern const ModClassModule awe_modUndb20ApproxClass;
extern const ModClassModule awe_modUndb20Fract32Class;
extern const ModClassModule awe_modVolumeControlClass;
extern const ModClassModule awe_modVolumeControlFract32Class;
extern const ModClassModule awe_modWetDryClass;
extern const ModClassModule awe_modWhiteNoiseClass;
extern const ModClassModule awe_modWhiteNoiseFract32Class;
extern const ModClassModule awe_modWithinRangeClass;
extern const ModClassModule awe_modWithinRangeFract32Class;
extern const ModClassModule awe_modZeroCrossingDetectorClass;
extern const ModClassModule awe_modZeroCrossingDetectorFract32Class;
extern const ModClassModule awe_modZeroSourceClass;

// Advanced library modules
extern const ModClassModule awe_modAGCAttackReleaseClass;
extern const ModClassModule awe_modAGCAttackReleaseFract32Class;
extern const ModClassModule awe_modAGCCompressorCoreClass;
extern const ModClassModule awe_modAGCCompressorCoreFract32Class;
extern const ModClassModule awe_modAGCCoreFract32Class;
extern const ModClassModule awe_modAGCVariableAttackReleaseClass;
extern const ModClassModule awe_modAllpassDelayClass;
extern const ModClassModule awe_modAllpassDelay16Class;
extern const ModClassModule awe_modAllpassDelay16Fract32Class;
extern const ModClassModule awe_modAllpassDelayFract32Class;
extern const ModClassModule awe_modAllpassDelayciClass;
extern const ModClassModule awe_modAllpassDelayiClass;
extern const ModClassModule awe_modAttackDecaySustainReleaseClass;
extern const ModClassModule awe_modAudioWeightingClass;
extern const ModClassModule awe_modAudioWeightingFract32Class;
extern const ModClassModule awe_modAveragerClass;
extern const ModClassModule awe_modAveragerFract32Class;
extern const ModClassModule awe_modBiquadClass;
extern const ModClassModule awe_modBiquadCascadeClass;
extern const ModClassModule awe_modBiquadCascadeFract32Class;
extern const ModClassModule awe_modBiquadCascadeHPClass;
extern const ModClassModule awe_modBiquadLoadingClass;
extern const ModClassModule awe_modBiquadLoadingFract32Class;
extern const ModClassModule awe_modBiquadSmoothedClass;
extern const ModClassModule awe_modBiquadSmoothedFract32Class;
extern const ModClassModule awe_modBiquadSmoothedHPClass;
extern const ModClassModule awe_modBlockConcatenateClass;
extern const ModClassModule awe_modBlockDelayClass;
extern const ModClassModule awe_modBlockExtractClass;
extern const ModClassModule awe_modBlockFlipClass;
extern const ModClassModule awe_modBlockMedianClass;
extern const ModClassModule awe_modBlockStatisticsClass;
extern const ModClassModule awe_modBlockStatisticsFract32Class;
extern const ModClassModule awe_modButterworthFilterHPClass;
extern const ModClassModule awe_modCfftClass;
extern const ModClassModule awe_modCfftFract32Class;
extern const ModClassModule awe_modCoeffTableClass;
extern const ModClassModule awe_modComplexAngleClass;
extern const ModClassModule awe_modComplexConjugateClass;
extern const ModClassModule awe_modComplexMagnitudeClass;
extern const ModClassModule awe_modComplexMagnitudeFract32Class;
extern const ModClassModule awe_modComplexMagnitudeSquaredClass;
extern const ModClassModule awe_modComplexModulateClass;
extern const ModClassModule awe_modComplexMultiplierClass;
extern const ModClassModule awe_modComplexToPolarClass;
extern const ModClassModule awe_modComplexToRealImagClass;
extern const ModClassModule awe_modComplexToRealImagFract32Class;
extern const ModClassModule awe_modConsecutiveCountIntClass;
extern const ModClassModule awe_modConvolveClass;
extern const ModClassModule awe_modCorrelateClass;
extern const ModClassModule awe_modCorrelationFract32Class;
extern const ModClassModule awe_modCycleBurnerClass;
extern const ModClassModule awe_modDateTimeClass;
extern const ModClassModule awe_modDeadBandClass;
extern const ModClassModule awe_modDelayClass;
extern const ModClassModule awe_modDelay16Class;
extern const ModClassModule awe_modDelayInterpClass;
extern const ModClassModule awe_modDelayInterpFract32Class;
extern const ModClassModule awe_modDelayNTapClass;
extern const ModClassModule awe_modDelayNTap16Fract32Class;
extern const ModClassModule awe_modDelayReaderClass;
extern const ModClassModule awe_modDelayStateWriterClass;
extern const ModClassModule awe_modDelayStateWriter16Class;
extern const ModClassModule awe_modDelayciFract32Class;
extern const ModClassModule awe_modDerivativeClass;
extern const ModClassModule awe_modDerivativeFract32Class;
extern const ModClassModule awe_modDitherFract32Class;
extern const ModClassModule awe_modDownsamplerClass;
extern const ModClassModule awe_modDownwardExpanderCoreClass;
extern const ModClassModule awe_modDownwardExpanderCoreFract32Class;
extern const ModClassModule awe_modDuckerClass;
extern const ModClassModule awe_modDuckerFract32Class;
extern const ModClassModule awe_modEmphasisFilterClass;
extern const ModClassModule awe_modEmphasisFilterFract32Class;
extern const ModClassModule awe_modFIRClass;
extern const ModClassModule awe_modFIRDecimatorClass;
extern const ModClassModule awe_modFIRDecimatorFract32Class;
extern const ModClassModule awe_modFIRFract32Class;
extern const ModClassModule awe_modFIRInterpolatorClass;
extern const ModClassModule awe_modFIRInterpolatorFract32Class;
extern const ModClassModule awe_modFIRLoadingClass;
extern const ModClassModule awe_modFIRLoadingFract32Class;
extern const ModClassModule awe_modFIRSparseClass;
extern const ModClassModule awe_modFIRSparseReaderClass;
extern const ModClassModule awe_modFIRSparseReaderFract16Class;
extern const ModClassModule awe_modFIRSparseReaderFract32Class;
extern const ModClassModule awe_modFOFControlClass;
extern const ModClassModule awe_modFOFControlFract32Class;
extern const ModClassModule awe_modFftClass;
extern const ModClassModule awe_modFftFract32Class;
extern const ModClassModule awe_modFifoInClass;
extern const ModClassModule awe_modFifoOutClass;
extern const ModClassModule awe_modFractionalDelayFract32Class;
extern const ModClassModule awe_modGraphicEQBandHPClass;
extern const ModClassModule awe_modGraphicEQBandHpFract32Class;
extern const ModClassModule awe_modHistogramClass;
extern const ModClassModule awe_modHysteresisClass;
extern const ModClassModule awe_modIfftClass;
extern const ModClassModule awe_modIfftFract32Class;
extern const ModClassModule awe_modIntegralClass;
extern const ModClassModule awe_modInterleavedSinkClass;
extern const ModClassModule awe_modInvertNClass;
extern const ModClassModule awe_modLMSClass;
extern const ModClassModule awe_modLMSNormFract32Class;
extern const ModClassModule awe_modLPF1ControlClass;
extern const ModClassModule awe_modMathExceptionClass;
extern const ModClassModule awe_modMeasurementClass;
extern const ModClassModule awe_modMuteSyncClass;
extern const ModClassModule awe_modMuteSyncFract32Class;
extern const ModClassModule awe_modMuteUnmuteClass;
extern const ModClassModule awe_modMuteUnmuteFract32Class;
extern const ModClassModule awe_modOscillatorClass;
extern const ModClassModule awe_modOverlapAddClass;
extern const ModClassModule awe_modOverlapAddFract32Class;
extern const ModClassModule awe_modPeriodicStreamingClass;
extern const ModClassModule awe_modPolarToComplexClass;
extern const ModClassModule awe_modPrivateHWInClass;
extern const ModClassModule awe_modPrivateHWOutClass;
extern const ModClassModule awe_modProbeFract32Class;
extern const ModClassModule awe_modRandClass;
extern const ModClassModule awe_modRandiClass;
extern const ModClassModule awe_modRandiFract32Class;
extern const ModClassModule awe_modRealImagToComplexClass;
extern const ModClassModule awe_modRealImagToComplexFract32Class;
extern const ModClassModule awe_modRebufferClass;
extern const ModClassModule awe_modRepWinOverlapClass;
extern const ModClassModule awe_modRunningMinMaxClass;
extern const ModClassModule awe_modRunningMinMaxFract32Class;
extern const ModClassModule awe_modRunningStatisticsClass;
extern const ModClassModule awe_modRunningStatisticsFract32Class;
extern const ModClassModule awe_modSOFCascadeHPClass;
extern const ModClassModule awe_modSOFControlClass;
extern const ModClassModule awe_modSOFControlFract32Class;
extern const ModClassModule awe_modSOFControlHPClass;
extern const ModClassModule awe_modSampleAndHoldClass;
extern const ModClassModule awe_modSampleAndHoldInt32Class;
extern const ModClassModule awe_modSampleMultiplexorControlClass;
extern const ModClassModule awe_modSampleRateClass;
extern const ModClassModule awe_modSampleStatisticsClass;
extern const ModClassModule awe_modSampleStatisticsFract32Class;
extern const ModClassModule awe_modSbAttackReleaseClass;
extern const ModClassModule awe_modSbComplexFIRClass;
extern const ModClassModule awe_modSbComplexFIRvlClass;
extern const ModClassModule awe_modSbDerivativeClass;
extern const ModClassModule awe_modSbNLMSClass;
extern const ModClassModule awe_modSbRMSClass;
extern const ModClassModule awe_modSbSOFClass;
extern const ModClassModule awe_modSbSmoothClass;
extern const ModClassModule awe_modSbSplitterClass;
extern const ModClassModule awe_modScalerFract32Class;
extern const ModClassModule awe_modScalerV2Class;
extern const ModClassModule awe_modSecondOrderFilterHPClass;
extern const ModClassModule awe_modSecondOrderFilterSmoothedFract32Class;
extern const ModClassModule awe_modSetWirePropertiesClass;
extern const ModClassModule awe_modShiftSamplesClass;
extern const ModClassModule awe_modSlewRateLimiterClass;
extern const ModClassModule awe_modStatusSetClass;
extern const ModClassModule awe_modSubblockStatisticsClass;
extern const ModClassModule awe_modSubblockStatisticsFract32Class;
extern const ModClassModule awe_modTableInterp2dClass;
extern const ModClassModule awe_modTableLookupIntFloatClass;
extern const ModClassModule awe_modTableLookupIntIntClass;
extern const ModClassModule awe_modTableLookupLogXClass;
extern const ModClassModule awe_modTimeOutClass;
extern const ModClassModule awe_modTimeOutFract32Class;
extern const ModClassModule awe_modTriggeredSinkClass;
extern const ModClassModule awe_modTwoPieceInterpV2Class;
extern const ModClassModule awe_modUnwrapClass;
extern const ModClassModule awe_modUpdateSampleRateClass;
extern const ModClassModule awe_modUpsamplerClass;
extern const ModClassModule awe_modVolumeControlHPClass;
extern const ModClassModule awe_modWAVInterp16OneShotSourceFract32Class;
extern const ModClassModule awe_modWhiteNoiseFract32Class;
extern const ModClassModule awe_modWindowClass;
extern const ModClassModule awe_modWindowAliasClass;
extern const ModClassModule awe_modWindowAliasFract32Class;
extern const ModClassModule awe_modWindowFract32Class;
extern const ModClassModule awe_modWindowOverlapClass;
extern const ModClassModule awe_modWindowOverlapFract32Class;
extern const ModClassModule awe_modZeroPaddingClass;
extern const ModClassModule awe_modZeroPaddingFract32Class;

//#define LISTOFCLASSOBJECTS \
//&awe_modSensoryTHFCortexMClass, \
//&awe_modSensoryTHFCortexMCmdsClass


#define LISTOFCLASSOBJECTS \
&awe_modSensoryTHFCortexMClass, \
&awe_modSensoryTHFCortexMCmdsClass, \
&awe_modAGCCoreClass, \
&awe_modAGCLimiterCoreClass, \
&awe_modAGCLimiterCoreFract32Class, \
&awe_modAGCMultiplierFract32Class, \
&awe_modAGCNoiseGateCoreClass, \
&awe_modAbsClass, \
&awe_modAbsFract32Class, \
&awe_modAcosClass, \
&awe_modAdderClass, \
&awe_modAdderFract32Class, \
&awe_modAdderInt32Class, \
&awe_modAsinClass, \
&awe_modAtanClass, \
&awe_modAtan2Class, \
&awe_modBalanceClass, \
&awe_modBalanceFract32Class, \
&awe_modBiquadClass, \
&awe_modBiquadCascadeClass, \
&awe_modBiquadCascadeFract32Class, \
&awe_modBiquadFract32Class, \
&awe_modBiquadNCascadeClass, \
&awe_modBiquadSmoothedClass, \
&awe_modBiquadSmoothedFract32Class, \
&awe_modBiquadV2Fract32Class, \
&awe_modBitsToIntClass, \
&awe_modBlockCounterClass, \
&awe_modBooleanInvertClass, \
&awe_modBooleanSinkClass, \
&awe_modBooleanSourceClass, \
&awe_modButterworthFilterClass, \
&awe_modButterworthFilterFract32Class, \
&awe_modCeilClass, \
&awe_modClipAsymClass, \
&awe_modClipAsymFract32Class, \
&awe_modClipIndicatorClass, \
&awe_modClipIndicatorFract32Class, \
&awe_modCopierClass, \
&awe_modCosClass, \
&awe_modCoshClass, \
&awe_modCounterClass, \
&awe_modCrossFaderClass, \
&awe_modDCSourceIntClass, \
&awe_modDCSourceV2Class, \
&awe_modDCSourceV2Fract32Class, \
&awe_modDb10Class, \
&awe_modDb10ApproxClass, \
&awe_modDb20Class, \
&awe_modDb20ApproxClass, \
&awe_modDb20Fract32Class, \
&awe_modDebounceClass, \
&awe_modDeinterleaveClass, \
&awe_modDelayClass, \
&awe_modDelayMsecClass, \
&awe_modDivideClass, \
&awe_modExpClass, \
&awe_modFIRClass, \
&awe_modFIRFract32Class, \
&awe_modFloorClass, \
&awe_modFmodClass, \
&awe_modFrexpClass, \
&awe_modGraphicEQBandClass, \
&awe_modGraphicEQBandFract32Class, \
&awe_modImpulseMsecSourceClass, \
&awe_modImpulseMsecSourceFract32Class, \
&awe_modImpulseSourceFract32Class, \
&awe_modIntToBitsClass, \
&awe_modInterleaveClass, \
&awe_modInvertClass, \
&awe_modInvertFract32Class, \
&awe_modLdexpClass, \
&awe_modLogClass, \
&awe_modLog10Class, \
&awe_modLog10Fract32Class, \
&awe_modLog2Class, \
&awe_modLog2Fract32Class, \
&awe_modLogicAllClass, \
&awe_modLogicAndConstFract32Class, \
&awe_modLogicAnyClass, \
&awe_modLogicBinaryOpClass, \
&awe_modLogicCompareClass, \
&awe_modLogicCompareConstClass, \
&awe_modLogicCompareConstFract32Class, \
&awe_modLogicCompareConstInt32Class, \
&awe_modLogicCompareFract32Class, \
&awe_modLogicCompareInt32Class, \
&awe_modMaxAbsClass, \
&awe_modMaxAbsFract32Class, \
&awe_modMeterClass, \
&awe_modMeterFract32Class, \
&awe_modMixerDenseFract32Class, \
&awe_modMixerFract32Class, \
&awe_modMixerSmoothedFract32Class, \
&awe_modMixerV3Class, \
&awe_modMixerV3Fract32Class, \
&awe_modModfClass, \
&awe_modMultiplexorClass, \
&awe_modMultiplexorV2Class, \
&awe_modMultiplexorV2Fract32Class, \
&awe_modMultiplierFract32Class, \
&awe_modMultiplierV2Class, \
&awe_modMultiplierV2Fract32Class, \
&awe_modMuteNSmoothedClass, \
&awe_modMuteNSmoothedFract32Class, \
&awe_modMuteSmoothedClass, \
&awe_modMuteSmoothedFract32Class, \
&awe_modMuteUnmuteClass, \
&awe_modMuteUnmuteFract32Class, \
&awe_modNullSinkClass, \
&awe_modParamGetClass, \
&awe_modParamSetClass, \
&awe_modPeriodicFunctionGenClass, \
&awe_modPeriodicSourceClass, \
&awe_modPeriodicSourceFract32Class, \
&awe_modPolynomialClass, \
&awe_modPolynomialFract32Class, \
&awe_modPowClass, \
&awe_modPow10toXClass, \
&awe_modPow2Fract32Class, \
&awe_modPowFract32Class, \
&awe_modPulseGenClass, \
&awe_modPulseGenFract32Class, \
&awe_modRMSClass, \
&awe_modRMSFract32Class, \
&awe_modRandClass, \
&awe_modReciprocalClass, \
&awe_modReciprocalFract32Class, \
&awe_modRemainderClass, \
&awe_modRoundClass, \
&awe_modRouterClass, \
&awe_modRouterSmoothedClass, \
&awe_modRouterSmoothedFract32Class, \
&awe_modSMixer2x1Class, \
&awe_modSMixer2x1Fract32Class, \
&awe_modSOFSmoothedCascadeFract32Class, \
&awe_modSawtoothFract32Class, \
&awe_modScaleOffsetClass, \
&awe_modScaleOffsetFract32Class, \
&awe_modScaleOffsetInt32Class, \
&awe_modScalerControlClass, \
&awe_modScalerControlBlockSmoothedFract32Class, \
&awe_modScalerControlSmoothedFract32Class, \
&awe_modScalerDBControlClass, \
&awe_modScalerDBFract32Class, \
&awe_modScalerDBSmoothedFract32Class, \
&awe_modScalerFract32Class, \
&awe_modScalerNDBSmoothedFract32Class, \
&awe_modScalerNFract32Class, \
&awe_modScalerNSmoothedFract32Class, \
&awe_modScalerNV2Class, \
&awe_modScalerSmoothedFract32Class, \
&awe_modScalerV2Class, \
&awe_modSecondOrderFilterSmoothedClass, \
&awe_modSecondOrderFilterSmoothedCascadeClass, \
&awe_modSecondOrderFilterSmoothedFract32Class, \
&awe_modShiftFract32Class, \
&awe_modSignClass, \
&awe_modSinClass, \
&awe_modSineGenControlFract32Class, \
&awe_modSineGenFract32Class, \
&awe_modSineSmoothedGenClass, \
&awe_modSineSmoothedGenFract32Class, \
&awe_modSinhClass, \
&awe_modSinkClass, \
&awe_modSinkFract32Class, \
&awe_modSinkIntClass, \
&awe_modSofV2HpFract32Class, \
&awe_modSoftClipClass, \
&awe_modSoftClipFract32Class, \
&awe_modSoloMuteClass, \
&awe_modSoloMuteFract32Class, \
&awe_modSourceClass, \
&awe_modSourceFract32Class, \
&awe_modSourceIntClass, \
&awe_modSqrtClass, \
&awe_modSqrtFract32Class, \
&awe_modSquareClass, \
&awe_modSquareAddClass, \
&awe_modSquareAddFract32Class, \
&awe_modSquareFract32Class, \
&awe_modSubtractClass, \
&awe_modSubtractFract32Class, \
&awe_modSubtractInt32Class, \
&awe_modSumDiffClass, \
&awe_modSumDiffFract32Class, \
&awe_modSumDiffInt32Class, \
&awe_modTableInterpClass, \
&awe_modTableInterpFract32Class, \
&awe_modTableLookupClass, \
&awe_modTanClass, \
&awe_modTanhClass, \
&awe_modThreeBandToneControlClass, \
&awe_modThreeBandToneControlFract32Class, \
&awe_modToggleButtonClass, \
&awe_modTypeConversionClass, \
&awe_modUndb10Class, \
&awe_modUndb10ApproxClass, \
&awe_modUndb10Fract32Class, \
&awe_modUndb20Class, \
&awe_modUndb20ApproxClass, \
&awe_modUndb20Fract32Class, \
&awe_modVolumeControlClass, \
&awe_modVolumeControlFract32Class, \
&awe_modWetDryClass, \
&awe_modWhiteNoiseClass, \
&awe_modWhiteNoiseFract32Class, \
&awe_modWithinRangeClass, \
&awe_modWithinRangeFract32Class, \
&awe_modZeroCrossingDetectorClass, \
&awe_modZeroCrossingDetectorFract32Class, \
&awe_modZeroSourceClass, \
&awe_modAGCAttackReleaseClass, \
&awe_modAGCAttackReleaseFract32Class, \
&awe_modAGCCompressorCoreClass, \
&awe_modAGCCompressorCoreFract32Class, \
&awe_modAGCCoreFract32Class, \
&awe_modAGCVariableAttackReleaseClass, \
&awe_modAllpassDelayClass, \
&awe_modAllpassDelay16Class, \
&awe_modAllpassDelay16Fract32Class, \
&awe_modAllpassDelayFract32Class, \
&awe_modAllpassDelayciClass, \
&awe_modAllpassDelayiClass, \
&awe_modAttackDecaySustainReleaseClass, \
&awe_modAudioWeightingClass, \
&awe_modAudioWeightingFract32Class, \
&awe_modAveragerClass, \
&awe_modAveragerFract32Class, \
&awe_modBiquadClass, \
&awe_modBiquadCascadeClass, \
&awe_modBiquadCascadeFract32Class, \
&awe_modBiquadCascadeHPClass, \
&awe_modBiquadLoadingClass, \
&awe_modBiquadLoadingFract32Class, \
&awe_modBiquadSmoothedClass, \
&awe_modBiquadSmoothedFract32Class, \
&awe_modBiquadSmoothedHPClass, \
&awe_modBlockConcatenateClass, \
&awe_modBlockDelayClass, \
&awe_modBlockExtractClass, \
&awe_modBlockFlipClass, \
&awe_modBlockMedianClass, \
&awe_modBlockStatisticsClass, \
&awe_modBlockStatisticsFract32Class, \
&awe_modButterworthFilterHPClass, \
&awe_modCfftClass, \
&awe_modCfftFract32Class, \
&awe_modCoeffTableClass, \
&awe_modComplexAngleClass, \
&awe_modComplexConjugateClass, \
&awe_modComplexMagnitudeClass, \
&awe_modComplexMagnitudeFract32Class, \
&awe_modComplexMagnitudeSquaredClass, \
&awe_modComplexModulateClass, \
&awe_modComplexMultiplierClass, \
&awe_modComplexToPolarClass, \
&awe_modComplexToRealImagClass, \
&awe_modComplexToRealImagFract32Class, \
&awe_modConsecutiveCountIntClass, \
&awe_modConvolveClass, \
&awe_modCorrelateClass, \
&awe_modCorrelationFract32Class, \
&awe_modCycleBurnerClass, \
&awe_modDateTimeClass, \
&awe_modDeadBandClass, \
&awe_modDelayClass, \
&awe_modDelay16Class, \
&awe_modDelayInterpClass, \
&awe_modDelayInterpFract32Class, \
&awe_modDelayNTapClass, \
&awe_modDelayNTap16Fract32Class, \
&awe_modDelayReaderClass, \
&awe_modDelayStateWriterClass, \
&awe_modDelayStateWriter16Class, \
&awe_modDelayciFract32Class, \
&awe_modDerivativeClass, \
&awe_modDerivativeFract32Class, \
&awe_modDitherFract32Class, \
&awe_modDownsamplerClass, \
&awe_modDownwardExpanderCoreClass, \
&awe_modDownwardExpanderCoreFract32Class, \
&awe_modDuckerClass, \
&awe_modDuckerFract32Class, \
&awe_modEmphasisFilterClass, \
&awe_modEmphasisFilterFract32Class, \
&awe_modFIRClass, \
&awe_modFIRDecimatorClass, \
&awe_modFIRDecimatorFract32Class, \
&awe_modFIRFract32Class, \
&awe_modFIRInterpolatorClass, \
&awe_modFIRInterpolatorFract32Class, \
&awe_modFIRLoadingClass, \
&awe_modFIRLoadingFract32Class, \
&awe_modFIRSparseClass, \
&awe_modFIRSparseReaderClass, \
&awe_modFIRSparseReaderFract16Class, \
&awe_modFIRSparseReaderFract32Class, \
&awe_modFOFControlClass, \
&awe_modFOFControlFract32Class, \
&awe_modFftClass, \
&awe_modFftFract32Class, \
&awe_modFifoInClass, \
&awe_modFifoOutClass, \
&awe_modFractionalDelayFract32Class, \
&awe_modGraphicEQBandHPClass, \
&awe_modGraphicEQBandHpFract32Class, \
&awe_modHistogramClass, \
&awe_modHysteresisClass, \
&awe_modIfftClass, \
&awe_modIfftFract32Class, \
&awe_modIntegralClass, \
&awe_modInterleavedSinkClass, \
&awe_modInvertNClass, \
&awe_modLMSClass, \
&awe_modLMSNormFract32Class, \
&awe_modLPF1ControlClass, \
&awe_modMathExceptionClass, \
&awe_modMeasurementClass, \
&awe_modMuteSyncClass, \
&awe_modMuteSyncFract32Class, \
&awe_modMuteUnmuteClass, \
&awe_modMuteUnmuteFract32Class, \
&awe_modOscillatorClass, \
&awe_modOverlapAddClass, \
&awe_modOverlapAddFract32Class, \
&awe_modPeriodicStreamingClass, \
&awe_modPolarToComplexClass, \
&awe_modProbeFract32Class, \
&awe_modRandClass, \
&awe_modRandiClass, \
&awe_modRandiFract32Class, \
&awe_modRealImagToComplexClass, \
&awe_modRealImagToComplexFract32Class, \
&awe_modRebufferClass, \
&awe_modRepWinOverlapClass, \
&awe_modRunningMinMaxClass, \
&awe_modRunningMinMaxFract32Class, \
&awe_modRunningStatisticsClass, \
&awe_modRunningStatisticsFract32Class, \
&awe_modSOFCascadeHPClass, \
&awe_modSOFControlClass, \
&awe_modSOFControlFract32Class, \
&awe_modSOFControlHPClass, \
&awe_modSampleAndHoldClass, \
&awe_modSampleAndHoldInt32Class, \
&awe_modSampleMultiplexorControlClass, \
&awe_modSampleRateClass, \
&awe_modSampleStatisticsClass, \
&awe_modSampleStatisticsFract32Class, \
&awe_modSbAttackReleaseClass, \
&awe_modSbComplexFIRClass, \
&awe_modSbComplexFIRvlClass, \
&awe_modSbDerivativeClass, \
&awe_modSbNLMSClass, \
&awe_modSbRMSClass, \
&awe_modSbSOFClass, \
&awe_modSbSmoothClass, \
&awe_modSbSplitterClass, \
&awe_modScalerFract32Class, \
&awe_modScalerV2Class, \
&awe_modSecondOrderFilterHPClass, \
&awe_modSecondOrderFilterSmoothedFract32Class, \
&awe_modSetWirePropertiesClass, \
&awe_modShiftSamplesClass, \
&awe_modSlewRateLimiterClass, \
&awe_modStatusSetClass, \
&awe_modSubblockStatisticsClass, \
&awe_modSubblockStatisticsFract32Class, \
&awe_modTableInterp2dClass, \
&awe_modTableLookupIntFloatClass, \
&awe_modTableLookupIntIntClass, \
&awe_modTableLookupLogXClass, \
&awe_modTimeOutClass, \
&awe_modTimeOutFract32Class, \
&awe_modTriggeredSinkClass, \
&awe_modTwoPieceInterpV2Class, \
&awe_modUnwrapClass, \
&awe_modUpdateSampleRateClass, \
&awe_modUpsamplerClass, \
&awe_modVolumeControlHPClass, \
&awe_modWAVInterp16OneShotSourceFract32Class, \
&awe_modWhiteNoiseFract32Class, \
&awe_modWindowClass, \
&awe_modWindowAliasClass, \
&awe_modWindowAliasFract32Class, \
&awe_modWindowFract32Class, \
&awe_modWindowOverlapClass, \
&awe_modWindowOverlapFract32Class, \
&awe_modZeroPaddingClass, \
&awe_modZeroPaddingFract32Class

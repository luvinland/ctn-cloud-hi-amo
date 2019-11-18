;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/


	AREA _audio, DATA, READONLY

	EXPORT  u32audioBegin
	EXPORT  u32audioEnd
    
u32audioBegin
;    INCBIN ..\AudioRes\Output\AudioRes_AudioInfoMerge2.ROM
	INCBIN .\NumberVP_command5.ROM
u32audioEnd        
    
    
    END
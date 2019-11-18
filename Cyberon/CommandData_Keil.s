;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/


    AREA _CMDData, DATA, READONLY

    EXPORT  u32CMDDataBegin
    EXPORT  u32CMDDataEnd

u32CMDDataBegin
	;;1 group Alexa, use Alexa_Adapt_pack_withRej.bin
	;INCBIN .\Alexa_Adapt_pack_withRej.bin
	;;2 groups Alexa + ok google/hey siri , use Alexa_okgoogle_2group_pack_withRej.bin
	;INCBIN .\Alexa_okgoogle_2group_pack_withRej.bin
	;INCBIN ..\SI\Cinderella\Cinderella_Adapt_pack_withRej.bin
	INCBIN ..\SI\Alexa_KR\Alexa_KR_pack_withRej.bin
u32CMDDataEnd

	END
;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/


	AREA _audio, DATA, READONLY

	EXPORT  _1Begin
	EXPORT  _1End
	EXPORT  _2Begin
	EXPORT  _2End
;	EXPORT  _3Begin
;	EXPORT  _3End
;	EXPORT  _4Begin
;	EXPORT  _4End
;	EXPORT  _5Begin
;	EXPORT  _5End
;	EXPORT  _6Begin
;	EXPORT  _6End
;	EXPORT  _7Begin
;	EXPORT  _7End
;	EXPORT  _8Begin
;	EXPORT  _8End
;	EXPORT  _9Begin
;	EXPORT  _9End	

_1Begin
	INCBIN audio\01_power_on_8khz_add.bin
_1End  
_2Begin
	INCBIN audio\02_power_off_8khz_add_mod.bin
_2End
;_3Begin
;	INCBIN audio\3.bin
;_3End  
;_4Begin
;	INCBIN audio\4.bin
;_4End  
;_5Begin
;	INCBIN audio\5.bin
;_5End  
;_6Begin
;	INCBIN audio\6.bin
;_6End  
;_7Begin
;	INCBIN audio\02_power_off_8khz_add.bin
;_7End  
;_8Begin
;	INCBIN audio\8.bin
;_8End  
;_9Begin
;	INCBIN audio\9.bin
;_9End  
    
    END
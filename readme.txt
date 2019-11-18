/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) Nuvoton Technology Corp. All rights reserved.                                              */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
---------------------------------------------------------------------------------------------------------
Purpose:
---------------------------------------------------------------------------------------------------------
    Demonstrate voice recognition after BF+NR algorism
    (1) Voice recognition result is shown by Voice Prompt and pulling GPIO low.
		User could connect speaker or add LED for verification.
    (2) Signal before processing and after processing are compared through UAC.
		
---------------------------------------------------------------------------------------------------------
Operation:
---------------------------------------------------------------------------------------------------------
Voice recognition:
    This example is a 2-stage voice recognition, user must say key word first. After key word is recognized, user can say the commands. 
    Key word  "Cinderella"     -> Voice Prompt "Ding", and PB1 is low
    Command   "Come in please" -> Voice Prompt "Command1", and PA13 is low
    	      "Welcome"        -> Voice Prompt "Command2", and PA9 is low
              "Here you are"   -> Voice Prompt "Command3", and PA5 is low
              "Go with me"     -> Voice Prompt "Command4", and PA1 is low

Signal before and after processing comparison
    Connect USB to PC and use any recoring software to record the sound.
    Left channel is before processing; right channel is after processing.
    The processed data vlaue might larger than unprocessed data because we had adjusted the gain for voice recognition.
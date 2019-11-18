// Generated by ver. 3.50.015

#ifndef _AUDIOINFO_H_
#define _AUDIOINFO_H_

enum Audio_ID_Table
{
	01_POWER_ON=0,	//NUO
	02_POWER_OFF=1,	//NUO
	03_1DAN_START=2,	//NUO
	04_2DAN_START=3,	//NUO
	05_3DAN_START=4,	//NUO
	06_TIME_RESERVE_1HOUR=5,	//NUO
	07_TIME_RESERVE_4HOUR=6,	//NUO
	08_TIME_RESERVE_8HOUR=7,	//NUO
	09_AI_MODE=8,	//NUO
	10_UNMUTE_VR=9,	//NUO
	11_MUTE_VR=10,	//NUO
};
#define AUDIOSYN_SOUND_MAX_ID	10

// Define AudioRes_AudioInfoMerge.ROM size (without MIDI WavTable)
#define AUDIOINFO_ROM_NO_WTB_SIZE	117756
// Define MIDI WavTable size
#define MIDISYN_WTB_SIZE	0
// Define AudioRes_AudioInfoMerge.ROM size (with MIDI WavTable)
#define AUDIOINFO_ROM_SIZE	117756

#endif


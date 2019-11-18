/*******************************************************************************
*
*               Audio Framework
*               ---------------
*
********************************************************************************
*     main.c
********************************************************************************
*
*     Description:  AudioWeaver target main program
*
*******************************************************************************/
#include "AwePlatform.h"
#include "PlatformAPI.h"
#include "I94100.h"
#include "SPIFlash.h"
#include "BoardSupport.h"
//#include "mic1_mic3_70mm_in3_out3_SCNRm5dB_endfireAWB.h"
#include "MIC1_MIC2_40mm_In3_Out6_SCNRn5AWB.h"


#include "Cyb_support2.h"  //SW 
#include "..\NuOne\AudioRom.h"	
#include "..\NuOne\PlayNuOne.h"

//BOOL bdataready = FALSE;

uint32_t g_nCurrentGroup = 0;

#if 1 // Jace. 191014. Add control interface using UART0.
#define TRIGGER     0x01
#define TIMEOUT     0x02
#define POWEROFF    0x10
#define POWERON     0x11
#define FAN1        0x21
#define FAN2        0x22
#define FAN3        0x23
#define TIMER1H     0x31
#define TIMER4H     0x34
#define TIMER8H     0x38
#define AIMODE      0x41
#define UNMUTE      0x50
#define MUTE        0x51

BOOL g_nPowerOn = FALSE;
BOOL g_nMute = FALSE;
uint8_t g_nCmdId = 0;
#endif

extern uint32_t u32CMDDataBegin;

extern int32_t DoVR_sep(uint32_t lpbyCYBase, uint32_t lpbyModel, uint32_t bShowOnly, uint32_t nTimeOut, int (*funcCheckKeyPress)(void));

int32_t UnpackBin(uint8_t lpbyBin[], uint8_t *lppbyModel[], int32_t nMaxNumModel)//SW
{
    uint32_t *lpnBin = (uint32_t *)lpbyBin;
    int32_t nNumBin = lpnBin[0];
    uint32_t *lpnBinSize = lpnBin + 1;
    int32_t i;

    lppbyModel[0] = (uint8_t *)(lpnBinSize + nNumBin);
    for (i = 1; i < nNumBin; i++)
    {
        if (i >= nMaxNumModel)
            break;
        lppbyModel[i] = lppbyModel[i-1] + lpnBinSize[i-1];
    }
    
    return i;
}

#if 0 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
#if 0 // Jace. 191014. Add control interface using UART0.
int CheckKeyPress(void)//SW
{
    int nDetCnt;

    // Key 0
    nDetCnt = 0;	
    while ((GPIO_GET_IN_DATA(PA) & BIT15) == 0)
    {
        nDetCnt++;
        if (nDetCnt >= 20)
            return KEYCODE_0;
    }
    return 0;
}
#else
int CheckKeyPress(void)//SW
{
	uint8_t nRet = 0;

	if(g_nCmdId)
	{
		nRet = g_nCmdId;
		g_nCmdId = 0;
		return nRet;
	}
	return nRet;
}
#endif
#endif

volatile uint8_t g_u8Recognized = 0;
volatile uint32_t g_u32FlashCount = 0;
#if 0 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
void FlashLED(uint8_t u8Led)//SW
{
    switch(u8Led)
    {
        case 1:
            PA13=0;
            break;
        case 2:
            PA9=0;
            break;
        case 3:
            PA5=0;
            break;  
        case 4:
            PA1=0;
            break;        
    }    
    g_u8Recognized = 1;
}

#if 1 // Jace. 191014. Add control interface using UART0.
void amo_vent_rx(uint8_t cmd_id)
{
	switch(cmd_id)
	{
		case 0x10: // Power off
			g_nPowerOn = FALSE;
			g_nMute = FALSE;
			g_nCmdId = cmd_id;
			break;
		case 0x11: // Power on
			g_nPowerOn = TRUE;
			g_nCmdId = cmd_id;
			break;
		case 0x21: // Fan 1
		case 0x22: // Fan 2
		case 0x23: // Fan 3
		case 0x31: // Timer 1h
		case 0x34: // Timer 4h
		case 0x38: // Timer 8h
		case 0x41: // AI mode
			g_nCmdId = cmd_id;
			break;
		case 0x50: // Unmute
			g_nMute = FALSE;
			g_nCmdId = cmd_id;
			break;
		case 0x51: // Mute
			g_nMute = TRUE;
			g_nCmdId = cmd_id;
			break;
		default:
			printf("Error cmd\n");
			break;
	}
}
#endif
#endif

// Jace. 191018. Ignore uart noise when power on. 540ms.
static void delay(uint32_t cycles)
{
  volatile uint32_t i;

  for(i = 0UL; i < cycles ;++i)
  {
    __NOP();
  }
}

typedef struct {
	uint8_t u8DeviceAddr;
	uint8_t pau8Cmd;
	uint8_t endFlag;
} S_ESP_I2CCTRL;

#if 1 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
extern volatile S_ESP_I2CCTRL s_ESP_I2CCtrl;

void esp_i2c_command(uint8_t cmd)
{
    s_ESP_I2CCtrl.u8DeviceAddr = 0x28;
    s_ESP_I2CCtrl.pau8Cmd = cmd;
    I2C_START(I2C0);
    while(s_ESP_I2CCtrl.endFlag == 0);
    s_ESP_I2CCtrl.endFlag = 0;
}
#endif

#if 1 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
#define     EMBED_SAMPLE_RATE   8000
#define     CLOUD_SAMPLE_RATE   16000

int16_t 	*pi16AudioAdd;		    							// Audio Start Address
int16_t 	*pi16AudioEnd;			    						// Audio End Address

extern uint32_t _1Begin, _1End, _2Begin, _2End, _3Begin, _3End, _4Begin, _4End, _5Begin, _5End, _6Begin, _6End, _7Begin, _7End, _8Begin, _8End, _9Begin, _9End;

void APROM_Pcm_Play(uint8_t u8idex)
{
    uint32_t i;

    SPK_Restart(EMBED_SAMPLE_RATE);

	switch(u8idex)
	{
		case 1:
			pi16AudioAdd = (int16_t *)&_1Begin;
			pi16AudioEnd = (int16_t *)&_1End;
			break;
		case 2:
			pi16AudioAdd = (int16_t *)&_2Begin;
			pi16AudioEnd = (int16_t *)&_2End;
			break;
		/*
		case 3:
			pi16AudioAdd = (int16_t *)&_3Begin;
			pi16AudioEnd = (int16_t *)&_3End;
			break;
		case 4:
			pi16AudioAdd = (int16_t *)&_4Begin;
			pi16AudioEnd = (int16_t *)&_4End;
			break;
		case 5:
			pi16AudioAdd = (int16_t *)&_5Begin;
			pi16AudioEnd = (int16_t *)&_5End;
			break;
		case 6:
			pi16AudioAdd = (int16_t *)&_6Begin;
			pi16AudioEnd = (int16_t *)&_6End;
			break;
		case 7:
			pi16AudioAdd = (int16_t *)&_7Begin;
			pi16AudioEnd = (int16_t *)&_7End;
			break;
		case 8:
			pi16AudioAdd = (int16_t *)&_8Begin;
			pi16AudioEnd = (int16_t *)&_8End;
			break;
		case 9:
			pi16AudioAdd = (int16_t *)&_9Begin;
			pi16AudioEnd = (int16_t *)&_9End;
			break;
		*/
					
	}

	while(pi16AudioAdd < pi16AudioEnd)
	{
		if( DPWM_IS_FIFOEMPTY(DPWM) )
		{
            for(i = 0; i < 80; i++)
            {
    			DPWM_WriteMonotoStereo(pi16AudioAdd, 16);
    			pi16AudioAdd += 16;
            }
		}
        else
            DPWM_CLEAR_FIFO(DPWM);
	}

    SPK_Restart(CLOUD_SAMPLE_RATE);

}
#endif

int main(void)
{    
    INT32 retVal;
    UINT32 pPos;
    static int result=0;
#if 0 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
	uint8_t cmd_id = 0; // Jace. 191014. Add control interface using UART0.
#endif

    int32_t nRet, nKey;//SW
    uint8_t *lpbyBin[4];//0:CYBase/1:SI Model 1/2:SI Model 2/3:Rej Model    //SW      
	uint8_t u8TotalAudioNum;
	
#if 0 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
	if (! SPIFlash_Initiate())
		while(1);
	u8TotalAudioNum= AudioRom_GetAudioNum( ReadDataCallback, AUDIOROM_STORAGE_START_ADDR);
	//printf("u8TotalAudioNum: %d\n", u8TotalAudioNum);
#endif
	
	//Check and unpack bin to 4 Models(CYBase/SI1/SI2/rejection)
    if (UnpackBin((uint8_t *)&u32CMDDataBegin, lpbyBin, 4) < 4)//SW
    goto L_ERROR;

	delay(3000000UL); // Jace. 191018. Ignore uart noise when power on. 540ms.

    // Initialize AWE platform
    awe_pltInit(); 

    // Initialize the framework
    awe_fwInit(&g_AWEInstance); 

    // Load the design from the compiled in data
    retVal = awe_fwPacketExecuteArray(&g_AWEInstance, InitCommands, InitCommands_Len, &pPos);

    while(TRUE) 
    {
        // Process any platform tasks
        awe_pltTick();
                
        // Process any framework tasks
        awe_fwTick(&g_AWEInstance);

		if(!g_nMute) // Jace. 191014. Add control interface using UART0.
		{
	        if (g_nCurrentGroup == 0)
	        {
				printf("trigger ready\n");
	            nRet = DoVR_sep((uint32_t)lpbyBin[0], (uint32_t)lpbyBin[1], 0, 0, NULL/*CheckKeyPress*/);  // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
				printf("trigger nRet: %x\n", nRet);
	            
				if (nRet <= 0) continue;
	            else if (nRet >= 1 && nRet <= 3)
	            {
	                g_nCurrentGroup = 1;
					PA0 = 0; //trigger led on
					esp_i2c_command(TRIGGER);
	            }
	        }
	        else
	        {
	            nRet = DoVR_sep((uint32_t)lpbyBin[0], (uint32_t)lpbyBin[2], 0, 800, NULL/*CheckKeyPress*/);  // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
				printf("command nRet: %x\n", nRet);
	            g_nCurrentGroup = 0;

				PA0 = 1; //trigger led off

				if(nRet >= 1 && nRet <= 18)
				{
					PA1 = 0; //response led on

					if(g_nPowerOn)
					{
						switch(nRet)
						{
							case 3: // Power off
							case 4:
								g_nPowerOn = FALSE;
                                esp_i2c_command(POWEROFF);
                                APROM_Pcm_Play(2);
								break;
							case 5: // Fan 1
							case 6:
								esp_i2c_command(FAN1);
								break;
							case 7: // Fan 2
							case 8:
                                esp_i2c_command(FAN2);
								break;
							case 9: // Fan 3
							case 10:
                                esp_i2c_command(FAN3);
								break;
							case 11: // Timer 1h
							case 12:
                                esp_i2c_command(TIMER1H);
								break;
							case 13: // Timer 4h
							case 14:
                                esp_i2c_command(TIMER4H);
								break;
							case 15: // Timer 8h
							case 16:
                                esp_i2c_command(TIMER8H);
								break;
							case 17: // AI mode
							case 18:
                                esp_i2c_command(AIMODE);
								break;
							default:
                                g_u8Recognized = 1;
								break;
						}
					}
					else
					{
						switch(nRet)
						{
							case 1: // Power on
							case 2:
								g_nPowerOn = TRUE;
                                esp_i2c_command(POWERON);
                                APROM_Pcm_Play(1);
								break;
							default:
                                g_u8Recognized = 1;
								break;
						}
					}
				}
                else
                {
                    g_u8Recognized = 1;
                }
	        } 
		}
    }// End while
	
		
L_ERROR:
    while(1);
  
}// End main


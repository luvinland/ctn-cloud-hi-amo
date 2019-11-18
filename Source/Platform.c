/*******************************************************************************
*
*               Audio Framework
*               ---------------
*
********************************************************************************
*     Platform.c
********************************************************************************
*
*     Description:  AWE Platform Interface
*
*******************************************************************************/
#include "AwePlatform.h"
#include "aweOptionalFeatures.h"
#include "BoardSupport.h"
#include "I94100.h"		        /* H/W SFR definition */

#define SYSTICK_1MS	 (SystemCoreClock/1000)

AWEInstance g_AWEInstance;

/* ----------------------------------------------------------------------
** Memory heaps
** ------------------------------------------------------------------- */
AWE_FW_SLOW_ANY_CONST UINT32 g_master_heap_size = MASTER_HEAP_SIZE;
AWE_FW_SLOW_ANY_CONST UINT32 g_slow_heap_size = SLOW_HEAP_SIZE;
AWE_FW_SLOW_ANY_CONST UINT32 g_fastb_heap_size = FASTB_HEAP_SIZE;

#if defined( __ICCARM__ )
    #pragma data_alignment=4
    UINT32 g_master_heap[MASTER_HEAP_SIZE];

    #pragma data_alignment=4
	UINT32 g_fastb_heap[FASTB_HEAP_SIZE] @ ".CCMRAM_Section";

    #pragma data_alignment=4
    UINT32 g_slow_heap[SLOW_HEAP_SIZE];
#else
	#if defined( __GNUC__ )
		__attribute__((__section__(".ccmram")))
	#else
        #if defined ( __CC_ARM )
		__attribute__((__section__(".data_CCMRAM")))
        #endif
	#endif
	
/*    
	__ALIGN_BEGIN uint32_t g_fastb_heap[FASTB_HEAP_SIZE] __ALIGN_END;
    __ALIGN_BEGIN uint32_t g_master_heap[MASTER_HEAP_SIZE] __ALIGN_END;
    __ALIGN_BEGIN uint32_t g_slow_heap[SLOW_HEAP_SIZE] __ALIGN_END;
*/
//JW modified the three lines above to the following
	#if defined   (__GNUC__)        /* GNU Compiler */
	  #ifndef __ALIGN_END
		#define __ALIGN_END    __attribute__ ((aligned (4)))
	  #endif /* __ALIGN_END */
	  #ifndef __ALIGN_BEGIN
		#define __ALIGN_BEGIN
	  #endif /* __ALIGN_BEGIN */
	#else
	  #ifndef __ALIGN_END
		#define __ALIGN_END
	  #endif /* __ALIGN_END */
	  #ifndef __ALIGN_BEGIN
		#if defined   (__CC_ARM)      /* ARM Compiler */
		  #define __ALIGN_BEGIN    __align(4)
		#elif defined (__ICCARM__)    /* IAR Compiler */
		  #define __ALIGN_BEGIN
		#endif /* __CC_ARM */
	  #endif /* __ALIGN_BEGIN */
	#endif /* __GNUC__ */
	
    __ALIGN_BEGIN UINT32 g_fastb_heap[FASTB_HEAP_SIZE] __ALIGN_END;
    __ALIGN_BEGIN UINT32 g_master_heap[MASTER_HEAP_SIZE] __ALIGN_END;
    __ALIGN_BEGIN UINT32 g_slow_heap[SLOW_HEAP_SIZE] __ALIGN_END;

#endif

/* ----------------------------------------------------------------------
** Module table
** ------------------------------------------------------------------- */

/* Array of pointers to module descriptors. This is initialized at compile time.
Each item is the address of a module descriptor that we need linked in. The
linker magic is such that only those modules referenced here will be in the
final program. */
const ModClassModule *g_module_descriptor_table[] =
{
	// The suitably cast pointers to the module descriptors.
    LISTOFCLASSOBJECTS
};

AWE_MOD_SLOW_DM_DATA UINT32 g_module_descriptor_table_size = sizeof(g_module_descriptor_table) / sizeof(g_module_descriptor_table[0]);
// Audio Running Flag
volatile INT32 s_AudioRunning = 0;

volatile BOOL g_bReboot = FALSE;

volatile BOOL g_bBlinkLED4ForBoardAlive = TRUE;

/** The only input pin for this core. */
static IOPinDescriptor s_InputPin[1];

/** The only output pin for this core. */
static IOPinDescriptor s_OutputPin[1];

extern INT32 g_target_baudrate;


//-----------------------------------------------------------------------------
// METHOD:  awe_pltGetCore
// PURPOSE: Return core descriptor
//-----------------------------------------------------------------------------
CoreDescriptor * awe_pltGetCore(void *pOwner, UINT32 coreID)
{
    return NULL;

}   // End awe_pltGetCore


//-----------------------------------------------------------------------------
// METHOD:  awe_pltGetCores
// PURPOSE: Report number of cores in use on this target
//-----------------------------------------------------------------------------
int awe_pltGetCores()
{
     return 1;

}   // End awe_pltGetCores


//-----------------------------------------------------------------------------
// METHOD:  awe_pltDestroyAll
// PURPOSE: Destroy all core instances
//-----------------------------------------------------------------------------
void awe_pltDestroyAll()
{
    awe_fwDestroy(&g_AWEInstance);

}   // End awe_pltDestroyAll

//-----------------------------------------------------------------------------
// METHOD:  AWEInstanceInit
// PURPOSE: Initialize AWE Instance with target details
//-----------------------------------------------------------------------------
void AWEInstanceInit()
{
    UINT32 nInputWireInfo = INFO1_PROPS(INPUT_CHANNEL_COUNT,
                                          CHANNEL_BLOCK_SIZE_IN_SAMPLES,
                                          IS_COMPLEX,
                                          SAMPLE_SIZE_IN_BYTES);

    UINT32 nOutputWireInfo = INFO1_PROPS(OUTPUT_CHANNEL_COUNT,
                                         CHANNEL_BLOCK_SIZE_IN_SAMPLES,
                                         IS_COMPLEX,
                                         SAMPLE_SIZE_IN_BYTES);

	memset(&g_AWEInstance, 0, sizeof(AWEInstance) );

	// Point to the start and stop functions.
	g_AWEInstance.m_pAwe_pltAudioStart = awe_pltAudioStart;
	g_AWEInstance.m_pAwe_pltAudioStop = awe_pltAudioStop;

	// Point to our private pins.
	g_AWEInstance.m_pInterleavedInputPin = s_InputPin;
	g_AWEInstance.m_pInterleavedOutputPin = s_OutputPin;

	// Point to the global module table.
	g_AWEInstance.m_module_descriptor_table_size = sizeof(g_module_descriptor_table) >> 2;
	g_AWEInstance.m_pModule_descriptor_table = g_module_descriptor_table;

	// This will be core ID 100.
	g_AWEInstance.m_coreID = CORE_ID;

	awe_fwInitTargetInfo(&g_AWEInstance,
                         CORE_ID,
                         CORE_SPEED,
                         SAMPLE_SPEED,
                         "I94000",
                         PROCESSOR_TYPE_CORTEXM4,
                         HAS_FLOAT_SUPPORT,
                         HAS_FLASH_FILESYSTEM,
                         NO_HW_INPUT_PINS,
                         NO_HW_OUTPUT_PINS,
                         IS_SMP,
                         NO_THREADS_SUPPORTED,
                         FIXED_SAMPLE_RATE,
                         INPUT_CHANNEL_COUNT,
                         OUTPUT_CHANNEL_COUNT,
                         VER_DAY, VER_MONTH, VER_YEAR,
                         AWE_FRAME_SIZE,
                         MAX_COMMAND_BUFFER_LEN
                         );

	awe_fwInit_io_pins(&g_AWEInstance, 1);

	s_InputPin[0].sampleRate = FIXED_SAMPLE_RATE;
	s_InputPin[0].wireInfo1 = nInputWireInfo;
	s_InputPin[0].wireInfo3 |= CLOCK_MASTER_BIT;
	awe_SetPackedName(s_InputPin[0].m_pinName, "Input");

	s_OutputPin[0].sampleRate = FIXED_SAMPLE_RATE;
	s_OutputPin[0].wireInfo1 = nOutputWireInfo;
	awe_SetPackedName(s_OutputPin[0].m_pinName, "Output");

	// Allocate the heaps.
    g_AWEInstance.m_master_heap_size = MASTER_HEAP_SIZE;
    g_AWEInstance.m_slow_heap_size = SLOW_HEAP_SIZE;
    g_AWEInstance.m_fastb_heap_size = FASTB_HEAP_SIZE;

	g_AWEInstance.m_master_heap = g_master_heap;
	g_AWEInstance.m_slow_heap = g_slow_heap;
	g_AWEInstance.m_fastb_heap = g_fastb_heap;

}   // End AWEInstanceInit

//-----------------------------------------------------------------------------
// METHOD:  awe_pltInit
// PURPOSE: Initialize AWE
//-----------------------------------------------------------------------------
AWE_OPTIMIZE_FOR_SPACE
AWE_FW_SLOW_CODE
void awe_pltInit(void)
{
    // Initialize the target info
    AWEInstanceInit();

    // Setup processor clocks, signal routing, timers, GPIO, PDMA, Memory, etc.
    I9400_System_Init();

    // Setup audio DMA, interrupt priorities, etc.
    AudioInit();

	// Initialize the communications handler
  	awe_fwTuningInit(s_PacketBuffer, MAX_COMMAND_BUFFER_LEN);

}	// End awe_pltInit


//-----------------------------------------------------------------------------
// METHOD:  awe_pltTick
// PURPOSE: Platform heartbeat tick
//-----------------------------------------------------------------------------
AWE_OPTIMIZE_FOR_SPEED
AWE_FW_SLOW_CODE
void awe_pltTick(void)
{
    BOOL bReplyReady;

    // Indicate that this idle loop call is getting CPU attention
    g_nPumpCount = 0;

    bReplyReady = awe_fwTuningTick(&g_AWEInstance);

    if (bReplyReady == REPLY_READY)
    {
#if 0 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
        UART0SendReply();
#endif

        if (g_bReboot)
        {
            g_bReboot = FALSE;
            //HAL_Delay(500);
            //HAL_NVIC_SystemReset();
        }
    }

    // If baud rate was changed wait for reply packet to be sent
    // and then change to the new baud rate
    if (g_bBaudRateChanged && g_bPacketSendComplete)
    {
        g_bBaudRateChanged = FALSE;

        // Initialize RS232 port at the new baudrate
#if 0 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
        UART0_Init(g_target_baudrate);
#endif
    }

    // Process any local controls
    //ProcessControlIO();

}	// End awe_pltTick


//-----------------------------------------------------------------------------
// METHOD:  awe_pltGetCycleCount
// PURPOSE: Returns the current value in the counter
//-----------------------------------------------------------------------------
UINT32 awe_pltGetCycleCount(void)
{
	UINT32 nCycles, nElapsedCycles;
    UINT32 nElapsedMilliSecs = 0;

    // This value is SYSTICK_1MS
    UINT32 nReloadValue = SysTick->LOAD + 1;

    nElapsedMilliSecs = g_msTicks;

    //Current COUNTDOWN value ( SYSTICK_1MS-1 )
	nCycles = SysTick->VAL;

    nElapsedCycles = (nElapsedMilliSecs * nReloadValue) + nReloadValue - nCycles;

	return nElapsedCycles;

}   // End awe_pltGetCycleCount


//-----------------------------------------------------------------------------
// METHOD:  awe_pltElapsedCycles
// PURPOSE: Returns the cycle count between start time and end time
//-----------------------------------------------------------------------------
UINT32 awe_pltElapsedCycles(UINT32 nStartTime, UINT32 nEndTime)
{
    UINT32 nElapsedTime;

    if (nEndTime > nStartTime)
    {
        nElapsedTime = nEndTime - nStartTime;
    }
    else
    {
        // Wrap around occurred
        nElapsedTime = ((((UINT32)0xFFFFFFFF) - nStartTime) + nEndTime + 1);

        // Correct for race condition reading ms tick and elapsed cycle count
        if ( (nElapsedTime > SYSTICK_1MS) && (nElapsedTime < (SYSTICK_1MS*2)) )
        {
          nElapsedTime -= SYSTICK_1MS;
        }
        else if (nElapsedTime > ( (UINT32)0xFFFFFFFF - SYSTICK_1MS) )
        {
          nElapsedTime += SYSTICK_1MS;
        }
    }

    return nElapsedTime;

}   // End awe_pltElapsedCycles


//----------------------------------------------------------------------------
// METHOD:  awe_pltAudioStart
// PURPOSE: Start processing audio with AWE
//----------------------------------------------------------------------------
AWE_OPTIMIZE_FOR_SPACE
AWE_FW_SLOW_CODE
INT32 awe_pltAudioStart(void)
{
    // At this point the model is fully instantiated and the control I/O can be setup
    //ControlInit();

    // Mark the audio as started
    s_AudioRunning = 1;

    return 0;

}   // End awe_pltAudioStart


//----------------------------------------------------------------------------
// METHOD:  awe_pltAudioStop
// PURPOSE: Stop processing audio with AWE
//----------------------------------------------------------------------------
AWE_OPTIMIZE_FOR_SPACE
AWE_FW_SLOW_CODE
INT32 awe_pltAudioStop(void)
{
    // Mark the audio as stopped
    s_AudioRunning = 0;

    return 0;

}   // End awe_pltAudioStop

INT32 awe_fwResetFileSystem(void)  
{
  return E_COMMAND_NOT_IMPLEMENTED; 
}

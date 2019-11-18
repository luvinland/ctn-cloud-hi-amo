/**
 * @addtogroup Platform
 * @{
 */

/*****************************************************************************
 *
 *    UART0 Driver
 *
 ******************************************************************************
 *  TuningDriver.c
 ******************************************************************************
 *
 *     Description:  UART 0 Driver
 *
 *****************************************************************************/

/**
 * @file TuningDriver.c
 *
 * This file contains the UART0 driver
 */
#include "AwePlatform.h"
#include "I94100.h"

UINT32 s_PacketBuffer[MAX_COMMAND_BUFFER_LEN] = {0};
INT32 g_nPacketBufferLen = MAX_COMMAND_BUFFER_LEN;

static volatile UINT32 g_nPacketBufferNdx = 0;
static volatile UINT32 g_nTotalBytesToSend = 0;

volatile BOOL g_bUARTPacketReceived = FALSE;
volatile BOOL g_bReadyToSend = TRUE;
volatile BOOL g_bPacketSendComplete = TRUE;
volatile BOOL g_bBaudRateChanged = FALSE;
INT32 g_target_baudrate = 115200;

extern void amo_vent_rx(uint8_t cmd_id); // Jace. 191014. Add control interface using UART0.

//***
//static volatile BOOL b_IsSnding = FALSE; 

//-----------------------------------------------------------------------------
// METHOD:  Interrupt handler for UART0
// PURPOSE: Called when the Transmit char buffer becomes empty
//          or the Receive char buffer has a character
//-----------------------------------------------------------------------------
#if 0 // Jace. 191014. Add control interface using UART0.
void UART0_IRQHandler(void)
{
	uint8_t ch;
	INT32 iir;

        iir = UART0->INTSTS;

	if (iir & UART_INTSTS_BUFERRIF_Msk)		// received TX/RX buffer error
	{
		UART0->FIFOSTS |= (UART_FIFOSTS_TXOVIF_Msk | UART_FIFOSTS_RXOVIF_Msk);	// clear error
	}
    
    if (iir & UART_INTSTS_RDAIF_Msk) // Received data avaialble interrupt
	{
		ch = UART0->DAT; 	// read one byte from UART0 data register;

		// Decode the received character and place in buffer
		awe_fwTuningRxByte(ch);
    }
    
	if (iir & UART_INTSTS_TXENDINT_Msk) // TX buffer empty
        //else if (( iir & UART_INTSTS_TXENDINT_Msk) && b_IsSnding)// TX buffer empty
	{
        // Get the next byte from sPacketBuffer to transmit
        g_bPacketSendComplete = awe_fwTuningTxByte(&ch);

        // g_bPacketSendComplete == FALSE means that ch has a byte that needs to be sent
        if (!g_bPacketSendComplete)
        {
            // Place byte in UART transmit register
            UART0->DAT = ch;
        }else
        { 
            UART_DisableInt(UART0, UART_INTEN_TXENDIEN_Msk);    //UART_INTEN_THREIEN_Msk);
            UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_BUFERRIEN_Msk);
        }
	}
}	// End UART0_IRQHandler
#else
void UART0_IRQHandler(void)
{
	uint8_t cmd_id;
	INT32 iir;

    iir = UART0->INTSTS;

	if (iir & UART_INTSTS_BUFERRIF_Msk)		// received TX/RX buffer error
	{
		UART0->FIFOSTS |= (UART_FIFOSTS_TXOVIF_Msk | UART_FIFOSTS_RXOVIF_Msk);	// clear error
	}
    
    if (iir & UART_INTSTS_RDAIF_Msk) // Received data avaialble interrupt
	{
#if 0 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
		cmd_id = UART0->DAT; 	// read one byte from UART0 data register;
		//printf("UART0 Data IN [%x]\n", cmd_id);
		amo_vent_rx(cmd_id);
#endif
    }
	
}
#endif

#if 0 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
//-----------------------------------------------------------------------------
// METHOD:  TuningInit
// PURPOSE: Set up command processing buffer
//
// configure UART0: PB8 as TX, PB9 as RX
// 115200, 8 bit, 1 stop bit, non-parity, no handshaking
//-----------------------------------------------------------------------------
void UART0_Init(INT32 nBaudRate)
{
	//CLK_SetCoreClock(49152000);
	CLK_SetModuleClock(UART0_MODULE, NULL, CLK_CLKDIV0_UART0(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, 0);

    CLK->CLKSEL1 |= CLK_CLKSEL1_UART0SEL_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB8MFP_Msk);
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB9MFP_Msk);
    SYS->GPB_MFPH |=  (SYS_GPB_MFPH_PB8MFP_UART0_TXD);
    SYS->GPB_MFPH |=  (SYS_GPB_MFPH_PB9MFP_UART0_RXD);

	SYS_ResetModule(UART0_RST); /* Reset IP */

    UART_Open(UART0, nBaudRate); /* Configure UART0 and set UART0 Baudrate */

	UART0->FIFO &= ~(0x0f<<4); // UART RX trigger FIFO level is 1 byte;

    //UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_BUFERRIEN_Msk | UART_INTEN_TXENDIEN_Msk);
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_BUFERRIEN_Msk);
    UART_DisableInt(UART0, UART_INTEN_TXENDIEN_Msk); //UART_INTEN_THREIEN_Msk); //UART_INTEN_TXENDIEN_Msk
     
	NVIC_SetPriority(UART0_IRQn, 0);
    NVIC_EnableIRQ(UART0_IRQn);

}   // End UART0_Init


//-----------------------------------------------------------------------------
// METHOD:  TuningInit
// PURPOSE: Set up command processing buffer
//-----------------------------------------------------------------------------
void UARTMsgInit(void)
{
    // Initialize the communication handler
    awe_fwTuningInit(s_PacketBuffer, MAX_COMMAND_BUFFER_LEN);

}   // End UARTMsgInit


//-----------------------------------------------------------------------------
// METHOD:  UART0SendReply
// PURPOSE: Setup to send reply to sender byte by byte
//-----------------------------------------------------------------------------
void UART0SendReply(void)
{
    BYTE nByteToSend;

    g_bPacketSendComplete = FALSE;

    awe_fwTuningTxByte(&nByteToSend);
    
    // Start the sending process by placing the first byte into the transmit register
    UART0->DAT = nByteToSend;
    UART_EnableInt(UART0, UART_INTEN_TXENDIEN_Msk); //UART_INTEN_THREIEN_Msk);

}   // End UART0SendReply
#endif


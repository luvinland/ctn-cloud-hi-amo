/*******************************************************************************
*
*               BSP Driver
*               ---------------
*
********************************************************************************
*     ControlDriver.h
********************************************************************************
*
*     Description:  AudioWeaver Control Driver
*
*******************************************************************************/  
 #ifndef __CONTROLDRIVER_H__
 #define __CONTROLDRIVER_H__
 
#define CTRL_NUM_INPUTS 1
#define CTRL_BUTTON1 30001

#define CTRL_NUM_OUTPUTS 1
#define CTRL_LED6 31001
 
void ControlInit(void);
void InitializeControlPeripheral(UINT32 nControlID);
void ProcessControlIO(void);

#endif

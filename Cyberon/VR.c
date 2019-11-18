#include <stdio.h>
#include <stdint.h>
#include "base_types.h"
#include "CSpotterSDKApi.h"
#include "I94100.h"
#include "Cyb_support2.h"//SW
#include "..\NuOne\OverLapRam.h"


extern BOOL bCybBufReady;//SW


/*---------------------------------------------------------------------------------------------------------*/
/* Define macro & global variables                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#define k_nStateSize		(96)
#define k_nMaxTime			(300)
#define k_nMemSize			(40*1024)//(60*1024)
#define k_nSampleNum		(128)//Recording buff size:256, Sample size for CSpotter is 128, so call CSpotter_AddSample 2 times

//__align(4) BYTE lpbyMemPool[k_nMemSize];
__align(4) BYTE lpbyState[k_nStateSize] = { 0 };

INT nCurrent_addr = 0x32000;

extern uint32_t u32RecordAddr;//SW
//extern void Record2DF(void);//SW

int32_t AddSample(HANDLE hCSpotter, SHORT *lpsSample, INT nNumSample)
{
	INT nRet;
	INT i;
	
	for(i =0; i<k_nSampleNum*2; i+=k_nSampleNum)
	{
		nRet = CSpotter_AddSample((HANDLE)hCSpotter, lpsSample+i, k_nSampleNum);
		if (nRet == CSPOTTER_SUCCESS)
			break;
	}
	
	return nRet;
}


//Add sample and get result, now support 2 groups(SI+SD) , check nID with pnWordIdx[0]
//Also check SV result, if nID is from second group
int32_t DoVR_get_multi_withSV(uint32_t hCSpotter, INT pnWordIdx[], uint32_t bShowOnly, uint32_t nTimeOut, int (*funcCheckKeyPress)(void))
{
	INT	nID, nWait;
	INT nRet = 0, nErr=0, nSVScore = 0;
	SHORT *lpsSample;
	INT nNumSample;
	
//	Cyb_Preprocess_StartRecord();
	nWait = 0;

	CSpotter_Reset((HANDLE)hCSpotter);
		
	if(nRet !=0)goto L_ERROR;
	
	while (1)
	{
		if (funcCheckKeyPress && (nID = funcCheckKeyPress()) > 0)
		{
			nRet = nID;
			break;
		}


		if (nNumSample < 0)
			break;
		if (nNumSample == 0)
			continue;
		//if (CSpotter_AddSample((HANDLE)hCSpotter, lpsSample, nNumSample) == CSPOTTER_SUCCESS)
		if (AddSample((HANDLE)hCSpotter, lpsSample, nNumSample) == CSPOTTER_SUCCESS)
		{
			nID = CSpotter_GetResult((HANDLE)hCSpotter);
			
			//Check Command index, if nID >= pnWordIdx[0](total command number of first group), means this command is from second group(SD)
			//if command is from second group(SD), need check SV result, if SV score <0, reject it
			if(nID >= pnWordIdx[0])
			{
				//SD, so get SV result
				nErr = CSpotter_GetSpeakerResult((HANDLE)hCSpotter, &nSVScore);
				if (nErr < 0)
				{
					nRet = 0;
					break;
				}
				if (nSVScore < 0)
				{
					nRet = 0;
					break;
				}
			}
			
			
			if (bShowOnly == 0)
			{
				nRet = nID + 1;
				break;
			}
			nWait = 0;
		}
		
		nWait++;
		if (nTimeOut > 0 && nWait >= nTimeOut)
			break;
	}

	CSpotter_Release((HANDLE)hCSpotter);
	
L_ERROR:
	CSpotter_Release((HANDLE)hCSpotter);	
	
	return nRet;

}

//Add sample and get result, check SV result
int32_t DoVR_get_withSV(uint32_t hCSpotter, uint32_t bShowOnly, uint32_t nTimeOut, int (*funcCheckKeyPress)(void))
{
	INT	nID, nWait;
	INT nRet = 0, nErr=0, nSVScore = 0;
	SHORT *lpsSample;
	INT nNumSample;
	
	nWait = 0;

	CSpotter_Reset((HANDLE)hCSpotter);

	if(nRet !=0)goto L_ERROR;
	
	while (1)
	{
		if (funcCheckKeyPress && (nID = funcCheckKeyPress()) > 0)
		{
			nRet = nID;
			break;
		}

		//For VR, do preprocess
		
		if (nNumSample < 0)
			break;
		if (nNumSample == 0)
			continue;
		//if (CSpotter_AddSample((HANDLE)hCSpotter, lpsSample, nNumSample) == CSPOTTER_SUCCESS)
		if (AddSample((HANDLE)hCSpotter, lpsSample, nNumSample) == CSPOTTER_SUCCESS)
		{
			nID = CSpotter_GetResult((HANDLE)hCSpotter);
			
			//get SV result
			nErr = CSpotter_GetSpeakerResult((HANDLE)hCSpotter, &nSVScore);
			if (nErr < 0)
			{
				nRet = 0;
				break;
			}
			if (nSVScore < 0)
			{
				nRet = 0;
				break;
			}
			
			if (bShowOnly == 0)
			{
				nRet = nID + 1;
				break;
			}
			nWait = 0;
		}

		
		nWait++;
		if (nTimeOut > 0 && nWait >= nTimeOut)
			break;
	}

	CSpotter_Release((HANDLE)hCSpotter);
	
L_ERROR:
	CSpotter_Release((HANDLE)hCSpotter);

	return nRet;
	
}


//Add sample and get result
int32_t DoVR_get(uint32_t hCSpotter, uint32_t bShowOnly, uint32_t nTimeOut, int (*funcCheckKeyPress)(void))
{
	INT	nID, nWait;
	INT nRet = 0;
	SHORT *lpsSample;
	INT nNumSample;
	BOOL nPlayback = FALSE; // Jace. 191014. Add control interface using UART0.

	nWait = 0;

	CSpotter_Reset((HANDLE)hCSpotter);

	while (1)
	{
		// Process any platform tasks
		awe_pltTick();//SW

		// Process any framework tasks
		awe_fwTick(&g_AWEInstance);    //SW

		if (funcCheckKeyPress && (nID = funcCheckKeyPress()) > 0)
		{
			nPlayback = TRUE; // Jace. 191014. Add control interface using UART0.
			nRet = nID;
			break;
		}

		//For VR, do preprocess

		lpsSample = ai16CyberonInputBuffer;//SW 0807 i16CybInputBuffer;//SW 
		nNumSample = PCM_SAMPLE_NUM;//SW

		if(bCybBufReady==TRUE)//SW
		{
#ifdef REC  //SW  
			if(u32RecordAddr<0x80000)//test
			{
				Record2DF();
			}
			else
			{
				PA9=0;
			}  
#endif  


			if (AddSample((HANDLE)hCSpotter, lpsSample, nNumSample) == CSPOTTER_SUCCESS)
			{
				nID = CSpotter_GetResult((HANDLE)hCSpotter);

				if (bShowOnly == 0)
				{
					nRet = nID + 1;
					break;
				}
				nWait = 0;
			}

			nWait++;
			if (nTimeOut > 0 && nWait >= nTimeOut) break;

			bCybBufReady=FALSE;   //SW 
		}//SW
	}

	CSpotter_Release((HANDLE)hCSpotter);

	L_ERROR:
	CSpotter_Release((HANDLE)hCSpotter);

#if 1 // Jace. 191014. Add control interface using UART0.
	if(nPlayback)
	{
		//printf("play back[%x]\n", nRet);
		switch(nRet)
		{
			case 0x10: // Power off
				nID = 1;
				break;
			case 0x11: // Power on
				nID = 0;
				break;
			case 0x21: // Fan 1
				nID = 2;
				break;
			case 0x22: // Fan 2
				nID = 3;
				break;
			case 0x23: // Fan 3
				nID = 4;
				break;
			case 0x31: // Timer 1h
				nID = 5;
				break;
			case 0x34: // Timer 4h
				nID = 6;
				break;
			case 0x38: // Timer 8h
				nID = 7;
				break;
			case 0x41: // AI mode
				nID = 8;
				break;
			case 0x50: // Unmute
				nID = 9;
				break;
			case 0x51: // Mute
				nID = 10;
				break;
			default:
				printf("Unknown cmd\n");
				break;
		}

#if 0 // Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32
		while(!PlayNuOnebyIdx(nID));
#endif

		nPlayback = FALSE;
	}
#endif

	return nRet;	
}


/*
 *DoVR_sep_withExtRej Recognize single group with Rej model
 */

int32_t DoVR_sep_withExtRej(uint32_t lpbyCYBase, uint32_t lpbyModel, uint32_t lpbyRejModel, uint32_t bShowOnly, uint32_t nTimeOut, int (*funcCheckKeyPress)(void))
{
	HANDLE hCSpotter = NULL;
	INT n, nErr = CSPOTTER_ERR_LeaveNoMemory;

	n = CSpotter_GetMemoryUsage_SepWithExtRejModel((BYTE *)lpbyCYBase, (BYTE *)lpbyModel, k_nMaxTime, (BYTE *)lpbyRejModel);
	if (n > k_nMemSize)
		return -1;

	hCSpotter = CSpotter_Init_SepWithExtRejModel((BYTE *)lpbyCYBase, (BYTE *)lpbyModel, k_nMaxTime, (BYTE *)lpbyRejModel, overlapMem.vrUseMemPool.lpbyMemPool, k_nMemSize, lpbyState, k_nStateSize, &nErr);
	if (hCSpotter == NULL)
		return -1;

	CSpotter_SetRejectionLevel(hCSpotter,-15);//SW 0227
	
	return DoVR_get((uint32_t)hCSpotter, bShowOnly, nTimeOut, funcCheckKeyPress);
}

/*
 *DoVR_sep00, Recognize single group		//Added by CTWu0
 */
int32_t DoVR_sep00(uint32_t lpbyCYBase, uint32_t lpbyModel, uint32_t bShowOnly, uint32_t nTimeOut, int (*funcCheckKeyPress)(void))
{
	HANDLE hCSpotter = NULL;
	INT n, nErr = CSPOTTER_ERR_LeaveNoMemory;
        

	n = CSpotter_GetMemoryUsage_Sep((BYTE *)lpbyCYBase, (BYTE *)lpbyModel, k_nMaxTime);
	if (n > k_nMemSize)
		return -1;

	hCSpotter = CSpotter_Init_Sep((BYTE *)lpbyCYBase, (BYTE *)lpbyModel, k_nMaxTime, overlapMem.vrUseMemPool.lpbyMemPool, k_nMemSize, lpbyState, k_nStateSize, &nErr);
	if (hCSpotter == NULL)
		return -1;
        
	//CSpotter_SetRejectionLevel(hCSpotter,5);
	return DoVR_get((uint32_t)hCSpotter, bShowOnly, nTimeOut, funcCheckKeyPress);
}

/*
 *DoVR_sep, Recognize single group
 */
int32_t DoVR_sep(uint32_t lpbyCYBase, uint32_t lpbyModel, uint32_t bShowOnly, uint32_t nTimeOut, int (*funcCheckKeyPress)(void))
{
	HANDLE hCSpotter = NULL;
	INT n, nErr = CSPOTTER_ERR_LeaveNoMemory;

	n = CSpotter_GetMemoryUsage_Sep((BYTE *)lpbyCYBase, (BYTE *)lpbyModel, k_nMaxTime);
	if (n > k_nMemSize)
		return -1;

	hCSpotter = CSpotter_Init_Sep((BYTE *)lpbyCYBase, (BYTE *)lpbyModel, k_nMaxTime, overlapMem.vrUseMemPool.lpbyMemPool, k_nMemSize, lpbyState, k_nStateSize, &nErr);
	if (hCSpotter == NULL)
		return -1;
	
	return DoVR_get((uint32_t)hCSpotter, bShowOnly, nTimeOut, funcCheckKeyPress);
}

/*
*DoVR_create_multi_withExtRej, Recognize multi-groups with Rej model
*/
int32_t DoVR_multi_withExtRej(uint32_t lpbyBaseModel, uint32_t lppbyModel, uint32_t nNumModel, uint32_t lppbyRejModel, uint32_t bShowOnly, uint32_t nTimeOut, int (*funcCheckKeyPress)(void))
{
	HANDLE hCSpotter = NULL;
	INT n, nErr = CSPOTTER_ERR_LeaveNoMemory;

	n = CSpotter_GetMemoryUsage_MultiWithExtRejModel((BYTE *)lpbyBaseModel, (BYTE **)lppbyModel, nNumModel, k_nMaxTime, (BYTE *)lppbyRejModel);
	if (n > k_nMemSize)
		return -1;

	hCSpotter = CSpotter_Init_MultiWithExtRejModel((BYTE *)lpbyBaseModel, (BYTE **)lppbyModel, nNumModel, k_nMaxTime, (BYTE *)lppbyRejModel, overlapMem.vrUseMemPool.lpbyMemPool, k_nMemSize, lpbyState, k_nStateSize, &nErr);
	if (hCSpotter == NULL)
		return -1;
	
	return DoVR_get((uint32_t)hCSpotter, bShowOnly, nTimeOut, funcCheckKeyPress);
}

/*
*DoVR_multi , Recognize multi-groups
*/
int32_t DoVR_multi(uint32_t lpbyBaseModel, uint32_t lppbyModel, uint32_t nNumModel, uint32_t bShowOnly, uint32_t nTimeOut, int (*funcCheckKeyPress)(void))
{
	HANDLE hCSpotter = NULL;
	INT n, nErr = CSPOTTER_ERR_LeaveNoMemory;
	
	n = CSpotter_GetMemoryUsage_Multi((BYTE *)lpbyBaseModel, (BYTE **)lppbyModel, nNumModel, k_nMaxTime);
	if (n > k_nMemSize)
		return -1;

	hCSpotter = CSpotter_Init_Multi((BYTE *)lpbyBaseModel, (BYTE **)lppbyModel, nNumModel, k_nMaxTime, overlapMem.vrUseMemPool.lpbyMemPool, k_nMemSize, lpbyState, k_nStateSize, &nErr);
	if (hCSpotter == NULL)
		return -1;
	
	return DoVR_get((uint32_t)hCSpotter, bShowOnly, nTimeOut, funcCheckKeyPress);
}

/*
*DoVR_multi_withSV_ExtRej, This function only support 2 groups(SI+SD) with Rej and SV Model
if want support more/less groups, modify pnWordIdx, also need modify DoVR_get_multi_withSV()
*/
int32_t DoVR_multi_withSV_ExtRej(uint32_t lpbyBaseModel, uint32_t lppbyModel, uint32_t nNumModel, uint32_t lppbySVModel, uint32_t lppbyRejModel,uint32_t bShowOnly, uint32_t nTimeOut, int (*funcCheckKeyPress)(void))
{
	HANDLE hCSpotter = NULL;
	INT n, nErr = CSPOTTER_ERR_LeaveNoMemory;	
	INT pnWordIdx[2] = { 0 };//SI+SD

	if (nNumModel < 2)
		return -1;

	n = CSpotter_GetMemoryUsage_MultiWithSpeakerAndExtRejModel((BYTE *)lpbyBaseModel, (BYTE **)lppbyModel, nNumModel, k_nMaxTime, (BYTE *)lppbySVModel, (BYTE *)lppbyRejModel);
	if (n > k_nMemSize)
		return -1;

	hCSpotter = CSpotter_Init_MultiWithExtRejModel((BYTE *)lpbyBaseModel, (BYTE **)lppbyModel, nNumModel, k_nMaxTime, (BYTE *)lppbyRejModel, overlapMem.vrUseMemPool.lpbyMemPool, k_nMemSize, lpbyState, k_nStateSize, &nErr);
	if (hCSpotter == NULL)
		return -1;
	
	nErr = CSpotter_SetSpeakerModel(hCSpotter, (BYTE *)lppbySVModel);
	if (nErr != CSPOTTER_SUCCESS)
		return -1;
	
	//Get total command number of 2 groups
	pnWordIdx[0] = CSpotter_GetNumWord(((BYTE **)lppbyModel)[0]);
	pnWordIdx[1] = pnWordIdx[0] + CSpotter_GetNumWord(((BYTE **)lppbyModel)[1]);
	
	return DoVR_get_multi_withSV((uint32_t)hCSpotter, pnWordIdx, bShowOnly, nTimeOut, funcCheckKeyPress);
}


/*
*DoVR_sep_withSV_ExtRej, This function only support 1 group(SD) with Rej and SV Model
*/
int32_t DoVR_sep_withSV_ExtRej(uint32_t lpbyCYBase, uint32_t lpbyModel, uint32_t lpbySVModel, uint32_t lpbyRejModel,uint32_t bShowOnly, uint32_t nTimeOut, int (*funcCheckKeyPress)(void))
{
	HANDLE hCSpotter = NULL;
	INT n, nErr = CSPOTTER_ERR_LeaveNoMemory;	

	n = CSpotter_GetMemoryUsage_SepWithExtRejModel((BYTE *)lpbyCYBase, (BYTE *)lpbyModel, k_nMaxTime, (BYTE *)lpbyRejModel);
	if (n > k_nMemSize)
		return -1;

	hCSpotter = CSpotter_Init_SepWithExtRejModel((BYTE *)lpbyCYBase, (BYTE *)lpbyModel, k_nMaxTime, (BYTE *)lpbyRejModel, overlapMem.vrUseMemPool.lpbyMemPool, k_nMemSize, lpbyState, k_nStateSize, &nErr);
	if (hCSpotter == NULL)
		return -1;
	
	nErr = CSpotter_SetSpeakerModel(hCSpotter, (BYTE *)lpbySVModel);
	if (nErr != CSPOTTER_SUCCESS)
		return -1;
		
	return DoVR_get_withSV((uint32_t)hCSpotter, bShowOnly, nTimeOut, funcCheckKeyPress);
}


/*******************************************************************************
*
*				Audio Framework
*				---------------
*
********************************************************************************
*	  PacketAPI.c
********************************************************************************
*
*	  Description:	AudioWeaver driver for packet handling
*
*	  Copyright:	DSP Concepts, Inc., 2007 - 2016
*					1800 Wyatt Drive, Suite 14
*					Sunnyvale, CA 95054
*
*******************************************************************************/


/* ----------------------------------------------------------------------------
 *	Included files
 * ------------------------------------------------------------------------- */

#include "ProxyIDs.h"
//#include "DebugAPI.h"
#include "FileSystem.h"
#include <stdio.h>

#if defined(WIN32) || defined(LINUX)
//#include "debugging.h"
#endif


#if defined(__ADSP21000__) || defined(__ADSPBLACKFIN__)
#pragma default_section(CODE, "awe_fw_slowcode")
#pragma default_section(ALLDATA, "awe_fw_slowanydata")
#pragma default_section(SWITCH, "awe_fw_slowanydata")
#endif

/* ----------------------------------------------------------------------
** Points to the 3 main Audio Weaver memory heaps.	This information
** is used for address translation.
** ------------------------------------------------------------------- */
#if defined(WIN32) || defined(LINUX) || defined(AWE_DYNAMIC_HEAPS)
#else
extern UINT32 g_master_heap[];
extern UINT32 g_slow_heap[];
extern UINT32 g_fastb_heap[];
#endif

/* ----------------------------------------------------------------------------
 *	Local Variables - use awe_fwGetPacketInfo to get extern access
 * ------------------------------------------------------------------------- */
static INT32  g_nPacketBufferLen = 0;
static UINT32 *g_PacketBuffer = NULL;


/* Process a received message. */
AWE_OPTIMIZE_FOR_SPEED
AWE_FW_SLOW_CODE
INT32 awe_fwPacketProcess(AWEInstance *pAWE)
{
	INT32 i, nLen;
	UINT32 crc = 0;
	INT32 retVal = E_SUCCESS;
    
	/* ----------------------------------------------------------------------
	** Verify that the received packet length is less than the message
	** buffer. Also verify that the packet CRC is correct
	** ------------------------------------------------------------------- */

	nLen = (g_PacketBuffer[0] >> 16);

	if (nLen > g_nPacketBufferLen)
	{
		retVal = E_MESSAGE_LENGTH_TOO_LONG;
		goto Packet_Error;
	}

	crc = 0;
    
	for (i = 0; i < nLen; i++)
	{
		crc ^= g_PacketBuffer[i];
	}
    
	if (crc != 0)
	{
		retVal = E_CRC_ERROR;
		goto Packet_Error;
	}

	/* ----------------------------------------------------------------------
	** Switch on the message type
	** ------------------------------------------------------------------- */
	switch (g_PacketBuffer[0] & 0xff)
	{

	case PFID_PrePump:
		{
			awe_fwPrePump(pAWE);
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = 0;
		}
		break;

	case PFID_WritePumpRead:
		{
			int ret = 0;
			UINT32 outlen = g_PacketBuffer[7];
			UINT32 cycles = 0;
			UINT32 timePerProcess = 0;
			int id = g_PacketBuffer[1];
			if (id < 0)
			{
				g_PacketBuffer[0] = (7 << 16);
				g_PacketBuffer[1] = 0;
				g_PacketBuffer[2] = 0;
				g_PacketBuffer[3] = 0;
				g_PacketBuffer[4] = 0;
				g_PacketBuffer[5] = 0;
				if (pAWE->m_pLayoutHead)
				{
					if (pAWE->m_pInterleavedInputPin->nBoundWires)
					{
						g_PacketBuffer[4] = pAWE->m_pInterleavedInputPin->boundWires[0].pWire->wireInfo1;
					}
					if (pAWE->m_pInterleavedOutputPin->nBoundWires)
					{
						g_PacketBuffer[5] = pAWE->m_pInterleavedOutputPin->boundWires[0].pWire->wireInfo1;
					}
				}
				break;
			}
			ret = awe_fwSetValues(pAWE, g_PacketBuffer[2],
					g_PacketBuffer[3],
					g_PacketBuffer[6],
					(const Sample *)(g_PacketBuffer + 8), 0);

			if (ret >= 0)
			{
				LayoutInstance *pLayout = pAWE->m_pLayoutHead;
				if (id > 0)
				{
					InstanceDescriptor *pInst;
					UINT32 classID;
					ret = awe_fwGetObjectByID(pAWE, g_PacketBuffer[1], &pInst, &classID);
					if (ret >= 0)
					{
						pLayout = (LayoutInstance *)pInst;
					}
				}
				ClassLayout_Process(pAWE, pLayout, &cycles, &timePerProcess);
			}

			if (ret >= 0)
			{
				ret = awe_fwFetchValues(pAWE, g_PacketBuffer[4],
					g_PacketBuffer[5],
					outlen,
					(Sample *)(g_PacketBuffer + 4), 0);
			}
			g_PacketBuffer[0] = (outlen + 5) << 16;
			g_PacketBuffer[1] = ret;
			g_PacketBuffer[2] = cycles;
			g_PacketBuffer[3] = timePerProcess;
		}
		break;

#if !defined(WIN32) && !defined(LINUX)
	case PFID_GetCores:
		 {
			int ret = awe_pltGetCores();

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;

		 }
		 break;

	case PFID_DestroyAll:
		 {
			 int ret = 0;
			if (pAWE->m_pAwe_pltAudioStop)
			{
				ret = pAWE->m_pAwe_pltAudioStop();
			}
			awe_pltDestroyAll();

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		 }
		 break;
#endif

	case PFID_SetInstanceID:
		{
			/* Call the function. */
			int ret = awe_fwSetInstanceID(pAWE, g_PacketBuffer[1], g_PacketBuffer[2]);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_DerefPointer:
		{
			/* Call the function. */
			int ret = 0;
			INT32 id = awe_fwDerefPointer(pAWE, g_PacketBuffer[1], &ret);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00040000UL;
			g_PacketBuffer[1] = ret;
			g_PacketBuffer[2] = id;
		}
		break;

	case PFID_SetPointer:
		{
			int ret = awe_fwSetPointer(pAWE, g_PacketBuffer[1], g_PacketBuffer[2], g_PacketBuffer[3]);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_GetPinType:
		{
			/* Call the function. */
			int ret = awe_fwGetPinType(pAWE, g_PacketBuffer[1], (UINT32 *)&g_PacketBuffer[2], (UINT32 *)&g_PacketBuffer[3], (float *)&g_PacketBuffer[4],
				(UINT32 *)&g_PacketBuffer[5], (UINT32 *)&g_PacketBuffer[6], (UINT32 *)&g_PacketBuffer[7],
				(UINT32 *)&g_PacketBuffer[8],  (UINT32 *)&g_PacketBuffer[9]);
		
			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x000b0000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_GetWireType:
		{
			/* Call the function. */
			int ret = awe_fwGetWireType(pAWE, g_PacketBuffer[1], (UINT32 *)&g_PacketBuffer[2], (float *)&g_PacketBuffer[3], (UINT32 *)&g_PacketBuffer[4], (UINT32 *)&g_PacketBuffer[5], (UINT32 *)&g_PacketBuffer[6]);
		
			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00080000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_SetCall:
		{
			/* Call the function. */
			int ret = awe_fwSetCall2(pAWE, g_PacketBuffer[1], g_PacketBuffer[2]);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_GetCall:
		{
			/* Call the function. */
			int ret = awe_fwGetCall2(pAWE, g_PacketBuffer[1], g_PacketBuffer[2]);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_GetClassType:
		{
			/* Call the function. */
			UINT classID = 0;
			InstanceDescriptor *pInst;
			int ret = awe_fwGetObjectByID(pAWE, g_PacketBuffer[1], &pInst, &classID);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00040000UL;
			g_PacketBuffer[1] = ret;
			g_PacketBuffer[2] = classID;
		}
		break;

	case PFID_ClassWire_Constructor:
		{
			/* Call the function - requires 4 args. */
			int ret = 0;
			InstanceDescriptor *pInst = ClassWire_Constructor(pAWE, &ret, g_PacketBuffer[1], g_PacketBuffer[2], g_PacketBuffer[3], g_PacketBuffer[4]);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00040000UL;
			g_PacketBuffer[1] = (pInst)? pInst->nUniqueInstanceID: 0;
			g_PacketBuffer[2] = ret;
		}
		break;

	case PFID_BindIOToWire:
		{
			/* Call the function. */
			int ret = BindIOToWire(pAWE, g_PacketBuffer[1], g_PacketBuffer[2]);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_FetchValue:
		{
			/* Call the function. */
			int ret = 0;
			INT value = awe_fwFetchValue(pAWE, g_PacketBuffer[1], &ret, g_PacketBuffer[2], 0);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00040000UL;
			g_PacketBuffer[1] = ret;
			g_PacketBuffer[2] = value;
		}
		break;

	case PFID_FetchValue_float:
		{
			/* Call the function. */
			int ret = 0;
			INT value = awe_fwFetchValue(pAWE, g_PacketBuffer[1], &ret, g_PacketBuffer[2], 1);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00040000UL;
			g_PacketBuffer[1] = ret;
			g_PacketBuffer[2] = value;
		}
		break;

	case PFID_GetCallFetchValue:
		{
			/* Call the function. */
			int ret = 0;
			INT value = awe_fwGetCallFetchValue(pAWE, g_PacketBuffer[1], g_PacketBuffer[2], &ret, g_PacketBuffer[3], 0);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00040000UL;
			g_PacketBuffer[1] = ret;
			g_PacketBuffer[2] = value;

		}
		break;

	case PFID_GetCallFetchValue_float:
		{
			/* Call the function. */
			int ret = 0;
			INT value = awe_fwGetCallFetchValue(pAWE, g_PacketBuffer[1], g_PacketBuffer[2], &ret, g_PacketBuffer[3], 1);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00040000UL;
			g_PacketBuffer[1] = ret;
			g_PacketBuffer[2] = value;

		}
		break;

	case PFID_SetValueCall:
	case PFID_SetValue:
		{
			/* Call the function. */
			int ret = awe_fwSetValue(pAWE, g_PacketBuffer[1], g_PacketBuffer[2], g_PacketBuffer[3], 0);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_SetValue_float:
		{
			/* Call the function. */
			int ret = awe_fwSetValue(pAWE, g_PacketBuffer[1], g_PacketBuffer[2], g_PacketBuffer[3], 1);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_SetValueSetCall:
		{
			/* Call the function. */
			int ret = awe_fwSetValueSetCall(pAWE, g_PacketBuffer[1], g_PacketBuffer[2], g_PacketBuffer[3], g_PacketBuffer[4], 0);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_SetValueSetCall_float:
		{
			/* Call the function. */
			int ret = awe_fwSetValueSetCall(pAWE, g_PacketBuffer[1], g_PacketBuffer[2], g_PacketBuffer[3], g_PacketBuffer[4], 1);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_GetHeapCount:
		{
			/* Call the function. */
			INT32 count = awe_fwGetHeapCount();

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = count;
		}
		break;

	case PFID_GetHeapSize:
		{
			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00090000UL;
			g_PacketBuffer[1] = awe_fwGetHeapSize(pAWE, (g_PacketBuffer + 2));
		}
		break;

#if defined(WIN32) || defined(LINUX)
	case PFID_DestroyAll:
		{
			int ret = 0;
			if (pAWE->m_pAwe_pltAudioStop)
			{
				ret = pAWE->m_pAwe_pltAudioStop();
			}
			awe_fwDestroy(pAWE);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		 }
		 break;
#endif

	case PFID_Destroy:
		{
			/* Call the functions. */
			if (pAWE->m_pAwe_pltAudioStop)
			{
				pAWE->m_pAwe_pltAudioStop();
			}
			awe_fwDestroy(pAWE);

			// For now, hard code this to the first layout
			awe_fwClearProfileValues(pAWE, 0);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00020000UL;
		}
		break;

	case PFID_GetCIModuleCount:
		{
			/* Call the function. */
			INT32 count = awe_fwGetCIModuleCount(pAWE);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = count;
		}
		break;

	case PFID_GetCIModuleInfo:
		{
			/* Call the function. */
			UINT32 classID;
			UINT32 numParameters;
			INT32 ret = awe_fwGetCIModuleInfo(pAWE, g_PacketBuffer[1], &classID, &numParameters);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00050000UL;
			g_PacketBuffer[1] = ret;
			g_PacketBuffer[2] = classID;
			g_PacketBuffer[3] = numParameters;
		}
		break;

	case PFID_ClassModule_Constructor:
		{
			/* Call the function. */
			int ret = 0;
			UINT32 nIO = g_PacketBuffer[2];
			UINT32 nWires = ClassMod_GetWireCount(nIO);

			ModInstanceDescriptor *pMod = ClassModule_Constructor2(pAWE,
				g_PacketBuffer[1],
				&ret,
				nIO,
				(UINT32 *)(g_PacketBuffer + 4),
				g_PacketBuffer[3],
				(Sample *)(g_PacketBuffer + 4 + nWires));

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00040000UL;
			g_PacketBuffer[1] = (pMod)? pMod->nUniqueInstanceID: (UINT32)ret;
			g_PacketBuffer[2] = ret;
		}
		break;

	case PFID_ClassLayout_Constructor:
		{
			/* Call the function. */
			int ret = 0;
			InstanceDescriptor *pLayout = ClassLayout_Constructor(pAWE, &ret,
					g_PacketBuffer[1],
					g_PacketBuffer[2]);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00040000UL;
			g_PacketBuffer[1] = (pLayout)? pLayout->nUniqueInstanceID: 0;
			g_PacketBuffer[2] = ret;
		}
		break;

	case PFID_AddModuleToLayout:
		{
			/* Call the function. */
			int ret = awe_fwAddModuleToLayout(pAWE, nLen - 2, (Sample *)&g_PacketBuffer[1]);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_SetModuleState:
		{
			/* Call the function. */
			InstanceDescriptor *pInst;
			UINT32 classID;
			int ret = awe_fwGetObjectByID(pAWE, g_PacketBuffer[1], &pInst, &classID);
			if (ret >= E_SUCCESS)
			{
				ret = awe_fwSetModuleState((ModuleInstanceDescriptor *)pInst,
					g_PacketBuffer[2]);
			}

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_GetModuleState:
		{
			InstanceDescriptor *pInst;
			UINT32 classID;
			int ret = awe_fwGetObjectByID(pAWE, g_PacketBuffer[1], &pInst, &classID);
			if (ret >= E_SUCCESS)
			{
				ret = awe_fwGetModuleState((ModuleInstanceDescriptor *)pInst);
			}

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_PumpModule:
		{
			InstanceDescriptor *pInst;
			UINT32 classID;
			int ret = awe_fwGetObjectByID(pAWE, g_PacketBuffer[1], &pInst, &classID);
			if (ret >= E_SUCCESS)
			{
				ret = awe_fwPumpModule(pAWE, (ModuleInstanceDescriptor *)pInst);
			}

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_ClassLayout_Process:
		{
			UINT32 cycles = 0;
			UINT32 timePerProcess = 0;
			int ret = 0;
			LayoutInstance *pLayout;
			InstanceDescriptor *pInst;
			UINT32 classID;
			if (g_PacketBuffer[1] == 0)
			{
				pLayout = pAWE->m_pLayoutHead;
				goto do_rest;
			}
			ret = awe_fwGetObjectByID(pAWE, g_PacketBuffer[1], &pInst, &classID);
			if (ret >= E_SUCCESS)
			{
				pLayout = (LayoutInstance *)pInst;
do_rest:
				if (!pLayout)
				{
					ret = E_NO_LAYOUTS;
					goto layout_err;
				}
				if (pLayout->nCurrentDivide == 0)
				{
					pLayout->nCurrentDivide = pLayout->nDivider - 1;

					/* Call the function. */
					ClassLayout_Process(pAWE, pLayout, &cycles, &timePerProcess);
				}
				else
				{
					pLayout->nCurrentDivide--;
				}
			}

			/* Construct the return buffer. */
layout_err:
			g_PacketBuffer[0] = 0x00050000UL;
			g_PacketBuffer[1] = ret;
			g_PacketBuffer[2] = cycles;
			g_PacketBuffer[3] = timePerProcess;
		}
		break;

	case PFID_GetFirstObject:
		{
			/* Call the function. */
			InstanceDescriptor *instance_address = NULL;
			UINT32 classID;
			int ret = awe_fwGetFirstObject2(pAWE, &instance_address, &classID);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00050000UL;
			g_PacketBuffer[1] = ret;
			g_PacketBuffer[2] = (instance_address)? instance_address->nUniqueInstanceID: 0;
			g_PacketBuffer[3] = classID;
		}
		break;

	case PFID_GetNextObject:
		{
			/* Call the function. */
			InstanceDescriptor *instance_address = 0;
			UINT32 classID;
			InstanceDescriptor *pInst;
			int ret = awe_fwGetObjectByID(pAWE, g_PacketBuffer[1], &pInst, &classID);
			if (ret >= E_SUCCESS)
			{
				ret = awe_fwGetNextObject(pInst,
						&instance_address,
						&classID);
			}

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00050000UL;
			g_PacketBuffer[1] = ret;
			g_PacketBuffer[2] = (instance_address)? instance_address->nUniqueInstanceID: 0;
			g_PacketBuffer[3] = classID;
		}
		break;

#if 0
	case PFID_GetObjectByIndex:
		{
			/* Call the function. */
			InstanceDescriptor *instance_address;
			UINT32 classID;
			int ret = awe_fwGetObjectByIndex(pAWE, g_PacketBuffer[1],
					&instance_address,
					&classID);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00050000UL;
			g_PacketBuffer[1] = ret;
			g_PacketBuffer[2] = (instance_address)? instance_address->nUniqueInstanceID: 0;
			g_PacketBuffer[3] = classID;
		}
		break;
#endif

	case PFID_GetObjectByID:
		{
			/* Call the function. */
			UINT32 classID;
			InstanceDescriptor *pInst;
			int ret = awe_fwGetObjectByID(pAWE, g_PacketBuffer[1], &pInst, &classID);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00040000UL;
			g_PacketBuffer[1] = ret;
			g_PacketBuffer[2] = classID;
		}
		break;

	case PFID_GetFirstIO:
		{
			/* Call the function. */
			InstanceDescriptor *instance_address = 0;
			UINT32 classID;
			int ret = awe_fwGetFirstIO(pAWE, &instance_address,
					&classID, (float *)&g_PacketBuffer[5],
					(UINT32 *)&g_PacketBuffer[6], (UINT32 *)&g_PacketBuffer[7], (UINT32 *)&g_PacketBuffer[8],
					(UINT32 *)&g_PacketBuffer[9], (UINT32 *)&g_PacketBuffer[10]);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x000c0000UL;
			g_PacketBuffer[1] = ret;
			g_PacketBuffer[2] = (instance_address)? instance_address->nUniqueInstanceID: 0;
			g_PacketBuffer[3] = classID;
			if (instance_address && (((WireInstance *)instance_address)->buffer))
			{
				g_PacketBuffer[4] = 1;
			}
			else
			{
				g_PacketBuffer[4] = 0;
			}
		}
		break;

	case PFID_GetNextIO:
		{
			/* Call the function. */
			InstanceDescriptor *instance_address = 0;
			UINT32 classID = 0;
			int ret = awe_fwGetNextIO(pAWE, g_PacketBuffer[1],
					&instance_address,
					&classID, (float *)&g_PacketBuffer[5],
					(UINT32 *)&g_PacketBuffer[6], (UINT32 *)&g_PacketBuffer[7], (UINT32 *)&g_PacketBuffer[8],
					(UINT32 *)&g_PacketBuffer[9], (UINT32 *)&g_PacketBuffer[10]);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x000c0000UL;
			g_PacketBuffer[1] = ret;
			g_PacketBuffer[2] = (instance_address)? instance_address->nUniqueInstanceID: 0;
			g_PacketBuffer[3] = classID;
			if (instance_address && (((IOPinDescriptor *)instance_address)->nBoundWires))
			{
				g_PacketBuffer[4] = 1;
			}
			else
			{
				g_PacketBuffer[4] = 0;
			}
		}
		break;

	case PFID_StartAudio:
		{
			/* Init the FW layout IO */
			int ret = 0;
			awe_fwAudioInitIO(pAWE);

			/* Call the function. */
			if (pAWE->m_pAwe_pltAudioStart)
			{
				ret = pAWE->m_pAwe_pltAudioStart();
			}
			else
			{
				ret = E_NOT_IMPLEMENTED_IN_RS232;
			}

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;


	case PFID_StopAudio:
		{
			/* Call the function. */
			int ret = 0;
			if (pAWE->m_pAwe_pltAudioStop)
			{
				ret = pAWE->m_pAwe_pltAudioStop();
			}
			else
			{
				ret = E_NOT_IMPLEMENTED_IN_RS232;
			}

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_FetchValues:
		{
			UINT32 offset = g_PacketBuffer[2];
			UINT32 argSize = g_PacketBuffer[3];
			int ret = awe_fwFetchValues(pAWE, g_PacketBuffer[1],
					offset,
					argSize,
					(Sample *)(g_PacketBuffer + 2), 0);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = (3 + argSize) << 16;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_FetchValues_float:
		{
			UINT32 offset = g_PacketBuffer[2];
			UINT32 argSize = g_PacketBuffer[3];
			int ret = awe_fwFetchValues(pAWE, g_PacketBuffer[1],
					offset,
					argSize,
					(Sample *)(g_PacketBuffer + 2), 1);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = (3 + argSize) << 16;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_GetCallFetchValues:
		{
			UINT32 offset = g_PacketBuffer[2];
			UINT32 argSize = g_PacketBuffer[4];
			UINT32 mask = g_PacketBuffer[3];
			int ret = awe_fwGetCallFetchValues(pAWE, g_PacketBuffer[1],
					offset,
					mask,
					argSize,
					(Sample *)(g_PacketBuffer + 2), 0);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = (3 + argSize) << 16;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_GetCallFetchValues_float:
		{
			UINT32 offset = g_PacketBuffer[2];
			UINT32 argSize = g_PacketBuffer[4];
			UINT32 mask = g_PacketBuffer[3];
			int ret = awe_fwGetCallFetchValues(pAWE, g_PacketBuffer[1],
					offset,
					mask,
					argSize,
					(Sample *)(g_PacketBuffer + 2), 1);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = (3 + argSize) << 16;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_SetValues:
		{
			//INT32 count;
			int ret = 0;
			UINT32 offset = g_PacketBuffer[2];
			UINT32 argCount = g_PacketBuffer[3];

			DISABLE_INTERRUPTS();
			ret = awe_fwSetValues(pAWE, g_PacketBuffer[1],
					offset,
					argCount,
					(const Sample *)(g_PacketBuffer + 4), 0);
			ENABLE_INTERRUPTS();

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_SetValues_float:
		{
			//INT32 count;
			int ret = 0;
			UINT32 offset = g_PacketBuffer[2];
			UINT32 argCount = g_PacketBuffer[3];

			DISABLE_INTERRUPTS();
			ret = awe_fwSetValues(pAWE, g_PacketBuffer[1],
					offset,
					argCount,
					(const Sample *)(g_PacketBuffer + 4), 1);
			ENABLE_INTERRUPTS();

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_SetValuesSetCall:
		{
			//INT32 count;
			int ret = 0;
			UINT32 offset = g_PacketBuffer[2];
			UINT32 argCount = g_PacketBuffer[4];
			UINT32 mask = g_PacketBuffer[3];

			//DISABLE_INTERRUPTS();
			ret = awe_fwSetValuesSetCall(pAWE, g_PacketBuffer[1],
					offset,
					mask,
					argCount,
					(const Sample *)(g_PacketBuffer + 5), 0);
			//ENABLE_INTERRUPTS();

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_SetValuesSetCall_float:
		{
			//INT32 count;
			int ret = 0;
			UINT32 offset = g_PacketBuffer[2];
			UINT32 argCount = g_PacketBuffer[4];
			UINT32 mask = g_PacketBuffer[3];

			//DISABLE_INTERRUPTS();
			ret = awe_fwSetValuesSetCall(pAWE, g_PacketBuffer[1],
					offset,
					mask,
					argCount,
					(const Sample *)(g_PacketBuffer + 5), 1);
			//ENABLE_INTERRUPTS();

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = ret;
		}
		break;

	case PFID_GetSizeofInt:
		{
			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = sizeof(INT32);
		}
		break;

	case PFID_GetFirstCore:
	case PFID_GetTargetInfo:
		{
			g_PacketBuffer[0] = 0x000e0000UL;
			g_PacketBuffer[1] = awe_fwGetTargetInfo(pAWE, (TargetInfo *)&g_PacketBuffer[2]);
		}
		break;

	case PFID_GetProfileValues:
		{
			g_PacketBuffer[0] = 0x00050000UL;
			g_PacketBuffer[1] = awe_fwGetProfileValues(pAWE, 0, (UINT32 *)&g_PacketBuffer[2], (UINT32 *)&g_PacketBuffer[3]);
		}
		break;

	case PFID_GetFirstFile:
		{
			UINT32 nStatus, n;
			UINT32 nPacketLen;
			PDIRECTORY_ENTRY pCurrentDirEntry;
			UINT32 *pData;

			nStatus = awe_fwGetFirstFile(&pCurrentDirEntry);

			if (nStatus == E_SUCCESS)
			{
				g_PacketBuffer[2] = awe_fwGetFileAttribute(pCurrentDirEntry);
				g_PacketBuffer[3] = pCurrentDirEntry->nDataDWordCnt;
			}

			pData = (UINT32 *)(pCurrentDirEntry->nFilename);

			for (n = 0; n < 6; n++)
			{
				g_PacketBuffer[n + 4] = *pData++;;
			}

			/* Account for the other words returned in the packet */
			nPacketLen = 11;

			/* Place packet length in high order word of 1st packet UINT32 */
			g_PacketBuffer[0] = nPacketLen << 16;

			/* Construct the return buffer. */
			g_PacketBuffer[1] = nStatus;
		}
		break;

	case PFID_GetNextFile:
		{
			UINT32 nStatus, n;
			UINT32 nPacketLen;
			PDIRECTORY_ENTRY pCurrentDirEntry;
			UINT32 *pData;

			nStatus = awe_fwGetNextFile(&pCurrentDirEntry);

			if (nStatus == E_SUCCESS)
			{
				g_PacketBuffer[2] = awe_fwGetFileAttribute(pCurrentDirEntry);
				g_PacketBuffer[3] = pCurrentDirEntry->nDataDWordCnt;
			}

			pData = (UINT32 *)(pCurrentDirEntry->nFilename);

			for (n = 0; n < 6; n++)
			{
				g_PacketBuffer[n + 4] = *pData++;;
			}

			/* Account for the other words returned in the packet */
			nPacketLen = 11;

			/* Place packet length in high order word of 1st packet UINT32 */
			g_PacketBuffer[0] = nPacketLen << 16;

			/* Construct the return buffer. */
			g_PacketBuffer[1] = nStatus;
		}
		break;

	case PFID_OpenFile:
		{
			UINT32 nStatus;
			UINT32 nFileLenInDWords = 0;
			BYTE nFileAttribute = (BYTE)g_PacketBuffer[1];

			nStatus = awe_fwOpenFile(nFileAttribute, &(g_PacketBuffer[2]),
					&nFileLenInDWords);

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00040000UL;
			g_PacketBuffer[1] = nStatus;
			g_PacketBuffer[2] = nFileLenInDWords;
		}
		break;

	case PFID_ReadFile:
		{
			UINT32 nStatus;
			UINT32 nLength;

			nStatus = awe_fwReadFile(g_PacketBuffer[1], &(g_PacketBuffer[3]), &(g_PacketBuffer[2]));
			nLength = (g_PacketBuffer[2] + 4) << 16;

			/* Construct the return buffer. Returned length includes appended
			CRC */
			g_PacketBuffer[0] = nLength;
			g_PacketBuffer[1] = nStatus;
		}
		break;

	case PFID_WriteFile:
		{
			UINT32 nStatus;

			nStatus = awe_fwWriteFile(g_PacketBuffer[1], &(g_PacketBuffer[2]), &(g_PacketBuffer[2]));

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = nStatus;
		}
		break;

	case PFID_CloseFile:
		{
			UINT32 nStatus;

			nStatus = awe_fwCloseFile();

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = nStatus;
		}
		break;

	case PFID_DeleteFile:
		{
			UINT32 nStatus;

			nStatus = awe_fwDeleteFile(&(g_PacketBuffer[2]));

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = nStatus;
		}
		break;

	case PFID_ExecuteFile:
		{
			UINT32 nStatus;

			nStatus = awe_fwExecuteFile(pAWE, &(g_PacketBuffer[2]));

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = nStatus;
		}
		break;

	case PFID_EraseFlash:
		{
			UINT32 nStatus;
			nStatus = awe_fwEraseFlash();

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = nStatus;
		}
		break;

	case PFID_GetFileSystemInfo:
		{
			g_PacketBuffer[0] = 0x000b0000UL;
			g_PacketBuffer[1] = GetFileSystemInfo((FileSystemInfo *)&g_PacketBuffer[2]);
		}
		break;

	case PFID_FileSystemReset:
		{
			UINT32 nStatus;

			nStatus = awe_fwResetFileSystem();

			/* Construct the return buffer. */
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = nStatus;
		}
		break;

	case PFID_CreateLookupTable:
		{
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = awe_fwcreateLookupTable(pAWE, g_PacketBuffer[1]);
		}
		break;

	case PFID_UpdateLookupTable:
		{
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = awe_fwUpdateLookupTable(pAWE);
		}
		break;

	case PFID_Tick:
		{
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = awe_fwTick(pAWE);
		}
		break;

	case PFID_Set_Baud_Rate:
    	{
            #if defined(__ADSP21000__)
				g_target_baudrate = g_PacketBuffer[1];
				g_bBaudRateChanged = TRUE;              
			#endif
			
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = 0;
	    }
    	break;
    	
    case PFID_Get_Flash_Erase_Time:
        {
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = 0;  
			
			#if defined(__ADSP21000__) || defined(DSPC_CORTEX_M) 
                #if defined(USE_FLASH_FILE_SYSTEM)
                    g_PacketBuffer[1] = awe_pltGetFlashEraseTime();
                #endif
			#endif
	    }
    	break;    
            	  	
    case PFID_Reboot_Target:
        {
            #if defined(__ADSP21000__) || defined(DSPC_CORTEX_M) 
                g_bReboot = TRUE;
            #endif
            
			g_PacketBuffer[0] = 0x00030000UL;
			g_PacketBuffer[1] = 0;            
        }  
    	break;

#ifdef FOR_C6
	// This opcode is only for C6 targets.
	case PFID_PumpLayout:
		{
			extern int awe_fwPumpLayout(unsigned int *pAverageCycles, unsigned int *pTimePerProcess);

			g_PacketBuffer[0] = 0x00050000UL;
			g_PacketBuffer[1] = awe_fwPumpLayout(&g_PacketBuffer[2], &g_PacketBuffer[3]);
		}
		break;
#endif

	default:
		retVal = E_BADPACKET;

Packet_Error:
		/* Construct the return buffer. */
		g_PacketBuffer[0] = 0x00030000UL;
		g_PacketBuffer[1] = retVal;

	}	/*	End switch */

	/* ----------------------------------------------------------------------
	** Create the CRC word at the end of the message
	** ------------------------------------------------------------------- */

	nLen = g_PacketBuffer[0] >> 16;

	crc = 0;
	for (i = 0; i < (nLen - 1); i++)
	{
		crc ^= g_PacketBuffer[i];
	}
	g_PacketBuffer[nLen - 1] = crc;

	return retVal;
}


/**
 * @brief Initialize packet communications.
 * @param[in] packet_buf Pointer to packet buffer
 * @param[in] buf_length Packet buffer length in UINT32s
 */
AWE_OPTIMIZE_FOR_SPACE 
AWE_FW_SLOW_CODE 
void awe_fwPacketInit(UINT32 * packet_buf, INT32 buf_length)
{
	// Set the global variables
	g_nPacketBufferLen = buf_length;
	g_PacketBuffer = packet_buf;
}


/**
 * @brief Initialize packet communications.
 * @param[out] packet_buf Updates pointer to packet buffer
 * @param[out] buf_length Updates packet buffer length
 */
AWE_OPTIMIZE_FOR_SPACE 
AWE_FW_SLOW_CODE 
void awe_fwGetPacketInfo(UINT32 **packet_buf, INT32 * buf_length)
{
	// Set the global variables
	*packet_buf = g_PacketBuffer;
	*buf_length = g_nPacketBufferLen;
}

/**
* @}
* End of file
*/

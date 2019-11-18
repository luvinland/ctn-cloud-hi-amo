#include <stdio.h>
#include "I94100.h"
#include "base_types.h"
#include "DataFlashApi.h"

//////////////////////////////////////////
// The API must be provided by caller
//////////////////////////////////////////

#define k_nFlashPageSize	0x1000//4K Bytes

INT DataFlash_Write(BYTE *dest, BYTE *src, INT nSize)
{
	// following is fake code

//	memcpy(dest, src, nSize);
//	return 0;
	uint32_t i;
	if (((uint32_t)dest & 0x3) != 0)
		return -1;
	for (i = 0; i < nSize; i += 4) {
		FMC_Write((uint32_t)dest + i, ((uint32_t *)src)[i >> 2]);
	}
	return 0;
}

INT DataFlash_Erase(BYTE *dest, INT nSize)
{
	// following is fake code
	uint32_t i;
	int32_t	i32Ret;
	if (((uint32_t)dest & (k_nFlashPageSize - 1)) != 0)
		return -1;
	for (i = 0; i < nSize; i += k_nFlashPageSize){
//		memset(dest + i, 0xff, k_nFlashPageSize);
		i32Ret = FMC_Erase((uint32_t)dest + i);
		if (i32Ret != 0)
			return i32Ret;
	}
	return 0;
}

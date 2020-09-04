/*! 
 *  \file   radarOsal_malloc.c
 *
 *  \brief   radarOsal_malloc functions for radar demo.
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <modules/utilities/radarOsal_malloc.h>


radarOsal_heapObj gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS];

/*!
   \fn     radarOsal_memInit
   \brief   OSAL function for heap memory structure initialization.

   \return    RADAROSAL_FAIL if heap init failed, RADAROSAL_PASS if heap init passed.
   \pre       none
   \post      none
 */
int32_t radarOsal_memInit(radarOsal_heapConfig * config, uint8_t numHeap)
{
	int32_t i;
	if (numHeap > RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS)
	{
		printf("Maximum supported heap is %d, cannot initialize %d heaps, exit!\n", RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS, numHeap);
		exit(1);
	}
    memset(gRadarOsal_heapObj, 0, sizeof(gRadarOsal_heapObj));

    for (i = 0; i < numHeap; i++)
    {
		gRadarOsal_heapObj[config[i].heapType].heapType   =  config[i].heapType;
		gRadarOsal_heapObj[config[i].heapType].heapAddr   =  config[i].heapAddr;
		gRadarOsal_heapObj[config[i].heapType].heapSize   =  config[i].heapSize;
		gRadarOsal_heapObj[config[i].heapType].heapAllocOffset   = 0;
		gRadarOsal_heapObj[config[i].heapType].scratchAddr   =(int8_t *) config[i].scratchAddr;
		gRadarOsal_heapObj[config[i].heapType].maxScratchSizeUsed   = 0;
		gRadarOsal_heapObj[config[i].heapType].scratchSize   = config[i].scratchSize;
    }
    return (RADARMEMOSAL_PASS);
}

/*!
   \fn     radarOsal_memDeInit
   \brief   OSAL function for heap memory structure de-initialization.

   \return    RADAROSAL_FAIL if heap deinit failed, RADAROSAL_PASS if heap deinit passed.
   \pre       none
   \post      none
 */
int32_t radarOsal_memDeInit(void)
{
    memset(gRadarOsal_heapObj, 0, sizeof(gRadarOsal_heapObj));
    return (RADARMEMOSAL_PASS);
}


/*!
   \fn     radarOsal_memAlloc
   \brief   OSAL function for memory allocation.

   \param[in]    memoryType
               input radarMemOsal_HeapType.

   \param[in]    scratchFlag
               Input flag to indicate whether request memory is a scratch that can be shared across modules. 1 for scratch memory request, and 0 otherwise.

   \param[in]    size
               Request memory size in number of bytes.

   \param[in]    alignment
               Request memory alignment in number of bytes. 0 for no alignment requirement. Alignment has to be number being power of 2.

   \return    NULL if malloc failed, void pointer if malloc passed.
   \pre       Note that L2 heap is static pointer assignment for BIOS-less implementation. No run-time malloc/free from L2 heap is supported.
              radarOsal_memDeInit() must be called to reset the pointers before L2 heap can be used by another application.
   \post      none
 */
void * radarOsal_memAlloc(uint8_t memoryType, uint8_t scratchFlag, uint32_t size, uint16_t alignment)
{
	void * pointer = NULL;
	uint32_t addrOffset;

	if (memoryType > (uint8_t) RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS)
		return pointer;


	if (scratchFlag == 0)
	{
	    if ( alignment <= 1 )
	    {
	    	pointer = (void *) &gRadarOsal_heapObj[memoryType].heapAddr[gRadarOsal_heapObj[memoryType].heapAllocOffset];
	    	gRadarOsal_heapObj[memoryType].heapAllocOffset += size;
	    }
	    else
	    {
			addrOffset 	=	alignment - ((uint32_t) &gRadarOsal_heapObj[memoryType].heapAddr[gRadarOsal_heapObj[memoryType].heapAllocOffset]) & (alignment - 1);
	    	pointer 	= (void *) &gRadarOsal_heapObj[memoryType].heapAddr[addrOffset + gRadarOsal_heapObj[memoryType].heapAllocOffset];
	    	gRadarOsal_heapObj[memoryType].heapAllocOffset += size + addrOffset;
	    }
    	if (gRadarOsal_heapObj[memoryType].heapAllocOffset > gRadarOsal_heapObj[memoryType].heapSize)
    	{
    		pointer = NULL;
    		if (memoryType == (uint8_t)RADARMEMOSAL_HEAPTYPE_DDR_CACHED)
    			printf("out of DDR heap memory!\n");
    		else if (memoryType == (uint8_t)RADARMEMOSAL_HEAPTYPE_LL2)
    			printf("out of L2 heap memory!\n");
    		else
    			printf("out of L1 heap memory!\n");
    	}
	}
	else
	{
		addrOffset	=	0;
	    if ( alignment <= 1 )
	    	pointer = gRadarOsal_heapObj[memoryType].scratchAddr;
	    else
	    {
			addrOffset 	=	alignment - ((uint32_t) gRadarOsal_heapObj[memoryType].scratchAddr) & (alignment - 1);
	    	pointer = &gRadarOsal_heapObj[memoryType].scratchAddr[addrOffset];
	    }
    	if (gRadarOsal_heapObj[memoryType].maxScratchSizeUsed < size)
			gRadarOsal_heapObj[memoryType].maxScratchSizeUsed = size;
	    if(size + addrOffset > gRadarOsal_heapObj[memoryType].scratchSize)
	    {
    		pointer = NULL;
    		if (memoryType == (uint8_t)RADARMEMOSAL_HEAPTYPE_DDR_CACHED)
    			printf("out of DDR scratch memory!\n");
    		else if (memoryType == (uint8_t)RADARMEMOSAL_HEAPTYPE_LL2)
    			printf("out of L2 scratch memory!\n");
    		else
    			printf("out of L1 scratch memory!\n");
	    }

	}

	return (pointer);
}


/*!
   \fn     radarOsal_memFree
   \brief   OSAL function for memory free.

   \param[in]    ptr
               input poointer to be freed.

   \param[in]    size
               Size of the memory to be freed, in number of bytes .

   \return    none.
   \pre       Note that L2 heap is static pointer assignment for BIOS-less implementation. No run-time malloc/free from L2 heap is supported.
              radarOsal_memFree() is a no-op for L2 heap, and radarOsal_memDeInit() must be called to reset the pointers before L2 heap can be used by another application.
   \post      none
 */
void radarOsal_memFree(void *ptr, uint32_t size)
{
	return;
}


/*!
   \fn     radarOsal_memFree
   \brief   OSAL function to print memory usage for DEMO modules.

   \return    none.
 */
void radarOsal_printHeapStats()
{
	if (gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAddr != NULL)
	{
		printf("DDR Heap : size %d (0x%x), used %d (0x%x)\n",
    		gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapSize,
			gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAllocOffset, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAllocOffset);
	}
	else
	{
		printf("DDR Heap : Not used!\n");
	}

	if (gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchAddr != NULL)
	{
		printf("DDR scratch : size %d (0x%x), used %d (0x%x)\n",
    		gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchSize,
			gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].maxScratchSizeUsed, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].maxScratchSizeUsed);
	}
	else
	{
		printf("DDR scratch : Not used!\n");
	}

	if (gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapAddr != NULL)
	{
		printf("LL2 Heap : size %d (0x%x), used %d (0x%x)\n",
    		gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapSize,
			gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapAllocOffset, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapAllocOffset);
	}
	else
	{
		printf("LL2 Heap : Not used!\n");
	}

	if (gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].scratchAddr != NULL)
	{
		printf("LL2 scratch : size %d (0x%x), used %d (0x%x)\n",
    		gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].scratchSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].scratchSize,
			gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].maxScratchSizeUsed, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].maxScratchSizeUsed);
	}
	else
	{
		printf("LL2 scratch : Not used!\n");
	}

	if (gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].heapAddr != NULL)
	{
		printf("L1 Heap : size %d (0x%x), used %d (0x%x)\n",
    		gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].heapSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].heapSize,
			gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].heapAllocOffset, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].heapAllocOffset);
	}
	else
	{
		printf("L1 Heap : Not used!\n");
	}

	if (gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].scratchAddr != NULL)
	{
		printf("L1 scratch : size %d (0x%x), used %d (0x%x)\n",
    		gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].scratchSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].scratchSize,
			gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].maxScratchSizeUsed, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].maxScratchSizeUsed);
	}
	else
	{
		printf("L1 scratch : Not used!\n");
	}

	if (gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].heapAddr != NULL)
	{
		printf("HSRAM Heap : size %d (0x%x), used %d (0x%x)\n",
    		gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].heapSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].heapSize,
			gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].heapAllocOffset, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].heapAllocOffset);
	}
	else
	{
		printf("HSRAM Heap : Not used!\n");
	}

	if (gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].scratchAddr != NULL)
	{
		printf("HSRAM scratch : size %d (0x%x), used %d (0x%x)\n",
    		gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].scratchSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].scratchSize,
			gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].maxScratchSizeUsed, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].maxScratchSizeUsed);
	}
	else
	{
		printf("HSRAM scratch : Not used!\n");
	}    



}

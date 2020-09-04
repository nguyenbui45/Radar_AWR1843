/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 *  ======== MemoryP_tirtos.c ========
 */
#include <ti/drivers/osal/MemoryP.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
/*
 *  ======== MemoryP_ctrlAlloc ========
 */
void* MemoryP_ctrlAlloc(uint32_t size, uint8_t alignment)
{
    uint32_t *memAddr = NULL;
    uint32_t *temp = NULL;
    uint8_t memOffset = 0;
    uint8_t addrAlign = (alignment / 8);
    memAddr = malloc (size);
    
    if(addrAlign > 1)
    {
        memOffset = ((uint32_t)memAddr % addrAlign);
        while(memOffset != 0)
        {
            free(memAddr);
            
            temp = malloc(addrAlign - memOffset);
            memAddr = malloc(size);
            free(temp);
        }
    }
    
    return(memAddr);
}

/*
 *  ======== MemoryP_ctrlFree ========
 */
void MemoryP_ctrlFree(void* ptr, uint32_t size)
{
    /* Free the memory to the appropriate heap: */
    free(ptr);
}

/*
 *  ======== MemoryP_dataAlloc ========
 */
void* MemoryP_dataAlloc(uint32_t size, uint8_t alignment)
{
    uint32_t *memAddr = NULL;
    uint32_t *temp = NULL;
    uint8_t memOffset = 0;
    uint8_t addrAlign = alignment/8;
    memAddr = malloc (size);
    
    if(addrAlign > 1)
    {
        memOffset = ((uint32_t)memAddr%addrAlign);
        while(memOffset != 0)
        {
            free(memAddr);
            
            temp = malloc(addrAlign - memOffset);
            memAddr = malloc(size);
            free(temp);
        }
    }
    
    return(memAddr);
}

/*
 *  ======== MemoryP_dataFree ========
 */
void MemoryP_dataFree(void* ptr, uint32_t size)
{
    /* Free the memory to the appropriate heap: */
    free(ptr);
}


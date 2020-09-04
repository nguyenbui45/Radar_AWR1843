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
 *  ======== HwiP_nonos.c ========
 */

#include <ti/drivers/osal/HwiP.h>
#include <ti/drivers/osal/DebugP.h>

#ifdef SUBSYS_MSS
#include "sys_vim.h"

#define MAX_INTERRUPTS          128
volatile static uint8_t g_vimInitStatus = 0;
uint8_t g_intToPriorityMap[MAX_INTERRUPTS] = {0};
void switch_to_user_mode(void)
{
    asm("   DSB             ");
    asm("   CPS      #0x10  ");
    asm("   BX       lr     ");
}
void switch_to_system_mode(void)
{
    asm("  MOV      r0, #0 ");
    asm("  SVC      #0     ");
    asm("  BX       lr     ");
}

#endif

#ifdef SUBSYS_DSS
#include "interrupt.h"

#define MAX_INTERRUPTS          128
volatile static uint8_t g_dspIntInitStatus = 0;
#endif

/*
 *  ======== HwiP_DispatchEntry ========
 */
typedef struct HwiP_DispatchEntry {
    HwiP_Fxn entry;
    uintptr_t arg;
    uint32_t intNum;
} HwiP_DispatchEntry;

HwiP_DispatchEntry HwiP_dispatchTable[MAX_INTERRUPTS] = {(HwiP_Fxn)0, 0, 0};
HwiP_DispatchEntry hwi;
uint32_t g_channelNum;

/*
 *  ======== HwiP_dispatch ========
 */
void HwiP_dispatch(void)
{
    hwi = HwiP_dispatchTable[g_channelNum];
    if (hwi.entry) {
        (hwi.entry)(hwi.arg);
    }
}

/*
 *  ======== HwiP_clearInterrupt ========
 */
void HwiP_clearInterrupt(int32_t interruptNum)
{
#ifdef SUBSYS_MSS
    uint32_t priority = g_intToPriorityMap[interruptNum];
    if(priority != 0)
    {
        vimIntClear(priority);
    }
#endif
#ifdef SUBSYS_DSS
    IntEventClear((uint32_t)interruptNum);
#endif
}

/*
 *  ======== HwiP_create ========
 */
HwiP_Handle HwiP_create(int32_t interruptNum, HwiP_Fxn hwiFxn,
                        HwiP_Params *params)
{
#ifdef SUBSYS_MSS
    uint32_t priority = 0;
    if(g_vimInitStatus == 0)
    {
        switch_to_system_mode();
        /* Initialise vim module if not done already */
        vimInit();
        vimInterruptsInit();
        g_vimInitStatus = 1;
    }

    if(params->priority < MAX_INTERRUPTS)
    {
        priority = params->priority;
    }
    else
    {
        priority = interruptNum;
    }
    /* map the interrupt number to the interrupt handler */
    vimChannelMap(interruptNum, priority, HwiP_dispatch);
    HwiP_dispatchTable[priority].intNum = interruptNum;
    HwiP_dispatchTable[priority].entry = hwiFxn;
    HwiP_dispatchTable[priority].arg = params->arg;
    g_intToPriorityMap[interruptNum] = priority;
        
    if(params->enableInt)
    {
        vimEnableInterrupt(priority, SYS_IRQ);
    }
    
    return((HwiP_Handle)interruptNum);
#endif
#ifdef SUBSYS_DSS
    if(g_dspIntInitStatus == 0)
    {
        IntDSPINTCInit();
        IntEventCombineInit(4,5,6,7);
        IntGlobalEnable();
        g_dspIntInitStatus = 1;
    }
    HwiP_dispatchTable[interruptNum].intNum = interruptNum;
    HwiP_dispatchTable[interruptNum].entry = hwiFxn;
    HwiP_dispatchTable[interruptNum].arg = params->arg;
    IntEventCombineRegister(interruptNum, HwiP_dispatch);
    IntEventCombineAdd(interruptNum);

    return((HwiP_Handle)interruptNum);
#endif
}

/*
 *  ======== HwiP_delete ========
 */
HwiP_Status HwiP_delete(HwiP_Handle handle)
{
#ifdef SUBSYS_MSS
    uint32_t priority;
    DebugP_assert(handle != NULL);
    
    priority = g_intToPriorityMap[(int)handle];
    if(priority != 0)
    {
        vimDisableInterrupt(priority);
        vimChannelMap((int)handle, priority, &phantomInterrupt);
        HwiP_dispatchTable[priority].entry = (HwiP_Fxn)0;
        HwiP_dispatchTable[priority].arg =  0;
        HwiP_dispatchTable[priority].intNum =  0;
        g_intToPriorityMap[(int)handle] = 0;
        return (HwiP_OK);
    }
    else
    {
        return (HwiP_FAILURE);
    }
#endif
#ifdef SUBSYS_DSS
    HwiP_dispatchTable[(uint32_t)handle].entry = (HwiP_Fxn)0;
    HwiP_dispatchTable[(uint32_t)handle].arg =  0;
    HwiP_dispatchTable[(uint32_t)handle].intNum =  0;
    IntEventCombineRemove((uint32_t)handle);
    IntEventCombineRegister((uint32_t)handle, NULL);
    return(HwiP_OK);
#endif
}

/*
 *  ======== HwiP_disable ========
 */
uintptr_t HwiP_disable(void)
{
#ifdef SUBSYS_MSS
    enableDisableIrqFiq(0x4);
    return (0);
#endif
#ifdef SUBSYS_DSS
    return IntGlobalDisable();
#endif
    

}

/*
 *  ======== HwiP_disableInterrupt ========
 */
void HwiP_disableInterrupt(int32_t interruptNum)
{
#ifdef SUBSYS_MSS
    uint32_t priority = g_intToPriorityMap[interruptNum];
    if(priority != 0)
    {
        vimDisableInterrupt(priority);
    }    
#endif
#ifdef SUBSYS_DSS
    IntEventCombineRemove((uint32_t)interruptNum);
#endif
}

/*
 *  ======== HwiP_enableInterrupt ========
 */
void HwiP_enableInterrupt(int32_t interruptNum)
{
#ifdef SUBSYS_MSS
    uint32_t priority = g_intToPriorityMap[interruptNum];
    if(priority != 0)
    {
        vimEnableInterrupt(priority, SYS_IRQ);
    }  
#endif
#ifdef SUBSYS_DSS
    /* Add the EDMA-B DONE system event to the Event Combiner */
    IntEventCombineAdd((uint32_t)interruptNum);
#endif
}

/*
 *  ======== HwiP_Params_init ========
 */
void HwiP_Params_init(HwiP_Params *params)
{
    params->name = NULL;
    params->arg = 0;
    params->enableInt = true;
#ifdef SUBSYS_MSS
    params->priority = 128;
    params->type = HwiP_Type_IRQ;
#endif
#ifdef SUBSYS_DSS
    params->priority = 0;
    params->type = HwiP_Type_IRQ;
#endif
}

/*
 *  ======== HwiP_restore ========
 */
void HwiP_restore(uintptr_t key)
{
#ifdef SUBSYS_MSS
    enableDisableIrqFiq(0x1);
#endif
#ifdef SUBSYS_DSS
    IntGlobalRestore((uint32_t)key);
#endif
}

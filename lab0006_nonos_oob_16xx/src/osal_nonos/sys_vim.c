/** @file sys_vim.c
 *   @brief VIM Driver Implementation File
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
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
 */

/* Include Header Files */
#include <stdint.h>
#include "sys_vim.h"
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/soc/include/reg_rcm.h>
#include <ti/common/sys_common.h>

#define rcmREG   ((RCMRegs*) SOC_XWR16XX_MSS_RCM_BASE_ADDRESS)
/** @typedef config_value_type_t
*   @brief config type Type Definition
*
*   This type is used to specify the Initial and Current value.
*/
#define InitialValue                  ((uint8_t)0U)
#define CurrentValue                  ((uint8_t)1U)

typedef volatile uint64_t        REG64;
typedef volatile uint32_t        REG32;
typedef volatile uint16_t        REG16;
typedef volatile uint8_t         REG8;

/*! \brief
 * Registers Read/Write MACROS
 */
#define M_REG_WRITE8(w_addr, c_data)       (*((REG8  *)((w_addr)))) = ((uint8_t)((c_data)))
#define M_REG_WRITE16(w_addr, h_data)      (*((REG16 *)((w_addr)))) = ((uint16_t)((h_data)))
#define M_REG_WRITE32(w_addr, w_data)      (*((REG32 *)((w_addr)))) = ((uint32_t)((w_data)))
#define M_REG_WRITE64(w_addr, l_data)      (*((REG64 *)((w_addr)))) = ((uint64)((l_data)))

#define M_REG_READ8(w_addr)                (*((REG8  *)((w_addr))))
#define M_REG_READ16(w_addr)               (*((REG16 *)((w_addr))))
#define M_REG_READ32(w_addr)               (*((REG32 *)((w_addr))))
#define M_REG_READ64(w_addr)               (*((REG64 *)((w_addr))))

/*! \brief
 * 32 bit register field read - bit fields input
 */
#define M_REG_BITS_READ32(w_addr, c_ebit, c_sbit)   \
            (((M_REG_READ32((w_addr))) & (M_ONES((c_ebit), (c_sbit)))) >> (c_sbit))

/*! \brief
 * Generates a pattern of ones between end bit and start bit
 * M_ONES(7,4) will give 0x000000F0
 */
#define M_ONES(c_ebit, c_sbit) (((1U << (((c_ebit) - (c_sbit)) + 1U)) - 1U) << (c_sbit))

/*! \brief
 * Generates a pattern of zeros between end bit and start bit
 * M_MASK(7,4) will give 0xFFFFFF0F
 */
#define M_MASK(c_ebit, c_sbit) (~(M_ONES((c_ebit), (c_sbit))))

/*! \brief
 * Bounds the value before writing to register
 * M_VAL_BOUND(0x1F, 7, 4) will remove the extra bit resulting in 0xF
 * TODO: Generate a warning if extra bit found
 */
#define M_VAL_BOUND(val, c_ebit, c_sbit)   ((M_ONES((c_ebit), (c_sbit)) >> (c_sbit)) & (val))

/** @def GET_BIT_VALUE
*   @brief Get multiple bit values from given location
*/
#define GET_BIT_VALUE(data, noOfBits, location)     ((((uint32_t)(data)) >> (location)) & (((uint32_t)((uint32_t)1U << (noOfBits))) - (uint32_t)1U))

/* Vim Ram Definition */
/** @struct vimRam
*   @brief Vim Ram Definition
*
*   This type is used to access the Vim Ram.
*/
/** @typedef vimRAM_t
*   @brief Vim Ram Type Definition
*
*   This type is used to access the Vim Ram.
*/
typedef volatile struct vimRam
{
    t_isrFuncPTR ISR[VIM_CHANNELS];
} vimRAM_t;

#pragma DATA_SECTION(s_vimRAM, ".tcmVimRam");
vimRAM_t  s_vimRAM;
vimRAM_t*  vimRAM  = &s_vimRAM;

#pragma CODE_STATE(vimParityErrorHandler, 32)
#pragma INTERRUPT(vimParityErrorHandler, IRQ)
#pragma WEAK(vimParityErrorHandler)

vim_nesting_handler_t s_vim_nesting_handler;

/** @fn void vimFiqDispatcher(void)
*   @brief This is C function of Fast Interrupt handler.
*
*
*/
#pragma CODE_STATE(vimFiqDispatcher, 32)
#pragma CODE_SECTION(vimFiqDispatcher, ".startup");

/** @fn uint32_t regWriteReadback(volatile uint32_t *regAddr, uint32_t endBit, uint32_t startBit, uint32_t wrValue)
*   @brief Write a register and Readback function
*   @return error flag
*
*   This function writes a register and reads back and compares the written values
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
static inline uint32_t regWriteReadback(volatile uint32_t *regAddr, uint32_t endBit, \
                              uint32_t startBit, uint32_t wrValue)
{
    uint32_t readVal = 0U;
    uint32_t wrStatus = 0U;
    uint32_t maskedWrValue;
    uint32_t maskVal = 0U;
    uint32_t boundVal = 0U;


    if ((endBit == 31U) && (startBit == 0U))
    {
        /* Write */
        M_REG_WRITE32(regAddr, wrValue);

        /* Read Back */
        readVal = M_REG_READ32(regAddr);

        /* Set Status */
        wrStatus = readVal ^ wrValue;
    }
    else
    {
        /* Read Register */
        readVal = M_REG_READ32(regAddr);
        maskVal = M_MASK(endBit, startBit);
        boundVal = M_VAL_BOUND(wrValue, endBit, startBit);
        maskedWrValue = (readVal & maskVal) | (boundVal << startBit);
        /* Write */
        M_REG_WRITE32(regAddr, maskedWrValue);

/*TI_INSPECTED 87 S : MISRAC_2012_R18.4
 * "Reason - Safety feature readback option of the register" */
        readVal = (uint32_t)M_REG_BITS_READ32(regAddr, endBit, startBit);

        /* Set Status */
        wrStatus = readVal ^ wrValue;

    }

    return wrStatus;
}

void vimFiqDispatcher(void)
{
    uint8_t channel;
    t_isrFuncPTR p_vectAddress;
    uint32_t fiqindex = vimREG->FIQINDEX;

    /*
     * Read FIQ channel index(0 or 1) number from VIM to clear the pend flag
     */
    channel = (uint8_t)(fiqindex & 0xFFU);
    if (channel > 0U)
    {
        channel -= 1U;
        /*
         * The fast interrupt service call
             */

            /*
             * Get FIQ ISR address
             */
            p_vectAddress = vimRAM->ISR[channel + 1U];
            (*p_vectAddress)();
     }
     else
     {
         phantomInterrupt();
     }
}
/** @fn void vimIrqDispatcher(void)
*   @brief This is C function of IRQ Interrupt dispatch handler
*
*
*/
#pragma CODE_STATE(vimIrqDispatcher, 32)
#pragma CODE_SECTION(vimIrqDispatcher, ".startup");
extern uint32_t g_channelNum;
void vimIrqDispatcher(void)
{
    /*
     * All the local variables initialized before use, so not initializing to zero to save cycles
     */
    uint32_t channel,
          chRegIndex,
          chSlotIndex,
          chRegIndexCopy,
          prevChannel;

    uint32_t lastMaskSetLcl[4U],
           chanRegVal;

    t_isrFuncPTR p_vectAddress;
    uint32_t nestinglevel;

    /*
     * Get IRQ channel index number from VIM
     */
    channel = (vimREG->IRQINDEX & 0xFFU);
    prevChannel = g_channelNum;
    g_channelNum = channel - 1 ;

    if (channel > 0U)
    {
        channel -= 1U;
        nestinglevel = s_vim_nesting_handler.nestLevel;
        nestinglevel++;
        s_vim_nesting_handler.nestLevel = nestinglevel;

        /*
         * Resolve the bit fields
         */
        chRegIndex  = channel >> 5U;                       /* Find the register to configure */
        chRegIndexCopy = chRegIndex;
        chSlotIndex = channel - (uint8_t)(chRegIndex << 5U);      /* Find the offset of the type    */

        /*
         * Disable the interrupt mask in VIM channel
         */
        lastMaskSetLcl[chRegIndex] = s_vim_nesting_handler.lastMaskSet[chRegIndex];
        chanRegVal = (uint32_t)(0xFFFFFFFFUL << chSlotIndex);
        if(chRegIndex == 0U)
        {
            vimREG->REQMASKCLR0 = chanRegVal;
        }
        else if(chRegIndex == 1U)
        {
            vimREG->REQMASKCLR1 = chanRegVal;
        }
        else if(chRegIndex == 2U)
        {
            vimREG->REQMASKCLR2 = chanRegVal;
        }
        else
        {
            vimREG->REQMASKCLR3 = chanRegVal;
        }

        s_vim_nesting_handler.lastMaskSet[chRegIndex] &= (uint32_t)(~chanRegVal);
        chRegIndex++;

        while(chRegIndex < 4U)
        {
            lastMaskSetLcl[chRegIndex] = s_vim_nesting_handler.lastMaskSet[chRegIndex];
            if(chRegIndex == 0U)
            {
                vimREG->REQMASKCLR0 = 0xFFFFFFFFU;
            }
            else if(chRegIndex == 1U)
            {
                vimREG->REQMASKCLR1 = 0xFFFFFFFFU;
            }
            else if(chRegIndex == 2U)
            {
                vimREG->REQMASKCLR2 = 0xFFFFFFFFU;
            }
            else
            {
                vimREG->REQMASKCLR3 = 0xFFFFFFFFU;
            }
            s_vim_nesting_handler.lastMaskSet[chRegIndex] = 0x0U;
            chRegIndex++;
        }

        /*
         * Get IRQ ISR address
         */
        p_vectAddress = vimRAM->ISR[channel + 1U];

        /*
         * Perform DSB, ISB to finish all ongoing VIM register writes and Enable interrupt
         */
        enableDisableIrqFiq(0x1U);

        (*p_vectAddress)();

         enableDisableIrqFiq(0x4U);

        nestinglevel = s_vim_nesting_handler.nestLevel;
        nestinglevel--;
        s_vim_nesting_handler.nestLevel = nestinglevel;
        chRegIndex = chRegIndexCopy;
        
        g_channelNum = prevChannel;
        /*
         * Enable the interrupt mask in VIM channel
         */
        chanRegVal = lastMaskSetLcl[chRegIndex];
        if(chRegIndex == 0U)
        {
            vimREG->REQMASKSET0 = chanRegVal;
        }
        else if(chRegIndex == 1U)
        {
            vimREG->REQMASKSET1 = chanRegVal;
        }
        else if(chRegIndex == 2U)
        {
            vimREG->REQMASKSET2 = chanRegVal;
        }
        else
        {
            vimREG->REQMASKSET3 = chanRegVal;
        }
        s_vim_nesting_handler.lastMaskSet[chRegIndex] = chanRegVal;
        chRegIndex++;

        while(chRegIndex < 4U)
        {
            chanRegVal = lastMaskSetLcl[chRegIndex];
            if(chRegIndex == 0U)
            {
                vimREG->REQMASKSET0 = chanRegVal;
            }
            else if(chRegIndex == 1U)
            {
                vimREG->REQMASKSET1 = chanRegVal;
            }
            else if(chRegIndex == 2U)
            {
                vimREG->REQMASKSET2 = chanRegVal;
            }
            else
            {
                vimREG->REQMASKSET3 = chanRegVal;
            }
            s_vim_nesting_handler.lastMaskSet[chRegIndex] = chanRegVal;
            chRegIndex++;
        }
    }
    else
    {
        phantomInterrupt();
    }
 }

void vimParityErrorHandler(void)
{
     uint32_t regWrFailStatus, temp;
     /*
     * Perform this in automic region, as another interruption could cause issue
     */
    enableDisableIrqFiq(0x4U);

    /* Identify the corrupted address */
    uint32_t errorAddr = VIM_ADDERR;

    /* Identify the channel number */
    uint32_t errorChannel = ((errorAddr & 0x1FFU) >> 2U) - 1U;

    /* Correct the corrupted location */
    if(vimRAM != (vimRAM_t *)NULL_PTR)
    {
        vimRAM->ISR[errorChannel + 1U] = &phantomInterrupt;
    }

    /* Clear Parity Error Flag */
    temp = (uint32_t)&(VIM_PARFLG);
    regWrFailStatus = regWriteReadback((volatile uint32_t*)temp, M_THIRTY_ONE, M_ZERO, 0x1U);

    /*
     * The common IRQ handler clears and sets the mask which triggers the next interrupt of same
     * index as pending flag of this index is not being cleared
     */
    enableDisableIrqFiq(0x1U);

    /* Raise a fatal error if any of above register writes and comparison failed */
    DebugP_assert(M_ZERO == regWrFailStatus);
}


/** @fn void vimInit(void)
*   @brief Initializes VIM module
*
*   This function initializes VIM RAM and registers
*/
void vimInit(void)
{
    uint32_t regWrFailStatus, temp, regVal;

    /* Initialize VIM table */
    {
        uint32_t i;

        for (i = 0U; i < VIM_CHANNELS; i++)
        {
            vimRAM->ISR[i] = &phantomInterrupt;
            if(i == 1U)
            {
                //vimRAM->ISR[i] = &esmHighInterrupt;
            }
        }
    }

    /* Set Fall-Back Address Parity Error Register */
    /*AR_CODE_REVIEW 439 S MR:R.11.3 <REVIEWED> " Need to store the address of a function in a 32
     * bit register - Advisory as per MISRA" */
    temp = (uint32_t)&(VIM_FBPARERR);
    regWrFailStatus = regWriteReadback((volatile uint32_t*)temp, M_THIRTY_ONE, M_ZERO,\
                                                                 (uint32_t)&vimParityErrorHandler);

    /* set IRQ/FIQ priorities */
    regVal          = (uint32_t)((uint32_t)SYS_FIQ << 0U)
                    | (uint32_t)((uint32_t)SYS_FIQ << 1U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 2U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 3U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 4U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 5U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 6U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 7U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 8U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 9U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 10U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 11U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 12U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 13U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 14U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 15U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 16U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 17U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 18U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 19U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 20U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 21U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 22U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 23U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 24U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 25U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 26U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 27U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 28U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 29U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 30U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 31U);
    regWrFailStatus |= regWriteReadback(&vimREG->FIRQPR0, M_THIRTY_ONE, M_ZERO, regVal);

    regVal          = (uint32_t)((uint32_t)SYS_IRQ << 0U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 1U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 2U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 3U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 4U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 5U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 6U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 7U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 8U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 9U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 10U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 11U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 12U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 13U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 14U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 15U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 16U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 17U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 18U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 19U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 20U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 21U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 22U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 23U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 24U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 25U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 26U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 27U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 28U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 29U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 30U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 31U);
    regWrFailStatus |= regWriteReadback(&vimREG->FIRQPR1, M_THIRTY_ONE, M_ZERO, regVal);

    regVal          = (uint32_t)((uint32_t)SYS_IRQ << 0U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 1U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 2U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 3U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 4U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 5U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 6U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 7U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 8U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 9U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 10U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 11U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 12U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 13U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 14U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 15U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 16U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 17U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 18U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 19U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 20U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 21U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 22U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 23U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 24U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 25U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 26U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 27U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 28U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 29U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 30U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 31U);
    regWrFailStatus |= regWriteReadback(&vimREG->FIRQPR2, M_THIRTY_ONE, M_ZERO, regVal);

    regVal          = (uint32_t)((uint32_t)SYS_IRQ << 0U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 1U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 2U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 3U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 4U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 5U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 6U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 7U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 8U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 9U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 10U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 11U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 12U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 13U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 14U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 15U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 16U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 17U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 18U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 19U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 20U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 21U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 22U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 23U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 24U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 25U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 26U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 27U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 28U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 29U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 30U)
                    | (uint32_t)((uint32_t)SYS_IRQ << 31U);
   regWrFailStatus |= regWriteReadback(&vimREG->FIRQPR3, M_THIRTY_ONE, M_ZERO, regVal);

    /* clear pending interrupts */
   vimREG->INTREQ0      = (uint32_t)((uint32_t)1U << 0U)
                        | (uint32_t)((uint32_t)1U << 1U)
                        | (uint32_t)((uint32_t)1U << 2U)
                        | (uint32_t)((uint32_t)1U << 3U)
                        | (uint32_t)((uint32_t)1U << 4U)
                        | (uint32_t)((uint32_t)1U << 5U)
                        | (uint32_t)((uint32_t)1U << 6U)
                        | (uint32_t)((uint32_t)1U << 7U)
                        | (uint32_t)((uint32_t)1U << 8U)
                        | (uint32_t)((uint32_t)1U << 9U)
                        | (uint32_t)((uint32_t)1U << 10U)
                        | (uint32_t)((uint32_t)1U << 11U)
                        | (uint32_t)((uint32_t)1U << 12U)
                        | (uint32_t)((uint32_t)1U << 13U)
                        | (uint32_t)((uint32_t)1U << 14U)
                        | (uint32_t)((uint32_t)1U << 15U)
                        | (uint32_t)((uint32_t)1U << 16U)
                        | (uint32_t)((uint32_t)1U << 17U)
                        | (uint32_t)((uint32_t)1U << 18U)
                        | (uint32_t)((uint32_t)1U << 19U)
                        | (uint32_t)((uint32_t)1U << 20U)
                        | (uint32_t)((uint32_t)1U << 21U)
                        | (uint32_t)((uint32_t)1U << 22U)
                        | (uint32_t)((uint32_t)1U << 23U)
                        | (uint32_t)((uint32_t)1U << 24U)
                        | (uint32_t)((uint32_t)1U << 25U)
                        | (uint32_t)((uint32_t)1U << 26U)
                        | (uint32_t)((uint32_t)1U << 27U)
                        | (uint32_t)((uint32_t)1U << 28U)
                        | (uint32_t)((uint32_t)1U << 29U)
                        | (uint32_t)((uint32_t)1U << 30U)
                        | (uint32_t)((uint32_t)1U << 31U);

    vimREG->INTREQ1     = (uint32_t)((uint32_t)1U << 0U)
                        | (uint32_t)((uint32_t)1U << 1U)
                        | (uint32_t)((uint32_t)1U << 2U)
                        | (uint32_t)((uint32_t)1U << 3U)
                        | (uint32_t)((uint32_t)1U << 4U)
                        | (uint32_t)((uint32_t)1U << 5U)
                        | (uint32_t)((uint32_t)1U << 6U)
                        | (uint32_t)((uint32_t)1U << 7U)
                        | (uint32_t)((uint32_t)1U << 8U)
                        | (uint32_t)((uint32_t)1U << 9U)
                        | (uint32_t)((uint32_t)1U << 10U)
                        | (uint32_t)((uint32_t)1U << 11U)
                        | (uint32_t)((uint32_t)1U << 12U)
                        | (uint32_t)((uint32_t)1U << 13U)
                        | (uint32_t)((uint32_t)1U << 14U)
                        | (uint32_t)((uint32_t)1U << 15U)
                        | (uint32_t)((uint32_t)1U << 16U)
                        | (uint32_t)((uint32_t)1U << 17U)
                        | (uint32_t)((uint32_t)1U << 18U)
                        | (uint32_t)((uint32_t)1U << 19U)
                        | (uint32_t)((uint32_t)1U << 20U)
                        | (uint32_t)((uint32_t)1U << 21U)
                        | (uint32_t)((uint32_t)1U << 22U)
                        | (uint32_t)((uint32_t)1U << 23U)
                        | (uint32_t)((uint32_t)1U << 24U)
                        | (uint32_t)((uint32_t)1U << 25U)
                        | (uint32_t)((uint32_t)1U << 26U)
                        | (uint32_t)((uint32_t)1U << 27U)
                        | (uint32_t)((uint32_t)1U << 28U)
                        | (uint32_t)((uint32_t)1U << 29U)
                        | (uint32_t)((uint32_t)1U << 30U)
                        | (uint32_t)((uint32_t)1U << 31U);

    vimREG->INTREQ2     = (uint32_t)((uint32_t)1U << 0U)
                        | (uint32_t)((uint32_t)1U << 1U)
                        | (uint32_t)((uint32_t)1U << 2U)
                        | (uint32_t)((uint32_t)1U << 3U)
                        | (uint32_t)((uint32_t)1U << 4U)
                        | (uint32_t)((uint32_t)1U << 5U)
                        | (uint32_t)((uint32_t)1U << 6U)
                        | (uint32_t)((uint32_t)1U << 7U)
                        | (uint32_t)((uint32_t)1U << 8U)
                        | (uint32_t)((uint32_t)1U << 9U)
                        | (uint32_t)((uint32_t)1U << 10U)
                        | (uint32_t)((uint32_t)1U << 11U)
                        | (uint32_t)((uint32_t)1U << 12U)
                        | (uint32_t)((uint32_t)1U << 13U)
                        | (uint32_t)((uint32_t)1U << 14U)
                        | (uint32_t)((uint32_t)1U << 15U)
                        | (uint32_t)((uint32_t)1U << 16U)
                        | (uint32_t)((uint32_t)1U << 17U)
                        | (uint32_t)((uint32_t)1U << 18U)
                        | (uint32_t)((uint32_t)1U << 19U)
                        | (uint32_t)((uint32_t)1U << 20U)
                        | (uint32_t)((uint32_t)1U << 21U)
                        | (uint32_t)((uint32_t)1U << 22U)
                        | (uint32_t)((uint32_t)1U << 23U)
                        | (uint32_t)((uint32_t)1U << 24U)
                        | (uint32_t)((uint32_t)1U << 25U)
                        | (uint32_t)((uint32_t)1U << 26U)
                        | (uint32_t)((uint32_t)1U << 27U)
                        | (uint32_t)((uint32_t)1U << 28U)
                        | (uint32_t)((uint32_t)1U << 29U)
                        | (uint32_t)((uint32_t)1U << 30U)
                        | (uint32_t)((uint32_t)1U << 31U);

    vimREG->INTREQ3     = (uint32_t)((uint32_t)1U << 0U)
                        | (uint32_t)((uint32_t)1U << 1U)
                        | (uint32_t)((uint32_t)1U << 2U)
                        | (uint32_t)((uint32_t)1U << 3U)
                        | (uint32_t)((uint32_t)1U << 4U)
                        | (uint32_t)((uint32_t)1U << 5U)
                        | (uint32_t)((uint32_t)1U << 6U)
                        | (uint32_t)((uint32_t)1U << 7U)
                        | (uint32_t)((uint32_t)1U << 8U)
                        | (uint32_t)((uint32_t)1U << 9U)
                        | (uint32_t)((uint32_t)1U << 10U)
                        | (uint32_t)((uint32_t)1U << 11U)
                        | (uint32_t)((uint32_t)1U << 12U)
                        | (uint32_t)((uint32_t)1U << 13U)
                        | (uint32_t)((uint32_t)1U << 14U)
                        | (uint32_t)((uint32_t)1U << 15U)
                        | (uint32_t)((uint32_t)1U << 16U)
                        | (uint32_t)((uint32_t)1U << 17U)
                        | (uint32_t)((uint32_t)1U << 18U)
                        | (uint32_t)((uint32_t)1U << 19U)
                        | (uint32_t)((uint32_t)1U << 20U)
                        | (uint32_t)((uint32_t)1U << 21U)
                        | (uint32_t)((uint32_t)1U << 22U)
                        | (uint32_t)((uint32_t)1U << 23U)
                        | (uint32_t)((uint32_t)1U << 24U)
                        | (uint32_t)((uint32_t)1U << 25U)
                        | (uint32_t)((uint32_t)1U << 26U)
                        | (uint32_t)((uint32_t)1U << 27U)
                        | (uint32_t)((uint32_t)1U << 28U)
                        | (uint32_t)((uint32_t)1U << 29U)
                        | (uint32_t)((uint32_t)1U << 30U)
                        | (uint32_t)((uint32_t)1U << 31U);


    /* enable interrupts */
    vimREG->REQMASKCLR0 = (uint32_t)((uint32_t)1U << 0U)
                        | (uint32_t)((uint32_t)1U << 1U)
                        | (uint32_t)((uint32_t)0U << 2U)
                        | (uint32_t)((uint32_t)0U << 3U)
                        | (uint32_t)((uint32_t)0U << 4U)
                        | (uint32_t)((uint32_t)0U << 5U)
                        | (uint32_t)((uint32_t)0U << 6U)
                        | (uint32_t)((uint32_t)0U << 7U)
                        | (uint32_t)((uint32_t)0U << 8U)
                        | (uint32_t)((uint32_t)0U << 9U)
                        | (uint32_t)((uint32_t)0U << 10U)
                        | (uint32_t)((uint32_t)0U << 11U)
                        | (uint32_t)((uint32_t)0U << 12U)
                        | (uint32_t)((uint32_t)0U << 13U)
                        | (uint32_t)((uint32_t)0U << 14U)
                        | (uint32_t)((uint32_t)0U << 15U)
                        | (uint32_t)((uint32_t)0U << 16U)
                        | (uint32_t)((uint32_t)0U << 17U)
                        | (uint32_t)((uint32_t)0U << 18U)
                        | (uint32_t)((uint32_t)0U << 19U)
                        | (uint32_t)((uint32_t)0U << 20U)
                        | (uint32_t)((uint32_t)0U << 21U)
                        | (uint32_t)((uint32_t)0U << 22U)
                        | (uint32_t)((uint32_t)0U << 23U)
                        | (uint32_t)((uint32_t)0U << 24U)
                        | (uint32_t)((uint32_t)0U << 25U)
                        | (uint32_t)((uint32_t)0U << 26U)
                        | (uint32_t)((uint32_t)0U << 27U)
                        | (uint32_t)((uint32_t)0U << 28U)
                        | (uint32_t)((uint32_t)0U << 29U)
                        | (uint32_t)((uint32_t)0U << 30U)
                        | (uint32_t)((uint32_t)0U << 31U);

    vimREG->REQMASKCLR1 = (uint32_t)((uint32_t)0U << 0U)
                        | (uint32_t)((uint32_t)0U << 1U)
                        | (uint32_t)((uint32_t)0U << 2U)
                        | (uint32_t)((uint32_t)0U << 3U)
                        | (uint32_t)((uint32_t)0U << 4U)
                        | (uint32_t)((uint32_t)0U << 5U)
                        | (uint32_t)((uint32_t)0U << 6U)
                        | (uint32_t)((uint32_t)0U << 7U)
                        | (uint32_t)((uint32_t)0U << 8U)
                        | (uint32_t)((uint32_t)0U << 9U)
                        | (uint32_t)((uint32_t)0U << 10U)
                        | (uint32_t)((uint32_t)0U << 11U)
                        | (uint32_t)((uint32_t)0U << 12U)
                        | (uint32_t)((uint32_t)0U << 13U)
                        | (uint32_t)((uint32_t)0U << 14U)
                        | (uint32_t)((uint32_t)0U << 15U)
                        | (uint32_t)((uint32_t)0U << 16U)
                        | (uint32_t)((uint32_t)0U << 17U)
                        | (uint32_t)((uint32_t)0U << 18U)
                        | (uint32_t)((uint32_t)0U << 19U)
                        | (uint32_t)((uint32_t)0U << 20U)
                        | (uint32_t)((uint32_t)0U << 21U)
                        | (uint32_t)((uint32_t)0U << 22U)
                        | (uint32_t)((uint32_t)0U << 23U)
                        | (uint32_t)((uint32_t)0U << 24U)
                        | (uint32_t)((uint32_t)0U << 25U)
                        | (uint32_t)((uint32_t)0U << 26U)
                        | (uint32_t)((uint32_t)0U << 27U)
                        | (uint32_t)((uint32_t)0U << 28U)
                        | (uint32_t)((uint32_t)0U << 29U)
                        | (uint32_t)((uint32_t)0U << 30U)
                        | (uint32_t)((uint32_t)0U << 31U);

    vimREG->REQMASKCLR2 = (uint32_t)((uint32_t)0U << 0U)
                        | (uint32_t)((uint32_t)0U << 1U)
                        | (uint32_t)((uint32_t)0U << 2U)
                        | (uint32_t)((uint32_t)0U << 3U)
                        | (uint32_t)((uint32_t)0U << 4U)
                        | (uint32_t)((uint32_t)0U << 5U)
                        | (uint32_t)((uint32_t)0U << 6U)
                        | (uint32_t)((uint32_t)0U << 7U)
                        | (uint32_t)((uint32_t)0U << 8U)
                        | (uint32_t)((uint32_t)0U << 9U)
                        | (uint32_t)((uint32_t)0U << 10U)
                        | (uint32_t)((uint32_t)0U << 11U)
                        | (uint32_t)((uint32_t)0U << 12U)
                        | (uint32_t)((uint32_t)0U << 13U)
                        | (uint32_t)((uint32_t)0U << 14U)
                        | (uint32_t)((uint32_t)0U << 15U)
                        | (uint32_t)((uint32_t)0U << 16U)
                        | (uint32_t)((uint32_t)0U << 17U)
                        | (uint32_t)((uint32_t)0U << 18U)
                        | (uint32_t)((uint32_t)0U << 19U)
                        | (uint32_t)((uint32_t)0U << 20U)
                        | (uint32_t)((uint32_t)0U << 21U)
                        | (uint32_t)((uint32_t)0U << 22U)
                        | (uint32_t)((uint32_t)0U << 23U)
                        | (uint32_t)((uint32_t)0U << 24U)
                        | (uint32_t)((uint32_t)0U << 25U)
                        | (uint32_t)((uint32_t)0U << 26U)
                        | (uint32_t)((uint32_t)0U << 27U)
                        | (uint32_t)((uint32_t)0U << 28U)
                        | (uint32_t)((uint32_t)0U << 29U)
                        | (uint32_t)((uint32_t)0U << 30U)
                        | (uint32_t)((uint32_t)0U << 31U);

    vimREG->REQMASKCLR3 = (uint32_t)((uint32_t)0U << 0U)
                        | (uint32_t)((uint32_t)0U << 1U)
                        | (uint32_t)((uint32_t)0U << 2U)
                        | (uint32_t)((uint32_t)0U << 3U)
                        | (uint32_t)((uint32_t)0U << 4U)
                        | (uint32_t)((uint32_t)0U << 5U)
                        | (uint32_t)((uint32_t)0U << 6U)
                        | (uint32_t)((uint32_t)0U << 7U)
                        | (uint32_t)((uint32_t)0U << 8U)
                        | (uint32_t)((uint32_t)0U << 9U)
                        | (uint32_t)((uint32_t)0U << 10U)
                        | (uint32_t)((uint32_t)0U << 11U)
                        | (uint32_t)((uint32_t)0U << 12U)
                        | (uint32_t)((uint32_t)0U << 13U)
                        | (uint32_t)((uint32_t)0U << 14U)
                        | (uint32_t)((uint32_t)0U << 15U)
                        | (uint32_t)((uint32_t)0U << 16U)
                        | (uint32_t)((uint32_t)0U << 17U)
                        | (uint32_t)((uint32_t)0U << 18U)
                        | (uint32_t)((uint32_t)0U << 19U)
                        | (uint32_t)((uint32_t)0U << 20U)
                        | (uint32_t)((uint32_t)0U << 21U)
                        | (uint32_t)((uint32_t)0U << 22U)
                        | (uint32_t)((uint32_t)0U << 23U)
                        | (uint32_t)((uint32_t)0U << 24U)
                        | (uint32_t)((uint32_t)0U << 25U)
                        | (uint32_t)((uint32_t)0U << 26U)
                        | (uint32_t)((uint32_t)0U << 27U)
                        | (uint32_t)((uint32_t)0U << 28U)
                        | (uint32_t)((uint32_t)0U << 29U)
                        | (uint32_t)((uint32_t)0U << 30U)
                        | (uint32_t)((uint32_t)0U << 31U);

    /* Set Capture event sources */
    regVal           = ((uint32_t)((uint32_t)0U << 0U)
                     | (uint32_t)((uint32_t)0U << 16U));
    regWrFailStatus |= regWriteReadback(&vimREG->CAPEVT, M_THIRTY_ONE, M_ZERO, regVal);

    /*Intialise nesting support*/
    s_vim_nesting_handler.nestLevel   = 0U;
    s_vim_nesting_handler.lastMaskSet[0U] = 0x0U;
    s_vim_nesting_handler.lastMaskSet[1U] = 0x0U;
    s_vim_nesting_handler.lastMaskSet[2U] = 0x0U;
    s_vim_nesting_handler.lastMaskSet[3U] = 0x0U;

    /* Raise a fatal error if any of above register writes and comparison failed */
    DebugP_assert(M_ZERO == regWrFailStatus);

}

/** @fn void vimECCEnable(volatile uint32_t *vimParityptr)
*   @brief enables ECC on VIM module
*
*   This function enables ECC on VIM
*/
void vimECCEnable(volatile uint32_t *vimParityptr)
{
    /* VIM RAM Parity Enable */
    *vimParityptr = 0xAU;
}

/** @fn void vimInterruptsInit(void)
*   @brief Initialise the VIM channels
*
*
*   This function will initialise the vim channels.
*
*/
void vimInterruptsInit(void)
{
    /*
     * Map all interrupts to VIM module
     */
    vimChannelMap(0x0U, 0x0U, NULL);

    vimWakeupConfigSet(0x0U);

    /*
     * Enable interrupt
     */
    vimEnableInterrupt(0x0U, SYS_FIQ);

    enableDisableIrqFiq(0x1U);
}

/** @fn void vimChannelMap(uint32_t request, uint32_t channel, t_isrFuncPTR handler)
*   @brief Map selected interrupt request to the selected channel
*
*    @param[in] request: Interrupt request number 2..127
*    @param[in] channel: VIM Channel number 2..127
*    @param[in] handler: Address of the interrupt handler
*
*   This function will map selected interrupt request to the selected channel.
*
*/
void vimChannelMap(uint32_t request, uint32_t channel, t_isrFuncPTR handler)
{
    uint32_t i, j;
    uint32_t regWrFailStatus, regVal;
    i = channel >> 2U;              /* Find the register to configure */
    j = channel - (i << 2U);        /* Find the offset of the type    */
    j = ((uint32_t)3U - j);                     /* reverse the byte order         */
    j = j << 3U;                    /* find the bit location          */

    /*Mapping the required interrupt request to the required channel*/
    regVal = vimREG->CHANCTRL[i];
    regVal &= ~(uint32_t)((uint32_t)0xFFU << j);
    regVal |= (request << j);
    regWrFailStatus = regWriteReadback(&vimREG->CHANCTRL[i], M_THIRTY_ONE, M_ZERO, regVal);

    /*Updating VIMRAM*/
    vimRAM->ISR[channel + 1U] = handler;

    /* Raise a fatal error if any of above register writes and comparison failed */
    DebugP_assert(M_ZERO == regWrFailStatus);
}

/** @fn void vimEnableInterrupt(uint32_t channel, uint32_t inttype)
*   @brief Enable interrupt for the the selected channel
*
*    @param[in] channel: VIM Channel number 2..127
*    @param[in] inttype: Interrupt type
*                        - SYS_IRQ: Selected channel will be enabled as IRQ
*                        - SYS_FIQ: Selected channel will be enabled as FIQ
*
*   This function will enable interrupt for the selected channel.
*
*/
void vimEnableInterrupt(uint32_t channel, uint32_t inttype)
{
    uint32_t regWrFailStatus, regVal;
    if (channel >= 96U)
    {
        if(inttype == SYS_IRQ)
        {
            regVal = vimREG->FIRQPR3;
            regVal &= ~(uint32_t)((uint32_t)1U << (channel-96U));
            regWrFailStatus = regWriteReadback(&vimREG->FIRQPR3, M_THIRTY_ONE, M_ZERO, regVal);
        }
        else
        {
            regVal = vimREG->FIRQPR3;
            regVal |= ((uint32_t)1U << (channel-96U));
            regWrFailStatus = regWriteReadback(&vimREG->FIRQPR3, M_THIRTY_ONE, M_ZERO, regVal);
        }
        vimREG->REQMASKSET3 = (uint32_t)1U << (channel-96U);
        s_vim_nesting_handler.lastMaskSet[3U] |= (uint32_t)1U << (channel-96U);
    }
    else if (channel >= 64U)
    {
        if(inttype == SYS_IRQ)
        {
            regVal = vimREG->FIRQPR2;
            regVal &= ~(uint32_t)((uint32_t)1U << (channel-64U));
            regWrFailStatus = regWriteReadback(&vimREG->FIRQPR2, M_THIRTY_ONE, M_ZERO, regVal);
        }
        else
        {
            regVal = vimREG->FIRQPR2;
            regVal |= ((uint32_t)1U << (channel-64U));
            regWrFailStatus = regWriteReadback(&vimREG->FIRQPR2, M_THIRTY_ONE, M_ZERO, regVal);
        }
        vimREG->REQMASKSET2 = (uint32_t)1U << (channel-64U);
        s_vim_nesting_handler.lastMaskSet[2U] |= (uint32_t)1U << (channel-64U);
    }
    else if (channel >= 32U)
    {
        if(inttype == SYS_IRQ)
        {
            regVal = vimREG->FIRQPR1;
            regVal &= ~(uint32_t)((uint32_t)1U << (channel-32U));
            regWrFailStatus = regWriteReadback(&vimREG->FIRQPR1, M_THIRTY_ONE, M_ZERO, regVal);
        }
        else
        {
            regVal = vimREG->FIRQPR1;
            regVal |= ((uint32_t)1U << (channel-32U));
            regWrFailStatus = regWriteReadback(&vimREG->FIRQPR1, M_THIRTY_ONE, M_ZERO, regVal);
        }
        vimREG->REQMASKSET1 = (uint32_t)1U << (channel-32U);
        s_vim_nesting_handler.lastMaskSet[1U] |= (uint32_t)1U << (channel-32U);
    }
    else if (channel >= 2U)
    {
        if(inttype == SYS_IRQ)
        {
            regVal = vimREG->FIRQPR0;
            regVal &= ~(uint32_t)((uint32_t)1U << channel);
            regWrFailStatus = regWriteReadback(&vimREG->FIRQPR0, M_THIRTY_ONE, M_ZERO, regVal);
        }
        else
        {
            regVal = vimREG->FIRQPR0;
            regVal |= ((uint32_t)1U << channel);
            regWrFailStatus = regWriteReadback(&vimREG->FIRQPR0, M_THIRTY_ONE, M_ZERO, regVal);
        }
        vimREG->REQMASKSET0 = (uint32_t)1U << channel;
        s_vim_nesting_handler.lastMaskSet[0U] |= ((uint32_t)1U << channel);
    }
    else
    {
        regWrFailStatus = 0U;
        regVal = 0U;
    }
    /* Raise a fatal error if any of above register writes and comparison failed */
    DebugP_assert(M_ZERO == regWrFailStatus);

}

/** @fn void vimDisableInterrupt(uint32_t channel)
*   @brief Disable interrupt for the the selected channel
*
*    @param[in] channel: VIM Channel number 2..127
*
*   This function will disable interrupt for the selected channel.
*
*/
void vimDisableInterrupt(uint32_t channel)
{
    if (channel >= 96U)
    {
        vimREG->REQMASKCLR3 = (uint32_t)1U << (channel-96U);
        s_vim_nesting_handler.lastMaskSet[3U] &= ~(uint32_t)((uint32_t)1U << (channel-96U));
    }
    else if (channel >= 64U)
    {
        vimREG->REQMASKCLR2 = (uint32_t)1U << (channel-64U);
        s_vim_nesting_handler.lastMaskSet[2U] &= ~(uint32_t)((uint32_t)1U << (channel-64U));
    }
    else if (channel >=32U)
    {
        vimREG->REQMASKCLR1 = (uint32_t)1U << (channel-32U);
        s_vim_nesting_handler.lastMaskSet[1U] &= ~(uint32_t)((uint32_t)1U << (channel-32U));
    }
    else if (channel >= 2U)
    {
        vimREG->REQMASKCLR0 = (uint32_t)1U << channel;
        s_vim_nesting_handler.lastMaskSet[0U] &= ~(uint32_t)((uint32_t)1U << channel);
    }
    else
    {
        /* Empty */
    }
}

/** @fn void vimTriggerSoftInt(uint32_t channel, uint8 intEntryValue)
*   @brief Trigger Software interrupt for given channel
*
*   @param[in] channel: vim interrupt channel
*   @param[in] swIntData: system software interrupt data
*
*   This function will trigger software interrupt with assigning interrupt data to it.
*
*/
void vimTriggerSoftInt(uint32_t channel, uint8_t swIntData)
{
    uint32_t regWrFailStatus, regVal;
    if(channel == SOC_XWR16XX_MSS_SYS_SW0_INT)
    {
        regVal = rcmREG->SWIRQA;
        regVal |= ((0xADU << 8U) | (uint32_t)((uint32_t)swIntData << 0U));
        regWrFailStatus = regWriteReadback(&rcmREG->SWIRQA, M_THIRTY_ONE, M_ZERO, regVal);
    }
    else if(channel == SOC_XWR16XX_MSS_SYS_SW2_INT)
    {
        regVal = rcmREG->SWIRQB;
        regVal |= ((0xADU << 8U) | (uint32_t)((uint32_t)swIntData << 0U));
        regWrFailStatus = regWriteReadback(&rcmREG->SWIRQB, M_THIRTY_ONE, M_ZERO, regVal);
    }
    else if(channel == SOC_XWR16XX_MSS_SYS_SW4_INT)
    {
        regVal = rcmREG->SWIRQC;
        regVal |= ((0xADU << 8U) | (uint32_t)((uint32_t)swIntData << 0U));
        regWrFailStatus = regWriteReadback(&rcmREG->SWIRQC, M_THIRTY_ONE, M_ZERO, regVal);
    }
    else if(channel == SOC_XWR16XX_MSS_SYS_SW1_INT)
    {
        regVal = rcmREG->SWIRQA;
        regVal |= ((0xADU << 24U) | (uint32_t)((uint32_t)swIntData << 16U));
        regWrFailStatus = regWriteReadback(&rcmREG->SWIRQA, M_THIRTY_ONE, M_ZERO, regVal);
    }
    else if(channel == SOC_XWR16XX_MSS_SYS_SW3_INT)
    {
        regVal = rcmREG->SWIRQB;
        regVal |= ((0xADU << 24U) | (uint32_t)((uint32_t)swIntData << 16U));
        regWrFailStatus = regWriteReadback(&rcmREG->SWIRQB, M_THIRTY_ONE, M_ZERO, regVal);
    }
    else if(channel == SOC_XWR16XX_MSS_SYS_SW5_INT)
    {
        regVal = rcmREG->SWIRQC;
        regVal |= ((0xADU << 24U) | (uint32_t)((uint32_t)swIntData << 16U));
        regWrFailStatus = regWriteReadback(&rcmREG->SWIRQC, M_THIRTY_ONE, M_ZERO, regVal);
    }
    else
    {
        /* Generate an error */
        regWrFailStatus = 0U;
        regVal = 0U;
    }
    /* Raise a fatal error if any of above register writes and comparison failed */
    DebugP_assert(M_ZERO == regWrFailStatus);
}


/** @fn uint8 vimGetSwIntData(uint32_t channel)
*   @brief Get Software interrupt data for given SW Vim Channel
*
*    @param[in] channel: vim interrupt channel
*   This function will ret Software interrupt data for given SW Vim Channel
*
*/
uint8_t vimGetSwIntData(uint32_t channel)
{
    uint8_t swIntData;

    if(channel == SOC_XWR16XX_MSS_SYS_SW0_INT)
    {
        swIntData = (uint8_t)GET_BIT_VALUE(rcmREG->SWIRQA, M_EIGHT, M_ZERO);
    }
    else if(channel == SOC_XWR16XX_MSS_SYS_SW2_INT)
    {
        swIntData = (uint8_t)GET_BIT_VALUE(rcmREG->SWIRQB, M_EIGHT, M_ZERO);
    }
    else if(channel == SOC_XWR16XX_MSS_SYS_SW4_INT)
    {
        swIntData = (uint8_t)GET_BIT_VALUE(rcmREG->SWIRQC, M_EIGHT, M_ZERO);
    }
    else if(channel == SOC_XWR16XX_MSS_SYS_SW1_INT)
    {
        swIntData = (uint8_t)GET_BIT_VALUE(rcmREG->SWIRQA, M_EIGHT, M_SIXTEEN);
    }
    else if(channel == SOC_XWR16XX_MSS_SYS_SW3_INT)
    {
        swIntData = (uint8_t)GET_BIT_VALUE(rcmREG->SWIRQB, M_EIGHT, M_SIXTEEN);
    }
    else if(channel == SOC_XWR16XX_MSS_SYS_SW5_INT)
    {
        swIntData = (uint8_t)GET_BIT_VALUE(rcmREG->SWIRQC, M_EIGHT, M_SIXTEEN);
    }
    else
    {
        swIntData = 0U;
    }

    return swIntData;
}

/** @fn void vimDisableSoftInt(uint32_t channel)
*   @brief Mask Software interrupt for given channel
*
*    @param[in] channel: vim interrupt channel
*   This function will mask Software interrupt and its data value for given channel
*
*/
void vimDisableSoftInt(RCMRegs *rcmBaseAddr, uint32_t channel)
{
    if(channel == SOC_XWR16XX_MSS_SYS_SW0_INT)
    {
        rcmBaseAddr->SWIRQA &= 0xFFFF0000U;
    }
    else if(channel == SOC_XWR16XX_MSS_SYS_SW2_INT)
    {
        rcmBaseAddr->SWIRQB &= 0xFFFF0000U;
    }
    else if(channel == SOC_XWR16XX_MSS_SYS_SW4_INT)
    {
        rcmBaseAddr->SWIRQC &= 0xFFFF0000U;
    }
    else if(channel == SOC_XWR16XX_MSS_SYS_SW1_INT)
    {
        rcmBaseAddr->SWIRQA &= 0x0000FFFFU;
    }
    else if(channel == SOC_XWR16XX_MSS_SYS_SW3_INT)
    {
        rcmBaseAddr->SWIRQB &= 0x0000FFFFU;
    }
    else if(channel == SOC_XWR16XX_MSS_SYS_SW5_INT)
    {
        rcmBaseAddr->SWIRQC &= 0x0000FFFFU;
    }
    else
    {
        /* Disable All Software interrupts */
        rcmBaseAddr->SWIRQA = 0x0U;
        rcmBaseAddr->SWIRQB = 0x0U;
        rcmBaseAddr->SWIRQC = 0x0U;
    }
}

/** @fn void vimGetConfigValue(vim_config_reg_t *config_reg, uint8 type)
*   @brief Get the initial or current values of the configuration registers
*
*   @param[in] *config_reg: pointer to the struct to which the initial or current value of the
*                           configuration registers need to be stored
*   @param[in] type:     whether initial or current value of the configuration registers
*                        need to be stored
*                        - InitialValue: initial value of the configuration registers will be
*                                        stored in the struct pointed by config_reg
*                        - CurrentValue: initial value of the configuration registers will be
*                                        stored in the struct pointed by config_reg
*
*   This function will copy the initial or current value (depending on the parameter 'type') of the
*   configuration registers to the struct pointed by config_reg
*
*/
void vimGetConfigValue(vim_config_reg_t *config_reg, uint8_t type)
{
    if (type == InitialValue)
    {
        config_reg->CONFIG_FIRQPR0 = VIM_FIRQPR0_CONFIGVALUE;
        config_reg->CONFIG_FIRQPR1 = VIM_FIRQPR1_CONFIGVALUE;
        config_reg->CONFIG_FIRQPR2 = VIM_FIRQPR2_CONFIGVALUE;
        config_reg->CONFIG_FIRQPR3 = VIM_FIRQPR3_CONFIGVALUE;
        config_reg->CONFIG_REQMASKSET0 = VIM_REQMASKSET0_CONFIGVALUE;
        config_reg->CONFIG_REQMASKSET1 = VIM_REQMASKSET1_CONFIGVALUE;
        config_reg->CONFIG_REQMASKSET2 = VIM_REQMASKSET2_CONFIGVALUE;
        config_reg->CONFIG_REQMASKSET3 = VIM_REQMASKSET3_CONFIGVALUE;
        config_reg->CONFIG_WAKEMASKSET0 = VIM_WAKEMASKSET0_CONFIGVALUE;
        config_reg->CONFIG_WAKEMASKSET1 = VIM_WAKEMASKSET1_CONFIGVALUE;
        config_reg->CONFIG_WAKEMASKSET2 = VIM_WAKEMASKSET2_CONFIGVALUE;
        config_reg->CONFIG_WAKEMASKSET3 = VIM_WAKEMASKSET3_CONFIGVALUE;
        config_reg->CONFIG_CAPEVT = VIM_CAPEVT_CONFIGVALUE;
        config_reg->CONFIG_CHANCTRL[0U] = VIM_CHANCTRL0_CONFIGVALUE;
        config_reg->CONFIG_CHANCTRL[1U] = VIM_CHANCTRL1_CONFIGVALUE;
        config_reg->CONFIG_CHANCTRL[2U] = VIM_CHANCTRL2_CONFIGVALUE;
        config_reg->CONFIG_CHANCTRL[3U] = VIM_CHANCTRL3_CONFIGVALUE;
        config_reg->CONFIG_CHANCTRL[4U] = VIM_CHANCTRL4_CONFIGVALUE;
        config_reg->CONFIG_CHANCTRL[5U] = VIM_CHANCTRL5_CONFIGVALUE;
        config_reg->CONFIG_CHANCTRL[6U] = VIM_CHANCTRL6_CONFIGVALUE;
        config_reg->CONFIG_CHANCTRL[7U] = VIM_CHANCTRL7_CONFIGVALUE;
        config_reg->CONFIG_CHANCTRL[8U] = VIM_CHANCTRL8_CONFIGVALUE;
        config_reg->CONFIG_CHANCTRL[9U] = VIM_CHANCTRL9_CONFIGVALUE;
        config_reg->CONFIG_CHANCTRL[10U] = VIM_CHANCTRL10_CONFIGVALUE;
        config_reg->CONFIG_CHANCTRL[11U] = VIM_CHANCTRL11_CONFIGVALUE;
        config_reg->CONFIG_CHANCTRL[12U] = VIM_CHANCTRL12_CONFIGVALUE;
        config_reg->CONFIG_CHANCTRL[13U] = VIM_CHANCTRL13_CONFIGVALUE;
        config_reg->CONFIG_CHANCTRL[14U] = VIM_CHANCTRL14_CONFIGVALUE;
        config_reg->CONFIG_CHANCTRL[15U] = VIM_CHANCTRL15_CONFIGVALUE;
    }
    else
    {
        config_reg->CONFIG_FIRQPR0 = vimREG->FIRQPR0;
        config_reg->CONFIG_FIRQPR1 = vimREG->FIRQPR1;
        config_reg->CONFIG_FIRQPR2 = vimREG->FIRQPR2;
        config_reg->CONFIG_FIRQPR3 = vimREG->FIRQPR3;
        config_reg->CONFIG_REQMASKSET0 = vimREG->REQMASKSET0;
        config_reg->CONFIG_REQMASKSET1 = vimREG->REQMASKSET1;
        config_reg->CONFIG_REQMASKSET2 = vimREG->REQMASKSET2;
        config_reg->CONFIG_REQMASKSET3 = vimREG->REQMASKSET3;
        config_reg->CONFIG_WAKEMASKSET0 = vimREG->WAKEMASKSET0;
        config_reg->CONFIG_WAKEMASKSET1 = vimREG->WAKEMASKSET1;
        config_reg->CONFIG_WAKEMASKSET2 = vimREG->WAKEMASKSET2;
        config_reg->CONFIG_WAKEMASKSET3 = vimREG->WAKEMASKSET3;
        config_reg->CONFIG_CAPEVT = vimREG->CAPEVT;
        config_reg->CONFIG_CHANCTRL[0U] = vimREG->CHANCTRL[0U];
        config_reg->CONFIG_CHANCTRL[1U] = vimREG->CHANCTRL[1U];
        config_reg->CONFIG_CHANCTRL[2U] = vimREG->CHANCTRL[2U];
        config_reg->CONFIG_CHANCTRL[3U] = vimREG->CHANCTRL[3U];
        config_reg->CONFIG_CHANCTRL[4U] = vimREG->CHANCTRL[4U];
        config_reg->CONFIG_CHANCTRL[5U] = vimREG->CHANCTRL[5U];
        config_reg->CONFIG_CHANCTRL[6U] = vimREG->CHANCTRL[6U];
        config_reg->CONFIG_CHANCTRL[7U] = vimREG->CHANCTRL[7U];
        config_reg->CONFIG_CHANCTRL[8U] = vimREG->CHANCTRL[8U];
        config_reg->CONFIG_CHANCTRL[9U] = vimREG->CHANCTRL[9U];
        config_reg->CONFIG_CHANCTRL[10U] = vimREG->CHANCTRL[10U];
        config_reg->CONFIG_CHANCTRL[11U] = vimREG->CHANCTRL[11U];
        config_reg->CONFIG_CHANCTRL[12U] = vimREG->CHANCTRL[12U];
        config_reg->CONFIG_CHANCTRL[13U] = vimREG->CHANCTRL[13U];
        config_reg->CONFIG_CHANCTRL[14U] = vimREG->CHANCTRL[14U];
        config_reg->CONFIG_CHANCTRL[15U] = vimREG->CHANCTRL[15U];

    }
}

 /** @fn void vimWakeupConfigSet(uint32_t channel)
 *   @brief This function enables the wakeup mask of channel
 *
 *    @param[in] channel: Interrupt channel number
 *
 *   This function enables the wakeup mask of channel
 *
 */
void vimWakeupConfigSet(uint32_t channel)
{
    uint8_t  chRegIndex,
           chSlotIndex;
    uint32_t chanRegVal;
    uint32_t regWrFailStatus;

    chRegIndex  = (uint8_t)channel >> 5U;                              /* Find the register to configure */
    chSlotIndex = (uint8_t)channel - (uint8_t)(chRegIndex << 5U);        /* Find the offset of the type    */

    /*
     * Enable the wakeup VIM channel
     */
    chanRegVal = (uint32_t)((uint32_t)0x1UL << chSlotIndex);

    if(chRegIndex == 0U)
    {
        regWrFailStatus = regWriteReadback(&vimREG->WAKEMASKSET0, M_THIRTY_ONE, M_ZERO,\
                                                             (vimREG->WAKEMASKSET0 | chanRegVal));
    }
    else if(chRegIndex == 1U)
    {
        regWrFailStatus = regWriteReadback(&vimREG->WAKEMASKSET1, M_THIRTY_ONE, M_ZERO,\
                                                             (vimREG->WAKEMASKSET1 | chanRegVal));
    }
    else if(chRegIndex == 2U)
    {
        regWrFailStatus = regWriteReadback(&vimREG->WAKEMASKSET2, M_THIRTY_ONE, M_ZERO,\
                                                             (vimREG->WAKEMASKSET2 | chanRegVal));
    }
    else
    {
        regWrFailStatus = regWriteReadback(&vimREG->WAKEMASKSET3, M_THIRTY_ONE, M_ZERO,\
                                                             (vimREG->WAKEMASKSET3 | chanRegVal));
    }
    /* Raise a fatal error if any of above register writes and comparison failed */
    DebugP_assert(M_ZERO == regWrFailStatus);
}


/** @fn void vimWakeupConfigSet(uint32_t channel)
*   @brief This function disables the wakeup mask of channel
*
*    @param[in] channel: Interrupt channel number
*
*   This function disables the wakeup mask of channel
*
*/
void vimWakeupConfigClr(uint32_t channel)
{
    uint8_t  chRegIndex,
           chSlotIndex;
    uint32_t chanRegVal;
    uint32_t regWrFailStatus;

    chRegIndex  = (uint8_t)channel >> 5U;                              /* Find the register to configure */
    chSlotIndex = (uint8_t)channel - (uint8_t)(chRegIndex << 5U);        /* Find the offset of the type    */

    /*
     *  Disable the wakeup VIM channel
     */
    chanRegVal = (uint32_t)((uint32_t)0x1UL << chSlotIndex);

    if(chRegIndex == 0U)
    {
        regWrFailStatus = regWriteReadback(&vimREG->WAKEMASKCLR0, M_THIRTY_ONE, M_ZERO, chanRegVal);
    }
    else if(chRegIndex == 1U)
    {
        regWrFailStatus = regWriteReadback(&vimREG->WAKEMASKCLR1, M_THIRTY_ONE, M_ZERO, chanRegVal);
    }
    else if(chRegIndex == 2U)
    {
        regWrFailStatus = regWriteReadback(&vimREG->WAKEMASKCLR2, M_THIRTY_ONE, M_ZERO, chanRegVal);
    }
    else
    {
        regWrFailStatus = regWriteReadback(&vimREG->WAKEMASKCLR3, M_THIRTY_ONE, M_ZERO, chanRegVal);
    }
    /* Raise a fatal error if any of above register writes and comparison failed */
    DebugP_assert(M_ZERO == regWrFailStatus);

}


/** @fn void vimGetIsrAddress(uint32_t channel)
*   @brief This function returns Isr address of channel
*
*    @param[in] channel: Interrupt channel number
*
*   This function returns Isr address of channel
*
*/
t_isrFuncPTR vimGetIsrAddress(uint32_t channel)
{
    t_isrFuncPTR p_vectAddress;

    DebugP_assert(channel < (uint8_t)(VIM_CHANNELS + 1U));

    if (channel > 0U)
    {
        p_vectAddress = vimRAM->ISR[channel + 1U];
    }
    else
    {
        p_vectAddress = phantomInterrupt;
    }

    return(p_vectAddress);
}

/** @fn void phantomInterrupt(void)
*   @brief This ISR is a default phanthom ISR routine, when
*                       VIM fails to read right channel or index is not updated
*                       it fetches the RAM 0, that is phantom interrupt
*
*
*/
void phantomInterrupt(void)
{
    /*
     * Dummy phantom intrrupt routine - just return.
     */
}


/** @fn void vimCaptureOnEvent(uint32_t interrupt_used)
*   @brief This function is used to capture events in VIM.
*
*    @param[in] interrupt_used: interrupt number.
*
*/
void vimCaptureOnEvent(uint32_t interrupt_used)
{
    uint32_t regWrFailStatus, regVal;
    regVal = vimREG->CAPEVT;
    regVal |= interrupt_used;
    regWrFailStatus = regWriteReadback(&vimREG->CAPEVT, M_THIRTY_ONE, M_ZERO, regVal);

    /* Raise a fatal error if any of above register writes and comparison failed */
    DebugP_assert(M_ZERO == regWrFailStatus);
}

void vimIntClear(uint32_t intPriority)
{
    uint32_t channelReg;
    uint8_t channelOffset;

    channelReg = intPriority / 32;
    channelOffset = intPriority % 32;

    *((&vimREG->INTREQ0) + channelReg) = 1U << channelOffset;
}

/** @fn void enableDisableIrqFiq(uint32_t option)
*   @brief This is C function is to enable or disable the interrupts.
*
*    @param[in] option: Input to enable.disable intrs.
*
*/
void enableDisableIrqFiq(uint32_t option)
{
   switch (option)
   {
   case 1U:
      /* enable IRQ interrupt */
      asm(" mrs   r0, cpsr");
      asm(" msr   spsr_cxsf, r0"); /*  store svc_cpsr value in svc_spsr */
      asm(" bic   r0, r0, #0x80");
      asm(" msr   cpsr_cf, r0");
      break;

   case 2U:
      /* enable FIQ interrupt */
      asm(" mrs   r0, cpsr");
      asm(" msr   spsr_cxsf, r0"); /* store svc_cpsr value in svc_spsr */
      asm(" bic   r0, r0, #0x40");
      asm(" msr   cpsr_cf, r0");
      break;

   case 3U:
      /* enable both interrupts */
      asm(" mrs   r0, cpsr");
      asm(" msr   spsr_cxsf, r0"); /* store svc_cpsr value in svc_spsr */
      asm(" bic   r0, r0, #0xC0");
      asm(" msr   cpsr_cf, r0");
      break;

   case 4U:
      /* disable IRQ interrupt */
      asm(" mrs   r0, cpsr");
      asm(" msr   spsr_cxsf, r0"); /* store svc_cpsr value in svc_spsr */
      asm(" orr   r0, r0, #0x80");
      asm(" msr   cpsr_cf, r0");
      break;

   case 5U:
      /* disable FIQ interrupt */
      asm(" mrs   r0, cpsr");
      asm(" orr   r0, r0, #0x40");
      asm(" msr   cpsr_cf, r0");
      break;
   case 6U:
      /* disable both interrupts */
      asm(" mrs   r0, cpsr");
      asm(" msr   spsr_cxsf, r0"); /* store svc_cpsr value in svc_spsr */
      asm(" orr   r0, r0, #0xC0");
      asm(" msr   cpsr_cf, r0");
      break;
   default:
      /* not a valid interrupt */
      break;
   }
}


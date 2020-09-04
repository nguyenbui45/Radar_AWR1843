/**
 *  @file sys_vim.h
 *
 *  @brief Vectored Interrupt Module Header File
 *
 *   This file contains:
 *   - VIM Type Definitions
 *   - VIM General Definitions
 *   .
 *   which are relevant for Vectored Interrupt Controller.
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


#ifndef SYS_VIM_H
#define SYS_VIM_H

#include "osal_nonos/reg_vim.h"
#include <ti/drivers/soc/include/reg_rcm.h>

#ifdef __cplusplus
extern "C" {
#endif

/* VIM Type Definitions */

/** @typedef t_isrFuncPTR
*   @brief ISR Function Pointer Type Definition
*
*   This type is used to access the ISR handler.
*/
typedef void (*t_isrFuncPTR)(void);

/** @def SYS_IRQ
*   @brief Alias for IRQ interrupt Definition
*/
#define SYS_IRQ 0U

/** @def SYS_FIQ
*   @brief Alias for FIQ interrupt Definition
*/
#define SYS_FIQ 1U


/* VIM General Configuration */

#define VIM_CHANNELS 128U


/* Interrupt Handlers */

void phantomInterrupt(void);

#ifndef NULL_PTR
    #define NULL_PTR ((void *)0x0)
#endif
#define M_NULL                 (0U)
#define M_INVALID              (-1)
#define M_TRUE                 (uint8_t)(1U)
#define M_FALSE                (uint8_t)(0U)
#define M_LOGICAL_TRUE         (TRUE != FALSE)
#define M_LOGICAL_FALSE        (TRUE == FALSE)
#define M_SET                  (1U)
#define M_CLEAR                (0U)
#define M_ZERO                 (0U)
#define M_ONE                  (1U)
#define M_TWO                  (2U)
#define M_THREE                (3U)
#define M_FOUR                 (4U)
#define M_FIVE                 (5U)
#define M_SIX                  (6U)
#define M_SEVEN                (7U)
#define M_EIGHT                (8U)
#define M_NINE                 (9U)
#define M_TEN                  (10U)
#define M_ELEVEN               (11U)
#define M_TWELVE               (12U)
#define M_THIRTEEN             (13U)
#define M_FOURTEEN             (14U)
#define M_FIFTEEN              (15U)
#define M_SIXTEEN              (16U)
#define M_SEVENTEEN            (17U)
#define M_EIGHTEEN             (18U)
#define M_NINETEEN             (19U)
#define M_TWENTY               (20U)
#define M_TWENTY_ONE           (21U)
#define M_TWENTY_TWO           (22U)
#define M_TWENTY_THREE         (23U)
#define M_TWENTY_FOUR          (24U)
#define M_TWENTY_FIVE          (25U)
#define M_TWENTY_SIX           (26U)
#define M_TWENTY_SEVEN         (27U)
#define M_TWENTY_EIGHT         (28U)
#define M_TWENTY_NINE          (29U)
#define M_THIRTY               (30U)
#define M_THIRTY_ONE           (31U)
#define M_THIRTY_TWO           (32U)
#define M_SIXTY_FOUR           (64U)
#define M_SIXTY_FIVE           (65U)
#define M_NINTY_SIX            (96U)
#define M_NINTY_SEVEN          (97U)
#define M_HUNDRED              (100U)
#define M_ONE_TWENTY_EIGHT     (128U)
#define M_VALUE_1024           (1024U)

#define VIM_PARFLG      (*(volatile uint32_t *)0xFFFFFDECU)
#define VIM_PARCTL      ((volatile uint32_t *)0xFFFFFDF0U)
#define VIM_ADDERR      (*(volatile uint32_t *)0xFFFFFDF4U)
#define VIM_FBPARERR    (*(volatile uint32_t *)0xFFFFFDF8U)

#define VIMRAMPARLOC    (*(volatile uint32_t *)0xFFF82400U)
#define VIMRAMLOC       (*(volatile uint32_t *)0xFFF82000U)

/* Configuration registers */
typedef struct vim_config_reg
{
    uint32_t CONFIG_FIRQPR0;
    uint32_t CONFIG_FIRQPR1;
    uint32_t CONFIG_FIRQPR2;
    uint32_t CONFIG_FIRQPR3;
    uint32_t CONFIG_REQMASKSET0;
    uint32_t CONFIG_REQMASKSET1;
    uint32_t CONFIG_REQMASKSET2;
    uint32_t CONFIG_REQMASKSET3;
    uint32_t CONFIG_WAKEMASKSET0;
    uint32_t CONFIG_WAKEMASKSET1;
    uint32_t CONFIG_WAKEMASKSET2;
    uint32_t CONFIG_WAKEMASKSET3;
    uint32_t CONFIG_CAPEVT;
    uint32_t CONFIG_CHANCTRL[16U];
} vim_config_reg_t;


/*
 * VIM nested interrupt handling book keeping data structure
 */
typedef volatile struct
{
    uint32_t     lastIntChInd;
    uint32_t     nestLevel;
    uint32_t     rsvd;
    uint32_t     lastMaskSet[4U] ;
} vim_nesting_handler_t;

/* Configuration registers initial value */
#define VIM_FIRQPR0_CONFIGVALUE ( (uint32_t)((uint32_t)SYS_FIQ << 0U)\
                                | (uint32_t)((uint32_t)SYS_FIQ << 1U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 2U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 3U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 4U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 5U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 6U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 7U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 8U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 9U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 10U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 11U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 12U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 13U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 14U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 15U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 16U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 17U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 18U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 19U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 20U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 21U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 22U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 23U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 24U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 25U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 26U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 27U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 28U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 29U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 30U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 31U))

#define VIM_FIRQPR1_CONFIGVALUE ( (uint32_t)((uint32_t)SYS_IRQ << 0U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 1U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 2U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 3U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 4U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 5U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 6U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 7U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 8U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 9U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 10U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 11U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 12U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 13U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 14U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 15U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 16U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 17U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 18U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 19U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 20U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 21U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 22U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 23U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 24U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 25U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 26U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 27U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 28U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 29U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 30U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 31U))

#define VIM_FIRQPR2_CONFIGVALUE ( (uint32_t)((uint32_t)SYS_IRQ << 0U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 1U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 2U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 3U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 4U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 5U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 6U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 7U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 8U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 9U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 10U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 11U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 12U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 13U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 14U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 15U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 16U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 17U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 18U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 19U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 20U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 21U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 22U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 23U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 24U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 25U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 26U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 27U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 28U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 29U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 30U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 31U))

#define VIM_FIRQPR3_CONFIGVALUE ( (uint32_t)((uint32_t)SYS_IRQ << 0U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 1U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 2U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 3U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 4U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 5U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 6U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 7U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 8U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 9U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 10U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 11U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 12U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 13U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 14U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 15U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 16U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 17U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 18U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 19U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 20U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 21U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 22U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 23U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 24U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 25U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 26U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 27U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 28U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 29U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 30U)\
                                | (uint32_t)((uint32_t)SYS_IRQ << 31U))

#define VIM_REQMASKSET0_CONFIGVALUE ( (uint32_t)((uint32_t)1U << 0U)\
                                    | (uint32_t)((uint32_t)1U << 1U)\
                                    | (uint32_t)((uint32_t)0U << 2U)\
                                    | (uint32_t)((uint32_t)0U << 3U)\
                                    | (uint32_t)((uint32_t)0U << 4U)\
                                    | (uint32_t)((uint32_t)0U << 5U)\
                                    | (uint32_t)((uint32_t)0U << 6U)\
                                    | (uint32_t)((uint32_t)0U << 7U)\
                                    | (uint32_t)((uint32_t)0U << 8U)\
                                    | (uint32_t)((uint32_t)0U << 9U)\
                                    | (uint32_t)((uint32_t)0U << 10U)\
                                    | (uint32_t)((uint32_t)0U << 11U)\
                                    | (uint32_t)((uint32_t)0U << 12U)\
                                    | (uint32_t)((uint32_t)0U << 13U)\
                                    | (uint32_t)((uint32_t)0U << 14U)\
                                    | (uint32_t)((uint32_t)0U << 15U)\
                                    | (uint32_t)((uint32_t)0U << 16U)\
                                    | (uint32_t)((uint32_t)0U << 17U)\
                                    | (uint32_t)((uint32_t)0U << 18U)\
                                    | (uint32_t)((uint32_t)0U << 19U)\
                                    | (uint32_t)((uint32_t)0U << 20U)\
                                    | (uint32_t)((uint32_t)0U << 21U)\
                                    | (uint32_t)((uint32_t)0U << 22U)\
                                    | (uint32_t)((uint32_t)0U << 23U)\
                                    | (uint32_t)((uint32_t)0U << 24U)\
                                    | (uint32_t)((uint32_t)0U << 25U)\
                                    | (uint32_t)((uint32_t)0U << 26U)\
                                    | (uint32_t)((uint32_t)0U << 27U)\
                                    | (uint32_t)((uint32_t)0U << 28U)\
                                    | (uint32_t)((uint32_t)0U << 29U)\
                                    | (uint32_t)((uint32_t)0U << 30U)\
                                    | (uint32_t)((uint32_t)0U << 31U))

#define VIM_REQMASKSET1_CONFIGVALUE ( (uint32_t)((uint32_t)0U << 0U)\
                                    | (uint32_t)((uint32_t)0U << 1U)\
                                    | (uint32_t)((uint32_t)0U << 2U)\
                                    | (uint32_t)((uint32_t)0U << 3U)\
                                    | (uint32_t)((uint32_t)0U << 4U)\
                                    | (uint32_t)((uint32_t)0U << 5U)\
                                    | (uint32_t)((uint32_t)0U << 6U)\
                                    | (uint32_t)((uint32_t)0U << 7U)\
                                    | (uint32_t)((uint32_t)0U << 8U)\
                                    | (uint32_t)((uint32_t)0U << 9U)\
                                    | (uint32_t)((uint32_t)0U << 10U)\
                                    | (uint32_t)((uint32_t)0U << 11U)\
                                    | (uint32_t)((uint32_t)0U << 12U)\
                                    | (uint32_t)((uint32_t)0U << 13U)\
                                    | (uint32_t)((uint32_t)0U << 14U)\
                                    | (uint32_t)((uint32_t)0U << 15U)\
                                    | (uint32_t)((uint32_t)0U << 16U)\
                                    | (uint32_t)((uint32_t)0U << 17U)\
                                    | (uint32_t)((uint32_t)0U << 18U)\
                                    | (uint32_t)((uint32_t)0U << 19U)\
                                    | (uint32_t)((uint32_t)0U << 20U)\
                                    | (uint32_t)((uint32_t)0U << 21U)\
                                    | (uint32_t)((uint32_t)0U << 22U)\
                                    | (uint32_t)((uint32_t)0U << 23U)\
                                    | (uint32_t)((uint32_t)0U << 24U)\
                                    | (uint32_t)((uint32_t)0U << 25U)\
                                    | (uint32_t)((uint32_t)0U << 26U)\
                                    | (uint32_t)((uint32_t)0U << 27U)\
                                    | (uint32_t)((uint32_t)0U << 28U)\
                                    | (uint32_t)((uint32_t)0U << 29U)\
                                    | (uint32_t)((uint32_t)0U << 30U)\
                                    | (uint32_t)((uint32_t)0U << 31U))

#define VIM_REQMASKSET2_CONFIGVALUE ( (uint32_t)((uint32_t)0U << 0U)\
                                    | (uint32_t)((uint32_t)0U << 1U)\
                                    | (uint32_t)((uint32_t)0U << 2U)\
                                    | (uint32_t)((uint32_t)0U << 3U)\
                                    | (uint32_t)((uint32_t)0U << 4U)\
                                    | (uint32_t)((uint32_t)0U << 5U)\
                                    | (uint32_t)((uint32_t)0U << 6U)\
                                    | (uint32_t)((uint32_t)0U << 7U)\
                                    | (uint32_t)((uint32_t)0U << 8U)\
                                    | (uint32_t)((uint32_t)0U << 9U)\
                                    | (uint32_t)((uint32_t)0U << 10U)\
                                    | (uint32_t)((uint32_t)0U << 11U)\
                                    | (uint32_t)((uint32_t)0U << 12U)\
                                    | (uint32_t)((uint32_t)0U << 13U)\
                                    | (uint32_t)((uint32_t)0U << 14U)\
                                    | (uint32_t)((uint32_t)0U << 15U)\
                                    | (uint32_t)((uint32_t)0U << 16U)\
                                    | (uint32_t)((uint32_t)0U << 17U)\
                                    | (uint32_t)((uint32_t)0U << 18U)\
                                    | (uint32_t)((uint32_t)0U << 19U)\
                                    | (uint32_t)((uint32_t)0U << 20U)\
                                    | (uint32_t)((uint32_t)0U << 21U)\
                                    | (uint32_t)((uint32_t)0U << 22U)\
                                    | (uint32_t)((uint32_t)0U << 23U)\
                                    | (uint32_t)((uint32_t)0U << 24U)\
                                    | (uint32_t)((uint32_t)0U << 25U)\
                                    | (uint32_t)((uint32_t)0U << 26U)\
                                    | (uint32_t)((uint32_t)0U << 27U)\
                                    | (uint32_t)((uint32_t)0U << 28U)\
                                    | (uint32_t)((uint32_t)0U << 29U)\
                                    | (uint32_t)((uint32_t)0U << 30U)\
                                    | (uint32_t)((uint32_t)0U << 31U))

#define VIM_REQMASKSET3_CONFIGVALUE ( (uint32_t)((uint32_t)0U << 0U)\
                                    | (uint32_t)((uint32_t)0U << 1U)\
                                    | (uint32_t)((uint32_t)0U << 2U)\
                                    | (uint32_t)((uint32_t)0U << 3U)\
                                    | (uint32_t)((uint32_t)0U << 4U)\
                                    | (uint32_t)((uint32_t)0U << 5U)\
                                    | (uint32_t)((uint32_t)0U << 6U)\
                                    | (uint32_t)((uint32_t)0U << 7U)\
                                    | (uint32_t)((uint32_t)0U << 8U)\
                                    | (uint32_t)((uint32_t)0U << 9U)\
                                    | (uint32_t)((uint32_t)0U << 10U)\
                                    | (uint32_t)((uint32_t)0U << 11U)\
                                    | (uint32_t)((uint32_t)0U << 12U)\
                                    | (uint32_t)((uint32_t)0U << 13U)\
                                    | (uint32_t)((uint32_t)0U << 14U)\
                                    | (uint32_t)((uint32_t)0U << 15U)\
                                    | (uint32_t)((uint32_t)0U << 16U)\
                                    | (uint32_t)((uint32_t)0U << 17U)\
                                    | (uint32_t)((uint32_t)0U << 18U)\
                                    | (uint32_t)((uint32_t)0U << 19U)\
                                    | (uint32_t)((uint32_t)0U << 20U)\
                                    | (uint32_t)((uint32_t)0U << 21U)\
                                    | (uint32_t)((uint32_t)0U << 22U)\
                                    | (uint32_t)((uint32_t)0U << 23U)\
                                    | (uint32_t)((uint32_t)0U << 24U)\
                                    | (uint32_t)((uint32_t)0U << 25U)\
                                    | (uint32_t)((uint32_t)0U << 26U)\
                                    | (uint32_t)((uint32_t)0U << 27U)\
                                    | (uint32_t)((uint32_t)0U << 28U)\
                                    | (uint32_t)((uint32_t)0U << 29U)\
                                    | (uint32_t)((uint32_t)0U << 30U)\
                                    | (uint32_t)((uint32_t)0U << 31U))

#define VIM_WAKEMASKSET0_CONFIGVALUE    0xFFFFFFFFU
#define VIM_WAKEMASKSET1_CONFIGVALUE    0xFFFFFFFFU
#define VIM_WAKEMASKSET2_CONFIGVALUE    0xFFFFFFFFU
#define VIM_WAKEMASKSET3_CONFIGVALUE    0xFFFFFFFFU
#define VIM_CAPEVT_CONFIGVALUE          ((uint32_t)((uint32_t)0U << 0U)|(uint32_t)((uint32_t)0U << 16U))

#define VIM_CHANCTRL0_CONFIGVALUE       0x00010203U
#define VIM_CHANCTRL1_CONFIGVALUE       0x04050607U
#define VIM_CHANCTRL2_CONFIGVALUE       0x08090A0BU
#define VIM_CHANCTRL3_CONFIGVALUE       0x0C0D0E0FU
#define VIM_CHANCTRL4_CONFIGVALUE       0x10111213U
#define VIM_CHANCTRL5_CONFIGVALUE       0x14151617U
#define VIM_CHANCTRL6_CONFIGVALUE       0x18191A1BU
#define VIM_CHANCTRL7_CONFIGVALUE       0x1C1D1E1FU
#define VIM_CHANCTRL8_CONFIGVALUE       0x20212223U
#define VIM_CHANCTRL9_CONFIGVALUE       0x24252627U
#define VIM_CHANCTRL10_CONFIGVALUE      0x28292A2BU
#define VIM_CHANCTRL11_CONFIGVALUE      0x2C2D2E2FU
#define VIM_CHANCTRL12_CONFIGVALUE      0x30313233U
#define VIM_CHANCTRL13_CONFIGVALUE      0x34353637U
#define VIM_CHANCTRL14_CONFIGVALUE      0x38393A3BU
#define VIM_CHANCTRL15_CONFIGVALUE      0x3C3D3E3FU
#define VIM_CHANCTRL16_CONFIGVALUE      0x40414243U
#define VIM_CHANCTRL17_CONFIGVALUE      0x44454647U
#define VIM_CHANCTRL18_CONFIGVALUE      0x48494A4BU
#define VIM_CHANCTRL19_CONFIGVALUE      0x4C4D4E4FU
#define VIM_CHANCTRL20_CONFIGVALUE      0x50515253U
#define VIM_CHANCTRL21_CONFIGVALUE      0x54555657U
#define VIM_CHANCTRL22_CONFIGVALUE      0x58595A5BU
#define VIM_CHANCTRL23_CONFIGVALUE      0x5C5D5E5FU
#define VIM_CHANCTRL24_CONFIGVALUE      0x60616263U
#define VIM_CHANCTRL25_CONFIGVALUE      0x64656667U
#define VIM_CHANCTRL26_CONFIGVALUE      0x68696A6BU
#define VIM_CHANCTRL27_CONFIGVALUE      0x6C6D6E6FU
#define VIM_CHANCTRL28_CONFIGVALUE      0x70717273U
#define VIM_CHANCTRL29_CONFIGVALUE      0x74757677U
#define VIM_CHANCTRL30_CONFIGVALUE      0x78797A7BU
#define VIM_CHANCTRL31_CONFIGVALUE      0x7C7D7E7FU



/**
 * @defgroup VIM VIM
 * @brief Vectored Interrupt Manager
 *
 * The vectored interrupt manager (VIM) provides hardware assistance for prioritizing and controlling the
 * many interrupt sources present on a device. Interrupts are caused by events outside of the normal flow of
 * program execution.
 *
 * Related files:
 * - reg_vim.h
 * - sys_vim.h
 * - sys_vim.c
 *
 * @addtogroup VIM
 * @{
 */
/*VIM Interface functions*/
void vimInit(void);
void vimECCEnable(volatile uint32_t *vimParityptr);
void vimInterruptsInit(void);
void vimChannelMap(uint32_t request, uint32_t channel, t_isrFuncPTR handler);
void vimEnableInterrupt(uint32_t channel, uint32_t inttype);
void vimDisableInterrupt(uint32_t channel);
void vimTriggerSoftInt(uint32_t channel, uint8_t swIntData);
uint8_t vimGetSwIntData(uint32_t channel);
void vimDisableSoftInt(RCMRegs *rcmBaseAddr,uint32_t channel);
void vimGetConfigValue(vim_config_reg_t *config_reg, uint8_t type);
void vimParityErrorHandler(void);
void vimWakeupConfigSet(uint32_t channel);
void vimWakeupConfigClr(uint32_t channel);
t_isrFuncPTR vimGetIsrAddress(uint32_t channel);
void vimIrqDispatcher(void);
void vimFiqDispatcher(void);
void vimCaptureOnEvent(uint32_t interrupt_used);
void vimIntClear(uint32_t intPriority);
void enableDisableIrqFiq(uint32_t option);

/*@}*/

#ifdef __cplusplus
}
#endif

#endif

/**
 * \file  interrupt.h
 *
 * \brief Contains DSP interrupt related API declarations and system interrupt
 *        event numbers. (Note: event numbers are device-specific.)
 */

/*
* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
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

#ifndef INTERRUPT_H
#define INTERRUPT_H

#ifdef __cplusplus
extern "C" {
#endif



/******************************************************************************
**                             INTERNAL TYPE DEFINITIONS
******************************************************************************/
typedef void (*c674xISR)(void);


/******************************************************************************
**                             EXTERNAL MACRO DEFINITIONS
******************************************************************************/
/* API Parameter: cpuINT, ecmINTx
 *
 * Brief: Used to select DSP CPU maskable interrupt.
 *
 * Functions:
 *        IntRegister
 *        IntUnRegister
 *        IntEventMap
 *        IntEventCombineInit
 *        IntEnable
 *        IntDisable */
#define DSS_MASK_INT4         4
#define DSS_MASK_INT5         5
#define DSS_MASK_INT6         6
#define DSS_MASK_INT7         7
#define DSS_MASK_INT8         8
#define DSS_MASK_INT9         9
#define DSS_MASK_INT10        10
#define DSS_MASK_INT11        11
#define DSS_MASK_INT12        12
#define DSS_MASK_INT13        13
#define DSS_MASK_INT14        14
#define DSS_MASK_INT15        15

/* Functions:
 *        IntRegister
 *        IntUnRegister  */
#define DSS_MASK_NMI          1
#define DSS_MASK_EXC          1
#define DSS_MASK_RSV2         2
#define DSS_MASK_RSV3         3

/* API Parameter: ecmINTx ONLY
 *
 * Brief: Used to specify unused Event Combiner system events
 *
 * Functions:
 *         IntEventCombineInit */
#define ECM0_UNUSED          -1
#define ECM1_UNUSED          -1
#define ECM2_UNUSED          -1
#define ECM3_UNUSED          -1

/* API Parameter: sysINT
 *
 * Brief: Used to select system interrupts.
 *
 * Functions:
 *         IntEventMap
 *         IntEventSet
 *         IntEventClear
 *         IntEventCombineAdd
 *         IntEventCombineRemove
 *         IntEventCombineRegister
 *         ExcCombineAdd
 *         ExcCombineRemove */

/*            Event Combiners */
#define SYS_INT_EVT0                       0
#define SYS_INT_EVT1                       1
#define SYS_INT_EVT2                       2
#define SYS_INT_EVT3                       3

#define INTH_INT_ID_DSS_NMI                        1
#define INTH_INT_ID_DSS_TPTC0_IRQ_DONE             16
#define INTH_INT_ID_DSS_TPTC0_IRQ_ERR              17
#define INTH_INT_ID_DSS_TPTC1_IRQ_DONE             18
#define INTH_INT_ID_DSS_TPTC1_IRQ_ERR              19
#define INTH_INT_ID_DSS_TPCC0_IRQ_DONE              20
#define INTH_INT_ID_DSS_TPCC0_IRQ_ERR               21
#define INTH_INT_ID_DSS_CBUFF_IRQ                  22
// #define INTH_INT_ID_DSS_CSI_IRQ                 23
#define INTH_INT_ID_DSS_CBUFF_ERR_INTR             24
// #define INTH_INT_ID_DSS_CSI_VBUSP2OCP_SRESP_ERR 25
#define INTH_INT_ID_DSS_FRAME_START_IRQ            26
#define INTH_INT_ID_DSS_CHIRP_AVAIL_IRQ            27
#define INTH_INT_ID_DSS_VIN_ERROR_IRQ              28

#define INTH_INT_ID_DSS_HW_ACC_PARAM_DONE_IRQ      29
#define INTH_INT_ID_DSS_HW_ACC_DONE_IRQ            30
#define INTH_INT_ID_DSS_HW_ACC_ERR_IRQ             31
#define INTH_INT_ID_DSS_CSI_PARITY_ERR             32
#define INTH_CRC_IRQ                               33

#define INTH_INT_ID_DSS_TPTC4_IRQ_DONE             48
#define INTH_INT_ID_DSS_TPTC4_IRQ_ERR              49
#define INTH_INT_ID_DSS_TPTC5_IRQ_DONE             50
#define INTH_INT_ID_DSS_TPTC5_IRQ_ERR              51
#define INTH_INT_ID_DSS_TPTC6_IRQ_DONE             52
#define INTH_INT_ID_DSS_TPTC6_IRQ_ERR              53
#define INTH_INT_ID_DSS_TPTC7_IRQ_DONE             54
#define INTH_INT_ID_DSS_TPTC7_IRQ_ERR              55

#define INTH_INT_ID_DSS_DMMSWINT4                  60
#define INTH_INT_ID_DSS_DMMSWINT5                  61
#define INTH_INT_ID_DSS_DMMSWINT6                  62
#define INTH_INT_ID_DSS_DMMSWINT7                  63
#define INTH_INT_ID_DSS_TPTC2_IRQ_DONE             64
#define INTH_INT_ID_DSS_TPTC2_IRQ_ERR              65
#define INTH_INT_ID_DSS_TPTC3_IRQ_DONE             66
#define INTH_INT_ID_DSS_TPTC3_IRQ_ERR              67
#define INTH_INT_ID_DSS_TPCC1_IRQ_DONE             68
#define INTH_INT_ID_DSS_TPCC1_IRQ_ERR              69
#define INTH_INT_ID_DSS_ADC_PING_PONG              70
#define INTH_INT_ID_DSS_UART_REQ0                  71
#define INTH_INT_ID_DSS_UART_REQ1                  72
#define INTH_INT_ID_DSS_RTI0_OVERFLOW_0            73
#define INTH_INT_ID_DSS_RTI0_OVERFLOW_1            74
#define INTH_INT_ID_DSS_RTI0_0                     75
#define INTH_INT_ID_DSS_RTI0_1                     76
#define INTH_INT_ID_DSS_RTI0_2                     77
#define INTH_INT_ID_DSS_RTI0_3                     78
#define INTH_INT_ID_DSS_RTI1_OVERFLOW_0            79
#define INTH_INT_ID_DSS_RTI1_OVERFLOW_1            80
#define INTH_INT_ID_DSS_RTI1_0                     81
#define INTH_INT_ID_DSS_RTI1_1                     82
#define INTH_INT_ID_DSS_RTI1_2                     83
#define INTH_INT_ID_DSS_RTI1_3                     84
#define INTH_INT_ID_DSS_BSS_MAILBOX_FULL           85
#define INTH_INT_ID_DSS_BSS_MAILBOX_EMPTY          86
#define INTH_INT_ID_DSS_GPIO_0                     87
#define INTH_INT_ID_DSS_GPIO_1                     88
#define INTH_INT_ID_DSS_GPIO_2                     89
#define INTH_INT_ID_DSS_GPIO_3                     90
#define INTH_INT_ID_DSS_MSS_MAILBOX_FULL           91
#define INTH_INT_ID_DSS_MSS_MAILBOX_EMPTY          92
#define INTH_INT_ID_DSS_LOGICAL_FRAME_START        93

#define INTH_INT_ID_SCI_IRQ0  INTH_INT_ID_DSS_UART_REQ0
#define INTH_INT_ID_SCI_IRQ1  INTH_INT_ID_DSS_UART_REQ1

#define INTH_INT_ID_DSS_RTI_CNTR0_OVFL_IRQ   (96 + 24)
#define INTH_INT_ID_DSS_RTI_CNTR1_OVFL_IRQ   (96 + 25)
#define INTH_INT_ID_DSS_RTI_COMPARE0_IRQ     (96 + 26)
#define INTH_INT_ID_DSS_RTI_COMPARE1_IRQ     (96 + 27)
#define INTH_INT_ID_DSS_RTI_COMPARE2_IRQ     (96 + 28)
#define INTH_INT_ID_DSS_RTI_COMPARE3_IRQ     (96 + 29)


#define INTH_INT_ID_DSS_INTERR                 96             /*  */
#define INTH_INT_ID_DSS_EMC_IDMAERR            97             /*  */
#define INTH_INT_ID_DSS_PBISTINTERR            98             /*  */
#define INTH_INT_ID_DSS_EFINTA                 100            /*  */
#define INTH_INT_ID_DSS_EFINTB                 101            /*  */
#define INTH_INT_ID_DSS_PMC_ED                 113            /*  */
#define INTH_INT_ID_DSS_UMCED1                 116            /*  */
#define INTH_INT_ID_DSS_UMCED2                 117            /*  */
#define INTH_INT_ID_DSS_PDC_INT                118 // PDC_INT PDC PDC sleep interrupt
#define INTH_INT_ID_DSS_SYS_CMPA               119 // SYS_CMPA SYS CPU memory protection fault
#define INTH_INT_ID_DSS_L1P_CMPA               120 // L1P_CMPA L1P CPU memory protection fault
#define INTH_INT_ID_DSS_L1P_DMPA               121 // L1P_DMPA L1P DMA memory protection fault
#define INTH_INT_ID_DSS_L1D_CMPA               122 // L1D_CMPA L1D CPU memory protection fault
#define INTH_INT_ID_DSS_L1D_DMPA               123 // L1D_DMPA L1D DMA memory protection fault
#define INTH_INT_ID_DSS_L2_CMPA                124 // L2_CMPA L2 CPU memory protection fault
#define INTH_INT_ID_DSS_L2_DMPA                125 // L2_DMPA L2 DMA memory protection fault
#define INTH_INT_ID_DSS_EMC_CMPA               126 // EMC_CMPA EMC CPU memory protection fault
#define INTH_INT_ID_DSS_EMC_BUSERR             127 // EMC_BUSERR EMC Bus error interrupt

/******************************************************************************
**                             FUNCTION DEFINITIONS
******************************************************************************/
extern void IntDSPINTCInit (void);
extern void IntRegister (uint32_t cpuINT, void (*userISR)(void));
extern void IntUnRegister (uint32_t cpuINT);
extern void IntEventMap (uint32_t cpuINT, uint32_t sysINT);
extern void IntEventSet (uint32_t sysINT);
extern void IntEventClear(uint32_t sysINT);
extern void IntEventCombineInit(int ecmINT0, int ecmINT1, int ecmINT2, int ecmINT3);
extern void IntEventCombineAdd(uint32_t sysINT);
extern void IntEventCombineRemove(uint32_t sysINT);
extern void IntEventCombineRegister(uint32_t sysINT, void (*userISR)(void));
extern void IntEnable (uint32_t cpuINT);
extern void IntDisable (uint32_t cpuINT);
extern void IntReset (void);
extern void IntGlobalEnable (void);
extern uint32_t IntGlobalDisable (void);
extern void IntGlobalRestore (uint32_t restoreValue);
extern void ExcGlobalEnable (void);
extern void ExcCombineAdd(uint32_t sysINT);
extern void ExcCombineRemove(uint32_t sysINT);

#ifdef __cplusplus
}
#endif

#endif /* INTERRUPT_H */

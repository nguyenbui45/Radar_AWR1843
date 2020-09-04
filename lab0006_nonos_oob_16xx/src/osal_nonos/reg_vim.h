/** @file reg_vim.h
*   @brief VIM Register Layer Header File
*
*   This file contains:
*   - Definitions
*   - Types
*   .
*   which are relevant for the System driver.
*/

/*
* Copyright (C) 2009-2014 Texas Instruments Incorporated - TI web adress www.ti.com
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


#ifndef REG_VIM_H
#define REG_VIM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Vim Register Frame Definition */
/** @struct vimBase
*   @brief Vim Register Frame Definition
*
*   This type is used to access the Vim Registers.
*/
/** @typedef vimBASE_t
*   @brief VIM Register Frame Type Definition
*
*   This type is used to access the VIM Registers.
*/
typedef volatile struct vimBase
{
    uint32_t      IRQINDEX;         /* 0x0000       */
    uint32_t      FIQINDEX;         /* 0x0004       */
    uint32_t        rsvd1;          /* 0x0008       */
    uint32_t        rsvd2;          /* 0x000C       */
    uint32_t      FIRQPR0;          /* 0x0010       */
    uint32_t      FIRQPR1;          /* 0x0014       */
    uint32_t      FIRQPR2;          /* 0x0018       */
    uint32_t      FIRQPR3;          /* 0x001C       */
    uint32_t      INTREQ0;          /* 0x0020       */
    uint32_t      INTREQ1;          /* 0x0024       */
    uint32_t      INTREQ2;          /* 0x0028       */
    uint32_t      INTREQ3;          /* 0x002C       */
    uint32_t      REQMASKSET0;      /* 0x0030       */
    uint32_t      REQMASKSET1;      /* 0x0034       */
    uint32_t      REQMASKSET2;      /* 0x0038       */
    uint32_t      REQMASKSET3;      /* 0x003C       */
    uint32_t      REQMASKCLR0;      /* 0x0040       */
    uint32_t      REQMASKCLR1;      /* 0x0044       */
    uint32_t      REQMASKCLR2;      /* 0x0048       */
    uint32_t      REQMASKCLR3;      /* 0x004C       */
    uint32_t      WAKEMASKSET0;     /* 0x0050       */
    uint32_t      WAKEMASKSET1;     /* 0x0054       */
    uint32_t      WAKEMASKSET2;     /* 0x0058       */
    uint32_t      WAKEMASKSET3;     /* 0x005C       */
    uint32_t      WAKEMASKCLR0;     /* 0x0060       */
    uint32_t      WAKEMASKCLR1;     /* 0x0064       */
    uint32_t      WAKEMASKCLR2;     /* 0x0068       */
    uint32_t      WAKEMASKCLR3;     /* 0x006C       */
    uint32_t      IRQVECREG;        /* 0x0070       */
    uint32_t      FIQVECREG;        /* 0x0074       */
    uint32_t      CAPEVT;           /* 0x0078       */
    uint32_t        rsvd3;          /* 0x007C       */
    uint32_t      CHANCTRL[16U];    /* 0x0080-0x0BC */
} vimBASE_t;

#define vimREG ((vimBASE_t *)0xFFFFFE00U)

#ifdef __cplusplus
}
#endif

#endif

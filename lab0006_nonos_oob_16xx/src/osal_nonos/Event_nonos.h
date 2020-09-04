/*
 *  Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/ 
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

/*
 *  ======== event_nonos.h ========
 */


#ifndef EVENT_NONOS_H
#define EVENT_NONOS_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>

/* Id_00 */
#define Event_Id_00 (0x1)

/* Id_01 */
#define Event_Id_01 (0x2)

/* Id_02 */
#define Event_Id_02 (0x4)

/* Id_03 */
#define Event_Id_03 (0x8)

/* Id_04 */
#define Event_Id_04 (0x10)

/* Id_05 */
#define Event_Id_05 (0x20)

/* Id_06 */
#define Event_Id_06 (0x40)

/* Id_07 */
#define Event_Id_07 (0x80)

/* Id_08 */
#define Event_Id_08 (0x100)

/* Id_09 */
#define Event_Id_09 (0x200)

/* Id_10 */
#define Event_Id_10 (0x400)

/* Id_11 */
#define Event_Id_11 (0x800)

/* Id_12 */
#define Event_Id_12 (0x1000)

/* Id_13 */
#define Event_Id_13 (0x2000)

/* Id_14 */
#define Event_Id_14 (0x4000)

/* Id_15 */
#define Event_Id_15 (0x8000)

/* Id_16 */
#define Event_Id_16 (0x10000)

/* Id_17 */
#define Event_Id_17 (0x20000)

/* Id_18 */
#define Event_Id_18 (0x40000)

/* Id_19 */
#define Event_Id_19 (0x80000)

/* Id_20 */
#define Event_Id_20 (0x100000)

/* Id_21 */
#define Event_Id_21 (0x200000)

/* Id_22 */
#define Event_Id_22 (0x400000)

/* Id_23 */
#define Event_Id_23 (0x800000)

/* Id_24 */
#define Event_Id_24 (0x1000000)

/* Id_25 */
#define Event_Id_25 (0x2000000)

/* Id_26 */
#define Event_Id_26 (0x4000000)

/* Id_27 */
#define Event_Id_27 (0x8000000)

/* Id_28 */
#define Event_Id_28 (0x10000000)

/* Id_29 */
#define Event_Id_29 (0x20000000)

/* Id_30 */
#define Event_Id_30 (0x40000000)

/* Id_31 */
#define Event_Id_31 (0x80000000)

/* Id_NONE */
#define Event_Id_NONE (0)

/*!
 *  @brief    Wait forever define
 */
#define EventP_WAIT_FOREVER     0xFFFFFFFFU

/*!
 *  @brief    No wait define
 */
#define EventP_NO_WAIT          0

typedef struct ti_nonOS_Event_Params_t
{
    size_t __size;
    const char* name;
}ti_nonOS_Event_Params;


/* Object */
typedef struct ti_nonOS_Event_Object_t
{
    volatile unsigned int postedEvents;
    unsigned int andMask;
    unsigned int orMask;
    char __dummy;
}ti_nonOS_Event_Object;


typedef ti_nonOS_Event_Object* ti_nonOS_Event_Handle;
typedef ti_nonOS_Event_Handle  Event_Handle;


ti_nonOS_Event_Handle Event_create(ti_nonOS_Event_Params *__paramsPtr, char *dummy);
void Event_post( ti_nonOS_Event_Handle eventHandle, unsigned int eventId);
unsigned int Event_pend( ti_nonOS_Event_Handle eventHandle, unsigned int andMask, unsigned int orMask, unsigned int timeout );
unsigned int Event_checkEvents (ti_nonOS_Event_Object *event, unsigned int andMask, unsigned int orMask);


#ifdef __cplusplus
}
#endif

#endif /* EVENT_NONOS_H */


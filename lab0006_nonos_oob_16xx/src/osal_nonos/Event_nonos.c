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

#include "osal_nonos/Event_nonos.h"

ti_nonOS_Event_Handle Event_create(ti_nonOS_Event_Params *__paramsPtr, char *dummy )
{
    ti_nonOS_Event_Object *event_non_os;
    
    //dynamically allocating memory for the event struct
    event_non_os = (ti_nonOS_Event_Object *)malloc(sizeof(ti_nonOS_Event_Object));

    return ((ti_nonOS_Event_Handle)event_non_os);
}


/*
 *  ======== Event_checkEvents ========
 *  Checks postedEvents for matching event conditions.
 *  Returns matchingEvents if a match and consumes matched events,
 *  else returns 0 and consumes nothing.
 *  Called with ints disabled
 */
unsigned int Event_checkEvents (ti_nonOS_Event_Object *event, unsigned int andMask, unsigned int orMask)
{
    unsigned int matchingEvents;

    matchingEvents = orMask & event->postedEvents;

    if ((andMask & event->postedEvents) == andMask) {
        matchingEvents |= andMask;
    }

    if (matchingEvents) {
        /* consume ALL the matching events */
        event->postedEvents &= ~matchingEvents;
    }

    return (matchingEvents);
}

void Event_post(ti_nonOS_Event_Handle eventHandle, unsigned int eventId)
{
    ti_nonOS_Event_Object *event_non_os = (ti_nonOS_Event_Object *)eventHandle;
    
    /* or in this eventId */
    event_non_os->postedEvents |= eventId;
    
    /* check for match, consume matching eventIds if so. */
    //Event_checkEvents(eventHandle, event_non_os->andMask, event_non_os->orMask);

}


unsigned int Event_pend( ti_nonOS_Event_Handle eventHandle, unsigned int andMask, unsigned int orMask, unsigned int timeout )
{
    ti_nonOS_Event_Object *event_non_os = (ti_nonOS_Event_Object *)eventHandle;
    unsigned int matchingEvents;
    
    event_non_os->andMask = andMask;
    event_non_os->orMask  = orMask;
    
    /* check if events are already available */
    matchingEvents = Event_checkEvents(eventHandle, andMask, orMask);

    if (matchingEvents != 0)
    {


    }
    else
    {
        while ((matchingEvents == 0) && (timeout == EventP_WAIT_FOREVER))
        {
            /* check if events are already available */
            matchingEvents = Event_checkEvents(eventHandle, andMask, orMask);
        }
    }


    return (matchingEvents);/* return with matching bits */
}

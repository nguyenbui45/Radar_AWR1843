/*!
 *   \file   odsdemo_clusteringDBscan3D_priv.h
 *
 *   \brief   Private functions for DBscan3D clustering module.
 *
 *  Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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

#ifndef _ODSDEMO_DBSCAN3D_PRIV_H_
#define _ODSDEMO_DBSCAN3D_PRIV_H_

#include <math.h>
#include <swpform.h>
#include <odsdemo_clusteringDBscan3D.h>
#include <radar_c674x.h>
#ifdef _TMS320C6X
#include "c6x.h"
#endif

#define DBSCAN3D_FIXEDWEIGHTSHIFT (3)  //ASSUMPTION: weight is smaller than 8!!!!
#define DBSCAN3D_PIOVER180 (3.141592653589793/180.0)      //!< define the pi/180

typedef enum
{
    POINT_UNKNOWN = 0,                                                  /**< To be added */
    POINT_VISITED_ALREADY                                               /**< To be added */
} RADARDEMO_clusteringDBscan3DPointState;

extern uint16_t ODSDemo_clusteringDBscan3D_findNeighbors2Fixed(
                            IN ODSDemo_clusteringDBscan3DPoint3dfxdp *RESTRICT pointArray,
							IN int16_t *RESTRICT speedArray,
                            IN uint16_t point,
                            IN uint16_t *RESTRICT neigh,
                            IN uint16_t numPoints,
                            IN int32_t epsilon2,
                            IN float weight,
                            IN int32_t vFactor,
                            IN char *RESTRICT visited,
                            OUT uint16_t *newCount);

extern void ODSDemo_clusteringDBscan3D_calcInfoFixed(
                            IN ODSDemo_clusteringDBscan3DPoint3dfxdp *pointArray,
							IN int16_t *speedArray,
							//IN float  *SNRArray,
							//IN float  *aoaVar,
                            IN uint16_t *neighStart,
                            IN uint16_t *neighLast,
                            OUT ODSDemo_clusteringDBscanReport *report);


#endif //_ODSDEMO_DBSCAN3D_PRIV_H_


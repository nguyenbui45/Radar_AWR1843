/*!
 *  \file   odsdemo_clusteringDBscan3D.h
 *
 *  \brief   DBscan clustering module.
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
#ifndef _ODSDEMO_DBSCAN3D_H_
#define _ODSDEMO_DBSCAN3D_H_

#include <radar_c674x.h>
#include <swpform.h>
#include <math.h>
#include <swpform.h>
#ifdef _TMS320C6X
#include "c6x.h"
#endif
//#include <odsdemo_clusteringDBscan3D_priv.h>

//#define DBSCAN_ERROR_CODE_OFFSET 100

//!  \brief   error code for ODSDemo_clusteringDBscan.
//!
typedef enum
{
    DBSCAN_OK = 0,                                                  /**< To be added */
    DBSCAN_ERROR_CLUSTER_LIMIT_REACHED                              /**< To be added */
} ODSDemo_clusteringDBscanErrorCodes;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} ODSDemo_clusteringDBscan3DPoint3dfxdp;

//!  \brief   Structure element of the list of descriptors for ODSDemo_clusteringDBscan3D input.
//!
typedef struct
{
    uint16_t numPoints;                      /**< Number of point for clustering */
    ODSDemo_clusteringDBscan3DPoint3dfxdp *pointArray;    /**< gr location info for each detected point, points are store in x0, y0, x1, y1 fashion */
    int16_t *speed;
} ODSDemo_clusteringDBscanInput;

//!  \brief   Structure for each cluster information report .
//!
typedef struct
{
    uint16_t    numPoints;             /**< number of points in this cluster */
    int16_t     xCenter;               /**< the clustering center on x direction */
    int16_t     yCenter;               /**< the clustering center on y direction */
	int16_t     zCenter;               /**< the clustering center on z direction */
    int16_t     xSize;                 /**< the clustering size on x direction */
    int16_t     ySize;                 /**< the clustering size on y direction */
    int16_t     zSize;                 /**< the clustering size on z direction */
    int16_t     avgVel;                /**< the min velocity within this cluster */
	//float       centerRangeVar;        /**< variance of the range estimation */
	//float       centerAngleVar;        /**< variance of the angle estimation */
	//float       centerDopplerVar;      /**< variance of the doppler estimation */
} ODSDemo_clusteringDBscanReport;

//!  \brief   Structure of clustering output.
//!
typedef struct
{
    uint16_t *IndexArray;                       /**< Clustering result index array */
    uint16_t numCluster;                        /**< number of cluster detected */
    ODSDemo_clusteringDBscanReport *report;   /**< information report for each cluster*/
} ODSDemo_clusteringDBscanOutput;


/*!
   \fn     ODSDemo_clusteringDBscan3D

   \brief   Run ODSDemo_clusteringDBscan3D module.

   \param[in]    input
               Pointer to input data to ODSDemo_clusteringDBscan3D module.

   \param[out]    output
               Pointer to output data from ODSDemo_clusteringDBscan3D module.

   \ret     Error code.

   \pre       none

   \post      none


 */
 extern ODSDemo_clusteringDBscanErrorCodes ODSDemo_clusteringDBscan3D( ODSDemo_clusteringDBscanInput *input, ODSDemo_clusteringDBscanOutput *output);


#endif //_ODSDEMO_DBSCAN3D_H_

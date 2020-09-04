/*!
 *  \file   odsdemo_clusteringDBscan3D.c
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

#include <math.h>
#include <string.h>

/* C674x mathlib */
#include <ti/mathlib/mathlib.h>
#include "radar_c674x.h"
#include "./common/odsdemo_common.h"
#include "./common/mmw_config.h"
#include <float.h>
#include "dss_mmw.h"
#include "swpform.h"

#include <odsdemo_clusteringDBscan3D.h>
#include <odsdemo_clusteringDBscan3D_priv.h>


extern ODSDemo_DataPathObj odsdemo_dataPathObj;
extern MmwDemo_DSS_MCB     gMmwDssMCB;
ODSDemo_clusteringDBscanInput odsdemo_dbscanInput;
ODSDemo_clusteringDBscanOutput odsdemo_dbscanOutput;

//! \copydoc ODSDemo_clusteringDBscan3D
ODSDemo_clusteringDBscanErrorCodes ODSDemo_clusteringDBscan3D( ODSDemo_clusteringDBscanInput *input,
                                  ODSDemo_clusteringDBscanOutput *output)
{

    char     *visited, *scope, *scratchPadPtr;
    uint16_t *neighLast,*neighCurrent, *neighbors;
    uint16_t neighCount;
    uint16_t newCount;
    uint16_t point, member;
    uint16_t numPoints;
    uint16_t clusterId;
    uint16_t ind;
    int32_t epsilon2fixed;
    MmwDemo_ClusteringCfg *cfg = &gMmwDssMCB.cliCfg[0].dbscanCfg;

    //total scratch memory Used:  ODSDEMO_DBSCAN_MAXINPUTPOINTS *(sizeof(char) *2 + sizeof(uint16_t));
    scratchPadPtr = (char *)&odsdemo_dataPathObj.scratchPad[0];
	visited = (char *)&scratchPadPtr[0];
	scope = (char *)&scratchPadPtr[ODSDEMO_DBSCAN_MAXINPUTPOINTS];
    neighbors  =  (uint16_t *)&scratchPadPtr[(ODSDEMO_DBSCAN_MAXINPUTPOINTS << 1)];
	
    numPoints	=	input->numPoints;
    clusterId	=	0;

    epsilon2fixed   =   (int32_t) (cfg->epsilon*cfg->epsilon * (float)(cfg->fixedPointScale * cfg->fixedPointScale));
    memset(visited, POINT_UNKNOWN, numPoints*sizeof(char));

    // scan through all the points to find its neighbors
    for(point = 0; point < numPoints; point++)
    {
        if(visited[point] != POINT_VISITED_ALREADY)
        {
            visited[point] = POINT_VISITED_ALREADY;

            neighCurrent = neighLast = neighbors;
            // scope is the local copy of visit
            memcpy(scope, visited, numPoints*sizeof(char));

            neighCount = ODSDemo_clusteringDBscan3D_findNeighbors2Fixed(
					(ODSDemo_clusteringDBscan3DPoint3dfxdp *)input->pointArray,
					input->speed,
					point, 
					neighLast, 
					numPoints, 
					epsilon2fixed, 
					cfg->weight,
					(int32_t)(cfg->vFactor * cfg->fixedPointScale),
					scope, 
					&newCount);
                // ZY: optimized function ODSDemo_clusteringDBscan_findNeighbors3Fixed need to rework
                //neighCount = ODSDemo_clusteringDBscan_findNeighbors3Fixed((ODSDemo_clusteringDBscanPoint2dfxdp *)input->pointArray, point, neighLast, numPoints, epsilon2fixed, cfg.scope, &newCount);

            if(neighCount < cfg->minPointsInCluster)
            {
                // This point is Noise
                output->IndexArray[point] = 0;
            }
            else
            {
                // This point belongs to a New Cluster
                clusterId++;                                // New cluster ID
                output->IndexArray[point] = clusterId;      // This point belong to this cluster

                // tag all the neighbors as visited in scope so that it will not be found again when searching neighbor's neighbor.
                for (ind = 0; ind < newCount; ind ++) {
                    member = neighLast[ind];
                    scope[member] = POINT_VISITED_ALREADY;
                }
                neighLast += newCount;

                while (neighCurrent != neighLast)               // neigh shall be at least minPoints in front of neighborhood pointer
                {
                    // Explore the neighborhood
                    member = *neighCurrent++;               // Take point from the neighborhood
                    output->IndexArray[member] = clusterId; // All points from the neighborhood also belong to this cluster
                    visited[member] = POINT_VISITED_ALREADY;

                    neighCount = ODSDemo_clusteringDBscan3D_findNeighbors2Fixed(
							(ODSDemo_clusteringDBscan3DPoint3dfxdp *)input->pointArray,
							input->speed, 
							member, 
							neighLast, 
							numPoints, 
							epsilon2fixed, 
							cfg->weight,
							(int32_t)(cfg->vFactor * cfg->fixedPointScale),
							scope, 
							&newCount);
                        // ZY: optimized function ODSDemo_clusteringDBscan_findNeighbors3Fixed need to rework
                        //neighCount = ODSDemo_clusteringDBscan_findNeighbors3Fixed((ODSDemo_clusteringDBscanPoint2dfxdp *)input->pointArray, member, neighLast, numPoints, epsilon2fixed, cfg.scope, &newCount);

                    if(neighCount >= cfg->minPointsInCluster)
                    {
                        for (ind = 0; ind < newCount; ind ++) {
                            member = neighLast[ind];
                            scope[member] = POINT_VISITED_ALREADY;
                        }
                        neighLast += newCount;              // Member is a core point, and its neighborhood is added to the cluster
                    }
                }
            	if (clusterId >= ODSDEMO_DBSCAN_MAXNUMCLUSTER)
            	   return DBSCAN_ERROR_CLUSTER_LIMIT_REACHED;

                // calculate the clustering center and edge information
            	ODSDemo_clusteringDBscan3D_calcInfoFixed(
						(ODSDemo_clusteringDBscan3DPoint3dfxdp *)input->pointArray,
						input->speed,
						neighbors, 
						neighLast, 
						(ODSDemo_clusteringDBscanReport *)&output->report[clusterId - 1]);
            }
        }
    } // for
    output->numCluster = clusterId;

    return DBSCAN_OK;
}



/**
 *  \file   odsdemo_heatmap.h
 *
 *  Copyright (c) Texas Instruments Incorporated 2018
 *
 *  Use of this software is controlled by the terms and conditions found in the
 *  license agreement under which this software has been supplied or provided.
 *
 */
#ifndef ODSDEMO_HEATMAP
#define ODSDEMO_HEATMAP

#include "dss_data_path.h"

extern void ODSDemo_Heatmap_aoaEstCaponBF(ODSDemo_DataPathObj *obj, float *heatmap);

extern void ODSDemo_Heatmap_aoaEstCaponBF_heatmap(uint8_t  bfFlag,
                                                  int32_t  nRxAnt,
                                                  int32_t  numAzimuthBins,
                                                  cplxf_t *steeringVec,
                                                  cplxf_t *invRnMatrices,
                                                  float   *rangeAzimuthHeatMap);


extern void ODSDemo_Heatmap_aoaEstCaponBF_covInv(uint8_t   invFlag,
                                                 uint8_t   clutterRmFlag,
                                                 float     gamma,
                                                 int32_t   nRxAnt,
                                                 int32_t   nChirps,
                                                 int32_t  *scratch,
                                                 cplx16_t *inputAntSamples,
                                                 cplxf_t  *invRnMatrices);

extern void ODSDemo_Heatmap_steeringVecGen(ODSDemo_DataPathObj *obj);

#endif //ODSDEMO_HEATMAP

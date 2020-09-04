/**
 *  \file   oddemo_heatmap.h
 *
 *  Copyright (c) Texas Instruments Incorporated 2018
 *
 *  Use of this software is controlled by the terms and conditions found in the
 *  license agreement under which this software has been supplied or provided.
 *
 */
#ifndef ODDEMO_HEATMAP
#define ODDEMO_HEATMAP

#include "dss_data_path.h"


#define ODDEMO_HEATMAP_MAX 32000

typedef struct
{
    float min;
    float max;
    float avg;
} ODDemo_Heatmap_Stats;


extern void ODDemo_Heatmap_aoaEstCaponBF(ODDemo_DataPathObj *obj);

extern void ODDemo_Heatmap_aoaEstCaponBF_heatmap(uint8_t  bfFlag,
                                                 int32_t  nRxAnt,
                                                 int32_t  numAzimuthBins,
                                                 cplxf_t *steeringVec,
                                                 cplxf_t *invRnMatrices,
                                                 float   *rangeAzimuthHeatMap);


extern void ODDemo_Heatmap_aoaEstCaponBF_covInv(uint8_t   invFlag,
                                                uint8_t   clutterRmFlag,
                                                float     gamma,
                                                int32_t   nRxAnt,
                                                int32_t   nChirps,
                                                int32_t   log2Chirps,
                                                int32_t  *scratch,
                                                cplx16_t *inputAntSamples,
                                                cplxf_t  *invRnMatrices);

extern void ODDemo_Heatmap_steeringVecGen(ODDemo_DataPathObj *obj);

extern void ODDemo_Heatmap_arc_removal(float *heatmap);

extern void ODDemo_Heatmap_scale_heatmap(float *heatin, uint16_t *heatout);

extern void ODDemo_Heatmap_smoothing(float *current_heatmap, float *previous_heatmap);

#endif //ODDEMO_HEATMAP

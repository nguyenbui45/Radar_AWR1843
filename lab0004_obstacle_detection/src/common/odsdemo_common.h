/**
 *  \file   odsdemo_common.h
 *
 *  \brief  Global constant and structure definitions
 *
 *  Copyright (c) Texas Instruments Incorporated 2018
 *
 *  Use of this software is controlled by the terms and conditions found in the
 *  license agreement under which this software has been supplied or provided.
 *
 */
#ifndef ODSDEMO_COMMON
#define ODSDEMO_COMMON

//#include "swpform.h"
#ifdef _TMS320C6X
#include "c6x.h"
#endif

//#define ODSDEMO_MAX_RANGE_CM           200.0 /* 2 meters in cm */
#define ODSDEMO_MAX_ANGLE_RANGE        90.0  /* half of field of view in degrees */
#define ODSDEMO_RANGE_RES_CM           4.327 /* range resolution in cm */
#define ODSDEMO_RANGE_RES             (ODSDEMO_RANGE_RES_CM / 100.0) /* range res in m */
#define ODSDEMO_ANGLE_RES              3.0   /* angle resolution in degrees */

#define ODSDEMO_MAX_RANGE_BINS             256 //((uint16_t)(ODSDEMO_MAX_RANGE_CM / ODSDEMO_RANGE_RES_CM))
#define ODSDEMO_AZIMUTH_BINS          ((uint16_t)(2 * ODSDEMO_MAX_ANGLE_RANGE / ODSDEMO_ANGLE_RES))
#define ODSDEMO_ELEVATION_BINS        ((uint16_t)(2 * ODSDEMO_MAX_ANGLE_RANGE / ODSDEMO_ANGLE_RES))
#define ODSDEMO_MAX_FFT2D_SIZE           64    /* chirps per frame */
#define ODSDEMO_MAX_EXPANSION_SIZE       16    /* cFar based */
#define ODSDEMO_MAX_RX_ANT             8
#define ODSDEMO_PHY_RX_ANT             4
#define ODSDEMO_NUM_ADOA_ANT           4
#define ODSDEMO_NUM_EDOA_ANT           4
#define ODSDEMO_MAX_OBJECTS            50
#define ODSDEMO_DOA2D_FFTSIZE          64
#define ODSDEMO_DOA2D_HALFFFTSIZE      (ODSDEMO_DOA2D_FFTSIZE >> 1)
#define ODSDEMO_DOA2D_HALFFFTSIZEINV   2.0f/(float)(ODSDEMO_DOA2D_FFTSIZE)

//#define ODSDEMO_RANGE_AZ_HEATMAP_BUF_SIZE  (ODSDEMO_MAX_RANGE_BINS * ODSDEMO_AZIMUTH_BINS)
//#define ODSDEMO_RANGE_EL_HEATMAP_BUF_SIZE  (ODSDEMO_MAX_RANGE_BINS * ODSDEMO_ELEVATION_BINS)


#define ODSDEMO_DBSCAN_MAXACCFRAME  10
#define ODSDEMO_DBSCAN_MAXINPUTPOINTS (ODSDEMO_MAX_OBJECTS << 2)
#define ODSDEMO_DBSCAN_MAXNUMCLUSTER  20


// SteeringVec Size (bytes): (nRxAnt-1) * steeringVecSize * sizeof(cplxf_t); For Ant0, since steering vector is = 1, it is not saved.
// steeringVecSize =   (uint32_t) ((2.f * estAngleRange) / estAngleResolution) + 0.5f);
// For estAngleRange = 60 (-60, 60) and 64 nAzimuthBins (angles), the resolution = 120/64 = 1.875
// steeringVecSize = 64

#define ODSDEMO_STEERINGVEC_L1_BUF_SIZE (ODSDEMO_AZIMUTH_BINS * ODSDEMO_MAX_RX_ANT)
#define ODSDEMO_SCRATCH_L1_BUF_SIZE      360*4 // for nRxAnt=8 for uint32_t

// size = [(nRxAnt * ( 1 + nRxAnt))/2] * sizeof(cplxf_t)
// upper triangle of nRxAnt x nRxAnt Hermitian matrix
#define ODSDEMO_INVRNMATRIX_BUF_SIZE     36  // for nRxAnt=8 for cplxf_t


//This type defines the demo's configurable parameters.
typedef struct
{
    float    gamma;
    uint32_t windowLen;
} ODSDEMO_Parms;

#endif //ODSDEMO_COMMON

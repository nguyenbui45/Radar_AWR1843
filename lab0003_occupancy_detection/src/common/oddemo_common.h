/**
 *  \file   oddemo_common.h
 *
 *  \brief  Global constant and structure definitions
 *
 *  Copyright (c) Texas Instruments Incorporated 2018
 *
 *  Use of this software is controlled by the terms and conditions found in the
 *  license agreement under which this software has been supplied or provided.
 *
 */
#ifndef ODDEMO_COMMON
#define ODDEMO_COMMON

//#include "swpform.h"
#ifdef _TMS320C6X
#include "c6x.h"
#endif

#define ODDEMO_MAX_RANGE               64
#define ODDEMO_MAX_AZIMUTH             48
#define ODDEMO_MAX_ZONES               8
#define ODDEMO_ZONE_PAIR              (ODDEMO_MAX_ZONES / 2)
#define ODDEMO_MAX_FRAME_HIST          16
#define ODDEMO_MATRIX_ROW_SIZE         6
#define ODDEMO_MATRIX_NUM_ROWS         4
#define ODDEMO_MATRIX_SIZE            (ODDEMO_MATRIX_ROW_SIZE * ODDEMO_MATRIX_NUM_ROWS)
#define ODDEMO_ANGLE_RANGE             60.0
#define ODDEMO_ANGLE_RESOLUTION       ((ODDEMO_ANGLE_RANGE * 2) / (float)ODDEMO_MAX_AZIMUTH)
#define ODDEMO_ROW_NOISE_FRAMES        64

#define ODDEMO_ANGLEHEATMAP_BUF_SIZE   (ODDEMO_MAX_RANGE * ODDEMO_MAX_AZIMUTH)

// SteeringVec Size (bytes): (nRxAnt-1) * steeringVecSize * sizeof(cplxf_t); For Ant0, since steering vector is = 1, it is not saved.
// steeringVecSize =   (uint32_t) ((2.f * estAngleRange) / estAngleResolution) + 0.5f);
// For estAngleRange = 60 (-60, 60) and 64 nAzimuthBins (angles), the resolution = 120/64 = 1.875
// steeringVecSize = 64

#define ODDEMO_STEERINGVEC_L1_BUF_SIZE (ODDEMO_MAX_AZIMUTH * 8)
#define ODDEMO_SCRATCH_L1_BUF_SIZE      360 // for nRxAnt=8 for uint32_t

// size = [(nRxAnt * ( 1 + nRxAnt))/2] * sizeof(cplxf_t)
// upper triangle of nRxAnt x nRxAnt Hermitian matrix
#define ODDEMO_INVRNMATRIX_BUF_SIZE     36  // for nRxAnt=8 for cplxf_t


//This type defines the position and size of an occupancy zone in the heat map.
typedef struct
{
  uint16_t range_start;
  uint16_t range_length;
  uint16_t range_max_idx;
  uint16_t azimuth_start;
  uint16_t azimuth_width;
  uint16_t azimuth_max_idx;
} ODDEMO_Zone;

//This type defines the feature vector contents for a pair of zones.
typedef struct
{
  float powerMA[2];
  float powRatio[2];
  float crossCorr;
} ODDEMO_Feature;

//This type defines the decision vector contents for a zone.
typedef struct
{
  float posPercent;     //percentage of positive detections during the window period
  float avgPower;       //windowed average zone power
  uint16_t range_idx;   //heatmap range idx for the center of the max 5x5
  uint16_t azimuth_idx; //heatmap azimuth idx for the center of the max 5x5
} ODDEMO_Decision;

//This type defines the demo's configurable parameters.
typedef struct
{
    uint16_t mode;      //1 = operational, 2 = zone tuning, 3 = coefficient data
    uint16_t windowLen; //window length for feature extract - multiple of 4
    float    gamma;
    float    threshold; //percentage of positive windowed detections to declare occupied
    uint16_t threshWin; //frames to average detections over
    float    smoothing; //0.0 disables smoothing. max should not exceed 0.95.
} ODDEMO_Parms;

#endif //ODDEMO_COMMON

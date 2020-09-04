/**
 *  \file   odsdemo_cfar.h
 *
 *  Copyright (c) Texas Instruments Incorporated 2018
 *
 *  Use of this software is controlled by the terms and conditions found in the
 *  license agreement under which this software has been supplied or provided.
 *
 */
#ifndef ODSDEMO_CFAR
#define ODSDEMO_CFAR

#include "./common/odsdemo_common.h"

#define CFAR_WIN_SIZE                             (ODSDEMO_MAX_EXPANSION_SIZE*2 + 1)
#define ODSDEMO_DETECTIONCFAR_TYPE_NOT_SUPPORTED   0
#define ODSDEMO_DETECTIONCFAR_UNION                1
#define ODSDEMO_DETECTIONCFAR_INTERSECTION         2

#define ODSDEMO_DETECTIONCFAR_NOISECALC_MEAN       0
#define ODSDEMO_DETECTIONCFAR_NOISECALC_LEFTORMEAN 1
#define ODSDEMO_DETECTIONCFAR_NOISECALC_MIN        2


typedef enum
{
    CFAR_OK = 0,                                                  /**< To be added */
    CFAR_ERROR_MAX_DETECTION_EXCEEDED                              /**< To be added */
} ODSDemo_cfarErrorCodes;


typedef struct
{
    uint16_t numObjDetected;       /**< number of objectes detected*/
    uint16_t rangeInd[ODSDEMO_MAX_OBJECTS];   /**< range index pointer*/
    //uint16_t dopplerInd[ODSDEMO_MAX_OBJECTS]; /**< Doppler index pointer*/
    float    rangeEst[ODSDEMO_MAX_OBJECTS];   /**< range estimation pointer*/
    //float    dopplerEst[ODSDEMO_MAX_OBJECTS]; /**< Doppler estimation pointer*/
    float    snrEst[ODSDEMO_MAX_OBJECTS];     /**< linear snr estimation pointer*/
    float    noise[ODSDEMO_MAX_OBJECTS];      /**< Total noise estimation*/
    uint16_t aInd[ODSDEMO_MAX_OBJECTS];   /**< angle index pointer*/
    uint16_t eInd[ODSDEMO_MAX_OBJECTS];   /**< angle index pointer*/
    //float       *rangeVar;           /**< Variance for range estimation*/
    //float       *dopplerVar;         /**< Variance for Doppler estimation*/
} ODSDEMO_detectionCFAR_output;


ODSDemo_cfarErrorCodes ODSDemo_detectionCFAR(float    *aHeatmap,
                           float    *eHeatmap,
                           float    *zeroDoppler,
                           uint16_t *aHeatmapMaxIndex,
                           uint16_t *eHeatmapMaxIndex,
                           ODSDEMO_detectionCFAR_output *cfarOut);

void ODSDemo_detectionCFAR_cfar1D(float    *heatmap,
                                  uint16_t  heapmap_width,
                                  uint16_t *heatmapMaxIndex,
                                  float    *zeroDoppler,
                                  MmwDemo_CfarCfg *cfarCfg,
                                  float    *sigPwr,
                                  float    *noisePwr,
                                  bool     *peakFlag);

#endif //ODSDEMO_CFAR

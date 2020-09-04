/*! 
 *  \file   detection.h
 *
 *  \brief   CFAR functions.
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

#ifndef _RADARDEMO_DETECTION_H
#define _RADARDEMO_DETECTION_H

#include <swpform.h>
#ifdef _WIN32
#include "C6xSimulator.h"
#include "C6xSimulator_type_modifiers.h"
#endif
#include <string.h>

#define MAX_NUM_ANTENNA 4

/**
 *  \def  DETECTION_MAX_NUM_OBJ
 *
 *  \brief   Maximum detected objects number from detection module.
 *
 *  \sa
 */
#define DETECTION_MAX_NUM_OBJ 100
/**
 *  \def  DETECTION_MAX_NUM_OBJ_RANGE
 *
 *  \brief   Maximum detected objects number in the first range dimension.
 *
 *  \sa
 */
#define DETECTION_MAX_NUM_OBJ_RANGE 600

/**
 *  \def  MAX_NUM_RANGE_LEN
 *
 *  \brief   Maximum number of range bins in one chirp.
 *
 *  \sa
 */
#define MAX_NUM_RANGE_LEN 1024
/**
 *  \def  MAX_NUM_DOPPLER_LEN
 *
 *  \brief   Maximum number of Doppler dimension length.
 *
 *  \sa
 */
#define MAX_NUM_DOPPLER_LEN 256

/**
 *  \def  C_IQMAG_THRESH
 *
 *  \brief   For signal power check, if lower than this threshold, no detection.
 *
 *  \sa
 */
#define C_IQMAG_THRESH					(1)


/**
 * \brief  Detection module parameters
 *
 *  \sa
 *
 */


typedef struct _detectionPara_
{

	uint16_t startRangeCell; /**< start index where range bins are kept, detected range index should be detectValue + startRangeCell. */
	uint16_t  rangeNumBins; /**< number of bins in range direction. */
	uint16_t  DopplerNumBins; /**< number of bins in Doppler direction. */
	uint16_t  refWinSize[2]; /**< reference window size in each side in two directions for clutter variance estimation. */
	uint16_t  guardWinSize[2]; /**< guard window size in each side in two directions. */
	float  thre; /**< threshold used for compare. */
	uint16_t  threWinSize; /**< window size for the second step threshold detection within neighborhood of the first detected point. */
	float  threRatio; /**< ration for neighborhood detection, thre = threRatio*peak. */
	float rangeRes; /**< range resolution in meters. */
	float velocityRes; /**< velocity resolution in meters/sec. */
	uint16_t detMethodID; /**< detection method ID, for future use for different type of detection method. */
	uint16_t  numAntenna; /**< number of antennas. */

}detectionParaConfig_t;

typedef enum
{
	DETECTION_OK = 0,
	DETECTION_ERROR_MEMORY_ALLOC_FAILED,
	DETECTION_ERROR_NOT_SUPPORTED
} DetectionErrorCodes;



typedef struct _activeBin_
{
	uint16_t rangeIndex;
	uint16_t DopplerIndex;
	//uint32_t noiseVar[MAX_NUM_ANTENNA]; /**< noise pwoer for each antenna. */
	float noisePower; /**< summed noise power. */
	//cplx16_t binVal[MAX_NUM_ANTENNA]; /**< measurements for 4 antennas. */
	cplxf_t binVal[MAX_NUM_ANTENNA]; /**< measurements for 4 antennas in float. */
	float range;  /**< range estimation. */
	float velocity; /**< velocity estimation. */
	float rangeVar; /**< range variance. */
	float velocityVar; /**< velocity variance. */

}activeBin_t;


/**
 *  \struct   _detectedResults_
 *   {
 *
 *    	uint16_t numObjects; //number of detected bins
 *
 *	   	uint16_t index;//for future use, make sure the data structure is 32 bits aligned
 *
 *	   	activeBin_t activebins[DETECTION_MAX_NUM_OBJ]; //detected range-doppler bin measurements
 *
 *   } detectedResults_t;
 *
 *  \brief   Detection result output
 *
 *  \sa
 *
 */

typedef struct _detectedResults_
{

	uint16_t numObjects; /**< number of detected bins. */

	uint16_t index; /**< for future use, make sure the data structure is 32 bits aligned. */

	activeBin_t activebins[DETECTION_MAX_NUM_OBJ]; /**< detected range-doppler bin measurements. */

}detectedResults_t;

/**
 *  @brief     Initiliaze the detection module with the given parameters
 *
 *
 *  @param[in]
 *
 *
 *  @remarks
 */
void *detectTargetsCreate(detectionParaConfig_t *detectionParams,
		                  void                  *pDataIntegrate);


/**
 *  @brief      Run detection algorithm.
 *
 *  @param[in]  handle               Pointer to instance handler
 *  @param[in]  pDataIn              Pointer to the input Data
 *  @param[out] pDataOut             Pointer to the output Data
 *
 *  @remarks
 */
int32_t detectTargetsRun(void *handle, cplxf_t *restrict pDataIn, detectedResults_t *restrict pDataOut);



/**
 *  @brief      Performs CFAR detection first along Doppler lines followed by range line
 *              according to the first range line detection result. Only the cross points from both dimension detection
 *              is used for final detection results.
 *
 *
 *  @param[in]  handle               Pointer to instance handler
 *  @param[in]  pDataIn              Pointer to the input Data
 *  @param[out] pDetectedTargets     Pointer to the array which stores the range&Doppler index for detected objs
 *
 *  @remarks
 */
int32_t detectTargetsRangeDopplerCross(void *handle, cplxf_t *restrict pDataIn, detectedResults_t *restrict pDetectedTargets);


/**
 *  @brief      Delete the resources of detection instance.
 *
 *  @param[in]  handle      Pointer to instance handler.
 *
 */
int32_t detectTargetsDelete(void *handle);

/**
 *  @brief      Return the debug data size.
 *
 *  @param[in]  handle      Pointer to instance handler.
 *
 */
int32_t detectTargetsDebugQuery(void *handle);


#endif

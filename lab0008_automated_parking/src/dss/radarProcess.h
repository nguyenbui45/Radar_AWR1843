/*!
 *  \file   radarProcess.h
 *
 *  \brief   Header file for radar signal processing chain.
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
#ifndef _RADARPROCESS_H
#define _RADARPROCESS_H

#define TMDEMOV1

#include <swpform.h>
#ifdef TMDEMOV1
#include <modules/clustering/dbscan3D/api/RADARDEMO_clusteringDBscan3D.h>
//#include <modules/tracking/clusterTracker/api/RADARDEMO_clusterTracker.h>
#endif

#if defined(_WIN32) || defined(CCS)
#include <stdio.h>
#endif

//#define TMDEMOV1

/**
 *  \def  M_PI
 *
 *  \brief   constant value used for pi.
 *
 *  \sa
 */
#define  M_PI 3.14159265358979323846f
//#define  RADARPROC_PIOVER180 (3.141592653589793/180.0)

#define  NUM_RADAR_SENSORS      1    // 1 radar sensor -> a maximum number of 256
#define  NUM_RADAR_TXANT        2    // 2 transmitting antennas
#define  NUM_RADAR_RXANT        4    // 4 receiving antennas

#define  DOA_OUTPUT_MAXPOINTS   200
#define  MAX_RESOLVED_OBJECTS_PER_FRAME   DOA_OUTPUT_MAXPOINTS
#define  CLUSTERING_MAX_CLUSTERS   50
#define  CLUSTERING_MAX_NUMPOINTS   2048
#define  MAX_NUM_RANGE_BINS (2048)

#define RADAR_DSP_BUFF_CONT_MAX_NUM_BUFF  2U

typedef enum
{
      LOW_RESOLUTION    = 0,
      HIGH_RESOLUTION   = 1,
      DUAL_RESOLUTION   = 2,
      TEST_TONE         = 3
} ResolutionMode_e;

typedef enum
{
    /** Detection Object Data */
    RadarDsp_outputDataType_OBJ_DATA = 0,

    /** Range-Doppler Heap Map. */
    RadarDsp_outputDataType_HEAT_MAP = 1,

    /** Marker for validation. */
    RadarDsp_outputDataType_MAX

} RadarDsp_outputDataType;

typedef struct
{
    /** Type of the data carried in the buffer. */
    RadarDsp_outputDataType     type;

    /** Data buffer pointer. */
    void                      * buff;

} RadarDsp_outputBuffType;

typedef struct
{
    /** Number of valid buffers in the array. */
    uint16_t                numBuff;

    /** Output buffers. */
    RadarDsp_outputBuffType elem[RADAR_DSP_BUFF_CONT_MAX_NUM_BUFF];

} RadarDsp_outputBuffCntxt;

#if 0// TMDEMOV1
// radar process output data structure
typedef struct {
	int16_t object_count;                         //number of objects (points)

#ifdef CCS
	int16_t          location[DOA_OUTPUT_MAXPOINTS][3];               //[0]->x, [1]->y, [2]->z // (in steps of 0.01m)
	int16_t          velocity[DOA_OUTPUT_MAXPOINTS][3];               //[0]->x, [1]->y, [2]->z // (in steps of 0.05m/s)
#else
	int16_t          location[DOA_OUTPUT_MAXPOINTS][2];               //[0]->x, [1]->y, [2]->z // (in steps of 0.01m)
	int16_t          velocity[DOA_OUTPUT_MAXPOINTS][2];               //[0]->x, [1]->y, [2]->z // (in steps of 0.05m/s)
#endif
	uint16_t  clusterOutFlag;
    uint16_t numCluster;                        /**< number of cluster detected */
    RADARDEMO_clusteringDBscanReport report[CLUSTERING_MAX_CLUSTERS];   /**< information report for each cluster*/
    uint16_t trackingOutFlag;
    uint16_t numTracks;                        /**< number of tracks detected */
    RADARDEMO_trackerOutput_dataType trackingInfo[RADARDEMO_CT_MAX_NUM_TRACKER];  /**< information report for each track*/
} radarProcessOutput_NetworkLink_t;
#endif

// Output data to Tracker
typedef struct {
    int32_t object_count;                         //number of objects (points)

    float          range[DOA_OUTPUT_MAXPOINTS];
    float          angle[DOA_OUTPUT_MAXPOINTS];
    float          eAngle[DOA_OUTPUT_MAXPOINTS];
    float          doppler[DOA_OUTPUT_MAXPOINTS];
    float          snr[DOA_OUTPUT_MAXPOINTS];
    float          angleVar[DOA_OUTPUT_MAXPOINTS];
    uint16_t       dopplerIdx[DOA_OUTPUT_MAXPOINTS];
    uint16_t       rangeIdx[DOA_OUTPUT_MAXPOINTS];
    //float          dopplerVar[DOA_OUTPUT_MAXPOINTS];
#ifdef TMDEMOV1
    uint16_t  clusterOutFlag;
    uint16_t numCluster;                        /**< number of cluster detected */
    RADARDEMO_clusteringDBscanReport dbscanReport[CLUSTERING_MAX_CLUSTERS];   /**< information report for each cluster*/
#endif
} radarProcessOutputToTracker;


// Zero doppler power profile
typedef struct {
    int32_t numDopplerBins;                         //number of objects (points)
    float zeroDopplerPowerProfile[MAX_NUM_RANGE_BINS];
} radarProcessZeroDopplerInfo;


typedef struct _radarProcessBenchmarkElem_
{
	uint32_t numDetPnts;
	uint32_t numInputPntsToDbscan;
	uint32_t numClustersToTracker;
	uint32_t numActiveTracks;
	uint32_t cfarDetectionCycles;
	uint32_t aoaCycles;
	uint32_t dataConvCycles;
	uint32_t dbscanCycles;
	uint32_t trackingCycles;
} radarProcessBenchmarkElem;


typedef struct _radarProcessBenchmarkObj_
{
	uint32_t bufferLen;
	uint32_t bufferIdx;
	radarProcessBenchmarkElem *buffer;
} radarProcessBenchmarkObj;


typedef struct _radarModuleCfarConfig_
{
	uint16_t	refWinSize[2]; /**< reference window size in each side in two directions for clutter variance estimation. */
	uint16_t	guardWinSize[2]; /**< guard window size in each side in two directions. */
	uint16_t	cfarDiscardLeft;  /**< Number of left cells discarded.*/
	uint16_t	cfarDiscardRight;  /**< Number of left cells discarded.*/
	uint16_t	cfarMethod; /**< CFAR_CA (2-pass) = 1; CFAR_OS = 2 (2-pass); CFAR_CASO = 3 (1-pass); CFAR_CASO = 4 (2-pass); */
	uint16_t    clRemoval;
	uint32_t    log2MagFlag; /**<use log2(mag) as input*/
	float		thre; /**< threshold used for compare. */
	float        dopplerSearchRelThr;  	        /**< Doppler search relative threshold.*/
	float		rangeRes; /**< range resolution in meters. */
	float		velocityRes; /**< velocity resolution in meters/sec. */
} radarModuleCfarConfig;

typedef struct _radarModuleDoaConfig_
{
	uint16_t	doaSideLobeLevel_dB;  /**< doaSideLobeLevel_dB for DoA_MPBF.*/

	//DoA parameters
	uint16_t	doaMethod;  /**< DOA method, 3 for MPBF, 4 for DML, 0 for SPBF.*/
	uint8_t     * antSpacing;  			/**< antenna spacing in unit of lambda/2, size of nRxAnt, only support uniform linear array*/
	uint8_t     vmaxUnrollFlag;         /**<flag for angle correction for Vmax unrolling */
	float	doaSearchRange;  /**< DoA search range, from -doaSearchRange to +doaSearchRange.*/
	float	doaSearchRes;  /**< DoA Resolution.*/
	float		doaGamma;  /**< gama for DoA_MPBF.*/
	float       doaVarThr;         /**< Minimum variance range (in degree) for the output estimation. Estimates with bigger confidence range will not be reported.*/
} radarModuleDoaConfig;

typedef struct radarModuleDBScanConfig_
{
    float       epsilon;              /**< clustering dbscan threshold*/
    float       weight;               /**< the weight between the distance and speed */
    float       vFactor;               /**< additional velocity factor for speed delta  */
    uint16_t    nAccFrames;           /**< number of frames to accumulate */
    uint16_t    minPointsInCluster;   /**< minimum number of points in a cluster*/
    uint16_t    maxPoints;            /**< Maximum number of points that can be services per dbscanRun call */
    uint16_t    fixedPointScale;     /**< Block scale value to convert x-y from floating-point to fixed-point.
                                          Should be the same the one used in converting r-theta to x-y, not used for floating-point input */
    uint16_t    xyzOutputQFormat;
} radarModuleDBScanConfig;



//user input configuration parameters
typedef struct _radarModuleConfig_
{
	//rangeFFT/Doppler parameters
	uint16_t	framePeriod; /**< Frame period in msec. */
	uint16_t	numAdcSamplePerChirp; /**< number of adc samples per chirp. */
	uint16_t	numAdcBitsPerSample; /**< number of adc bits per sample. */
	uint16_t	numChirpPerFrame; /**< number of chirps per frame. */
	uint16_t	numTxAntenna; /**< number of antennas. */
	uint16_t	numAntenna; /**< number of virtual antennas. */
	uint16_t    numAziAntenna; /**< number of antenna can be used for azimuth calculation.*/
	uint16_t	numPhyRxAntenna; /**< number of physical RX antennas. */
	//uint16_t	rangeWinSize; /**< range window size. */
	uint16_t	DopplerWinSize; /**< Doppler window size. */
	uint16_t    mimoModeFlag;    /**<Flag for MIMO mode: 0 -- SIMO, 1 -- TDM MIMO, 2 -- FDM or BF*/
	uint32_t    numTotalChirpProfile;    /**<number of chirp profiles*/
	uint32_t    numUniqueChirpProfile;    /**<number of unique chirp profiles*/
	//float		rangeWindow[16]; /**< pointer to pre-calcuted window coeff for range FFT. */
	float		dopplerWindow[4];/**< pointer to pre-calcuted window coeff for Doppler FFT. */
	//float       chirpInterval;
	//float       frequencySlopeMHzMicoSec;
	//float       adcSamplePeriodMicoSec;
	//float       bandWidth;
	//float       centerFrequency;

	float		antPhaseCompCoeff[24]; /**< pointer to pre-calcuted window coeff for range FFT. */

	//detection parameters
	radarModuleCfarConfig cfarConfig;

	radarModuleDoaConfig doaConfig;
#ifdef TMDEMOV1
	radarModuleDBScanConfig dbscanConfig;
#endif

	uint16_t	fftSize1D;
	uint16_t	fftSize2D;
	cplx16_t    * pFFT1DBuffer;
	uint16_t	maxNumDetObj;
	uint32_t    heatMapMemSize;
	float       *heatMapMem;
	radarProcessBenchmarkObj * benchmarkPtr;
}radarProcessConfig_t;
typedef enum
{
	PROCESS_OK = 0,
	PROCESS_ERROR_INIT_MEMALLOC_FAILED,
	PROCESS_ERROR_RANGEPROC_INIT_FAILED,
	PROCESS_ERROR_RANGEPROC_INOUTALLOC_FAILED,
	PROCESS_ERROR_DOPPLERPROC_INIT_FAILED,
	PROCESS_ERROR_DOPPLERPROC_INOUTALLOC_FAILED,
	PROCESS_ERROR_CFARPROC_NONSUPPORTEDMETHOD,
	PROCESS_ERROR_CFARPROC_INIT_FAILED,
	PROCESS_ERROR_CFARPROC_INOUTALLOC_FAILED,
	PROCESS_ERROR_DOAPROC_NONSUPPORTEDMETHOD,
	PROCESS_ERROR_DOAPROC_INIT_FAILED,
	PROCESS_ERROR_DOAPROC_INOUTALLOC_FAILED,
	PROCESS_ERROR_DBSCAN_INIT_FAILED,
	PROCESS_ERROR_DBSCAN_INOUTALLOC_FAILED,
	PROCESS_ERROR_TRACKING_INIT_FAILED,
	PROCESS_ERROR_TRACKING_INOUTALLOC_FAILED,
	PROCESS_ERROR_NOT_SUPPORTED
} ProcessErrorCodes;


//! \brief      initializes data processing task
//!
void *radarProcessCreate(radarProcessConfig_t  * config, ProcessErrorCodes * procErrorCode);

/**
 *  @brief      Perform range processing. Processing is done per antenna per chirp.
 *
 *  @param[in]  handle               Pointer to instance handler
 *  @param[in]  pDataIn              Pointer to the input Data
 *  @param[out] pDataOut             Pointer to the output Data
 *
 *  @remarks
 */

extern uint8_t radarRangeProcessRun(void *handle, cplx16_t * pDataIn, cplx16_t * pDataOut);

/**
 *  @brief      Perform Doppler processing. Processing is done per all antennas per range bin.
 *
 *  @param[in]  handle               Pointer to instance handler
 *  @param[in]  pDataIn              Pointer to the input data (1D FFT output) for all antennas.
 *  @param[out] pDataOut             Pointer to the output integrated signal.
 *
 *  @remarks
 */

extern uint8_t radarDopplerProcessRun(void *handle, cplx16_t *pDataIn, float * pDataOut);

/**
 *  @brief      Perform frame processing.
 *
 *  @param[in]  handle               Pointer to instance handler
 *  @param[out] outBuffer             Pointer to the output X-Y Data
 *
 *  @remarks
 */

extern uint8_t radarFrameProcessRun(void *handle, void * outBuffer);


//! \brief      release process instance
//!
int32_t processDelete(void *handle);




#endif  // _PROCESS_H

/* Nothing past this point */

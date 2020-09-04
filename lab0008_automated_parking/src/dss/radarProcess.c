/*!
 *  \file   radarProcess.c
 *
 *  \brief   radar signal processing chain.
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


#include <radarProcess.h>
#include <math.h>
#ifdef TMDEMOV1
#include <modules/clustering/dbscan3D/api/RADARDEMO_clusteringDBscan3D.h>
#endif
#include <modules/detection/CFAR/api/RADARDEMO_detectionCFAR.h>
#include <modules/dopplerProc/dopplerProc/api/RADARDEMO_dopplerProc.h>
//#include <modules/rangeProc/rangeProc/api/RADARDEMO_rangeProc.h>
#include <modules/DoA/common/api/RADARDEMO_aoaEst_commonDef.h>
//#include <modules/DoA/DML/api/RADARDEMO_aoaEstDML.h>
#include <modules/DoA/BF/api/RADARDEMO_aoaEstBF.h>
#include <modules/utilities/cycle_measure.h>
#include <modules/utilities/radarOsal_malloc.h>
#include <modules/utilities/radar_c674x.h>


//#ifdef SOC_XWR18XX
#include <xdc/runtime/System.h>
//#endif

#define ONEOVERFACTORIAL3 (1.f/6.f)
#define ONEOVERFACTORIAL5 (1.f/230.f)
#define ONEOVERFACTORIAL7 (1.f/5040.f)
#define MAXANT (8)
#define MAXWIN1DSize (16)
#define MAXWIN2DSize (4)
//user input configuration parameters
typedef struct _processInstance_
{
    // frame timing in ms
	uint32_t dbscan_frameIdx;

	float framePeriod;
    void  * rangeFFTInstance;
    void  * dopplerFFInstance;
    void  * detectionInstance;
    void  * DoAInstance;

	//RADARDEMO_rangeProc_input *rangeProcInput;
    cplx16_t *pFFT1DBuffer;

	//float *localDopplerInBufPtr;
	RADARDEMO_dopplerProc_input *dopplerInput;
	float * localPDP;
	float ** localPDPPtr;

	RADARDEMO_detectionCFAR_output * detectionCFAROutput;

	uint8_t mimoModeFlag;  /**<Flag for MIMO mode: 0 -- SIMO, 1 -- TDM MIMO, 2 -- FDM or BF*/
	uint16_t doaMethod;  /**< DOA method, 3 for MPBF, 4 for DML, 0 for SPBF.*/
	RADARDEMO_aoAEst_input *aoaInput;
	float * aoaInputSignal;
	RADARDEMO_aoAEst_output *aoaOutput;

#ifdef TMDEMOV1
    void  * dbscanInstance;
    uint8_t  dbscan_enabled;
    //uint8_t  tracking_enabled;
    //void  * trackingInstance;

	RADARDEMO_clusteringDBscanInput *dbscanInputData;
    RADARDEMO_clusteringDBscanOutput *dbscanOutputData;
    RADARDEMO_clusteringDBscanErrorCodes dbscanErrorCode;
	uint16_t dbscan_maxPoints;
	//uint16_t dbscan_maxClusters;
	float	dbscan_inputScale;

    //number of frame accumulate for each clustering call
    uint16_t    numFrameAccumulate;   /**< number of frames to accumulate for each clustering call*/
    RADARDEMO_clusteringDBscanInputFormat inputPntPrecision;
    uint16_t frameCnt;  /**< frame counter used for DBscan*/
    //uint16_t pointCnt;
#endif

	//RADARDEMO_rangeProc_errorCode rangeProcErrorCode;
	RADARDEMO_dopplerProc_errorCode dopplerProcErrorCode;
	RADARDEMO_detectionCFAR_errorCode cfarErrorCode;
	//RADARDEMO_aoAEstDML_errorCode aoaDMLErrorCode;
	RADARDEMO_aoAEstBF_errorCode aoaBFErrorCode;

	int32_t fftSize1D;
	int32_t fftSize2D;
	int32_t numChirpsPerFrame;
	int32_t numAdcSamplePerChirp;
	int32_t nRxAnt;
	int32_t numAziAntenna;
	int32_t maxNumDetObj;
	float   rangeRes;
	float   dopplerRes;
	cplxf_t *antPhaseCompCoeff;

	//float    * scratchBuffer;
	radarProcessBenchmarkObj * benchmarkPtr;
}radarProcessInstance_t;



#define RAD_TO_DEG      ( 3.1415926f/180.f)
#define DEG_TO_RAD      ( 180.f/3.1415926f)

extern void radarProcess_gernerateGTrackerInput(radarProcessInstance_t *processInst, radarProcessOutputToTracker *radarPacketPtr);


void *radarProcessCreate(radarProcessConfig_t  * config, ProcessErrorCodes * procErrorCode)
{
    radarProcessInstance_t *inst;
	int32_t    i;
	//int16_t win1D[MAXWIN1DSize];
	float wind2DFloat[MAXWIN2DSize];
	int32_t itemp;
	ProcessErrorCodes errorCode = PROCESS_OK;
	
    inst = (radarProcessInstance_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(radarProcessInstance_t), 8);

	inst->antPhaseCompCoeff		=	(cplxf_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, 12 * sizeof(cplxf_t), 8);

	for (i = 0; i < config->numAntenna; i++)
	{
		inst->antPhaseCompCoeff[i].real = config->antPhaseCompCoeff[2 * i];
		inst->antPhaseCompCoeff[i].imag = config->antPhaseCompCoeff[2 * i + 1];
	}

	/*for (i = 0; i < (int32_t)config->rangeWinSize; i++ )
	{
		itemp		=	(int32_t) (32768.f * config->rangeWindow[i] + 0.5f);
		if (itemp == 32768) itemp = 32767;
		win1D[i]	=	itemp;
	}*/
	for (i = 0; i < (int32_t)config->DopplerWinSize; i++ )
	{
		wind2DFloat[i]	=	config->dopplerWindow[i];
	}

    itemp                               =   config->numAdcSamplePerChirp;
    if ((1 << (30 - _norm(itemp))) == itemp)
        inst->fftSize1D                 =   itemp;
    else
        inst->fftSize1D                 =   1 << (31 - _norm(itemp));
    config->fftSize1D                   =   inst->fftSize1D;
    inst->numChirpsPerFrame     =   config->numChirpPerFrame;
    inst->numAdcSamplePerChirp  =   config->numAdcSamplePerChirp;
    inst->nRxAnt                =   config->numAntenna;
    inst->numAziAntenna         =   config->numAziAntenna;

    /* zy: disable range proc */
	{/*
		RADARDEMO_rangeProc_config rangeProcConfig0;
		RADARDEMO_rangeProc_config *rangeProcConfig = &rangeProcConfig0;

		rangeProcConfig->nSamplesPerChirp	=	config->numAdcSamplePerChirp;
        rangeProcConfig->fft1DSize          =   inst->fftSize1D;
		rangeProcConfig->nRxAnt				=	(uint8_t)config->numPhyRxAntenna;
		rangeProcConfig->numChirpsPerFrame	=	config->numChirpPerFrame * config->numTxAntenna;
		rangeProcConfig->adcNumOutputBits	=	(uint8_t) config->numAdcBitsPerSample;
		rangeProcConfig->win1D				=	win1D;
		rangeProcConfig->win2D				=	NULL; 
		rangeProcConfig->adjustDCforInput	=	RADARDEMO_RANGEPROC_ADC_NO_ADJUST; 
		rangeProcConfig->win1DLength		=	(int32_t)config->rangeWinSize;

		rangeProcConfig->fft1DType			=	RADARDEMO_RANGEPROC_1DFFT_16x16; 

		rangeProcConfig->include2DwinFlag	=	0;  // hardcoded, memory limitation from XWR16xx

		rangeProcConfig->outputFixedpFlag	=	1;  

		inst->rangeFFTInstance		= (void *) RADARDEMO_rangeProc_create(rangeProcConfig, &inst->rangeProcErrorCode);
		if (inst->rangeProcErrorCode > RADARDEMO_RANGEPROC_NO_ERROR)
		{
			errorCode 	=	PROCESS_ERROR_RANGEPROC_INIT_FAILED;
		}
		inst->rangeProcInput	=	(RADARDEMO_rangeProc_input *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, 1*sizeof(RADARDEMO_rangeProc_input), 1);
		if (inst->rangeProcInput == NULL)
		{
			errorCode 	=	PROCESS_ERROR_RANGEPROC_INOUTALLOC_FAILED;
		}
		*/
	}


	/* Doppler proc */
	{
		RADARDEMO_dopplerProc_config dopplerProcConfig0;
		RADARDEMO_dopplerProc_config *dopplerProcConfig = &dopplerProcConfig0;

		itemp							=	config->numChirpPerFrame;
		if ((1 << (30 - _norm(itemp))) == itemp)
			inst->fftSize2D					=	itemp;
		else
			inst->fftSize2D					=	1 << (31 - _norm(itemp));
		dopplerProcConfig->fft2DSize		=	inst->fftSize2D; 
		dopplerProcConfig->numChirpsPerFrame =	config->numChirpPerFrame;
		dopplerProcConfig->win2DSize 		=	config->DopplerWinSize;
		config->fftSize2D					=	inst->fftSize2D; 
		dopplerProcConfig->nRxAnt			=	(uint8_t)config->numAntenna;
        dopplerProcConfig->nRxPhyAnt        =   (uint8_t)config->numPhyRxAntenna;
		dopplerProcConfig->win2D			=	wind2DFloat;		/* hardcoded*/
		dopplerProcConfig->fft2DType		=	RADARDEMO_DOPPLERPROC_2DFFT_SPxSP;
		dopplerProcConfig->intgrType		=	RADARDEMO_DOPPLERPROC_INTGRTYPE_SP;
		dopplerProcConfig->include2DwinFlag =	1;			/* hardcoded*/
		dopplerProcConfig->fxdpInputFlag	=	1;			/* hardcoded*/
		dopplerProcConfig->DCRemovalFlag    =   config->cfarConfig.clRemoval;

		inst->dopplerFFInstance = (void *) RADARDEMO_dopplerProc_create(dopplerProcConfig, &inst->dopplerProcErrorCode);
		if (inst->dopplerProcErrorCode > RADARDEMO_DOPPLERPROC_NO_ERROR)
		{
			errorCode 	=	PROCESS_ERROR_DOPPLERPROC_INIT_FAILED;
		}
		inst->dopplerInput	=	(RADARDEMO_dopplerProc_input *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_dopplerProc_input), 1);
		inst->dopplerInput->dopplerProcInput = (void **)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, config->numAntenna * sizeof(float *), 1);
		
		if ((inst->dopplerInput == NULL) || (inst->dopplerInput->dopplerProcInput == NULL))
		{
			errorCode 	=	PROCESS_ERROR_DOPPLERPROC_INOUTALLOC_FAILED;
		}
	}

	/* Detection CFAR */
	{
		RADARDEMO_detectionCFAR_config cfarConfig0;
		RADARDEMO_detectionCFAR_config *cfarConfig = &cfarConfig0;
		memset(cfarConfig, 0, sizeof(RADARDEMO_detectionCFAR_config));

		cfarConfig->fft1DSize		=	inst->fftSize1D; 
		cfarConfig->fft2DSize		=	inst->fftSize2D; 
		if (config->cfarConfig.cfarMethod == 6)
		{
			cfarConfig->cfarType		=	RADARDEMO_DETECTIONCFAR_RA_CASOCFAR;
			cfarConfig->enableSecondPassSearch		=	1;
		}
		else if (config->cfarConfig.cfarMethod == 4)
		{
			cfarConfig->cfarType		=	RADARDEMO_DETECTIONCFAR_CASOCFAR;
			cfarConfig->enableSecondPassSearch		=	1;
		}
		else if (config->cfarConfig.cfarMethod == 3)
		{
			cfarConfig->cfarType		=	RADARDEMO_DETECTIONCFAR_CASOCFAR;
			cfarConfig->enableSecondPassSearch		=	0;
		}
		else if (config->cfarConfig.cfarMethod == 2)
		{
			cfarConfig->cfarType		=	RADARDEMO_DETECTIONCFAR_CAVGCFAR;
			cfarConfig->enableSecondPassSearch		=	1;
		}
		else if (config->cfarConfig.cfarMethod == 1)
		{
			cfarConfig->cfarType		=	RADARDEMO_DETECTIONCFAR_CAVGCFAR;
			cfarConfig->enableSecondPassSearch		=	0;
		}
		else
		{
			errorCode 	=	PROCESS_ERROR_CFARPROC_NONSUPPORTEDMETHOD;
		}
		
		
		cfarConfig->inputType		=	RADARDEMO_DETECTIONCFAR_INPUTTYPE_SP; 
		cfarConfig->pfa				=	(float)1e-6; //hardcoded, not used for now;
		cfarConfig->rangeRes		=	config->cfarConfig.rangeRes * config->numAdcSamplePerChirp * _rcpsp(inst->fftSize1D);
		inst->rangeRes              =   cfarConfig->rangeRes;
		cfarConfig->dopplerRes		=	config->cfarConfig.velocityRes * config->numChirpPerFrame * _rcpsp(inst->fftSize2D);
        inst->dopplerRes            =   cfarConfig->dopplerRes;
		cfarConfig->maxNumDetObj	=	DOA_OUTPUT_MAXPOINTS;     //hardcoded
		cfarConfig->searchWinSizeRange	=	(uint8_t)config->cfarConfig.refWinSize[0];
		cfarConfig->guardSizeRange		=	(uint8_t)config->cfarConfig.guardWinSize[0];
		//hard coded for now for CASO
		cfarConfig->searchWinSizeDoppler=	(uint8_t)config->cfarConfig.refWinSize[1];
		cfarConfig->guardSizeDoppler	=	(uint8_t)config->cfarConfig.guardWinSize[1];
		cfarConfig->K0					=	config->cfarConfig.thre; 
		cfarConfig->dopplerSearchRelThr	=	config->cfarConfig.dopplerSearchRelThr; 
		cfarConfig->leftSkipSize		=	(uint8_t)config->cfarConfig.cfarDiscardLeft;
		cfarConfig->rightSkipSize		=	(uint8_t)config->cfarConfig.cfarDiscardRight;
		cfarConfig->log2MagFlag			=	config->cfarConfig.log2MagFlag; 
		cfarConfig->shortened1DInput    =   0;

		inst->detectionInstance = (void *) RADARDEMO_detectionCFAR_create(cfarConfig, &inst->cfarErrorCode);

        if (inst->cfarErrorCode > RADARDEMO_DETECTIONCFAR_NO_ERROR)
		{
			errorCode 	=	PROCESS_ERROR_CFARPROC_INIT_FAILED;
		}

		inst->maxNumDetObj	=	config->maxNumDetObj;         
		inst->detectionCFAROutput		=	(RADARDEMO_detectionCFAR_output *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_detectionCFAR_output), 1);
		inst->detectionCFAROutput->rangeInd	=	(uint16_t *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarConfig->maxNumDetObj * sizeof(uint16_t), 1);
		inst->detectionCFAROutput->dopplerInd	=	(uint16_t *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarConfig->maxNumDetObj * sizeof(uint16_t), 1);
		inst->detectionCFAROutput->rangeEst	=	(float *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarConfig->maxNumDetObj * sizeof(float), 1);
		inst->detectionCFAROutput->dopplerEst	=	(float *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarConfig->maxNumDetObj * sizeof(float), 1);
		inst->detectionCFAROutput->snrEst		=	(float *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarConfig->maxNumDetObj * sizeof(float), 1);
		inst->detectionCFAROutput->noise		=	(float *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarConfig->maxNumDetObj * sizeof(float), 1);

		if ( (inst->detectionCFAROutput == NULL)
			|| (inst->detectionCFAROutput->rangeInd == NULL)
			|| (inst->detectionCFAROutput->dopplerInd == NULL)
			|| (inst->detectionCFAROutput->rangeEst == NULL)
			|| (inst->detectionCFAROutput->dopplerEst == NULL)
			|| (inst->detectionCFAROutput->snrEst == NULL)
			|| (inst->detectionCFAROutput->noise == NULL))
		{
			errorCode 	=	PROCESS_ERROR_CFARPROC_INOUTALLOC_FAILED;
		}

	}

	inst->doaMethod = config->doaConfig.doaMethod;
	if ((config->doaConfig.doaMethod == 3) || (config->doaConfig.doaMethod == 1)) /* BF: MP or SP*/
	{
		RADARDEMO_aoAEstBF_config aoaBFConfigParam0;
		RADARDEMO_aoAEstBF_config *aoaBFConfigParam = &aoaBFConfigParam0;
		uint8_t antSpacing[MAXANT];

		aoaBFConfigParam->antSpacing = &antSpacing[0];
		for (i = 0; i < (int32_t)config->numAziAntenna; i++ )
		{
			aoaBFConfigParam->antSpacing[i] = i;
		}

		if(config->doaConfig.doaMethod == 3)
			aoaBFConfigParam->enableMultiPeakSearch = 1;
		else
			aoaBFConfigParam->enableMultiPeakSearch = 0;
		aoaBFConfigParam->vmaxUnrollFlag = config->doaConfig.vmaxUnrollFlag; 
		aoaBFConfigParam->nRxAnt = config->numAziAntenna;
		aoaBFConfigParam->estResolution = config->doaConfig.doaSearchRes;
		aoaBFConfigParam->estRange = config->doaConfig.doaSearchRange;
		aoaBFConfigParam->sidelobeLevel_dB = config->doaConfig.doaSideLobeLevel_dB;
		aoaBFConfigParam->maxOutputVar	=	config->doaConfig.doaVarThr;
		aoaBFConfigParam->angleEstBound	=	config->doaConfig.doaSearchRange + 7.5f;   //hardcoded

		inst->DoAInstance = (void *) RADARDEMO_aoaEstimationBF_create(aoaBFConfigParam, &inst->aoaBFErrorCode);
		if (inst->aoaBFErrorCode > RADARDEMO_AOABF_NO_ERROR)
		{
			errorCode 	=	PROCESS_ERROR_DOAPROC_INIT_FAILED;
		}
		inst->aoaOutput	=	(RADARDEMO_aoAEst_output *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, config->maxNumDetObj*sizeof(RADARDEMO_aoAEst_output), 1);
		inst->aoaInput		=	(RADARDEMO_aoAEst_input *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_aoAEst_input), 1);
	    inst->aoaInputSignal = (float *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, 2 * config->numAntenna * sizeof (float), 8);

		if ((inst->aoaOutput == NULL) || (inst->aoaInput == NULL) || (inst->aoaInputSignal == NULL))
		{
			errorCode 	=	PROCESS_ERROR_DOAPROC_INOUTALLOC_FAILED;
		}
	}
	else
	{
		errorCode 	=	PROCESS_ERROR_DOAPROC_NONSUPPORTEDMETHOD;
	}

	{//dbscan3D
#ifdef TMDEMOV1
	    RADARDEMO_clusteringDBscanConfig clusteringDBscanPara;
	    inst->dbscan_enabled			=	1;
	    //inst->tracking_enabled			=	0;
        clusteringDBscanPara.epsilon = config->dbscanConfig.epsilon;
        clusteringDBscanPara.weight = config->dbscanConfig.weight ;
        clusteringDBscanPara.maxClusters = CLUSTERING_MAX_CLUSTERS;
        clusteringDBscanPara.minPointsInCluster = config->dbscanConfig.minPointsInCluster;
        clusteringDBscanPara.maxPoints = DOA_OUTPUT_MAXPOINTS * config->dbscanConfig.nAccFrames;
        clusteringDBscanPara.inputPntPrecision = DBSCAN_INPUT_16BITFIXEDPT; //DBSCAN_INPUT_16BITFIXEDPT; //DBSCAN_INPUT_FLOATINGPT; //both point point and float point are matching, but the optimized code is no longer working.
        clusteringDBscanPara.fixedPointScale = config->dbscanConfig.fixedPointScale;
        inst->inputPntPrecision = clusteringDBscanPara.inputPntPrecision;
        inst->dbscan_inputScale = clusteringDBscanPara.fixedPointScale;
        inst->numFrameAccumulate = config->dbscanConfig.nAccFrames;
        inst->dbscan_maxPoints = clusteringDBscanPara.maxPoints;
        //inst->dbscan_maxPoints = clusteringDBscanPara.maxPoints;

        inst->dbscanInstance  =   (void *) RADARDEMO_clusteringDBscanCreate(&clusteringDBscanPara);
        if (inst->dbscanInstance == NULL)
            errorCode   =   PROCESS_ERROR_DBSCAN_INIT_FAILED;

        inst->dbscanInputData           =   (RADARDEMO_clusteringDBscanInput *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_clusteringDBscanInput), 1);
        inst->dbscanInputData->pointArray.pointArrayInt16   =   (int16_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, 3 * clusteringDBscanPara.maxPoints * sizeof(int16_t), 1);
        inst->dbscanInputData->speed.speedInt16     =   (int16_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, clusteringDBscanPara.maxPoints * sizeof(int16_t), 1);
        inst->dbscanInputData->numPoints    =   0;
        inst->dbscanInputData->inputPntPrecision = inst->inputPntPrecision;

        inst->dbscanOutputData          =   (RADARDEMO_clusteringDBscanOutput *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_clusteringDBscanOutput), 1);
        inst->dbscanOutputData->IndexArray      =   (uint16_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, inst->dbscan_maxPoints * sizeof(uint16_t), 1);
        //inst->dbscanOutputData->report  =   (RADARDEMO_clusteringDBscanReport *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, clusteringDBscanPara.maxClusters * sizeof(RADARDEMO_clusteringDBscanReport), 1);

        if ((inst->dbscanInputData == NULL) || (inst->dbscanInputData->pointArray.pointArrayInt16 == NULL) || (inst->dbscanInputData->speed.speedInt16 == NULL)
            ||(inst->dbscanOutputData == NULL) || (inst->dbscanOutputData->IndexArray == NULL))
        {
            errorCode   =   PROCESS_ERROR_DBSCAN_INOUTALLOC_FAILED;
        }
        inst->frameCnt = 0;
#endif
	}



	//inst->framePeriod  = config->framePeriod;
	inst->pFFT1DBuffer = config->pFFT1DBuffer; //(cplx16_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, inst->numChirpsPerFrame * inst->nRxAnt * inst->fftSize1D * sizeof (cplx16_t), 8);

	inst->mimoModeFlag  	=  config->mimoModeFlag;
	inst->localPDP  	=  (float * ) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, inst->fftSize2D * inst->fftSize1D * sizeof(float), 8);
	inst->localPDPPtr	=	(float **)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, inst->fftSize2D*sizeof(float * ), 1);
	for (i = 0; i < (int32_t) inst->fftSize2D; i++)
	{
		inst->localPDPPtr[i]	=	(float *)&inst->localPDP[i * inst->fftSize1D];
	}
    config->heatMapMemSize = inst->fftSize1D * inst->fftSize2D * sizeof(float);
    config->heatMapMem = inst->localPDP;

	//inst->scratchBuffer		=	(float *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, 2 * inst->fftSize1D * sizeof(float), 8);

	inst->benchmarkPtr = (radarProcessBenchmarkObj *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(radarProcessBenchmarkObj), 1);
	inst->benchmarkPtr->bufferLen = 20;
	inst->benchmarkPtr->bufferIdx = 0;
	inst->benchmarkPtr->buffer    = (radarProcessBenchmarkElem *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, inst->benchmarkPtr->bufferLen * sizeof(radarProcessBenchmarkElem), 1);

	memset(inst->benchmarkPtr->buffer, 0, inst->benchmarkPtr->bufferLen * sizeof(radarProcessBenchmarkElem));

	if ((inst->localPDP == NULL) || (inst->localPDPPtr== NULL)|| (inst->pFFT1DBuffer == NULL)
		//||(inst->scratchBuffer == NULL) 
		||(inst->benchmarkPtr == NULL) || (inst->benchmarkPtr->buffer == NULL))
		errorCode 	=	PROCESS_ERROR_INIT_MEMALLOC_FAILED;


	config->benchmarkPtr = inst->benchmarkPtr;
	//config->pFFT1DBuffer = inst->pFFT1DBuffer;

	*procErrorCode = errorCode;

#ifndef CCS
	System_printf("processCreate: (radarProcessInstance_t *)0x%x\n", (uint32_t)inst);
	System_printf("processCreate: (RADARDEMO_rangeProc_handle *)0x%x\n", (uint32_t)(inst->rangeFFTInstance));
	System_printf("processCreate: (RADARDEMO_dopplerProc_handle *)0x%x\n", (uint32_t)(inst->dopplerFFInstance));
	System_printf("processCreate: (RADARDEMO_detectionCFAR_handle *)0x%x\n", (uint32_t)(inst->detectionInstance));
	if((config->doaConfig.doaMethod == 3) || (config->doaConfig.doaMethod == 1))
		System_printf("processCreate: (RADARDEMO_aoAEstBF_handle *)0x%x\n", (uint32_t)(inst->DoAInstance));
	else
	System_printf("processCreate: (RADARDEMO_aoAEstDML_handle *)0x%x\n", (uint32_t)(inst->DoAInstance));
#ifdef TMDEMOV1
	System_printf("processCreate: (RADARDEMO_clusteringDBscanInstance *)0x%x\n", (uint32_t)(inst->dbscanInstance));
	//System_printf("processCreate: (RADARDEMO_clusterTracker_handle *)0x%x\n", (uint32_t)(inst->trackingInstance));
#endif
	System_printf("processCreate: (radarProcessBenchmarkObj *)0x%x\n", (uint32_t)(inst->benchmarkPtr));
	System_printf("processCreate: heatmap (float *)0x%x\n", (uint32_t)(inst->localPDP));
#endif
	return (void *) inst;
}

//zy: disable range Proc
/*uint8_t radarRangeProcessRun(void *handle, cplx16_t * pDataIn, cplx16_t * pDataOut)
{
    radarProcessInstance_t *processInst = (radarProcessInstance_t *)handle;

	processInst->rangeProcInput->inputSignal = pDataIn;
	processInst->rangeProcInput->chirpNumber = 0;			//irrelavent here because 2D windowing is done in Doppler processing.
	processInst->rangeProcErrorCode = RADARDEMO_rangeProc_run(
							processInst->rangeFFTInstance,
							processInst->rangeProcInput,
							(void *)pDataOut);
	return((uint8_t)processInst->rangeProcErrorCode);
}*/

uint8_t radarDopplerProcessRun(void *handle, cplx16_t * pDataIn, float *pDataOut)
{
    radarProcessInstance_t *processInst = (radarProcessInstance_t *)handle;
	int32_t j;

	for (j = 0; j < processInst->nRxAnt; j++)
	{
		processInst->dopplerInput->dopplerProcInput[j]	=	(void * )&pDataIn[j * processInst->fftSize2D];
	}
	
	processInst->dopplerInput->reGen2DFFTout4AoAFlag = 0;
	processInst->dopplerInput->dopplerComp4TDMMimo = 0;

	processInst->dopplerProcErrorCode = RADARDEMO_dopplerProc_run(
						processInst->dopplerFFInstance,
						processInst->dopplerInput,
						pDataOut);
	return((uint8_t)processInst->dopplerProcErrorCode);
}


uint8_t radarFrameProcessRun(void *handle, void * outBuffer)
{
    radarProcessInstance_t *processInst = (radarProcessInstance_t *)handle;
    uint8_t status = PROCESS_OK;
	int32_t i, j; //, antID;
	uint32_t timeStart, timeStamp;
	radarProcessBenchmarkObj * benchmarks = processInst->benchmarkPtr;
    RadarDsp_outputBuffCntxt * outBuffCntxt;

	outBuffCntxt = (RadarDsp_outputBuffCntxt*)outBuffer;

    //TSCL    = 0;
    //t_start = _itoll(TSCH, TSCL);
	timeStart	=	ranClock();
	timeStamp 	=	timeStart;
	if (benchmarks->bufferIdx > benchmarks->bufferLen)
		benchmarks->bufferIdx = 0;

	// Detection
	processInst->cfarErrorCode	=	RADARDEMO_detectionCFAR_run(
                            processInst->detectionInstance,
							processInst->localPDPPtr,
							processInst->detectionCFAROutput);

	timeStart	=	ranClock();
	benchmarks->buffer[benchmarks->bufferIdx].cfarDetectionCycles 	=	timeStart - timeStamp;
	benchmarks->buffer[benchmarks->bufferIdx].numDetPnts 	=	processInst->detectionCFAROutput->numObjDetected;
	timeStamp 	=	timeStart;

	for(i = 0; i < processInst->detectionCFAROutput->numObjDetected; i++ )
	{
		uint16_t rangeIndex, dopplerIndex;

		rangeIndex		=	processInst->detectionCFAROutput->rangeInd[i];
		dopplerIndex	=	processInst->detectionCFAROutput->dopplerInd[i];

		for (j = 0; j < processInst->nRxAnt; j++)
		{
			processInst->dopplerInput->dopplerProcInput[j]	=	(void * )&processInst->pFFT1DBuffer[rangeIndex * processInst->nRxAnt * processInst->numChirpsPerFrame + j * processInst->numChirpsPerFrame];
		}

		processInst->dopplerInput->reGen2DFFTout4AoAFlag = 1;
		processInst->dopplerInput->dopplerIndex		=	dopplerIndex;
		if (processInst->mimoModeFlag == 1)
			processInst->dopplerInput->dopplerComp4TDMMimo = 1;
		else
			processInst->dopplerInput->dopplerComp4TDMMimo = 0;

		RADARDEMO_dopplerProc_run(
						processInst->dopplerFFInstance,
						processInst->dopplerInput,
						(float *) processInst->aoaInputSignal);

		processInst->aoaInput->inputAntSamples	=	(cplxf_t *) processInst->aoaInputSignal;
		processInst->aoaInput->inputNoisePow	=	(float) processInst->detectionCFAROutput->noise[i];


		if (( processInst->doaMethod == 3 ) || ( processInst->doaMethod == 1 ))//MPBF
		{
			__float2_t f2temp;
			for (j = 0; j < processInst->nRxAnt; j++)
			{
				f2temp		=	_amem8_f2(&processInst->aoaInput->inputAntSamples[j]);
				f2temp		=	_complex_mpysp(f2temp, _amem8_f2(&processInst->antPhaseCompCoeff[j]));
				_amem8_f2(&processInst->aoaInput->inputAntSamples[j])	=	f2temp;
			}
			processInst->aoaBFErrorCode = RADARDEMO_aoaEstimationBF_run(
                            processInst->DoAInstance,
							processInst->aoaInput,
							&processInst->aoaOutput[i]);
		}
	}
    timeStart   =   ranClock();
    benchmarks->buffer[benchmarks->bufferIdx].aoaCycles   =   timeStart - timeStamp;
    timeStamp   =   timeStart;
	

    radarProcessOutputToTracker                 * pDataOut;
    pDataOut     = (radarProcessOutputToTracker *)outBuffCntxt->elem[RadarDsp_outputDataType_OBJ_DATA].buff;

    radarProcess_gernerateGTrackerInput(processInst, pDataOut);
    timeStart   =   ranClock();
    benchmarks->buffer[benchmarks->bufferIdx].dataConvCycles    =   timeStart - timeStamp;
    timeStamp   =   timeStart;

	if (processInst->cfarErrorCode == RADARDEMO_DETECTIONCFAR_CORRUPTED_2DSIZE)
		status = PROCESS_ERROR_CFARPROC_INOUTALLOC_FAILED;

    benchmarks->bufferIdx++;
    if (benchmarks->bufferIdx > benchmarks->bufferLen)
        benchmarks->bufferIdx = 0;

#ifdef TMDEMOV1
    processInst->frameCnt ++;
    //processInst->dbscanOutputData->IndexArray = &(pDataOut->IndexArray[0]);
    //processInst->dbscanOutputData->report = &(pDataOut->report[0]);
    if (processInst->numFrameAccumulate == processInst->frameCnt)
    {
        //call clustering function every numFrameAccumulate frames, hardcoded for now
        processInst->dbscanOutputData->numCluster = 0;
        processInst->dbscanOutputData->report = pDataOut->dbscanReport;
        processInst->dbscanErrorCode = RADARDEMO_clusteringDBscanRun( processInst->dbscanInstance,processInst->dbscanInputData, processInst->dbscanOutputData);
        pDataOut->numCluster = processInst->dbscanOutputData->numCluster;
        pDataOut->clusterOutFlag = 1;
        //reset values
        processInst->dbscanInputData->numPoints = 0;
        processInst->frameCnt = 0;
    }
    else
        pDataOut->clusterOutFlag = 0;

#endif



    return status;

}


int32_t processDelete(void *handle)
{
    radarProcessInstance_t *inst = (radarProcessInstance_t *)handle;
    //EDMA3_DRV_Result edma3Result = EDMA3_DRV_SOK;

    //RADARDEMO_rangeProc_delete(inst->rangeFFTInstance);
    RADARDEMO_dopplerProc_delete(inst->dopplerFFInstance);
    RADARDEMO_detectionCFAR_delete(inst->detectionInstance);
	if ( (inst->doaMethod == 3) ||  (inst->doaMethod == 1)) //MPBF
	    RADARDEMO_aoaEstimationBF_delete(inst->DoAInstance);
	//else if ( inst->doaMethod == 4 ) //DML
	//	RADARDEMO_aoaEstimationDML_delete(inst->DoAInstance);

#ifdef TMDEMOV1
    RADARDEMO_clusteringDBscanDelete(inst->dbscanInstance);
	//RADARDEMO_clusterTracker_delete(inst->trackingInstance);
    radarOsal_memFree(inst->dbscanInputData, sizeof(RADARDEMO_clusteringDBscanInput));
    radarOsal_memFree(inst->dbscanInputData->pointArray.pointArrayInt16, 3 * inst->dbscan_maxPoints * sizeof(int16_t));
    radarOsal_memFree(inst->dbscanInputData->speed.speedInt16, inst->dbscan_maxPoints * sizeof(int16_t));
    radarOsal_memFree(inst->dbscanOutputData, sizeof(RADARDEMO_clusteringDBscanOutput));
    radarOsal_memFree(inst->dbscanOutputData->IndexArray, inst->dbscan_maxPoints * sizeof(uint16_t));
#endif

    //radarOsal_memFree(inst->rangeProcInput, sizeof(RADARDEMO_rangeProc_input));
    radarOsal_memFree(inst->dopplerInput->dopplerProcInput, inst->nRxAnt * sizeof(float *));
    radarOsal_memFree(inst->dopplerInput, sizeof(RADARDEMO_dopplerProc_input));

    radarOsal_memFree(inst->localPDPPtr, inst->fftSize2D*sizeof(float * ));
    radarOsal_memFree(inst->localPDP, inst->fftSize2D * inst->fftSize1D * sizeof(float));

    radarOsal_memFree(inst->antPhaseCompCoeff, 8 * sizeof(cplxf_t));

    radarOsal_memFree(inst->detectionCFAROutput->rangeInd, inst->maxNumDetObj *sizeof(uint16_t ));
    radarOsal_memFree(inst->detectionCFAROutput->dopplerInd, inst->maxNumDetObj *sizeof(uint16_t ));
    radarOsal_memFree(inst->detectionCFAROutput->rangeEst, inst->maxNumDetObj *sizeof(float ));
    radarOsal_memFree(inst->detectionCFAROutput->dopplerEst, inst->maxNumDetObj *sizeof(float ));
    radarOsal_memFree(inst->detectionCFAROutput->snrEst, inst->maxNumDetObj *sizeof(float ));
    radarOsal_memFree(inst->detectionCFAROutput->noise, inst->maxNumDetObj *sizeof(float ));
    radarOsal_memFree(inst->detectionCFAROutput, sizeof(RADARDEMO_detectionCFAR_output));

    radarOsal_memFree(inst->aoaOutput, inst->maxNumDetObj *sizeof(RADARDEMO_aoAEst_output ));
    radarOsal_memFree(inst->aoaInput, sizeof(RADARDEMO_aoAEst_input));

    //radarOsal_memFree(inst->pFFT1DBuffer, 2 * inst->nRxAnt * inst->fftSize2D * inst->fftSize1D * sizeof (float));
    radarOsal_memFree(inst->aoaInputSignal, 2 * inst->nRxAnt * sizeof (float));
    //radarOsal_memFree(inst->scratchBuffer, 2 * inst->fftSize1D * sizeof (float));

    radarOsal_memFree(inst->benchmarkPtr->buffer, inst->benchmarkPtr->bufferLen * sizeof(radarProcessBenchmarkElem));
    radarOsal_memFree(inst->benchmarkPtr, sizeof(radarProcessBenchmarkObj));

    radarOsal_memFree(handle, sizeof(radarProcessInstance_t));

    return PROCESS_OK;

}



void radarProcess_gernerateGTrackerInput(radarProcessInstance_t *processInst, radarProcessOutputToTracker *radarPacketPtr)
{
    int32_t i_det, i_angle, objCnt, rangeIdx, velocityIdx;
    float range, velocity, angle, eAngle, linearSNR;
    RADARDEMO_detectionCFAR_output *detectionResultsPtr = (RADARDEMO_detectionCFAR_output *)processInst->detectionCFAROutput;
    RADARDEMO_aoAEst_output *DoAestOutput = (RADARDEMO_aoAEst_output *)processInst->aoaOutput;

#ifdef TMDEMOV1
    int16_t * pointArrayPtr, * speedArrayPtr;
    pointArrayPtr = &processInst->dbscanInputData->pointArray.pointArrayInt16[3 * processInst->dbscanInputData->numPoints];
    speedArrayPtr = &processInst->dbscanInputData->speed.speedInt16[processInst->dbscanInputData->numPoints];
#endif

    memset(radarPacketPtr, 0, sizeof(radarProcessOutputToTracker));
    objCnt = 0;
    for(i_det = 0; i_det < detectionResultsPtr->numObjDetected; i_det++)
    {
        range       =   detectionResultsPtr->rangeEst[i_det];
        velocity    =   detectionResultsPtr->dopplerEst[i_det];
        rangeIdx       =   detectionResultsPtr->rangeInd[i_det];
        velocityIdx    =   detectionResultsPtr->dopplerInd[i_det];
        linearSNR   =   detectionResultsPtr->snrEst[i_det];

        for (i_angle = 0; i_angle < (int32_t)DoAestOutput[i_det].numOutput;  i_angle ++)
        {
            angle            = DoAestOutput[i_det].outputAngles[i_angle];
            eAngle           = DoAestOutput[i_det].outputEAngles[i_angle];
            radarPacketPtr->range[objCnt]           =  range;
            radarPacketPtr->snr[objCnt]        		=  linearSNR;
            radarPacketPtr->doppler[objCnt]         =  velocity;
            radarPacketPtr->rangeIdx[objCnt]        =  rangeIdx;
            radarPacketPtr->snr[objCnt]             =  linearSNR;
            radarPacketPtr->dopplerIdx[objCnt]      =  velocityIdx;
            //radarPacketPtr->dopplerVar[objCnt]      =  velocityVar;
            radarPacketPtr->angle[objCnt]           =  angle;
            radarPacketPtr->eAngle[objCnt]          =  eAngle;
            radarPacketPtr->angleVar[objCnt]        =  DoAestOutput[i_det].outputVar[i_angle];

#ifdef TMDEMOV1
            if (processInst->dbscan_enabled)
            {
                float angle_, eAngle_;
                angle_ = angle * 0.0175; //RADARPROC_PIOVER180;
                eAngle_ = eAngle * 0.0175; //RADARPROC_PIOVER180;
                pointArrayPtr[0] = (int16_t)(range *(float)(sin(angle_)) * cos(eAngle_) * (float)(processInst->dbscan_inputScale) + 0.5);
                pointArrayPtr[1] = (int16_t)(range *(float)(cos(angle_)) * cos(eAngle_) * (float)(processInst->dbscan_inputScale) + 0.5);
                pointArrayPtr[2] = (int16_t)(range *(float)(sin(eAngle_)) * (float)(processInst->dbscan_inputScale) + 0.5);
                speedArrayPtr[0] = (int16_t)(velocity * (float)(processInst->dbscan_inputScale) + 0.5);

                processInst->dbscanInputData->numPoints ++;
                pointArrayPtr = pointArrayPtr + 3;
                speedArrayPtr ++;
            }
#endif

            objCnt++;
            if (objCnt > DOA_OUTPUT_MAXPOINTS)
                break;
        }
        if (objCnt > DOA_OUTPUT_MAXPOINTS)
            break;
    }

    radarPacketPtr->object_count = objCnt;
}

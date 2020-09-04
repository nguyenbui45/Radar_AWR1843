/**
*   @file  dss_data_path.c
*
*   @brief
*      Implements Data path processing functionality.
*
*  \par
*  NOTE:
*      (C) Copyright 2018 Texas Instruments, Inc.
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
*/

/**************************************************************************
*************************** Include Files ********************************
**************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#if defined (SUBSYS_DSS)
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>
#endif
#define DebugP_ASSERT_ENABLED 1
#include <ti/drivers/osal/DebugP.h>
#include <assert.h>
#include <ti/common/sys_common.h>
#include <ti/drivers/osal/SemaphoreP.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/soc/soc.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

#include <ti/alg/mmwavelib/mmwavelib.h>
/* C64P dsplib (fixed point part for C674X) */
//#include "gen_twiddle_fft32x32.h"
#include "gen_twiddle_fft16x16_imre.h"
//#include "DSP_fft32x32.h"
//#include "DSP_fft16x16.h"
#include "DSP_fft16x16_imre.h"

/* C674x mathlib */
/* Suppress the mathlib.h warnings
 *  #48-D: incompatible redefinition of macro "TRUE"
 *  #48-D: incompatible redefinition of macro "FALSE"
 */
#pragma diag_push
#pragma diag_suppress 48
#include <ti/mathlib/mathlib.h>
#pragma diag_pop

#include "../common/pa_config_consts.h"
#include "dss_data_path.h"
#include "dss_pa.h"
#include "dss_config_edma_util.h"


#define MMW_ADCBUF_SIZE     0x4000U

/*! @brief L2 heap used for allocating buffers in L2 SRAM,
mostly scratch buffers */
#define MMW_L2_HEAP_SIZE    0x11000U
#define SOC_XWR18XX_DSS_L2_SCRATCH_SIZE           0x2000U
#define SOC_XWR18XX_DSS_L2_BUFF_SIZE              0xF000U

/*! @brief L1 heap used for allocating buffers in L1D SRAM,
mostly scratch buffers */
#define MMW_L1_HEAP_SIZE    0x4000U

/*! L3 RAM buffer */
#pragma DATA_SECTION(gMmwL3, ".l3data");
#pragma DATA_ALIGN(gMmwL3, 8);
uint8_t gMmwL3[SOC_L3RAM_SIZE];

/*! L2 RAM buffer */
#pragma DATA_SECTION(gMmwL2, ".l2data");
#pragma DATA_ALIGN(gMmwL2, 8);
uint8_t gMmwL2[SOC_XWR18XX_DSS_L2_BUFF_SIZE];

/*! L2 RAM scratch */
#pragma DATA_SECTION(gMmwL2, ".l2data");
#pragma DATA_ALIGN(gMmwL2, 8);
uint8_t gMmwL2Scratch[SOC_XWR18XX_DSS_L2_SCRATCH_SIZE];

/*! L1 Heap */
#pragma DATA_SECTION(gMmwL1, ".l1data");
#pragma DATA_ALIGN(gMmwL1, 8);
uint8_t gMmwL1[MMW_L1_HEAP_SIZE];

/*! HSRAM buffer */
/*meta information for detected Objects shared with MSS */
//#pragma DATA_SECTION(detOutputHdr, ".hsramdata");
//#pragma DATA_ALIGN(detOutputHdr, 8);
//MmwDemo_detOutputHdr detOutputHdr;
//#pragma DATA_SECTION(gMmwHSRAM, ".demoSharedMem");
//#pragma DATA_ALIGN(gMmwHSRAM, 8);
//uint8_t gMmwHSRAM[SOC_XWR18XX_DSS_HSRAM_SIZE];


/*! Types of FFT window */
/*! FFT window 16 - samples format is int16_t */
#define FFT_WINDOW_INT16 0
/*! FFT window 32 - samples format is int32_t */
#define FFT_WINDOW_INT32 1

/* FFT Window */
/*! Hanning window */
#define MMW_WIN_HANNING  0
/*! Blackman window */
#define MMW_WIN_BLACKMAN 1
/*! Rectangular window */
#define MMW_WIN_RECT     2
/* Main control structure. */
extern Mrr_DSS_MCB gMrrDSSMCB;
/* Local defines. */
#define pingPongId(x) ((x) & 0x1U)
#define isPong(x) (pingPongId(x))

#define PING          0
#define PONG          1



void MmwDemo_genWindow(void *win,
    uint32_t windowDatumType,
    uint32_t winLen,
    uint32_t winGenLen,
    int32_t oneQformat,
    uint32_t winType);

int16_t disambiguateVel(uint16_t * restrict sumAbs, float fastChirpVel, uint16_t fastChirpPeakVal, MmwDemo_DSS_DataPathObj * obj);

uint32_t findKLargestPeaks(uint16_t * restrict cfarDetObjIndexBuf,
                           uint16_t * restrict cfarDetObjSNR,
                           uint32_t numDetObjPerCfar,
                           uint16_t * restrict sumAbs,
                           uint16_t numBins,
                           uint16_t K);

uint32_t pruneToPeaks(uint16_t* restrict cfarDetObjIndexBuf,
                      uint16_t* restrict cfarDetObjSNR,
                      uint32_t numDetObjPerCfar,
                      uint16_t* restrict sumAbs,
                      uint16_t numBins);

uint32_t pruneToPeaksOrNeighbourOfPeaks(uint16_t* restrict cfarDetObjIndexBuf,
                      uint16_t* restrict cfarDetObjSNR,
                      uint32_t numDetObjPerCfar,
                      uint16_t* restrict sumAbs,
                      uint16_t numBins);

uint32_t findandPopulateIntersectionOfDetectedObjects(
    MmwDemo_DSS_DataPathObj * restrict obj,
    uint32_t numDetObjPerCfar,
    uint16_t dopplerLine,
    uint32_t numDetObj2D,
    uint16_t * restrict sumAbsRange);

uint32_t findandPopulateDetectedObjects(MmwDemo_DSS_DataPathObj * restrict obj,    uint32_t numDetObjPerCfar,
                                        uint16_t dopplerLine, uint32_t numDetObj2D, uint16_t * restrict sumAbsRange);

uint32_t MmwDemo_cfarPeakGrouping(MmwDemo_detectedObjActual*  objOut, uint32_t numDetectedObjects,
                                uint16_t* detMatrix, uint32_t numRangeBins, uint32_t numDopplerBins,
                                uint32_t groupInDopplerDirection, uint32_t groupInRangeDirection);

uint32_t secondDimFFTandLog2Computation(MmwDemo_DSS_DataPathObj *obj, uint16_t * sumAbs,
                                        uint16_t checkDetMatrixTx, uint16_t rangeIdx,
                                        uint32_t * pingPongIdxPtr);


uint32_t cfarCa_SO_dBwrap_withSNR(const uint16_t inp[restrict],
                                uint16_t out[restrict],
                                uint16_t outSNR[restrict],
                                uint32_t len,
                                uint32_t const1, uint32_t const2,
                                uint32_t guardLen, uint32_t noiseLen);
uint32_t cfarCadB_SO_withSNR(const uint16_t inp[restrict],
                            uint16_t out[restrict],
                            uint16_t outSNR[restrict], uint32_t len,
                            uint32_t const1, uint32_t const2,
                            uint32_t guardLen, uint32_t noiseLen,
                            uint32_t minIndxToIgnoreHPF);

//uint32_t aziEleProcessing(MmwDemo_DSS_DataPathObj *obj, uint32_t subframeIndx);

//uint16_t computeSinAzimSNR(float * azimuthMagSqr, uint16_t azimIdx, uint16_t numVirtualAntAzim, uint16_t numAngleBins, uint16_t xyzOutputQFormat);

float antilog2(int32_t inputActual, uint16_t fracBitIn);


float quadraticInterpLog2ShortPeakLoc(uint16_t * restrict y, int32_t len, int32_t indx, uint16_t fracBitIn);

extern volatile cycleLog_t gCycleLog;



void populateOutputs(MmwDemo_DSS_DataPathObj *obj)
{
    uint32_t ik;
    radarProcessOutputToTracker                 * pDataOut;
    pDataOut     = (radarProcessOutputToTracker *)obj->outBuffCntxt->elem[RadarDsp_outputDataType_OBJ_DATA].buff;

#ifndef OGMAP_INTERFACE
    float range, angle, eAngle;
    float oneQFormat = (float) (1U << obj->xyzOutputQFormat);
    MmwDemo_detectedObjForTx * restrict detObjFinal = obj->detObjFinal;
    obj->numDetObj = pDataOut->object_count;

    _nassert((uint32_t) detObjFinal % 8 == 0);

    /* 1. Detected Object List */
    for (ik = 0; ik < obj->numDetObj; ik ++)
    {
        range = pDataOut->range[ik];
        angle = pDataOut->angle[ik] * 0.0175; //RADARPROC_PIOVER180;
        eAngle = pDataOut->eAngle[ik] * 0.0175; //RADARPROC_PIOVER180;
        detObjFinal[ik].peakVal  = 0;
        detObjFinal[ik].speedIdx = pDataOut->dopplerIdx[ik];
        detObjFinal[ik].rangeIdx = pDataOut->rangeIdx[ik];
        detObjFinal[ik].x        = (int16_t)(range *(float)(sin(angle)) * cos(eAngle) * oneQFormat);
        detObjFinal[ik].y        = (int16_t)(range *(float)(cos(angle)) * cos(eAngle) * oneQFormat);
        detObjFinal[ik].z        = (int16_t)(range *(float)(sin(eAngle)) * oneQFormat);
    }

#else
//    float angle, eAngle;
    RadarDsp_DetObjInfo * restrict ogmapInput = &obj->outputForOGMAP->objInfo[0];

    obj->outputForOGMAP->numObj = pDataOut->object_count;
    for (ik = 0; ik < pDataOut->object_count; ik ++)
    {
//        angle = pDataOut->angle[ik];
//        eAngle = pDataOut->eAngle[ik];
        ogmapInput[ik].velocity = pDataOut->doppler[ik];
        ogmapInput[ik].range = pDataOut->range[ik];
        ogmapInput[ik].azimuthAngle = pDataOut->angle[ik];
        ogmapInput[ik].elevAngle    = pDataOut->eAngle[ik];
        ogmapInput[ik].azimAngleVarEst = pDataOut->angleVar[ik];
        ogmapInput[ik].snr        = pDataOut->snr[ik];
    }
#endif

#ifdef TMDEMOV1
    /* 2. Clustering output for the point cloud subframe. */
    if (pDataOut->clusterOutFlag == 1)
    {
        for (ik = 0; ik < pDataOut->numCluster; ik ++)
        {
            obj->dbscanReportFinal[ik].xCenter      =  pDataOut->dbscanReport[ik].xCenter;
            obj->dbscanReportFinal[ik].yCenter      =  pDataOut->dbscanReport[ik].yCenter;;
            obj->dbscanReportFinal[ik].zCenter      =  pDataOut->dbscanReport[ik].zCenter;;
            obj->dbscanReportFinal[ik].xSize        =  pDataOut->dbscanReport[ik].xSize;
            obj->dbscanReportFinal[ik].ySize        =  pDataOut->dbscanReport[ik].ySize;;
            obj->dbscanReportFinal[ik].zSize        =  pDataOut->dbscanReport[ik].zSize;;
        }
        obj->numCluster = pDataOut->numCluster;
        obj->clusterGenFlag = 1;
    }
    else
        obj->clusterGenFlag = 0;
#endif

}

/**
*  @b Description
*  @n
*      selects one of four channels based on the subframe and the 'ping pong' ID
*
*/
uint8_t select_channel(uint8_t subframeIndx,
    uint8_t pingPongId,
    uint8_t option0ping,
    uint8_t option1ping,
    uint8_t option0pong,
    uint8_t option1pong)
{
    uint8_t chId;
    if (pingPongId == 0)
    {
        if (subframeIndx == 0)
        {
            chId = option0ping;
        }
        else
        {
            chId = option1ping;
        }
    }
    else
    {
        if (subframeIndx == 0)
        {
            chId = option0pong;
        }
        else
        {
            chId = option1pong;
        }
    }
    return chId;
}

/**
*  @b Description
*  @n
*      Starts a DMA transfer on a specifed channel corresponding to a given subframe.
*
*/
void MmwDemo_startDmaTransfer(EDMA_Handle handle, uint8_t channelId0, uint8_t channelId1, uint8_t subframeIndx)
{
    if (subframeIndx == 0)
    {
        EDMA_startDmaTransfer(handle, channelId0);
    }
    else
    {
        EDMA_startDmaTransfer(handle, channelId1);
    }
}

/**
*  @b Description
*  @n
*      Resets the Doppler line bit mask. Doppler line bit mask indicates Doppler
*      lines (bins) on wich the CFAR in Doppler direction detected objects.
*      After the CFAR in Doppler direction is completed for all range bins, the
*      CFAR in range direction is performed on indicated Doppler lines.
*      The array dopplerLineMask is uint32_t array. The LSB bit of the first word
*      corresponds to Doppler line (bin) zero.
*
*/
void MmwDemo_resetDopplerLines(MmwDemo_1D_DopplerLines_t * ths)
{
    memset((void *)ths->dopplerLineMask, 0, ths->dopplerLineMaskLen * sizeof(uint32_t));
    ths->currentIndex = 0;
}

/**
*  @b Description
*  @n
*      Sets the bit in the Doppler line bit mask dopplerLineMask corresponding to Doppler
*      line on which CFAR in Doppler direction detected object. Indicating the Doppler
*      line being active in observed frame. @sa MmwDemo_resetDopplerLines
*/
void MmwDemo_setDopplerLine(MmwDemo_1D_DopplerLines_t * ths, uint16_t dopplerIndex)
{
    uint32_t word = dopplerIndex >> 5;
    uint32_t bit = dopplerIndex & 31;

    ths->dopplerLineMask[word] |= (0x1 << bit);
}

/**
*  @b Description
*  @n
*      Checks whether Doppler line is active in the observed frame. It checks whether the bit
*      is set in the Doppler line bit mask corresponding to Doppler
*      line on which CFAR in Doppler direction detected object.
*      @sa MmwDemo_resetDopplerLines
*/
uint32_t MmwDemo_isSetDopplerLine(MmwDemo_1D_DopplerLines_t * ths, uint16_t index)
{
    uint32_t dopplerLineStat;
    uint32_t word = index >> 5;
    uint32_t bit = index & 31;

    if (ths->dopplerLineMask[word] & (0x1 << bit))
    {
        dopplerLineStat = 1;
    }
    else
    {
        dopplerLineStat = 0;
    }
    return dopplerLineStat;
}

/**
*  @b Description
*  @n
*      Gets the Doppler index from the Doppler line bit mask, starting from the
*      smallest active Doppler lin (bin). Subsequent calls return the next
*      active Doppler line. @sa MmwDemo_resetDopplerLines
*
*/
int32_t MmwDemo_getDopplerLine(MmwDemo_1D_DopplerLines_t * ths)
{
    uint32_t index = ths->currentIndex;
    uint32_t word = index >> 5;
    uint32_t bit = index & 31;

    while (((ths->dopplerLineMask[word] >> bit) & 0x1) == 0)
    {
        index++;
        bit++;
        if (bit == 32)
        {
            word++;
            bit = 0;

        }
    }
    ths->currentIndex = index + 1;
    return index;
}


/**
*  @b Description
*  @n
*      Power of 2 round up function.
*/
uint32_t MmwDemo_pow2roundup(uint32_t x)
{
    uint32_t result = 1;
    while (x > result)
    {
        result <<= 1;
    }
    return result;
}



/**
*  @b Description
*  @n
*      Waits for 1D FFT data to be transferrd to input buffer.
*      This is a blocking function.
*
*  @param[in] obj  Pointer to data path object
*  @param[in] pingPongId ping-pong id (ping is 0 and pong is 1)
*  @param[in] subframeIndx
*
*  @retval
*      NONE
*/
void MmwDemo_dataPathWait1DInputData(MmwDemo_DSS_DataPathObj *obj, uint32_t pingPongId, uint32_t subframeIndx)
{
    /* wait until transfer done */
    volatile bool isTransferDone;
    uint8_t chId;
    if (pingPongId == 0)
    {
        if (subframeIndx == 0)
        {
            chId = MRR_SF0_EDMA_CH_1D_IN_PING;
        }
        else
        {
            chId = MRR_SF1_EDMA_CH_1D_IN_PING;
        }
    }
    else
    {

        if (subframeIndx == 0)
        {
            chId = MRR_SF0_EDMA_CH_1D_IN_PONG;
        }
        else
        {
            chId = MRR_SF1_EDMA_CH_1D_IN_PONG;
        }

    }
    do {
        if (EDMA_isTransferComplete(obj->edmaHandle[EDMA_INSTANCE_DSS],
            chId,
            (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            MmwDemo_dssAssert(0);
        }
    } while (isTransferDone == false);
}

/**
*  @b Description
*  @n
*      Waits for 1D FFT data to be transferred to output buffer.
*      This is a blocking function.
*
*  @param[in] obj  Pointer to data path object
*  @param[in] pingPongId ping-pong id (ping is 0 and pong is 1)
*  @param[in] subframeIndx
*
*  @retval
*      NONE
*/
void MmwDemo_dataPathWait1DOutputData(MmwDemo_DSS_DataPathObj *obj, uint32_t pingPongId, uint32_t subframeIndx)
{
    volatile bool isTransferDone;
    /* select the right EDMA channel based on the subframeIndx, and the pinPongId. */
    uint8_t chId = select_channel(subframeIndx, pingPongId, \
        MRR_SF0_EDMA_CH_1D_OUT_PING, MRR_SF1_EDMA_CH_1D_OUT_PING, \
        MRR_SF0_EDMA_CH_1D_OUT_PONG, MRR_SF1_EDMA_CH_1D_OUT_PONG);

    /* wait until transfer done */
    do {
        if (EDMA_isTransferComplete(obj->edmaHandle[EDMA_INSTANCE_DSS],
            chId,
            (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            MmwDemo_dssAssert(0);
        }
    } while (isTransferDone == false);
}

/**
*  @b Description
*  @n
*      Waits for 1D FFT data to be transferred to input buffer for 2D-FFT caclulation.
*      This is a blocking function.
*
*  @param[in] obj  Pointer to data path object
*  @param[in] pingPongId ping-pong id (ping is 0 and pong is 1)
*  @param[in] subframe Index
*
*  @retval
*      NONE
*/
void MmwDemo_dataPathWait2DInputData(MmwDemo_DSS_DataPathObj *obj, uint32_t pingPongId, uint32_t subframeIndx)
{
    volatile bool isTransferDone;

    /* select the right EDMA channel based on the subframeIndx, and the pinPongId. */
    uint8_t chId = select_channel(subframeIndx, pingPongId, \
        MRR_SF0_EDMA_CH_2D_IN_PING, MRR_SF1_EDMA_CH_2D_IN_PING, \
        MRR_SF0_EDMA_CH_2D_IN_PONG, MRR_SF1_EDMA_CH_2D_IN_PONG);

    /* wait until transfer done */
    do {
        if (EDMA_isTransferComplete(obj->edmaHandle[EDMA_INSTANCE_DSS],
            chId,
            (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            MmwDemo_dssAssert(0);
        }
    } while (isTransferDone == false);
}

/**
*  @b Description
*  @n
*      Waits for 1D FFT data to be transferred to input buffer for 3D-FFT calculation.
*      This is a blocking function.
*
*  @param[in] obj  Pointer to data path object
*  @param[in] pingPongId ping-pong id (ping is 0 and pong is 1)
*  @param[in] subframeIndx
*
*  @retval
*      NONE
*/
void MmwDemo_dataPathWait3DInputData(MmwDemo_DSS_DataPathObj *obj, uint32_t pingPongId, uint32_t subframeIndx)
{
    /* wait until transfer done */
    volatile bool isTransferDone;
    uint8_t chId = select_channel(subframeIndx, pingPongId, \
        MRR_SF0_EDMA_CH_3D_IN_PING, MRR_SF1_EDMA_CH_3D_IN_PING, \
        MRR_SF0_EDMA_CH_3D_IN_PONG, MRR_SF1_EDMA_CH_3D_IN_PONG);
    do
    {
        if (EDMA_isTransferComplete(obj->edmaHandle[EDMA_INSTANCE_DSS],
            chId,
            (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            MmwDemo_dssAssert(0);
        }
    } while (isTransferDone == false);
}

/**
*  @b Description
*  @n
*      Waits for 2D FFT calculated data to be transferred out from L2 memory
*      to detection matrix located in L3 memory.
*      This is a blocking function.
*
*  @param[in] obj  Pointer to data path object
*  @param[in] subframeIndx
*
*  @retval
*      NONE
*/
void MmwDemo_dataPathWaitTransDetMatrix(MmwDemo_DSS_DataPathObj *obj, uint8_t subframeIndx)
{
    volatile bool isTransferDone;
    uint8_t chId;
    if (subframeIndx == 0)
    {
        chId = MRR_SF0_EDMA_CH_DET_MATRIX;
    }
    else
    {
        chId = MRR_SF1_EDMA_CH_DET_MATRIX;
    }

    do
    {
        if (EDMA_isTransferComplete(obj->edmaHandle[EDMA_INSTANCE_DSS],
            (uint8_t)chId,
            (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            MmwDemo_dssAssert(0);
        }
    } while (isTransferDone == false);
}

/**
*  @b Description
*  @n
*      Waits for 2D FFT data to be transferred from detection matrix in L3
*      memory to L2 memory for CFAR detection in range direction.
*      This is a blocking function.
*
*  @param[in] obj  Pointer to data path object
*  @param[in] subframeIndx
*
*  @retval
*      NONE
*/
void MmwDemo_dataPathWaitTransDetMatrix2(MmwDemo_DSS_DataPathObj *obj, uint32_t subframeIndx)
{

    /* wait until transfer done */
    volatile bool isTransferDone;
    uint8_t chId;
    if (subframeIndx == 0)
    {
        chId = MRR_SF0_EDMA_CH_DET_MATRIX2;
    }
    else
    {
        chId = MRR_SF1_EDMA_CH_DET_MATRIX2;
    }

    do
    {
        if (EDMA_isTransferComplete(obj->edmaHandle[EDMA_INSTANCE_DSS],
            (uint8_t)chId,
            (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            MmwDemo_dssAssert(0);
        }
    } while (isTransferDone == false);
}

/**
*  @b Description
*  @n
*      Configures all EDMA channels and param sets used in data path processing
*
*  This function is very similar to the dataPathConfigEDMA from the OOB demo, but
*  with the difference that we have two subframes, and one subframe can support the
*  maximum velocity enhancement modification. In this method , the 2nd dimension has
*  two kinds of chirps and each chirp is repeated 'numDopplerBins' times, and each
*  chirp has the same  number of adc samples.
*
*  We would also like to ensure that when the data is transferred to
*  L3 RAM, a range gate (i.e. doppler bins corresponding to a range bin) of each
*  'chirptype' is contiguous, so that a single EDMA can pull them both out in
*  the 2nd dimension processing.
*
*  Hence the EDMAs corresponding to the transfer of 1D data to L3 and the transfer of data
*  from L3 to L2 are modified.
*
*  @param[in] obj  Pointer to data path object array.
*
*  @retval
*      -1 if error, 0 for no error
*/
int32_t MmwDemo_dataPathConfigEdma(MmwDemo_DSS_DataPathObj *obj)
{
    uint32_t eventQueue;
    uint16_t shadowParam = EDMA_NUM_DMA_CHANNELS;
    int32_t retVal = 0;
    uint8_t chId;
    uint8_t subframeIndx;
    uint32_t numChirpTypes = 1;
    uint32_t ADCBufferoffset = (32 * 1024)/4;

    for (subframeIndx = 0; subframeIndx < NUM_SUBFRAMES; subframeIndx++, obj++)
    {


        numChirpTypes = 1;
        if (obj->processingPath == MAX_VEL_ENH_PROCESSING)
        {
            numChirpTypes = 2;
        }

        /*****************************************************
        * EDMA configuration for getting ADC data from ADC buffer
        * to L2 (prior to 1D FFT)
        * For ADC Buffer to L2 use EDMA-A TPTC =1
        *****************************************************/
        eventQueue = 0U;

        /* Ping - copies chirp samples from even antenna numbers (e.g. RxAnt0 and RxAnt2) */

        if (subframeIndx == 0)
        {
            chId = MRR_SF0_EDMA_CH_1D_IN_PING;
        }
        else
        {
            chId = MRR_SF1_EDMA_CH_1D_IN_PING;
        }

        retVal =
            EDMAutil_configType1(obj->edmaHandle[EDMA_INSTANCE_DSS],
                (uint8_t *)(&obj->ADCdataBuf[0]),
                (uint8_t *)(SOC_translateAddress((uint32_t)&obj->adcDataIn[0], SOC_TranslateAddr_Dir_TO_EDMA, NULL)),
                chId,
                false,
                shadowParam++,
                obj->numAdcSamples * BYTES_PER_SAMP_1D,
                MAX(obj->numRxAntennas / 2, 1),
                ADCBufferoffset * 2,
                0,
                eventQueue,
                NULL,
                (uintptr_t)obj);
        if (retVal < 0)
        {
            return -1;
        }

        if (subframeIndx == 0)
        {
            chId = MRR_SF0_EDMA_CH_1D_IN_PONG;
        }
        else
        {
            chId = MRR_SF1_EDMA_CH_1D_IN_PONG;
        }

        /* Pong - copies chirp samples from odd antenna numbers (e.g. RxAnt1 and RxAnt3)
         * Note that ADCBufferoffset is in bytes, but ADCdataBuf is in cmplx16ReIm_t.
         * There are four bytes in one cmplx16ReIm_t*/
        retVal =
            EDMAutil_configType1(obj->edmaHandle[EDMA_INSTANCE_DSS],
                (uint8_t *)(&obj->ADCdataBuf[(ADCBufferoffset>>2)]),
                (uint8_t *)(SOC_translateAddress((uint32_t)(&obj->adcDataIn[obj->numRangeBins]), SOC_TranslateAddr_Dir_TO_EDMA, NULL)),
                chId,
                false,
                shadowParam++,
                obj->numAdcSamples * BYTES_PER_SAMP_1D,
                MAX(obj->numRxAntennas / 2, 1),
                ADCBufferoffset * 2,
                0,
                eventQueue,
                NULL,
                (uintptr_t)obj);
        if (retVal < 0)
        {
            return -1;
        }

        /* using different event queue between input and output to parallelize better */
        eventQueue = 1U;
        /*
        * EDMA configuration for storing 1d fft output in transposed manner to L3.
        * It copies all Rx antennas of the chirp per trigger event.
        */


        /* Ping - Copies from ping FFT output (even chirp indices)  to L3 */
        if (subframeIndx == 0)
        {
            chId = MRR_SF0_EDMA_CH_1D_OUT_PING;
        }
        else
        {
            chId = MRR_SF1_EDMA_CH_1D_OUT_PING;
        }


        retVal =
            EDMAutil_configType2a(obj->edmaHandle[EDMA_INSTANCE_DSS],
                (uint8_t *)(SOC_translateAddress((uint32_t)(&obj->fftOut1D[0]), SOC_TranslateAddr_Dir_TO_EDMA, NULL)),
                (uint8_t *)(&obj->radarCube[0]),
                chId,
                false,
                shadowParam++,
                BYTES_PER_SAMP_1D,
                obj->numRangeBins,
                obj->numTxAntennas * numChirpTypes,
                obj->numRxAntennas,
                obj->numDopplerBins,
                eventQueue,
                NULL,
                (uintptr_t)obj);
        if (retVal < 0)
        {
            return -1;
        }

        /* Ping - Copies from pong FFT output (odd chirp indices)  to L3 */
        if (subframeIndx == 0)
        {
            chId = MRR_SF0_EDMA_CH_1D_OUT_PONG;
        }
        else
        {
            chId = MRR_SF1_EDMA_CH_1D_OUT_PONG;
        }

        retVal =
            EDMAutil_configType2a(obj->edmaHandle[EDMA_INSTANCE_DSS],
                (uint8_t *)(SOC_translateAddress((uint32_t)(&obj->fftOut1D[obj->numRxAntennas * obj->numRangeBins]),
                    SOC_TranslateAddr_Dir_TO_EDMA, NULL)),
                (uint8_t *)(&obj->radarCube[0]),
                chId,
                false,
                shadowParam++,
                BYTES_PER_SAMP_1D,
                obj->numRangeBins,
                obj->numTxAntennas * numChirpTypes,
                obj->numRxAntennas,
                obj->numDopplerBins,
                eventQueue,
                NULL,
                (uintptr_t)obj);
        if (retVal < 0)
        {
            return -1;
        }


        /*****************************************
        * Interframe processing related EDMA configuration
        *****************************************/
        eventQueue = 0U;

        /* For the max-vel enh implementation, we pull out twice as much range-gates per range bin.
         * Hence  EDMA BCNT is multiplied by 2. */

        /* Ping: This DMA channel is programmed to fetch the 1D FFT data from radarCube
        * matrix in L3 mem of even antenna rows into the Ping Buffer in L2 mem*/
        if (subframeIndx == 0)
        {
            chId = MRR_SF0_EDMA_CH_2D_IN_PING;
        }
        else
        {
            chId = MRR_SF1_EDMA_CH_2D_IN_PING;
        }

        retVal =
            EDMAutil_configType1(obj->edmaHandle[EDMA_INSTANCE_DSS],
                (uint8_t *)(&obj->radarCube[0]),
                (uint8_t *)(SOC_translateAddress((uint32_t)(&obj->dstPingPong[0]), SOC_TranslateAddr_Dir_TO_EDMA, NULL)),
                chId,
                false,
                shadowParam++,
                obj->numDopplerBins * BYTES_PER_SAMP_1D,
                (obj->numRangeBins * obj->numRxAntennas * obj->numTxAntennas * numChirpTypes) / 2,
                obj->numDopplerBins * BYTES_PER_SAMP_1D * 2,
                0,
                eventQueue,
                NULL,
                (uintptr_t)obj);
        if (retVal < 0)
        {
            return -1;
        }

        /* Pong: This DMA channel is programmed to fetch the 1D FFT data from radarCube
        * matrix in L3 mem of odd antenna rows into thePong Buffer in L2 mem*/
        if (subframeIndx == 0)
        {
            chId = MRR_SF0_EDMA_CH_2D_IN_PONG;
        }
        else
        {
            chId = MRR_SF1_EDMA_CH_2D_IN_PONG;
        }

        retVal =
            EDMAutil_configType1(obj->edmaHandle[EDMA_INSTANCE_DSS],
                (uint8_t *)(&obj->radarCube[obj->numDopplerBins]),
                (uint8_t *)(SOC_translateAddress((uint32_t)(&obj->dstPingPong[obj->numDopplerBins]),
                    SOC_TranslateAddr_Dir_TO_EDMA, NULL)),
                chId,
                false,
                shadowParam++,
                obj->numDopplerBins * BYTES_PER_SAMP_1D,
                (obj->numRangeBins * obj->numRxAntennas * obj->numTxAntennas * numChirpTypes) / 2,
                obj->numDopplerBins * BYTES_PER_SAMP_1D * 2,
                0,
                eventQueue,
                NULL,
                (uintptr_t)obj);
        if (retVal < 0)
        {
            return -1;
        }


        /* This EDMA Channel brings selected range bins  from detection matrix in
        * L3 mem (reading in transposed manner) into L2 mem for CFAR detection (in
        * range direction). */
        if (subframeIndx == 0)
        {
            chId = MRR_SF0_EDMA_CH_DET_MATRIX2;
        }
        else
        {
            chId = MRR_SF1_EDMA_CH_DET_MATRIX2;
        }

        retVal =
            EDMAutil_configType3(obj->edmaHandle[EDMA_INSTANCE_DSS],
                (uint8_t *)0,
                (uint8_t *)0,
                chId,
                false,
                shadowParam++,
                BYTES_PER_SAMP_DET, \
                obj->numRangeBins,
                (obj->numDopplerBins * BYTES_PER_SAMP_DET),
                BYTES_PER_SAMP_DET,
                eventQueue,
                NULL,
                (uintptr_t)obj);
        if (retVal < 0)
        {
            return -1;
        }

        /*********************************************************
        * These EDMA Channels are for Azimuth calculation. They bring
        * 1D FFT data for 2D DFT and Azimuth FFT calculation.
        ********************************************************/
        /* Ping: This DMA channel is programmed to fetch the 1D FFT data from radarCube
        * matrix in L3 mem of even antenna rows into the Ping Buffer in L2 mem.
        */
        if (subframeIndx == 0)
        {
            chId = MRR_SF0_EDMA_CH_3D_IN_PING;
        }
        else
        {
            chId = MRR_SF1_EDMA_CH_3D_IN_PING;
        }

        retVal =
            EDMAutil_configType1(obj->edmaHandle[EDMA_INSTANCE_DSS],
                (uint8_t *)NULL,
                (uint8_t *)(SOC_translateAddress((uint32_t)(&obj->dstPingPong[0]), SOC_TranslateAddr_Dir_TO_EDMA, NULL)),
                chId,
                false,
                shadowParam++,
                obj->numDopplerBins * BYTES_PER_SAMP_1D,
                MAX((obj->numRxAntennas * obj->numTxAntennas) / 2, 1),
                (obj->numDopplerBins * BYTES_PER_SAMP_1D * 2),
                0,
                eventQueue,
                NULL,
                (uintptr_t)obj);
        if (retVal < 0)
        {
            return -1;
        }

        /* Pong: This DMA channel is programmed to fetch the 1D FFT data from radarCube
        * matrix in L3 mem of odd antenna rows into the Pong Buffer in L2 mem*/
        if (subframeIndx == 0)
        {
            chId = MRR_SF0_EDMA_CH_3D_IN_PONG;
        }
        else
        {
            chId = MRR_SF1_EDMA_CH_3D_IN_PONG;
        }

        retVal =
            EDMAutil_configType1(obj->edmaHandle[EDMA_INSTANCE_DSS],
                (uint8_t *)NULL,
                (uint8_t *)(SOC_translateAddress((uint32_t)(&obj->dstPingPong[obj->numDopplerBins]), SOC_TranslateAddr_Dir_TO_EDMA, NULL)),
                chId,
                false,
                shadowParam++,
                obj->numDopplerBins * BYTES_PER_SAMP_1D,
                MAX((obj->numRxAntennas * obj->numTxAntennas) / 2, 1),
                obj->numDopplerBins * BYTES_PER_SAMP_1D * 2,
                0,
                eventQueue,
                NULL,
                (uintptr_t)obj);
        if (retVal < 0)
        {
            return -1;
        }

    }

    return(0);
}

/**
*  @b Description
*  @n
*    This function populates the ObjOut based on the objRaw. It includes one more layer
*    of pruning which prevent objects beyond the maximum range or minimum range from
*    being populated.
*    Additionally we change the SNR requirement as a function of the range, requiring
*    larger SNR for objects closer to the car, and lower SNR for objects farther from
*    the car.
*  @param[out]   objOut             Output array of  detected objects after peak grouping
*  @param[in]    objRaw             Array of detected objects after CFAR detection
*  @param[in]    SNRThresh          A list of SNR thresholds for a list of ranges.
*  @param[in]    SNRThresh          A list of peakVal thresholds for a list of ranges.
*  @param[in]    numDetectedObjects Number of detected objects by CFAR
*  @param[in]    numDopplerBins     Number of Doppler bins
*  @param[in]    maxRange           Maximum range (in ONE_QFORMAT)
*  @param[in]    minRange           Minimum range (in ONE_QFORMAT)
*
*  @retval
*      Number of detected objects after grouping
*/
uint32_t rangeBasedPruning(
    MmwDemo_detectedObjActual*  restrict objOut,
    MmwDemo_objRaw2D_t * restrict objRaw,
    RangeDependantThresh_t * restrict SNRThresh,
    RangeDependantThresh_t * restrict peakValThresh,
    uint32_t numDetectedObjects,
    uint32_t numDopplerBins,
    uint32_t maxRange,
    uint32_t minRange)
{
    int32_t i, j, k;
    uint32_t numObjOut = 0;
    uint32_t searchSNRThresh = 0;
    uint32_t searchpeakValThresh = 0;
    j = 0;
    k = 0;
    /* No grouping, copy all detected objects to the output matrix within specified min max range
     * with the necessary SNR. */
    for (i = 0; i < numDetectedObjects; i++)
    {
        if ((objRaw[i].range <= maxRange) && ((objRaw[i].range >= minRange)))
        {
            /* We change the SNR requirement as a function of the range, requiring larger
             * SNR for objects closer to the car, and lower SNR for objects farther from
             * the car. */
            searchSNRThresh = 0;

            /* Check if  the range (of the target) lies between SNRThresh[j].rangelim and
             * SNRThresh[j-1].rangelim. If it doesn't search for a new SNR threshold. */
            if (objRaw[i].range > SNRThresh[j].rangelim)
            {
                searchSNRThresh = 1;
            }
            else if (j > 0)
            {
                if (objRaw[i].range < SNRThresh[j-1].rangelim)
                {
                    searchSNRThresh = 1;
                }
            }

            if (searchSNRThresh == 1)
            {
                /* MAX_NUM_SNR_THRESH_LIM is typically 3; A linear search should be fine */
                for (j = 0; j < MAX_NUM_RANGE_DEPENDANT_SNR_THRESHOLDS - 1; j++)
                {
                    if (objRaw[i].range < SNRThresh[j].rangelim)
                    {
                        break;
                    }
                }
            }

            /* Ditto for the peakValThresh. */
            searchpeakValThresh = 0;

            if (objRaw[i].range > peakValThresh[k].rangelim)
            {
                searchpeakValThresh = 1;
            }
            else if (k > 0)
            {
                if (objRaw[i].range < peakValThresh[k-1].rangelim)
                {
                    searchpeakValThresh = 1;
                }
            }

            if (searchpeakValThresh == 1)
            {
                for (k = 0; k < MAX_NUM_RANGE_DEPENDANT_SNR_THRESHOLDS - 1; k++)
                {
                    if (objRaw[i].range < peakValThresh[k].rangelim)
                    {
                        break;
                    }
                }
            }


            if ( (objRaw[i].rangeSNRdB > SNRThresh[j].threshold) &&
                 (objRaw[i].peakVal > peakValThresh[k].threshold) )
            {
                objOut[numObjOut].rangeIdx      = objRaw[i].rangeIdx;
                objOut[numObjOut].dopplerIdx    = objRaw[i].dopplerIdx;
                objOut[numObjOut].range         = objRaw[i].range;
                objOut[numObjOut].speed         = objRaw[i].speed;
                objOut[numObjOut].peakVal       = objRaw[i].peakVal;
                objOut[numObjOut].rangeSNRdB    = objRaw[i].rangeSNRdB;
                objOut[numObjOut].dopplerSNRdB  = objRaw[i].dopplerSNRdB;
                numObjOut++;

                if (numObjOut == MRR_MAX_OBJ_OUT)
                {
                    break;
                }
            }

        }
    }
    return numObjOut;
}

/**
*  @b Description
*  @n
*    Outputs magnitude squared float array of input complex32 array
*
*  @retval
*      Not Applicable.
*/
void MmwDemo_magnitudeSquared(cmplx32ReIm_t * restrict inpBuff, float * restrict magSqrdBuff, uint32_t numSamples)
{
    uint32_t i;
    for (i = 0; i < numSamples; i++)
    {
        magSqrdBuff[i] = (float)inpBuff[i].real * (float)inpBuff[i].real +
            (float)inpBuff[i].imag * (float)inpBuff[i].imag;
    }
}

#if 0
/**
*  @b Description
*  @n
*    Compensation of DC range antenna signature (unused currently)
*
*
*  @retval
*      Not Applicable.
*/
void MmwDemo_dcRangeSignatureCompensation(MmwDemo_DSS_DataPathObj *obj, uint8_t chirpPingPongId)
{
    uint32_t rxAntIdx, binIdx;
    uint32_t ind;
    int32_t chirpPingPongOffs;
    int32_t chirpPingPongSize;

    chirpPingPongSize = obj->numRxAntennas * (obj->calibDcRangeSigCfg.positiveBinIdx - obj->calibDcRangeSigCfg.negativeBinIdx + 1);
    if (obj->dcRangeSigCalibCntr == 0)
    {
        memset(obj->dcRangeSigMean, 0, obj->numTxAntennas * chirpPingPongSize * sizeof(cmplx32ImRe_t));
    }

    chirpPingPongOffs = chirpPingPongId * chirpPingPongSize;

    /* Calibration */
    if (obj->dcRangeSigCalibCntr < (obj->calibDcRangeSigCfg.numAvgChirps * obj->numTxAntennas))
    {
        /* Accumulate */
        ind = 0;
        for (rxAntIdx = 0; rxAntIdx < obj->numRxAntennas; rxAntIdx++)
        {
            uint32_t chirpInOffs = chirpPingPongId * (obj->numRxAntennas * obj->numRangeBins) + (obj->numRangeBins * rxAntIdx);
            int64_t *meanPtr = (int64_t *)&obj->dcRangeSigMean[chirpPingPongOffs];
            uint32_t *fftPtr = (uint32_t *)&obj->fftOut1D[chirpInOffs];
            int64_t meanBin;
            uint32_t fftBin;
            int32_t Re, Im;

            for (binIdx = 0; binIdx <= obj->calibDcRangeSigCfg.positiveBinIdx; binIdx++)
            {
                meanBin = _amem8(&meanPtr[ind]);
                fftBin = _amem4(&fftPtr[binIdx]);
                Im = _loll(meanBin) + _ext(fftBin, 0, 16);
                Re = _hill(meanBin) + _ext(fftBin, 16, 16);
                _amem8(&meanPtr[ind]) = _itoll(Re, Im);
                ind++;
            }

            chirpInOffs = chirpPingPongId * (obj->numRxAntennas * obj->numRangeBins) + (obj->numRangeBins * rxAntIdx) + obj->numRangeBins + obj->calibDcRangeSigCfg.negativeBinIdx;
            fftPtr = (uint32_t *)&obj->fftOut1D[chirpInOffs];
            for (binIdx = 0; binIdx < -obj->calibDcRangeSigCfg.negativeBinIdx; binIdx++)
            {
                meanBin = _amem8(&meanPtr[ind]);
                fftBin = _amem4(&fftPtr[binIdx]);
                Im = _loll(meanBin) + _ext(fftBin, 0, 16);
                Re = _hill(meanBin) + _ext(fftBin, 16, 16);
                _amem8(&meanPtr[ind]) = _itoll(Re, Im);
                ind++;
            }
        }
        obj->dcRangeSigCalibCntr++;

        if (obj->dcRangeSigCalibCntr == (obj->calibDcRangeSigCfg.numAvgChirps * obj->numTxAntennas))
        {
            /* Divide */
            int64_t *meanPtr = (int64_t *)obj->dcRangeSigMean;
            int32_t Re, Im;
            int64_t meanBin;
            int32_t divShift = obj->log2NumAvgChirps;
            for (ind = 0; ind < (obj->numTxAntennas * chirpPingPongSize); ind++)
            {
                meanBin = _amem8(&meanPtr[ind]);
                Im = _sshvr(_loll(meanBin), divShift);
                Re = _sshvr(_hill(meanBin), divShift);
                _amem8(&meanPtr[ind]) = _itoll(Re, Im);
            }
        }
    }
    else
    {
        /* fftOut1D -= dcRangeSigMean */
        ind = 0;
        for (rxAntIdx = 0; rxAntIdx < obj->numRxAntennas; rxAntIdx++)
        {
            uint32_t chirpInOffs = chirpPingPongId * (obj->numRxAntennas * obj->numRangeBins) + (obj->numRangeBins * rxAntIdx);
            int64_t *meanPtr = (int64_t *)&obj->dcRangeSigMean[chirpPingPongOffs];
            uint32_t *fftPtr = (uint32_t *)&obj->fftOut1D[chirpInOffs];
            int64_t meanBin;
            uint32_t fftBin;
            int32_t Re, Im;
            for (binIdx = 0; binIdx <= obj->calibDcRangeSigCfg.positiveBinIdx; binIdx++)
            {
                meanBin = _amem8(&meanPtr[ind]);
                fftBin = _amem4(&fftPtr[binIdx]);
                Im = _ext(fftBin, 0, 16) - _loll(meanBin);
                Re = _ext(fftBin, 16, 16) - _hill(meanBin);
                _amem4(&fftPtr[binIdx]) = _pack2(Im, Re);
                ind++;
            }

            chirpInOffs = chirpPingPongId * (obj->numRxAntennas * obj->numRangeBins) + (obj->numRangeBins * rxAntIdx) + obj->numRangeBins + obj->calibDcRangeSigCfg.negativeBinIdx;
            fftPtr = (uint32_t *)&obj->fftOut1D[chirpInOffs];
            for (binIdx = 0; binIdx < -obj->calibDcRangeSigCfg.negativeBinIdx; binIdx++)
            {
                meanBin = _amem8(&meanPtr[ind]);
                fftBin = _amem4(&fftPtr[binIdx]);
                Im = _ext(fftBin, 0, 16) - _loll(meanBin);
                Re = _ext(fftBin, 16, 16) - _hill(meanBin);
                _amem4(&fftPtr[binIdx]) = _pack2(Im, Re);
                ind++;
            }
        }
    }
}
#endif

/**
*  @b Description
*  @n
*    Interchirp processing. It is executed per chirp event, after ADC
*    buffer is filled with chirp samples.
*
*  @retval
*      Not Applicable.
*/
void MmwDemo_interChirpProcessing(MmwDemo_DSS_DataPathObj *obj, uint32_t chirpPingPongId, uint8_t subframeIndx)
{
    uint32_t antIndx, waitingTime;
    volatile uint32_t startTime;
    volatile uint32_t startTime1;

    waitingTime = 0;
    startTime = Cycleprofiler_getTimeStamp();

    /* Kick off DMA to fetch data from ADC buffer for first channel */
    MmwDemo_startDmaTransfer(obj->edmaHandle[EDMA_INSTANCE_DSS],
        MRR_SF0_EDMA_CH_1D_IN_PING,
        MRR_SF1_EDMA_CH_1D_IN_PING,
        subframeIndx);

    /* 1d fft for first antenna, followed by kicking off the DMA of fft output */
    for (antIndx = 0; antIndx < obj->numRxAntennas; antIndx++)
    {
        /* kick off DMA to fetch data for next antenna */
        if (antIndx < (obj->numRxAntennas - 1))
        {
            if (isPong(antIndx))
            {
                MmwDemo_startDmaTransfer(obj->edmaHandle[EDMA_INSTANCE_DSS],
                    MRR_SF0_EDMA_CH_1D_IN_PING,
                    MRR_SF1_EDMA_CH_1D_IN_PING,
                    subframeIndx);
            }
            else
            {
                MmwDemo_startDmaTransfer(obj->edmaHandle[EDMA_INSTANCE_DSS],
                    MRR_SF0_EDMA_CH_1D_IN_PONG,
                    MRR_SF1_EDMA_CH_1D_IN_PONG,
                    subframeIndx);
            }
        }

        /* verify if DMA has completed for current antenna */
        startTime1 = Cycleprofiler_getTimeStamp();
        MmwDemo_dataPathWait1DInputData(obj, pingPongId(antIndx), subframeIndx);
        waitingTime += Cycleprofiler_getTimeStamp() - startTime1;


        mmwavelib_windowing16x16(
            (int16_t *)&obj->adcDataIn[pingPongId(antIndx) * obj->numRangeBins],
            (int16_t *)obj->window1D,
            obj->numAdcSamples);
        memset((void *)&obj->adcDataIn[pingPongId(antIndx) * obj->numRangeBins + obj->numAdcSamples],
            0, (obj->numRangeBins - obj->numAdcSamples) * sizeof(cmplx16ReIm_t));


        DSP_fft16x16_imre(
            (int16_t *)obj->twiddle16x16_1D,
            obj->numRangeBins,
            (int16_t *)&obj->adcDataIn[pingPongId(antIndx) * obj->numRangeBins],
            (int16_t *)&obj->fftOut1D[chirpPingPongId * (obj->numRxAntennas * obj->numRangeBins) +
            (obj->numRangeBins * antIndx)]);
    }

    gCycleLog.interChirpProcessingTime += Cycleprofiler_getTimeStamp() - startTime - waitingTime;
    gCycleLog.interChirpWaitTime += waitingTime;
}

static uint32_t copyTranspose(uint32_t * src, uint32_t * dest, uint32_t size, int32_t offset, uint32_t stride, uint32_t pairs)
{
    int32_t i, j, k;
    j = 0;
    if (pairs == 1)
    {
        for(i = 0; i < (int32_t)size; i++)
        {
            dest[j+i*offset] = src[i];
            j += (int32_t)stride;
        }

    }
    else
    {
        for(i = 0; i < (int32_t)size; i++)
        {
            for (k = 0; k < (int32_t)pairs; k++)
            {
                dest[j+k+i*offset] = src[pairs * i + k];
            }
            j += (int32_t)stride;
        }
    }
    return(1);
}

static uint32_t copyBlock(uint32_t * src, uint32_t * dest, uint32_t size)
{
    int32_t i;
    for(i = 0; i < (int32_t)size; i++)
    {
        dest[i] = (int32_t)src[i];
    }
    return(1);
}
/**
*  @b Description
*  @n
*    Interframe processing. It is called from MmwDemo_dssDataPathProcessEvents
*    after all chirps of the frame have been received and 1D FFT processing on them
*    has been completed.
*
*  @retval
*      Not Applicable.
*/
void MmwDemo_interFrameProcessing(MmwDemo_DSS_DataPathObj *obj, uint8_t subframeIndx)
{
    uint32_t rangeIdx;
    volatile uint32_t startTime;
    volatile uint32_t startTime1;
    volatile uint32_t startTime2;
    volatile uint32_t startTimePoll, startTimePoll2;

    startTime = Cycleprofiler_getTimeStamp();

    for (rangeIdx = 0; rangeIdx < obj->numRangeBins; rangeIdx++)
    {
        copyBlock((uint32_t *)&obj->radarCube[rangeIdx * obj->numVirtualAntennas * obj->numDopplerBins], (uint32_t *)&obj->dstPingPong[0], obj->numDopplerBins * obj->numVirtualAntennas);

        radarDopplerProcessRun(obj->radarProcessHandle, (cplx16_t *) &obj->dstPingPong[0], (float *) obj->dopplerProcOut[PING]);

        copyTranspose((uint32_t *)obj->dopplerProcOut[PING], (uint32_t *)&(obj->radarProcConfig.heatMapMem[rangeIdx]), obj->numDopplerBins, 0, obj->numRangeBins, 1);
    }
    obj->cycleLog.dopplerProcCycles = Cycleprofiler_getTimeStamp() - startTime;

    radarFrameProcessRun(obj->radarProcessHandle, (void *) obj->outBuffCntxt);
    obj->cycleLog.cfarProcCycles = obj->radarProcConfig.benchmarkPtr->buffer[obj->radarProcConfig.benchmarkPtr->bufferIdx].cfarDetectionCycles;
    obj->cycleLog.doaProcCycles = obj->radarProcConfig.benchmarkPtr->buffer[obj->radarProcConfig.benchmarkPtr->bufferIdx].aoaCycles;

    populateOutputs(obj);
    //gCycleLog.interFrameProcessingTime += Cycleprofiler_getTimeStamp() - startTime - waitingTime;
    //gCycleLog.interFrameWaitTime += waitingTime;
}

/**
*  @b Description
*  @n
*    Chirp processing. It is called from MmwDemo_dssDataPathProcessEvents. It
*    is executed per chirp.
*
*    The range FFT output is transferred in a transpose manner to L3 using an EDMA. This is done
*    so that the 2nd FFT data can be pulled out using a non-transpose EDMA (which is more efficient)
*
*    The EDMA transfer requires a destination offset (radarCubeOffset) that is proportional with
*    the chirp number.
*
*    For the MAX_VEL_ENH chirp, there are two chirp types (fast and slow), they are
*    stored consecutively ( for e.g. chirp 1 of the fast chirp is directly followed by chirp 1
*    of the slow chirp.
*
*  @retval
*      Not Applicable.
*/
//uint32_t logRadarOffset[128];
//uint32_t idx =0;
//uint32_t logChirpcnt[128];
void MmwDemo_processChirp(MmwDemo_DSS_DataPathObj *obj, uint8_t subframeIndx)
{
    volatile uint32_t startTime;
    uint32_t radarCubeOffset;
    uint8_t chId;

    /** 1. Book keeping. */
    startTime = Cycleprofiler_getTimeStamp();

    if (obj->chirpCount > 1) //verify if ping(or pong) buffer is free for odd(or even) chirps
    {
        MmwDemo_dataPathWait1DOutputData(obj, pingPongId(obj->chirpCount), subframeIndx);
    }
    gCycleLog.interChirpWaitTime += Cycleprofiler_getTimeStamp() - startTime;

    /** 2.  Range processing. */
    MmwDemo_interChirpProcessing(obj, pingPongId(obj->chirpCount), subframeIndx);

    /* Modify destination address in Param set and DMA for sending 1DFFT output (for all antennas) to L3  */
    if (isPong(obj->chirpCount))
    {
        /* select the appropriate channel based on the index of the subframe. */
        if (subframeIndx == 0)
        {
            chId = MRR_SF0_EDMA_CH_1D_OUT_PONG;
        }
        else
        {
            chId = MRR_SF1_EDMA_CH_1D_OUT_PONG;
        }

        radarCubeOffset = (obj->numDopplerBins * obj->numRxAntennas * (obj->txAntennaCount))
                            + obj->dopplerBinCount
                            + (obj->numDopplerBins * obj->numRxAntennas * obj->numTxAntennas * obj->chirpTypeCount);
        EDMAutil_triggerType3(
            obj->edmaHandle[EDMA_INSTANCE_DSS],
            (uint8_t *)NULL,
            (uint8_t *)(&obj->radarCube[radarCubeOffset]),
            (uint8_t)chId,
            (uint8_t)MRR_EDMA_TRIGGER_ENABLE);
    }
    else
    {
        if (subframeIndx == 0)
        {
            chId = MRR_SF0_EDMA_CH_1D_OUT_PING;
        }
        else
        {
            chId = MRR_SF1_EDMA_CH_1D_OUT_PING;
        }
        radarCubeOffset = (obj->numDopplerBins * obj->numRxAntennas * (obj->txAntennaCount))
                            + obj->dopplerBinCount
                            + (obj->numDopplerBins * obj->numRxAntennas * obj->numTxAntennas * obj->chirpTypeCount);

        EDMAutil_triggerType3(
            obj->edmaHandle[EDMA_INSTANCE_DSS],
            (uint8_t *)NULL,
            (uint8_t *)(&obj->radarCube[radarCubeOffset]),
            (uint8_t)chId,
            (uint8_t)MRR_EDMA_TRIGGER_ENABLE);
    }

    //logRadarOffset[idx] = radarCubeOffset;
    //
    //logChirpcnt[idx] = obj->chirpCount;
    //idx++;


    obj->chirpCount++;
    obj->txAntennaCount++;
    if (obj->txAntennaCount == obj->numTxAntennas)
    {
        obj->txAntennaCount = 0;
        obj->dopplerBinCount++;
        if (obj->dopplerBinCount == obj->numDopplerBins)
        {
            if (obj->processingPath == MAX_VEL_ENH_PROCESSING)
            {
                obj->chirpTypeCount++;
                obj->dopplerBinCount = 0;
                if (obj->chirpTypeCount == SUBFRAME_MRR_NUM_CHIRPTYPES)
                {
                    obj->chirpTypeCount = 0;
                    obj->chirpCount = 0;
                }
            }
            else
            {
               obj->chirpTypeCount = 0;
               obj->dopplerBinCount = 0;
               obj->chirpCount = 0;
            }
        }
    }
}

/**
*  @b Description
*  @n
*  Wait for transfer of data corresponding to the last 2 chirps (ping/pong)
*  to the radarCube matrix before starting interframe processing.
*  @retval
*      Not Applicable.
*/
void MmwDemo_waitEndOfChirps(MmwDemo_DSS_DataPathObj *obj, uint8_t subframeIndx)
{
    volatile uint32_t startTime;

    startTime = Cycleprofiler_getTimeStamp();
    /* Wait for transfer of data corresponding to last 2 chirps (ping/pong) */
    MmwDemo_dataPathWait1DOutputData(obj, 0, subframeIndx);
    MmwDemo_dataPathWait1DOutputData(obj, 1, subframeIndx);

    gCycleLog.interChirpWaitTime += Cycleprofiler_getTimeStamp() - startTime;
}

/**
*  @b Description
*  @n
*  Generate SIN/COS table in Q15 (SIN to even int16 location, COS to
*  odd int16 location. Also generates Sin/Cos at half the bin value
*  The table is generated as
*  T[i]=cos[2*pi*i/N] - 1j*sin[2*pi*i/N] for i=0,...,N where N is dftLen
*  The half bn value is calculated as:
*  TH = cos[2*pi*0.5/N] - 1j*sin[2*pi*0.5/N]
*
*  @param[out]    dftSinCosTable Array with generated Sin Cos table
*  @param[out]    dftHalfBinVal  Sin/Cos value at half the bin
*  @param[in]     dftLen Length of the DFT
*
*  @retval
*      Not Applicable.
*/
void MmwDemo_genDftSinCosTable(cmplx16ImRe_t *dftSinCosTable,
    cmplx16ImRe_t *dftHalfBinVal,
    uint32_t dftLen)
{
    uint32_t i;
    int32_t itemp;
    float temp;
    for (i = 0; i < dftLen; i++)
    {
        temp = ONE_Q15 * -sin(2 * PI_*i / dftLen);
        itemp = (int32_t)ROUND(temp);

        if (itemp >= ONE_Q15)
        {
            itemp = ONE_Q15 - 1;
        }
        dftSinCosTable[i].imag = itemp;

        temp = ONE_Q15 * cos(2 * PI_*i / dftLen);
        itemp = (int32_t)ROUND(temp);

        if (itemp >= ONE_Q15)
        {
            itemp = ONE_Q15 - 1;
        }
        dftSinCosTable[i].real = itemp;
    }

    /*Calculate half bin value*/
    temp = ONE_Q15 * -sin(PI_ / dftLen);
    itemp = (int32_t)ROUND(temp);

    if (itemp >= ONE_Q15)
    {
        itemp = ONE_Q15 - 1;
    }
    dftHalfBinVal[0].imag = itemp;

    temp = ONE_Q15 * cos(PI_ / dftLen);
    itemp = (int32_t)ROUND(temp);

    if (itemp >= ONE_Q15)
    {
        itemp = ONE_Q15 - 1;
    }
    dftHalfBinVal[0].real = itemp;
}


/**
*  @b Description
*  @n
*   This is a callback function for EDMA  errors.
*
*  @param[in] handle EDMA Handle.
*  @param[in] errorInfo EDMA error info.
*
*  @retval n/a.
*/
void MmwDemo_edmaErrorCallbackFxn(EDMA_Handle handle, EDMA_errorInfo_t *errorInfo)
{
    MmwDemo_dssAssert(0);
}

/**
*  @b Description
*  @n
*   This is a callback function for EDMA TC errors.
*
*  @param[in] handle EDMA Handle.
*  @param[in] errorInfo EDMA TC error info.
*
*  @retval n/a.
*/
void MmwDemo_edmaTransferControllerErrorCallbackFxn(EDMA_Handle handle,
    EDMA_transferControllerErrorInfo_t *errorInfo)
{
    MmwDemo_DSS_DataPathObj * dataPathObj;
    /* Copy the error into the output structure (for debug) */
    dataPathObj = &gMrrDSSMCB.dataPathObj[gMrrDSSMCB.subframeIndx];
    dataPathObj->EDMA_transferControllerErrorInfo = *errorInfo;
    MmwDemo_dssAssert(0);
}


/**
*  @b Description
*  @n
*   This function initializes some of the states (counters) used for 1D processing.
*
*  @param[in,out] obj             data path object.
*
*  @retval success/failure.
*/
void MmwDemo_dataPathInit1Dstate(MmwDemo_DSS_DataPathObj *obj)
{
    int8_t subframeIndx = 0;

    for (subframeIndx = 0; subframeIndx < NUM_SUBFRAMES; subframeIndx++, obj++)
    {
        obj->chirpCount = 0;
        obj->dopplerBinCount = 0;
        obj->txAntennaCount = 0;
        obj->chirpTypeCount = 0;
    }

    /* reset profiling logs before start of frame */
    memset((void *)&gCycleLog, 0, sizeof(cycleLog_t));
}

/**
*  @b Description
*  @n
*   This function copies the EDMA handles to all of the remaining data path objects.
*
*  @param[in,out] obj             data path object.
*
*  @retval success/failure.
*/
int32_t MmwDemo_dataPathInitEdma(MmwDemo_DSS_DataPathObj *obj)
{
    uint8_t numInstances;
    int32_t errorCode;
    EDMA_Handle handle;
    EDMA_errorConfig_t errorConfig;
    uint32_t instanceId;
    EDMA_instanceInfo_t instanceInfo;

    numInstances = EDMA_getNumInstances();

    /* Initialize the edma instance to be tested */
    for (instanceId = 0; instanceId < numInstances; instanceId++)
    {
        EDMA_init(instanceId);

        handle = EDMA_open(instanceId, &errorCode, &instanceInfo);
        if (handle == NULL)
        {
            // System_printf("Error: Unable to open the edma Instance, erorCode = %d\n", errorCode);
            return -1;
        }
        obj->edmaHandle[instanceId] = handle;

        errorConfig.isConfigAllEventQueues = true;
        errorConfig.isConfigAllTransferControllers = true;
        errorConfig.isEventQueueThresholdingEnabled = true;
        errorConfig.eventQueueThreshold = EDMA_EVENT_QUEUE_THRESHOLD_MAX;
        errorConfig.isEnableAllTransferControllerErrors = true;
        errorConfig.callbackFxn = MmwDemo_edmaErrorCallbackFxn;
        errorConfig.transferControllerCallbackFxn = MmwDemo_edmaTransferControllerErrorCallbackFxn;
        if ((errorCode = EDMA_configErrorMonitoring(handle, &errorConfig)) != EDMA_NO_ERROR)
        {
            // System_printf("Debug: EDMA_configErrorMonitoring() failed with errorCode = %d\n", errorCode);
            return -1;
        }
    }
    return 0;
}

/**
*  @b Description
*  @n
*   This function copies the EDMA handles to all of the remaining data path objects.
*
*  @param[in,out] obj             data path object.
*
*  @retval success.
*/
int32_t MmwDemo_dataPathCopyEdmaHandle(MmwDemo_DSS_DataPathObj *objOutput, MmwDemo_DSS_DataPathObj *objInput)
{
    uint8_t numInstances;
    uint32_t instanceId;

    numInstances = EDMA_getNumInstances();

    /* Initialize the edma instance to be tested */
    for (instanceId = 0; instanceId < numInstances; instanceId++)
    {
        objOutput->edmaHandle[instanceId] = objInput->edmaHandle[instanceId];
    }
    return 0;
}

/**
*  @b Description
*  @n
*   This function holds the last remaining 'printf' in the entire MRR, and prints the space
*   remaining in the global heap.
*
*  @param[in,out] obj             data path object.
*
*  @retval na.
*/
//void MmwDemo_printHeapStats(char *name, uint32_t heapUsed, uint32_t heapSize)
//{
//    System_printf("Heap %s : size %d (0x%x), free %d (0x%x)\n", name, heapSize, heapSize, heapSize - heapUsed, heapSize - heapUsed);
//}

void MmwDemo_printHeapStats()
{
    System_printf("DDR Heap : size %d (0x%x), used %d (0x%x)\n",
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapSize,
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAllocOffset, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAllocOffset);
    System_printf("LL2 Heap : size %d (0x%x), used %d (0x%x)\n",
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapSize,
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapAllocOffset, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].heapAllocOffset);
    System_printf("LL2 Scratch : size %d (0x%x), used %d (0x%x)\n",
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].scratchSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].scratchSize,
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].maxScratchSizeUsed, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL2].maxScratchSizeUsed);
    System_printf("LL1 Scratch : size %d (0x%x), used %d (0x%x)\n",
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].scratchSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].scratchSize,
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].maxScratchSizeUsed, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_LL1].maxScratchSizeUsed);
    System_printf("HSRAM Heap : size %d (0x%x), used %d (0x%x)\n",
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].heapSize, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].heapSize,
            gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].heapAllocOffset, gRadarOsal_heapObj[RADARMEMOSAL_HEAPTYPE_HSRAM].heapAllocOffset);


}

/**
*  @b Description
*  @n
*   This function assigns memory locations to the different data buffers used in the MRR design.
*
*   Processing radar signals require a large number of scratch buffers for each step each
*   of the processing stages (be it 1D-FFT, 2D-FFT, 3D-FFT, detection, angle estimation etc.
*   However, since these stages occur serially, the memory assigned to a scratch buffer
*   used in a previous stage can be re-used in the current stage. The Macro MMW_ALLOC_BUF
*   in the following code allows specifying the start addresses such that the memory
*   locations can be overlaid for efficient memory utilization.
*
*   In the MRR TI Design, there are two sub-frames per frame, and both sub-frames are processed
*   separately. Therefore, nearly every scratch buffer memory location can be overlaid
*   between the two. The allocation code is called twice to allocate memory for both
*   sub-frames.
*
*   Certain memory locations are only necessary for a given processing path and are left
*   unassigned for different programming paths.
*
*   Memory locations that correspond to the windowing functions, and  twiddle factors, and
*   estimated mean chirp need to be saved between sub-frames and as such cannot be overlaid.
*
*  @param[in,out] obj             data path object.
*
*  @retval na.
*/
#define SOC_MAX_NUM_RX_ANTENNAS 4
#define SOC_MAX_NUM_TX_ANTENNAS 3
void MmwDemo_dataPathConfigBuffers(MmwDemo_DSS_DataPathObj *dataPathObj, uint32_t adcBufAddress)
{

    volatile MmwDemo_DSS_DataPathObj *obj = &dataPathObj[0];

    uint32_t subframeIndx, maxNum1, maxNum2, maxNum3;
    uint32_t numChirpTypes = 1;
    radarOsal_heapConfig heapconfig[RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS];
    ProcessErrorCodes procErrorCode;
    obj->ADCdataBuf = (cmplx16ImRe_t *)adcBufAddress;

    // heap init
    memset(heapconfig, 0, sizeof(heapconfig));
    heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapType = RADARMEMOSAL_HEAPTYPE_DDR_CACHED;
    heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAddr   = (int8_t *) &gMmwL3[0];
    heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapSize   = SOC_L3RAM_SIZE;
    heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchAddr   = NULL;  /* not DDR scratch for SRR demo  */
    heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchSize   = 0;     /* not DDR scratch for SRR demo  */

    heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapType = RADARMEMOSAL_HEAPTYPE_LL2;
    heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapAddr   = (int8_t *) &gMmwL2[0];
    heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapSize   = SOC_XWR18XX_DSS_L2_BUFF_SIZE; //MMW_L2_HEAP_SIZE;
    heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].scratchAddr   = (int8_t *)&gMmwL2Scratch[0];;
    heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].scratchSize   = SOC_XWR18XX_DSS_L2_SCRATCH_SIZE;

    heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapType = RADARMEMOSAL_HEAPTYPE_LL1;
    heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapAddr   = NULL; /* not used as L1 heap in SRR demo  */
    heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapSize   = 0;    /* not used as L1 heap in SRR demo  */
    heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].scratchAddr   = (int8_t *) &gMmwL1[0];
    heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].scratchSize   = MMW_L1_HEAP_SIZE;

    heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].heapType = RADARMEMOSAL_HEAPTYPE_HSRAM;
    heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].heapAddr   = NULL;
    heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].heapSize   = 0;
    heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].scratchAddr   = NULL;   /* not HSRAM scratch for SRR demo  */
    heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].scratchSize   = 0;    /* not HSRAM scratch for SRR demo  */

    if(radarOsal_memInit(&heapconfig[0], RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS) == RADARMEMOSAL_FAIL)
    {
        System_printf("Error: radarOsal_memInit fail\n");
    }

    /*initializing configuration structure for signal processing chain */
    //MmwDemo_initConfigStruct(demoCfg, &dataPathObj->radarProcConfig);
    obj = &dataPathObj[0];
    maxNum1 = 0;
    maxNum2 = 0;
    maxNum3 = 0;
    for (subframeIndx = 0; subframeIndx < NUM_SUBFRAMES; subframeIndx++, obj++)
    {
        if (obj->processingPath == MAX_VEL_ENH_PROCESSING)
            numChirpTypes = 2;
        else
            numChirpTypes = 1;

        maxNum1 = MAX(obj->numRangeBins, maxNum1);
        maxNum2 = MAX(obj->numRxAntennas * obj->numRangeBins, maxNum2);
        maxNum3 = MAX(obj->numRangeBins * obj->numDopplerBins * obj->numRxAntennas * obj->numTxAntennas * numChirpTypes, maxNum3);

        obj->twiddle16x16_1D = (cmplx16ImRe_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, obj->numRangeBins*sizeof(cmplx16ImRe_t), 8);
        obj->window1D = (int16_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, (obj->numAdcSamples >> 1) * sizeof(int16_t), 8);
    }
    dataPathObj->radarCube = (cmplx16ImRe_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, maxNum3*sizeof(cmplx16ImRe_t), 8);
    dataPathObj->radarProcConfig.pFFT1DBuffer = (cplx16_t *)dataPathObj->radarCube;
    dataPathObj->fftOut1D = (cmplx16ImRe_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, 2 * maxNum2*sizeof(cmplx16ImRe_t), 8);
    dataPathObj->dopplerProcOut[PING] = (float *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(float)*dataPathObj->numDopplerBins, 8);
    dataPathObj->dopplerProcOut[PONG] = (float *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(float)*dataPathObj->numDopplerBins, 8);

#ifndef OGMAP_INTERFACE
    dataPathObj->detObjFinal = (MmwDemo_detectedObjForTx *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(MmwDemo_detectedObjForTx)*DOA_OUTPUT_MAXPOINTS, 8);
    dataPathObj->dbscanReportFinal = (radar_dbscanReportForGui *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(radar_dbscanReportForGui)*CLUSTERING_MAX_CLUSTERS, 8);

#else
    dataPathObj->outputForOGMAP = (radarProcessOutput_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(radarProcessOutput_t), 8);
#endif
    dataPathObj->outBuffCntxt = (RadarDsp_outputBuffCntxt *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RadarDsp_outputBuffCntxt), 8);
    dataPathObj->outBuffCntxt->numBuff = RadarDsp_outputDataType_MAX;
    dataPathObj->outBuffCntxt->elem[RadarDsp_outputDataType_OBJ_DATA].type = RadarDsp_outputDataType_OBJ_DATA;
    dataPathObj->outBuffCntxt->elem[RadarDsp_outputDataType_OBJ_DATA].buff = (radarProcessOutputToTracker *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(radarProcessOutputToTracker), 8);

    dataPathObj->scratchBuf = (int32_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 1, MMW_SRR_DEMO_EDMASCATCHBUF_SIZE, 8); //fixed to 16k byte
    dataPathObj->adcDataIn =  (cmplx16ImRe_t *)&dataPathObj->scratchBuf[0];

    dataPathObj->dstPingPong  = (cmplx16ImRe_t *)&dataPathObj->scratchBuf[2 * dataPathObj->radarProcConfig.fftSize1D];
    //dataPathObj->dopplerProcOut[PING] = (float *)&dataPathObj->scratchBuf[2 * dataPathObj->radarProcConfig.fftSize1D + 2 * dataPathObj->radarProcConfig.numAntenna * dataPathObj->radarProcConfig.fftSize2D];
    //dataPathObj->dopplerProcOut[PONG] = (float *)&dataPathObj->scratchBuf[2 * dataPathObj->radarProcConfig.fftSize1D + 2 * dataPathObj->radarProcConfig.numAntenna * dataPathObj->radarProcConfig.fftSize2D + dataPathObj->radarProcConfig.fftSize2D];


    dataPathObj->radarProcessHandle = (void *) radarProcessCreate(&dataPathObj->radarProcConfig, &procErrorCode);
    if (procErrorCode > PROCESS_OK)
    {
        System_printf("radar process create error! Exit!");
    }


    //dataPathObj->fftOut1D  = (cmplx16ImRe_t *)dataPathObj->radarProcConfig.pFFT1DBuffer;
    //dataPathObj->numAdcSamples  = (uint32_t)dataPathObj->radarProcConfig.numAdcSamplePerChirp;
    //dataPathObj->numRangeBins  = (uint32_t)dataPathObj->radarProcConfig.fftSize1D;
    //dataPathObj->numDopplerBins  = (uint32_t)dataPathObj->radarProcConfig.fftSize2D;
    //dataPathObj->chirpThreshold  = 1;

    MmwDemo_printHeapStats();
    radarOsal_memDeInit();
}


/**
*  @b Description
*  @n
*      Function to populate the twiddle factors for FFTS needed for the data path object.
*
*  @param[in,out] obj             data path object.
*
*  @retval waitingTime.
*/
void MmwDemo_dataPathConfigFFTs(MmwDemo_DSS_DataPathObj *obj)
{
    MmwDemo_genWindow((void *)obj->window1D,
        FFT_WINDOW_INT16,
        obj->numAdcSamples,
        obj->numAdcSamples / 2,
        ONE_Q15,
        MMW_WIN_HANNING);

/*    MmwDemo_genWindow((void *)obj->window2D,
        FFT_WINDOW_INT32,
        obj->numDopplerBins,
        obj->numDopplerBins / 2,
        ONE_Q19,
        MMW_WIN_HANNING);
*/
    /* Generate twiddle factors for 1D FFT. */
    gen_twiddle_fft16x16_imre((int16_t *)obj->twiddle16x16_1D, obj->numRangeBins);

    /* Generate twiddle factors for 2D FFT */
    //gen_twiddle_fft32x32((int32_t *)obj->twiddle32x32_2D, obj->numDopplerBins, 2147483647.5);

    /* Generate twiddle factors for azimuth FFT */
    //gen_twiddle_fft32x32((int32_t *)obj->azimuthTwiddle32x32, obj->numAngleBins, 2147483647.5);

    /* Generate SIN/COS table for single point DFT */
    //MmwDemo_genDftSinCosTable(obj->azimuthModCoefs,
    //    &obj->azimuthModCoefsHalfBin,
    //    obj->numDopplerBins);
}

/**
*  @b Description
*  @n
*      Function to generate a single FFT window sample.
*
*  @param[out] win             Pointer to calculated window samples.
*  @param[in]  windowDatumType Window samples data format. For windowDatumType = @ref FFT_WINDOW_INT16,
*              the samples format is int16_t. For windowDatumType = @ref FFT_WINDOW_INT32,
*              the samples format is int32_t.
*  @param[in]  winLen          Nominal window length
*  @param[in]  winGenLen       Number of generated samples
*  @param[in]  oneQformat      Q format of samples, oneQformat is the value of
*                              one in the desired format.
*  @param[in]  winType         Type of window, one of @ref MMW_WIN_BLACKMAN, @ref MMW_WIN_HANNING,
*              or @ref MMW_WIN_RECT.
*  @retval none.
*/
void MmwDemo_genWindow(void *win,
    uint32_t windowDatumType,
    uint32_t winLen,
    uint32_t winGenLen,
    int32_t oneQformat,
    uint32_t winType)
{
    uint32_t winIndx;
    int32_t winVal;
    int16_t * win16 = (int16_t *)win;
    int32_t * win32 = (int32_t *)win;

    float phi = 2 * PI_ / ((float)winLen - 1);

    if (winType == MMW_WIN_BLACKMAN)
    {
        //Blackman window
        float a0 = 0.42;
        float a1 = 0.5;
        float a2 = 0.08;
        for (winIndx = 0; winIndx < winGenLen; winIndx++)
        {
            winVal = (int32_t)((oneQformat * (a0 - a1*cos(phi * winIndx) +
                a2*cos(2 * phi * winIndx))) + 0.5);
            if (winVal >= oneQformat)
            {
                winVal = oneQformat - 1;
            }
            switch (windowDatumType)
            {
            case FFT_WINDOW_INT16:
                win16[winIndx] = (int16_t)winVal;
                break;
            case FFT_WINDOW_INT32:
                win32[winIndx] = (int32_t)winVal;
                break;
            }

        }
    }
    else if (winType == MMW_WIN_HANNING)
    {
        //Hanning window
        for (winIndx = 0; winIndx < winGenLen; winIndx++)
        {
            winVal = (int32_t)((oneQformat * 0.5* (1 - cos(phi * winIndx))) + 0.5);

            if (winVal >= oneQformat)
            {
                winVal = oneQformat - 1;
            }

            switch (windowDatumType)
            {
            case FFT_WINDOW_INT16:
                win16[winIndx] = (int16_t)winVal;
                break;
            case FFT_WINDOW_INT32:
                win32[winIndx] = (int32_t)winVal;
                break;
            }
        }
    }
    else if (winType == MMW_WIN_RECT)
    {
        //Rectangular window
        for (winIndx = 0; winIndx < winGenLen; winIndx++)
        {
            switch (windowDatumType)
            {
            case FFT_WINDOW_INT16:
                win16[winIndx] = (int16_t)(oneQformat - 1);
                break;
            case FFT_WINDOW_INT32:
                win32[winIndx] = (int32_t)(oneQformat - 1);
                break;
            }
        }
    }
}










/**
*  @b Description
*  @n
*  The MRR subframe achieves a maximum unambiguous velocity of 90kmph
*  by using signal processing techniques that help disambiguate
*  velocity.  This method works by using two different estimates of
*  velocity from the two kinds of chirps (‘fast chirps’ and ‘slow
*  chirps’) transmitted in the MRR subframe. If the two velocity estimates
*  do not agree, then velocity disambiguation is necessary. To
*  disambiguate it is necessary to rationalize the two velocity
*  measurements, and find out the disambiguation factor, k.  If the
*  naive maximum unambiguous velocity of the 'fast chirp' is v_f, and
*  that of the 'slow chirp' is v_s. Then after the disambiguation process,
*  the disambiguated velocity would  〖2kv〗_f+v, where v is the naïve
*  estimated velocity from the ‘fast chirps’.
*
*  The disambiguation process works by using the 'fast chirp' velocity to
*  compute different disambiguated velocity hypotheses. This is done by
*  taking the 'fast chirp' velocity and adding 2k v_f, where k ∈ {-1,0,1}
*  (an unwrapping process on the velocity estimate). These hypotheses are
*  then converted to indices of the 'slow chirp' by finding the equivalent
*  estimated velocities in the 'slow chirp' configuration ( essentially,
*  undoing the unwrapping using v_s as the maximum unambiguous velocity).
*
*  If the index corresponding to one of the hypotheses has significant
*  energy, then that hypothesis is considered to be valid. Disambiguation of
*  up to 3x of the naive max-velocity is possible with this method, however,
*  testing has only been done up to 90 kmph.
*
*  @param[in,out] sumAbs    The slow chirps doppler bins at a certain range.
*  @param[in] fastChirpVel    The velocity estimate using the fast chirps (pre-disambiguation).
*  @param[in] fastChirpPeakVal    The peak value of the index of the detected
*                                object from the fast chirp.
*  @param[in] obj   data path object.
*
*
*  @retval
*    Ambiguity index.
*/
#define N_HYPOTHESIS ((2 * (MAX_VEL_ENH_NUM_NYQUIST - 1)) + 1)
int16_t disambiguateVel(uint16_t * restrict sumAbs, float fastChirpVel, uint16_t fastChirpPeakVal, MmwDemo_DSS_DataPathObj * obj)
{
    int16_t peakIndx, velIndx, velIndxTmp;
    int32_t AmbIndx, spreadIndx, prevIndx, nextIndx, numMult, peakVal, diff;
    float fastChirpAmbVel, AmbIndxActual, velIndxFlt;
    uint16_t slowChirpPeakArr[N_HYPOTHESIS];
    int16_t validArr[N_HYPOTHESIS];
    uint16_t thresh = obj->maxVelEnhStruct.maxVelAssocThresh;
    uint16_t threshActual = obj->maxVelEnhStruct.maxVelAssocThresh;

    /* From the fast Chirp's estimated target velocity, create a list of ambiguous velocities.
    * fastChirpAmbVel` is the ambiguous velocity for current object. */
    for (AmbIndx = 0; AmbIndx < N_HYPOTHESIS; AmbIndx++)
    {
        if (AmbIndx == 1)
        {
            threshActual = thresh;
        }
        else
        {   /* At higher velocities allow for more variation between the processed results of the
             * 'fast chirp', and the 'slow chirp' due to range migration. */
            threshActual = thresh*2;
        }

        /* Initialize the result for this ambiguous velocity to an invalid state.*/
        validArr[AmbIndx] = -1;

        /* Construct the velocity hypothesis. */
        AmbIndxActual = (float)(AmbIndx - (int32_t)(MAX_VEL_ENH_NUM_NYQUIST - 1));
        fastChirpAmbVel = (AmbIndxActual* (2.0f * obj->maxUnambiguousVel)) + fastChirpVel;

        /* convert the ambiguous velocities to the slow chirp's indices. */
        velIndxFlt = (fastChirpAmbVel * ((float)obj->maxVelEnhStruct.invVelResolutionSlowChirp));

        /* Make sure that the indices lie within the slowChirp limits.
         * Also, perform a flooring operation. */
        if (velIndxFlt < 0)
        {
            velIndxTmp = (int16_t)(-velIndxFlt);

            numMult = (velIndxTmp) >> LOG2_APPROX(SUBFRAME_MRR_CHIRPTYPE_1_NUM_CHIRPS);
            velIndxFlt += (float)(SUBFRAME_MRR_CHIRPTYPE_1_NUM_CHIRPS*(numMult + 1));
            velIndx = (int16_t)velIndxFlt;
        }
        else if (velIndxFlt >= SUBFRAME_MRR_CHIRPTYPE_1_NUM_CHIRPS)
        {
            velIndxTmp = (int16_t)(velIndxFlt);

            numMult = (velIndxTmp) >> LOG2_APPROX(SUBFRAME_MRR_CHIRPTYPE_1_NUM_CHIRPS);
            velIndxFlt -= (float)(SUBFRAME_MRR_CHIRPTYPE_1_NUM_CHIRPS*numMult);
            velIndx = (int16_t)velIndxFlt;
        }
        else
        {
            velIndx = (int16_t)velIndxFlt;
        }

        /* Inorder to compensate for the effect of flooring, we check for both the ceil and floor of
         * slowChirpArr. i.e. we assume that either velIndx (the floor) or velIndx + 1 (the ceil),
         * could be a peak in the slowChirp's doppler array. */
        for (spreadIndx = 0; spreadIndx < MAX_VEL_IMPROVEMENT_NUM_SPREAD; spreadIndx++)
        {
            velIndxTmp = velIndx + spreadIndx;

            if (velIndxTmp > SUBFRAME_MRR_CHIRPTYPE_1_NUM_CHIRPS - 1)
            {
                velIndxTmp = velIndxTmp - SUBFRAME_MRR_CHIRPTYPE_1_NUM_CHIRPS;
            }

            /* Is sumAbs[velIndxTmp] (or sumAbs[velIndxTmp+1]) a peak? */
            if (velIndxTmp == 0)
            {
                prevIndx = SUBFRAME_MRR_CHIRPTYPE_1_NUM_CHIRPS - 1;
            }
            else
            {
                prevIndx = velIndxTmp - 1;
            }

            if (velIndxTmp == (SUBFRAME_MRR_CHIRPTYPE_1_NUM_CHIRPS - 1))
            {
                nextIndx = 0;
            }
            else
            {
                nextIndx = velIndxTmp + 1;
            }

            if ((sumAbs[velIndxTmp] <= sumAbs[nextIndx]) ||
                (sumAbs[velIndxTmp] <= sumAbs[prevIndx]))
            {
                continue;
            }

            if ((sumAbs[nextIndx] == 0) ||
                (sumAbs[prevIndx] == 0))
            {
                continue;
            }

            /* Is the amplitude of the peaks of subframe1, and subframe2 within a certain dB of each other? */
            diff = _abs(((int32_t)sumAbs[velIndxTmp]) - ((int32_t)fastChirpPeakVal));

            if (diff > threshActual)
            {
                continue;
            }

            /* Our tests have passed, mark this as one of the possible options. */
            slowChirpPeakArr[AmbIndx] = sumAbs[velIndxTmp];
            validArr[AmbIndx] = velIndx;

            break;
        }

      }

    /* We now have a list of peak values indexed by the ambiguous velocity hypotheses.
    * Select the strongest one.*/
    peakIndx = -1;
    peakVal = 0;

    for (AmbIndx = 0; AmbIndx < (2 * (MAX_VEL_ENH_NUM_NYQUIST - 1)) + 1; AmbIndx++)
    {
        if (validArr[AmbIndx] >= 0)
        {
            if (slowChirpPeakArr[AmbIndx] > peakVal)
            {
                peakVal = slowChirpPeakArr[AmbIndx];
                peakIndx = AmbIndx;
            }
        }
    }

    if (peakIndx != -1)
    {
        velIndx = validArr[peakIndx];
        /* Since we have an association, zero out those velocity indices in the current sumAbs from
         * from being used again (in subsequent max-vel enhancement associations). */
        for (spreadIndx = 0; spreadIndx < MAX_VEL_IMPROVEMENT_NUM_SPREAD; spreadIndx++)
        {
            velIndxTmp = velIndx + spreadIndx;
            if (velIndxTmp > SUBFRAME_MRR_CHIRPTYPE_1_NUM_CHIRPS - 1)
            {
                velIndxTmp = velIndxTmp - SUBFRAME_MRR_CHIRPTYPE_1_NUM_CHIRPS;
            }

            sumAbs[velIndxTmp] = 0;
        }
    }
    else
    {
        /* A cheat.
         * Always allow the detections to go through even if the disambiguation process fails.
         * In case of failure, we use the non-disambiguated velocity.
         * The justification being that most target velocities should be below 55km/hr and
         * that higher layer (as yet unimplemented) tracking algorithms can help disambiguate. */
        peakIndx = 1;
    }

    return peakIndx;
}




#define TWENTY_TWO_DB_DOPPLER_SNR ((22 *(256))/6)
#define EIGHTEEN_DB_DOPPLER_SNR ((18 *(256))/6)
#define ZERO_POINT_FIVE_METERS  (0.5f * 128)
#define FOUR_POINT_ZERO_METERS  (4 * 128)


/**
 *  @b Description
 *  @n
 *    The function performs a quadractic peak interpolation to compute the
 *  fractional offset of the peak location. It is primarily intended to be
 *  used in oversampled FFTs.
 *
 *  @param[in]    y             the array of data.
 *  @param[in]    len           length of the array.
 *  @param[in]    indx          coarse peak location.
 *
 *  @retval
 *      interpolated peak location (varies from -1 to +1).
 */
float quadraticInterpFltPeakLoc(float * restrict y, int32_t len, int32_t indx)
{
    float y0 = y[indx];
    float ym1, yp1, thetaPk;
    float Den;
    if (indx == len - 1)
    {
        yp1 = y[0];
    }
    else
    {
        yp1 = y[indx + 1];
    }

    if (indx == 0)
    {
        ym1 = y[len - 1];
    }
    else
    {
        ym1 = y[indx - 1];
    }

    Den = (2.0f*( (2.0f*y0) - yp1 - ym1));

    /* A reasonable restriction on the inverse.
     * Note that y0 is expected to be larger than
     * yp1 and ym1. */
    if (Den > 0.15)
    {
        thetaPk = (yp1 - ym1)* recipsp(Den);
    }
    else
    {
        thetaPk = 0.0f;
    }
    return thetaPk;
}



#ifdef COMPENSATE_FOR_GAIN_PHASE_OFFSET
/*!*****************************************************************************************************************
 * \brief
 * Function Name       :    MmwDemo_rxChanPhaseBiasCompensation
 *
 * \par
 * <b>Description</b>  : Compensation of rx channel phase bias
 *
 * @param[in]               rxChComp : rx channel compensation coefficient
 * @param[in]               input : 32-bit complex input symbols must be 64 bit aligned
 * @param[in]               numAnt : number of symbols
 *
 * @return                  void
 *
 *******************************************************************************************************************
 */
static inline void MmwDemo_rxChanPhaseBiasCompensation(uint32_t *rxChComp,
                                         int64_t *input,
                                         uint32_t numAnt)
{
    int64_t azimuthVal;
    int32_t Re, Im;
    uint32_t antIndx;
    uint32_t rxChCompVal;


    /* Compensation */
    for (antIndx = 0; antIndx < numAnt; antIndx++)
    {
        azimuthVal = _amem8(&input[antIndx]);

        rxChCompVal = _amem4(&rxChComp[antIndx]);

        Re = _ssub(_mpyhir(rxChCompVal, _loll(azimuthVal) ),
                    _mpylir(rxChCompVal, _hill(azimuthVal)));
        Im = _sadd(_mpylir(rxChCompVal, _loll(azimuthVal)),
                    _mpyhir(rxChCompVal, _hill(azimuthVal)));
        _amem8(&input[antIndx]) =  _itoll(Im, Re);
    }
}
#endif








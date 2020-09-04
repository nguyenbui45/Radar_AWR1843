/**
 *   @file  dss_mrr.h
 *
 *   @brief
 *      This is the main header file for the DSS Parking Assist TI Design.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2017 Texas Instruments, Inc.
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
#ifndef DSS_MRR_H
#define DSS_MRR_H

/* MMWAVE Driver Include Files */
#include <ti/common/mmwave_error.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/drivers/adcbuf/ADCBuf.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/osal/DebugP.h>

/* BIOS/XDC Include Files */
#include <ti/sysbios/knl/Semaphore.h>

/* MMWAVE library Include Files */
#include <ti/control/mmwave/mmwave.h>

/* MMW Demo Include Files */
#include "dss_data_path.h"
#include "../common/pa_config_consts.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  Millimeter Wave Demo state
 *
 * @details
 *  The enumeration is used to hold the data path states for the
 *  Millimeter Wave demo
 */
typedef enum MmwDemo_DSS_STATE_e
{
    /*! @brief   State after data path is initialized */
    MmwDemo_DSS_STATE_INIT = 0,

    /*! @brief   State after data path is started */
    MmwDemo_DSS_STATE_STARTED,

    /*! @brief   State after data path is stopped */
    MmwDemo_DSS_STATE_STOPPED,

    /*! @brief   State after STOP request was received by DSP 
                 but complete stop is on-going */
    MmwDemo_DSS_STATE_STOP_PENDING
    
}MmwDemo_DSS_STATE;

typedef struct MmwDemo_DSS_STATS_t
{
    /*! @brief   Counter which tracks the number of config events  
                 The config event is triggered in mmwave config callback function 
                 when remote sends configuration */
    uint8_t      configEvt;

    /*! @brief   Counter which tracks the number of start events 
                 The start event is triggered in mmwave start callback function 
                 when remote calls mmwave_start() */
    uint8_t      startEvt;

    /*! @brief   Counter which tracks the number of stop events
                 The start event is triggered in mmwave stop callback function 
                 when remote calls mmwave_stop() */
    uint8_t      stopEvt;

    /*! @brief   Counter which tracks the number of chirp interrupt skipped due to stopped state of sensor */
    uint32_t     chirpIntSkipCounter;

    /*! @brief   Counter which tracks the number of chirp interrupt skipped due to stopped state of sensor */
    uint32_t     frameIntSkipCounter;

    /*! @brief   Counter which tracks the number of chirp interrupt detected */
    uint32_t     chirpIntCounter;

    /*! @brief   Counter which tracks the number of frame start interrupt  detected */
    uint32_t     frameStartIntCounter;

    /*! @brief   Counter which tracks the number of chirp event detected 
                 The chirp event is triggered in the ISR for chirp interrupt */
    uint32_t     chirpEvt;

    /*! @brief   Counter which tracks the number of frame start event detected 
                 The frame start event is triggered in the ISR for frame start interrupt */
    uint32_t     frameStartEvt;

    /*! @brief   Counter which tracks the number of frames triggered in BSS detected 
                 The frame trigger event is triggered in the mmwave async event callback function */
    uint32_t     frameTrigEvt;

    /*! @brief   Counter which tracks the number of Failed Timing Reports received from BSS  */
    uint32_t     numFailedTimingReports;

    /*! @brief   Counter which tracks the number of Calibration Report received from BSS  */
    uint32_t     numCalibrationReports;    

    /*! @brief   Counter which tracks the number of times saving detected objects in 
                 logging buffer is skipped */
    uint32_t     detObjLoggingSkip;

    /*! @brief   Counter which tracks the number of times saving detected objects in 
                 logging buffer has an error */
    uint32_t     detObjLoggingErr;
}MmwDemo_DSS_STATS;


/**
 * @brief
 *  DSS MRR TI Design  Master control block
 *
 * @details
 *  The structure is used to hold information pertinent to the DSS MRR TI
 *  design.
 */
typedef struct Mrr_DSS_MCB_t
{
    /**
     * @brief   Handle to the SOC Module
     */
    SOC_Handle                  socHandle;

    /**
     * @brief   mmWave control handle use to initialize the link infrastructure
     * which allows communication between the MSS and BSS
     */
    MMWave_Handle               ctrlHandle;

    /**
     * @brief   Handle to the ADCBUF Driver
     */
    ADCBuf_Handle               adcBufHandle;

    /**
     * @brief   Handle to the SOC chirp interrupt listener Handle
     */
    SOC_SysIntListenerHandle    chirpIntHandle;

    /**
     * @brief   Handle to the SOC chirp interrupt listener Handle
     */
    SOC_SysIntListenerHandle    frameStartIntHandle;

    /*! @brief   Semaphore handle for the mailbox communication */
    Semaphore_Handle            mboxSemHandle;

    /**
     * @brief   Mailbox Handle: This is used to exchange messages between the MSS
     * and DSS
     */
    Mbox_Handle              peerMailbox;
    
    /*! @brief   mmw Demo state */
    MmwDemo_DSS_STATE           state;

    /*! @brief   mmw Demo statistics */
    MmwDemo_DSS_STATS           stats;

    /*! @brief   Data Path object */
    MmwDemo_DSS_DataPathObj     dataPathObj[NUM_SUBFRAMES];
    
    /*! @brief   Logging buffer flag */
    uint8_t                     loggingBufferAvailable;
    
    /*! @brief   Subframe index */
    uint8_t                     subframeIndx;

    /*! @brief   inter frameProc token */
    uint8_t                        interFrameProcToken;

    /*! @brief   frame start token */
    uint8_t                        frameStartIntToken;

    /*! @brief   chirpProc token */
    uint8_t                        chirpProcToken;

    /*! @brief   'mailbox has a message' token */
    uint8_t                        mboxProcToken;

    /*! @brief   frame process token */
    uint8_t                        frameProcToken;
}Mrr_DSS_MCB;


/*******************************************************************************************
 * Extern Definitions:
 *******************************************************************************************/
extern Mrr_DSS_MCB gMrrDSSMCB;

extern void _MmwDemo_dssAssert(int32_t expression, const char *file, int32_t line);
#define MmwDemo_dssAssert(expression) {                                      \
                                    _MmwDemo_dssAssert(expression,           \
                                             __FILE__, __LINE__);         \
                                    DebugP_assert(expression);            \
                                   }


/*******************************************************************************************
 * Extern CFG API:
 *******************************************************************************************/
extern void Cfg_AdvFrameCfgInitParams (rlAdvFrameCfg_t* ptrAdvFrameCfg);
extern void Cfg_FrameCfgInitParams (rlFrameCfg_t* ptrFrameCfg);
extern void Cfg_ProfileCfgInitParams (uint8_t profileNum, rlProfileCfg_t* ptrProfileCfg);
extern void Cfg_ChirpCfgInitParams (uint8_t chirpNum, rlChirpCfg_t* ptrChirpCfg);
extern void Cfg_LowPowerModeInitParams (rlLowPowerModeCfg_t* ptrLowPowerMode);
extern void Cfg_ChannelCfgInitParams (rlChanCfg_t* ptrChannelCfg);
extern void Cfg_ADCOutCfgInitParams (rlAdcOutCfg_t* ptrADCOutCfg);

#ifdef __cplusplus
}
#endif

#endif /* DSS_MRR_H */



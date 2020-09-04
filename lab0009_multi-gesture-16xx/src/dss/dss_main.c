/*
 *   @file  dss_main.c
 *
 *   @brief
 *      Millimeter Wave Demo running on DSS
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2019 Texas Instruments, Inc.
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
#include <ti/sysbios/family/c64p/Cache.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>
#include <ti/sysbios/utils/Load.h>


/* MMWSDK Include Files. */
#include <ti/drivers/soc/soc.h>
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

/* MMWAVE Demo Include Files */
#include "dss_gesture.h"
#include "../common/gesture_messages.h"
#include "../common/gesture_config_consts.h"
#include "dss_data_path.h"
#include "Features.h"
#include "../common/link_config.h"

/* C674x mathlib */
/* Suppress the mathlib.h warnings
 *  #48-D: incompatible redefinition of macro "TRUE"
 *  #48-D: incompatible redefinition of macro "FALSE" 
 */
#pragma diag_push
#pragma diag_suppress 48
#include <ti/mathlib/mathlib.h>
#pragma diag_pop

/* Related to linker copy table for copying from L3 to L1PSRAM for example */
#include <cpy_tbl.h>

/**************************************************************************
 ***************************      MACRO definitions      ******************
 **************************************************************************/
#define MMWDEMO_SPEED_OF_LIGHT_IN_METERS_PER_SEC (3.0e8)

#define THRESHOLD_TO_SWITCH_PROFILE      (profileSwitchThreshold)
#define WAIT_FRAME_TO_IDLE_PROFILE       100

#define IDLE_PROFILE_MODE                0U
#define ACTIVE_PROFILE_MODE              1U

/* To enable:0, disable:1 Power saving mode */
#define DISABLE_POWER_SAVING_MODE         0

/*! @brief Azimuth FFT size */
#define MMW_NUM_ANGLE_BINS NUM_ANGLE_BINS_FEATURES

/* enable this to gather debug informations */
#define DBG
/**************************************************************************
 *************************** External DSS Functions ******************
 **************************************************************************/

extern float GestDemo_interFrameProcessingPart1(GestDemo_DSS_DataPathObj *obj, uint16_t fcounter);
extern void GestDemo_interFrameProcessingPart2(GestDemo_DSS_DataPathObj *obj);

/**************************************************************************
 *************************** Global Definitions ********************************
 **************************************************************************/
/* Set profile Type and device power state mode based on selection of Power saving feature */
#if DISABLE_POWER_SAVING_MODE
volatile uint8_t gProfileType = ACTIVE_PROFILE_CFG_SELECTED;
volatile uint8_t gDevPowerState = ACTIVE_PROFILE_MODE;
#else
volatile uint8_t gProfileType = IDLE_PROFILE_CFG_SELECTED;
volatile uint8_t gDevPowerState = IDLE_PROFILE_MODE;
#endif
/* condition to switch the profile b/w IDLE and ACTIVE mode */
volatile bool isSwitchProfile = false;
/* Frame count to switch from ACTIVE to IDLE mode */
uint8_t frameCountToSwitch = 0U;
float gEnergySum = 0;
/* threshold of Enegry level detected near Device to switch to ACTIVE Mode */
float profileSwitchThreshold = 2.6;
/* parameter to store Heap Allocation state */
uint8_t isHeapAllocated = 0;

/**
 * @brief
 *  DSS stores demo output and DSS to MSS ISR information (for fast exception 
 *  signalling) in HSRAM.
 */
typedef struct GestDemo_HSRAM_t_ {
#define MMW_DATAPATH_DET_PAYLOAD_SIZE (SOC_XWR16XX_DSS_HSRAM_SIZE -  sizeof(uint8_t))
    /*! @brief data path processing/detection related message payloads, these
               messages are signalled through DSS to MSS mailbox */ 
    uint8_t  dataPathDetectionPayload[MMW_DATAPATH_DET_PAYLOAD_SIZE];

    /*! @brief Information relayed through DSS triggering software interrupt to
               MSS. It stores one of the exception IDs @ref DSS_TO_MSS_EXCEPTION_IDS */
    uint8_t  dss2MssIsrInfo;
} GestDemo_HSRAM_t;

#pragma DATA_SECTION(gHSRAM, ".demoSharedMem");
#pragma DATA_ALIGN(gHSRAM, 4);
GestDemo_HSRAM_t gHSRAM;

/* Structure which store information related to gesture features */
#pragma DATA_SECTION(gfeatures, ".l3data");
Features_t gfeatures;

/**
 * @brief
 *  Global Variable for tracking information required by the Gest Demo
 */
GestDemo_DSS_MCB    gGestDssMCB;
/* Store ACTIVE and IDLE profile configurations which are used to re-configure the RadarSS 
   while switching b/w ACTIVE/IDLE modes */
GestDemo_IdleActiveCfgParams gIdleActiveCfgParams[2];

volatile cycleLog_t gCycleLog;

/* copy table related */
extern far COPY_TABLE _GestDemo_fastCode_L1PSRAM_copy_table;

/**************************************************************************
 ************************* GestDemo Functions Prototype  **********************
 **************************************************************************/

/* Copy table related */
static void GestDemo_edmaBlockCopy(EDMA_Handle handle, uint32_t loadAddr, 
                                  uint32_t runAddr, uint16_t size);
static void GestDemo_copyTable(EDMA_Handle handle, COPY_TABLE *tp);

/* Internal DataPath Functions */
int32_t GestDemo_dssDataPathInit(void);
static int32_t GestDemo_dssDataPathConfig(uint8_t profileType);
static int32_t GestDemo_dssDataPathStart(uint8_t profileType);
static int32_t GestDemo_dssDataPathStop(void);
static int32_t GestDemo_dssDataPathProcessEvents(UInt event, uint16_t fcounter, float gesture_pktno);
static int32_t GestDemo_dssDataPathReconfig(GestDemo_DSS_DataPathObj *obj);
static void GestDemo_measurementResultOutput(GestDemo_DSS_DataPathObj *obj);

/* Internal Interrupt handler */
static void GestDemo_dssChirpIntHandler(uintptr_t arg);
static void GestDemo_dssFrameStartIntHandler(uintptr_t arg);

/* Internal mmwDemo Tasks running on DSS */
static void GestDemo_dssInitTask(UArg arg0, UArg arg1);
static void GestDemo_dssDataPathTask(UArg arg0, UArg arg1);

/* Internal mmwDemo function to trigger DSS to MSS ISR for urgent exception signalling */
static void GestDemo_triggerDss2MssISR(uint8_t dss2MssIsrInfo);

/* external sleep function when in idle (used in .cfg file) */
void GestDemo_sleep(void);

static int32_t GestDemo_dssSendProcessOutputToMSS
(
    uint8_t           *ptrHsmBuffer,
    uint32_t           outputBufSize,
    GestDemo_DSS_DataPathObj   *obj,
    float              gestsure_pktno
);
void GestDemo_dssDataPathOutputLogging(GestDemo_DSS_DataPathObj   * dataPathObj, float gesture_pktno);

/**************************************************************************
 *************************** GestDemo DSS Functions **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Interrupt handler callback for chirp available. It runs in the ISR context.
 *
 *  @retval
 *      Not Applicable.
 */
 #ifdef DBG
 #define NUM_CHIRP_TIME_STAMPS 128
 uint32_t gChirpProcesTimeStamp[NUM_CHIRP_TIME_STAMPS];
 uint32_t gFrameProcessTime[NUM_CHIRP_TIME_STAMPS];
 uint32_t gChirpTimeStampIdx = 0;
 #endif

static void GestDemo_dssChirpIntHandler(uintptr_t arg)
{
    GestDemo_DSS_DataPathObj * dpObj = &gGestDssMCB.dataPathObj;

    if((gGestDssMCB.state == GestDemo_DSS_STATE_STOPPED) ||
       (gGestDssMCB.dataPathContext.interFrameProcToken <= 0))
    {
        gGestDssMCB.stats.chirpIntSkipCounter++;
        return;
    }

#ifdef DBG
    /* time profiling of the Chirp interrupt */
    if (gChirpTimeStampIdx == NUM_CHIRP_TIME_STAMPS)
    {
       gChirpTimeStampIdx = 0;
   }

   gFrameProcessTime[gChirpTimeStampIdx++] = Cycleprofiler_getTimeStamp();
#endif

    if (dpObj->chirpCount == 0)
    {
        GestDemo_DSS_DataPathObj * dpObjPrev = dpObj;

        /* Note: this is valid after the first frame */
        dpObjPrev->timingInfo.interFrameProcessingEndMargin =
            Cycleprofiler_getTimeStamp() - dpObjPrev->timingInfo.interFrameProcessingEndTime -
            dpObjPrev->timingInfo.subFrameSwitchingCycles;
    }
    else if (dpObj->chirpCount == dpObj->numChirpsPerChirpEvent)
    {
        dpObj->timingInfo.chirpProcessingEndMarginMin =
            Cycleprofiler_getTimeStamp() - dpObj->timingInfo.chirpProcessingEndTime;
        dpObj->timingInfo.chirpProcessingEndMarginMax =
            dpObj->timingInfo.chirpProcessingEndMarginMin;
    }
    else
    {
        uint32_t margin = Cycleprofiler_getTimeStamp() - dpObj->timingInfo.chirpProcessingEndTime;
        if (margin > dpObj->timingInfo.chirpProcessingEndMarginMax)
        {
            dpObj->timingInfo.chirpProcessingEndMarginMax = margin;
        }
        if (margin < dpObj->timingInfo.chirpProcessingEndMarginMin)
        {
            dpObj->timingInfo.chirpProcessingEndMarginMin = margin;
        }
    }
    /* Increment interrupt counter for debugging purpose */
    gGestDssMCB.stats.chirpIntCounter++;

    /* Check if previous chirp processing has completed */
    if (gGestDssMCB.dataPathContext.chirpProcToken != 0)
    {
        GestDemo_triggerDss2MssISR(GESTDEMO_DSS2MSS_CHIRP_PROC_DEADLINE_MISS_EXCEPTION);
        DebugP_assert(0);
    }

    gGestDssMCB.dataPathContext.chirpProcToken++;

    /* Post event to notify chirp available interrupt */
    Event_post(gGestDssMCB.eventHandle, GESTDEMO_CHIRP_EVT);
}

/**
 *  @b Description
 *  @n
 *      Interrupt handler callback for frame start ISR.
 *
 *  @retval
 *      Not Applicable.
 */
static void GestDemo_dssFrameStartIntHandler(uintptr_t arg)
{
#ifdef DBG
    gChirpProcesTimeStamp[gGestDssMCB.stats.frameStartIntCounter % NUM_CHIRP_TIME_STAMPS] = Cycleprofiler_getTimeStamp();
#endif
    if(gGestDssMCB.state == GestDemo_DSS_STATE_STOPPED)
    {
        gGestDssMCB.stats.frameIntSkipCounter++;
        return;
    }

    /* Check if previous chirp processing has completed */
    if (gGestDssMCB.dataPathContext.interFrameProcToken != 0)
    {
        GestDemo_triggerDss2MssISR(GESTDEMO_DSS2MSS_FRAME_PROC_DEADLINE_MISS_EXCEPTION);
        DebugP_assert(0);
    }

    gGestDssMCB.dataPathContext.interFrameProcToken++;

    /* Increment interrupt counter for debugging purpose */    
    if (gGestDssMCB.subFrameIndx == 0)
    {
        gGestDssMCB.stats.frameStartIntCounter++;
    }

    /* Post event to notify frame start interrupt */
    Event_post(gGestDssMCB.eventHandle, GESTDEMO_FRAMESTART_EVT);
}

/**
 *  @b Description
 *  @n
 *      Function to send a message to peer through Mailbox virtural channel
 *
 *  @param[in]  message
 *      Pointer to the Captuere demo message.
 *
 *  @retval
 *      Success    - 0
 *      Fail       < -1
 */
static int32_t GestDemo_mboxWrite(GestDemo_message    *message)
{
    int32_t                  retVal = -1;

    retVal = Mailbox_write (gGestDssMCB.peerMailbox, (uint8_t*)message, sizeof(GestDemo_message));
    if (retVal == sizeof(GestDemo_message))
    {
        retVal = 0;
    }
    return retVal;
}

static void GestDemo_cfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum)
{    
    /* if subFrameNum undefined, broadcast to all sub-frames */
    if(subFrameNum == GESTDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG)
    {
        memcpy((void *)((uint32_t) &gGestDssMCB.cliCfg + offset), srcPtr, size);
        
    }
    else
    {
        /* This application is built with legacy Frame configuration, 
         * so it doesn't support Adv-Frame Config
         */
    }
}

/**
 *  @b Description
 *  @n
 *      Function that acts upon receiving message that BSS has stopped
 *      successfully.
 *
 *  @retval
 *      Not applicable
 */
static void GestDemo_bssStopDone(void)
{    
    /*Change state to stop_pending*/
    gGestDssMCB.state = GestDemo_DSS_STATE_STOP_PENDING;    
    
    if(gGestDssMCB.dataPathContext.interFrameProcToken == 0)
    {
        /*BSS stop message received after inter-frame processing
          is completed (including sending out UART data).
          */
         Event_post(gGestDssMCB.eventHandle, GESTDEMO_STOP_COMPLETE_EVT);       
    }
    else
    {
        /*BSS stop message received during inter-frame processing.
          Switch to stop pending state and stop once inter frame
          processing done. Nothing to be done here.
          */        
    }
}

/**
 *  @b Description
 *  @n
 *      The Task is used to handle  the mmw demo messages received from Mailbox virtual channel.
 *
 *  @param[in]  arg0
 *      arg0 of the Task. Not used
 *  @param[in]  arg1
 *      arg1 of the Task. Not used
 *
 *  @retval
 *      Not Applicable.
 */
static void GestDemo_mboxReadTask(UArg arg0, UArg arg1)
{
    GestDemo_message      message;
    int32_t              retVal = 0;
    int8_t               subFrameNum;
    uint32_t             log2NumAvgChirpsTemp;

    /* wait for new message and process all the messsages received from the peer */
    while(1)
    {
        Semaphore_pend(gGestDssMCB.mboxSemHandle, BIOS_WAIT_FOREVER);

        /* Read the message from the peer mailbox: We are not trying to protect the read
         * from the peer mailbox because this is only being invoked from a single thread */
        retVal = Mailbox_read(gGestDssMCB.peerMailbox, (uint8_t*)&message, sizeof(GestDemo_message));
        if (retVal < 0)
        {
            /* Error: Unable to read the message. Setup the error code and return values */
            System_printf ("Error: Mailbox read failed [Error code %d]\n", retVal);
        }
        else if (retVal == 0)
        {
            /* We are done: There are no messages available from the peer execution domain. */
            continue;
        }
        else
        {
            /* Flush out the contents of the mailbox to indicate that we are done with the message. This will
             * allow us to receive another message in the mailbox while we process the received message. */
            Mailbox_readFlush (gGestDssMCB.peerMailbox);

            /* Process the received message: */
            subFrameNum = message.subFrameNum;                     

            switch (message.type)
            {
                case GESTDEMO_MSS2DSS_SET_CONFIG:
                {
                    /* DSS recieves this notification when MSS gets basic config command over CLI,
                       In this case DSS configure ADCBUFF and RadarSS with default configurations */
                    Event_post(gGestDssMCB.eventHandle, GESTDEMO_CONFIG_EVT);
                    break;
                }
                case GESTDEMO_MSS2DSS_SENSOR_START_CFG:
                {
                    /* DSS recieves this notification when MSS gets sensor start command over CLI,
                       In this case DSS triggers the frame*/
                    Event_post(gGestDssMCB.eventHandle, GESTDEMO_START_EVT);
                    break;
                }                
                case GESTDEMO_MSS2DSS_GUIMON_CFG:
                {
                    /* Save guimon configuration */
                    GestDemo_cfgUpdate((void *)&message.body.guiMonSel, 
                                         offsetof(GestDemo_CliCfg_t, guiMonSel),
                                         sizeof(GestDemo_GuiMonSel), subFrameNum);
                    break;
                }
                case GESTDEMO_MSS2DSS_CALIB_DC_RANGE_SIG: 
                {
                    /* Save  of DC range antenna signature configuration */
                    GestDemo_cfgUpdate((void *)&message.body.calibDcRangeSigCfg, 
                                         offsetof(GestDemo_CliCfg_t, calibDcRangeSigCfg),
                                         sizeof(MmwDemo_CalibDcRangeSigCfg), subFrameNum);
                                      
                    log2NumAvgChirpsTemp = GestDemo_floorLog2(message.body.calibDcRangeSigCfg.numAvgChirps);
                    /* Save log2NumAvgChirps  */
                    if (subFrameNum == GESTDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG)
                    {
                        gGestDssMCB.dataPathObj.dcRangeSigCalibCntr = 0;
                        gGestDssMCB.dataPathObj.log2NumAvgChirps = log2NumAvgChirpsTemp;
                    }
                    else
                    {
                        /* This application is built with legacy Frame configuration, 
                         * so it doesn't support Adv-Frame Config
                         */
                    }
                    break;
                }
                case GESTDEMO_MSS2DSS_NEAR_FIELD_CFG:
                {
                    /* Save  of near field correction configuration */
                    GestDemo_cfgUpdate((void *)&message.body.nearFieldCorrectionCfg,
                                         offsetof(MmwDemo_CliCfg_t, nearFieldCorrectionCfg),
                                         sizeof(MmwDemo_NearFieldCorrectionCfg), subFrameNum);
                    break;
                }
                case GESTDEMO_MSS2DSS_CLUTTER_REMOVAL:
                {
                    /* Save  clutter removal configuration */
                    GestDemo_cfgUpdate((void *)&message.body.clutterRemovalCfg,
                                         offsetof(MmwDemo_CliCfg_t, clutterRemovalCfg),
                                         sizeof(MmwDemo_ClutterRemovalCfg), subFrameNum);
                    break;
                }
                case GESTDEMO_MSS2DSS_ADCBUFCFG:
                {
                    /* Save ADCBUF configuration */
                    GestDemo_cfgUpdate((void *)&message.body.adcBufCfg,
                                         offsetof(MmwDemo_CliCfg_t, adcBufCfg),
                                         sizeof(MmwDemo_ADCBufCfg), subFrameNum);
                    break;
                }
                case GESTDEMO_MSS2DSS_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE:
                {
                    /* Save range bias and Rx channels phase compensation */
                    memcpy((void *) &gGestDssMCB.cliCommonCfg.compRxChanCfg, (void *)\
                           &message.body.compRxChanCfg, sizeof(MmwDemo_compRxChannelBiasCfg_t));
                    break;
                }
                case GESTDEMO_MSS2DSS_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE:
                {
                    /* Save range bias and Rx channels phase compensation */
                    memcpy((void *) &gGestDssMCB.cliCommonCfg.measureRxChanCfg, (void *)\
                           &message.body.measureRxChanCfg, sizeof(MmwDemo_measureRxChannelBiasCfg_t));
                    break;
                }
                case GESTDEMO_MSS2DSS_LVDSSTREAM_CFG:
                {
                    /* Save LVDS Stream configuration */
                    GestDemo_cfgUpdate((void *)&message.body.lvdsStreamCfg, 
                                         offsetof(MmwDemo_CliCfg_t, lvdsStreamCfg),
                                         sizeof(MmwDemo_LvdsStreamCfg), subFrameNum);
                    break;
                }
                case GESTDEMO_MSS2DSS_CQ_SATURATION_MONITOR:
                {
                    uint8_t     profileIdx;
                    profileIdx = message.body.cqSatMonCfg.profileIndx;
                    if(profileIdx < RL_MAX_PROFILES_CNT)
                    {
                        /* Save CQ RX saturation monitor configuration */
                        memcpy((void *) &gGestDssMCB.cliCommonCfg.cqSatMonCfg[profileIdx],
                                (void *)&message.body.cqSatMonCfg,
                                sizeof(rlRxSatMonConf_t));
                    }
                    break;
                }
                case GESTDEMO_MSS2DSS_CQ_SIGIMG_MONITOR:
                {
                    uint8_t     profileIdx;
                    profileIdx = message.body.cqSigImgMonCfg.profileIndx;
                    if(profileIdx < RL_MAX_PROFILES_CNT)
                    {
                        /* Save CQ RX signal & Image band monitor configuration */
                        memcpy((void *) &gGestDssMCB.cliCommonCfg.cqSigImgMonCfg[profileIdx],
                                (void *)&message.body.cqSigImgMonCfg,
                                sizeof(rlSigImgMonConf_t));
                    }
                    break;
                }
                case GESTDEMO_MSS2DSS_ANALOG_MONITOR:
                {
                    /* Save analog monitor configuration */
                    memcpy((void *) &gGestDssMCB.cliCommonCfg.anaMonCfg,
                            (void *)&message.body.anaMonCfg,
                            sizeof(MmwDemo_AnaMonitorCfg));
                    break;
                }

                case GESTDEMO_MSS2DSS_GESTURE_FEATURE_CFG:
                {
                    /* Store Gesture Feature parameters sent over CLI */
                    gfeatures.rangebin_start       = message.body.GestureFeatCfg.rangebin_start;
                    gfeatures.rangebin_stop        = message.body.GestureFeatCfg.rangebin_stop;
                    gfeatures.posdopplerbin_start  = message.body.GestureFeatCfg.dopplerbin_start;
                    gfeatures.posdopplerbin_stop   = message.body.GestureFeatCfg.dopplerbin_stop;
                    gfeatures.negdopplerbin_start  = (-1)*message.body.GestureFeatCfg.dopplerbin_start;
                    gfeatures.negdopplerbin_stop   = (-1)*message.body.GestureFeatCfg.dopplerbin_stop;
                    gfeatures.detthresh            = message.body.GestureFeatCfg.det_thresh_featuregen;

                    break;
                }
                case GESTDEMO_MSS2DSS_DETOBJ_SHIPPED:
                {
                    GestDemo_DSS_DataPathObj *dataPathCurrent;

                    dataPathCurrent = &gGestDssMCB.dataPathObj;
                    dataPathCurrent->timingInfo.transmitOutputCycles =
                        Cycleprofiler_getTimeStamp() - dataPathCurrent->timingInfo.interFrameProcessingEndTime;

                    gGestDssMCB.subFrameIndx++;
                    if (gGestDssMCB.subFrameIndx == gGestDssMCB.numSubFrames)
                    {
                        gGestDssMCB.subFrameIndx = 0;
                    }

                    dataPathCurrent->timingInfo.subFrameSwitchingCycles = 0;
                    
                    gGestDssMCB.dataPathContext.interFrameProcToken--;

                    gGestDssMCB.loggingBufferAvailable = 1;

                    /* Post event to complete stop operation, if pending */
                    if ((gGestDssMCB.state == GestDemo_DSS_STATE_STOP_PENDING) && (gGestDssMCB.subFrameIndx == 0))
                    {
                        Event_post(gGestDssMCB.eventHandle, GESTDEMO_STOP_COMPLETE_EVT);
                    }
                    break;
                }
                case GESTDEMO_MSS2DSS_SET_DATALOGGER:
                {
                    gGestDssMCB.cfg.dataLogger = message.body.dataLogger;
                    break;
                }
                default:
                {
                    /* Message not support */
                    System_printf ("Error: unsupport Mailbox message id=%d\n", message.type);
                    GestDemo_dssAssert(0);
                    break;
                }
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      This function is a callback funciton that invoked when a message is received from the peer.
 *
 *  @param[in]  handle
 *      Handle to the Mailbox on which data was received
 *  @param[in]  peer
 *      Peer from which data was received

 *  @retval
 *      Not Applicable.
 */
void GestDemo_mboxCallback
(
    Mbox_Handle  handle,
    Mailbox_Type    peer
)
{
    /* Message has been received from the peer endpoint. Wakeup the mmWave thread to process
     * the received message. */
    Semaphore_post (gGestDssMCB.mboxSemHandle);
}

/**
 *  @b Description
 *  @n
 *      Function to send detected objects to MSS logger.
 *
 *  @param[in]  ptrHsmBuffer
 *      Pointer to the output buffer
 *  @param[in]  outputBufSize
 *      Size of the output buffer
 *  @param[in]  obj
 *      Handle to the Data Path Object
 *
 *  @retval
 *      =0    Success
 *      <0    Failed
 */

int32_t GestDemo_dssSendProcessOutputToMSS
(
    uint8_t           *ptrHsmBuffer,
    uint32_t           outputBufSize,
    GestDemo_DSS_DataPathObj   *obj,
    float              gesture_pktno
)
{
    uint8_t             *ptrCurrBuffer;
    uint32_t            totalHsmSize = 0;
    uint32_t            totalPacketLen = sizeof(GestDemo_output_message_header);
    uint32_t            itemPayloadLen;
    int32_t             retVal = 0;
    GestDemo_message     message;
    GestDemo_GuiMonSel   *pGuiMonSel;
    uint16_t            cnt = 0, tlvIdx = 0;

    /* Get Gui Monitor configuration */
    pGuiMonSel = &obj->cliCfg->guiMonSel;
    float * gestureMetrics;

    /* Validate input params */
    if(ptrHsmBuffer == NULL)
    {
        retVal = -1;
        goto Exit;
    }

    /* Clear message to MSS */
    memset((void *)&message, 0, sizeof(GestDemo_message));
    message.type = GESTDEMO_DSS2MSS_DETOBJ_READY;
    /* Header: 
        4 * 2 Bytes - magic word {0x0102,0x0304,0x0506,0x0708}
        4 Bytes - version
        4 Bytes - Total packet length
        4 Bytes - Platform
        4 Bytes - Frame no.
        4 Bytes - DSP Time CPU cycles
        4 Bytes - numDetected Objects
        4 Bytes - no. of TLV
        4 Bytes - sub-frame no.
        */
    message.body.detObj.header.platform = 0xA1642;
    message.body.detObj.header.magicWord[0] = 0x0102;
    message.body.detObj.header.magicWord[1] = 0x0304;
    message.body.detObj.header.magicWord[2] = 0x0506;
    message.body.detObj.header.magicWord[3] = 0x0708;
    message.body.detObj.header.numDetectedObj = obj->numDetObj;
    message.body.detObj.header.version =    MMWAVE_SDK_VERSION_BUILD |   //DEBUG_VERSION
                                            (MMWAVE_SDK_VERSION_BUGFIX << 8) |
                                            (MMWAVE_SDK_VERSION_MINOR << 16) |
                                            (MMWAVE_SDK_VERSION_MAJOR << 24);

    /* Set pointer to HSM buffer */
    ptrCurrBuffer = ptrHsmBuffer;


    /** NOTE: In this demo only 'sendGestureFeatures' condition is being used & tested, other 
        conditions are not tested with this demo but retains for future purpose */

    if (pGuiMonSel->sendGestureFeatures == 1)
    {
         /* Data format: 6 features and other helping Bytes.
         
            Bytes:Detail
            [4] : TLV Type
            [4] : TLV Length
            [4] : Weighted Doppler Feature
            [4] : Instance Enegry Feature
            [4] : Weighted Range Feature
            [4] : Azimuth Doppler Correlation Feature
            [4] : Weighted Azimuth Freq Feature
            [4] : Weighted Elevation Freq Feature
            [4] : Frame Counter
            [4*7] : Possibility of Gesture (including background)
            [4] : reserved
           */
           /* Note: Feature order should be same as given to Neural network while training it. 
                 Please refer the pre-proc_scripts 'generate_feature_vec' function */
        itemPayloadLen = (8 /* TLV length & Type */+ NUM_OF_GESTURE_FEATURES * sizeof(float) /* featured data */
                           + sizeof(uint32_t) /* FrameCnt */ + (NUM_OF_GESTURE_FEATURES +2)*sizeof(float));

        totalHsmSize += itemPayloadLen;
        if(totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }

        gestureMetrics=(float*)ptrCurrBuffer;

        cnt = 0;

        /* Following the same Order as given to Neural network while training it. 
         * Please refer the pre-proc_scripts 'generate_feature_vec' function 
         */
        gestureMetrics[cnt++] = gfeatures.pWtdoppler[gfeatures.currindx];
        gestureMetrics[cnt++] = gfeatures.pInstenergy;
        gestureMetrics[cnt++] = gfeatures.pWtrange;
        gestureMetrics[cnt++] = gfeatures.pAzdoppcorr;
        gestureMetrics[cnt++] = gfeatures.pMaxazfreq[gfeatures.currindx];
        gestureMetrics[cnt++] = gfeatures.pMaxelfreq;
        /* send the packet/frame No */
        memcpy((void*)&gestureMetrics[cnt], &gGestDssMCB.stats.frameStartIntCounter, 4U);
        cnt++;

        uint8_t *ptrMatrix = (uint8_t *)&gestureMetrics[cnt];
        cnt = 0;
        
        /* creating dummy fillers in the data-packet, this will be populated in MSS [8 4bytes words] */
        memset(&ptrMatrix[cnt], 0, (NUM_OF_GESTURE_FEATURES+2)*sizeof(float));



        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen;
        message.body.detObj.tlv[tlvIdx].type = GESTDEMO_OUTPUT_MSG_GESTURE_FEATURES;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) ptrCurrBuffer;
        tlvIdx++;

        /* Incrementing pointer to HSM buffer */
        ptrCurrBuffer = (uint8_t *)((uint32_t)ptrHsmBuffer + totalHsmSize);
        totalPacketLen += sizeof(MmwDemo_output_message_tl) + itemPayloadLen;


    }

    /* Sending stats information  */
    if (pGuiMonSel->statsInfo == 1)
    {
        MmwDemo_output_message_stats stats;
        itemPayloadLen = sizeof(MmwDemo_output_message_stats);
        totalHsmSize += itemPayloadLen;
        if(totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }

        stats.interChirpProcessingMargin = (uint32_t) (obj->timingInfo.chirpProcessingEndMarginMin/DSP_CLOCK_MHZ);
        stats.interFrameProcessingMargin = (uint32_t) (obj->timingInfo.interFrameProcessingEndMargin/DSP_CLOCK_MHZ);
        stats.interFrameProcessingTime = (uint32_t) (obj->timingInfo.interFrameProcCycles/DSP_CLOCK_MHZ);
        stats.transmitOutputTime = (uint32_t) (obj->timingInfo.transmitOutputCycles/DSP_CLOCK_MHZ);
        stats.activeFrameCPULoad = obj->timingInfo.activeFrameCPULoad;
        stats.interFrameCPULoad = obj->timingInfo.interFrameCPULoad;
        memcpy(ptrCurrBuffer, (void *)&stats, itemPayloadLen);

        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen;
        message.body.detObj.tlv[tlvIdx].type = GESTDEMO_OUTPUT_MSG_STATS;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) ptrCurrBuffer;;
        tlvIdx++;

        /* Incrementing pointer to HSM buffer */
        ptrCurrBuffer = (uint8_t *)((uint32_t)ptrHsmBuffer + totalHsmSize);
        totalPacketLen += sizeof(MmwDemo_output_message_tl) + itemPayloadLen;
    }

    if( retVal == 0)
    {
        message.body.detObj.header.numTLVs = tlvIdx;
        /* Round up packet length to multiple of GESTDEMO_OUTPUT_MSG_SEGMENT_LEN */
        message.body.detObj.header.totalPacketLen = GESTDEMO_OUTPUT_MSG_SEGMENT_LEN *
                ((totalPacketLen + (GESTDEMO_OUTPUT_MSG_SEGMENT_LEN-1))/GESTDEMO_OUTPUT_MSG_SEGMENT_LEN);
        message.body.detObj.header.timeCpuCycles =  Cycleprofiler_getTimeStamp();
        message.body.detObj.header.frameNumber = gGestDssMCB.stats.frameStartIntCounter;
        message.body.detObj.header.subFrameNumber = gGestDssMCB.subFrameIndx;
        if (GestDemo_mboxWrite(&message) != 0)
        {
            retVal = -1;
        }
    }
Exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function to trigger DSS to MSS ISR for fast signalling
 *
 *  @retval
 *      Not Applicable.
 */
static void GestDemo_triggerDss2MssISR(uint8_t dss2MssIsrInfo)
{
    int32_t errCode;

    gHSRAM.dss2MssIsrInfo = dss2MssIsrInfo;
    if (SOC_triggerDSStoMSSsoftwareInterrupt(gGestDssMCB.socHandle, 
            GESTDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_DSS, &errCode) != 0)
    {
        System_printf("Failed to trigger software interrupt %d, error code = %d\n",
            GESTDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_DSS, errCode);
    }
}

/**
 *  @b Description
 *  @n
 *      Function to send data path detection output.
 *
 *  @retval
 *      Not Applicable.
 */
void GestDemo_dssDataPathOutputLogging(GestDemo_DSS_DataPathObj   * dataPathObj, float gesture_pktno)
{
    /* Sending detected objects to logging buffer and shipped out from MSS UART */
    if (gGestDssMCB.loggingBufferAvailable == 1)
    {
        /* Set the logging buffer available flag to be 0 */
        gGestDssMCB.loggingBufferAvailable = 0;

        /* Save output in logging buffer - HSRAM memory and a message is sent to MSS to notify
           logging buffer is ready */
        if (GestDemo_dssSendProcessOutputToMSS((uint8_t *)&gHSRAM.dataPathDetectionPayload[0],
                                             (uint32_t)MMW_DATAPATH_DET_PAYLOAD_SIZE,
                                             dataPathObj, gesture_pktno) < 0)
        {
                /* Increment logging error */
                gGestDssMCB.stats.detObjLoggingErr++;
        }
    }
    else
    {
        /* Logging buffer is not available, skip saving detected objects to logging buffer */
        gGestDssMCB.stats.detObjLoggingSkip++;
    }
}

/**
 *  @b Description
 *  @n
 *      Notify MSS about current profile used (ACTIVE/IDLE mode)
 *
 *  @retval
 *      Not Applicable.
 */
void GestDemo_notifyMssProfileType()
{
    /* notify MSS if DSS switch to ACTIVE_PROFILE_MODE */
    GestDemo_message      message;
    /* Send a message to MSS to log the output data */
    memset((void *)&message, 0, sizeof(GestDemo_message));

    if(gProfileType == ACTIVE_PROFILE_CFG_SELECTED)
    {
        message.type = GESTDEMO_MSS2DSS_ACTIVE_PROFILE_SELECT;
    }
    else
    {
        message.type = GESTDEMO_MSS2DSS_IDLE_PROFILE_SELECT;
    }
    
    if (GestDemo_mboxWrite(&message) != 0)
    {
        System_printf ("Error: Mailbox send message id=%d failed \n", message.type);
    }
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Initialization on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t GestDemo_dataPathAdcBufInit(GestDemo_DSS_dataPathContext_t *context)
{
    ADCBuf_Params       ADCBufparams;

    /*****************************************************************************
     * Initialize ADCBUF driver
     *****************************************************************************/
    ADCBuf_init();

    /* ADCBUF Params initialize */
    ADCBuf_Params_init(&ADCBufparams);
    ADCBufparams.chirpThresholdPing = GESUTURE_ADCBUFF_CHIRP_THRESHOLD;
    ADCBufparams.chirpThresholdPong = GESUTURE_ADCBUFF_CHIRP_THRESHOLD;
    ADCBufparams.continousMode  = 0;

    /* Open ADCBUF driver */
   context->adcbufHandle = ADCBuf_open(0, &ADCBufparams);
    if (context->adcbufHandle == NULL)
    {
        System_printf("Error: MMWDemoDSS Unable to open the ADCBUF driver\n");
        return -1;
    }
    System_printf("Debug: MMWDemoDSS ADCBUF Instance(0) %p has been opened successfully\n", 
        context->adcbufHandle);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Performs linker generated copy table copy using EDMA. Currently this is
 *      used to page in fast code from L3 to L1PSRAM.
 *  @param[in]  handle EDMA handle
 *  @param[in]  tp Pointer to copy table
 *
 *  @retval
 *      Not Applicable.
 */
static void GestDemo_copyTable(EDMA_Handle handle, COPY_TABLE *tp)
{
    uint16_t i;
    COPY_RECORD crp;
    uint32_t loadAddr;
    uint32_t runAddr;

    for (i = 0; i < tp->num_recs; i++)
    {
        crp = tp->recs[i];
        loadAddr = (uint32_t)crp.load_addr;
        runAddr = (uint32_t)crp.run_addr;

        /* currently we use only one count of EDMA which is 16-bit so we cannot
           handle tables bigger than 64 KB */
        GestDemo_dssAssert(crp.size <= 65536U);

        if (crp.size)
        {
            GestDemo_edmaBlockCopy(handle, loadAddr, runAddr, crp.size);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Performs simple block copy using EDMA. Used for the purpose of copying
 *      linker table for L3 to L1PSRAM copy. memcpy cannot be used because there is
 *      no data bus access to L1PSRAM.
 *
 *  @param[in]  handle EDMA handle
 *  @param[in]  loadAddr load address
 *  @param[in]  runAddr run address
 *  @param[in]  size size in bytes
 *
 *  @retval
 *      Not Applicable.
 */
static void GestDemo_edmaBlockCopy(EDMA_Handle handle, uint32_t loadAddr, uint32_t runAddr, uint16_t size)
{
    EDMA_channelConfig_t config;
    volatile bool isTransferDone;

    config.channelId = EDMA_TPCC0_REQ_FREE_0;
    config.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    config.paramId = (uint16_t)EDMA_TPCC0_REQ_FREE_0;
    config.eventQueueId = 0;

    config.paramSetConfig.sourceAddress = (uint32_t) SOC_translateAddress((uint32_t)loadAddr,
        SOC_TranslateAddr_Dir_TO_EDMA, NULL);
    config.paramSetConfig.destinationAddress = (uint32_t) SOC_translateAddress((uint32_t)runAddr,
        SOC_TranslateAddr_Dir_TO_EDMA, NULL);

    config.paramSetConfig.aCount = size;
    config.paramSetConfig.bCount = 1U;
    config.paramSetConfig.cCount = 1U;
    config.paramSetConfig.bCountReload = 0U;

    config.paramSetConfig.sourceBindex = 0U;
    config.paramSetConfig.destinationBindex = 0U;

    config.paramSetConfig.sourceCindex = 0U;
    config.paramSetConfig.destinationCindex = 0U;

    config.paramSetConfig.linkAddress = EDMA_NULL_LINK_ADDRESS;
    config.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_A;
    config.paramSetConfig.transferCompletionCode = (uint8_t) EDMA_TPCC0_REQ_FREE_0;
    config.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    config.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;

    /* don't care because of linear addressing modes above */
    config.paramSetConfig.fifoWidth = (uint8_t) EDMA3_FIFO_WIDTH_8BIT;

    config.paramSetConfig.isStaticSet = false;
    config.paramSetConfig.isEarlyCompletion = false;
    config.paramSetConfig.isFinalTransferInterruptEnabled = true;
    config.paramSetConfig.isIntermediateTransferInterruptEnabled = false;
    config.paramSetConfig.isFinalChainingEnabled = false;
    config.paramSetConfig.isIntermediateChainingEnabled = false;
    config.transferCompletionCallbackFxn = NULL;
    config.transferCompletionCallbackFxnArg = NULL;

    if (EDMA_configChannel(handle, &config, false) != EDMA_NO_ERROR)
    {
        GestDemo_dssAssert(0);
    }

    if (EDMA_startDmaTransfer(handle, config.channelId) != EDMA_NO_ERROR)
    {
        GestDemo_dssAssert(0);
    }

    /* wait until transfer done */
    do {
        if (EDMA_isTransferComplete(handle,
                config.paramSetConfig.transferCompletionCode,
                (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            GestDemo_dssAssert(0);
        }
    } while (isTransferDone == false);

    /* make sure to disable channel so it is usable later */
    EDMA_disableChannel(handle, config.channelId, config.channelType);
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Initialization on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
int32_t GestDemo_dssDataPathInit(void)
{
    int32_t retVal;
    SOC_SysIntListenerCfg  socIntCfg;
    int32_t errCode;
    GestDemo_DSS_dataPathContext_t *context;

    context = &gGestDssMCB.dataPathContext;

    GestDemo_DSS_DataPathObj *obj;

    obj = &gGestDssMCB.dataPathObj;
    GestDemo_dataPathObjInit(obj,
                            context,
                            &gGestDssMCB.cliCfg,
                            &gGestDssMCB.cliCommonCfg);
    GestDemo_dataPathInit1Dstate(obj);

    retVal = GestDemo_dataPathInitEdma(context);
    if (retVal < 0)
    {
        return -1;
    }

    /* Copy code from L3 to L1PSRAM, this code related to data path processing */
    GestDemo_copyTable(context->edmaHandle[0], &_GestDemo_fastCode_L1PSRAM_copy_table);

    retVal = GestDemo_dataPathAdcBufInit(context);
    if (retVal < 0)
    {
        return -1;
    }

    /* Register chirp interrupt listener */
    socIntCfg.systemInterrupt  = SOC_XWR16XX_DSS_INTC_EVENT_CHIRP_AVAIL;
    socIntCfg.listenerFxn      = GestDemo_dssChirpIntHandler;
    socIntCfg.arg              = (uintptr_t)NULL;
    if (SOC_registerSysIntListener(gGestDssMCB.socHandle, &socIntCfg, &errCode) == NULL)
    {
        System_printf("Error: Unable to register chirp interrupt listener , error = %d\n", errCode);
        return -1;
    }

    /* Register frame start interrupt listener */
    socIntCfg.systemInterrupt  = SOC_XWR16XX_DSS_INTC_EVENT_FRAME_START;
    socIntCfg.listenerFxn      = GestDemo_dssFrameStartIntHandler;
    socIntCfg.arg              = (uintptr_t)NULL;
    if (SOC_registerSysIntListener(gGestDssMCB.socHandle, &socIntCfg, &errCode) == NULL)
    {
        System_printf("Error: Unable to register frame start interrupt listener , error = %d\n", errCode);
        return -1;
    }

    /* Initialize detected objects logging */
    gGestDssMCB.loggingBufferAvailable = 1;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to configure ADCBUF driver based on CLI inputs.
 *  @param[in] ptrDataPathObj Pointer to data path object.
 *
 *  @retval
 *      0 if no error, else error (there will be system prints for these).
 */
static int32_t GestDemo_dssDataPathConfigAdcBuf(GestDemo_DSS_DataPathObj *ptrDataPathObj)
{
    ADCBuf_dataFormat   dataFormat;
    ADCBuf_RxChanConf   rxChanConf;
    int32_t             retVal;
    uint8_t             channel;
    uint8_t             numBytePerSample = 0;
    MMWave_OpenCfg*     ptrOpenCfg;
    uint32_t            chirpThreshold;
    uint32_t            rxChanMask = 0xF;
    GestDemo_DSS_dataPathContext_t *context = ptrDataPathObj->context;
    MmwDemo_ADCBufCfg   *adcBufCfg = &ptrDataPathObj->cliCfg->adcBufCfg;

    /* Get data path object and control configuration */
    ptrOpenCfg = &gGestDssMCB.cfg.openCfg;

    /*****************************************************************************
     * Data path :: ADCBUF driver Configuration
     *****************************************************************************/
    /* On XWR16xx only channel non-interleaved mode is supported */
    if(adcBufCfg->chInterleave != 1)
    {
        GestDemo_dssAssert(0); /* Not supported */
    }

    /* Populate data format from configuration */
    dataFormat.adcOutFormat       = adcBufCfg->adcFmt;
    dataFormat.channelInterleave  = adcBufCfg->chInterleave;
    dataFormat.sampleInterleave   = adcBufCfg->iqSwapSel;

    /* Disable all ADCBuf channels */
    if ((retVal = ADCBuf_control(context->adcbufHandle, ADCBufMMWave_CMD_CHANNEL_DISABLE, (void *)&rxChanMask)) < 0)
    {
       System_printf("Error: Disable ADCBuf channels failed with [Error=%d]\n", retVal);
       return retVal;
    }

    retVal = ADCBuf_control(context->adcbufHandle, ADCBufMMWave_CMD_CONF_DATA_FORMAT, (void *)&dataFormat);
    if (retVal < 0)
    {
        System_printf ("Error: MMWDemoDSS Unable to configure the data formats\n");
        return -1;
    }

    memset((void*)&rxChanConf, 0, sizeof(ADCBuf_RxChanConf));
    
    chirpThreshold = ptrDataPathObj->numChirpsPerChirpEvent;
    numBytePerSample = ptrDataPathObj->numBytePerSample;
    
    /* Enable Rx Channels */
    for (channel = 0; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        if(ptrOpenCfg->chCfg.rxChannelEn & (0x1U << channel))
        {
            /* Populate the receive channel configuration: */
            rxChanConf.channel = channel;
            retVal = ADCBuf_control(context->adcbufHandle, ADCBufMMWave_CMD_CHANNEL_ENABLE, (void *)&rxChanConf);
            if (retVal < 0)
            {
                System_printf("Error: MMWDemoDSS ADCBuf Control for Channel %d Failed with error[%d]\n", channel, retVal);
                return -1;
            }
            rxChanConf.offset  += ptrDataPathObj->numAdcSamples * numBytePerSample * chirpThreshold;
        }
    }

    /* Set ping/pong chirp threshold: */
    retVal = ADCBuf_control(context->adcbufHandle, ADCBufMMWave_CMD_SET_PING_CHIRP_THRESHHOLD,
                            (void *)&chirpThreshold);
    if(retVal < 0)
    {
        System_printf("Error: ADCbuf Ping Chirp Threshold Failed with Error[%d]\n", retVal);
        return -1;
    }
    retVal = ADCBuf_control(context->adcbufHandle, ADCBufMMWave_CMD_SET_PONG_CHIRP_THRESHHOLD,
                            (void *)&chirpThreshold);
    if(retVal < 0)
    {
        System_printf("Error: ADCbuf Pong Chirp Threshold Failed with Error[%d]\n", retVal);
        return -1;
    }

    return 0;
}


/**
 *  @b Description
 *  @n
 *      parses Profile, Chirp and Frame config and extracts parameters
 *      needed for processing chain configuration
         profileType: ACTIVE/IDLE profile
 */
bool GestDemo_parseProfileAndChirpConfig(GestDemo_DSS_DataPathObj *dataPathObj, uint8_t profileType)
{
    uint16_t        frameChirpStartIdx;
    uint16_t        frameChirpEndIdx;
    uint16_t        numLoops;
    uint32_t        chirpLoopIdx;
    bool            foundValidProfile = false;
    uint16_t        channelTxEn;
    uint8_t         channel;
    uint8_t         numRxChannels = 0;
    uint8_t         rxAntOrder [SYS_COMMON_NUM_RX_CHANNEL];
    uint8_t         txAntOrder [SYS_COMMON_NUM_TX_ANTENNAS];
    int32_t         i;
    int32_t         txIdx, rxIdx;
    /* we support only one profile in this processing chain */
    uint32_t        numTxAntAzim = 0;
    uint32_t        numTxAntElev = 0;
    uint32_t    mmWaveNumChirps = 0;
    bool        validProfileHasOneTxPerChirp = true;
    uint16_t    validProfileTxEn = 0;
    uint16_t    validChirpTxEnBits[32] = {0};
    rlChanCfg_t channelCfg;
    rlProfileCfg_t profileCfg = MmwaveLink_getProfileCfg(profileType);
    rlFrameCfg_t frameCfg = MmwaveLink_getFrameCfg(profileType);
    
    /* Get data path object and control configuration */
    channelCfg = gGestDssMCB.cfg.openCfg.chCfg;

    /* Get the Transmit channel enable mask: */
    channelTxEn = channelCfg.txChannelEn;
    
    /* Find total number of Rx Channels */
    for (channel = 0; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        rxAntOrder[channel] = 0;
        if(channelCfg.rxChannelEn & (0x1U << channel))
        {
            rxAntOrder[numRxChannels] = channel;
            /* Track the number of receive channels: */
            numRxChannels += 1;
        }
    }

    gIdleActiveCfgParams[profileType].numRxAntennas = numRxChannels;

    /* read frameCfg chirp start/stop*/
    numLoops = frameCfg.numLoops;

    frameChirpEndIdx = frameCfg.chirpEndIdx;
    frameChirpStartIdx = frameCfg.chirpStartIdx;
    mmWaveNumChirps   = frameChirpEndIdx - frameChirpStartIdx + 1;

    /* since validChirpTxEnBits is static array of 32 */
    GestDemo_dssAssert(mmWaveNumChirps <= 32);
    
    if(profileType == IDLE_PROFILE_CFG_SELECTED)
    {
        chirpLoopIdx = 0;
    }
    /* ACTIVE Profile selected: ACTIVE_PROFILE_SELECTED */
    else
    {
        chirpLoopIdx = 2;
    }
    
    /* loop for chirps and find if it has valid chirps for the frame
       looping around for all chirps in a profile, in case
       there are duplicate chirps
     */
    for (i=0; i < mmWaveNumChirps; i++)
    {
        rlChirpCfg_t chirpCfg = MmwaveLink_getChirpCfg(chirpLoopIdx++);
        uint16_t chirpTxEn = chirpCfg.txEnable;
        /* do chirps fall in range and has valid antenna enabled */
        if ((chirpCfg.chirpStartIdx >= frameChirpStartIdx) &&
            (chirpCfg.chirpEndIdx <= frameChirpEndIdx) &&
            ((chirpTxEn & channelTxEn) > 0))
        {
            uint16_t idx = 0;
            for (idx = (chirpCfg.chirpStartIdx - frameChirpStartIdx);
                 idx <= (chirpCfg.chirpEndIdx - frameChirpStartIdx); idx++)
            {
                validChirpTxEnBits[idx] = chirpTxEn;
                foundValidProfile = true;
                /* save the antennas actually enabled in this profile */
                validProfileTxEn |= chirpTxEn;
            }
        }
    }

    gIdleActiveCfgParams[profileType].validProfileIdx = profileCfg.profileId;

    if (validProfileTxEn & 0x1)
    {
        numTxAntAzim++;
    }
    if (validProfileTxEn & 0x2)
    {
        numTxAntAzim++;
    }

    gIdleActiveCfgParams[profileType].numTxAntennas = numTxAntAzim + numTxAntElev;
    gIdleActiveCfgParams[profileType].numVirtualAntAzim = numTxAntAzim * gIdleActiveCfgParams[profileType].numRxAntennas;
    gIdleActiveCfgParams[profileType].numVirtualAntElev = numTxAntElev * gIdleActiveCfgParams[profileType].numRxAntennas;
    gIdleActiveCfgParams[profileType].numVirtualAntennas = gIdleActiveCfgParams[profileType].numVirtualAntAzim + \
                                                    gIdleActiveCfgParams[profileType].numVirtualAntElev;
            
    /* Sanity Check: Ensure that the number of antennas is within system limits */
    GestDemo_dssAssert (gIdleActiveCfgParams[profileType].numVirtualAntennas > 0);
    GestDemo_dssAssert (gIdleActiveCfgParams[profileType].numVirtualAntennas <= (SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL));

    /* Copy the Rx channel compensation coefficients from common area to data path structure */
    if (validProfileHasOneTxPerChirp)
    {
        for (i = 0; i < gIdleActiveCfgParams[profileType].numTxAntennas; i++)
        {
            txAntOrder[i] = GestDemo_floorLog2(validChirpTxEnBits[i]);

        }
        for (txIdx = 0; txIdx < gIdleActiveCfgParams[profileType].numTxAntennas; txIdx++)
        {
            for (rxIdx = 0; rxIdx < gIdleActiveCfgParams[profileType].numRxAntennas; rxIdx++)
            {
                dataPathObj->compRxChanCfg.rxChPhaseComp[txIdx*gIdleActiveCfgParams[profileType].numRxAntennas + rxIdx] =
                        dataPathObj->cliCommonCfg->compRxChanCfg.rxChPhaseComp[txAntOrder[txIdx]*SYS_COMMON_NUM_RX_CHANNEL + rxAntOrder[rxIdx]];
            }
        }
    }
    else
    {
        cmplx16ImRe_t one;
        one.imag = 0;
        one.real = 0x7fff;
        for (txIdx = 0; txIdx < gIdleActiveCfgParams[profileType].numTxAntennas; txIdx++)
        {
            for (rxIdx = 0; rxIdx < gIdleActiveCfgParams[profileType].numRxAntennas; rxIdx++)
            {
                dataPathObj->compRxChanCfg.rxChPhaseComp[txIdx*gIdleActiveCfgParams[profileType].numRxAntennas + rxIdx] = one;
            }
        }
    }

    /* multiplicity of 4 due to windowing library function requirement */
    if ((profileCfg.numAdcSamples % 4) != 0)
    {
        System_printf("Number of ADC samples must be multiple of 4\n");
        GestDemo_dssAssert(0);
    }

    gIdleActiveCfgParams[profileType].numAdcSamples = profileCfg.numAdcSamples;
    gIdleActiveCfgParams[profileType].numRangeBins = GestDemo_pow2roundup(gIdleActiveCfgParams[profileType].numAdcSamples);
    gIdleActiveCfgParams[profileType].numChirpsPerFrame = (frameChirpEndIdx -frameChirpStartIdx + 1) *
                                                            numLoops;
    gIdleActiveCfgParams[profileType].numAngleBins = MMW_NUM_ANGLE_BINS;
    gIdleActiveCfgParams[profileType].numDopplerBins = gIdleActiveCfgParams[profileType].numChirpsPerFrame/gIdleActiveCfgParams[profileType].numTxAntennas;
    
    /* Multiplicity of 4 due to windowing library function requirement.
       Minimum size of 16 due to DSPLib restriction - FFT size must be bigger than 16.*/
    if (((gIdleActiveCfgParams[profileType].numDopplerBins % 4) != 0) ||
        (gIdleActiveCfgParams[profileType].numDopplerBins < 16))
    {
        System_printf("Number of Doppler bins must be at least 16 and it must be a multiple of 4.\n");
        GestDemo_dssAssert(0);
    }

#ifndef MMW_ENABLE_NEGATIVE_FREQ_SLOPE
    /** NOTE: not required for this demo, retained from mmw-demo app */
    /* Check frequency slope */
    if (profileCfg.freqSlopeConst < 0)
    {
        System_printf("Frequency slope must be positive\n");
        GestDemo_dssAssert(0);
    }
#endif
    gIdleActiveCfgParams[profileType].rangeResolution = MMWDEMO_SPEED_OF_LIGHT_IN_METERS_PER_SEC *
        profileCfg.digOutSampleRate * 1e3 /
        (2 * profileCfg.freqSlopeConst * ((3.6*1e3*900) /
         (1U << 26)) * 1e12 * gIdleActiveCfgParams[profileType].numRangeBins);
    gIdleActiveCfgParams[profileType].xyzOutputQFormat = (uint8_t) ceil(log10(16. /
        fabs(gIdleActiveCfgParams[profileType].rangeResolution))/log10(2));

    return foundValidProfile;
}

/**
 *  @b Description
 *  @n
 *      This function ADCBuf configuration and save info in datapath object to be used by 
 *   data path modules.
 */
void GestDemo_parseAdcBufCfg(GestDemo_DSS_DataPathObj *dataPathObj)
{
    uint8_t             numBytePerSample = 0;
    uint32_t            chirpThreshold;
    uint32_t            maxChirpThreshold;
    uint32_t            bytesPerChirp;
    MmwDemo_ADCBufCfg   *adcBufCfg = &dataPathObj->cliCfg->adcBufCfg;
    rlAdcOutCfg_t adcOutCfg = MmwaveLink_getAdcOutCfg();
    
    /* Check if ADC configuration is supported:*/
    /* ADC out bits: must be 16 Bits */
    GestDemo_dssAssert(adcOutCfg.fmt.b2AdcBits == 2);
    
    /* ADC data format: must be complex */
    /*adcCfg command*/
    if((adcOutCfg.fmt.b2AdcOutFmt != 1) &&
       (adcOutCfg.fmt.b2AdcOutFmt != 2))
    {
        GestDemo_dssAssert(0);
    }    
    /*adcbufCfg command*/
    GestDemo_dssAssert(adcBufCfg->adcFmt == 0);
    
    /* ADC channel interleave mode: must be non-interleaved */
    GestDemo_dssAssert(adcBufCfg->chInterleave == 1);
    
    /* Complex dataFormat has 4 bytes */
    numBytePerSample =  4;

    /* calculate max possible chirp threshold */
    bytesPerChirp = dataPathObj->numAdcSamples * dataPathObj->numRxAntennas * numBytePerSample;

    /* find maximum number of full chirps that can fit in the ADCBUF memory, while
       also being able to divide numChirpsPerFrame, we do not want remainder processing */
    maxChirpThreshold = SOC_XWR16XX_DSS_ADCBUF_SIZE / bytesPerChirp;
    
    /* There is a maximum of 8 CPs and CQs, so lets limit maxChirpThreshold to be that*/
    if(maxChirpThreshold > SYS_COMMON_CQ_MAX_CHIRP_THRESHOLD)
    {
        maxChirpThreshold = SYS_COMMON_CQ_MAX_CHIRP_THRESHOLD;
    }

    if (maxChirpThreshold >= dataPathObj->numChirpsPerFrame)
    {
        maxChirpThreshold = dataPathObj->numChirpsPerFrame;
    }
    else
    {
        /* find largest divisor of numChirpsPerFrame no bigger than maxChirpThreshold */
        while (dataPathObj->numChirpsPerFrame % maxChirpThreshold)
        {
            maxChirpThreshold--;
        }
    }

    /* ADCBuf control function requires argument alignment at 4 bytes boundary */
    chirpThreshold = adcBufCfg->chirpThreshold;

    /* if automatic, set to the calculated max */
    if (chirpThreshold == 0)
    {
        chirpThreshold = maxChirpThreshold;
    }
    else
    {
        if (chirpThreshold > maxChirpThreshold)
        {
            System_printf("Desired chirpThreshold %d higher than max possible of %d, setting to max\n",
                chirpThreshold, maxChirpThreshold);
            chirpThreshold = maxChirpThreshold;
        }
        else
        {
            /* check for divisibility of the user provided threshold */
            GestDemo_dssAssert((dataPathObj->numChirpsPerFrame % chirpThreshold) == 0);
        }
    }

    /* Save info in data path Object */
    dataPathObj->numChirpsPerChirpEvent = chirpThreshold;
    dataPathObj->numBytePerSample = numBytePerSample;
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Re-Configuration on DSS.
 *      This is used when switching between ACTIVE/IDLE Mode.
 *
 *  @retval
 *      -1 if error, 0 otherwise.
 */
static int32_t GestDemo_dssDataPathReconfig(GestDemo_DSS_DataPathObj *obj)
{
    int32_t retVal;

    retVal = GestDemo_dssDataPathConfigAdcBuf(obj);
    if (retVal < 0)
    {
        return -1;
    }

    GestDemo_dataPathConfigFFTs(obj);

    /* must be after GestDemo_dssDataPathConfigAdcBuf above as it calculates 
       numChirpsPerChirpEvent that is used in EDMA configuration */
    GestDemo_dataPathConfigEdma(obj);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Update parameters prehand required for ACTIVE/IDLE mode configuration.
 *
 *  @retval
 *      -1 if error, 0 otherwise.
 */
void GestDemo_UpdatePreParams(GestDemo_DSS_DataPathObj *dataPathObj, uint8_t profileType)
{
    dataPathObj->numRxAntennas = gIdleActiveCfgParams[profileType].numRxAntennas;
    dataPathObj->validProfileIdx = gIdleActiveCfgParams[profileType].validProfileIdx;

    dataPathObj->numTxAntennas       = gIdleActiveCfgParams[profileType].numTxAntennas;
    dataPathObj->numVirtualAntAzim   = gIdleActiveCfgParams[profileType].numVirtualAntAzim;
    dataPathObj->numVirtualAntElev   = gIdleActiveCfgParams[profileType].numVirtualAntElev;
    dataPathObj->numVirtualAntennas  = gIdleActiveCfgParams[profileType].numVirtualAntennas;

    dataPathObj->numAdcSamples       = gIdleActiveCfgParams[profileType].numAdcSamples;
    dataPathObj->numRangeBins        = gIdleActiveCfgParams[profileType].numRangeBins;
    dataPathObj->numChirpsPerFrame   = gIdleActiveCfgParams[profileType].numChirpsPerFrame;
    dataPathObj->numAngleBins        = MMW_NUM_ANGLE_BINS;
    dataPathObj->numDopplerBins      = gIdleActiveCfgParams[profileType].numDopplerBins;

            
    dataPathObj->rangeResolution = gIdleActiveCfgParams[profileType].rangeResolution;
    dataPathObj->xyzOutputQFormat = gIdleActiveCfgParams[profileType].xyzOutputQFormat;
}

/* Get the Max Heap allocation required out of ACTIVE/IDLE mode configuration, 
   so that Heap allocation is done only once */
void getMaxHeapAlloc(GestDemo_DSS_DataPathObj *obj)
{
    obj->numRangeBins = gIdleActiveCfgParams[ACTIVE_PROFILE_CFG_SELECTED].numRangeBins;
    obj->numDopplerBins = gIdleActiveCfgParams[ACTIVE_PROFILE_CFG_SELECTED].numDopplerBins;
    obj->numAngleBins = gIdleActiveCfgParams[ACTIVE_PROFILE_CFG_SELECTED].numAngleBins;
    obj->numRxAntennas = gIdleActiveCfgParams[ACTIVE_PROFILE_CFG_SELECTED].numRxAntennas;
    obj->numTxAntennas = gIdleActiveCfgParams[ACTIVE_PROFILE_CFG_SELECTED].numTxAntennas;
}

void restoreHeapAllocParams(GestDemo_DSS_DataPathObj *obj)
{
    obj->numRangeBins = gIdleActiveCfgParams[IDLE_PROFILE_CFG_SELECTED].numRangeBins;
    obj->numDopplerBins = gIdleActiveCfgParams[IDLE_PROFILE_CFG_SELECTED].numDopplerBins;
    obj->numAngleBins = gIdleActiveCfgParams[IDLE_PROFILE_CFG_SELECTED].numAngleBins;
    obj->numRxAntennas = gIdleActiveCfgParams[IDLE_PROFILE_CFG_SELECTED].numRxAntennas;
    obj->numTxAntennas = gIdleActiveCfgParams[IDLE_PROFILE_CFG_SELECTED].numTxAntennas;
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Configuration on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t GestDemo_dssDataPathConfig(uint8_t profileType)
{
    int32_t             retVal = 0;
    GestDemo_DSS_DataPathObj *dataPathObj;

    gGestDssMCB.numSubFrames = 1;
    

    dataPathObj  = &gGestDssMCB.dataPathObj;
    /*****************************************************************************
     * Data path :: Algorithm Configuration
     *****************************************************************************/
    /* Parse the profile and chirp configs and get the valid number of TX Antennas */
    GestDemo_UpdatePreParams(dataPathObj, profileType);

    if(!isHeapAllocated)
    {
        getMaxHeapAlloc(dataPathObj);
        /* Data path configurations */
        GestDemo_dataPathConfigBuffers(dataPathObj, SOC_XWR16XX_DSS_ADCBUF_BASE_ADDRESS, &gfeatures);
        GestDemo_dataPathComputeDerivedConfig(dataPathObj);
#if !(DISABLE_POWER_SAVING_MODE)
        restoreHeapAllocParams(dataPathObj);
#endif
        isHeapAllocated = 1;
        
        /* Find out number of chirp per chirp interrupt
           It will be used for both ADCBuf config and CQ config
         */
        GestDemo_parseAdcBufCfg(dataPathObj);
    }

    /* Below configurations are to be reconfigured every sub-frame so do only for first one */
    retVal = GestDemo_dssDataPathReconfig(dataPathObj);
    if (retVal < 0)
    {
        return -1;
    }
    
    /* If profile switch is required then post an event to configure other profile */
    if(isSwitchProfile)
    {
        /* start the frame with different profile */
        Event_post(gGestDssMCB.eventHandle, GESTDEMO_START_EVT);
    }

    return 0;
}

uint32_t profileResetTimeStamp[10];

/**
 *  @b Description
 *  @n
 *      Function to start Data Path on DSS.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t GestDemo_dssDataPathStart(uint8_t profileType)
{
    int32_t    errCode;

    gGestDssMCB.dataPathContext.chirpProcToken = 0;
    gGestDssMCB.dataPathContext.interFrameProcToken = 0;

    /* If Profile Switch event raised then wait for SensorStop Async Event before next sensorStart */
    if(isSwitchProfile)
    {
        MmwaveLink_waitFrameEndAE();
        isSwitchProfile = false;            
    }

    /* FrameConfig with Idle/Active ProfileConfig */
    /* mmWave Link set Frame configuration: start with IDLE configuration */
    if((errCode = MmwaveLink_setFrameConfig(profileType)) != 0)
    {
        System_printf ("mmWaveLink Frame config failed! [%d]", errCode);
        return errCode;
    }

    /* Trigger the Frame. */
    if ((errCode = MmwaveLink_sensorStart()) < 0)
    {
        /* Error: Unable to start the mmWave control */
        System_printf ("Error: frame trigger failed [Error code %d]\n", errCode);
        return errCode;
    }

    profileResetTimeStamp[3] = Cycleprofiler_getTimeStamp();
    
    return 0;
}

int32_t checkProfileSwitchCond(float energySum)
{
#if !(DISABLE_POWER_SAVING_MODE)
    gEnergySum = energySum;
    /* Switch IDLE to ACTIVE Immediately if any activity/object */
    if((gDevPowerState == IDLE_PROFILE_MODE) && (((energySum) > THRESHOLD_TO_SWITCH_PROFILE) || \
    ((-1*energySum) > THRESHOLD_TO_SWITCH_PROFILE)))
    {
        gDevPowerState = ACTIVE_PROFILE_MODE;
        isSwitchProfile = true;
        frameCountToSwitch = 0U;
    }
    /* if currently in ACTIVE mode and no activity/object then wait for few frame with no activity
       then switch to IDLE mode */
    else if(gDevPowerState == ACTIVE_PROFILE_MODE)
    {
         if(energySum < THRESHOLD_TO_SWITCH_PROFILE)
         {
             frameCountToSwitch++;
             if(frameCountToSwitch > WAIT_FRAME_TO_IDLE_PROFILE)
             {
                 gDevPowerState = IDLE_PROFILE_MODE;
                 isSwitchProfile = true;
             }
         }
         else
         {
             frameCountToSwitch = 0U;
         }
    }
    
    /* Decide if we need to stop frame and set new profileType */
    if(isSwitchProfile)
    {
        /* If need to stop the frame then reset the frameProcess counters,
        in this case we are not shipping the data to MSS    */
        gGestDssMCB.dataPathContext.interFrameProcToken = 0;
        
        GestDemo_bssStopDone();
        
        gProfileType = (~gProfileType & 0x01U);
        
        profileResetTimeStamp[0] = Cycleprofiler_getTimeStamp();
        return 0xFF;
    }
    else
    {
        return 0;
    }
#else
        return 0;
#endif
}
/**
 *  @b Description
 *  @n
 *      Function to process Data Path events at runtime.
 *
 *  @param[in]  event
 *      Data Path Event
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t GestDemo_dssDataPathProcessEvents(UInt event, uint16_t fcounter, float gesture_pktno)
{
    GestDemo_DSS_DataPathObj *dataPathObj;
    volatile uint32_t startTime;

    dataPathObj = &gGestDssMCB.dataPathObj;

    /* Handle dataPath events */
    switch(event)
    {
        case GESTDEMO_CHIRP_EVT:
            /* Increment event stats */
            gGestDssMCB.stats.chirpEvt++;
            {
                uint16_t chirpIndex;
                
                for (chirpIndex = 0; chirpIndex < dataPathObj->numChirpsPerChirpEvent; chirpIndex++)
                {
                    GestDemo_processChirp(dataPathObj, (uint16_t) chirpIndex);
                }
            }
            gGestDssMCB.dataPathContext.chirpProcToken--;
            dataPathObj->timingInfo.chirpProcessingEndTime = Cycleprofiler_getTimeStamp();

            if (dataPathObj->chirpCount == 0)
            {
                float energySum;
                GestDemo_waitEndOfChirps(dataPathObj);
                Load_update();
                dataPathObj->timingInfo.activeFrameCPULoad = Load_getCPULoad();

                dataPathObj->cycleLog.interChirpProcessingTime = gCycleLog.interChirpProcessingTime;
                dataPathObj->cycleLog.interChirpWaitTime = gCycleLog.interChirpWaitTime;
                gCycleLog.interChirpProcessingTime = 0;
                gCycleLog.interChirpWaitTime = 0;

                startTime = Cycleprofiler_getTimeStamp();
                energySum = GestDemo_interFrameProcessingPart1(dataPathObj, fcounter);
                
                /* if need to switch the profile */
                if(checkProfileSwitchCond(energySum) > 0)
                {
                    break;
                }
                else
                {
                    /* If IDLE_PROFILE_MODE then don't call InterFrame Part2 processing */
                    GestDemo_interFrameProcessingPart2(dataPathObj);
                }
                
                dataPathObj->timingInfo.interFrameProcCycles = (Cycleprofiler_getTimeStamp() - startTime);
#ifdef DBG
                if (gChirpTimeStampIdx == NUM_CHIRP_TIME_STAMPS)
                {
                    gChirpTimeStampIdx = 0;
                }
                gFrameProcessTime[gChirpTimeStampIdx++] = dataPathObj->timingInfo.interFrameProcCycles;
#endif
                dataPathObj->cycleLog.interFrameProcessingTime = gCycleLog.interFrameProcessingTime;
                dataPathObj->cycleLog.interFrameWaitTime = gCycleLog.interFrameWaitTime;
                gCycleLog.interFrameProcessingTime = 0;
                gCycleLog.interFrameWaitTime = 0;

                /* Sending range bias and Rx channel phase offset measurements to MSS and from there to CLI */
                if(dataPathObj->cliCommonCfg->measureRxChanCfg.enabled)
                {
                    GestDemo_measurementResultOutput (dataPathObj);
                }

                /* Sending detected objects to logging buffer */
                GestDemo_dssDataPathOutputLogging (dataPathObj, gesture_pktno);
                dataPathObj->timingInfo.interFrameProcessingEndTime = Cycleprofiler_getTimeStamp();

#ifdef DBG
    if (gChirpTimeStampIdx == NUM_CHIRP_TIME_STAMPS)
    {
        gChirpTimeStampIdx = 0;
    }

    gFrameProcessTime[gChirpTimeStampIdx++] = dataPathObj->timingInfo.interFrameProcessingEndTime - startTime;
#endif
            }
            break;

        case GESTDEMO_FRAMESTART_EVT:
            /* Increment event stats */
            gGestDssMCB.stats.frameStartEvt++;
            Load_update();
            dataPathObj->timingInfo.interFrameCPULoad = Load_getCPULoad();
            GestDemo_dssAssert(dataPathObj->chirpCount == 0);
            break;

        case GESTDEMO_BSS_FRAME_TRIGGER_READY_EVT:
            /* Increment event stats */
            gGestDssMCB.stats.frameTrigEvt++;
            break;

        default:
            break;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to stop Data Path on DSS. Assume BSS has been stopped by mmWave already.
 *      This also sends the STOP done message back to MSS to signal the procssing
 *      chain has come to a stop.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t GestDemo_dssDataPathStop(void)
{
    uint32_t numFrameEvtsHandled = gGestDssMCB.stats.frameStartEvt;


    /* move to stop state */
    gGestDssMCB.state = GestDemo_DSS_STATE_STOPPED;

    /* Update stats */
    gGestDssMCB.stats.chirpEvt = 0;
    gGestDssMCB.stats.frameStartEvt = 0;
    gGestDssMCB.stats.frameTrigEvt = 0;
    gGestDssMCB.stats.numFailedTimingReports = 0;
    gGestDssMCB.stats.numCalibrationReports = 0;

    profileResetTimeStamp[1] = Cycleprofiler_getTimeStamp();
    MmwaveLink_sensorStop();
    profileResetTimeStamp[2] = Cycleprofiler_getTimeStamp();
    if(isSwitchProfile)
    {
        /* start the frame with different profile */
        Event_post(gGestDssMCB.eventHandle, GESTDEMO_CONFIG_EVT);
        /* notify MSS about Profile Switching */
        GestDemo_notifyMssProfileType();
    }
    System_printf ("Debug: MMWDemoDSS Data Path stop succeeded stop%d,frames:%d \n",
                    gGestDssMCB.stats.stopEvt,numFrameEvtsHandled);

    return 0;
}


/**
 *  @b Description
 *  @n
 *      Data Path main task that handles events from remote and do dataPath processing.
 *  This task is created when MSS is responsible for the mmwave Link and DSS is responsible
 *  for data path processing.
 *
 *  @retval
 *      Not Applicable.
 */
static void GestDemo_dssDataPathTask (UArg arg0, UArg arg1)
{
    int32_t       retVal = 0;
    UInt          event;
    uint16_t      framecounter = 0;
    float         gesture_pktno = 0;
    
    /************************************************************************
     * Data Path :: Main loop
     ************************************************************************/
    while (1)
    {
        event = Event_pend(gGestDssMCB.eventHandle,
                          Event_Id_NONE,
                          GESTDEMO_FRAMESTART_EVT | GESTDEMO_CHIRP_EVT |
                          GESTDEMO_BSS_STOP_COMPLETE_EVT | GESTDEMO_CONFIG_EVT |
                          GESTDEMO_STOP_COMPLETE_EVT | GESTDEMO_START_EVT, 
                          BIOS_WAIT_FOREVER);


        if(event & GESTDEMO_BSS_STOP_COMPLETE_EVT)
        {
            GestDemo_bssStopDone();
        }

        /************************************************************************
         * Data Path process frame start event
         ************************************************************************/
        if(event & GESTDEMO_FRAMESTART_EVT)
        {
            if((gGestDssMCB.state == GestDemo_DSS_STATE_STARTED) || (gGestDssMCB.state == GestDemo_DSS_STATE_STOP_PENDING))
            {
                gesture_pktno +=1;
                if(framecounter >= (GESTURE_FEATURE_CONSTANT - (GESTURE_FEATURE_CONSTANT % GESTURE_FEATURE_LENGTH)))
                {
                    framecounter = 0; // reset the global frame counter to prevent corner case condition
                }
                
                if ((retVal = GestDemo_dssDataPathProcessEvents(GESTDEMO_FRAMESTART_EVT, framecounter, gesture_pktno)) < 0 )
                {
                    System_printf ("Error: MMWDemoDSS Data Path process frame start event failed with Error[%d]\n",
                                  retVal);
                }
                else
                {
                    framecounter +=1U;
                }
            }
        }

        /************************************************************************
         * Data Path process chirp event
         ************************************************************************/
        if(event & GESTDEMO_CHIRP_EVT)
        {
            if((gGestDssMCB.state == GestDemo_DSS_STATE_STARTED) || (gGestDssMCB.state == GestDemo_DSS_STATE_STOP_PENDING))
            {
                if ((retVal = GestDemo_dssDataPathProcessEvents(GESTDEMO_CHIRP_EVT, framecounter, gesture_pktno)) < 0 )
                {
                    System_printf ("Error: MMWDemoDSS Data Path process chirp event failed with Error[%d]\n",
                                  retVal);
                }
            }
        }

        /************************************************************************
         * Data Path re-config, only supported reconfiguration in stop state
         ************************************************************************/
        if(event & GESTDEMO_CONFIG_EVT)
        {
            if ((gGestDssMCB.state == GestDemo_DSS_STATE_STOPPED) || (gGestDssMCB.state == GestDemo_DSS_STATE_INIT))
            {
                /* configure RadarSS with Basic configurations only once */
                if(gGestDssMCB.isRadarSSConfigured == false)
                {
                    /* Configure the RadarSS */
                    if ((retVal = MmwaveLink_configRadarSS()) < 0)
                    {
                        System_printf ("Debug: MMWDemoDSS RadarSS config failed with Error[%d]\n",retVal);
                        goto exit;
                    }
                    gGestDssMCB.isRadarSSConfigured = true;
                }
                if ((retVal = GestDemo_dssDataPathConfig(gProfileType)) < 0 )
                {
                    System_printf ("Debug: MMWDemoDSS Data Path config failed with Error[%d]\n",retVal);
                    goto exit;
                }
                gGestDssMCB.state = GestDemo_DSS_STATE_CONFIG;
            }
            else
            {
                System_printf ("Error: MMWDemoDSS Data Path config event in wrong state[%d]\n", gGestDssMCB.state);
                goto exit;
            }
        }

        /************************************************************************
         * Quick start after stop
         ************************************************************************/
        if(event & GESTDEMO_START_EVT)
        {
            /* If state is STOPPED then we need to do the frame-config for Idle/Active mode 
                and sensor-start again */
            if(gGestDssMCB.state == GestDemo_DSS_STATE_STOPPED)
            {
                /* RF start is done by MSS in this case; so just do DSS start */
                if ((retVal = GestDemo_dssDataPathStart(gProfileType)) < 0 )
                {
                    System_printf ("Error: MMWDemoDSS Data Path start failed with Error[%d]\n",retVal);
                    goto exit;
                }
                gGestDssMCB.state = GestDemo_DSS_STATE_STARTED;
            }
            /* State is CONFIG i.e. first time config with IDLE configuration */
            else if(gGestDssMCB.state == GestDemo_DSS_STATE_CONFIG)
            {
                /* RF start is done by MSS in this case; so just do DSS start */
                if ((retVal = GestDemo_dssDataPathStart(gProfileType)) < 0 )
                {
                    System_printf ("Error: MMWDemoDSS Data Path start failed with Error[%d]\n",retVal);
                    goto exit;
                }
                gGestDssMCB.state = GestDemo_DSS_STATE_STARTED;
            }
            else
            {
                System_printf ("Error: MMWDemoDSS Data Path config event in wrong state[%d]\n", gGestDssMCB.state);
                goto exit;
            }
        }

        /************************************************************************
         * Data Path process frame start event
         ************************************************************************/
        if(event & GESTDEMO_STOP_COMPLETE_EVT)
        {
            if (gGestDssMCB.state == GestDemo_DSS_STATE_STOP_PENDING)
            {
                /************************************************************************
                 * Local Data Path Stop
                 ************************************************************************/
                if ((retVal = GestDemo_dssDataPathStop()) < 0 )
                {
                    System_printf ("Debug: MMWDemoDSS Data Path stop failed with Error[%d]\n",retVal);
                }
            }
        }
    }
exit:
    System_printf("Debug: MMWDemoDSS Data path exit\n");
    while(1);
}

void GestDemo_InitDefaultConfig (void)
{
    GestDemo_DSS_DataPathObj *dataPathObj;
    
    /* single frame */
    dataPathObj  = &gGestDssMCB.dataPathObj;
    
    /* copy all the default configurations to global structure */
    gGestDssMCB.cfg.openCfg.chCfg = MmwaveLink_getChannelCfg();
    
    gGestDssMCB.cfg.openCfg.lowPowerMode = MmwaveLink_getLowPwrCfg();
    
    gGestDssMCB.cfg.openCfg.adcOutCfg = MmwaveLink_getAdcOutCfg();
    
    gGestDssMCB.cfg.ctrlCfg.dfeDataOutputMode = MMWave_DFEDataOutputMode_FRAME;
    //gGestDssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg = MmwaveLink_getFrameCfg(IDLE_PROFILE_CFG_SELECTED);
   

   GestDemo_parseProfileAndChirpConfig(dataPathObj, IDLE_PROFILE_CFG_SELECTED);
   GestDemo_parseProfileAndChirpConfig(dataPathObj, ACTIVE_PROFILE_CFG_SELECTED);
   
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
static void GestDemo_dssInitTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    Task_Params         taskParams;
    Semaphore_Params    semParams;
    Mailbox_Config      mboxCfg;
    Error_Block         eb;
    GestDemo_message     message;

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_DSS);

    /*****************************************************************************
     * Open & configure the drivers:
     *****************************************************************************/

    /*****************************************************************************
     * Create mailbox Semaphore:
     *****************************************************************************/
    /* Create a binary semaphore which is used to handle mailbox interrupt. */
    Semaphore_Params_init(&semParams);
    semParams.mode             = Semaphore_Mode_BINARY;
    gGestDssMCB.mboxSemHandle = Semaphore_create(0, &semParams, NULL);

    /* Setup the default mailbox configuration */
    Mailbox_Config_init(&mboxCfg);

    /* Setup the configuration: */
    mboxCfg.chType       = MAILBOX_CHTYPE_MULTI;
    mboxCfg.chId         = MAILBOX_CH_ID_0;
    mboxCfg.writeMode    = MAILBOX_MODE_BLOCKING;
    mboxCfg.readMode     = MAILBOX_MODE_CALLBACK;
    mboxCfg.readCallback = &GestDemo_mboxCallback;

    /* DSS to MSS mailbox */
    gGestDssMCB.peerMailbox = Mailbox_open(MAILBOX_TYPE_MSS, &mboxCfg, &errCode);
    if (gGestDssMCB.peerMailbox == NULL)
    {
        /* Error: Unable to open the mailbox */
        System_printf("Error: Unable to open the Mailbox to the DSS [Error code %d]\n", errCode);
        return;
    }
    /* Remote Mailbox is operational: Set the DSS Link State */
    SOC_setMMWaveDSSLinkState (gGestDssMCB.socHandle, 1U, &errCode);

     /* Debug Message: */
    System_printf("Debug: DSS Mailbox Handle %p\n", gGestDssMCB.peerMailbox);

    /* Create task to handle mailbox messges */
    Task_Params_init(&taskParams);
    Task_create(GestDemo_mboxReadTask, &taskParams, NULL);

    /* sync with MSS */
    SOC_setMMWaveDSSLinkState(gGestDssMCB.socHandle, 1U, &errCode);
    
    /*****************************************************************************
     * Create Event to handle mmwave callback and system datapath events
     *****************************************************************************/
    /* Default instance configuration params */
    Error_init(&eb);
    gGestDssMCB.eventHandle = Event_create(NULL, &eb);
    if (gGestDssMCB.eventHandle == NULL)
    {
        /* FATAL_TBD */
        System_printf("Error: MMWDemoDSS Unable to create an event handle\n");
        return ;
    }
    System_printf("Debug: MMWDemoDSS create event handle succeeded\n");

    /* initialize mmWaveLink */
    MmwaveLink_initLink(RL_AR_DEVICETYPE_16XX, RL_PLATFORM_DSS);

    /* mmWave module has been opened. */
    gGestDssMCB.isMMWaveLinkOpen = true;

    /******************************************************************************
     * TEST: Synchronization
     * - The synchronization API always needs to be invoked.
     ******************************************************************************/
    while (1)
    {
        int32_t syncStatus;
        /* Get the synchronization status: */
        syncStatus = SOC_isMMWaveMSSOperational (gGestDssMCB.socHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the mmWave control module */
            System_printf ("Error: MMWDemoDSS mmWave Control Synchronization failed [Error code %d]\n", errCode);
            return;
        }
        if (syncStatus == 1)
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        //MmwDemo_sleep();
    }
    
    /* Send the DSS to MSS signalling ISR payload address to the DSS. Note this
       should be done after both sides mailboxes have been opened, and because
       MMWave_sync above is a good one to check for synchronization, this is a good place */
    message.type = GESTDEMO_DSS2MSS_ISR_INFO_ADDRESS;
    message.body.dss2mssISRinfoAddress = (uint32_t) &gHSRAM.dss2MssIsrInfo;
    GestDemo_mboxWrite(&message);
    
    /*****************************************************************************
     * Data path Startup
     *****************************************************************************/
    if ((errCode = GestDemo_dssDataPathInit()) < 0 )
    {
        System_printf ("Error: MMWDemoDSS Data Path init failed with Error[%d]\n",errCode);
        return;
    }
    System_printf ("Debug: MMWDemoDSS Data Path init succeeded\n");
    
    /* Initialize all the default configurations from the header file */
    GestDemo_InitDefaultConfig();
    
    gGestDssMCB.state = GestDemo_DSS_STATE_INIT;

    /* Start data path task */
    Task_Params_init(&taskParams);
    taskParams.priority = 5;
    taskParams.stackSize = 4*1024;
    Task_create(GestDemo_dssDataPathTask, &taskParams, NULL);

    System_printf("Debug: MMWDemoDSS initTask exit\n");
    return;
}

/**
 *  @b Description
 *  @n
 *     Function to sleep the DSP using IDLE instruction. When DSP has no work left to do,
 *     the BIOS will be in Idle thread and will call this function. The DSP will
 *     wake-up on any interrupt (e.g chirp interrupt).
 *
 *  @retval
 *      Not Applicable.
 */
void GestDemo_sleep(void)
{
    /* issue IDLE instruction */
    asm(" IDLE ");
}

/**
 *  @b Description
 *  @n
 *      Sends DSS assert information to MSS
 *
 *  @retval
 *      Not Applicable.
 */
void _GestDemo_dssAssert(int32_t expression,const char *file, int32_t line)
{
    GestDemo_message  message;
    uint32_t         nameSize;

    if (!expression) 
    {
        message.type = GESTDEMO_DSS2MSS_ASSERT_INFO;
        nameSize = strlen(file);
        if(nameSize > GESTDEMO_MAX_FILE_NAME_SIZE)
            nameSize = GESTDEMO_MAX_FILE_NAME_SIZE;
            
        memcpy((void *) &message.body.assertInfo.file[0], (void *)file, nameSize);
        message.body.assertInfo.line = (uint32_t)line;
        if (GestDemo_mboxWrite(&message) != 0)
        {
            System_printf ("Error: Failed to send exception information to MSS.\n");
        }
        
    }    
}        

/**
 *  @b Description
 *  @n
 *      Sends DSS range bias and rx channel phase offset measurement information to MSS
 *
 *  @retval
 *      Not Applicable.
 */
void GestDemo_measurementResultOutput(GestDemo_DSS_DataPathObj *obj)
{
    GestDemo_message  message;

    message.type = GESTDEMO_DSS2MSS_MEASUREMENT_INFO;

    memcpy((void *) &message.body.compRxChanCfg, (void *)&obj->cliCommonCfg->compRxChanCfg, sizeof(MmwDemo_compRxChannelBiasCfg_t));
    if (GestDemo_mboxWrite(&message) != 0)
    {
        System_printf ("Error: Failed to send measurement information to MSS.\n");
    }
}

/**
 *  @b Description
 *  @n
 *      Entry point into the test code.
 *
 *  @retval
 *      Not Applicable.
 */
int main (void)
{
    Task_Params    taskParams;
    SOC_Cfg        socCfg;
    int32_t        errCode;

    /* Initialize and populate the demo MCB */
    memset ((void*)&gGestDssMCB, 0, sizeof(GestDemo_DSS_MCB));

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_BYPASS_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    gGestDssMCB.socHandle = SOC_init (&socCfg, &errCode);
    if (gGestDssMCB.socHandle == NULL)
    {
        System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the DEMO configuration: */
    gGestDssMCB.cfg.sysClockFrequency = DSS_SYS_VCLK;
    gGestDssMCB.cfg.loggingBaudRate   = 921600;

    Cycleprofiler_init();

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 3*1024;
    Task_create(GestDemo_dssInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}


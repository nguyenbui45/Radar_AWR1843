/*
 *   @file  dss_main.c
 *
 *   @brief
 *      This is the implementation of the DSS MRR TI Design 
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
#include <ti/sysbios/family/c64p/Cache.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>
#include <ti/sysbios/utils/Load.h>

/* mmWave SDK Include Files: */
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/drivers/adcbuf/ADCBuf.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/drivers/soc/soc.h>
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

/* Application Include Files: */
#include "dss_mrr.h"
#include "dss_data_path.h"
#include "../common/mrr_config_consts.h"
#include "../common/mmw_messages.h"
#include "config_hwa_util.h"

/* Tracking. */
#include "EKF_XYZ_Interface.h"

/* Related to linker copy table for copying from L3 to L1PSRAM for example */
#include <cpy_tbl.h>

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/
/**
 * @brief
 *  DSS stores demo output and DSS to MSS ISR information (for fast exception 
 *  signalling) in HSRAM.
 */
/*!   */
#define SOC_XWR18XX_DSS_HSRAM_SIZE                  0x8000U        /* Size: 32KB */
typedef struct MmwDemo_HSRAM_t_ {
#define MMW_DATAPATH_DET_PAYLOAD_SIZE (SOC_XWR18XX_DSS_HSRAM_SIZE -  sizeof(uint8_t))
    /*! @brief data path processing/detection related message payloads, these
               messages are signalled through DSS to MSS mailbox */ 
    uint8_t  dataPathDetectionPayload[MMW_DATAPATH_DET_PAYLOAD_SIZE];

    /*! @brief Information relayed through DSS triggering software interrupt to
               MSS. It stores one of the exception IDs @ref DSS_TO_MSS_EXCEPTION_IDS */
    uint8_t  dss2MssIsrInfo;
} MmwDemo_HSRAM_t;

#pragma DATA_SECTION(gHSRAM, ".demoSharedMem");
#pragma DATA_ALIGN(gHSRAM, 4);
MmwDemo_HSRAM_t gHSRAM;

/*! @brief Azimuth FFT size */
#define MMW_NUM_ANGLE_BINS 64

#define MMWDEMO_SPEED_OF_LIGHT_IN_METERS_PER_SEC (3.0e8)

#define MMWDEMO_SPEED_OF_LIGHT_IN_METERS_PER_USEC (3.0e2) 

extern far COPY_TABLE _MmwDemo_fastCode_L1PSRAM_copy_table;
/**
 * @brief
 *  Global Variable for tracking information required by the design
 */
Mrr_DSS_MCB gMrrDSSMCB;

extern mmwHwaBuf_t gMmwHwaMemBuf[MMW_HWA_NUM_MEM_BUFS];

volatile cycleLog_t gCycleLog;

volatile int32_t hwa_1d_done_flag;
volatile int32_t edma_1d_done_flag;
volatile int32_t edma_2d_done_flag;

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/
//static void MRR_DSS_chirpIntCallback(uintptr_t arg);
static void MRR_DSS_frameStartIntCallback(uintptr_t arg);
static int32_t MRR_DSS_dssDataPathConfigAdcBuf();

/* Functions to configure the processing path. */
void MmwDemo_dataPathConfigPopulate(MmwDemo_DSS_DataPathObj* obj);
void MmwDemo_populateMRR(MmwDemo_DSS_DataPathObj* obj, uint16_t subframeIndx);
void MmwDemo_populateUSRR(MmwDemo_DSS_DataPathObj* obj, uint16_t subframeIndx);
void MmwDemo_dBScanConfigBuffers(MmwDemo_DSS_DataPathObj * obj);


/* Copy table related */
static void MmwDemo_edmaBlockCopy(EDMA_Handle handle, uint32_t loadAddr, uint32_t runAddr, uint16_t size);
static void MmwDemo_copyTable(EDMA_Handle handle, COPY_TABLE *tp);

/* Tasks. */
static void MRR_DSS_initTask(UArg arg0, UArg arg1);
static void MRR_DSS_mmWaveTask(UArg arg0, UArg arg1);

/*  DataPath Functions */
int32_t MRR_DSS_DataPathInit(void);

/* Internal Interrupt handler */
//static void MRR_DSS_chirpIntCallback(uintptr_t arg);
static void MRR_DSS_frameStartIntCallback(uintptr_t arg);

/* Output logging.  */
static int32_t MRR_DSS_SendProcessOutputToMSS(uint8_t *ptrHsmBuffer,
                                                 uint32_t outputBufSize,
                                                 MmwDemo_DSS_DataPathObj *obj);
static void MRR_DSS_DataPathOutputLogging(MmwDemo_DSS_DataPathObj * dataPathObj);
static void MmwDemo_mboxReadProc();
void MmwDemo_mboxCallback(Mbox_Handle handle, Mailbox_Type peer);

/* external sleep function when in idle (used in .cfg file) */
void MmwDemo_sleep(void);

/* Utility functions. */
uint16_t convertSNRdBtoThreshold(uint16_t numInteg, float ThresholdIndB, uint16_t bitwidth);
static int32_t MmwDemo_mboxWrite(MmwDemo_message *message);
/**************************************************************************
 ********************** DSS MRR TI Design Functions ***********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      This is the callback function registered with the ADC Driver which is invoked
 *      when a chirp is available. This is executed in the ISR context.
 *
 *  @param[in]  arg
 *      Application registered argument
 *
 *  @retval
 *      Not Applicable.
 */
static void MRR_DSS_chirpIntCallback(uintptr_t arg)
{
    gMrrDSSMCB.chirpProcToken = 1;
    gMrrDSSMCB.stats.chirpIntCounter++;
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the callback function registered when a frame is triggered. 
 *      This is executed in the ISR context.
 *
 *  @param[in]  arg
 *      Application registered argument
 *
 *  @retval
 *      Not Applicable.
 */
static void MRR_DSS_frameStartIntCallback(uintptr_t arg)
{
    gMrrDSSMCB.frameStartIntToken = 1; 
    gMrrDSSMCB.stats.frameStartIntCounter++;
    return;
}


/**
 *  @b Description
 *  @n
 *      This is the task which provides an execution context for
 *      the mmWave control module.
 *
 *  @param[in]  arg0
 *      Argument0 with which the task was created
 *  @param[in]  arg1
 *      Argument1 with which the task was created
 *
 *  @retval
 *      Not Applicable.
 */
static void MRR_DSS_mmWaveTask(UArg arg0, UArg arg1)
{
    volatile uint32_t startTime;
    MmwDemo_DSS_DataPathObj *dataPathObj;

    // Begin with MRR
    gMrrDSSMCB.subframeIndx = 0;
    dataPathObj = &gMrrDSSMCB.dataPathObj[gMrrDSSMCB.subframeIndx];

    while (1)
    {
        if (gMrrDSSMCB.frameStartIntToken == 1)
        {
            gMrrDSSMCB.frameStartIntToken = 0;

            /* Increment event stats */
            gMrrDSSMCB.stats.frameStartEvt++;

            /* Check if the previous frame has been completely processed. */
//            MmwDemo_dssAssert(dataPathObj->chirpCount != 0);
        }
        else if (gMrrDSSMCB.chirpProcToken == 1)
        {
            /* We have begun processing a frame */
            gMrrDSSMCB.frameProcToken = 1;

            /* Increment event stats */
            gMrrDSSMCB.stats.chirpEvt++;
            MmwDemo_processChirp(dataPathObj, gMrrDSSMCB.subframeIndx);
            gMrrDSSMCB.chirpProcToken = 0;

            dataPathObj->timingInfo.chirpProcessingEndTime = Cycleprofiler_getTimeStamp();

            if (dataPathObj->chirpCount == 0)
            {
                MmwDemo_waitEndOfChirps();

                dataPathObj->cycleLog.interChirpProcessingTime = gCycleLog.interChirpProcessingTime;
                dataPathObj->cycleLog.interChirpWaitTime = gCycleLog.interChirpWaitTime;
                gCycleLog.interChirpProcessingTime = 0;
                gCycleLog.interChirpWaitTime = 0;

                startTime = Cycleprofiler_getTimeStamp();
                MmwDemo_interFrameProcessing(dataPathObj, gMrrDSSMCB.subframeIndx);
                dataPathObj->timingInfo.interFrameProcCycles = Cycleprofiler_getTimeStamp() - startTime;

                dataPathObj->cycleLog.interFrameProcessingTime = gCycleLog.interFrameProcessingTime;
                dataPathObj->cycleLog.interFrameWaitTime = gCycleLog.interFrameWaitTime;
                gCycleLog.interFrameProcessingTime = 0;
                gCycleLog.interFrameWaitTime = 0;

                /* Sending detected objects to logging buffer */
                MRR_DSS_DataPathOutputLogging(dataPathObj);
                dataPathObj->timingInfo.interFrameProcessingEndTime = Cycleprofiler_getTimeStamp();

                /* Update the subframeIndx */
                {
                    gMrrDSSMCB.subframeIndx ++;
                    if (gMrrDSSMCB.subframeIndx >= NUM_SUBFRAMES)
                    {
                        gMrrDSSMCB.subframeIndx = 0;
                    }
                    
                    /* Program the ADC for the next subframe. */
                    dataPathObj = &gMrrDSSMCB.dataPathObj[gMrrDSSMCB.subframeIndx];                    
                }
                /* Prepare HWA+EDMA for the next frame start */
                MmwDemo_config1D_HWA(dataPathObj, gMrrDSSMCB.subframeIndx);
                MmwDemo_dataPathTrigger1D(dataPathObj, gMrrDSSMCB.subframeIndx);
            }

            gMrrDSSMCB.frameProcToken = 0;
        }

//        else if (gMrrDSSMCB.mboxProcToken == 1)
        if (gMrrDSSMCB.mboxProcToken == 1)
        {
            gMrrDSSMCB.mboxProcToken = 0;
            /* If the mailbox has a message and the frame processing task has finished. */
            MmwDemo_mboxReadProc();
        }
    }
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
void MmwDemo_sleep(void)
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
void _MmwDemo_dssAssert(int32_t expression,const char *file, int32_t line)
{
    MmwDemo_message  message;
    uint32_t         nameSize;

    if (!expression) 
    {
        message.type = MMWDEMO_DSS2MSS_ASSERT_INFO;
        nameSize = strlen(file);
        if(nameSize > MMWDEMO_MAX_FILE_NAME_SIZE)
            nameSize = MMWDEMO_MAX_FILE_NAME_SIZE;
            
        memcpy((void *) &message.body.assertInfo.file[0], (void *)file, nameSize);
        message.body.assertInfo.line = (uint32_t)line;
        if (MmwDemo_mboxWrite(&message) != 0)
        {
            System_printf ("Error: Failed to send exception information to MSS.\n");
        }
        
    }    
}        
/**
 *  @b Description
 *  @n
 *      Entry point into the DSS MRR TI Design
 *
 *  @retval
 *      Not Applicable.
 */
int32_t main(void)
{
    Task_Params taskParams;
    MmwDemo_DSS_DataPathObj* obj;
    int32_t errCode, ik;

    /* Initialize the global variables */
    memset((void*) &gMrrDSSMCB, 0, sizeof(Mrr_DSS_MCB));
    
    /* Initialize entire data path object to a known state. */
    for (ik = 0, obj = &gMrrDSSMCB.dataPathObj[0]; ik < NUM_SUBFRAMES; ik ++, obj++ )
    {
        memset((void *) obj, 0, sizeof(MmwDemo_DSS_DataPathObj));
    }

    /* Populate the chirp configuration in the DSS for all the data path objects. */
    MmwDemo_dataPathConfigPopulate(&gMrrDSSMCB.dataPathObj[0]);

    /* Initialize the state counters. */
    for (ik = 0, obj = &gMrrDSSMCB.dataPathObj[0]; ik < NUM_SUBFRAMES; ik ++, obj ++)
    {
        MmwDemo_dataPathInit1Dstate(obj);
    }

    /* Initialize the EDMA. */
    MmwDemo_dataPathInitEdma(&gMrrDSSMCB.dataPathObj[0]);

#if NUM_SUBFRAMES > 1
    /* Copy the EDMA handles to the other data path objects. */
    for (ik = 1, obj = &gMrrDSSMCB.dataPathObj[1]; ik < NUM_SUBFRAMES; ik ++, obj++ )
    {
        MmwDemo_dataPathCopyEdmaHandle(obj, &gMrrDSSMCB.dataPathObj[0]);
    }
#endif

    obj = &gMrrDSSMCB.dataPathObj[0];
    /* Copy code from L3 to L1PSRAM, this code related to data path processing */
    MmwDemo_copyTable(obj->edmaHandle[EDMA_INSTANCE_DSS], &_MmwDemo_fastCode_L1PSRAM_copy_table);

    /* Initialize the SOC configuration: */
    {
        SOC_Cfg socCfg;
        /* Initialize the SOC configuration: */
        memset((void *) &socCfg, 0, sizeof(SOC_Cfg));

        /* Populate the SOC configuration: */
        socCfg.clockCfg = SOC_SysClock_BYPASS_INIT;

        /* Initialize the SOC Module: This is done as soon as the application is started
         * to ensure that the MPU is correctly configured. */
        gMrrDSSMCB.socHandle = SOC_init(&socCfg, &errCode);
        if (gMrrDSSMCB.socHandle == NULL)
        {

            return -1;
        }
    }

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 3 * 1024;
    Task_create(MRR_DSS_initTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();

    return 0;
}

void rxGainPhaseParam_Init(MmwDemo_DSS_DataPathObj* obj)
{
    int32_t indx = 0;
    if (obj->processingPath == MAX_VEL_ENH_PROCESSING)
    {
           
        obj->rxChPhaseComp[indx].imag = 5863;
        obj->rxChPhaseComp[indx].real = 19158;indx++;

        obj->rxChPhaseComp[indx].imag = 2858;
        obj->rxChPhaseComp[indx].real = 23451;indx++;

        obj->rxChPhaseComp[indx].imag = 4569;
        obj->rxChPhaseComp[indx].real = 22093;indx++;

        obj->rxChPhaseComp[indx].imag = 0;
        obj->rxChPhaseComp[indx].real = 16399;indx++;
    }
    else if (obj->processingPath == POINT_CLOUD_PROCESSING)
    {
        
    
        obj->rxChPhaseComp[indx].imag = 21646;
        obj->rxChPhaseComp[indx].real = 7035;indx++;

        obj->rxChPhaseComp[indx].imag =  21172;
        obj->rxChPhaseComp[indx].real = 18420;indx++;

        obj->rxChPhaseComp[indx].imag = 15490;
        obj->rxChPhaseComp[indx].real = 21080;indx++;

        obj->rxChPhaseComp[indx].imag = -3905;
        obj->rxChPhaseComp[indx].real = 25130;indx++;


        obj->rxChPhaseComp[indx].imag = 0;
        obj->rxChPhaseComp[indx].real = 16399;indx++;

        obj->rxChPhaseComp[indx].imag = -7985;
        obj->rxChPhaseComp[indx].real = 18443;indx++;

        obj->rxChPhaseComp[indx].imag = -10962;
        obj->rxChPhaseComp[indx].real = 15291;indx++;

   
        obj->rxChPhaseComp[indx].imag =-17653;
        obj->rxChPhaseComp[indx].real = 3133;indx++;

        obj->rxChPhaseComp[indx].imag =  386;
        obj->rxChPhaseComp[indx].real = -17208;indx++;

        obj->rxChPhaseComp[indx].imag = 8587;
        obj->rxChPhaseComp[indx].real = -18744;indx++;
  
        obj->rxChPhaseComp[indx].imag = 11857;
        obj->rxChPhaseComp[indx].real =  -15772;indx++;

        obj->rxChPhaseComp[indx].imag = 18493;
        obj->rxChPhaseComp[indx].real = -2907;indx++;

    }
  
}


/**
 *  @b Description
 *  @n
 *      DSS Initialization Task which initializes the various
 *      components in the DSS subsystem.
 *
 *  @param[in]  arg0
 *      Argument0 with which the task was created
 *  @param[in]  arg1
 *      Argument1 with which the task was created
 *
 *  @retval
 *      Not Applicable.
 */
static void MRR_DSS_initTask(UArg arg0, UArg arg1)
{
    MmwDemo_DSS_DataPathObj* obj;
    int32_t errCode, ik;
    Mailbox_Config      mboxCfg;
    Task_Params taskParams;
    // awr1843_1dfft_hwa
    Semaphore_Params    semParams;
    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/

    /* Initialize the Mailbox */
    Mailbox_init (MAILBOX_TYPE_DSS);

    /* Initialize the ADC Buffer */
    {
        ADCBuf_Params adcBufParams;

        ADCBuf_init();

        /*****************************************************************************
         * Open ADCBUF driver:
         *****************************************************************************/
        ADCBuf_Params_init(&adcBufParams);
        adcBufParams.chirpThresholdPing = 1;
        adcBufParams.chirpThresholdPong = 1;
        adcBufParams.continousMode = 0;

        /* Open ADCBUF driver */
        gMrrDSSMCB.adcBufHandle = ADCBuf_open(0, &adcBufParams);
        if (gMrrDSSMCB.adcBufHandle == NULL)
        {
            //System_printf("Error: Unable to open the ADCBUF driver\n");
            return;
        }

        /* One time ADC Buffer configuration.*/
        MRR_DSS_dssDataPathConfigAdcBuf ();
    }

    {
        /* Register interrupts. */
        SOC_SysIntListenerCfg listenerCfg;

        /* Register Chirp Available Listener */
        memset((void*) &listenerCfg, 0, sizeof(SOC_SysIntListenerCfg));
        listenerCfg.systemInterrupt = SOC_XWR18XX_DSS_INTC_EVENT_CHIRP_AVAIL;
        listenerCfg.listenerFxn = MRR_DSS_chirpIntCallback;
        listenerCfg.arg = 0;
        gMrrDSSMCB.chirpIntHandle = SOC_registerSysIntListener(gMrrDSSMCB.socHandle, &listenerCfg, &errCode);
        if (gMrrDSSMCB.chirpIntHandle == NULL)
        {
            return;
        }

        /* Register Frame Start Listener */
        memset((void*) &listenerCfg, 0, sizeof(SOC_SysIntListenerCfg));
        listenerCfg.systemInterrupt = SOC_XWR18XX_DSS_INTC_EVENT_FRAME_START;
        listenerCfg.listenerFxn = MRR_DSS_frameStartIntCallback;
        listenerCfg.arg = 0;
        gMrrDSSMCB.frameStartIntHandle = SOC_registerSysIntListener(gMrrDSSMCB.socHandle, &listenerCfg, &errCode);

        if (gMrrDSSMCB.frameStartIntHandle == NULL)
        {
            return;
        }
    }

    /* Setup the default mailbox configuration */
    Mailbox_Config_init(&mboxCfg);

    /* Setup the configuration: */
    mboxCfg.chType       = MAILBOX_CHTYPE_MULTI;
    mboxCfg.chId         = MAILBOX_CH_ID_0;
    mboxCfg.writeMode    = MAILBOX_MODE_BLOCKING;
    mboxCfg.readMode     = MAILBOX_MODE_CALLBACK;
    mboxCfg.readCallback = &MmwDemo_mboxCallback;

    gMrrDSSMCB.peerMailbox = Mailbox_open(MAILBOX_TYPE_MSS, &mboxCfg, &errCode);
    if (gMrrDSSMCB.peerMailbox == NULL)
    {
        /* Error: Unable to open the mailbox */
        return;
    }
    
    /* Get DataPath Object handle */
    obj = &gMrrDSSMCB.dataPathObj[0];

    /* Data path configuration for both subframes */
    MmwDemo_dataPathConfigBuffers(obj, SOC_XWR18XX_DSS_ADCBUF_BASE_ADDRESS);
   
    for (ik = 0; ik < NUM_SUBFRAMES; ik ++, obj++)
    {
        /* generate the twiddle factors and windowing inputs for subframe 0 and 1. */
        MmwDemo_dataPathConfigFFTs(obj);
        
        /* Configure the dBscan memories for subframe 0 and 1. */
        MmwDemo_dBScanConfigBuffers(obj);
        /*Initialize the compensation params based on subframe index*/
        rxGainPhaseParam_Init(obj);
        
        /* Configure and initialize the tracking (only for MRR subframe). */
        if (obj->processingPath == MAX_VEL_ENH_PROCESSING)
        {
            ekfInit(obj);
        }
        
        /* Initialize the parking assist module . */
        if (obj->processingPath == POINT_CLOUD_PROCESSING)
        {
            parkingAssistInit(obj);
        }
    }
    
    /* Get DataPath Object handle */
    obj = &gMrrDSSMCB.dataPathObj[0];

    /* Initialize HWA */
    MmwDemo_hwaInit(obj);

    /* Start HWA driver */
    MmwDemo_hwaOpen(obj, gMrrDSSMCB.socHandle);

    //The HWA handle needs to be copied for subframe 1.
    gMrrDSSMCB.dataPathObj[1].hwaHandle = obj->hwaHandle;

    /* Configure one-time EDMA and HWA parameters */
    MmwDemo_dataPathConfig_1D_FFT_HWA_Common(obj);


    /* Config HWA+EDMA for 1D processing and keep it ready for immediate processing
       as soon as Front End starts generating chirps. We configure it for the MRR
       subframe first, then it will switch between MRR and USRR. */
    MmwDemo_config1D_HWA(obj, 0);
    MmwDemo_dataPathTrigger1D(obj, 0);
    /* Notice that the two calls above could also be placed at the beginning of the
       task "MRR_DSS_mmWaveTask" before the while() loop*/

    /* in AWR1443 these functions are called in MmwDemo_dataPathConfig () in main.c.
       This function MmwDemo_dataPathConfig () is called in MmwDemo_doSensorStart()
       just after starting the sensor */

    /* Notice that in AWR1443, the drivers (hwa, edma,adcbuf) are open in the function
       MmwDemo_dataPathOpen() which is called as part of the system init task "MmwDemo_initTask"" */

    // awr1843_1dfft_hwa

    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;


    /* EDMA Configuration for both subframes. */
    if (MmwDemo_dataPathConfigEdma(obj) == -1)
    {
        System_printf("Error: MmwDemo_dataPathConfigEdma() failed\n");
        return;
    }

    /* The logging buffer is marked as available. */
    gMrrDSSMCB.loggingBufferAvailable = 1;

    /* Start data path task */
    Task_Params_init(&taskParams);
    taskParams.priority = 2;
    taskParams.stackSize = 4 * 1024;
    Task_create(MRR_DSS_mmWaveTask, &taskParams, NULL);
    
    return;
}

/**
 *  @b Description
 *  @n
 *      populates the data path object array with the 
 *      MRR configuration. 
 *
 *  @param[in]  obj
 *      pointer to the data path object.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathConfigPopulate(MmwDemo_DSS_DataPathObj* obj)
{
#ifdef SUBFRAME_CONF_MRR_USRR
        MmwDemo_populateMRR(obj, 0);
        MmwDemo_populateUSRR(obj+1, 1);
#else 
#ifdef SUBFRAME_CONF_MRR
        MmwDemo_populateMRR(obj, 0);
#endif 
#ifdef SUBFRAME_CONF_USRR
        MmwDemo_populateUSRR(obj, 0);
#endif
#endif
}

/**
 *  @b Description
 *  @n
 *      Function to send a message to peer through Mailbox virtural channel
 *
 *  @param[in]  message
 *      Pointer to the Capture demo message.
 *
 *  @retval
 *      Success    - 0
 *      Fail       < -1
 */
static int32_t MmwDemo_mboxWrite(MmwDemo_message *message)
{
    int32_t retVal = -1;

    retVal = Mailbox_write(gMrrDSSMCB.peerMailbox, (uint8_t*) message,
                           sizeof(MmwDemo_message));
    if (retVal == sizeof(MmwDemo_message))
    {
        retVal = 0;
    }
    return retVal;
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
static void MmwDemo_mboxReadProc()
{
    MmwDemo_message      message;
    int32_t              retVal = 0;

    
    /* Read the message from the peer mailbox: We are not trying to protect the read
     * from the peer mailbox because this is only being invoked from a single thread */
    retVal = Mailbox_read(gMrrDSSMCB.peerMailbox, (uint8_t*)&message, sizeof(MmwDemo_message));
    if (retVal < 0)
    {
        /* Error: Unable to read the message. Setup the error code and return values */
        return;
    }
    else if (retVal == 0)
    {
          /* We are done: There are no messages available from the peer execution domain. */
        return;
    }
    else
    {
        /* Flush out the contents of the mailbox to indicate that we are done with the message. This will
         * allow us to receive another message in the mailbox while we process the received message. */
        Mailbox_readFlush (gMrrDSSMCB.peerMailbox);

        /* Process the received message: */
        switch (message.type)
        {
            case MMWDEMO_MSS2DSS_DETOBJ_SHIPPED:
            {
                uint8_t prevSubframeIndx;
                MmwDemo_DSS_DataPathObj *   dataPathObj;

                if (gMrrDSSMCB.subframeIndx == 0)
                {
                    prevSubframeIndx = NUM_SUBFRAMES - 1;
                } 
                else
                {
                    prevSubframeIndx = gMrrDSSMCB.subframeIndx - 1;
                }

                dataPathObj = &gMrrDSSMCB.dataPathObj[prevSubframeIndx];
                dataPathObj->timingInfo.transmitOutputCycles = Cycleprofiler_getTimeStamp() - dataPathObj->timingInfo.interFrameProcessingEndTime;

                gMrrDSSMCB.loggingBufferAvailable = 1;

                break;
            }
            default:
            {
                /* Message not support */
                // System_printf ("Error: unsupport Mailbox message id=%d\n", message.type);
                MmwDemo_dssAssert(0);
                break;
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      This function is a callback function that invoked when a message is received from the peer.
 *
 *  @param[in]  handle
 *      Handle to the Mailbox on which data was received
 *  @param[in]  peer
 *      Peer from which data was received

 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mboxCallback
(
    Mbox_Handle  handle,
    Mailbox_Type    peer
)
{
    /* Message has been received from the peer endpoint. */
    gMrrDSSMCB.mboxProcToken = 1;
}

/**
 *  @b Description
 *  @n
 *      Function to send detected objects to MSS logger.
 *
 *  @param[in]  ptrOutputBuffer
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
int32_t MRR_DSS_SendProcessOutputToMSS(uint8_t *ptrHsmBuffer,
                                          uint32_t outputBufSize,
                                          MmwDemo_DSS_DataPathObj *obj)
{
    uint8_t *ptrCurrBuffer;
    uint32_t totalHsmSize = 0;
    uint32_t totalPacketLen = sizeof(MmwDemo_output_message_header);
    uint32_t itemPayloadLen;
    int32_t retVal = 0;
    MmwDemo_message message;
    MmwDemo_output_message_dataObjDescr descr;
    uint32_t tlvIdx = 0;

    /* Set pointer to HSM buffer */
    ptrCurrBuffer = ptrHsmBuffer;

    /* Clear message to MSS */
    memset((void *) &message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_DSS2MSS_DETOBJ_READY;

    /* Header: */
    message.body.detObj.header.platform = 0xA1642 ;
    message.body.detObj.header.magicWord[0] = 0x0102;
    message.body.detObj.header.magicWord[1] = 0x0304;
    message.body.detObj.header.magicWord[2] = 0x0506;
    message.body.detObj.header.magicWord[3] = 0x0708;
    message.body.detObj.header.numDetectedObj = obj->numDetObj;
    message.body.detObj.header.version = MMWAVE_SDK_VERSION_BUILD | (MMWAVE_SDK_VERSION_BUGFIX << 8) | (MMWAVE_SDK_VERSION_MINOR << 16)  | (MMWAVE_SDK_VERSION_MAJOR << 24);

    /* Put detected Objects in HSM buffer: sizeof(MmwDemo_objOut_t) * numDetObj  */
    if  (obj->numDetObj > 0)
    {
        /* Add objects descriptor */
        descr.numDetetedObj = obj->numDetObj;
        descr.xyzQFormat = obj->xyzOutputQFormat;

        itemPayloadLen = sizeof(MmwDemo_output_message_dataObjDescr);
        totalHsmSize += itemPayloadLen;
        if (totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }
        memcpy(ptrCurrBuffer, (void *) &descr, itemPayloadLen);

        /* Add array of objects */
        itemPayloadLen = sizeof(MmwDemo_detectedObjForTx) * obj->numDetObj;
        totalHsmSize += itemPayloadLen;
        if (totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }
        memcpy(&ptrCurrBuffer[sizeof(MmwDemo_output_message_dataObjDescr)],
               (void *) obj->detObjFinal, itemPayloadLen);

        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen
                + sizeof(MmwDemo_output_message_dataObjDescr);
        message.body.detObj.tlv[tlvIdx].type =
                MMWDEMO_OUTPUT_MSG_DETECTED_POINTS;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) ptrCurrBuffer;
        tlvIdx++;

        /* Incrementing pointer to HSM buffer */
        ptrCurrBuffer += itemPayloadLen
                + sizeof(MmwDemo_output_message_dataObjDescr);
        totalPacketLen += sizeof(MmwDemo_output_message_tl) + itemPayloadLen
                + sizeof(MmwDemo_output_message_dataObjDescr);
    }

    if ((obj->processingPath == POINT_CLOUD_PROCESSING) && (obj->dbScanReport.numCluster > 0))  
    {
        /* Add objects descriptor */
        /* In the point cloud processing path, the dbScanReport holds the number of clusters. */
        descr.numDetetedObj = obj->dbScanReport.numCluster;
            
        descr.xyzQFormat = obj->xyzOutputQFormat;

        itemPayloadLen = sizeof(MmwDemo_output_message_dataObjDescr);
        totalHsmSize += itemPayloadLen;
        if (totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }
        memcpy(ptrCurrBuffer, (void *) &descr, itemPayloadLen);

        /* Add array of cluster reports. */
        itemPayloadLen = sizeof(clusteringDBscanReportForTx) * obj->dbScanReport.numCluster;
        totalHsmSize += itemPayloadLen;
        if (totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }
        memcpy(&ptrCurrBuffer[sizeof(MmwDemo_output_message_dataObjDescr)],
               (void *) obj->clusterOpFinal, itemPayloadLen);

        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen
                + sizeof(MmwDemo_output_message_dataObjDescr);
        message.body.detObj.tlv[tlvIdx].type =
                MMWDEMO_OUTPUT_MSG_CLUSTERS;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) ptrCurrBuffer;
        tlvIdx++;

        /* Incrementing pointer to HSM buffer */
        ptrCurrBuffer += itemPayloadLen
                + sizeof(MmwDemo_output_message_dataObjDescr);
        totalPacketLen += sizeof(MmwDemo_output_message_tl) + itemPayloadLen
                + sizeof(MmwDemo_output_message_dataObjDescr);
    }
    
    if (obj->processingPath == POINT_CLOUD_PROCESSING) 
    {
        /* Add objects descriptor */
        descr.numDetetedObj = obj->parkingAssistNumBins;
        descr.xyzQFormat = obj->xyzOutputQFormat;

        itemPayloadLen = sizeof(MmwDemo_output_message_dataObjDescr);
        totalHsmSize += itemPayloadLen;
        if (totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }
        memcpy(ptrCurrBuffer, (void *) &descr, itemPayloadLen);

        /* Add array of c;uster reports. */
        itemPayloadLen = sizeof(uint16_t) * obj->parkingAssistNumBins;
        totalHsmSize += itemPayloadLen;
        if (totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }
        memcpy(&ptrCurrBuffer[sizeof(MmwDemo_output_message_dataObjDescr)],
               (void *) obj->parkingAssistBins, itemPayloadLen);

        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen
                + sizeof(MmwDemo_output_message_dataObjDescr);
        message.body.detObj.tlv[tlvIdx].type =
                MMWDEMO_OUTPUT_MSG_PARKING_ASSIST;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) ptrCurrBuffer;
        tlvIdx++;

        /* Incrementing pointer to HSM buffer */
        ptrCurrBuffer += itemPayloadLen
                + sizeof(MmwDemo_output_message_dataObjDescr);
        totalPacketLen += sizeof(MmwDemo_output_message_tl) + itemPayloadLen
                + sizeof(MmwDemo_output_message_dataObjDescr);
    }

    if ((obj->processingPath == MAX_VEL_ENH_PROCESSING) && 
        (obj->numActiveTrackers > 0))
    {
        /* Add objects descriptor */
        descr.numDetetedObj = obj->numActiveTrackers;
        descr.xyzQFormat = obj->xyzOutputQFormat;

        itemPayloadLen = sizeof(MmwDemo_output_message_dataObjDescr);
        totalHsmSize += itemPayloadLen;
        if (totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }
        memcpy(ptrCurrBuffer, (void *) &descr, itemPayloadLen);

        /* Add array of tracked objects. */
        itemPayloadLen = sizeof(trackingReportForTx) * obj->numActiveTrackers;
        totalHsmSize += itemPayloadLen;
        if (totalHsmSize > outputBufSize)
        {
            retVal = -1;
            goto Exit;
        }
        memcpy(&ptrCurrBuffer[sizeof(MmwDemo_output_message_dataObjDescr)],
               (void *) obj->trackerOpFinal, itemPayloadLen);

        message.body.detObj.tlv[tlvIdx].length = itemPayloadLen
                + sizeof(MmwDemo_output_message_dataObjDescr);
        message.body.detObj.tlv[tlvIdx].type =
                MMWDEMO_OUTPUT_MSG_TRACKED_OBJECTS;
        message.body.detObj.tlv[tlvIdx].address = (uint32_t) ptrCurrBuffer;
        tlvIdx++;

        /* Incrementing pointer to HSM buffer */
        ptrCurrBuffer += itemPayloadLen + sizeof(MmwDemo_output_message_dataObjDescr);
        totalPacketLen += sizeof(MmwDemo_output_message_tl) + itemPayloadLen + sizeof(MmwDemo_output_message_dataObjDescr);
    }

    if (tlvIdx >= MMWDEMO_OUTPUT_MSG_MAX)
    {
        retVal = -1;
    }

    if (retVal == 0)
    {
        
        message.body.detObj.header.numTLVs = tlvIdx;
        /* Round up packet length to multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN. */
        message.body.detObj.header.totalPacketLen =
                MMWDEMO_OUTPUT_MSG_SEGMENT_LEN
                        * ((totalPacketLen
                                + (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - 1))
                                / MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
        message.body.detObj.header.timeCpuCycles = Cycleprofiler_getTimeStamp();
        message.body.detObj.header.frameNumber   = gMrrDSSMCB.stats.frameStartIntCounter;

        /* The GUI reads the subframe number to decide on the plotting type.
         *  a 0 => MAX_VEL_ENH_PROCESSING. 
         *  a 1 => POINT_CLOUD_PROCESSING. */
        message.body.detObj.header.subFrameNumber  = gMrrDSSMCB.dataPathObj[gMrrDSSMCB.subframeIndx].processingPath;
        if (MmwDemo_mboxWrite(&message) != 0)
        {
            retVal = -1;
        }
    }
    Exit: return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function to send data path detection output.
 *
 *  @retval
 *      Not Applicable.
 */
void MRR_DSS_DataPathOutputLogging(MmwDemo_DSS_DataPathObj * dataPathObj)
{
    volatile int32_t waitCounter = 0;
    /* if the logging buffer is not available, wait a little for the transfer message to come from 
     * the MSS. */
    if(gMrrDSSMCB.loggingBufferAvailable == 0) 
    {
        while((waitCounter < 6000000) && (gMrrDSSMCB.loggingBufferAvailable == 0))
        {
            waitCounter++;
            
            if (gMrrDSSMCB.mboxProcToken == 1) 
            {
                gMrrDSSMCB.mboxProcToken = 0;
                /* If the mailbox has a message and the frame processing task has finished. */
                MmwDemo_mboxReadProc();
            }
        }
    }
    
    /* Sending detected objects to logging buffer and shipped out from MSS UART */
    if (gMrrDSSMCB.loggingBufferAvailable == 1)
    {
        /* Set the logging buffer available flag to be 0 */
        gMrrDSSMCB.loggingBufferAvailable = 0;

        /* Save output in logging buffer - HSRAM memory and a message is sent to MSS to notify
         logging buffer is ready */
        if (MRR_DSS_SendProcessOutputToMSS((uint8_t *) &gHSRAM, (uint32_t) SOC_XWR18XX_DSS_HSRAM_SIZE,
                                            dataPathObj) < 0)
        {
            /* Increment logging error */
            MmwDemo_dssAssert(0);
            gMrrDSSMCB.stats.detObjLoggingErr++;
        }
        
    }
    else
    {
        /* Logging buffer is not available, skip saving detected objectes to logging buffer */
        gMrrDSSMCB.stats.detObjLoggingSkip++;
    }
}

/**
 *  @b Description
 *  @n
 *      Function to configure ADCBUF driver based on CLI inputs.
 *  @param[out] numRxChannels Number of receive channels.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t MRR_DSS_dssDataPathConfigAdcBuf()
{
    ADCBuf_dataFormat dataFormat;
    ADCBuf_RxChanConf rxChanConf;
    int32_t retVal;
    uint8_t channel;
    uint8_t adcFmt;
    uint8_t chInterleave;
    uint8_t sampleInterleave;
    uint32_t chirpThreshold;
    uint32_t rxChanMask = 0xF; /* All channels are enabled. */
    uint32_t offset; 

    adcFmt = 0; /* Complex  mode */
    chInterleave = 1; /* Channel interleave mode is 1. i.e. no interleaving */
    sampleInterleave = 1; /* Sample Interleave must be I(MSB) Q(LSB) for HWA usage. */
    chirpThreshold = 1;

    /* Divide 16KB of the ADC buffer into 4 parts. Note that in the ADCBuf mode
     * we are using, there is a single input buffer to the HWA for all chirps. */
    offset = ((16 * 1024)/4);

    /*****************************************************************************
     * Data path :: ADCBUF driver Configuration
     *****************************************************************************/

    /* Populate data format from configuration */
    dataFormat.adcOutFormat = adcFmt;
    dataFormat.channelInterleave = chInterleave;
    dataFormat.sampleInterleave = sampleInterleave;

    /* Disable all ADCBuf channels */
    if ((retVal = ADCBuf_control(gMrrDSSMCB.adcBufHandle,
                                 ADCBufMMWave_CMD_CHANNEL_DISABLE,
                                 (void *) &rxChanMask)) < 0)
    {
        // ("Error: Disable ADCBuf channels failed with [Error=%d]\n", retVal);
        return retVal;
    }

    retVal = ADCBuf_control(gMrrDSSMCB.adcBufHandle,
                            ADCBufMMWave_CMD_CONF_DATA_FORMAT,
                            (void *) &dataFormat);
    if (retVal < 0)
    {
        // ("Error: MMWDemoDSS Unable to configure the data formats\n");
        return -1;
    }

    memset((void*) &rxChanConf, 0, sizeof(ADCBuf_RxChanConf));

    /* Enable all Rx Channels */
    for (channel = 0; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        /* Populate the receive channel configuration: */
        rxChanConf.channel = channel;
        retVal = ADCBuf_control(gMrrDSSMCB.adcBufHandle,
                                ADCBufMMWave_CMD_CHANNEL_ENABLE,
                                (void *) &rxChanConf);

        if (retVal < 0)
        {
            // ("Error: MMWDemoDSS ADCBuf Control for Channel %d Failed with error[%d]\n", channel, retVal);
            return -1;
        }

        rxChanConf.offset += offset;
    }

    /* Set ping/pong chirp threshold: */
    retVal = ADCBuf_control(gMrrDSSMCB.adcBufHandle,
                            ADCBufMMWave_CMD_SET_PING_CHIRP_THRESHHOLD,
                            (void *) &chirpThreshold);
    if (retVal < 0)
    {
        // System_printf("Error: ADCbuf Ping Chirp Threshold Failed with Error[%d]\n", retVal);
        return -1;
    }
    retVal = ADCBuf_control(gMrrDSSMCB.adcBufHandle,
                            ADCBufMMWave_CMD_SET_PONG_CHIRP_THRESHHOLD,
                            (void *) &chirpThreshold);
    if (retVal < 0)
    {
        // System_printf("Error: ADCbuf Pong Chirp Threshold Failed with Error[%d]\n", retVal);
        return -1;
    }

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
static void MmwDemo_copyTable(EDMA_Handle handle, COPY_TABLE *tp)
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
        MmwDemo_dssAssert(crp.size <= 65536U);

        if (crp.size)
        {
            MmwDemo_edmaBlockCopy(handle, loadAddr, runAddr, crp.size);
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
static void MmwDemo_edmaBlockCopy(EDMA_Handle handle, uint32_t loadAddr, uint32_t runAddr, uint16_t size)
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
        MmwDemo_dssAssert(0);
    }

    if (EDMA_startDmaTransfer(handle, config.channelId) != EDMA_NO_ERROR)
    {
        MmwDemo_dssAssert(0);
    }

    /* wait until transfer done */
    do 
    {
        if (EDMA_isTransferComplete(handle,
                config.paramSetConfig.transferCompletionCode,
                (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            MmwDemo_dssAssert(0);
        }
    } 
    while (isTransferDone == false);

    /* make sure to disable channel so it is usable later */
    EDMA_disableChannel(handle, config.channelId, config.channelType);
}

/**
 *  @b Description
 *  @n
 *      Converts an SNR (in dB) to an SNR Threshold that the CFAR algo
 *      can use. 
 *  @param[in]  number of integrations in the detection matrix 
 *              Typically the number of virtual antennas
 *  @param[in]  Threshold in dB (float)
 *
 *  @retval
 *      Threshold for CFAR algorithm.
 */
uint16_t convertSNRdBtoThreshold(uint16_t numInteg, float ThresholdIndB,uint16_t bitwidth)
{
    float scaleFac = (float) ((1 << bitwidth) * numInteg);
    float convertFrom_10Log10_to_20Log2 = ThresholdIndB * (1.0f / 6.0f);
    
    return (uint16_t) (scaleFac * convertFrom_10Log10_to_20Log2);
}

/**
 *  @b Description
 *  @n
 *      Configures the dBScan buffers.  
 *  @param[in]  data path Object
 *
 *  @retval
 *      Not applicable.
 */
void MmwDemo_dBScanConfigBuffers(MmwDemo_DSS_DataPathObj * obj)
{

    obj->dbScanInstance.scratchPad    =   (char *)obj->dBscanScratchPad;
    obj->dbScanInstance.visited       =   (char *)& obj->dBscanScratchPad[0];
    obj->dbScanInstance.scope         =   (char *)& obj->dBscanScratchPad[obj->dbScanInstance.maxPoints];
    obj->dbScanInstance.neighbors     =   (uint16_t *)& obj->dBscanScratchPad[2 * obj->dbScanInstance.maxPoints];
    
    obj->dbScanReport.IndexArray        =    obj->dbscanOutputDataIndexArray;
    obj->dbScanReport.report    =    obj->dbscanOutputDataReport;
    obj->dbScanReport.numCluster    =    0;
    
    if (obj->processingPath == POINT_CLOUD_PROCESSING)
    {
        uint32_t i;
        for (i = 0; i < obj->dbScanInstance.maxClusters; i++)
        {
            obj->dbScanState[i].numPoints = 0;    
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Populates the configuration for the MRR max-vel-enhancement subframe.   
 *  @param[in]  data path Object
 *
 *  @retval
 *      Not applicable.
 */
void MmwDemo_populateMRR(MmwDemo_DSS_DataPathObj* obj, uint16_t subframeIndx)
{
    /* Subframe 0 */
    obj->processingPath = MAX_VEL_ENH_PROCESSING;
    obj->subframeIndx = subframeIndx;
    obj->numRxAntennas = NUM_RX_CHANNELS;
    // awr1843_1dfft_hwa
    obj->numChirpsPerFrame = SUBFRAME_MRR_NUM_CHIRPS_TOTAL;
    obj->chirpThreshold = ADCBUFF_CHIRP_THRESHOLD;
    obj->numTxAntennas = SUBFRAME_MRR_NUM_TX;
    obj->numVirtualAntennas = SUBFRAME_MRR_NUM_VIRT_ANT;
    obj->numVirtualAntAzim = SUBFRAME_MRR_NUM_VIRT_ANT;
    obj->numVirtualAntElev = 0; /* No elevation on the 1642 EVM */
    obj->numAdcSamples = SUBFRAME_MRR_NUM_CMPLX_ADC_SAMPLES;
    obj->numRangeBins = SUBFRAME_MRR_NUM_CMPLX_ADC_SAMPLES;
    obj->numChirpsPerChirpType = SUBFRAME_MRR_CHIRPTYPE_0_NUM_CHIRPS;
    obj->numAngleBins = SUBFRAME_MRR_NUM_ANGLE_BINS;
    obj->invNumAngleBins = 1.0f/((float)SUBFRAME_MRR_NUM_ANGLE_BINS);
    obj->numDopplerBins = SUBFRAME_MRR_CHIRPTYPE_0_NUM_CHIRPS;
    obj->maxNumObj2DRaw = MAX_DET_OBJECTS_RAW_MAX_VEL_ENH_PROCESSING;
#if (SUBFRAME_MRR_CHIRPTYPE_0_NUM_CHIRPS != SUBFRAME_MRR_CHIRPTYPE_1_NUM_CHIRPS)
    #error "the number of chirps in each chirp type is not equal. Processing will fail in surprising ways. "
#endif
    obj->log2NumDopplerBins = LOG2_APPROX(SUBFRAME_MRR_CHIRPTYPE_0_NUM_CHIRPS);
    obj->rangeResolution = PROFILE_MRR_RANGE_RESOLUTION_METERS;
    /* In the max-vel enhancement chirp, we use the velocity resolution of the first (fast) chirp as 
     * the default velocity resolution. */
    obj->velResolution = SUBFRAME_MRR_CHIRPTYPE_0_VEL_RESOLUTION_M_P_S;
    
    obj->maxUnambiguousVel = SUBFRAME_MRR_CHIRPTYPE_0_MAX_VEL_M_P_S;
    
    /*! @brief Q format of the output x/y/z coordinates */
    obj->xyzOutputQFormat = 7; /* 7 fractional bits are enough for most cases. */
    obj->invOneQFormat = 1.0f/((float)(1U << obj->xyzOutputQFormat));
    obj->sinAzimQFormat = 14;
    obj->invOneSinAzimFormat = 1.0f/((float)(1U << obj->sinAzimQFormat));
    
    /*! @brief CFAR configuration in Doppler direction 
     *   we do a log CFAR-CA-SO (with wrap)*/
    obj->cfarCfgDoppler.averageMode = MMW_NOISE_AVG_MODE_CFAR_CA; 
    obj->cfarCfgDoppler.winLen = 8;
    obj->cfarCfgDoppler.guardLen = 4;
    obj->cfarCfgDoppler.noiseDivShift = 4;  /* Should be log2(2*winLen) */
    obj->cfarCfgDoppler.cyclicMode = 0;
    obj->cfarCfgDoppler.thresholdScale = convertSNRdBtoThreshold(obj->numRxAntennas*obj->numTxAntennas, SUBFRAME_MRR_MIN_SNR_dB, CFARTHRESHOLD_N_BIT_FRAC);

    /*! @brief CFAR configuration in Range direction 
     *   we do a log CFAR-CA-SO (without wrap)*/
    obj->cfarCfgRange.averageMode = MMW_NOISE_AVG_MODE_CFAR_CASO;
    obj->cfarCfgRange.winLen = 8;
    obj->cfarCfgRange.guardLen = 4;
    obj->cfarCfgRange.noiseDivShift = 4; /* Should be log2(2*winLen) */
    obj->cfarCfgRange.cyclicMode = 0;
    obj->cfarCfgRange.thresholdScale =  convertSNRdBtoThreshold(obj->numRxAntennas*obj->numTxAntennas, SUBFRAME_MRR_MIN_SNR_dB, CFARTHRESHOLD_N_BIT_FRAC);;
    obj->cfarCfgRange_minIndxToIgnoreHPF = 0; 
   
    
    /*! @brief Multi object beam forming configuration */
    obj->multiObjBeamFormingCfg.enabled = 0;
    obj->multiObjBeamFormingCfg.multiPeakThrsScal = 1.0f;

    /*! @brief DC Range antenna signature callibration configuration */
    obj->calibDcRangeSigCfg.enabled = 0;
    obj->txAntennaCount = SUBFRAME_MRR_NUM_TX;
   
#ifdef LOW_THRESHOLD_FOR_USRR
   /*! @brief Min and max range configuration. */
    obj->minRange = (uint16_t) (0.3f * (1 << obj->xyzOutputQFormat));
    obj->maxRange = (uint16_t) ROUND_TO_INT32(obj->rangeResolution*SUBFRAME_MRR_NUM_CMPLX_ADC_SAMPLES * (0.9f) * (1 << obj->xyzOutputQFormat));

    /*! @brief CFAR thresholds are varied as a function of range */
    obj->SNRThresholds[0].rangelim = (uint16_t) (10.0f * (float)(1 << obj->xyzOutputQFormat));
    obj->SNRThresholds[0].threshold = convertSNRdBtoThreshold(1, 16.0f,CFARTHRESHOLD_N_BIT_FRAC);

    obj->SNRThresholds[1].rangelim = (uint16_t) (35.0f * (float)(1 << obj->xyzOutputQFormat));
    obj->SNRThresholds[1].threshold = convertSNRdBtoThreshold(1, 15.0f, CFARTHRESHOLD_N_BIT_FRAC);

    obj->SNRThresholds[2].rangelim = 65535;
    obj->SNRThresholds[2].threshold = convertSNRdBtoThreshold(1, SUBFRAME_MRR_MIN_SNR_dB, CFARTHRESHOLD_N_BIT_FRAC);

    /*! @brief peakVal thresholds are varied as a function of range (disabled for the MRR configuration)*/
    obj->peakValThresholds[0].rangelim = 65535;
    obj->peakValThresholds[0].threshold = 0;

    obj->peakValThresholds[1].rangelim = 65535;
    obj->peakValThresholds[1].threshold = 0;

    obj->peakValThresholds[2].rangelim = 65535;
    obj->peakValThresholds[2].threshold = 0;
#else    
    /*! @brief Min and max range configuration. */
    obj->minRange = (uint16_t) (0.5f * (1 << obj->xyzOutputQFormat));
    obj->maxRange = (uint16_t) ROUND_TO_INT32(obj->rangeResolution*SUBFRAME_MRR_NUM_CMPLX_ADC_SAMPLES * (0.9f) * (1 << obj->xyzOutputQFormat));

    /*! @brief CFAR thresholds are varied as a function of range */
    obj->SNRThresholds[0].rangelim = (uint16_t) (10.0f * (float)(1 << obj->xyzOutputQFormat));
    obj->SNRThresholds[0].threshold = convertSNRdBtoThreshold(1, 18.0f,CFARTHRESHOLD_N_BIT_FRAC);

    obj->SNRThresholds[1].rangelim = (uint16_t) (35.0f * (float)(1 << obj->xyzOutputQFormat));
    obj->SNRThresholds[1].threshold = convertSNRdBtoThreshold(1, 16.0f, CFARTHRESHOLD_N_BIT_FRAC);

    obj->SNRThresholds[2].rangelim = 65535;
    obj->SNRThresholds[2].threshold = convertSNRdBtoThreshold(1, SUBFRAME_MRR_MIN_SNR_dB, CFARTHRESHOLD_N_BIT_FRAC);

    /*! @brief peakVal thresholds are varied as a function of range */
    obj->peakValThresholds[0].rangelim = (uint16_t) (7.0f * (float)(1 << obj->xyzOutputQFormat));;
    obj->peakValThresholds[0].threshold = 4250;

    obj->peakValThresholds[1].rangelim = 65535;
    obj->peakValThresholds[1].threshold = 0;

    obj->peakValThresholds[2].rangelim = 65535;
    obj->peakValThresholds[2].threshold = 0;
#endif
    obj->log2numVirtAnt = LOG2_APPROX(SUBFRAME_MRR_NUM_VIRT_ANT);
    
    obj->maxVelEnhStruct.velResolutionFastChirp     =   SUBFRAME_MRR_CHIRPTYPE_0_VEL_RESOLUTION_M_P_S; /* Meters/sec */
    obj->maxVelEnhStruct.invVelResolutionSlowChirp      =   (1.0f/((float)SUBFRAME_MRR_CHIRPTYPE_1_VEL_RESOLUTION_M_P_S)); /* Meters/sec */
    obj->maxVelEnhStruct.maxVelAssocThresh        =   MAX_VEL_IMPROVEMENT_ASSOCIATION_THRESH;
    
//    MmwDemo_dssAssert((obj->numAngleBins % obj->numVirtualAntAzim) == 0);
    
    obj->dbScanInstance.epsilon       =   2.5f;  /* Neighbour distance threshold (in meters). */
    obj->dbScanInstance.vFactor       =   3.0f;   /* Neighbour speed  threshold (in meters/sec). */
    obj->dbScanInstance.weight        =   1.3f * 1.3f * obj->dbScanInstance.epsilon;
    obj->dbScanInstance.maxClusters   =   MAX_NUM_CLUSTER_MRR;
    obj->dbScanInstance.minPointsInCluster    =   1;
    obj->dbScanInstance.maxPoints     =   MRR_MAX_OBJ_OUT;
    obj->dbScanInstance.fixedPointScale      =   (1 << obj->xyzOutputQFormat);    
    obj->dbScanInstance.dBScanNeighbourLim  = 4;
 
    obj->trackerInstance.rangeAssocThresh  =   2.0f;            /* in meters. */
    obj->trackerInstance.velAssocThresh     =   2.0f;           /* in meters/sec. */
    obj->trackerInstance.azimAssocThresh   =   (5.0f/180.0f);   /* in sin(omega). */
    obj->trackerInstance.maxTrackers    =   MAX_TRK_OBJs;
    obj->trackerInstance.fixedPointScale      =   (1 << obj->xyzOutputQFormat);
    obj->trackerInstance.distAssocThreshSq  =   1.65f;            /* in meters sq. */
    
    /* No parking assist with the MRR chirp (because they are designed to have poor range and 
     * angular resolution). */
    obj->parkingAssistNumBins = 0;
}

/**
 *  @b Description
 *  @n
 *      Populates the configuration for the USRR point cloud subframe.   
 *  @param[in]  data path Object
 *
 *  @retval
 *      Not applicable.
 */
void MmwDemo_populateUSRR(MmwDemo_DSS_DataPathObj* obj, uint16_t subframeIndx)
{
    obj->processingPath = POINT_CLOUD_PROCESSING;
    obj->subframeIndx = subframeIndx;
    obj->numRxAntennas = NUM_RX_CHANNELS;
    obj->numChirpsPerFrame = SUBFRAME_USRR_NUM_CHIRPS_TOTAL;
    obj->chirpThreshold = ADCBUFF_CHIRP_THRESHOLD;
    obj->numTxAntennas = SUBFRAME_USRR_NUM_TX;
    obj->numVirtualAntennas = SUBFRAME_USRR_NUM_VIRT_ANT;
    obj->numVirtualAntAzim = 8;
    obj->numVirtualAntElev = 4; 
    obj->numAdcSamples = SUBFRAME_USRR_NUM_CMPLX_ADC_SAMPLES;
	//obj->numChirpsPerFrame =  (SUBFRAME_USRR_CHIRP_END_IDX - SUBFRAME_USRR_CHIRP_START_IDX + 1) * SUBFRAME_USRR_LOOP_COUNT;
    obj->numRangeBins = SUBFRAME_USRR_NUM_CMPLX_ADC_SAMPLES;
    obj->numChirpsPerChirpType = SUBFRAME_USRR_CHIRPTYPE_0_NUM_CHIRPS;
    obj->numAngleBins = SUBFRAME_USRR_NUM_ANGLE_BINS;
    obj->invNumAngleBins = 1.0f/((float)SUBFRAME_USRR_NUM_ANGLE_BINS);
    obj->numDopplerBins = SUBFRAME_USRR_CHIRPTYPE_0_NUM_CHIRPS;
    obj->maxNumObj2DRaw = MAX_DET_OBJECTS_RAW_POINT_CLOUD_PROCESSING;
    
#if (SUBFRAME_USRR_CHIRPTYPE_0_NUM_CHIRPS != SUBFRAME_USRR_CHIRPTYPE_1_NUM_CHIRPS)
#error "the number of chirps in each chirp type is not equal"
#endif
    obj->log2NumDopplerBins = LOG2_APPROX(SUBFRAME_USRR_CHIRPTYPE_0_NUM_CHIRPS);
    obj->rangeResolution = PROFILE_USRR_RANGE_RESOLUTION_METERS;
    
    /* For the point-cloud chirp, both chirptypes offer the same velocity resolution. */
    obj->velResolution = SUBFRAME_USRR_VEL_RESOLUTION_M_P_S;

    obj->maxUnambiguousVel = SUBFRAME_USRR_MAX_VEL_M_P_S;
    
    /*! @brief Q format of the output x/y/z coordinates */
    obj->xyzOutputQFormat = 7;
    obj->invOneQFormat = 1.0f/((float)(1U << obj->xyzOutputQFormat));
    obj->sinAzimQFormat = 14;
    obj->invOneSinAzimFormat = 1.0f/((float)(1U << obj->sinAzimQFormat));
    
    /*! @brief CFAR configuration in Doppler direction */
    obj->cfarCfgDoppler.averageMode = MMW_NOISE_AVG_MODE_CFAR_CA;
    obj->cfarCfgDoppler.winLen = 8;
    obj->cfarCfgDoppler.guardLen = 4;
    obj->cfarCfgDoppler.noiseDivShift = 4; /* Should be log2(2*winLen) */ 
    obj->cfarCfgDoppler.cyclicMode = 0;
    obj->cfarCfgDoppler.thresholdScale =  convertSNRdBtoThreshold(obj->numRxAntennas*obj->numTxAntennas, 15.0f, CFARTHRESHOLD_N_BIT_FRAC);

    /*! @brief CFAR configuration in Range direction */
    obj->cfarCfgRange.averageMode = MMW_NOISE_AVG_MODE_CFAR_CASO;
    obj->cfarCfgRange.winLen = 8;
    obj->cfarCfgRange.guardLen = 4;
    obj->cfarCfgRange.noiseDivShift = 4; /* Should be log2(2*winLen) */
    obj->cfarCfgRange.cyclicMode = 0;
    obj->cfarCfgRange.thresholdScale =  convertSNRdBtoThreshold(obj->numRxAntennas*obj->numTxAntennas, 12.0f, CFARTHRESHOLD_N_BIT_FRAC);
    obj->cfarCfgRange_minIndxToIgnoreHPF = 54; /* The left side of the CFAR sum is ignored upto  ~800 kHz (54+12)*6222/512* */
    /*! @brief min and max range configuration */
    obj->minRange = (uint16_t) (0.10f * (1U << obj->xyzOutputQFormat));
    obj->maxRange = (uint16_t) ROUND_TO_INT32(obj->rangeResolution*SUBFRAME_USRR_NUM_CMPLX_ADC_SAMPLES * (0.9f) * (1U << obj->xyzOutputQFormat));

    /*! @brief Multi object beam forming configuration */
    obj->multiObjBeamFormingCfg.enabled = 1;
    obj->multiObjBeamFormingCfg.multiPeakThrsScal = 0.85f;

    /*! @brief DC Range antenna signature calibration configuration. */
    obj->calibDcRangeSigCfg.enabled = 0;
    obj->txAntennaCount = SUBFRAME_USRR_NUM_TX;

    obj->log2numVirtAnt = LOG2_APPROX(SUBFRAME_USRR_NUM_VIRT_ANT);
#ifdef LOW_THRESHOLD_FOR_USRR
    /*! @brief CFAR thresholds are varied as a function of range */
    obj->SNRThresholds[0].rangelim = (uint16_t) (6.0f * (float)(1U << obj->xyzOutputQFormat));
    obj->SNRThresholds[0].threshold = convertSNRdBtoThreshold(1U, 13.0f, CFARTHRESHOLD_N_BIT_FRAC);

    obj->SNRThresholds[1].rangelim = (uint16_t) (10.0f * (float)(1U << obj->xyzOutputQFormat));
    obj->SNRThresholds[1].threshold = convertSNRdBtoThreshold(1U, 12.0f, CFARTHRESHOLD_N_BIT_FRAC);

    obj->SNRThresholds[2].rangelim = 65535;
    obj->SNRThresholds[2].threshold = convertSNRdBtoThreshold(1U, 12.0f, CFARTHRESHOLD_N_BIT_FRAC);
    
    /*! @brief peakVal thresholds are varied as a function of range (meant to remove cases of 
     * clutter being detected too when we drive the car.) Thresholds were derived from experiments.*/
    obj->peakValThresholds[0].rangelim = (uint16_t) (3.0f * (float) (1U << obj->xyzOutputQFormat));
    obj->peakValThresholds[0].threshold = (27000 >> obj->log2numVirtAnt);
    
    obj->peakValThresholds[1].rangelim = 65535;
    obj->peakValThresholds[1].threshold = 0;

    obj->peakValThresholds[2].rangelim = 65535;
    obj->peakValThresholds[2].threshold = 0;
#else
    /*! @brief CFAR thresholds are varied as a function of range */
    obj->SNRThresholds[0].rangelim = (uint16_t) (6.0f * (float)(1U << obj->xyzOutputQFormat));
    obj->SNRThresholds[0].threshold = convertSNRdBtoThreshold(1, 16.0f, CFARTHRESHOLD_N_BIT_FRAC);

    obj->SNRThresholds[1].rangelim = (uint16_t) (10.0f * (float)(1U << obj->xyzOutputQFormat));
    obj->SNRThresholds[1].threshold = convertSNRdBtoThreshold(1U, 15.0f, CFARTHRESHOLD_N_BIT_FRAC);

    obj->SNRThresholds[2].rangelim = 65535;
    obj->SNRThresholds[2].threshold = convertSNRdBtoThreshold(1U, 14.0f, CFARTHRESHOLD_N_BIT_FRAC);
    
    /*! @brief peakVal thresholds are varied as a function of range (meant to remove cases of 
     * clutter being detected too when we drive the car.) Thresholds were derived from experiments.*/
    obj->peakValThresholds[0].rangelim = (uint16_t) (3.0f * (float) (1U << obj->xyzOutputQFormat));
    obj->peakValThresholds[0].threshold = (350 >> obj->log2numVirtAnt);
    
    obj->peakValThresholds[1].rangelim = 65535;
    obj->peakValThresholds[1].threshold = 0;

    obj->peakValThresholds[2].rangelim = 65535;
    obj->peakValThresholds[2].threshold = 0;
#endif
    
    /*! Configuring the dbSCan for car like objects. These numbers are derived from field tests */
    obj->dbScanInstance.epsilon       =   1.7f;
    obj->dbScanInstance.vFactor       =   3.0f;
    obj->dbScanInstance.weight        =   1.3f * 1.3f * obj->dbScanInstance.epsilon;
    obj->dbScanInstance.maxClusters   =   MAX_NUM_CLUSTER_USRR;
    obj->dbScanInstance.minPointsInCluster    =   3;
    obj->dbScanInstance.maxPoints     =   MRR_MAX_OBJ_OUT;
    obj->dbScanInstance.fixedPointScale      =   (1 << obj->xyzOutputQFormat);    
    obj->dbScanInstance.dBScanNeighbourLim = 7; 
    
    /*! Configuring the 'parking assist' occupancy detect. */
    obj->parkingAssistNumBins = 32;
    obj->parkingAssistMaxRange  = 20 * (1 << obj->xyzOutputQFormat);
    obj->parkingAssistNumBinsLog2 = LOG2_APPROX(32);
    
    /* Azimuth SNR computation depend on the following computation. */
//    MmwDemo_dssAssert((obj->numAngleBins % obj->numVirtualAntAzim) == 0);
}


/**
 *  @b Description
 *  @n
 *      Configure HWA for 1D processing.
 *  @param[in]  obj Pointer to data path obj.
 *  @retval none.
 */
void MmwDemo_config1D_HWA(MmwDemo_DSS_DataPathObj *obj, uint16_t subframe)
{
    int32_t             errCode;
    int32_t             startIdx;
    HWA_CommonConfig    hwaCommonConfig;
    uint8_t             hwaTriggerMode;

    /* Disable the HWA */
    errCode = HWA_enable(obj->hwaHandle,0); // set 1 to enable
    if (errCode != 0)
    {
        //System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        MmwDemo_dssAssert (0);
        return;
    }

    hwaTriggerMode = HWA_TRIG_MODE_DFE;

    if (subframe == 0) //mrr
    {
      startIdx = MMW_HWA_START_PARAMSET_SF0_1D;

      HWAutil_configRangeFFT(obj->hwaHandle,
          startIdx,
          obj->numAdcSamples,
          obj->numRangeBins,
          obj->numRxAntennas,
          obj->hwa_1DwinOff,
          MMW_HWA_DMA_TRIGGER_SOURCE_1D_PING,
          MMW_HWA_DMA_TRIGGER_SOURCE_1D_PONG,
          MMW_HWA_DMA_DEST_CHANNEL_1D_PING,
          MMW_HWA_DMA_DEST_CHANNEL_1D_PONG,
          ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_1D_ADCBUF_INP),
          ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_1D_OUT_PING),
          ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_1D_OUT_PONG),
          hwaTriggerMode);
    }
    else //USRR
    {
      startIdx = MMW_HWA_START_PARAMSET_SF1_1D;

      HWAutil_configRangeFFT(obj->hwaHandle,
          startIdx,
          obj->numAdcSamples,
          obj->numRangeBins,
          obj->numRxAntennas,
          obj->hwa_1DwinOff,
          MMW_HWA_DMA_TRIGGER_SOURCE_1D_PING+2,
          MMW_HWA_DMA_TRIGGER_SOURCE_1D_PONG+2,
          MMW_HWA_DMA_DEST_CHANNEL_1D_PING+2,
          MMW_HWA_DMA_DEST_CHANNEL_1D_PONG+2,
          ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_1D_ADCBUF_INP),
          ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_1D_OUT_PING),
          ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_1D_OUT_PONG),
          hwaTriggerMode);
    }

    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    /* Config Common Registers */
    hwaCommonConfig.configMask = HWA_COMMONCONFIG_MASK_NUMLOOPS |
                               HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                               HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
                               HWA_COMMONCONFIG_MASK_FFT1DENABLE |
                               HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;

    //Since numLoops for HWA means how many times to process the set of paramsets,
    //and we've defined paramsets for 1 ping and 1 pong, divide the total chirps by 2.
    hwaCommonConfig.numLoops = obj->numChirpsPerFrame / 2;
    hwaCommonConfig.paramStartIdx = startIdx;
    hwaCommonConfig.paramStopIdx  = startIdx + 3;

    /* HWA will input data from ADC buffer memory*/
    hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_ENABLE;

    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
    errCode = HWA_configCommon(obj->hwaHandle,&hwaCommonConfig);
    if (errCode != 0)
    {
        //System_printf("Error: HWA_configCommon returned %d\n",errCode);
        MmwDemo_dssAssert (0);
        return;
    }
}


/**
 *  @b Description
 *  @n
 *      Configure HWA for 2D processing.
 *
 *  @param[in]  obj Pointer to data path obj.
 *  @retval none.
 */
void MmwDemo_config2D_HWA(MmwDemo_DSS_DataPathObj *obj, uint16_t subframe)
{
    int32_t  errCode;
    uint32_t dopplerBinsPerRangeBin;
    HWA_CommonConfig hwaCommonConfig;

    /* Disable the HWA */
    errCode = HWA_enable(obj->hwaHandle,0);
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        return;
    }

    if (subframe == 0) //MRR
      dopplerBinsPerRangeBin = obj->numDopplerBins * 2; //for MRR, there are two chirp types
    else
      dopplerBinsPerRangeBin = obj->numDopplerBins;     //for MRR, there is one chirp type

    /***************************************/
    /* Doppler FFT chain                   */
    /***************************************/
    HWAutil_configDopplerFFT(obj->hwaHandle,
        MMW_HWA_START_PARAMSET_SF0_2D,     // paramSetStartIdx,
        obj->numDopplerBins,               // dopplerFftSize,
        dopplerBinsPerRangeBin,            // doppler bins (chirps) per range bin,
        obj->numVirtualAntennas,           // numVirtualAnt,
        MMW_NUM_RANGE_BINS_PER_TRANSFER,   // numRangeBinsPerIter,
        obj->hwa_2DwinOff,

        MMW_HWA_DMA_TRIGGER_SOURCE_2D_PING,
        MMW_HWA_DMA_TRIGGER_SOURCE_2D_PONG,
        MMW_HWA_DMA_DEST_CHANNEL_2D_PING,
        MMW_HWA_DMA_DEST_CHANNEL_2D_PONG,

        ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_2D_INP_PING), //hwaMemSourcePingOffset
        ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_2D_INP_PONG), //hwaMemSourcePongOffset
        ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_2D_OUT_PING), //hwaMemDestPingOffset
        ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_2D_OUT_PONG), //hwaMemDestPongOffset
        DATA_PATH_CHAIN_COMBINED_LOGMAG);

    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    /* Config Common Registers */
    hwaCommonConfig.configMask =
        HWA_COMMONCONFIG_MASK_NUMLOOPS |
        HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
        HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
        HWA_COMMONCONFIG_MASK_FFT1DENABLE |
        HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;


    hwaCommonConfig.numLoops = obj->numRangeBins/(2*MMW_NUM_RANGE_BINS_PER_TRANSFER);
    hwaCommonConfig.paramStartIdx = MMW_HWA_START_PARAMSET_SF0_2D;
    hwaCommonConfig.paramStopIdx = MMW_HWA_START_PARAMSET_SF0_2D + 7;
    hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
    errCode = HWA_configCommon(obj->hwaHandle,&hwaCommonConfig);
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_configCommon returned %d\n",errCode);
        return;
    }
}

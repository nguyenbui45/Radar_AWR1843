/**
 *   @file  mss_srr.h
 *
 *   @brief
 *      This is the main header file for the MSS for the SRR TI Design 
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
#ifndef MSS_SRR_H
#define MSS_SRR_H

/* MMWAVE Driver Include Files */
#include <ti/common/mmwave_error.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/drivers/cbuff/cbuff.h>

#include <ti/drivers/adcbuf/ADCBuf.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/osal/DebugP.h>

/* BIOS/XDC Include Files */
#include <ti/sysbios/knl/Semaphore.h>

/* MMWAVE library Include Files */
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/mmwave/mmwave.h>
#include "../common/srr_config_consts.h"


#ifdef __cplusplus
extern "C" {
#endif

/*! @brief   sensor start CLI event */
#define MMWDEMO_CLI_SENSORSTART_EVT                     Event_Id_00

/*! @brief   sensor stop CLI  event */
#define MMWDEMO_CLI_SENSORSTOP_EVT                      Event_Id_01

/*! @brief   sensor frame start CLI  event */
#define MMWDEMO_CLI_FRAMESTART_EVT                      Event_Id_02

/*! @brief   BSS CPUFAULT event */
#define MMWDEMO_BSS_CPUFAULT_EVT                        Event_Id_03

/*! @brief   BSS ESMFAULT event */
#define MMWDEMO_BSS_ESMFAULT_EVT                        Event_Id_04

/*! @brief   Monitoring report event */
#define MMWDEMO_BSS_MONITORING_REP_EVT                  Event_Id_05

/*! @brief   BSS Calibration report event */
#define MMWDEMO_BSS_CALIBRATION_REP_EVT                 Event_Id_06

/*! @brief   start completed event from DSS/MSS */
#define MMWDEMO_DSS_START_COMPLETED_EVT                 Event_Id_07

/*! @brief   stop completed event from DSS */
#define MMWDEMO_DSS_STOP_COMPLETED_EVT                  Event_Id_08

/*! @brief   start failed event from DSS/MSS */
#define MMWDEMO_DSS_START_FAILED_EVT                    Event_Id_09




/* All CLI events */
#define MMWDEMO_CLI_EVENTS                              (MMWDEMO_CLI_SENSORSTART_EVT |    \
                                                         MMWDEMO_CLI_SENSORSTOP_EVT |     \
                                                         MMWDEMO_CLI_FRAMESTART_EVT)

/* All BSS faults events */
#define MMWDEMO_BSS_FAULT_EVENTS                        (MMWDEMO_BSS_CPUFAULT_EVT |     \
                                                         MMWDEMO_BSS_ESMFAULT_EVT )

/**
 * @brief
 *  Millimeter Wave Demo statistics
 *
 * @details
 *  The structure is used to hold the statistics information for the
 *  Millimeter Wave demo
 */
typedef struct MmwDemo_MSS_STATS_t
{
    /*! @brief   CLI event for sensorStart */
    uint8_t      cliSensorStartEvt;

    /*! @brief   CLI event for sensorStop */
    uint8_t      cliSensorStopEvt;

    /*! @brief   CLI event for frameStart */
    uint8_t      cliFrameStartEvt;

    /*! @brief   Counter which tracks the number of datapath config event detected
     *           The event is triggered in mmwave config callback function */
    uint8_t      datapathConfigEvt;

    /*! @brief   Counter which tracks the number of datapath start event  detected 
     *           The event is triggered in mmwave start callback function */
    uint8_t      datapathStartEvt;

    /*! @brief   Counter which tracks the number of datapath stop event detected 
     *           The event is triggered in mmwave stop callback function */
    uint8_t      datapathStopEvt;

    /*! @brief   Counter which tracks the number of failed calibration reports
     *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     numFailedTimingReports;

    /*! @brief   Counter which tracks the number of calibration reports received
     *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     numCalibrationReports;
}MmwDemo_MSS_STATS;



/**
 * @brief
 *  MSS SRR TI Design Master control block
 *
 * @details
 *  The structure is used to hold information pertinent to the MSS SRR TI Design.
 */
typedef struct Srr_MSS_MCB_t
{
    /**
     * @brief  An token for frame start events. 
     */
    int32_t frameStartToken;

    /**
     * @brief  The number of subframes transmitted derived from  chirp available interrupts. 
     */
    int32_t subframeCntFromChirpInt;
    
    /**
     * @brief  The number of subframes transmitted derived from the frame start interrupts. 
     */
    int32_t subframeCntFromFrameStart;
        
    /**
     * @brief  The total number of chirp available interrupts. 
     */
    int32_t chirpIntcumSum;

    /**
     * @brief  A counter for chirp interrupts. It is reset every subframe. 
     */
    int32_t chirpInt;

    /**
     * @brief  The number of chirps per subframe. 
     */
    int32_t numChirpsPerSubframe[NUM_SUBFRAMES];

    /**
     * @brief  An indicator for the current subframe. 
     */
    int32_t subframeId;
   
    /**
     * @brief  Handle to the DMA to transfer data.
     */
    DMA_Handle  dmaHandle;
    
    /**
     * @brief   Handle to the SOC Module
     */
    SOC_Handle                  socHandle;

	/*! @brief   UART Logging Handle */
    UART_Handle                 loggingUartHandle;

    /**
     * @brief   UART Command Handle used to interface with the CLI
     */
    UART_Handle                 commandUartHandle;

    /**
     * @brief   mmWave control handle use to initialize the link infrastructure
     * which allows communication between the MSS and BSS
     */
    MMWave_Handle               ctrlHandle;

    /**
     * @brief   Handle to the ADCBUF Driver
     */
    ADCBuf_Handle               adcBufHandle;
    
    /*!@brief   Handle to the peer Mailbox */
    Mbox_Handle              peerMailbox;

    /*! @brief   Semaphore handle for the mailbox communication */
    Semaphore_Handle            mboxSemHandle;

    /*! @brief   MSS system event handle */
    Event_Handle                eventHandle;

    /*! @brief   MSS system event handle */
    Event_Handle                eventHandleNotify;

    /*! @brief   Handle to the CBUFF Module */
    CBUFF_Handle              cbuffHandle;

    /*! @brief   Handle to the SW Triggered Session which streams out application data */
    CBUFF_SessionHandle       swSessionHandle;

    /**
     * @brief   Handle to the SOC chirp interrupt listener Handle
     */
    SOC_SysIntListenerHandle    chirpIntHandle;

    /**
     * @brief   Handle to the SOC chirp interrupt listener Handle
     */
    SOC_SysIntListenerHandle    frameStartIntHandle;

    /**
     * @brief   This is a flag which indicates if the mmWave link has been configured
     * or not.
     */
    bool                        cfgStatus;
    
    /**
     * @brief   This is a flag which indicates if the radar is tranmsitting or not
     * or not.
     */
    bool                        runningStatus;
    
    /**
     * @brief   This is a flag which indicates if the basic radar configuration is completed.
     */
    bool                        isMMWaveOpen;

    /*! @brief   mmw Demo stats */
    MmwDemo_MSS_STATS           stats;

}Srr_MSS_MCB;

/*******************************************************************************************
 * Extern Global Structures:
 *******************************************************************************************/
extern Srr_MSS_MCB gSrrMSSMCB;

/*******************************************************************************************
 * Extern IPC API:
 *******************************************************************************************/
extern int32_t IPC_init (Mailbox_Type remoteEndPoint);
extern int32_t IPC_sendChannelCfg (rlChanCfg_t* ptrChannelCfg);
extern int32_t IPC_sendLowPowerModeCfg (rlLowPowerModeCfg_t* ptrLowPowerMode);
extern int32_t IPC_sendADCOutCfg (rlAdcOutCfg_t* ptrADCOutCfg);
extern int32_t IPC_sendProfileCfg (rlProfileCfg_t* ptrProfileCfg);
extern int32_t IPC_sendChirpCfg (rlChirpCfg_t* ptrChirpCfg);
extern int32_t IPC_sendFrameCfg (rlFrameCfg_t* ptrFrameCfg);
extern int32_t IPC_sendAdvFrameCfg (rlAdvFrameCfg_t* ptrAdvFrameCfg);
extern int32_t IPC_sendSensorStart (void);
extern int32_t IPC_sendSensorStop (void);

/*******************************************************************************************
 * Extern CLI API:
 *******************************************************************************************/
extern void SRR_MSS_CLIInit (void);

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

#endif /* MSS_SRR_H */


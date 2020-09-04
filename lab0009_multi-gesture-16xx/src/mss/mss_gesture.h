/**
 *   @file  mss_gesture.h
 *
 *   @brief
 *      This is the main header file for the Millimeter Wave Demo
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
#ifndef MSS_GESTURE_H
#define MSS_GESTURE_H

#include <ti/sysbios/knl/Event.h>
#include <ti/common/mmwave_error.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/mailbox/mailbox.h>
//#include <ti/control/mmwave/mmwave.h>

#include <ti/sysbios/knl/Semaphore.h>

/* MMW Demo Include Files */
#include "../include/mmw_config.h"

#ifdef __cplusplus
extern "C" {
#endif



#define NUM_ANGLE_BINS_ONE_SIDED (16) //This should be half of what is defined in MSS DOA
#define INST_ENERGY_NORMFACTOR_dB (200)

#define NUM_NODES_FIRST_LAYER (150)
#define NUM_NODES_SECOND_LAYER (7)
#define INP_DIM (60)


/*! @brief   sensor start CLI event */
#define GESTDEMO_CLI_SENSORSTART_EVT                     Event_Id_00

/*! @brief   sensor stop CLI  event */
#define GESTDEMO_CLI_SENSORSTOP_EVT                      Event_Id_01

/*! @brief   sensor frame start CLI  event */
#define GESTDEMO_CLI_FRAMESTART_EVT                      Event_Id_02

/*! @brief   BSS CPUFAULT event */
#define GESTDEMO_BSS_CPUFAULT_EVT                        Event_Id_03

/*! @brief   BSS ESMFAULT event */
#define GESTDEMO_BSS_ESMFAULT_EVT                        Event_Id_04

/*! @brief   Monitoring report event */
#define GESTDEMO_BSS_MONITORING_REP_EVT                  Event_Id_05

/*! @brief   BSS Calibration report event */
#define GESTDEMO_BSS_CALIBRATION_REP_EVT                 Event_Id_06

/*! @brief   start completed event from DSS/MSS */
#define GESTDEMO_DSS_START_COMPLETED_EVT                 Event_Id_07

/*! @brief   stop completed event from DSS */
#define GESTDEMO_DSS_STOP_COMPLETED_EVT                  Event_Id_08

/*! @brief   start failed event from DSS/MSS */
#define GESTDEMO_DSS_START_FAILED_EVT                    Event_Id_09

/*! @brief  DSS chirp processing deadline miss event */
#define GESTDEMO_DSS_CHIRP_PROC_DEADLINE_MISS_EVT        Event_Id_10

/*! @brief  DSS frame processing deadline miss event */
#define GESTDEMO_DSS_FRAME_PROC_DEADLINE_MISS_EVT        Event_Id_11

/*Event to run ANN on MSS */
#define GESTDEMO_DSS_TO_MSS_FEATURE_TRANSFER_DONE_EVT    Event_Id_12
#define GESTDEMO_MSS_TO_APP_ANN_INITIATE_EVT             Event_Id_13


/* All CLI events */
#define GESTDEMO_CLI_EVENTS                              (GESTDEMO_CLI_SENSORSTART_EVT |    \
                                                         GESTDEMO_CLI_SENSORSTOP_EVT |     \
                                                         GESTDEMO_CLI_FRAMESTART_EVT)

/* All BSS faults events */
#define GESTDEMO_BSS_FAULT_EVENTS                        (GESTDEMO_BSS_CPUFAULT_EVT |     \
                                                         GESTDEMO_BSS_ESMFAULT_EVT )
                                                         
/* All DSS exception events signalled directly through DSS2MSS SW1 ISR, these are 
   exception events that happen in the ISRs of DSS such as missing processing deadlines
   for chirp or frame for which DSS cannot use the mailbox path to communicate with MSS
   because of the urgent nature of the events. DSP will do a local assert in its
   ISR after signalling the exception to the MSS. */
#define GESTDEMO_DSS_EXCEPTION_EVENTS            (GESTDEMO_DSS_CHIRP_PROC_DEADLINE_MISS_EVT |   \
                                                 GESTDEMO_DSS_FRAME_PROC_DEADLINE_MISS_EVT )

                                                 
                                                 
/**
 * @brief
 *  Millimeter Wave Demo Data Path ANN based detection wieghts, biases, internal/final ops.
 *
 * @details
 *  The structure is used to hold all the relevant information for ANN based detection in
 *  the data path.
 */
typedef struct ANN_struct_t_
{
    float b_1[NUM_NODES_FIRST_LAYER];
    float b_2[NUM_NODES_SECOND_LAYER];
    float W_1[NUM_NODES_FIRST_LAYER][INP_DIM];
    float W_2[NUM_NODES_SECOND_LAYER][NUM_NODES_FIRST_LAYER];
    float op_layer1[NUM_NODES_FIRST_LAYER];
    float op_layer2[NUM_NODES_SECOND_LAYER];
    float prob[NUM_NODES_SECOND_LAYER];

} ANN_struct_t;


/**
 * @brief
 *  Millimeter Wave Demo statistics
 *
 * @details
 *  The structure is used to hold the statistics information for the
 *  Millimeter Wave demo
 */
typedef struct GestDemo_MSS_STATS_t
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
}GestDemo_MSS_STATS;

/**
 * @brief
 *  Millimeter Wave Demo MCB
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  Millimeter Wave demo
 */
typedef struct GestDemo_MCB_t
{
    /*! @brief   Configuration which is used to execute the demo */
    MmwDemo_Cfg                 cfg;

    /*! @brief   CLI related configuration */    
    MmwDemo_CliCfg_t            cliCfg[RL_MAX_SUBFRAMES];

    /*! @brief   CLI related configuration common across all subframes */
    MmwDemo_CliCommonCfg_t      cliCommonCfg;
 
    /*! * @brief   Handle to the SOC Module */
    SOC_Handle                  socHandle;

    /*! @brief   UART Logging Handle */
    UART_Handle                 loggingUartHandle;

    /*! @brief   UART Command Rx/Tx Handle */
    UART_Handle                 commandUartHandle;

    /*! @brief   This is the mmWave control handle which is used
     * to configure the BSS. */
    MMWave_Handle               ctrlHandle;

    /*!@brief   Handle to the peer Mailbox */
    Mbox_Handle              peerMailbox;

    /*! @brief   Semaphore handle for the mailbox communication */
    Semaphore_Handle            mboxSemHandle;

    /*! @brief   MSS system event handle */
    Event_Handle                eventHandle;

    /*! @brief   MSS system event handle */
    Event_Handle                eventHandleNotify;

    /*! @brief   Handle to the SOC chirp interrupt listener Handle */
    SOC_SysIntListenerHandle    chirpIntHandle;

    /*! @brief   Handle to the SOC frame start interrupt listener Handle */
    SOC_SysIntListenerHandle    frameStartIntHandle;

    /*! @brief   Current status of the sensor */
    bool                         isSensorStarted;

    /*! @brief   Has the mmWave module been opened? */
    bool                        isMMWaveLinkOpen;

    bool                        isRadarSSConfigured;
    /*! @brief   mmw Demo stats */
    GestDemo_MSS_STATS           stats;
    
    /*! @brief DSS to MSS Isr Info Address */
    uint32_t                   dss2mssIsrInfoAddress;
} GestDemo_MCB;

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
extern int32_t GestDemo_mssDataPathConfig(void);
extern int32_t GestDemo_mssDataPathStart(void);
extern int32_t GestDemo_mssDataPathStop(void);

/* Sensor Management Module Exported API */
extern int32_t GestDemo_notifySensorStart(bool doReconfig);
extern int32_t GestDemo_notifySensorStop(void);
extern int32_t GestDemo_waitSensorStartComplete(void);
extern void GestDemo_waitSensorStopComplete(void);

extern void _GestDemo_mssAssert(int32_t expression, const char *file, int32_t line);
#define GestDemo_mssAssert(expression) {                                      \
                                         _GestDemo_mssAssert(expression,      \
                                                  __FILE__, __LINE__);         \
                                         DebugP_assert(expression);             \
                                        }

/**
 *  @b Description
 *  @n
 *      Preprocess the inputs to ANN detector, centers the input vector b/w [-1,1]
 *  @retval
 *     None
 */
void GetsDemo_preprocessANNInputs(float FeaturevectorDSS[], float FeaturevectorMSS[]);


#ifdef __cplusplus
}
#endif

#endif /* MSS_GESTURE_H */


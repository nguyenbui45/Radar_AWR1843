/**
 *   @file  mmw_messages.h
 *
 *   @brief
 *      This is the main header file for the Millimeter Wave Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
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
#ifndef MMW_MESSAGES_H
#define MMW_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif
/* UART API */
#include <ti/demo/io_interface/mmw_output.h>


/* All CLI events */
#define MMWDEMO_CLI_EVENTS                              (MMWDEMO_CLI_SENSORSTART_EVT |    \
                                                         MMWDEMO_CLI_SENSORSTOP_EVT |     \
                                                         MMWDEMO_CLI_FRAMESTART_EVT)

/* All BSS faults events */
#define MMWDEMO_BSS_FAULT_EVENTS                        (MMWDEMO_BSS_CPUFAULT_EVT |     \
                                                         MMWDEMO_BSS_ESMFAULT_EVT )

/* All DSS exception events signalled directly through DSS2MSS SW1 ISR, these are
   exception events that happen in the ISRs of DSS such as missing processing deadlines
   for chirp or frame for which DSS cannot use the mailbox path to communicate with MSS
   because of the urgent nature of the events. DSP will do a local assert in its
   ISR after signalling the exception to the MSS. */
#define MMWDEMO_DSS_EXCEPTION_EVENTS            (MMWDEMO_DSS_CHIRP_PROC_DEADLINE_MISS_EVT |   \
                                                 MMWDEMO_DSS_FRAME_PROC_DEADLINE_MISS_EVT )

                                                 
/*! @brief   mmwave Config event triggered from mmwave config call back function */
#define MMWDEMO_CONFIG_EVT                                  Event_Id_00

/*! @brief   mmwave Start event triggered from mmwave start call back function */
#define MMWDEMO_START_EVT                                   Event_Id_01

/*! @brief   BSS CPUFAULT event */
#define MMWDEMO_BSS_CPUFAULT_EVT                            Event_Id_02

/*! @brief   BSS ESMFAULT event */
#define MMWDEMO_BSS_ESMFAULT_EVT                            Event_Id_03

/*! @brief   BSS stop complete event  */
#define MMWDEMO_BSS_STOP_COMPLETE_EVT                       Event_Id_04

/*! @brief   Frame start interupt triggered event */
#define MMWDEMO_FRAMESTART_EVT                              Event_Id_05

/*! @brief   Chirp available interrupt triggered event */
#define MMWDEMO_CHIRP_EVT                                   Event_Id_06

/*! @brief   Stop complete event  */
#define MMWDEMO_DSS_STOP_COMPLETED_EVT                      Event_Id_07

/*! @brief   start completed event from DSS/MSS */
#define MMWDEMO_DSS_START_COMPLETED_EVT                     Event_Id_08

/*! @brief   start failed event from DSS/MSS */
#define MMWDEMO_DSS_START_FAILED_EVT                        Event_Id_09

/*! @brief   BSS frame trigger ready event */
#define MMWDEMO_BSS_FRAME_TRIGGER_READY_EVT                 Event_Id_10

/*! @brief   Monitoring report event */
#define MMWDEMO_BSS_MONITORING_REP_EVT                      Event_Id_11

/*! @brief   BSS Calibration report event */
#define MMWDEMO_BSS_CALIBRATION_REP_EVT                     Event_Id_12

#define MMWDEMO_DSS_GETVERSION_DATA_EVT                     Event_Id_13
#define MMWDEMO_DSS_INIT_CONFIG_DONE_EVT                    Event_Id_14
#define MMWDEMO_DSS_MMWAVELINK_FAILURE_EVT                  Event_Id_15
#define MMWDEMO_DATAPATH_STOP_EVT                           Event_Id_16
/*! @brief  DSS chirp processing deadline miss event */
#define MMWDEMO_DSS_CHIRP_PROC_DEADLINE_MISS_EVT            Event_Id_17

/*! @brief  DSS frame processing deadline miss event */
#define MMWDEMO_DSS_FRAME_PROC_DEADLINE_MISS_EVT            Event_Id_18

/**
 * @brief
 *  Message types used in Millimeter Wave Demo for Mailbox communication 
 * between MSS and DSS.
 *
 * @details
 *  The enum is used to hold all the messages types used for Mailbox communication
 * between MSS and DSS in mmw Demo.
 */
typedef enum MmwDemo_message_type_e 
{
    /*! @brief   message types for MSS to DSS communication */
    MMWDEMO_MSS2DSS_GUIMON_CFG = 0xFEED0001,
    MMWDEMO_MSS2DSS_CFAR_RANGE_CFG,
    MMWDEMO_MSS2DSS_CFAR_DOPPLER_CFG,
    MMWDEMO_MSS2DSS_PEAK_GROUPING_CFG,
    MMWDEMO_MSS2DSS_MULTI_OBJ_BEAM_FORM,
    MMWDEMO_MSS2DSS_CALIB_DC_RANGE_SIG,
    MMWDEMO_MSS2DSS_DETOBJ_SHIPPED,
    MMWDEMO_MSS2DSS_SET_DATALOGGER,
    MMWDEMO_MSS2DSS_ADCBUFCFG,
    MMWDEMO_MSS2DSS_EXTENDED_MAX_VELOCITY,
    MMWDEMO_MSS2DSS_CLUTTER_REMOVAL,
    MMWDEMO_MSS2DSS_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE,
    MMWDEMO_MSS2DSS_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE,
    MMWDEMO_MSS2DSS_BPM_CFG,
    MMWDEMO_MSS2DSS_NEAR_FIELD_CFG,
    MMWDEMO_MSS2DSS_CQ_SATURATION_MONITOR,
    MMWDEMO_MSS2DSS_LVDSSTREAM_CFG,
    MMWDEMO_MSS2DSS_CQ_SIGIMG_MONITOR,
    MMWDEMO_MSS2DSS_ANALOG_MONITOR,
    MMWDEMO_MSS2DSS_PROFILE_CFG,
    MMWDEMO_MSS2DSS_CHIRP_CFG,
    MMWDEMO_MSS2DSS_OPEN_CFG,
    MMWDEMO_MSS2DSS_CONTROL_CFG,
    MMWDEMO_MSS2DSS_SENSOR_START_CMD,
    MMWDEMO_MSS2DSS_SENSOR_STOP_CMD,

    /*! @brief   message types for DSS to MSS communication */
    MMWDEMO_DSS2MSS_CONFIGDONE = 0xFEED0100,
    MMWDEMO_DSS2MSS_DETOBJ_READY,
    MMWDEMO_DSS2MSS_STOPDONE,
    MMWDEMO_DSS2MSS_ASSERT_INFO,
    MMWDEMO_DSS2MSS_ISR_INFO_ADDRESS,
    MMWDEMO_DSS2MSS_MEASUREMENT_INFO,
    MMWDEMO_DSS2MSS_STATUS_INFO,
    MMWDEMO_DSS2MSS_VERSION_INFO

}MmwDemo_message_type;


/*! @brief Software interrupt number used by DSS to signal exception from DSS to MSS */
#define MMWDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_DSS    1
/*! @brief Software interrupt ID on MSS corresponding to 
           @ref MMWDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_DSS */
#define MMWDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_MSS    SOC_XWR16XX_MSS_DSS2MSS_SW1_INT

/** @defgroup DSS_TO_MSS_EXCEPTION_IDS DSS to MSS Exception IDs
 *
 * @brief
 *  Exception ID definitions for DSS to MSS urgent exception signalling through 
 *  software interrupt.
 *
 @{ */

 /*! @brief   DSS to MSS chirp processing deadline miss exception ID */
#define MMWDEMO_DSS2MSS_CHIRP_PROC_DEADLINE_MISS_EXCEPTION 0

/*! @brief   DSS to MSS frame processing deadline miss exception ID */
#define MMWDEMO_DSS2MSS_FRAME_PROC_DEADLINE_MISS_EXCEPTION 1

/*! @brief   DSS to MSS frame trigger notification ID */
#define MMWDEMO_DSS2MSS_FRAME_TRIGGERED_EVENT              2

/** @}*/ /* end defgroup DSS_TO_MSS_EXCEPTION_IDS */

/**
 * @brief
 *  TLV part of the message from DSS to MSS on data path detection information.
 *
 * @details
 *  The structure describes the message item 
 */
typedef struct MmwDemo_msgTlv_t
{
    /*! @brief   Payload type */
    uint32_t    type;

    /*! @brief   Length in bytes */
    uint32_t    length;

    /*! @brief Address of the payload */
    uint32_t   address;
} MmwDemo_msgTlv;

/**
 * @brief
 *  Message for reporting detection information from data path to MSS
 *
 * @details
 *  The structure defines the message body for detection information from from data path to MSS.
 */
typedef struct MmwDemo_detObjMsg_t
{
    /*! @brief Header of the detection information message */
    MmwDemo_output_message_header header;

    /*! @brief TLVs of the detection information */
    MmwDemo_msgTlv   tlv[MMWDEMO_OUTPUT_MSG_MAX];
} MmwDemo_detInfoMsg;

#define MMWDEMO_MAX_FILE_NAME_SIZE 128
/**
 * @brief
 *  Message for reporting DSS assertion information
 *
 * @details
 *  The structure defines the message body for the information
 *  on a DSS exception that should be forwarded to the MSS.
 */
typedef struct MmwDemo_dssAssertInfoMsg_t
{
    /*! @brief file name */
    char     file[MMWDEMO_MAX_FILE_NAME_SIZE];

    /*! @brief line number */
    uint32_t line;
} MmwDemo_dssAssertInfoMsg;


typedef struct MmwDemo_dssStatusMsg_t
{
    uint16_t   eventType;
    int32_t    errVal;
}MmwDemo_dssStatusMsg;

/**
 * @brief
 *  Message body used in Millimeter Wave Demo for passing configuration from MSS
 * to DSS.
 *
 * @details
 *  The union defines the message body for various configuration messages. 
 */
typedef union MmwDemo_message_body_u 
{
    /*! @brief   Gui Monitor Selection */
    MmwDemo_GuiMonSel     guiMonSel;

    /*! @brief   CFAR Range/Doppler configuraiton */
    MmwDemo_CfarCfg       cfarCfg;

    /*! @brief   Peak grouping configuration */
     MmwDemo_PeakGroupingCfg peakGroupingCfg;

     /*! @brief   Multi object beam forming configuration */
     MmwDemo_MultiObjBeamFormingCfg multiObjBeamFormingCfg;

     /*! @brief   Calibrate DC (zero) range signature */
     MmwDemo_CalibDcRangeSigCfg calibDcRangeSigCfg;

     /*! @brief   Extended maximum velocity configuration */
     MmwDemo_ExtendedMaxVelocityCfg extendedMaxVelocityCfg;

     /*! @brief   Near field correction configuration */
     MmwDemo_NearFieldCorrectionCfg nearFieldCorrectionCfg;

     /*! @brief   ChirpQuality - RX saturation monitor */
     rlRxSatMonConf_t            cqSatMonCfg;

     /*! @brief   ChirpQuality - Signal and image band energy monitor */
     rlSigImgMonConf_t           cqSigImgMonCfg;

     /*! @brief   ChirpQuality - Signal and image band energy monitor */
     MmwDemo_AnaMonitorCfg       anaMonCfg;

     /*! @brief  LVDS stream configuration */
     MmwDemo_LvdsStreamCfg lvdsStreamCfg;

     /*! @brief   Clutter removal configuration */
     MmwDemo_ClutterRemovalCfg clutterRemovalCfg;

    /*! @brief   Detection Information message */
    MmwDemo_detInfoMsg     detObj;

    /*! @brief   ADCBUF configuration */
    MmwDemo_ADCBufCfg       adcBufCfg;

    /*! @brief   Datapath output logger setting */
    uint8_t               dataLogger;

    /*! @brief   Configuration for compensation for range bias 
                 and rx channel phase offset */
    MmwDemo_compRxChannelBiasCfg_t compRxChanCfg;

    /*! @brief   Configuration for measurement of range bias
                 and rx channel phase offset */
    MmwDemo_measureRxChannelBiasCfg_t measureRxChanCfg;

    /*! @brief   BPM cfg */
    MmwDemo_BpmCfg bpmCfg;

    /*! @brief   DSS assertion information */
    MmwDemo_dssAssertInfoMsg  assertInfo;

    /*! @brief   Misc DSS status info */
    MmwDemo_dssStatusMsg      dssStatusInfo;

    rlChirpCfg_t            chirpCfg;
    rlProfileCfg_t          profileCfg;
    rlBpmChirpCfg_t         bpmChirpCfg;
    rlVersion_t             devVerArgs;
    
    MMWave_CtrlCfg           ctrlCfg;

    MMWave_OpenCfg          openCfg;

    /*! @brief Address of DSS to MSS ISR information storage, typically in HSRAM */
    uint32_t  dss2mssISRinfoAddress;
} MmwDemo_message_body;



/*! @brief For advanced frame config, below define means the configuration given is
 * global at frame level and therefore it is broadcast to all sub-frames.
 */
#define MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG (-1)

/**
 * @brief
 *  DSS/MSS communication messages
 *
 * @details
 *  The structure defines the message structure used to commuincate between MSS
 * and DSS.
 */
typedef struct MmwDemo_message_t
{
    /*! @brief   message type */
    MmwDemo_message_type      type;

    /*! @brief   message length : PROC_TBA does body need to be pointer and not a union structure?*/
    //uint32_t                  len;

    /*! @brief   Subframe number for which this message is applicable. 
                 Valid only when advanced frame config is used.
                 When advanced frame config is not used, this field should
                 be set to MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG.
    */
    int8_t                   subFrameNum;

    /*! @brief  message body */
    MmwDemo_message_body      body;

} MmwDemo_message;

#ifdef __cplusplus
}
#endif

#endif /* MMW_MESSAGES_H */

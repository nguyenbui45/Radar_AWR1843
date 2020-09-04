/*
 *   @file  mss_gesture_cli.c
 *
 *   @brief
 *      MSS Minimal CLI Implementation for the Gesture TI Design
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

/* BIOS/XDC Include Files. */
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/cli/cli.h>

/* Application Include Files: */
#include "mss_gesture.h"
#include "../common/gesture_messages.h"
#include "../common/gesture_config_consts.h"


#define MMWDEMO_SATURATE_HIGH(x) ( (x) > 32767 ? (x) = 32767 : (x))
#define MMWDEMO_SATURATE_LOW(x) ( (x) < -32768 ? (x) = -32768 : (x))

/**************************************************************************
 *************************** Local Functions ******************************
 **************************************************************************/
static int32_t GestDemo_CLISensorStart (int32_t argc, char* argv[]);
static int32_t GestDemo_CLISensorStop (int32_t argc, char* argv[]);
static int32_t GestDemo_CLIGuiMonSel (int32_t argc, char* argv[]);
static int32_t GestDemo_CLIGestureFeatureCfg (int32_t argc, char* argv[]);
static int32_t GestDemo_CLIGestureTrainingParams (int32_t argc, char* argv[]);
static int32_t GestDemo_CLICalibDcRangeSig (int32_t argc, char* argv[]);
static int32_t GestDemo_CLINearFieldCorrection (int32_t argc, char* argv[]);
static int32_t GestDemo_CLIClutterRemoval (int32_t argc, char* argv[]);
static int32_t GestDemo_CLIADCBufCfg (int32_t argc, char* argv[]);
static int32_t GestDemo_CLICompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
static int32_t GestDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);

/**************************************************************************
 *************************** Extern Definitions *******************************
 **************************************************************************/
extern GestDemo_MCB    gGestMssMCB;
extern int32_t GestDemo_mboxWrite(GestDemo_message     * message);
extern int32_t MmwaveLink_configRadarSS(void);

/**************************************************************************
 ****************************** CLI Functions *****************************
 **************************************************************************/

/* It will do all the basic Configuration and send Default config data to DSS */
static int32_t GestDemo_CLIBasicCfg (int32_t argc, char* argv[])
{
    GestDemo_message     message;

    gGestMssMCB.isRadarSSConfigured = true;
 
    GestDemo_CLICalibDcRangeSig(0,0);
    
    GestDemo_CLINearFieldCorrection(0,0);
    
    GestDemo_CLIClutterRemoval(0,0);
    
    GestDemo_CLIADCBufCfg(0,0);
    
    GestDemo_CLICompRangeBiasAndRxChanPhaseCfg(0,0);
    
    GestDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg(0,0);
    

    memset((void *)&message, 0, sizeof(GestDemo_message));

    /* notify DSS for Sensor-start */
    message.type = GESTDEMO_MSS2DSS_SET_CONFIG;

    if (GestDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for starting the sensor
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t GestDemo_CLISensorStart (int32_t argc, char* argv[])
{
    bool doReconfig = true;
    if (argc==2)
    {
        doReconfig = (bool) atoi (argv[1]);
    }
    /* Post sensorSTart event to notify configuration is done */
    GestDemo_notifySensorStart(doReconfig);
    /* Pend for completion */
    return (GestDemo_waitSensorStartComplete());
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for stopping the sensor
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t GestDemo_CLISensorStop (int32_t argc, char* argv[])
{
    /* Not Supported for Gesture Demo, as this demo uses fixed configuration
     * so user can't change the config and do the sensorStop
     */
#if 0
    /* Post sensorSTOP event to notify sensor stop command */
    GestDemo_notifySensorStop();
    /* Pend for completion */
    GestDemo_waitSensorStopComplete();
#else
    CLI_write ("This command is not supported in this demo. \n");
#endif
    return 0;
}

static int32_t GestDemo_CLIGetSubframe (int32_t argc, char* argv[], int32_t expectedArgc, int8_t* subFrameNum)
{
    int8_t subframe;
    
    /* Sanity Check: Minimum argument check */
    if (argc != expectedArgc)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Subframe info is always in position 1*/
    subframe = (int8_t) atoi(argv[1]);

    if(subframe >= (int8_t)RL_MAX_SUBFRAMES)
    {
        CLI_write ("Error: Subframe number is invalid\n");
        return -1;
    }
    
    *subFrameNum = (int8_t)subframe;

    return 0;
}

static void GestDemo_mssCfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum)
{    
    /* if subFrameNum undefined, broadcast to all sub-frames */
    if(subFrameNum == GESTDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG)
    {
        uint8_t  indx;
        for(indx = 0; indx < RL_MAX_SUBFRAMES; indx++)
        {
            memcpy((void *)((uint32_t) &gGestMssMCB.cliCfg[indx] + offset), srcPtr, size);
        }
    }
    else
    {
        /* Apply configuration to specific subframe (or to position zero for the legacy case
           where there is no advanced frame config) */
        memcpy((void *)((uint32_t) &gGestMssMCB.cliCfg[subFrameNum] + offset), srcPtr, size);
    }
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for gui monitoring configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t GestDemo_CLIGuiMonSel (int32_t argc, char* argv[])
{
    GestDemo_GuiMonSel   guiMonSel;
    GestDemo_message     message;
    int8_t              subFrameNum;

    if(GestDemo_CLIGetSubframe(argc, argv, 9, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize the guiMonSel configuration: */
    memset ((void *)&guiMonSel, 0, sizeof(GestDemo_GuiMonSel));

    /* Populate configuration: */
    guiMonSel.detectedObjects           = atoi (argv[2]);
    guiMonSel.logMagRange               = atoi (argv[3]);
    guiMonSel.noiseProfile              = atoi (argv[4]);
    guiMonSel.rangeAzimuthHeatMap       = atoi (argv[5]);
    guiMonSel.rangeDopplerHeatMap       = atoi (argv[6]);
    guiMonSel.statsInfo                 = atoi (argv[7]);
    guiMonSel.sendGestureFeatures       = atoi (argv[8]);
    
    GestDemo_mssCfgUpdate((void *)&guiMonSel, offsetof(MmwDemo_CliCfg_t, guiMonSel), 
        sizeof(GestDemo_GuiMonSel), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(GestDemo_message));

    message.type = GESTDEMO_MSS2DSS_GUIMON_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.guiMonSel, (void *)&guiMonSel, sizeof(GestDemo_GuiMonSel));

    if (GestDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the version command.
 *      Visualizer required this API to get the SDK version.
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  \ingroup CLI_UTIL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MMWaveVersion (int32_t argc, char* argv[])
{
    int32_t        retVal;
    SOC_PartNumber  devicePartNumber;
    int32_t         errCode;
    extern GestDemo_MCB    gGestMssMCB;

    /* print the platform */
    CLI_write ("Platform                : xWR16xx\n");

    /* Get the version string: */
    retVal = SOC_getDevicePartNumber(gGestMssMCB.socHandle, &devicePartNumber, &errCode);
    if (retVal < 0)
    {
        CLI_write ("Error: Unable to get the device version from SOC[Error %d]\n", errCode);
        return -1;
    }

    /* Display the version information on the CLI Console: */
    CLI_write ("mmWave SDK Version      : %02d.%02d.%02d.%02d\n",
                            MMWAVE_SDK_VERSION_MAJOR,
                            MMWAVE_SDK_VERSION_MINOR,
                            MMWAVE_SDK_VERSION_BUGFIX,
                            MMWAVE_SDK_VERSION_BUILD);

    /* Version string has been formatted successfully. */
    return 0;
}

uint32_t log2Approx(uint32_t x)
{
    uint32_t idx, detectFlag = 0;

    if ( x < 2)
    {
        return (0);
    }

    idx = 32U;
    while((detectFlag==0U) || (idx==0U))
    {
        if(x & 0x80000000U)
        {
            detectFlag = 1;
        }
        x <<= 1U;
        idx--;
    }

    if(x != 0)
    {
        idx = idx + 1;
    }

    return(idx);
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for DC range calibration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t GestDemo_CLICalibDcRangeSig (int32_t argc, char* argv[])
{
    MmwDemo_CalibDcRangeSigCfg cfg;
    GestDemo_message            message;
    uint32_t                   log2NumAvgChirps;
    int8_t                     subFrameNum;

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_CalibDcRangeSigCfg));

#ifdef FEATURES_ENABLE_OVER_CLI
    if(GestDemo_CLIGetSubframe(argc, argv, 6, &subFrameNum) < 0)
    {
        return -1;
    }
    /* Populate configuration: */
    cfg.enabled          = (uint16_t) atoi (argv[2]);
    cfg.negativeBinIdx   = (int16_t)  atoi (argv[3]);
    cfg.positiveBinIdx   = (int16_t)  atoi (argv[4]);
    cfg.numAvgChirps     = (uint16_t)  atoi (argv[5]);
#else
    
    subFrameNum = -1;
    /* Populate configuration: */
    cfg.enabled          = (uint16_t) GESTURE_CALIB_DC_ENABLE;
    cfg.negativeBinIdx   = (int16_t)  GESTURE_CALIB_DC_NEG_BIN_IDX;
    cfg.positiveBinIdx   = (int16_t)  GESTURE_CALIB_DC_POS_BIN_IDX;
    cfg.numAvgChirps     = (uint16_t) GESTURE_CALIB_DC_NEG_AVG_CHIRP;
#endif


    if (cfg.negativeBinIdx > 0)
    {
        CLI_write ("Error: Invalid negative bin index\n");
        return -1;
    }
    if ((cfg.positiveBinIdx - cfg.negativeBinIdx + 1) > DC_RANGE_SIGNATURE_COMP_MAX_BIN_SIZE)
    {
        CLI_write ("Error: Number of bins exceeds the limit\n");
        return -1;
    }
    log2NumAvgChirps = (uint32_t) log2Approx (cfg.numAvgChirps);
    if (cfg.numAvgChirps != (1 << log2NumAvgChirps))
    {
        CLI_write ("Error: Number of averaged chirps is not power of two\n");
        return -1;
    }

    /* Save Configuration to use later */
    GestDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, calibDcRangeSigCfg),
        sizeof(MmwDemo_CalibDcRangeSigCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(GestDemo_message));

    message.type = GESTDEMO_MSS2DSS_CALIB_DC_RANGE_SIG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.calibDcRangeSigCfg, (void *)&cfg, sizeof(MmwDemo_CalibDcRangeSigCfg));

    if (GestDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for Near field correction Configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t GestDemo_CLINearFieldCorrection (int32_t argc, char* argv[])
{
    MmwDemo_NearFieldCorrectionCfg cfg;
    GestDemo_message                message;
    int8_t                         subFrameNum;

    /* Initialize configuration for Near Field Correction */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_NearFieldCorrectionCfg));

#ifdef FEATURES_ENABLE_OVER_CLI
    if(GestDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Populate configuration: */
    cfg.enabled       = (uint8_t) atoi(argv[2]);
    cfg.startRangeIdx = (uint16_t) atoi(argv[3]);
    cfg.endRangeIdx   = (uint16_t) atoi(argv[4]);
#else
    subFrameNum = -1;
    /* Populate configuration: */
    cfg.enabled       = (uint8_t) GESTURE_NEARFIELD_ENABLE;
    cfg.startRangeIdx = (uint16_t) GESTURE_NEARFIELD_START_RANGE_IDX;
    cfg.endRangeIdx   = (uint16_t) GESTURE_NEARFIELD_END_RANGE_IDX;
#endif

    /* Save Configuration to use later */
    GestDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, nearFieldCorrectionCfg),
        sizeof(MmwDemo_NearFieldCorrectionCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(GestDemo_message));

    message.type = GESTDEMO_MSS2DSS_NEAR_FIELD_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.nearFieldCorrectionCfg, (void *)&cfg, 
           sizeof(MmwDemo_NearFieldCorrectionCfg));

    if (GestDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      Clutter removal Configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t GestDemo_CLIClutterRemoval (int32_t argc, char* argv[])
{
    MmwDemo_ClutterRemovalCfg cfg;
    GestDemo_message     message;
    int8_t              subFrameNum;

    /* Initialize configuration for clutter removal */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_ClutterRemovalCfg));
#ifdef FEATURES_ENABLE_OVER_CLI
    if(GestDemo_CLIGetSubframe(argc, argv, 3, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Populate configuration: */
    cfg.enabled          = (uint16_t) atoi (argv[2]);
#else
    subFrameNum = -1;
    /* Populate configuration: */
    cfg.enabled          = (uint16_t) GESTURE_CLUTTER_REMOVAL_ENABLE;
#endif

    /* Save Configuration to use later */
    GestDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, clutterRemovalCfg),
        sizeof(MmwDemo_ClutterRemovalCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(GestDemo_message));

    message.type = GESTDEMO_MSS2DSS_CLUTTER_REMOVAL;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.clutterRemovalCfg, (void *)&cfg, sizeof(MmwDemo_ClutterRemovalCfg));

    if (GestDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for data logger set command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t GestDemo_CLIADCBufCfg (int32_t argc, char* argv[])
{
    MmwDemo_ADCBufCfg   adcBufCfg;
    GestDemo_message     message;
    int8_t              subFrameNum;

    /* Initialize the ADC Output configuration: */
    memset ((void *)&adcBufCfg, 0, sizeof(MmwDemo_ADCBufCfg));

#ifdef FEATURES_ENABLE_OVER_CLI
    if(GestDemo_CLIGetSubframe(argc, argv, 6, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Populate configuration: */
    adcBufCfg.adcFmt          = (uint8_t) atoi (argv[2]);
    adcBufCfg.iqSwapSel       = (uint8_t) atoi (argv[3]);
    adcBufCfg.chInterleave    = (uint8_t) atoi (argv[4]);
    adcBufCfg.chirpThreshold  = (uint8_t) atoi (argv[5]);
#else
    subFrameNum = -1;
    /* Populate configuration: */
    adcBufCfg.adcFmt          = (uint8_t) GESTURE_ADCBUF_OUT_FORMAT;
    adcBufCfg.iqSwapSel       = (uint8_t) GESTURE_ADCBUF_IQ_SWAP;
    adcBufCfg.chInterleave    = (uint8_t) GESTURE_ADCBUF_INTERLEAVED;
    adcBufCfg.chirpThreshold  = (uint8_t) GESUTURE_ADCBUFF_CHIRP_THRESHOLD;
#endif
    /* Save Configuration to use later */
    GestDemo_mssCfgUpdate((void *)&adcBufCfg, offsetof(MmwDemo_CliCfg_t, adcBufCfg),
        sizeof(MmwDemo_ADCBufCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(GestDemo_message));
    message.type = GESTDEMO_MSS2DSS_ADCBUFCFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.adcBufCfg, (void *)&adcBufCfg, sizeof(MmwDemo_ADCBufCfg));

    if (GestDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for compensation of range bias and channel phase offsets
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t GestDemo_CLICompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    MmwDemo_compRxChannelBiasCfg_t   cfg;
    GestDemo_message     message;
    int32_t Re, Im;
    int32_t argInd;
    int32_t i;

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_compRxChannelBiasCfg_t));

#ifdef FEATURES_ENABLE_OVER_CLI
    /* Sanity Check: Minimum argument check */
    if (argc != (1+1+SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    /* Populate configuration: */
    cfg.rangeBias          = (float) atof (argv[1]);

    argInd = 2;
    for (i=0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        Re = (int32_t) (atof (argv[argInd++]) * 32768.);
        Re = MMWDEMO_SATURATE_HIGH(Re);
        Re = MMWDEMO_SATURATE_LOW(Re);
        cfg.rxChPhaseComp[i].real = (int16_t) Re;

        Im = (int32_t) (atof (argv[argInd++]) * 32768.);
        Im = MMWDEMO_SATURATE_HIGH(Im);
        Im = MMWDEMO_SATURATE_LOW(Im);
        cfg.rxChPhaseComp[i].imag = (int16_t) Im;

    }
#else
    float cfgData[17] =  {0.0, 1, 0, 1, 0,1,0,1,0,1,0,1,0,1,0,1,0};
    
    /* Populate configuration: */
    cfg.rangeBias          = (float) cfgData[1];

    argInd = 2;
    for (i=0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        Re = (int32_t) (cfgData[argInd++] * 32768.);
        Re = MMWDEMO_SATURATE_HIGH(Re);
        Re = MMWDEMO_SATURATE_LOW(Re);
        cfg.rxChPhaseComp[i].real = (int16_t) Re;

        Im = (int32_t) (cfgData[argInd++] * 32768.);
        Im = MMWDEMO_SATURATE_HIGH(Im);
        Im = MMWDEMO_SATURATE_LOW(Im);
        cfg.rxChPhaseComp[i].imag = (int16_t) Im;
    }
#endif
    /* Save Configuration to use later */
    memcpy((void *) &gGestMssMCB.cliCommonCfg.compRxChanCfg, &cfg, sizeof(MmwDemo_compRxChannelBiasCfg_t));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(GestDemo_message));
    message.type = GESTDEMO_MSS2DSS_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE;
    memcpy((void *)&message.body.compRxChanCfg, (void *)&cfg, sizeof(MmwDemo_compRxChannelBiasCfg_t));

    if (GestDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for measurement configuration of range bias
 *      and channel phase offsets
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t GestDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    MmwDemo_measureRxChannelBiasCfg_t   cfg;
    GestDemo_message     message;

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_measureRxChannelBiasCfg_t));

#ifdef FEATURES_ENABLE_OVER_CLI
    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    cfg.enabled          = (uint8_t) atoi (argv[1]);
    cfg.targetDistance   = (float) atof (argv[2]);
    cfg.searchWinSize   = (float) atof (argv[3]);
#else
    /* Populate configuration: */
    cfg.enabled          = (uint8_t) 0;
    cfg.targetDistance   = (float) 1.5;
    cfg.searchWinSize   = (float) 0.2;
#endif

    /* Save Configuration to use later */
    memcpy((void *) &gGestMssMCB.cliCommonCfg.measureRxChanCfg, &cfg, sizeof(MmwDemo_measureRxChannelBiasCfg_t));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(GestDemo_message));
    message.type = GESTDEMO_MSS2DSS_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE;
    memcpy((void *)&message.body.measureRxChanCfg, (void *)&cfg, sizeof(MmwDemo_measureRxChannelBiasCfg_t));

    if (GestDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for configuring CQ RX Saturation monitor
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t GestDemo_CLIChirpQualityRxSatMonCfg (int32_t argc, char* argv[])
{
    rlRxSatMonConf_t        cqSatMonCfg;
    GestDemo_message         message;

    /* Sanity Check: Minimum argument check */
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cqSatMonCfg, 0, sizeof(rlRxSatMonConf_t));

    /* Populate configuration: */
    cqSatMonCfg.profileIndx                 = (uint8_t) atoi (argv[1]);
    
    if(cqSatMonCfg.profileIndx < RL_MAX_PROFILES_CNT)
    {
        
        cqSatMonCfg.satMonSel                   = (uint8_t) atoi (argv[2]);
        cqSatMonCfg.primarySliceDuration        = (uint16_t) atoi (argv[3]);
        cqSatMonCfg.numSlices                   = (uint16_t) atoi (argv[4]);
        cqSatMonCfg.rxChannelMask               = (uint8_t) atoi (argv[5]);
        
        /* Save Configuration to use later */
        memcpy((void *) &gGestMssMCB.cliCommonCfg.cqSatMonCfg[cqSatMonCfg.profileIndx], 
                       &cqSatMonCfg, 
                       sizeof(rlRxSatMonConf_t));

        /* Send configuration to DSS */
        memset((void *)&message, 0, sizeof(GestDemo_message));
        message.type = GESTDEMO_MSS2DSS_CQ_SATURATION_MONITOR;
        memcpy((void *)&message.body.cqSatMonCfg, (void *)&cqSatMonCfg, sizeof(rlRxSatMonConf_t));

        if (GestDemo_mboxWrite(&message) == 0)
            return 0;
        else
            return -1;
    }
    else
    {
        return -1;
    }
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for configuring CQ Singal & Image band monitor
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t GestDemo_CLIChirpQualitySigImgMonCfg (int32_t argc, char* argv[])
{
    rlSigImgMonConf_t       cqSigImgMonCfg;
    GestDemo_message         message;

    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cqSigImgMonCfg, 0, sizeof(rlSigImgMonConf_t));

    /* Populate configuration: */
    cqSigImgMonCfg.profileIndx              = (uint8_t) atoi (argv[1]);

    if(cqSigImgMonCfg.profileIndx < RL_MAX_PROFILES_CNT)
    {
    
        cqSigImgMonCfg.numSlices            = (uint8_t) atoi (argv[2]);
        cqSigImgMonCfg.timeSliceNumSamples  = (uint16_t) atoi (argv[3]);

        /* Save Configuration to use later */
        memcpy((void *) &gGestMssMCB.cliCommonCfg.cqSigImgMonCfg[cqSigImgMonCfg.profileIndx], 
                &cqSigImgMonCfg, 
                sizeof(rlSigImgMonConf_t));

        /* Send configuration to DSS */
        memset((void *)&message, 0, sizeof(GestDemo_message));
        message.type = GESTDEMO_MSS2DSS_CQ_SIGIMG_MONITOR;
        memcpy((void *)&message.body.cqSigImgMonCfg, (void *)&cqSigImgMonCfg, sizeof(rlSigImgMonConf_t));

        if (GestDemo_mboxWrite(&message) == 0)
            return 0;
        else
            return -1;
    }
    else
    {
        return -1;
    }
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for enabling analog monitors
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t GestDemo_CLIAnalogMonitorCfg (int32_t argc, char* argv[])
{
    GestDemo_message     message;
    
    /* Sanity Check: Minimum argument check */
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Save Configuration to use later */
    gGestMssMCB.cliCommonCfg.anaMonCfg.rxSatMonEn = atoi (argv[1]);
    gGestMssMCB.cliCommonCfg.anaMonCfg.sigImgMonEn = atoi (argv[2]);
    
    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(GestDemo_message));
    message.type = GESTDEMO_MSS2DSS_ANALOG_MONITOR;
    memcpy((void *)&message.body.anaMonCfg, 
            (void *)&gGestMssMCB.cliCommonCfg.anaMonCfg, 
            sizeof(MmwDemo_AnaMonitorCfg));

    if (GestDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the High Speed Interface
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t GestDemo_CLILvdsStreamCfg (int32_t argc, char* argv[])
{

    int8_t                  subFrameNum;
    MmwDemo_LvdsStreamCfg   cfg;
    GestDemo_message         message;
    
    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    
    if(GestDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_LvdsStreamCfg));

    /* Populate configuration: */
    cfg.isHeaderEnabled = (bool) atoi(argv[2]) ;
    cfg.dataFmt         = (uint8_t) atoi(argv[3]) ;
    cfg.isSwEnabled     = (bool) atoi(argv[4]) ;

    /* Save Configuration to use later */
    GestDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, lvdsStreamCfg),
        sizeof(MmwDemo_LvdsStreamCfg), subFrameNum);
        
    message.type = GESTDEMO_MSS2DSS_LVDSSTREAM_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.lvdsStreamCfg, (void *)&cfg, sizeof(MmwDemo_LvdsStreamCfg));

    if (GestDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/* These parameters will be overwritten using CLI command/config */
extern float detect_feature_threshold;
extern int16_t rangebin_stop;
extern int16_t posdopplerbin_stop;


static int32_t GestDemo_CLIGestureFeatureCfg (int32_t argc, char* argv[])
{
    GestDemo_GestureFeatureCfg_t   cfg;
    GestDemo_message     message;
    int8_t              subFrameNum;

    if(GestDemo_CLIGetSubframe(argc, argv, 7, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(GestDemo_GestureFeatureCfg_t));

    /* Populate configuration: */
    cfg.rangebin_start   = (uint16_t) atoi (argv[2]);
    cfg.rangebin_stop    = (uint16_t) atoi (argv[3]);
    cfg.dopplerbin_start     = (uint16_t) atoi (argv[4]);
    cfg.dopplerbin_stop      = (uint16_t) atoi (argv[5]);
    cfg.det_thresh_featuregen   = (float) atof (argv[6]);

    /*copy detect feature threshold to MSS space */
    detect_feature_threshold = cfg.det_thresh_featuregen;
    rangebin_stop = cfg.rangebin_stop;
    posdopplerbin_stop = cfg.dopplerbin_stop;

    /* Save Configuration to use later */
    GestDemo_mssCfgUpdate((void *)&cfg, offsetof(GestDemo_CliCfg_t, gestFeatureCfg),
        sizeof(GestDemo_GestureFeatureCfg_t), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(GestDemo_message));

    message.type = GESTDEMO_MSS2DSS_GESTURE_FEATURE_CFG;
    memcpy((void *)&message.body.GestureFeatCfg, (void *)&cfg, sizeof(GestDemo_GestureFeatureCfg_t));

    if (GestDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

static int32_t GestDemo_CLIGestureTrainingParams (int32_t argc, char* argv[])
{
    /* TODO: Future feature may be part of this application,
        to receive the Gesture Training Parameters and forward it to DSS */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  @retval
 *      Not Applicable.
 */
void GestDemo_CLIInit (void)
{
    CLI_Cfg     cliCfg;
    char        demoBanner[256];
    uint16_t cnt = 0;
    
    /* Create Demo Banner to be printed out by CLI */
    sprintf(&demoBanner[0], 
                       "******************************************\n" \
                       "xWR16xx Gesture Demo %02d.%02d.%02d.%02d\n"  \
                       "******************************************\n", 
                        MMWAVE_SDK_VERSION_MAJOR,
                        MMWAVE_SDK_VERSION_MINOR,
                        MMWAVE_SDK_VERSION_BUGFIX,
                        MMWAVE_SDK_VERSION_BUILD
            );

    /* Initialize the CLI configuration: */
    memset ((void *)&cliCfg, 0, sizeof(CLI_Cfg));

    /* Populate the CLI configuration: */
    cliCfg.cliPrompt                    = "GestureDemo:/>";
    cliCfg.cliBanner                    = demoBanner;
#ifdef CAN_AS_COMM_MODE

#else
    cliCfg.cliUartHandle                = gGestMssMCB.commandUartHandle;
#endif
    cliCfg.taskPriority                 = 3;
    cliCfg.enableMMWaveExtension        = 0U; /* mmwave extension is not being used with this demo */
    cliCfg.usePolledMode                = true;

    cliCfg.tableEntry[cnt].cmd            = "version";
    cliCfg.tableEntry[cnt].helpString     = "get the device and SDK version";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveVersion;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "basicConfig";
    cliCfg.tableEntry[cnt].helpString     = "Does the basic default configurations";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = GestDemo_CLIBasicCfg;
    cnt++;
    
    cliCfg.tableEntry[cnt].cmd            = "sensorStart";
    cliCfg.tableEntry[cnt].helpString     = "Start the sensor; ensure that the configuration is completed";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = GestDemo_CLISensorStart;
    cnt++;
    
    cliCfg.tableEntry[cnt].cmd            = "sensorStop";
    cliCfg.tableEntry[cnt].helpString     = "Stop the sensor";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = GestDemo_CLISensorStop;
    cnt++;
    
    cliCfg.tableEntry[cnt].cmd            = "guiMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <detectedObjects> <logMagRange> <noiseProfile> <rangeAzimuthHeatMap> <rangeDopplerHeatMap> <statsInfo> <gestureInfo>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = GestDemo_CLIGuiMonSel;
    cnt++;
    
    cliCfg.tableEntry[cnt].cmd            = "gestureFeatureCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <rangebin_start> <rangebin_stop> <dopplerbin_start> <dopplerbin_stop> <det_thresh_featuregen>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = GestDemo_CLIGestureFeatureCfg;
    cnt++;
    
    cliCfg.tableEntry[cnt].cmd            = "newGestureParams";
    cliCfg.tableEntry[cnt].helpString     = "<lenghtOfParams> <training parameters>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = GestDemo_CLIGestureTrainingParams;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "CQRxSatMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<profile> <satMonSel> <priSliceDuration> <numSlices> <rxChanMask>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = GestDemo_CLIChirpQualityRxSatMonCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "CQSigImgMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<profile> <numSlices> <numSamplePerSlice>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = GestDemo_CLIChirpQualitySigImgMonCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "analogMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<rxSaturation> <sigImgBand>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = GestDemo_CLIAnalogMonitorCfg;
    cnt++;
    
    cliCfg.tableEntry[cnt].cmd            = "lvdsStreamCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enableHeader> <dataFmt> <enableSW>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = GestDemo_CLILvdsStreamCfg;
    cnt++;   
    
    /* Open the CLI: */
    if (CLI_open (&cliCfg) < 0)
    {
        System_printf ("Error: Unable to open the CLI\n");
        return;
    }
    System_printf ("Debug: CLI is operational\n");

    /* The link is not configured. */
    gGestMssMCB.isRadarSSConfigured = false;
    gGestMssMCB.isMMWaveLinkOpen = false;
    return;
}



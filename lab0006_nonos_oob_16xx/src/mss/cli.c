/*
 *   @file  cli.c
 *
 *   @brief
 *      Mmw (Milli-meter wave) DEMO CLI Implementation
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/uart/UART.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/cli/cli.h>

/* Demo Include Files */
#include "mss_mmw.h"
#include "common/mmw_messages.h"

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/* CLI Extended Command Functions */
static int32_t MmwDemo_CLICfarCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIPeakGroupingCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIMultiObjBeamForming (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLICalibDcRangeSig (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIExtendedMaxVelocity (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIClutterRemoval (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStart (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStop (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIGuiMonSel (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISetDataLogger (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIADCBufCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIBpmCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLINearFieldCorrection (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIChirpQualityRxSatMonCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIChirpQualitySigImgMonCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIAnalogMonitorCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLILvdsStreamCfg (int32_t argc, char* argv[]);

/* CLI Command Functions */
static int32_t CLI_MMWaveVersion (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveFlushCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveDataOutputMode (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveChannelCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveADCCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveProfileCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveChirpCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveFrameCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveAdvFrameCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveSubFrameCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveLowPowerCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveContModeCfg (int32_t argc, char* argv[]);


/**************************************************************************
 *************************** Extern Definitions *******************************
 **************************************************************************/


uint8_t                 cmdString[256];

/**
 * @brief
 *  CLI Master control block
 *
 * @details
 *  This is the MCB which tracks the CLI module
 */
typedef struct CLI_MCB_t
{
    /**
     * @brief   Configuration which was used to configure the CLI module
     */
    CLI_Cfg         cfg;

    /**
     * @brief   This is the number of CLI commands which have been added to the module
     */
    uint32_t        numCLICommands;

    /**
     * @brief   CLI Task Handle:
     */
    //Task_Handle     cliTaskHandle;
}MmwDemo_CliMCB;

/**
 * @brief   Global variable which tracks the CLI MCB
 */
MmwDemo_CliMCB     gCLI;

extern MmwDemo_MCB    gMmwMssMCB;
extern int32_t MmwDemo_mboxWrite(MmwDemo_message     * message);
extern int32_t CLI_MMWaveExtensionInit(CLI_Cfg* ptrCLICfg);

/**************************************************************************
 *************************** CLI  Function Definitions **************************
 **************************************************************************/

/**
 * @brief
 *  This is the mmWave extension table added to the CLI.
 */
CLI_CmdTableEntry gCLIMMWaveExtensionTable[] =
{
    {
        "version",
#ifdef CLI_MMWAVE_HELP_SUPPORT
        "No arguments",
#else
        NULL,
#endif
        CLI_MMWaveVersion
    },
    {
        "flushCfg",
#ifdef CLI_MMWAVE_HELP_SUPPORT
        "No arguments",
#else
        NULL,
#endif
        CLI_MMWaveFlushCfg
    },
    {
        "dfeDataOutputMode",
#ifdef CLI_MMWAVE_HELP_SUPPORT
        "<modeType>   1-Chirp and 2-Continuous",
#else
        NULL,
#endif
        CLI_MMWaveDataOutputMode
    },
    {
        "channelCfg",
#ifdef CLI_MMWAVE_HELP_SUPPORT
        "<rxChannelEn> <txChannelEn> <cascading>",
#else
        NULL,
#endif
        CLI_MMWaveChannelCfg
    },
    {
        "adcCfg",
#ifdef CLI_MMWAVE_HELP_SUPPORT
        "<numADCBits> <adcOutputFmt>",
#else
        NULL,
#endif
        CLI_MMWaveADCCfg
    },
    {
        "profileCfg",
#ifdef CLI_MMWAVE_HELP_SUPPORT
        "<profileId> <startFreq> <idleTime> <adcStartTime> <rampEndTime> <txOutPower> <txPhaseShifter> <freqSlopeConst> <txStartTime> <numAdcSamples> <digOutSampleRate> <hpfCornerFreq1> <hpfCornerFreq2> <rxGain>",
#else
        NULL,
#endif
        CLI_MMWaveProfileCfg
    },
    {
        "chirpCfg",
#ifdef CLI_MMWAVE_HELP_SUPPORT
        "<startIdx> <endIdx> <profileId> <startFreqVar> <freqSlopeVar> <idleTimeVar> <adcStartTimeVar> <txEnable>",
#else
        NULL,
#endif
        CLI_MMWaveChirpCfg
    },
    {
        "frameCfg",
#ifdef CLI_MMWAVE_HELP_SUPPORT
        "<chirpStartIdx> <chirpEndIdx> <numLoops> <numFrames> <framePeriodicity> <triggerSelect> <frameTriggerDelay>",
#else
        NULL,
#endif
        CLI_MMWaveFrameCfg
    },
    {
        "advFrameCfg",
#ifdef CLI_MMWAVE_HELP_SUPPORT
        "<numOfSubFrames> <forceProfile> <numFrames> <triggerSelect> <frameTrigDelay>",
#else
        NULL,
#endif
        CLI_MMWaveAdvFrameCfg
    },
    {
        "subFrameCfg",
#ifdef CLI_MMWAVE_HELP_SUPPORT
        "<subFrameNum> <forceProfileIdx> <chirpStartIdx> <numOfChirps> <numLoops> <burstPeriodicity> <chirpStartIdxOffset> <numOfBurst> <numOfBurstLoops> <subFramePeriodicity>",
#else
        NULL,
#endif
        CLI_MMWaveSubFrameCfg
    },
    {
        "lowPower",
#ifdef CLI_MMWAVE_HELP_SUPPORT
        "<reserved> <lpAdcMode>",
#else
        NULL,
#endif
        CLI_MMWaveLowPowerCfg
    },
    {
        "contModeCfg",
#ifdef CLI_MMWAVE_HELP_SUPPORT
        "<startFreq> <txOutPower> <txPhaseShifter> <digOutSampleRate> <hpfCornerFreq1> <hpfCornerFreq2> <rxGain> <reserved> <numSamples>",
#else
        NULL,
#endif
        CLI_MMWaveContModeCfg,
    }
#if 0
    ,
    {
        "bpmCfgAdvanced",
#ifdef CLI_MMWAVE_HELP_SUPPORT
        "<chirpStartIdx> <chirpEndIdx> <constBpmVal>",
#else
        NULL,
#endif
        CLI_MMWaveBPMCfgAdvanced
    },
    {
        NULL,
        NULL,
        NULL
    }
#endif
};

/**
 * @brief
 *  Global MMWave configuration tracked by the module.
 */
MMWave_CtrlCfg      gCLIMMWaveControlCfg;

/**
 * @brief
 *  Global MMWave open configuration tracked by the module.
 */
MMWave_OpenCfg      gCLIMMWaveOpenCfg;

/**************************************************************************
 ********************** CLI mmWave Extension Functions ********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the version command
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
    extern rlVersion_t gDevVersion;

    /* print the platform */
#ifdef SOC_XWR14XX
    CLI_write ("Platform          : xWR14xx\n");
#else
    CLI_write ("Platform          : xWR16xx\n");
#endif

    /* Display the version information on the CLI Console: */
    CLI_write ("mmWave SDK Version: %02d.%02d.%02d.%02d\n",
                            MMWAVE_SDK_VERSION_MAJOR,
                            MMWAVE_SDK_VERSION_MINOR,
                            MMWAVE_SDK_VERSION_BUGFIX,
                            MMWAVE_SDK_VERSION_BUILD);
    /* Display the version information */
    CLI_write ("RF H/W Version    : %02d.%02d\n",
               gDevVersion.rf.hwMajor, gDevVersion.rf.hwMinor);
    CLI_write ("RF F/W Version    : %02d.%02d.%02d.%02d.%02d.%02d.%02d\n",
               gDevVersion.rf.fwMajor, gDevVersion.rf.fwMinor, gDevVersion.rf.fwBuild, gDevVersion.rf.fwDebug,
               gDevVersion.rf.fwYear, gDevVersion.rf.fwMonth, gDevVersion.rf.fwDay);
    CLI_write ("mmWaveLink Version: %02d.%02d.%02d.%02d\n",
                gDevVersion.mmWaveLink.major, gDevVersion.mmWaveLink.minor,
                gDevVersion.mmWaveLink.build, gDevVersion.mmWaveLink.debug);

    /* Version string has been formatted successfully. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the flush configuration command
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
static int32_t CLI_MMWaveFlushCfg (int32_t argc, char* argv[])
{
    /* Reset the global configuration: */
    memset ((void*)&gCLIMMWaveControlCfg, 0, sizeof(MMWave_CtrlCfg));

    /* Reset the open configuration: */
    memset ((void*)&gCLIMMWaveOpenCfg, 0, sizeof(MMWave_OpenCfg));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the DFE Data Output mode.
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
static int32_t CLI_MMWaveDataOutputMode (int32_t argc, char* argv[])
{
    uint32_t cfgMode;

    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Get the configuration mode: */
    cfgMode = atoi (argv[1]);
    switch (cfgMode)
    {
        case 1U:
        {
            gCLIMMWaveControlCfg.dfeDataOutputMode = MMWave_DFEDataOutputMode_FRAME;
            break;
        }
        case 2U:
        {
            gCLIMMWaveControlCfg.dfeDataOutputMode = MMWave_DFEDataOutputMode_CONTINUOUS;
            break;
        }
        case 3U:
        {
            gCLIMMWaveControlCfg.dfeDataOutputMode = MMWave_DFEDataOutputMode_ADVANCED_FRAME;
            break;
        }
        default:
        {
            /* Error: Invalid argument. */
            CLI_write ("Error: Invalid mode.\n");
            return -1;
        }
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the channel configuration command
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
static int32_t CLI_MMWaveChannelCfg (int32_t argc, char* argv[])
{
    rlChanCfg_t     chCfg;

    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the channel configuration: */
    memset ((void *)&chCfg, 0, sizeof(rlChanCfg_t));

    /* Populate the channel configuration: */
    chCfg.rxChannelEn = atoi (argv[1]);
    chCfg.txChannelEn = atoi (argv[2]);
    chCfg.cascading   = atoi (argv[3]);

    /* Save Configuration to use later */
    memcpy((void *)&gCLIMMWaveOpenCfg.chCfg, (void *)&chCfg, sizeof(rlChanCfg_t));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the ADC configuration command
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
static int32_t CLI_MMWaveADCCfg (int32_t argc, char* argv[])
{
    rlAdcOutCfg_t   adcOutCfg;
    int32_t         retVal = 0;

    /* Sanity Check: Minimum argument check */
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&adcOutCfg, 0, sizeof(rlAdcOutCfg_t));

    /* Populate the ADC Output configuration: */
    adcOutCfg.fmt.b2AdcBits   = atoi (argv[1]);
    adcOutCfg.fmt.b2AdcOutFmt = atoi (argv[2]);

    /* Save Configuration to use later */
    memcpy((void *)&gCLIMMWaveOpenCfg.adcOutCfg, (void *)&adcOutCfg, sizeof(rlAdcOutCfg_t));
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the profile configuration command
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
static int32_t CLI_MMWaveProfileCfg (int32_t argc, char* argv[])
{
    rlProfileCfg_t          profileCfg;
    MmwDemo_message         message;

    /* Sanity Check: Minimum argument check */
    if (argc != 15)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Sanity Check: Profile configuration is valid only for the Frame or 
                     Advanced Frame Mode: */
    if ((gCLIMMWaveControlCfg.dfeDataOutputMode != MMWave_DFEDataOutputMode_FRAME) &&
        (gCLIMMWaveControlCfg.dfeDataOutputMode != MMWave_DFEDataOutputMode_ADVANCED_FRAME))
    {
        CLI_write ("Error: Configuration is valid only if the DFE Output Mode is Frame or Advanced Frame\n");
        return -1;
    }
    
    /* Initialize the profile configuration: */
    memset ((void *)&profileCfg, 0, sizeof(rlProfileCfg_t));

    /* Populate the profile configuration: */
    profileCfg.profileId             = atoi (argv[1]);
    profileCfg.startFreqConst        = (uint32_t) (atof(argv[2]) * (1U << 26) / 3.6);
    profileCfg.idleTimeConst         = (uint32_t)((float)atof (argv[3]) * 1000 / 10);
    profileCfg.adcStartTimeConst     = (uint32_t)((float)atof (argv[4]) * 1000 / 10);
    profileCfg.rampEndTime           = (uint32_t)((float)atof (argv[5]) * 1000 / 10);
    profileCfg.txOutPowerBackoffCode = atoi (argv[6] );
    profileCfg.txPhaseShifter        = atoi (argv[7]);
    profileCfg.freqSlopeConst        = (int16_t)(atof (argv[8]) * (1U << 26) / (3.6*1e3*900)); //2^26 * 1e6/((3.6*1e9)*900)
    profileCfg.txStartTime           = (int32_t)((float)atof (argv[9]) * 1000 / 10);
    profileCfg.numAdcSamples         = atoi (argv[10]);
    profileCfg.digOutSampleRate      = atoi (argv[11]);
    profileCfg.hpfCornerFreq1        = atoi (argv[12]);
    profileCfg.hpfCornerFreq2        = atoi (argv[13]);
    profileCfg.rxGain                = atoi (argv[14]);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_PROFILE_CFG;
    memcpy((void *)&message.body.profileCfg, (void *)&profileCfg, sizeof(rlProfileCfg_t));

    /* Send this chirp config to DSS */
    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the chirp configuration command
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
static int32_t CLI_MMWaveChirpCfg (int32_t argc, char* argv[])
{
    rlChirpCfg_t            chirpCfg;
    MmwDemo_message         message;

    /* Sanity Check: Minimum argument check */
    if (argc != 9)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Sanity Check: Chirp configuration is valid only for the Frame or
                     Advanced Frame Mode: */
    if ((gCLIMMWaveControlCfg.dfeDataOutputMode != MMWave_DFEDataOutputMode_FRAME) &&
        (gCLIMMWaveControlCfg.dfeDataOutputMode != MMWave_DFEDataOutputMode_ADVANCED_FRAME))
    {
        CLI_write ("Error: Configuration is valid only if the DFE Output Mode is Chirp\n");
        return -1;
    }

    /* Initialize the chirp configuration: */
    memset ((void *)&chirpCfg, 0, sizeof(rlChirpCfg_t));

    /* Populate the chirp configuration: */
    chirpCfg.chirpStartIdx   = atoi (argv[1]);
    chirpCfg.chirpEndIdx     = atoi (argv[2]);
    chirpCfg.profileId       = atoi (argv[3]);
    chirpCfg.startFreqVar    = (uint32_t) ((float)atof(argv[4]) * (1U << 26) / (3.6 * 1e3)); /* in MHz */
    chirpCfg.freqSlopeVar    = (uint16_t) ((float)atof(argv[5]) * (1U << 26) / (3.6*1e6*900)); /* in KHz/usec */
    chirpCfg.idleTimeVar     = (uint32_t)((float)atof (argv[6]) * 1000 / 10);
    chirpCfg.adcStartTimeVar = (uint32_t)((float)atof (argv[7]) * 1000 / 10);
    chirpCfg.txEnable        = atoi (argv[8]);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_CHIRP_CFG;
    memcpy((void *)&message.body.chirpCfg, (void *)&chirpCfg, sizeof(rlChirpCfg_t));

    /* Send this chirp config to DSS */
    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the frame configuration command
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
static int32_t CLI_MMWaveFrameCfg (int32_t argc, char* argv[])
{
    rlFrameCfg_t    frameCfg;

    /* Sanity Check: Minimum argument check */
    if (argc != 8)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Sanity Check: Frame configuration is valid only for the Frame or
                     Advanced Frame Mode: */
    if (gCLIMMWaveControlCfg.dfeDataOutputMode != MMWave_DFEDataOutputMode_FRAME)
    {
        CLI_write ("Error: Configuration is valid only if the DFE Output Mode is Chirp\n");
        return -1;
    }

    /* Initialize the frame configuration: */
    memset ((void *)&frameCfg, 0, sizeof(rlFrameCfg_t));

    /* Populate the frame configuration: */
    frameCfg.chirpStartIdx      = atoi (argv[1]);
    frameCfg.chirpEndIdx        = atoi (argv[2]);
    frameCfg.numLoops           = atoi (argv[3]);
    frameCfg.numFrames          = atoi (argv[4]);
    frameCfg.framePeriodicity   = (uint32_t)((float)atof(argv[5]) * 1000000 / 5);
    frameCfg.triggerSelect      = atoi (argv[6]);
    frameCfg.frameTriggerDelay  = (uint32_t)((float)atof(argv[7]) * 1000000 / 5);

    /* Save Configuration to use later */
    memcpy((void *)&gCLIMMWaveControlCfg.u.frameCfg.frameCfg, (void *)&frameCfg, sizeof(rlFrameCfg_t));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the advanced frame configuration command
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
static int32_t CLI_MMWaveAdvFrameCfg (int32_t argc, char* argv[])
{
    rlAdvFrameCfg_t  advFrameCfg;

    /* Sanity Check: Minimum argument check */
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    
    /* Sanity Check: Frame configuration is valid only for the Frame or
                     Advanced Frame Mode: */
    if (gCLIMMWaveControlCfg.dfeDataOutputMode != MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        CLI_write ("Error: Configuration is valid only if the DFE Output Mode is Advanced Frame\n");
        return -1;
    }

    /* Initialize the frame configuration: */
    memset ((void *)&advFrameCfg, 0, sizeof(rlAdvFrameCfg_t));

    /* Populate the frame configuration: */
    advFrameCfg.frameSeq.numOfSubFrames      = atoi (argv[1]);
    advFrameCfg.frameSeq.forceProfile        = atoi (argv[2]);
    advFrameCfg.frameSeq.numFrames           = atoi (argv[3]);
    advFrameCfg.frameSeq.triggerSelect       = atoi (argv[4]);
    advFrameCfg.frameSeq.frameTrigDelay      = (uint32_t)((float)atof(argv[5]) * 1000000 / 5);

    /* Save Configuration to use later */
    memcpy ((void *)&gCLIMMWaveControlCfg.u.advancedFrameCfg.frameCfg,
            (void *)&advFrameCfg, sizeof(rlAdvFrameCfg_t));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the subframe configuration command.
 *      Only valid when used in conjunction with the advanced frame configuration.
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
static int32_t CLI_MMWaveSubFrameCfg (int32_t argc, char* argv[])
{
    rlSubFrameCfg_t  subFrameCfg;
    uint8_t          subFrameNum;

    /* Sanity Check: Minimum argument check */
    if (argc != 11)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    
    /* Sanity Check: Sub Frame configuration is valid only for the Advanced Frame Mode: */
    if (gCLIMMWaveControlCfg.dfeDataOutputMode != MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        CLI_write ("Error: Configuration is valid only if the DFE Output Mode is Advanced Frame\n");
        return -1;
    }

    /* Initialize the frame configuration: */
    memset ((void *)&subFrameCfg, 0, sizeof(rlSubFrameCfg_t));

    /* Populate the frame configuration: */
    subFrameNum                                  = (uint8_t)atoi (argv[1]);
    if (subFrameNum > gCLIMMWaveControlCfg.u.advancedFrameCfg.frameCfg.frameSeq.numOfSubFrames)
    {
        CLI_write ("Error: Invalid subframe number.\n");
        return -1;
    }
    subFrameCfg.forceProfileIdx     = atoi (argv[2]);
    subFrameCfg.chirpStartIdx       = atoi (argv[3]);
    subFrameCfg.numOfChirps         = atoi (argv[4]);
    subFrameCfg.numLoops            = atoi (argv[5]);
    subFrameCfg.burstPeriodicity    = (uint32_t)((float)atof(argv[6]) * 1000000 / 5);
    subFrameCfg.chirpStartIdxOffset = atoi (argv[7]);
    subFrameCfg.numOfBurst          = atoi (argv[8]);
    subFrameCfg.numOfBurstLoops     = atoi (argv[9]);
    subFrameCfg.subFramePeriodicity = (uint32_t)((float)atof(argv[10]) * 1000000 / 5);
    
    /* Save Configuration to use later */
    memcpy((void *)&gCLIMMWaveControlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameNum],
        (void *)&subFrameCfg, sizeof(rlSubFrameCfg_t));
    return 0;
    
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the low power command
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
static int32_t CLI_MMWaveLowPowerCfg (int32_t argc, char* argv[])
{
    rlLowPowerModeCfg_t     lowPowerCfg;

    /* Sanity Check: Minimum argument check */
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the channel configuration: */
    memset ((void *)&lowPowerCfg, 0, sizeof(rlLowPowerModeCfg_t));

    /* Populate the channel configuration: */
    lowPowerCfg.lpAdcMode     = atoi (argv[2]);

    /* Save Configuration to use later */
    memcpy((void *)&gCLIMMWaveOpenCfg.lowPowerMode, (void *)&lowPowerCfg, sizeof(rlLowPowerModeCfg_t));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the continuous mode
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
static int32_t CLI_MMWaveContModeCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 10)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Sanity Check: Continuous configuration is valid only for the Continuous Mode: */
    if (gCLIMMWaveControlCfg.dfeDataOutputMode != MMWave_DFEDataOutputMode_CONTINUOUS)
    {
        CLI_write ("Error: Configuration is valid only if the DFE Output Mode is Continuous\n");
        return -1;
    }

    /* Populate the configuration: */
    gCLIMMWaveControlCfg.u.continuousModeCfg.cfg.startFreqConst        = (uint32_t) (atof(argv[1]) * (1U << 26) / 3.6);
    gCLIMMWaveControlCfg.u.continuousModeCfg.cfg.txOutPowerBackoffCode = (uint32_t) atoi (argv[2]);
    gCLIMMWaveControlCfg.u.continuousModeCfg.cfg.txPhaseShifter        = (uint32_t) atoi (argv[3]);
    gCLIMMWaveControlCfg.u.continuousModeCfg.cfg.digOutSampleRate      = (uint16_t) atoi (argv[4]);
    gCLIMMWaveControlCfg.u.continuousModeCfg.cfg.hpfCornerFreq1        = (uint8_t)  atoi (argv[5]);
    gCLIMMWaveControlCfg.u.continuousModeCfg.cfg.hpfCornerFreq2        = (uint8_t)  atoi (argv[6]);
    gCLIMMWaveControlCfg.u.continuousModeCfg.cfg.rxGain                = (uint16_t) atoi (argv[7]);
    /*argv[8] is reserved*/
    gCLIMMWaveControlCfg.u.continuousModeCfg.dataTransSize             = (uint16_t) atoi (argv[9]);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave extension initialization API
 *
 *  \ingroup CLI_UTIL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t CLI_MMWaveExtensionInit(CLI_Cfg* ptrCLICfg)
{
    /* Initialize the mmWave control configuration: */
    memset ((void *)&gCLIMMWaveControlCfg, 0, sizeof(MMWave_CtrlCfg));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave extension handler which executes mmWave extension
 *      commands. This is invoked by the main CLI wrapper only if the extension
 *      was enabled.
 *
 *  @param[in]  argc
 *      Number of detected arguments
 *  @param[in] argv
 *      Detected arguments
 *
 *  \ingroup CLI_UTIL_INTERNAL_FUNCTION
 *
 *  @retval
 *      0   -   Matching mmWave extension command found
 *  @retval
 *      -1  -   No Matching mmWave extension command
 */
int32_t CLI_MMWaveExtensionHandler(int32_t argc, char* argv[])
{
    CLI_CmdTableEntry*  ptrCLICommandEntry;
    int32_t             cliStatus;
    int32_t             retVal = 0;

    /* Get the pointer to the mmWave extension table */
    ptrCLICommandEntry = &gCLIMMWaveExtensionTable[0];

    /* Cycle through all the registered externsion CLI commands: */
    while (ptrCLICommandEntry->cmdHandlerFxn != NULL)
    {
        /* Do we have a match? */
        if (strcmp(ptrCLICommandEntry->cmd, argv[0]) == 0)
        {
            /* YES: Pass this to the CLI registered function */
            cliStatus = ptrCLICommandEntry->cmdHandlerFxn (argc, argv);
            if (cliStatus == 0)
            {
                /* Successfully executed the CLI command: */
                CLI_write ("Done\n");
            }
            else
            {
                /* Error: The CLI command failed to execute */
                CLI_write ("Error %d\n", cliStatus);
            }
            break;
        }

        /* Get the next entry: */
        ptrCLICommandEntry++;
    }

    /* Was this a valid CLI command? */
    if (ptrCLICommandEntry->cmdHandlerFxn == NULL)
    {
        /* NO: The command was not a valid CLI mmWave extension command. Setup
         * the return value correctly. */
        retVal = -1;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave extension handler which is invoked by the
 *      CLI Help command handler only if the extension was enabled.
 *
 *  \ingroup CLI_UTIL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void CLI_MMWaveExtensionHelp(void)
{
    CLI_CmdTableEntry*  ptrCLICommandEntry;

    /* Get the pointer to the mmWave extension table */
    ptrCLICommandEntry = &gCLIMMWaveExtensionTable[0];

    /* Display the banner: */
    CLI_write ("****************************************************\n");
    CLI_write ("mmWave Extension Help\n");
    CLI_write ("****************************************************\n");

    /* Cycle through all the registered externsion CLI commands: */
    while (ptrCLICommandEntry->cmdHandlerFxn != NULL)
    {
        /* Display the help string*/
        CLI_write ("%s: %s\n",
                    ptrCLICommandEntry->cmd,
                   (ptrCLICommandEntry->helpString == NULL) ?
                    "No help available" :
                    ptrCLICommandEntry->helpString);

        /* Get the next entry: */
        ptrCLICommandEntry++;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      This is an API provided by the CLI mmWave extension handler to get
 *      the mmWave control configuration.
 *
 *  @param[out]  ptrCtrlCfg
 *      Pointer to the control configuration populated by the API
 *
 *  \ingroup CLI_UTIL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void CLI_getMMWaveExtensionConfig(MMWave_CtrlCfg* ptrCtrlCfg)
{
    memcpy ((void*)ptrCtrlCfg, (void*)&gCLIMMWaveControlCfg, sizeof(MMWave_CtrlCfg));
    return;
}

/**
 *  @b Description
 *  @n
 *      This is an API provided by the CLI mmWave extension handler to get
 *      the mmWave control configuration.
 *
 *  @param[out]  ptrOpenCfg
 *      Pointer to the open configuration populated by the API
 *
 *  \ingroup CLI_UTIL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void CLI_getMMWaveExtensionOpenConfig(MMWave_OpenCfg* ptrOpenCfg)
{
    memcpy ((void*)ptrOpenCfg, (void*)&gCLIMMWaveOpenCfg, sizeof(MMWave_OpenCfg));
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the sensor start command
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
static int32_t MmwDemo_CLISensorStart (int32_t argc, char* argv[])
{
    bool doReconfig = true;
    int32_t retVal = 0x0;
    
    if (argc==2)
    {
        doReconfig = (bool) atoi (argv[1]);
    }
    /* Post sensorSTart event to notify configuration is done */
    MmwDemo_notifySensorStart(doReconfig);
#if 0
    while(retVal > 0)
    {
        /* check for sensor-start completion */
        retVal = MmwDemo_waitSensorStartComplete();
        if(retVal != 0)
        {
            /* call Non OS loop once to check DSS mailbox for sensor-start status event */
            MmwDemo_nonOsLoop(1);
        }
    }
#endif
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the sensor stop command
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
static int32_t MmwDemo_CLISensorStop (int32_t argc, char* argv[])
{
    int32_t retVal = 0xF;
    
    /* Post sensorSTOP event to notify sensor stop command */
    MmwDemo_notifySensorStop();
    
    while(retVal > 0)
    {
        /* check for sensor-stop completion */
        retVal = MmwDemo_waitSensorStopComplete();
        if(retVal != 0)
        {
            /* call Non OS loop once to check DSS mailbox for sensor-start status event */
            MmwDemo_nonOsLoop(1);
        }
    }

    return retVal;
}

static int32_t MmwDemo_CLIGetSubframe (int32_t argc, char* argv[], int32_t expectedArgc, int8_t* subFrameNum)
{
    int8_t subframe;
    
    /* Sanity Check: Minimum argument check */
    if (argc != expectedArgc)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /*Subframe info is always in position 1*/
    subframe = (int8_t) atoi(argv[1]);

    if(subframe >= (int8_t)RL_MAX_SUBFRAMES)
    {
        CLI_write ("Error: Subframe number is invalid\n");
        return -1;
    }
    
    *subFrameNum = (int8_t)subframe;

    return 0;
}

static void MmwDemo_mssCfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum)
{    
    /* if subFrameNum undefined, broadcast to all sub-frames */
    if(subFrameNum == MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG)
    {
        uint8_t  indx;
        for(indx = 0; indx < RL_MAX_SUBFRAMES; indx++)
        {
            memcpy((void *)((uint32_t) &gMmwMssMCB.cliCfg[indx] + offset), srcPtr, size);
        }
        
    }
    else
    {
        /* Apply configuration to specific subframe (or to position zero for the legacy case
           where there is no advanced frame config) */
        memcpy((void *)((uint32_t) &gMmwMssMCB.cliCfg[subFrameNum] + offset), srcPtr, size);
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
static int32_t MmwDemo_CLIGuiMonSel (int32_t argc, char* argv[])
{
    MmwDemo_GuiMonSel   guiMonSel;
    MmwDemo_message     message;
    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 8, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize the guiMonSel configuration: */
    memset ((void *)&guiMonSel, 0, sizeof(MmwDemo_GuiMonSel));

    /* Populate configuration: */
    guiMonSel.detectedObjects           = atoi (argv[2]);
    guiMonSel.logMagRange               = atoi (argv[3]);
    guiMonSel.noiseProfile              = atoi (argv[4]);
    guiMonSel.rangeAzimuthHeatMap       = atoi (argv[5]);
    guiMonSel.rangeDopplerHeatMap       = atoi (argv[6]);
    guiMonSel.statsInfo                 = atoi (argv[7]);

    MmwDemo_mssCfgUpdate((void *)&guiMonSel, offsetof(MmwDemo_CliCfg_t, guiMonSel), 
        sizeof(MmwDemo_GuiMonSel), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_GUIMON_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.guiMonSel, (void *)&guiMonSel, sizeof(MmwDemo_GuiMonSel));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for CFAR configuration
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
static int32_t MmwDemo_CLICfarCfg (int32_t argc, char* argv[])
{
    MmwDemo_CfarCfg     cfarCfg;
    MmwDemo_message     message;
    uint32_t            procDirection;
    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 9, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfarCfg, 0, sizeof(MmwDemo_CfarCfg));

    /* Populate configuration: */
    procDirection             = (uint32_t) atoi (argv[2]);
    cfarCfg.averageMode       = (uint8_t) atoi (argv[3]);
    cfarCfg.winLen            = (uint8_t) atoi (argv[4]);
    cfarCfg.guardLen          = (uint8_t) atoi (argv[5]);
    cfarCfg.noiseDivShift     = (uint8_t) atoi (argv[6]);
    cfarCfg.cyclicMode        = (uint8_t) atoi (argv[7]);
    cfarCfg.thresholdScale    = (uint16_t) atoi (argv[8]);

    /* Save Configuration to use later */     
    if (procDirection == 0)
    {
        MmwDemo_mssCfgUpdate((void *)&cfarCfg, offsetof(MmwDemo_CliCfg_t, cfarCfgRange),
            sizeof(MmwDemo_CfarCfg), subFrameNum);    
    }
    else
    {
        MmwDemo_mssCfgUpdate((void *)&cfarCfg, offsetof(MmwDemo_CliCfg_t, cfarCfgDoppler),
            sizeof(MmwDemo_CfarCfg), subFrameNum);    
    }

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    if (procDirection == 0)
    {
        message.type = MMWDEMO_MSS2DSS_CFAR_RANGE_CFG;
    }
    else if (procDirection == 1)
    {
        message.type = MMWDEMO_MSS2DSS_CFAR_DOPPLER_CFG;
    }
    else
    {
        return -1;
    }

    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.cfarCfg, (void *)&cfarCfg, sizeof(MmwDemo_CfarCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;    
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for Peak grouping configuration
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
static int32_t MmwDemo_CLIPeakGroupingCfg (int32_t argc, char* argv[])
{
    MmwDemo_PeakGroupingCfg peakGroupingCfg;
    MmwDemo_message         message;
    int8_t                  subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 7, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&peakGroupingCfg, 0, sizeof(MmwDemo_PeakGroupingCfg));

    /* Populate configuration: */
    peakGroupingCfg.scheme               = (uint8_t) atoi (argv[2]);
    peakGroupingCfg.inRangeDirectionEn   = (uint8_t) atoi (argv[3]);
    peakGroupingCfg.inDopplerDirectionEn = (uint8_t) atoi (argv[4]);
    peakGroupingCfg.minRangeIndex    = (uint16_t) atoi (argv[5]);
    peakGroupingCfg.maxRangeIndex    = (uint16_t) atoi (argv[6]);

    if (peakGroupingCfg.scheme != 1 && peakGroupingCfg.scheme != 2)
    {
        CLI_write ("Error: Invalid peak grouping scheme\n");
        return -1;
    }

    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&peakGroupingCfg, offsetof(MmwDemo_CliCfg_t, peakGroupingCfg),
        sizeof(MmwDemo_PeakGroupingCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_PEAK_GROUPING_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.peakGroupingCfg, (void *)&peakGroupingCfg, sizeof(MmwDemo_PeakGroupingCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for multi object beam forming configuration
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
static int32_t MmwDemo_CLIMultiObjBeamForming (int32_t argc, char* argv[])
{
    MmwDemo_MultiObjBeamFormingCfg cfg;
    MmwDemo_message     message;
    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 4, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_MultiObjBeamFormingCfg));

    /* Populate configuration: */
    cfg.enabled                     = (uint8_t) atoi (argv[2]);
    cfg.multiPeakThrsScal           = (float) atof (argv[3]);

    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, multiObjBeamFormingCfg),
        sizeof(MmwDemo_MultiObjBeamFormingCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_MULTI_OBJ_BEAM_FORM;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.multiObjBeamFormingCfg, (void *)&cfg, sizeof(MmwDemo_MultiObjBeamFormingCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
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
static int32_t MmwDemo_CLICalibDcRangeSig (int32_t argc, char* argv[])
{
    MmwDemo_CalibDcRangeSigCfg cfg;
    MmwDemo_message            message;
    uint32_t                   log2NumAvgChirps;
    int8_t                     subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 6, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_CalibDcRangeSigCfg));

    /* Populate configuration: */
    cfg.enabled          = (uint16_t) atoi (argv[2]);
    cfg.negativeBinIdx   = (int16_t)  atoi (argv[3]);
    cfg.positiveBinIdx   = (int16_t)  atoi (argv[4]);
    cfg.numAvgChirps     = (uint16_t)  atoi (argv[5]);

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
    MmwDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, calibDcRangeSigCfg),
        sizeof(MmwDemo_CalibDcRangeSigCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_CALIB_DC_RANGE_SIG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.calibDcRangeSigCfg, (void *)&cfg, sizeof(MmwDemo_CalibDcRangeSigCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for Velocity Disambiguation Configuration
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
static int32_t MmwDemo_CLIExtendedMaxVelocity (int32_t argc, char* argv[])
{
    MmwDemo_ExtendedMaxVelocityCfg cfg;
    MmwDemo_message                message;
    int8_t                         subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 3, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_ExtendedMaxVelocityCfg));

    /* Populate configuration: */
    cfg.enabled          = (uint16_t) atoi (argv[2]);


    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, extendedMaxVelocityCfg),
        sizeof(MmwDemo_ExtendedMaxVelocityCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_EXTENDED_MAX_VELOCITY;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.extendedMaxVelocityCfg, (void *)&cfg, sizeof(MmwDemo_ExtendedMaxVelocityCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
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
static int32_t MmwDemo_CLIClutterRemoval (int32_t argc, char* argv[])
{
    MmwDemo_ClutterRemovalCfg cfg;
    MmwDemo_message     message;
    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 3, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for clutter removal */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_ClutterRemovalCfg));

    /* Populate configuration: */
    cfg.enabled          = (uint16_t) atoi (argv[2]);


    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, clutterRemovalCfg),
        sizeof(MmwDemo_ClutterRemovalCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_CLUTTER_REMOVAL;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.clutterRemovalCfg, (void *)&cfg, sizeof(MmwDemo_ClutterRemovalCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
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
static int32_t MmwDemo_CLISetDataLogger (int32_t argc, char* argv[])
{
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }


    /* Save Configuration to use later */
    if (strcmp(argv[1], "mssLogger") == 0)  
        gMmwMssMCB.cfg.dataLogger = 0;
    else if (strcmp(argv[1], "dssLogger") == 0)  
        gMmwMssMCB.cfg.dataLogger = 1;
    else
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
       
    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_SET_DATALOGGER;
    message.body.dataLogger = gMmwMssMCB.cfg.dataLogger;

    if (MmwDemo_mboxWrite(&message) == 0)
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
static int32_t MmwDemo_CLIADCBufCfg (int32_t argc, char* argv[])
{
    MmwDemo_ADCBufCfg   adcBufCfg;
    MmwDemo_message     message;
    int8_t              subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 6, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&adcBufCfg, 0, sizeof(MmwDemo_ADCBufCfg));

    /* Populate configuration: */
    adcBufCfg.adcFmt          = (uint8_t) atoi (argv[2]);
    adcBufCfg.iqSwapSel       = (uint8_t) atoi (argv[3]);
    adcBufCfg.chInterleave    = (uint8_t) atoi (argv[4]);
    adcBufCfg.chirpThreshold  = (uint8_t) atoi (argv[5]);

    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&adcBufCfg, offsetof(MmwDemo_CliCfg_t, adcBufCfg),
        sizeof(MmwDemo_ADCBufCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_MSS2DSS_ADCBUFCFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.adcBufCfg, (void *)&adcBufCfg, sizeof(MmwDemo_ADCBufCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
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
static int32_t MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    MmwDemo_compRxChannelBiasCfg_t   cfg;
    MmwDemo_message     message;
    int32_t Re, Im;
    int32_t argInd;
    int32_t i;

    /* Sanity Check: Minimum argument check */
    if (argc != (1+1+SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_compRxChannelBiasCfg_t));

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
    /* Save Configuration to use later */
    memcpy((void *) &gMmwMssMCB.cliCommonCfg.compRxChanCfg, &cfg, sizeof(MmwDemo_compRxChannelBiasCfg_t));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_MSS2DSS_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE;
    memcpy((void *)&message.body.compRxChanCfg, (void *)&cfg, sizeof(MmwDemo_compRxChannelBiasCfg_t));

    if (MmwDemo_mboxWrite(&message) == 0)
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
static int32_t MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    MmwDemo_measureRxChannelBiasCfg_t   cfg;
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_measureRxChannelBiasCfg_t));

    /* Populate configuration: */
    cfg.enabled          = (uint8_t) atoi (argv[1]);
    cfg.targetDistance   = (float) atof (argv[2]);
    cfg.searchWinSize   = (float) atof (argv[3]);

    /* Save Configuration to use later */
    memcpy((void *) &gMmwMssMCB.cliCommonCfg.measureRxChanCfg, &cfg, sizeof(MmwDemo_measureRxChannelBiasCfg_t));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_MSS2DSS_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE;
    memcpy((void *)&message.body.measureRxChanCfg, (void *)&cfg, sizeof(MmwDemo_measureRxChannelBiasCfg_t));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for BPM configuration supported by the mmw Demo
 *      Note that there is a generic BPM configuration command supported by
 *      utils/cli and mmwave. The generic BPM command is not supported by the
 *      demo as the mmw demo assumes a specific BPM pattern for the TX antennas.
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
static int32_t MmwDemo_CLIBpmCfg (int32_t argc, char* argv[])
{

    int8_t           subFrameNum;
    MmwDemo_BpmCfg   cfg;
    MmwDemo_message  message;

    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    if(MmwDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_BpmCfg));

    /* Populate configuration: */
    cfg.isEnabled = (bool) atoi(argv[2]) ;
    cfg.chirp0Idx = (uint16_t) atoi(argv[3]) ;
    cfg.chirp1Idx = (uint16_t) atoi(argv[4]) ;

    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, bpmCfg),
        sizeof(MmwDemo_BpmCfg), subFrameNum);

    message.type = MMWDEMO_MSS2DSS_BPM_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.bpmCfg, (void *)&cfg, sizeof(MmwDemo_BpmCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
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
static int32_t MmwDemo_CLINearFieldCorrection (int32_t argc, char* argv[])
{
    MmwDemo_NearFieldCorrectionCfg cfg;
    MmwDemo_message                message;
    int8_t                         subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for Near Field Correction */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_NearFieldCorrectionCfg));

    /* Populate configuration: */
    cfg.enabled       = (uint8_t) atoi(argv[2]);
    cfg.startRangeIdx = (uint16_t) atoi(argv[3]);
    cfg.endRangeIdx   = (uint16_t) atoi(argv[4]);


    /* Save Configuration to use later */
    MmwDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, nearFieldCorrectionCfg),
        sizeof(MmwDemo_NearFieldCorrectionCfg), subFrameNum);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_NEAR_FIELD_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.nearFieldCorrectionCfg, (void *)&cfg,
           sizeof(MmwDemo_NearFieldCorrectionCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
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
static int32_t MmwDemo_CLIChirpQualityRxSatMonCfg (int32_t argc, char* argv[])
{
    /* In Non-OS environment CQ feature is disabled but added this function to avoid any error at visualizer.
     * As this CLI command is mandatory to have in *.cfg file to proceed at Visualizer end.
     * If User needs this feature then it is requested to implement it on DSS end (take reference from SDK)
     */
#if 0
    rlRxSatMonConf_t        cqSatMonCfg;
    MmwDemo_message         message;

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
        memcpy((void *) &gMmwMssMCB.cliCommonCfg.cqSatMonCfg[cqSatMonCfg.profileIndx],
                       &cqSatMonCfg,
                       sizeof(rlRxSatMonConf_t));

        /* Send configuration to DSS */
        memset((void *)&message, 0, sizeof(MmwDemo_message));
        message.type = MMWDEMO_MSS2DSS_CQ_SATURATION_MONITOR;
        memcpy((void *)&message.body.cqSatMonCfg, (void *)&cqSatMonCfg, sizeof(rlRxSatMonConf_t));

        if (MmwDemo_mboxWrite(&message) == 0)
            return 0;
        else
            return -1;
    }
    else
    {
        return -1;
    }
#else
    return 0;
#endif
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
static int32_t MmwDemo_CLIChirpQualitySigImgMonCfg (int32_t argc, char* argv[])
{
    /* In Non-OS environment CQ feature is disabled but added this function to avoid any error at visualizer.
     * As this CLI command is mandatory to have in *.cfg file to proceed at Visualizer end.
     * If User needs this feature then it is requested to implement it on DSS end (take reference from SDK)
     */
#if 0
    rlSigImgMonConf_t       cqSigImgMonCfg;
    MmwDemo_message         message;

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
        memcpy((void *) &gMmwMssMCB.cliCommonCfg.cqSigImgMonCfg[cqSigImgMonCfg.profileIndx],
                &cqSigImgMonCfg,
                sizeof(rlSigImgMonConf_t));

        /* Send configuration to DSS */
        memset((void *)&message, 0, sizeof(MmwDemo_message));
        message.type = MMWDEMO_MSS2DSS_CQ_SIGIMG_MONITOR;
        memcpy((void *)&message.body.cqSigImgMonCfg, (void *)&cqSigImgMonCfg, sizeof(rlSigImgMonConf_t));

        if (MmwDemo_mboxWrite(&message) == 0)
            return 0;
        else
            return -1;
    }
    else
    {
        return -1;
    }
#else
    return 0;
#endif
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
static int32_t MmwDemo_CLIAnalogMonitorCfg (int32_t argc, char* argv[])
{
    /* In Non-OS environment Analog Monitoring is disabled but added this function to avoid any error at visualizer.
     * As this CLI command is mandatory to have in *.cfg file to proceed at Visualizer end.
     * If User needs this feature then it is requested to implement it on DSS end (take reference from SDK)
     */
#if 0
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Save Configuration to use later */
    gMmwMssMCB.cliCommonCfg.anaMonCfg.rxSatMonEn = atoi (argv[1]);
    gMmwMssMCB.cliCommonCfg.anaMonCfg.sigImgMonEn = atoi (argv[2]);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_MSS2DSS_ANALOG_MONITOR;
    memcpy((void *)&message.body.anaMonCfg,
            (void *)&gMmwMssMCB.cliCommonCfg.anaMonCfg,
            sizeof(MmwDemo_AnaMonitorCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
#else
    return 0;
#endif
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
static int32_t MmwDemo_CLILvdsStreamCfg (int32_t argc, char* argv[])
{
    /* In Non-OS environment LVDS feature is disabled but added this function to avoid any error at visualizer.
     * As this CLI command is mandatory to have in *.cfg file to proceed at Visualizer end.
     * If User needs this feature then it is requested to implement it on DSS end (take reference from SDK)
     */
#if 0
    int8_t                  subFrameNum;
    MmwDemo_LvdsStreamCfg   cfg;
    MmwDemo_message         message;

    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    if(MmwDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
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
    MmwDemo_mssCfgUpdate((void *)&cfg, offsetof(MmwDemo_CliCfg_t, lvdsStreamCfg),
        sizeof(MmwDemo_LvdsStreamCfg), subFrameNum);

    message.type = MMWDEMO_MSS2DSS_LVDSSTREAM_CFG;
    message.subFrameNum = subFrameNum;
    memcpy((void *)&message.body.lvdsStreamCfg, (void *)&cfg, sizeof(MmwDemo_LvdsStreamCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
#else
    return 0;
#endif
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_CLIInit (void)
{
    CLI_Cfg     cliCfg;
    char        demoBanner[256];
    uint32_t    cnt;

    /* Create Demo Banner to be printed out by CLI */
    sprintf(&demoBanner[0], 
                       "******************************************\n" \
                       "xWR16xx MMW Demo %02d.%02d.%02d.%02d\n"  \
                       "******************************************\n", 
                        MMWAVE_SDK_VERSION_MAJOR,
                        MMWAVE_SDK_VERSION_MINOR,
                        MMWAVE_SDK_VERSION_BUGFIX,
                        MMWAVE_SDK_VERSION_BUILD
            );

    /* Initialize the CLI configuration: */
    memset ((void *)&cliCfg, 0, sizeof(CLI_Cfg));

    /* Populate the CLI configuration: */
    cliCfg.cliPrompt                    = "mmwDemo:/>";
    cliCfg.cliBanner                    = demoBanner;
    cliCfg.cliUartHandle                = gMmwMssMCB.commandUartHandle;
    cliCfg.taskPriority                 = 3;
    cliCfg.enableMMWaveExtension        = 1U;
    cliCfg.usePolledMode                = true;
    cnt=0;
    cliCfg.tableEntry[cnt].cmd            = "sensorStart";
    cliCfg.tableEntry[cnt].helpString     = "[doReconfig(optional, default:enabled)]";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLISensorStart;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "sensorStop";
    cliCfg.tableEntry[cnt].helpString     = "No arguments";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLISensorStop;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "guiMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <detectedObjects> <logMagRange> <noiseProfile> <rangeAzimuthHeatMap> <rangeDopplerHeatMap> <statsInfo>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIGuiMonSel;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "cfarCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <procDirection> <averageMode> <winLen> <guardLen> <noiseDiv> <cyclicMode> <thresholdScale>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICfarCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "peakGrouping";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <groupingMode> <rangeDimEn> <dopplerDimEn> <startRangeIdx> <endRangeIdx>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIPeakGroupingCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "dataLogger";
    cliCfg.tableEntry[cnt].helpString     = "<mssLogger | dssLogger>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLISetDataLogger;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "multiObjBeamForming";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled> <threshold>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIMultiObjBeamForming;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "calibDcRangeSig";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled> <negativeBinIdx> <positiveBinIdx> <numAvgFrames>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICalibDcRangeSig;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "extendedMaxVelocity";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIExtendedMaxVelocity;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "clutterRemoval";
    cliCfg.tableEntry[cnt].helpString     = "<enabled>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIClutterRemoval;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "adcbufCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <adcOutputFmt> <SampleSwap> <ChanInterleave> <ChirpThreshold>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIADCBufCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "compRangeBiasAndRxChanPhase";
    cliCfg.tableEntry[cnt].helpString     = "<rangeBias> <Re00> <Im00> <Re01> <Im01> <Re02> <Im02> <Re03> <Im03> <Re10> <Im10> <Re11> <Im11> <Re12> <Im12> <Re13> <Im13> ";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "measureRangeBiasAndRxChanPhase";
    cliCfg.tableEntry[cnt].helpString     = "<enabled> <targetDistance> <searchWin>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "bpmCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled> <chirp0Idx> <chirp1Idx>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIBpmCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "nearFieldCfg";
    cliCfg.tableEntry[cnt].helpString     = "<enabled> <startRangeIndex> <endRangeIndex>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLINearFieldCorrection;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "CQRxSatMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<profile> <satMonSel> <priSliceDuration> <numSlices> <rxChanMask>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIChirpQualityRxSatMonCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "CQSigImgMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<profile> <numSlices> <numSamplePerSlice>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIChirpQualitySigImgMonCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "analogMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<rxSaturation> <sigImgBand>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIAnalogMonitorCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "lvdsStreamCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enableHeader> <dataFmt> <enableSW>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLILvdsStreamCfg;
    cnt++;

    /* Open the CLI: */
    if (CLI_open (&cliCfg) < 0)
    {
        printf ("Error: Unable to open the CLI\n");
        return;
    }
    printf ("Debug: CLI is operational\n");
    return;
}




/**************************************************************************
 **************************** CLI Functions *******************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is the HELP generated by the CLI Module
 *
 *  \ingroup CLI_UTIL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t CLI_help (int32_t argc, char* argv[])
{
    uint32_t    index;

    /* Display the banner: */
    CLI_write ("Help: This will display the usage of the CLI commands\n");
    CLI_write ("Command: Help Description\n");

    /* Cycle through all the registered CLI commands: */
    for (index = 0; index < gCLI.numCLICommands; index++)
    {
        /* Display the help string*/
        CLI_write ("%s: %s\n",
                    gCLI.cfg.tableEntry[index].cmd,
                   (gCLI.cfg.tableEntry[index].helpString == NULL) ?
                    "No help available" :
                    gCLI.cfg.tableEntry[index].helpString);
    }

    /* Is the mmWave Extension enabled? */
    if (gCLI.cfg.enableMMWaveExtension == 1U)
    {
        /* YES: Pass the control to the extension help handler. */
        CLI_MMWaveExtensionHelp ();
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  \ingroup CLI_UTIL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void CLI_task(void)
{
    char*                   tokenizedArgs[CLI_MAX_ARGS];
    char*                   ptrCLICommand;
    char                    delimitter[] = " \r\n";
    uint32_t                argIndex;
    CLI_CmdTableEntry*      ptrCLICommandEntry;
    int32_t                 cliStatus;
    uint32_t                index;

    /* Read the command message from the UART: */
    UART_read (gCLI.cfg.cliUartHandle, &cmdString[0], (sizeof(cmdString) - 1));

    /* Reset all the tokenized arguments: */
    memset ((void *)&tokenizedArgs, 0, sizeof(tokenizedArgs));
    argIndex      = 0;
    ptrCLICommand = (char*)&cmdString[0];

    /* comment lines found - ignore the whole line*/
    if (cmdString[0]=='%') {
        CLI_write ("Skipped\n");
        return;
    }

    /* Set the CLI status: */
    cliStatus = -1;

    /* The command has been entered we now tokenize the command message */
    while (1)
    {
        /* Tokenize the arguments: */
        tokenizedArgs[argIndex] = strtok(ptrCLICommand, delimitter);
        if (tokenizedArgs[argIndex] == NULL)
            break;

        /* Increment the argument index: */
        argIndex++;
        if (argIndex >= CLI_MAX_ARGS)
            break;

        /* Reset the command string */
        ptrCLICommand = NULL;
    }

    /* Were we able to tokenize the CLI command? */
    if (argIndex == 0)
        goto END;

    /* Cycle through all the registered CLI commands: */
    for (index = 0; index < gCLI.numCLICommands; index++)
    {
        ptrCLICommandEntry = &gCLI.cfg.tableEntry[index];

        /* Do we have a match? */
        if (strcmp(ptrCLICommandEntry->cmd, tokenizedArgs[0]) == 0)
        {
            /* YES: Pass this to the CLI registered function */
            cliStatus = ptrCLICommandEntry->cmdHandlerFxn (argIndex, tokenizedArgs);
            if (cliStatus == 0)
            {
                CLI_write ("Done\n");
            }
            else
            {
                CLI_write ("Error %d\n", cliStatus);
            }
            break;
        }
    }

    /* Did we get a matching CLI command? */
    if (index == gCLI.numCLICommands)
    {
        /* NO matching command found. Is the mmWave extension enabled? */
        if (gCLI.cfg.enableMMWaveExtension == 1U)
        {
            /* Yes: Pass this to the mmWave extension handler */
            cliStatus = CLI_MMWaveExtensionHandler (argIndex, tokenizedArgs);
        }

        /* Was the CLI command found? */
        if (cliStatus == -1)
        {
            /* No: The command was still not found */
            CLI_write ("'%s' is not recognized as a CLI command\n", tokenizedArgs[0]);
        }
    }

END:
    /* Demo Prompt: */
    CLI_write (gCLI.cfg.cliPrompt);

    /* Reset the command string: */
    memset ((void *)&cmdString[0], 0, sizeof(cmdString));


}

/**
 *  @b Description
 *  @n
 *      Logging function which can log the messages to the CLI console
 *
 *  @param[in]  format
 *      Format string
 *
 *  \ingroup CLI_UTIL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void CLI_write (const char* format, ...)
{
    va_list     arg;
    char        logMessage[256];
    int32_t     sizeMessage;

    /* Format the message: */
    va_start (arg, format);
    sizeMessage = vsnprintf (&logMessage[0], sizeof(logMessage), format, arg);
    va_end (arg);

    /* Log the message on the UART CLI console: */
    if (gCLI.cfg.usePolledMode == true)
    {
        /* Polled mode: */
        UART_writePolling (gCLI.cfg.cliUartHandle, (uint8_t*)&logMessage[0], sizeMessage);
    }
    else
    {
        /* Blocking Mode: */
        UART_write (gCLI.cfg.cliUartHandle, (uint8_t*)&logMessage[0], sizeMessage);
    }
}

/**
 *  @b Description
 *  @n
 *      This is the function which is used to initialize and setup the CLI
 *
 *  @param[in]  ptrCLICfg
 *      Pointer to the CLI configuration
 *
 *  \ingroup CLI_UTIL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t CLI_open (CLI_Cfg* ptrCLICfg)
{
    uint32_t        index;

    /* Sanity Check: Validate the arguments */
    if (ptrCLICfg == NULL)
        return -1;

    /* Initialize the CLI MCB: */
    memset ((void*)&gCLI, 0, sizeof(MmwDemo_CliMCB));

    /* Copy over the configuration: */
    memcpy ((void *)&gCLI.cfg, (void *)ptrCLICfg, sizeof(CLI_Cfg));

    /* Cycle through and determine the number of supported CLI commands: */
    for (index = 0; index < CLI_MAX_CMD; index++)
    {
        /* Do we have a valid entry? */
        if (gCLI.cfg.tableEntry[index].cmd == NULL)
        {
            /* NO: This is the last entry */
            break;
        }
        else
        {
            /* YES: Increment the number of CLI commands */
            gCLI.numCLICommands = gCLI.numCLICommands + 1;
        }
    }

    /* Is the mmWave Extension enabled? */
    if (gCLI.cfg.enableMMWaveExtension == 1U)
    {
        /* YES: Initialize the CLI Extension: */
        if (CLI_MMWaveExtensionInit (ptrCLICfg) < 0)
            return -1;
    }

    /* Do we have a CLI Prompt specified?  */
    if (gCLI.cfg.cliPrompt == NULL)
        gCLI.cfg.cliPrompt = "CLI:/>";

    /* The CLI provides a help command by default:
     * - Since we are adding this at the end of the table; a user of this module can also
     *   override this to provide its own implementation. */
    gCLI.cfg.tableEntry[gCLI.numCLICommands].cmd           = "help";
    gCLI.cfg.tableEntry[gCLI.numCLICommands].helpString    = NULL;
    gCLI.cfg.tableEntry[gCLI.numCLICommands].cmdHandlerFxn = CLI_help;

    /* Increment the number of CLI commands: */
    gCLI.numCLICommands++;

    /* Do we have a banner to be displayed? */
    if (gCLI.cfg.cliBanner != NULL)
    {
        /* YES: Display the banner */
        CLI_write (gCLI.cfg.cliBanner);
    }
    
    /* Demo Prompt: */
    CLI_write (gCLI.cfg.cliPrompt);

    /* Reset the command string: */
    memset ((void *)&cmdString[0], 0, sizeof(cmdString));

    return 0;
}


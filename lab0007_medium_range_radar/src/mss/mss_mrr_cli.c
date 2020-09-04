/*
 *   @file  mss_mrr_cli.c
 *
 *   @brief
 *      MSS Minimal CLI Implementation for the MRR TI Design
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
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/cli/cli.h>

/* Application Include Files: */
#include "mss_mrr.h"
#include "../common/mrr_config_consts.h"

/**************************************************************************
 *************************** Local Functions ******************************
 **************************************************************************/
static int32_t MRR_MSS_CLISensorStart (int32_t argc, char* argv[]);
static int32_t MRR_MSS_CLISensorStop (int32_t argc, char* argv[]);
static int32_t MRR_MSS_CLIBasicCfg (int32_t argc, char* argv[]);
static int32_t MRR_MSS_CLIAdvancedFrameCfg (int32_t argc, char* argv[]);

/**************************************************************************
 ****************************** CLI Functions *****************************
 **************************************************************************/

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
static int32_t MRR_MSS_CLISensorStart (int32_t argc, char* argv[])
{
    MMWave_CalibrationCfg   calibrationCfg;
    int32_t                 errCode;
    
    if (gMrrMSSMCB.runningStatus == true)
    {   
        /* Already running. */
        return 0; 
    }
    /* The sensor can only be started; if the link has been configured */
    if (gMrrMSSMCB.cfgStatus == true)
    {
        /* Initialize the calibration configuration: */
        memset ((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

        /* Populate the calibration configuration: */
        calibrationCfg.dfeDataOutputMode                          = 
        MMWave_DFEDataOutputMode_ADVANCED_FRAME;
        calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = true;
        calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = true;
        calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

        System_printf ("Debug: Sensor will start momentarily. \n");

        /* Start the mmWave: */
        if (MMWave_start (gMrrMSSMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
        {
            /* Error: Unable to start the mmWave control module */
            System_printf ("Error: mmWave start failed [Error code %d]\n", errCode);
            return -1;
        }
        
        gMrrMSSMCB.runningStatus = true;
        return 0;
    }
    else
    {
        /* Invalid CLI use case; doing a sensor start without executing the basic or advanced configuration
         * command. Inform the user and return an error code. */
        System_printf ("Error: Please ensure that the XXXCfg CLI command is invoked before starting the sensor\n");
        return -1;
    }
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
static int32_t MRR_MSS_CLISensorStop (int32_t argc, char* argv[])
{
    int32_t errCode;

    if (gMrrMSSMCB.runningStatus == false)
    {
        return 0; // Already stopped. 
    }

    /* The sensor can only be stopped; if the link has been configured */
    if (gMrrMSSMCB.cfgStatus == true)
    {
        /* Stop the sensor */
        if (MMWave_stop (gMrrMSSMCB.ctrlHandle, &errCode) < 0)
        {
            /* Error: Unable to stop the mmWave control module */
            System_printf ("Error: mmWave stop failed [Error code %d]\n", errCode);
            return -1;
        }
        System_printf ("Debug: Sensor has been stopped\n");

        /* The link is no longer configured. */
        gMrrMSSMCB.runningStatus = true; 
        return 0;
    }
    else
    {
        /* Invalid CLI use case; doing a sensor stop multiple times. Inform the user and return an error code. */
        System_printf ("Error: Sensor has already been stopped. Reconfigure and start the sensor if required\n");
        return -1;
    }
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for basic configuration
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
static int32_t MRR_MSS_CLIBasicCfg (int32_t argc, char* argv[])
{
    MMWave_OpenCfg          openCfg;
    int32_t                 errCode;
    rlProfileCfg_t          profileCfg;
    rlChirpCfg_t            chirpCfg;
    rlFrameCfg_t            frameCfg;
    int32_t                 retVal;

    if (gMrrMSSMCB.cfgStatus == true)
    {
        /* Radar has already been configured. */
        return 0;
    }

    /* Setup the calibration frequency: */
    openCfg.freqLimitLow  = 760U;
    openCfg.freqLimitHigh = 810U;
    openCfg.defaultAsyncEventHandler = MMWave_DefaultAsyncEventHandler_MSS;
    
    /* Initialize the minimal configuration: */
    Cfg_ChannelCfgInitParams  (&openCfg.chCfg);
    Cfg_LowPowerModeInitParams(&openCfg.lowPowerMode);
    Cfg_ADCOutCfgInitParams   (&openCfg.adcOutCfg);

    /* Open the mmWave module: */
    if (MMWave_open (gMrrMSSMCB.ctrlHandle, &openCfg, NULL, &errCode) < 0)
    {
        System_printf ("Error: MMWDemoMSS mmWave open configuration failed [Error code %d]\n", errCode);
        return -1;
    }

    /********************************************************************************
     * MMWave Link and BSS is operational now. In minimal mode we have access to all
     * the mmWave Link API to perform the configuration
     *
     * Profile configuration:
     ********************************************************************************/
    Cfg_ProfileCfgInitParams (0U, &profileCfg);
    retVal = rlSetProfileConfig (RL_DEVICE_MAP_INTERNAL_BSS, 1U, &profileCfg);
    if (retVal != RL_RET_CODE_OK)
    {
        System_printf ("Error: Unable to configure the profile [Error %d]\n", retVal);
        return -1;
    }

    /********************************************************************************
     * Chirp configuration:
     ********************************************************************************/
    Cfg_ChirpCfgInitParams (0U, &chirpCfg);
    retVal = rlSetChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, 1U, &chirpCfg);
    if (retVal != RL_RET_CODE_OK)
    {
        System_printf ("Error: Unable to configure the chirp [Error %d]\n", retVal);
        return -1;
    }

    /********************************************************************************
     * Frame configuration:
     ********************************************************************************/
    Cfg_FrameCfgInitParams (&frameCfg);
    retVal = rlSetFrameConfig(RL_DEVICE_MAP_INTERNAL_BSS, &frameCfg);
    if (retVal != RL_RET_CODE_OK)
    {
        System_printf ("Error: Unable to configure the frame [Error %d]\n", retVal);
        return -1;
    }

    /* The link has been configured. */
    gMrrMSSMCB.cfgStatus = true;
    System_printf ("Debug: Basic configuration completed. Start the sensor...\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for advanced frame configuration
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
static int32_t MRR_MSS_CLIAdvancedFrameCfg (int32_t argc, char* argv[])
{
    MMWave_OpenCfg          openCfg;
    int32_t                 errCode;
    rlProfileCfg_t          profileCfg;
    rlChirpCfg_t            chirpCfg;
    rlAdvFrameCfg_t         advFrameCfg;
    int32_t                 retVal;
    int32_t                 indx;
    rlRfLdoBypassCfg_t rfLdoBypassCfg[1] = {0};
    
    if (gMrrMSSMCB.cfgStatus == true)
    {
        /* Radar has already been configured. */
        return 0;
    }

    /* Setup the calibration frequency: */
    openCfg.freqLimitLow  = 760U;
    openCfg.freqLimitHigh = 810U;
    openCfg.defaultAsyncEventHandler = MMWave_DefaultAsyncEventHandler_MSS;
    
    /* Initialize the minimal configuration: */
    Cfg_ChannelCfgInitParams  (&openCfg.chCfg);
    Cfg_LowPowerModeInitParams(&openCfg.lowPowerMode);
    Cfg_ADCOutCfgInitParams   (&openCfg.adcOutCfg);

    /* Open the mmWave module: */
    if (MMWave_open (gMrrMSSMCB.ctrlHandle, &openCfg, NULL, &errCode) < 0)
    {
        System_printf ("Error: MMWDemoMSS mmWave open configuration failed [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Set LDO Bypass\n");
    rfLdoBypassCfg->ldoBypassEnable = 0x03;
    retVal = rlRfSetLdoBypassConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlRfLdoBypassCfg_t*)rfLdoBypassCfg);
    if (retVal != RL_RET_CODE_OK)
    {
        System_printf ("Error: LDO Bypass config failed [Error %d]\n", retVal);
        return -1;
    }
    
        
    /********************************************************************************
     * MMWave Link and BSS is operational now. In minimal mode we have access to all
     * the mmWave Link API to perform the configuration
     *
     * Profile configurations:
     ********************************************************************************/
    for (indx = 0; indx < NUM_PROFILES; indx++)
    {
        
        Cfg_ProfileCfgInitParams (indx, &profileCfg);
        retVal = rlSetProfileConfig (RL_DEVICE_MAP_INTERNAL_BSS, 1U, &profileCfg);
        if (retVal != RL_RET_CODE_OK)
        {
            System_printf ("Error: Unable to configure the profile %d [Error %d]\n", indx, retVal);
            return -1;
        }
       
    }
    
    /********************************************************************************
     * Chirp configurations:
     ********************************************************************************/
    for (indx = 0; indx < NUM_CHIRP_PROG; indx++)
    {
        Cfg_ChirpCfgInitParams (indx, &chirpCfg);
        retVal = rlSetChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, 1U, &chirpCfg);
        if (retVal != RL_RET_CODE_OK)
        {
            System_printf ("Error: Unable to configure chirp %d [Error %d]\n", indx, retVal);
            return -1;
        }
    }
    

    /* Advanced Frame configuration. */
    Cfg_AdvFrameCfgInitParams (&advFrameCfg);
    retVal = rlSetAdvFrameConfig(RL_DEVICE_MAP_INTERNAL_BSS, &advFrameCfg);
    if (retVal != RL_RET_CODE_OK)
    {
        System_printf ("Error: Advanced Frame configuration failed [Error %d]\n", retVal);
        return -1;
    }
    
    /* The link has been configured. */
    gMrrMSSMCB.cfgStatus = true;
    System_printf ("Debug: MMWave has been configured for MRR.\n");
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
void MRR_MSS_CLIInit (void)
{
    CLI_Cfg     cliCfg;
    static char* dummy[1];

    /* Initialize the CLI configuration: */
    memset ((void *)&cliCfg, 0, sizeof(CLI_Cfg));

    /* Populate the CLI configuration: */
    cliCfg.cliPrompt                    = "MrrTIDesign:/>";
    cliCfg.cliUartHandle                = gMrrMSSMCB.commandUartHandle;
    cliCfg.taskPriority                 = 3;
    cliCfg.mmWaveHandle                 = gMrrMSSMCB.ctrlHandle;
    cliCfg.enableMMWaveExtension        = 0U;
    cliCfg.usePolledMode                = true;

    cliCfg.tableEntry[0].cmd            = "basicCfg";
    cliCfg.tableEntry[0].helpString     = "Basic Cfg [Hardcoded Parameters]";
    cliCfg.tableEntry[0].cmdHandlerFxn  = MRR_MSS_CLIBasicCfg;

    cliCfg.tableEntry[1].cmd            = "advFrameCfg";
    cliCfg.tableEntry[1].helpString     = "Advanced Frame Cfg [Hardcoded Parameters]";
    cliCfg.tableEntry[1].cmdHandlerFxn  = MRR_MSS_CLIAdvancedFrameCfg;

    cliCfg.tableEntry[2].cmd            = "sensorStart";
    cliCfg.tableEntry[2].helpString     = "Start the sensor; ensure that the configuration is completed";
    cliCfg.tableEntry[2].cmdHandlerFxn  = MRR_MSS_CLISensorStart;

    cliCfg.tableEntry[3].cmd            = "sensorStop";
    cliCfg.tableEntry[3].helpString     = "Stop the sensor";
    cliCfg.tableEntry[3].cmdHandlerFxn  = MRR_MSS_CLISensorStop;

    #if 0
    /* Open the CLI: */
    if (CLI_open (&cliCfg) < 0)
    {
        System_printf ("Error: Unable to open the CLI\n");
        return;
    }
    System_printf ("Debug: CLI is operational\n");
    #endif
    
    /* The link is not configured. */
    gMrrMSSMCB.cfgStatus = false;
    gMrrMSSMCB.runningStatus = false;
    gMrrMSSMCB.isMMWaveOpen = false;
    
    MRR_MSS_CLIAdvancedFrameCfg(1, dummy);
    
    
    MRR_MSS_CLISensorStart(2, dummy);
    return;
}



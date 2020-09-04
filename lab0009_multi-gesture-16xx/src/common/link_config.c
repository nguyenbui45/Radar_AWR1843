/*
 *   @file  link_config.c
 *
 *   @brief
 *      mmWaveLink configuration to RadarSS from DSS/MSS
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
#include <ti/drivers/edma/edma.h>

#ifdef SUBSYS_MSS
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>
#else
#include <ti/sysbios/family/c64p/Hwi.h>
#endif

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/drivers/crc/crc.h>
#include <ti/control/mmwavelink/include/rl_driver.h>


/* Test include files: */
#include "gesture_config_consts.h"
#include "dss_gesture.h"
#define GestDemo_Assert         GestDemo_dssAssert

/**************************************************************************
 *************************** Local Structures *****************************
 **************************************************************************/
#define MAX_CHIRP_COUNT    6

typedef struct MmWaveLink_CFG
{
    rlChanCfg_t             channelCfg;
    rlAdcOutCfg_t           adcOutCfg;
    rlLowPowerModeCfg_t     lowPwrCfg;
    rlProfileCfg_t          profileCfg[2]; /* profile0- IDLE mode, profile1- ACTIVE mode */
    rlChirpCfg_t            chirpCfg[MAX_CHIRP_COUNT]; /* [0:1]- 2 chirps for IDLE mode and [2:5]- 4 chirps for ACTIVE mode */
    rlFrameCfg_t            frameCfg[2]; /* frame0 - IDLE mode, frame1 - ACTIVE mode */
    rlDynPwrSave_t          dynPwrSaveCfg;
    rlRfDevCfg_t            rfDevCfg;   //TODO JIT May not need, call directly from MSS/DSS
    rlRunTimeCalibConf_t    runTimeCalib;
}MmWaveLink_CFG_t;

/**
 * @brief
 *  Mmwave Link Master Control Block
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  Mmwave Link.
 */
typedef struct MmwaveLink_MCB
{
    /**
     * @brief   Handle to the BSS Mailbox
     */
    Mbox_Handle              bssMailbox;

    /**
     * @brief   Semaphore handle for the mmWave Link
     */
    Semaphore_Handle            linkSemaphore;

    /**
     * @brief   mmWave Link Spawning function
     */
    RL_P_OSI_SPAWN_ENTRY        spawnFxn;

    /**
     * @brief   Status of the BSS:
     */
    volatile uint32_t           bssStatus;

    /**
     * @brief   Counter which tracks of the number of times the spawn function was
     * overrun.
     */
    uint32_t                    spawnOverrun;
    /**
     * @brief   Handle to the CRC Channel
     */
    CRC_Handle                  crcHandle;
}MmwaveLink_MCB;

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/* Global Variable for tracking information required by the mmWave Link */
MmwaveLink_MCB    gMmwaveLinkMCB;
MmWaveLink_CFG_t  gMmwaveLinkCfg;

uint32_t gMonitorHdrCnt = 0U;
uint32_t gInitTimeCalibStatus = 0U;
uint32_t gRunTimeCalibStatus = 0U;
volatile uint32_t gFrameStartStatus = 0U;
rlUInt16_t gMonitoringStatus = 0U;
rlUInt8_t isGetGpAdcMeasData = 0U;

RL_P_EVENT_HANDLER g_MailboxInterruptFunc;

/* received GPAdc Data over Async Event */
rlRecvdGpAdcData_t rcvGpAdcData = {0};

uint16_t monAeCnt [25] = { 0 };
uint16_t monFailRepCheck [25] = { 0 };
uint32_t anaMonEn = 0;

/* Async Event Enable and Direction configuration */
rlRfDevCfg_t rfDevCfg = {0};

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
extern rlInt32_t Osal_mutexCreate(rlOsiMutexHdl_t* mutexHdl, rlInt8_t* name);
extern rlInt32_t Osal_mutexLock(rlOsiMutexHdl_t* mutexHdl, rlOsiTime_t timeout);
extern rlInt32_t Osal_mutexUnlock(rlOsiMutexHdl_t* mutexHdl);
extern rlInt32_t Osal_mutexDelete(rlOsiMutexHdl_t* mutexHdl);
extern rlInt32_t Osal_semCreate(rlOsiSemHdl_t* semHdl, rlInt8_t* name);
extern rlInt32_t Osal_semWait(rlOsiSemHdl_t* semHdl, rlOsiTime_t timeout);
extern rlInt32_t Osal_semSignal(rlOsiSemHdl_t* semHdl);
extern rlInt32_t Osal_semDelete(rlOsiSemHdl_t* semHdl);
extern rlInt32_t Osal_delay(rlUInt32_t delay);

/**************************************************************************
 ************************* Link Unit Test Functions ***********************
 **************************************************************************/

 
/**
 *  @b Description
 *  @n  
 *      The function initializes the frame configuration with the default
 *      parameters.
 *
 *  @param[out] ptrFrameCfg
 *      Pointer to the frame configuration
 *
 *  @retval
 *      Not applicable
 */
void MmwaveLink_FrameCfgInitParams (void)
{
    /* Initialize the configuration: */
    memset ((void*)&gMmwaveLinkCfg.frameCfg[0], 0, sizeof(rlFrameCfg_t));

    /* Populate the default configuration: */
    gMmwaveLinkCfg.frameCfg[0].chirpEndIdx        = FRAME_IDLE_CHIRP_END_IDX;
    gMmwaveLinkCfg.frameCfg[0].chirpStartIdx      = FRAME_IDLE_CHIRP_START_IDX;
    gMmwaveLinkCfg.frameCfg[0].numFrames          = FRAME_IDLE_COUNT;
    gMmwaveLinkCfg.frameCfg[0].numLoops           = FRAME_IDLE_LOOP_COUNT;
    gMmwaveLinkCfg.frameCfg[0].triggerSelect      = FRAME_IDLE_TIGGER_SELECT;
    gMmwaveLinkCfg.frameCfg[0].framePeriodicity   = FRAME_IDLE_PERIODICITY_VAL;
    gMmwaveLinkCfg.frameCfg[0].frameTriggerDelay  = FRAME_IDLE_TRIGGER_DELAY_VAL;
    gMmwaveLinkCfg.frameCfg[0].numAdcSamples      = FRAME_IDLE_NUM_CMPLX_ADC_SAMPLES;
    
    /* Initialize the configuration: */
    memset ((void*)&gMmwaveLinkCfg.frameCfg[1], 0, sizeof(rlFrameCfg_t));

    /* Populate the default configuration: */
    gMmwaveLinkCfg.frameCfg[1].chirpEndIdx        = FRAME_ACTIVE_CHIRP_END_IDX;
    gMmwaveLinkCfg.frameCfg[1].chirpStartIdx      = FRAME_ACTIVE_CHIRP_START_IDX;
    gMmwaveLinkCfg.frameCfg[1].numFrames          = FRAME_ACTIVE_COUNT;
    gMmwaveLinkCfg.frameCfg[1].numLoops           = FRAME_ACTIVE_LOOP_COUNT;
    gMmwaveLinkCfg.frameCfg[1].triggerSelect      = FRAME_ACTIVE_TIGGER_SELECT;
    gMmwaveLinkCfg.frameCfg[1].framePeriodicity   = FRAME_ACTIVE_PERIODICITY_VAL;
    gMmwaveLinkCfg.frameCfg[1].frameTriggerDelay  = FRAME_ACTIVE_TRIGGER_DELAY_VAL;
    gMmwaveLinkCfg.frameCfg[1].numAdcSamples      = FRAME_ACTIVE_NUM_CMPLX_ADC_SAMPLES;
    
    return;
}

/**
 *  @b Description
 *  @n
 *      The function initializes the profile configuration with the default
 *      parameters.
 *
 *  @param[in] profileNum
 *      Profile number to be initialized
 *  @param[out] ptrProfileCfg
 *      Pointer to the profile configuration
 *
 *  @retval
 *      Not applicable
 */
void MmwaveLink_ProfileCfgInitParams (void)
{
    /* Initialize the configuration: */
    memset ((void*)&gMmwaveLinkCfg.profileCfg[0], 0, sizeof(rlProfileCfg_t));
    /* Populate the default configuration for profile 0  */
    gMmwaveLinkCfg.profileCfg[0].profileId             = PROFILE_IDLE_PROFILE_ID;
    gMmwaveLinkCfg.profileCfg[0].startFreqConst        = PROFILE_IDLE_START_FREQ_VAL;
    gMmwaveLinkCfg.profileCfg[0].idleTimeConst         = PROFILE_IDLE_IDLE_TIME_VAL;
    gMmwaveLinkCfg.profileCfg[0].adcStartTimeConst     = PROFILE_IDLE_ADC_START_TIME_VAL;
    gMmwaveLinkCfg.profileCfg[0].rampEndTime           = PROFILE_IDLE_RAMP_END_TIME_VAL;
    gMmwaveLinkCfg.profileCfg[0].txOutPowerBackoffCode = PROFILE_IDLE_TXOUT_POWER_BACKOFF;
    gMmwaveLinkCfg.profileCfg[0].txPhaseShifter        = PROFILE_IDLE_TXPHASESHIFTER_VAL;
    gMmwaveLinkCfg.profileCfg[0].freqSlopeConst        = PROFILE_IDLE_FREQ_SLOPE_VAL;
    gMmwaveLinkCfg.profileCfg[0].txStartTime           = PROFILE_IDLE_TX_START_TIME_VAL;
    gMmwaveLinkCfg.profileCfg[0].numAdcSamples         = PROFILE_IDLE_ADC_SAMPLE_VAL;
    gMmwaveLinkCfg.profileCfg[0].digOutSampleRate      = PROFILE_IDLE_DIGOUT_SAMPLERATE_VAL;
    gMmwaveLinkCfg.profileCfg[0].hpfCornerFreq1        = PROFILE_IDLE_HPFCORNER_FREQ1_VAL;
    gMmwaveLinkCfg.profileCfg[0].hpfCornerFreq2        = PROFILE_IDLE_HPFCORNER_FREQ2_VAL;
    gMmwaveLinkCfg.profileCfg[0].rxGain                = PROFILE_IDLE_RX_GAIN_VAL;

        
    /* Initialize the configuration: */
    memset ((void*)&gMmwaveLinkCfg.profileCfg[1], 0, sizeof(rlProfileCfg_t));
    /* Populate the default configuration for profile 1  */
    gMmwaveLinkCfg.profileCfg[1].profileId             = PROFILE_ACTIVE_PROFILE_ID;
    gMmwaveLinkCfg.profileCfg[1].startFreqConst        = PROFILE_ACTIVE_START_FREQ_VAL;
    gMmwaveLinkCfg.profileCfg[1].idleTimeConst         = PROFILE_ACTIVE_IDLE_TIME_VAL;
    gMmwaveLinkCfg.profileCfg[1].adcStartTimeConst     = PROFILE_ACTIVE_ADC_START_TIME_VAL;
    gMmwaveLinkCfg.profileCfg[1].rampEndTime           = PROFILE_ACTIVE_RAMP_END_TIME_VAL;
    gMmwaveLinkCfg.profileCfg[1].txOutPowerBackoffCode = PROFILE_ACTIVE_TXOUT_POWER_BACKOFF;
    gMmwaveLinkCfg.profileCfg[1].txPhaseShifter        = PROFILE_ACTIVE_TXPHASESHIFTER_VAL;
    gMmwaveLinkCfg.profileCfg[1].freqSlopeConst        = PROFILE_ACTIVE_FREQ_SLOPE_VAL;
    gMmwaveLinkCfg.profileCfg[1].txStartTime           = PROFILE_ACTIVE_TX_START_TIME_VAL;
    gMmwaveLinkCfg.profileCfg[1].numAdcSamples         = PROFILE_ACTIVE_ADC_SAMPLE_VAL;
    gMmwaveLinkCfg.profileCfg[1].digOutSampleRate      = PROFILE_ACTIVE_DIGOUT_SAMPLERATE_VAL;
    gMmwaveLinkCfg.profileCfg[1].hpfCornerFreq1        = PROFILE_ACTIVE_HPFCORNER_FREQ1_VAL;
    gMmwaveLinkCfg.profileCfg[1].hpfCornerFreq2        = PROFILE_ACTIVE_HPFCORNER_FREQ2_VAL;
    gMmwaveLinkCfg.profileCfg[1].rxGain                = PROFILE_ACTIVE_RX_GAIN_VAL;

    return;
}

/**
 *  @b Description
 *  @n
 *      The function initializes the chirp configuration with the default
 *      parameters.
 *
 *  @param[out] chirpNum
 *      Chirp Number to be configured
 *  @param[out] ptrChirpCfg
 *      Pointer to the chirp configuration
 *
 *  @retval
 *      Not applicable
 */
void MmwaveLink_ChirpCfgInitParams (void)
{
    /* Initialize all the chirp configuration: */
    memset ((void*)&gMmwaveLinkCfg.chirpCfg[0], 0, sizeof(rlChirpCfg_t)*MAX_CHIRP_COUNT);

    /* Populate the default configuration for IDLE Mode chirp 0. */
    gMmwaveLinkCfg.chirpCfg[0].profileId       = CHIRP_IDLE_0_PROFILE_ID;
    gMmwaveLinkCfg.chirpCfg[0].adcStartTimeVar = CHIRP_IDLE_0_ADC_START_TIME_VAL;
    gMmwaveLinkCfg.chirpCfg[0].chirpEndIdx     = CHIRP_IDLE_0_END_INDEX;
    gMmwaveLinkCfg.chirpCfg[0].chirpStartIdx   = CHIRP_IDLE_0_START_INDEX;
    gMmwaveLinkCfg.chirpCfg[0].idleTimeVar     = CHIRP_IDLE_0_IDLE_TIME_VAL;
    gMmwaveLinkCfg.chirpCfg[0].txEnable        = CHIRP_IDLE_0_TX_CHANNEL;
    gMmwaveLinkCfg.chirpCfg[0].startFreqVar    = CHIRP_IDLE_0_START_FREQ_VAL;
    gMmwaveLinkCfg.chirpCfg[0].freqSlopeVar    = CHIRP_IDLE_0_FREQ_SLOPE_VAL;

    /* Populate the default configuration for IDLE Mode chirp 1. */
    gMmwaveLinkCfg.chirpCfg[1].profileId       = CHIRP_IDLE_1_PROFILE_ID;
    gMmwaveLinkCfg.chirpCfg[1].adcStartTimeVar = CHIRP_IDLE_1_ADC_START_TIME_VAL;
    gMmwaveLinkCfg.chirpCfg[1].chirpEndIdx     = CHIRP_IDLE_1_END_INDEX;
    gMmwaveLinkCfg.chirpCfg[1].chirpStartIdx   = CHIRP_IDLE_1_START_INDEX;
    gMmwaveLinkCfg.chirpCfg[1].idleTimeVar     = CHIRP_IDLE_1_IDLE_TIME_VAL;
    gMmwaveLinkCfg.chirpCfg[1].txEnable        = CHIRP_IDLE_1_TX_CHANNEL;
    gMmwaveLinkCfg.chirpCfg[1].startFreqVar    = CHIRP_IDLE_1_START_FREQ_VAL;
    gMmwaveLinkCfg.chirpCfg[1].freqSlopeVar    = CHIRP_IDLE_1_FREQ_SLOPE_VAL;

    /* Populate the default configuration for ACTIVE Mode chirp 0. */
    gMmwaveLinkCfg.chirpCfg[2].profileId       = CHIRP_ACTIVE_0_PROFILE_ID;
    gMmwaveLinkCfg.chirpCfg[2].adcStartTimeVar = CHIRP_ACTIVE_0_ADC_START_TIME_VAL;
    gMmwaveLinkCfg.chirpCfg[2].chirpEndIdx     = CHIRP_ACTIVE_0_END_INDEX;
    gMmwaveLinkCfg.chirpCfg[2].chirpStartIdx   = CHIRP_ACTIVE_0_START_INDEX;
    gMmwaveLinkCfg.chirpCfg[2].idleTimeVar     = CHIRP_ACTIVE_0_IDLE_TIME_VAL;
    gMmwaveLinkCfg.chirpCfg[2].txEnable        = CHIRP_ACTIVE_0_TX_CHANNEL;
    gMmwaveLinkCfg.chirpCfg[2].startFreqVar    = CHIRP_ACTIVE_0_START_FREQ_VAL;
    gMmwaveLinkCfg.chirpCfg[2].freqSlopeVar    = CHIRP_ACTIVE_0_FREQ_SLOPE_VAL;
    
    /* Populate the default configuration for ACTIVE Mode chirp 1. */
    gMmwaveLinkCfg.chirpCfg[3].profileId       = CHIRP_ACTIVE_1_PROFILE_ID;
    gMmwaveLinkCfg.chirpCfg[3].adcStartTimeVar = CHIRP_ACTIVE_1_ADC_START_TIME_VAL;
    gMmwaveLinkCfg.chirpCfg[3].chirpEndIdx     = CHIRP_ACTIVE_1_END_INDEX;
    gMmwaveLinkCfg.chirpCfg[3].chirpStartIdx   = CHIRP_ACTIVE_1_START_INDEX;
    gMmwaveLinkCfg.chirpCfg[3].idleTimeVar     = CHIRP_ACTIVE_1_IDLE_TIME_VAL;
    gMmwaveLinkCfg.chirpCfg[3].txEnable        = CHIRP_ACTIVE_1_TX_CHANNEL;
    gMmwaveLinkCfg.chirpCfg[3].startFreqVar    = CHIRP_ACTIVE_1_START_FREQ_VAL;
    gMmwaveLinkCfg.chirpCfg[3].freqSlopeVar    = CHIRP_ACTIVE_1_FREQ_SLOPE_VAL;

    /* Populate the default configuration for ACTIVE Mode chirp 2. */
    gMmwaveLinkCfg.chirpCfg[4].profileId       = CHIRP_ACTIVE_2_PROFILE_ID;
    gMmwaveLinkCfg.chirpCfg[4].adcStartTimeVar = CHIRP_ACTIVE_2_ADC_START_TIME_VAL;
    gMmwaveLinkCfg.chirpCfg[4].chirpEndIdx     = CHIRP_ACTIVE_2_END_INDEX;
    gMmwaveLinkCfg.chirpCfg[4].chirpStartIdx   = CHIRP_ACTIVE_2_START_INDEX;
    gMmwaveLinkCfg.chirpCfg[4].idleTimeVar     = CHIRP_ACTIVE_2_IDLE_TIME_VAL;
    gMmwaveLinkCfg.chirpCfg[4].txEnable        = CHIRP_ACTIVE_2_TX_CHANNEL;
    gMmwaveLinkCfg.chirpCfg[4].startFreqVar    = CHIRP_ACTIVE_2_START_FREQ_VAL;
    gMmwaveLinkCfg.chirpCfg[4].freqSlopeVar    = CHIRP_ACTIVE_2_FREQ_SLOPE_VAL;
    
    /* Populate the default configuration for ACTIVE Mode chirp 3. */
    gMmwaveLinkCfg.chirpCfg[5].profileId       = CHIRP_ACTIVE_3_PROFILE_ID;
    gMmwaveLinkCfg.chirpCfg[5].adcStartTimeVar = CHIRP_ACTIVE_3_ADC_START_TIME_VAL;
    gMmwaveLinkCfg.chirpCfg[5].chirpEndIdx     = CHIRP_ACTIVE_3_END_INDEX;
    gMmwaveLinkCfg.chirpCfg[5].chirpStartIdx   = CHIRP_ACTIVE_3_START_INDEX;
    gMmwaveLinkCfg.chirpCfg[5].idleTimeVar     = CHIRP_ACTIVE_3_IDLE_TIME_VAL;
    gMmwaveLinkCfg.chirpCfg[5].txEnable        = CHIRP_ACTIVE_3_TX_CHANNEL;
    gMmwaveLinkCfg.chirpCfg[5].startFreqVar    = CHIRP_ACTIVE_3_START_FREQ_VAL;
    gMmwaveLinkCfg.chirpCfg[5].freqSlopeVar    = CHIRP_ACTIVE_3_FREQ_SLOPE_VAL;
    return;
}

/**
 *  @b Description
 *  @n
 *      The function initializes the low power configuration with the default
 *      parameters.
 *
 *  @param[out] ptrLowPowerMode
 *      Pointer to the low power mode configuration
 *
 *  @retval
 *      Not applicable
 */
void MmwaveLink_LowPowerModeInitParams (void)
{
    /* Initialize the configuration: */
    memset ((void*)&gMmwaveLinkCfg.lowPwrCfg, 0, sizeof(rlLowPowerModeCfg_t));

    /* Populate the low power configuration: */
    //TODO use the 'low power mode' i.e. LP_ADC_MODE_LOW_POWER in 
    // version 1.2. 
    gMmwaveLinkCfg.lowPwrCfg.lpAdcMode     = LP_ADC_MODE_LOW_POWER;
    return;
}

/**
 *  @b Description
 *  @n
 *      The function initializes the channel configuration with the default
 *      parameters.
 *
 *  @param[out] ptrChannelCfg
 *      Pointer to the channel configuration
 *
 *  @retval
 *      Not applicable
 */
void MmwaveLink_ChannelCfgInitParams (void)
{
    /* Initialize the configuration: */
    memset ((void*)&gMmwaveLinkCfg.channelCfg, 0, sizeof(rlChanCfg_t));

    /* Populate the default configuration: */
    gMmwaveLinkCfg.channelCfg.rxChannelEn = GESTURE_RX_CHANNEL_SELECTION;
    gMmwaveLinkCfg.channelCfg.txChannelEn = GESTURE_TX_CHANNEL_SELECTION;
    gMmwaveLinkCfg.channelCfg.cascading   = 0; /* Single Chip (no cascading)*/
    
    return;
}

/**
 *  @b Description
 *  @n
 *      The function initializes the ADCOut configuration with the default
 *      parameters.
 *
 *  @param[out] ptrADCOutCfg
 *      Pointer to the ADCOutput configuration
 *
 *  @retval
 *      Not applicable
 */
void MmwaveLink_ADCOutCfgInitParams (void)
{
    /* Initialize the configuration: */
    memset ((void*)&gMmwaveLinkCfg.adcOutCfg, 0, sizeof(rlAdcOutCfg_t));

    /* Populate the default configuration: */
    gMmwaveLinkCfg.adcOutCfg.fmt.b2AdcBits   = GESTURE_ADC_BIT;
    gMmwaveLinkCfg.adcOutCfg.fmt.b2AdcOutFmt = GESTURE_ADC_FORMAT;
    gMmwaveLinkCfg.adcOutCfg.fmt.b8FullScaleReducFctr = 0;
    
    return;
}

void MmwaveLink_RfDevCfgInitParams (void)
{
    /* Initialize the configuration: */
    memset((void*)&gMmwaveLinkCfg.rfDevCfg, 0, sizeof(rlRfDevCfg_t));
    
    /* Populate the default configuration: */
#ifdef SUBSYS_DSS
    gMmwaveLinkCfg.rfDevCfg.aeDirection = ((2<<2)|  /* Global AE to DSS */
                                           (2<<0)); /* Monitoring AE to DSS */
#else
    gMmwaveLinkCfg.rfDevCfg.aeDirection = ((0<<2)|  /* Global AE to MSS */
                                           (0<<0)); /* Monitoring AE to MSS */
#endif
    gMmwaveLinkCfg.rfDevCfg.aeControl   = 0; /* b0:0 AE enable for frame-start, b1:0 AE enable for frame-stop */
    gMmwaveLinkCfg.rfDevCfg.aeCrcConfig = 0; /* 0:CRC16, 1:CRC32, 2:CRC64 */
    
    return;
}

void MmwaveLink_RunTimeCalibInitParams (void)
{
    /* Initialize the configuration: */
    memset((void*)&gMmwaveLinkCfg.runTimeCalib, 0, sizeof(rlRunTimeCalibConf_t));
    
    /* Enable calibration: Always enable one time calibration and all calibrations are enabled */
    gMmwaveLinkCfg.runTimeCalib.reportEn           = 0x1;
    gMmwaveLinkCfg.runTimeCalib.oneTimeCalibEnMask = CSL_FMKR (4U,  4U,  1U) | /* LODIST calibration   */
                                                     CSL_FMKR (9U,  9U,  1U) | /* TX Power calibration */
                                                     CSL_FMKR (10U, 10U, 1U);  /* RX gain calibration  */
    gMmwaveLinkCfg.runTimeCalib.periodicCalibEnMask = gMmwaveLinkCfg.runTimeCalib.oneTimeCalibEnMask;
    gMmwaveLinkCfg.runTimeCalib.calibPeriodicity    = 10;
}

rlChanCfg_t MmwaveLink_getChannelCfg (void)
{
    return gMmwaveLinkCfg.channelCfg;
}

rlChirpCfg_t MmwaveLink_getChirpCfg (uint8_t chirpCfgIdx)
{
    return gMmwaveLinkCfg.chirpCfg[chirpCfgIdx];
}

rlProfileCfg_t MmwaveLink_getProfileCfg (uint8_t profileId)
{
    return gMmwaveLinkCfg.profileCfg[profileId];
}

rlFrameCfg_t MmwaveLink_getFrameCfg (uint8_t frameCfgIdx)
{
    return gMmwaveLinkCfg.frameCfg[frameCfgIdx];
}

rlLowPowerModeCfg_t MmwaveLink_getLowPwrCfg (void)
{
    return gMmwaveLinkCfg.lowPwrCfg;
}

rlAdcOutCfg_t MmwaveLink_getAdcOutCfg (void)
{
    return gMmwaveLinkCfg.adcOutCfg;
}

/**
 *  @b Description
 *  @n
 *      Mailbox registered function which is invoked on the reception of data
 *
 *  @retval
 *      Success - Communicate Interface Channel Handle
 *  @retval
 *      Error   - NULL
 */
static void MmwaveLink_mboxCallbackFxn (Mbox_Handle handle, Mailbox_Type remoteEndpoint)
{
    /* Indicate to the Radar Link that a message has been received. */
    g_MailboxInterruptFunc(0, NULL);
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to open the communication
 *      interface channel
 *
 *  @retval
 *      Success - Communicate Interface Channel Handle
 *  @retval
 *      Error   - NULL
 */
static rlComIfHdl_t MmwaveLink_mboxOpen(rlUInt8_t deviceIndex, uint32_t flags)
{
    Mailbox_Config  cfg;
    int32_t         errCode;

    /* Initialize the mailbox configuration: */
    if(Mailbox_Config_init(&cfg) < 0)
    {
        System_printf("Error: Unable to initialize mailbox configuration\n");
        return NULL;
    }

    cfg.writeMode    = MAILBOX_MODE_POLLING;
    cfg.readMode     = MAILBOX_MODE_CALLBACK;
    cfg.readCallback = MmwaveLink_mboxCallbackFxn;

    /* Open the Mailbox to the BSS */
    gMmwaveLinkMCB.bssMailbox = Mailbox_open(MAILBOX_TYPE_BSS, &cfg, &errCode);
    if (gMmwaveLinkMCB.bssMailbox == NULL)
    {
        System_printf("Error: Unable to open the Mailbox Instance [Error code %d]\n", errCode);
        return NULL;
    }
    System_printf("Debug: BSS Mailbox Handle %p\n", gMmwaveLinkMCB.bssMailbox);
    return gMmwaveLinkMCB.bssMailbox;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to close the communication
 *      interface channel
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t MmwaveLink_mboxClose(rlComIfHdl_t fd)
{
    int32_t errCode;

    /* Close the Mailbox */
    errCode = Mailbox_close ((Mbox_Handle)fd);
    if (errCode < 0)
        System_printf ("Error: Unable to close the BSS Mailbox [Error code %d]\n", errCode);

    return errCode;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to read data from the communication
 *      interface channel
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t MmwaveLink_mboxRead(rlComIfHdl_t fd, uint8_t* pBuff, uint16_t len)
{
    return Mailbox_read((Mbox_Handle)fd, pBuff, len);
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to write data to the communication
 *      interface channel
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t MmwaveLink_mboxWrite(rlComIfHdl_t fd, uint8_t* pBuff, uint16_t len)
{
    int32_t    status;
    /*
      Currently, the mmwavelink can not detect the error condition where it did not receive a mailbox layer ACK from BSS.

      For instance:
      - The mmwavelink may try to send a message before an ACK was received for the previous message.
      - The mmwavelink may try to resend a message that did not receive a mmwavelink layer ACK back from BSS. It is possible that the
      message did not receive a mailbox layer ACK as well from BSS.

      In either case, Mailbox_writeReset() has to be called before another message is sent to BSS.

      The mmwavelink has no hooks to call the Mailbox_writeReset().
      Therefore, a write reset is done if it is detected that a mailbox layer ACK was not received for the
      previous message (MAILBOX_ETXFULL).
     */

    status = Mailbox_write((Mbox_Handle)fd, pBuff, len);
    if(status == MAILBOX_ETXFULL)
    {
        Mailbox_writeReset((Mbox_Handle)fd);
        status = Mailbox_write((Mbox_Handle)fd, pBuff, len);
    }

    return status;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to power on the AR1XX Device
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static rlInt32_t MmwaveLink_enableDevice(rlUInt8_t deviceIndex)
{
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to power off the AR1XX Device
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static rlInt32_t MmwaveLink_disableDevice(rlUInt8_t deviceIndex)
{
    System_printf("Debug: Disabling the device\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to mask the interrupts.
 *      In the case of Mailbox communication interface the driver will
 *      handle the interrupt management. This function is a dummy stub
 *
 *  @retval
 *      Not applicable
 */
static void MmwaveLink_maskHostIRQ(rlComIfHdl_t fd)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to umask the interrupts.
 *      In the case of the mailbox driver we will flush out and close the
 *      read buffer.
 *
 *  @retval
 *      Not applicable
 */
void MmwaveLink_unmaskHostIRQ(rlComIfHdl_t fd)
{
    Mailbox_readFlush((Mbox_Handle)fd);
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to register the Interrupt Handler.
 *      In the case of the Mailbox the driver does the interrupt registeration and
 *      so this function is a dummy stub.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static rlInt32_t MmwaveLink_registerInterruptHandler(rlUInt8_t deviceIndex, RL_P_EVENT_HANDLER pHandler, void* pValue)
{
    g_MailboxInterruptFunc = pHandler;
    return 0;
}


/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to handle asynchronous events
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static void MmwaveLink_asyncEventHandler(uint8_t devIndex, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);
    uint16_t msgId   = RL_GET_MSGID_FROM_SBID(sbId);

    /* Process the received message: */
    switch (msgId)
    {
        case RL_RF_ASYNC_EVENT_MSG:
        {
            /* Received Asychronous Message: */
            switch (asyncSB)
            {
                case RL_RF_AE_CPUFAULT_SB:
                {
                    System_printf ("Debug: CPU Fault has been detected\n");
                    GestDemo_Assert(0);
                    break;
                }
                case RL_RF_AE_ESMFAULT_SB:
                {
                    System_printf ("Debug: ESM Fault has been detected\n");
                    GestDemo_Assert(0);
                    break;
                }
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    gInitTimeCalibStatus = ((rlRfInitComplete_t*)payload)->calibStatus;
                    if(gInitTimeCalibStatus != 0U)
                    {
                        System_printf ("Debug: Init time calibration status [0x%x] \n", gInitTimeCalibStatus);
                    }
                    else
                    {
                        System_printf ("Error: All Init time calibrations Failed:\n");
                    }
                    break;
                }
                case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
                {
                    gFrameStartStatus = 1U;
                    break;
                }
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    System_printf ("Debug: Monitoring FAIL Report received \n");
                    break;
                }
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                {
                    gRunTimeCalibStatus = ((rlRfRunTimeCalibReport_t*)payload)->calibErrorFlag;
                    if(gRunTimeCalibStatus == 0U)
                    {
                        monFailRepCheck[22]++;
                        System_printf ("Error: All Run time calibrations Failed:\n");
                    }
                    break;
                }
                case RL_RF_AE_FRAME_END_SB:
                {
                    gFrameStartStatus = 0U;
                    System_printf ("Debug:  Frame Stop Async Event \n");
                    break;
                }
                case RL_RF_AE_ANALOG_FAULT_SB:
                {
                    System_printf ("Debug:  Analog Fault Async Event \n");
                    break;
                }
                default:
                {
                    System_printf ("Error: Asynchronous Event SB Id %d not handled with msg ID [0x%x] \n", asyncSB,msgId);
                    break;
                }
            }
            break;
        }
        default:
        {
            System_printf ("Error: Asynchronous message %d is NOT handled\n", msgId);
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to call the function in a different context
 *      This function is invoked from the Interrupt context.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static rlInt32_t MmwaveLink_spawn (RL_P_OSI_SPAWN_ENTRY pEntry, const void* pValue, uint32_t flags)
{
    /* Record the function which is to be spawned. */
    if (gMmwaveLinkMCB.spawnFxn != NULL)
        gMmwaveLinkMCB.spawnOverrun++;

    /* Record the entry to be spawned. */
    gMmwaveLinkMCB.spawnFxn = pEntry;

    /* Post the semaphore and wake up the link management task */
    Semaphore_post (gMmwaveLinkMCB.linkSemaphore);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to compute the CRC.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static rlInt32_t MmwaveLink_computeCRC(rlUInt8_t* data, rlUInt32_t dataLen, rlUInt8_t crcType, rlUInt8_t* crc)
{
    CRC_SigGenCfg   signGenCfg;
    int32_t         errCode;
    uint64_t        signature;
    uint32_t        index;
    uint8_t*        ptrSignature;
    uint8_t         crcLength;

    /* Initialize the signature generation configuration */
    memset ((void *)&signGenCfg, 0, sizeof(CRC_SigGenCfg));

    /* Allocate a unique transaction id: */
    if (CRC_getTransactionId (gMmwaveLinkMCB.crcHandle, &signGenCfg.transactionId, &errCode) < 0)
    {
        System_printf ("Error: CRC Driver Get transaction id failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Populate the signature generation configuration: */
    signGenCfg.ptrData = (uint8_t*)data;
    signGenCfg.dataLen = dataLen;

    /* Compute the signature for the specific data on Channel-1 */
    if (CRC_computeSignature (gMmwaveLinkMCB.crcHandle, &signGenCfg, &errCode) < 0)
    {
        System_printf ("Error: CRC Driver compute signature failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Get the Signature for Channel */
    if (CRC_getSignature (gMmwaveLinkMCB.crcHandle, signGenCfg.transactionId, &signature, &errCode) < 0)
    {
        System_printf ("Error: CRC Driver get signature failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Get the pointer to the CRC Signature: */
    ptrSignature = (uint8_t*)&signature;

    /* Determine the length of the CRC: */
    switch (crcType)
    {
        case RL_CRC_TYPE_16BIT_CCITT:
        {
            crcLength = 2;
            break;
        }
        case RL_CRC_TYPE_32BIT:
        {
            crcLength = 4;
            break;
        }
        case RL_CRC_TYPE_64BIT_ISO:
        {
            crcLength = 8;
            break;
        }
        default:
        {
            System_printf ("Error: Unknown CRC Type passed from mmWave Link: %d\n", crcType);
            return -1;
        }
    }

    /* Copy the CRC signature into CRC output array */
    for(index = 0U; index < crcLength; index++)
        *(crc + index) = *(ptrSignature + index);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the task which handles the mmWave Link communication
 *      messages between the BSS and MSS.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwaveLink_mmwaveLinkMgmtTask (UArg arg0, UArg arg1)
{
    RL_P_OSI_SPAWN_ENTRY    spawnFxn;
    uintptr_t               key;
    Semaphore_Params        semParams;

    /* Debug Message: */
    System_printf("Debug: Launched the mmwaveLink Management Task\n");

    /* Initialize the mmWave Link Semaphore: */
    Semaphore_Params_init(&semParams);
    semParams.mode  = Semaphore_Mode_BINARY;
    gMmwaveLinkMCB.linkSemaphore = Semaphore_create(0, &semParams, NULL);

    /* Execute forever: */
    while (1)
    {
        /* Pending on the link semaphore */
        Semaphore_pend (gMmwaveLinkMCB.linkSemaphore, BIOS_WAIT_FOREVER);

        /* Critical Section: We record the spawn function which is to be executed */
        key = Hwi_disable();
        spawnFxn = gMmwaveLinkMCB.spawnFxn;
        gMmwaveLinkMCB.spawnFxn = NULL;
        Hwi_restore (key);

        /* Execute the spawn function: */
        spawnFxn (NULL);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to get and display the version information
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_CalMonTimeUnitCfg (void)
{
    int32_t                     retVal;
    rlRfCalMonTimeUntConf_t     timeCfg;
    rlRfCalMonFreqLimitConf_t   freqLimit;
    rlRfInitCalConf_t           rfInitCalib;
    
    
    /* Initialize the configurations: */
    memset ((void *)&freqLimit,   0, sizeof(rlRfCalMonFreqLimitConf_t));
    memset ((void *)&rfInitCalib, 0, sizeof(rlRfInitCalConf_t));
    memset ((void *)&timeCfg,     0, sizeof(rlRfCalMonTimeUntConf_t));

    /****************************************************************************************
     * Setup the RF Calibration Time unit:
     * - Periodicity is set to 1 Frame
     ****************************************************************************************/
    timeCfg.calibMonTimeUnit = 1U;
    retVal = rlRfSetCalMonTimeUnitConfig(RL_DEVICE_MAP_INTERNAL_BSS, &timeCfg);
    
        /****************************************************************************************
     * Setup the RF Calibration Frequency limit:
     ****************************************************************************************/
    freqLimit.freqLimitLow  = FREQUENCY_LIMIT_LOW;
    freqLimit.freqLimitHigh = FREQUENCY_LIMIT_HIGH;
    retVal = rlRfSetCalMonFreqLimitConfig(RL_DEVICE_MAP_INTERNAL_BSS, &freqLimit);
    
    /****************************************************************************************
     * Enable calibrations:
     ****************************************************************************************/
    rfInitCalib.calibEnMask = ((1<< 4U)     |   /* LODIST calibration            */
                               (1<< 5U)     |   /* RX ADC DC offset calibration  */
                               (1<< 6U)     |   /* HPF cutoff calibration        */
                               (1<< 7U)     |   /* LPF cutoff calibration        */
                               (1<< 8U)     |   /* Peak detector calibration     */
                               (1<< 9U)     |   /* TX Power calibration          */
                               (1<<10U));       /* RX gain calibration           */
    retVal = rlRfInitCalibConfig(RL_DEVICE_MAP_INTERNAL_BSS, &rfInitCalib);
    
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get and display the version information
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_getVersion (void)
{
    rlVersion_t verArgs;
    int32_t     retVal;
    /* currently patch binaries are available for AWR16 ES2.0 & IWR16 ES2.0 only*/
    int8_t rfPatchBuildVer, rfPatchDebugVer;

    /* Get the version string: */
    retVal = rlDeviceGetVersion(RL_DEVICE_MAP_INTERNAL_BSS, &verArgs);
    if (retVal != 0)
    {
        System_printf ("Error: Unable to get the device version from mmWave link [Error %d]\n", retVal);
        return retVal;
    }

    /* Display the version information */
    System_printf ("RF H/W Version    : %02d.%02d\n",
                    verArgs.rf.hwMajor, verArgs.rf.hwMinor);
    System_printf ("RF F/W Version    : %02d.%02d.%02d.%02d.%02d.%02d.%02d\n",
                    verArgs.rf.fwMajor, verArgs.rf.fwMinor, verArgs.rf.fwBuild, verArgs.rf.fwDebug,
                    verArgs.rf.fwYear, verArgs.rf.fwMonth, verArgs.rf.fwDay);
    rfPatchDebugVer = ((verArgs.rf.patchBuildDebug) & 0x0F);
    rfPatchBuildVer = (((verArgs.rf.patchBuildDebug) & 0xF0) >> 4);
    
    System_printf ("RF F/W Patch Version : %02d.%02d.%02d.%02d.%02d.%02d.%02d\n",
                    verArgs.rf.patchMajor, verArgs.rf.patchMinor, rfPatchBuildVer, rfPatchDebugVer,
                    verArgs.rf.patchYear, verArgs.rf.patchMonth, verArgs.rf.patchDay);                    
    System_printf ("mmWaveLink Version: %02d.%02d.%02d.%02d\n",
                    verArgs.mmWaveLink.major, verArgs.mmWaveLink.minor,
                    verArgs.mmWaveLink.build, verArgs.mmWaveLink.debug);
    return retVal;
}

int32_t MmwaveLink_getRfBootupStatus (void)
{
    int32_t         retVal;
    rlRfBootStatusCfg_t statusCfg = {0};
    
    /* Set channel configuration */
    retVal = rlGetRfBootupStatus(RL_DEVICE_MAP_INTERNAL_BSS, &statusCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        System_printf("Error: rlGetRfBootupStatus retVal=%d\n", retVal);
        return retVal;
    }
    
    System_printf("Debug: Finished get radarSS bootup status to BSS\n");

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set/send mmWave Link Channel Configuration to the BSS.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setChannelConfig (void)
{
    int32_t         retVal;

    /* Set channel configuration */
    retVal = rlSetChannelConfig(RL_DEVICE_MAP_INTERNAL_BSS,(rlChanCfg_t*)&gMmwaveLinkCfg.channelCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        System_printf("Error: setChannelConfig retVal=%d\n", retVal);
        return retVal;
    }
    
    System_printf("Debug: Finished set channel configurations to BSS\n");

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set/send the Adc out configuration to the BSS.
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setAdcOutConfig (void)
{
    int32_t         retVal;

    retVal = rlSetAdcOutConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlAdcOutCfg_t*)&gMmwaveLinkCfg.adcOutCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        System_printf("Error: setAdcOutConfig retVal=%d\n", retVal);
        return retVal;
    }
    
    System_printf("Debug: Finished setAdcOutConfig to BSS\n");
    
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set low power mode.
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setLowPowerModeConfig (void)
{
    int32_t         retVal;

    retVal = rlSetLowPowerModeConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlLowPowerModeCfg_t*)&gMmwaveLinkCfg.lowPwrCfg);

    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        System_printf("Error: setLowPowerMode retVal=%d\n", retVal);
        return retVal;
    }
    
    System_printf("Debug: Finished setLowPowerMode\n");
    
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test Set HSI clock API.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setHsiClk (void)
{
    int32_t     retVal;
    rlDevHsiClk_t deviceHsiClk = {0};
    /* Setup the HSI Clock as per the Radar Interface Document:
     * - This is set to 600Mhz DDR Mode */
    deviceHsiClk.hsiClk = 0x9;
    
    /* Set HSI clock */
    retVal = rlDeviceSetHsiClk(RL_DEVICE_MAP_INTERNAL_BSS, (rlDevHsiClk_t*)&deviceHsiClk);
    if (retVal != 0)
    {
        System_printf ("Error: Unable to set HSI clock [Error %d]\n", retVal);
        return retVal;
    }
    System_printf("Debug: Set HSI clock successfully\n");

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test RF Init/Calibration API.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_rfCalibration (void)
{
    int32_t     retVal;

    retVal = rlRfInit(RL_DEVICE_MAP_INTERNAL_BSS);
    if (retVal != 0)
    {
        System_printf ("Error: Unable to start RF [Error %d]\n", retVal);
        return retVal;
    }
    while(gInitTimeCalibStatus == 0U)
    {
        /* Sleep and poll again: */
        Task_sleep(1);
    }
    gInitTimeCalibStatus = 0U;
    System_printf("Debug: RF start successfully\n");

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set Profile configuration.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setProfileConfig (uint8_t numOfProfileCfg)
{
    int32_t         retVal;
   
    retVal = rlSetProfileConfig(RL_DEVICE_MAP_INTERNAL_BSS, numOfProfileCfg, (rlProfileCfg_t*)&gMmwaveLinkCfg.profileCfg[0U]);

    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
    System_printf("Error: rlSetProfileConfig retVal=%d\n", retVal);
        return retVal;
    }
    
    System_printf("Debug: Finished rlSetProfileConfig\n");
    
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set Chirp configuration.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setChirpConfig (uint8_t numOfChirpCfg)
{
    int32_t         retVal;

    retVal = rlSetChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, numOfChirpCfg, (rlChirpCfg_t*)&gMmwaveLinkCfg.chirpCfg[0]);
           
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        System_printf("Error: rlSetChirpConfig retVal=%d\n", retVal);
        return retVal;
    }
    
    System_printf("Debug: Finished rlSetChirpConfig\n");
    
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set Frame configuration.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setFrameConfig (uint8_t frameCfgIdx)
{
    int32_t         retVal;
    
    retVal = rlSetFrameConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlFrameCfg_t*)&gMmwaveLinkCfg.frameCfg[frameCfgIdx]);
           
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        System_printf("Error: rlSetFrameConfig retVal=%d\n", retVal);
        return retVal;
    }
    
    System_printf("Debug: Finished rlSetFrameConfig\n");
    
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test start sensor API.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_sensorStart (void)
{
    int32_t     retVal;

    /* Trigger the frame */
    retVal = rlSensorStart(RL_DEVICE_MAP_INTERNAL_BSS);
    if (retVal != 0)
    {
        System_printf ("Error: Unable to start Sensor [Error %d]\n", retVal);
        return retVal;
    }
    #if 0
    /* if frame start async event is enable by rlRfSetDeviceCfg API,
       then wait for Frame start async event */
    if ((rfDevCfg.aeControl & 0x1) == 0x0)
    {
        while(gFrameStartStatus == 0U)
        {
            /* Sleep and poll again: */
            Task_sleep(1);
        }
    }
    #endif
    return retVal;
}

int32_t MmwaveLink_waitFrameEndAE(void)
{
    int32_t retVal = 0;
    /* if frame stop async event is enable by rlRfSetDeviceCfg API,
    then wait for Frame stop async event */
    if ((rfDevCfg.aeControl & 0x2) == 0x0)
    {
        while(gFrameStartStatus == 1U)
        {
            /* Sleep and poll again: */
            Task_sleep(1);
        }
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test stop sensor API.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_sensorStop (void)
{
    int32_t     retVal;

    /* Stop the frame */
    retVal = rlSensorStop(RL_DEVICE_MAP_INTERNAL_BSS);
    if (retVal != 0)
    {
        if(retVal == RL_RET_CODE_FRAME_ALREADY_ENDED)
        {
            System_printf ("Debug: Frames are already stopped  [%d]\n", retVal);
            return 0;
        }
        System_printf ("Error: Unable to stop Sensor [Error %d]\n", retVal);
        return retVal;
    }
    
#if 0
    /* if frame stop async event is enable by rlRfSetDeviceCfg API,
        then wait for Frame stop async event */
    if ((rfDevCfg.aeControl & 0x2) == 0x0)
    {
        while(gFrameStartStatus == 1U)
        {
            /* Sleep and poll again: */
            Task_sleep(1);
        }
    }
#endif
    System_printf("Debug: Sensor stop successfully\n");

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set run time calibration.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRunTimeCalibConfig (void)
{
    int32_t         retVal;

    retVal = rlRfRunTimeCalibConfig(RL_DEVICE_MAP_INTERNAL_BSS, 
                                (rlRunTimeCalibConf_t*)&gMmwaveLinkCfg.runTimeCalib);

    if (retVal != 0)
    {
        System_printf ("Error: Unable to rlRfRunTimeCalibConfig [Error %d]\n", retVal);
        return retVal;
    }

    while(gRunTimeCalibStatus == 0U)
    {
        /* Sleep and poll again: */
        Task_sleep(1);
    }
    gRunTimeCalibStatus = 0U;

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to Configure dynamic power saving feature
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_rfDynamicPowerSave (void)
{
    int32_t         retVal;
    
    retVal = rlRfDynamicPowerSave(RL_DEVICE_MAP_INTERNAL_BSS, (rlDynPwrSave_t*)&gMmwaveLinkCfg.lowPwrCfg);

    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
    System_printf("Error: rlRfDynamicPowerSave retVal=%d\n", retVal);
        return retVal;
    }
    
    System_printf("Debug: Finished rlRfDynamicPowerSave\n");
    
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to Configure asynchronous event direction for device
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfDevCfg (void)
{
    int32_t         retVal;

    retVal = rlRfSetDeviceCfg(RL_DEVICE_MAP_INTERNAL_BSS, &gMmwaveLinkCfg.rfDevCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        System_printf("Error: rlRfSetDeviceCfg retVal=%d\n", retVal);
        return retVal;
    }
    
    System_printf("Debug: Finished rlRfSetDeviceCfg\n");
    
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is used to initialize and setup the mmWave link
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t MmwaveLink_initLink (rlUInt8_t deviceType, rlUInt8_t platform)
{
    Task_Params            taskParams;
    CRC_Config          crcCfg;
    rlClientCbs_t       RlApp_ClientCtx;
    int32_t             errCode;

    /* Initialize and populate the Mmwave Link MCB */
    memset ((void*)&gMmwaveLinkMCB, 0, sizeof(MmwaveLink_MCB));

    /*****************************************************************************
     * Start CRC driver:
     *****************************************************************************/

    /* Setup the default configuration: */
    CRC_initConfigParams(&crcCfg);

    /*******************************************************************************
     * This is the configuration for the 16bit CRC Type:
     *******************************************************************************/
    crcCfg.channel  = CRC_Channel_CH1;
    crcCfg.mode     = CRC_Operational_Mode_FULL_CPU;
    crcCfg.type     = CRC_Type_16BIT;
    crcCfg.bitSwap  = CRC_BitSwap_MSB;
    crcCfg.byteSwap = CRC_ByteSwap_ENABLED;
    crcCfg.dataLen  = CRC_DataLen_16_BIT;


    /* Open the CRC Driver */
    gMmwaveLinkMCB.crcHandle = CRC_open (&crcCfg, &errCode);
    if (gMmwaveLinkMCB.crcHandle == NULL)
    {
        System_printf ("Error: Unable to open the CRC Channel [Error Code %d]\n", errCode);
        return -1;
    }
    System_printf("Debug: CRC Channel %p has been opened successfully\n", gMmwaveLinkMCB.crcHandle);

    /*****************************************************************************
     * Launch the Mmwave Link Tasks:
     *****************************************************************************/

    /* Initialize and Launch the mmWave Link Management Task: */
    Task_Params_init(&taskParams);
    taskParams.priority = 5;
    Task_create(MmwaveLink_mmwaveLinkMgmtTask, &taskParams, NULL);

    /*****************************************************************************
     * Initialize the mmWave Link: We need to have the link management task
     * operational to be able to process the SPAWN function.
     *****************************************************************************/

    /* Reset the client context: */
    memset ((void *)&RlApp_ClientCtx, 0, sizeof(rlClientCbs_t));

    RlApp_ClientCtx.ackTimeout  = 1000U;

    /* Setup the crc Type in the mmWave link and synchronize this with the
     * created CRC Channel. */
    RlApp_ClientCtx.crcType = RL_CRC_TYPE_16BIT_CCITT;

    /* Setup the platform on which the mmWave Link executes */
    RlApp_ClientCtx.platform  = platform;
    RlApp_ClientCtx.arDevType = deviceType;

    /* Initialize the Communication Interface API: */
    RlApp_ClientCtx.comIfCb.rlComIfOpen     = MmwaveLink_mboxOpen;
    RlApp_ClientCtx.comIfCb.rlComIfClose    = MmwaveLink_mboxClose;
    RlApp_ClientCtx.comIfCb.rlComIfRead     = MmwaveLink_mboxRead;
    RlApp_ClientCtx.comIfCb.rlComIfWrite    = MmwaveLink_mboxWrite;

    /* Initialize OSI Mutex Interface */
    RlApp_ClientCtx.osiCb.mutex.rlOsiMutexCreate = Osal_mutexCreate;
    RlApp_ClientCtx.osiCb.mutex.rlOsiMutexLock   = Osal_mutexLock;
    RlApp_ClientCtx.osiCb.mutex.rlOsiMutexUnLock = Osal_mutexUnlock;
    RlApp_ClientCtx.osiCb.mutex.rlOsiMutexDelete = Osal_mutexDelete;

    /* Initialize OSI Semaphore Interface */
    RlApp_ClientCtx.osiCb.sem.rlOsiSemCreate    = Osal_semCreate;
    RlApp_ClientCtx.osiCb.sem.rlOsiSemWait      = Osal_semWait;
    RlApp_ClientCtx.osiCb.sem.rlOsiSemSignal    = Osal_semSignal;
    RlApp_ClientCtx.osiCb.sem.rlOsiSemDelete    = Osal_semDelete;

    /* Initialize OSI Queue Interface */
    RlApp_ClientCtx.osiCb.queue.rlOsiSpawn      = MmwaveLink_spawn;

    /* Initialize OSI Timer Interface */
    RlApp_ClientCtx.timerCb.rlDelay             = NULL;

    /* Initialize the CRC Interface */
    RlApp_ClientCtx.crcCb.rlComputeCRC          = MmwaveLink_computeCRC;

    /* Initialize Device Control Interface */
    RlApp_ClientCtx.devCtrlCb.rlDeviceDisable            = MmwaveLink_disableDevice;
    RlApp_ClientCtx.devCtrlCb.rlDeviceEnable             = MmwaveLink_enableDevice;
    RlApp_ClientCtx.devCtrlCb.rlDeviceMaskHostIrq        = MmwaveLink_maskHostIRQ;
    RlApp_ClientCtx.devCtrlCb.rlDeviceUnMaskHostIrq      = MmwaveLink_unmaskHostIRQ;
    RlApp_ClientCtx.devCtrlCb.rlRegisterInterruptHandler = MmwaveLink_registerInterruptHandler;

    /* Initialize the Asynchronous Event Handler: */
    RlApp_ClientCtx.eventCb.rlAsyncEvent    = MmwaveLink_asyncEventHandler;

    /* Power on the Device: */
    if (rlDevicePowerOn(1U, RlApp_ClientCtx) != 0)
    {
        System_printf("Error: Power on request to the BSS failed\n");
        return -1;
    }
    System_printf("Debug: Power on request successfully passed to the BSS\n");

    /* initialize all the configurations to default value */
    MmwaveLink_FrameCfgInitParams();
    MmwaveLink_ProfileCfgInitParams();
    MmwaveLink_ChirpCfgInitParams();
    MmwaveLink_LowPowerModeInitParams();
    MmwaveLink_ChannelCfgInitParams();
    MmwaveLink_ADCOutCfgInitParams();
    MmwaveLink_RfDevCfgInitParams();
    MmwaveLink_RunTimeCalibInitParams();
    return 0;
}

int32_t MmwaveLink_configRadarSS(void)
{
    int32_t retVal;
    
    
    /* Get RadarSS version */
    if((retVal = MmwaveLink_getVersion()) < 0)
    {
        return retVal;
    }
    
    if((retVal = MmwaveLink_CalMonTimeUnitCfg()) < 0)
    {
        System_printf ("Calib Mon Unit Cfg failed! [%d]", retVal);
        return retVal;
    }
    /* Set Rx/Tx Channel config */
    if((retVal = MmwaveLink_setChannelConfig()) < 0)
    {
        System_printf ("Set Channel config failed! [%d]", retVal);
        return retVal;
    }
    /* Set mmWave Link ADC Out Configuration to the BSS */
    if((retVal = MmwaveLink_setAdcOutConfig()) < 0)
    {
        System_printf ("Set ADC Out config failed! [%d]", retVal);
        return retVal;
    }
    /* Set mmWave Link low power mode config*/
    if((retVal = MmwaveLink_setLowPowerModeConfig()) < 0)
    {
        System_printf ("Set Low power config failed! [%d]", retVal);
        return retVal;
    }
    
    /* mmWave Link set Async event configuration */
    if((retVal = MmwaveLink_setRfDevCfg()) < 0)
    {
        System_printf ("Set Async event config failed! [%d]", retVal);
        return retVal;
    }
    
    /* Set device HSI clock */
    if((retVal = MmwaveLink_setHsiClk()) < 0)
    {
        System_printf ("Set HSI Clock config failed! [%d]", retVal);
        return retVal;
    }
    
    /* mmWave Link RF Init/Calibration */
    if((retVal = MmwaveLink_rfCalibration()) < 0)
    {
        System_printf ("mmWaveLink RF-Init calib failed! [%d]", retVal);
        return retVal;
    }
    
    /* mmWave Link set profile configuration */
    if((retVal = MmwaveLink_setProfileConfig(2)) < 0)
    {
        System_printf ("mmWaveLink Profile config failed! [%d]", retVal);
        return retVal;
    }

    /* mmWave Link set Chirp configuration */
    if((retVal = MmwaveLink_setChirpConfig(MAX_CHIRP_COUNT)) < 0)
    {
        System_printf ("mmWaveLink Chirp config failed! [%d]", retVal);
        return retVal;
    }

    return retVal;
}

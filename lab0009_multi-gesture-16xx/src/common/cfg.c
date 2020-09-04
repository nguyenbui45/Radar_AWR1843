/*
 *   @file  cfg.c
 *
 *   @brief
 *      The file configures the SRR TI Design and demonstrates the 
 *      use of the minimal mode. 
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

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/uart/UART.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/cli/cli.h>
#include "srr_config_consts.h"

/**************************************************************************
 ****************************** CFG Functions *****************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function initializes the frame configuration with the default
 *      parameters.
 *
 *  @param[out] ptrAdvFrameCfg
 *      Pointer to the adavance frame configuration
 *
 *  @retval
 *      Not applicable
 */
void Cfg_AdvFrameCfgInitParams (rlAdvFrameCfg_t* ptrAdvFrameCfg)
{
    uint8_t numOfSubFrame = 0U;

    /* Initialize the configuration: */
    memset ((void*)ptrAdvFrameCfg, 0, sizeof(rlAdvFrameCfg_t));

    /* Populate the default configuration: */
    ptrAdvFrameCfg->frameSeq.forceProfile  = 0;// 1: force Profile,  0: Don't force profile
    ptrAdvFrameCfg->frameSeq.numFrames     = 0;//infinite
    ptrAdvFrameCfg->frameSeq.triggerSelect = 1;//SW Trigger
    ptrAdvFrameCfg->frameSeq.frameTrigDelay= 0;

#if NUM_SUBFRAMES == 2

    /* The low resolution 80m subframe */
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].forceProfileIdx    = 0;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numLoops           = SUBFRAME_SRR_LOOP_COUNT;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfBurst         = 1;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfBurstLoops    = 1;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].chirpStartIdxOffset= 0;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfChirps        = SUBFRAME_SRR_CHIRP_END_IDX - SUBFRAME_SRR_CHIRP_START_IDX + 1;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].chirpStartIdx      = SUBFRAME_SRR_CHIRP_START_IDX;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].subFramePeriodicity= SUBFRAME_SRR_PERIODICITY_VAL;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].burstPeriodicity   = SUBFRAME_SRR_PERIODICITY_VAL;
    ptrAdvFrameCfg->frameData.subframeDataCfg[0].numAdcSamples = PROFILE_SRR_ADC_SAMPLE_VAL*2;
    ptrAdvFrameCfg->frameData.subframeDataCfg[0].totalChirps   = SUBFRAME_SRR_NUM_CHIRPS_TOTAL;
    ptrAdvFrameCfg->frameData.subframeDataCfg[0].numChirpsInDataPacket = 1;
    numOfSubFrame++;

    /* The high resolution 20m subframe */
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].forceProfileIdx    = 0;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].numLoops           = SUBFRAME_USRR_LOOP_COUNT;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].numOfBurst         = 1;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].numOfBurstLoops    = 1;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].chirpStartIdxOffset= 0;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].numOfChirps        = SUBFRAME_USRR_CHIRP_END_IDX - SUBFRAME_USRR_CHIRP_START_IDX + 1;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].chirpStartIdx      = SUBFRAME_USRR_CHIRP_START_IDX;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].subFramePeriodicity= SUBFRAME_USRR_PERIODICITY_VAL;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].burstPeriodicity   = SUBFRAME_USRR_PERIODICITY_VAL;
    ptrAdvFrameCfg->frameData.subframeDataCfg[1].numAdcSamples = PROFILE_USRR_ADC_SAMPLE_VAL*2;
    ptrAdvFrameCfg->frameData.subframeDataCfg[1].totalChirps   = SUBFRAME_USRR_NUM_CHIRPS_TOTAL;
    ptrAdvFrameCfg->frameData.subframeDataCfg[1].numChirpsInDataPacket = 1;
    numOfSubFrame++;
#else
	#ifdef SUBFRAME_CONF_SRR
	    /* The low resolution 80m subframe */
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].forceProfileIdx    = 0;
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numLoops           = SUBFRAME_SRR_LOOP_COUNT;
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfBurst         = 1;
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfBurstLoops    = 1;
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].chirpStartIdxOffset= 0;
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfChirps        = SUBFRAME_SRR_CHIRP_END_IDX - SUBFRAME_SRR_CHIRP_START_IDX + 1;
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].chirpStartIdx      = SUBFRAME_SRR_CHIRP_START_IDX;
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].subFramePeriodicity= SUBFRAME_SRR_PERIODICITY_VAL;
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].burstPeriodicity   = SUBFRAME_SRR_PERIODICITY_VAL;
	    ptrAdvFrameCfg->frameData.subframeDataCfg[0].numAdcSamples = PROFILE_SRR_ADC_SAMPLE_VAL*2;
	    ptrAdvFrameCfg->frameData.subframeDataCfg[0].totalChirps   = SUBFRAME_SRR_NUM_CHIRPS_TOTAL;
	    ptrAdvFrameCfg->frameData.subframeDataCfg[0].numChirpsInDataPacket = 1;
	    numOfSubFrame++;
    #else
	#ifdef SUBFRAME_CONF_USRR
	    /* The high resolution 20m subframe */
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].forceProfileIdx    = 0;
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numLoops           = SUBFRAME_USRR_LOOP_COUNT;
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfBurst         = 1;
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfBurstLoops    = 1;
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].chirpStartIdxOffset= 0;
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfChirps        = SUBFRAME_USRR_CHIRP_END_IDX - SUBFRAME_USRR_CHIRP_START_IDX + 1;
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].chirpStartIdx      = SUBFRAME_USRR_CHIRP_START_IDX;
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].subFramePeriodicity= SUBFRAME_USRR_PERIODICITY_VAL;
	    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].burstPeriodicity   = SUBFRAME_USRR_PERIODICITY_VAL;
	    ptrAdvFrameCfg->frameData.subframeDataCfg[0].numAdcSamples = PROFILE_USRR_ADC_SAMPLE_VAL*2;
	    ptrAdvFrameCfg->frameData.subframeDataCfg[0].totalChirps   = SUBFRAME_USRR_NUM_CHIRPS_TOTAL;
	    ptrAdvFrameCfg->frameData.subframeDataCfg[0].numChirpsInDataPacket = 1;
	    numOfSubFrame++;
	#endif
	#endif
#endif
    ptrAdvFrameCfg->frameSeq.numOfSubFrames = numOfSubFrame;
    ptrAdvFrameCfg->frameData.numSubFrames  = numOfSubFrame;

    return;
}

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
void Cfg_FrameCfgInitParams (rlFrameCfg_t* ptrFrameCfg)
{
    /* Initialize the configuration: */
    memset ((void*)ptrFrameCfg, 0, sizeof(rlFrameCfg_t));

    /* Populate the default configuration: */
    ptrFrameCfg->chirpEndIdx        = FRAME_CHIRP_END_IDX;
    ptrFrameCfg->chirpStartIdx      = FRAME_CHIRP_START_IDX;
    ptrFrameCfg->numFrames          = FRAME_COUNT_VAL;
    ptrFrameCfg->numLoops           = FRAME_LOOP_COUNT;
    ptrFrameCfg->triggerSelect      = RL_FRAMESTRT_SYNCIN_TRIGGER;
    ptrFrameCfg->framePeriodicity   = FRAME_PERIODICITY_VAL;
    ptrFrameCfg->frameTriggerDelay  = FRAME_TRIGGER_DELAY_VAL;
    ptrFrameCfg->numAdcSamples      = FRAME_NUM_REAL_ADC_SAMPLES;
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
void Cfg_ProfileCfgInitParams (uint8_t profileNum, rlProfileCfg_t* ptrProfileCfg)
{
    /* Initialize the configuration: */
    memset ((void*)ptrProfileCfg, 0, sizeof(rlProfileCfg_t));

    if (profileNum == 0U)
    {
        /* Populate the default configuration for profile 0  */
        ptrProfileCfg->profileId             = PROFILE_SRR_PROFILE_ID;
        ptrProfileCfg->startFreqConst        = PROFILE_SRR_START_FREQ_VAL;
        ptrProfileCfg->idleTimeConst         = PROFILE_SRR_IDLE_TIME_VAL;
        ptrProfileCfg->adcStartTimeConst     = PROFILE_SRR_ADC_START_TIME_VAL;
        ptrProfileCfg->rampEndTime           = PROFILE_SRR_RAMP_END_TIME_VAL;
        ptrProfileCfg->txOutPowerBackoffCode = PROFILE_SRR_TXOUT_POWER_BACKOFF;
        ptrProfileCfg->txPhaseShifter        = PROFILE_SRR_TXPHASESHIFTER_VAL;
        ptrProfileCfg->freqSlopeConst        = PROFILE_SRR_FREQ_SLOPE_VAL;
        ptrProfileCfg->txStartTime           = PROFILE_SRR_TX_START_TIME_VAL;
        ptrProfileCfg->numAdcSamples         = PROFILE_SRR_ADC_SAMPLE_VAL;
        ptrProfileCfg->digOutSampleRate      = PROFILE_SRR_DIGOUT_SAMPLERATE_VAL;
        ptrProfileCfg->hpfCornerFreq1        = PROFILE_SRR_HPFCORNER_FREQ1_VAL;
        ptrProfileCfg->hpfCornerFreq2        = PROFILE_SRR_HPFCORNER_FREQ2_VAL;
        ptrProfileCfg->rxGain                = PROFILE_SRR_RX_GAIN_VAL;
    }
    else
    {
        /* Populate the default configuration for profile 1  */
        ptrProfileCfg->profileId             = PROFILE_USRR_PROFILE_ID;
        ptrProfileCfg->startFreqConst        = PROFILE_USRR_START_FREQ_VAL;
        ptrProfileCfg->idleTimeConst         = PROFILE_USRR_IDLE_TIME_VAL;
        ptrProfileCfg->adcStartTimeConst     = PROFILE_USRR_ADC_START_TIME_VAL;
        ptrProfileCfg->rampEndTime           = PROFILE_USRR_RAMP_END_TIME_VAL;
        ptrProfileCfg->txOutPowerBackoffCode = PROFILE_USRR_TXOUT_POWER_BACKOFF;
        ptrProfileCfg->txPhaseShifter        = PROFILE_USRR_TXPHASESHIFTER_VAL;
        ptrProfileCfg->freqSlopeConst        = PROFILE_USRR_FREQ_SLOPE_VAL;
        ptrProfileCfg->txStartTime           = PROFILE_USRR_TX_START_TIME_VAL;
        ptrProfileCfg->numAdcSamples         = PROFILE_USRR_ADC_SAMPLE_VAL;
        ptrProfileCfg->digOutSampleRate      = PROFILE_USRR_DIGOUT_SAMPLERATE_VAL;
        ptrProfileCfg->hpfCornerFreq1        = PROFILE_USRR_HPFCORNER_FREQ1_VAL;
        ptrProfileCfg->hpfCornerFreq2        = PROFILE_USRR_HPFCORNER_FREQ2_VAL;
        ptrProfileCfg->rxGain                = PROFILE_USRR_RX_GAIN_VAL;
    }
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
void Cfg_ChirpCfgInitParams (uint8_t chirpNum, rlChirpCfg_t* ptrChirpCfg)
{
    /* Initialize the configuration: */
    memset ((void*)ptrChirpCfg, 0, sizeof(rlChirpCfg_t));

    if (chirpNum == 0U)
    {
        /* Populate the default configuration for chirp 0. 
         *  - SRR Fast Chirp for max-velocity enhancement. */
        ptrChirpCfg->profileId       = CHIRP_SRR_0_PROFILE_ID;
        ptrChirpCfg->adcStartTimeVar = CHIRP_SRR_0_ADC_START_TIME_VAL;
        ptrChirpCfg->chirpEndIdx     = CHIRP_SRR_0_END_INDEX;
        ptrChirpCfg->chirpStartIdx   = CHIRP_SRR_0_START_INDEX;
        ptrChirpCfg->idleTimeVar     = CHIRP_SRR_0_IDLE_TIME_VAL;
        ptrChirpCfg->txEnable        = CHIRP_SRR_0_TX_CHANNEL;
        ptrChirpCfg->startFreqVar    = CHIRP_SRR_0_START_FREQ_VAL;
        ptrChirpCfg->freqSlopeVar    = CHIRP_SRR_0_FREQ_SLOPE_VAL;
    }
    else if (chirpNum == 1U)
    {
        /* Populate the default configuration for chirp 2 
         *  - SRR Slow Chirp for max-velocity enhancement. */
        ptrChirpCfg->profileId       = CHIRP_SRR_1_PROFILE_ID;
        ptrChirpCfg->adcStartTimeVar = CHIRP_SRR_1_ADC_START_TIME_VAL;
        ptrChirpCfg->chirpEndIdx     = CHIRP_SRR_1_END_INDEX;
        ptrChirpCfg->chirpStartIdx   = CHIRP_SRR_1_START_INDEX;
        ptrChirpCfg->idleTimeVar     = CHIRP_SRR_1_IDLE_TIME_VAL;
        ptrChirpCfg->txEnable        = CHIRP_SRR_1_TX_CHANNEL;
        ptrChirpCfg->startFreqVar    = CHIRP_SRR_1_START_FREQ_VAL;
        ptrChirpCfg->freqSlopeVar    = CHIRP_SRR_1_FREQ_SLOPE_VAL;
    }
    else if (chirpNum == 2U)
    {
        /* Populate the default configuration for chirp 3 
         *  - USRR Tx1 . */
        ptrChirpCfg->profileId       = CHIRP_USRR_0_PROFILE_ID;
        ptrChirpCfg->adcStartTimeVar = CHIRP_USRR_0_ADC_START_TIME_VAL;
        ptrChirpCfg->chirpEndIdx     = CHIRP_USRR_0_END_INDEX;
        ptrChirpCfg->chirpStartIdx   = CHIRP_USRR_0_START_INDEX;
        ptrChirpCfg->idleTimeVar     = CHIRP_USRR_0_IDLE_TIME_VAL;
        ptrChirpCfg->txEnable        = CHIRP_USRR_0_TX_CHANNEL;
        ptrChirpCfg->startFreqVar    = CHIRP_USRR_0_START_FREQ_VAL;
        ptrChirpCfg->freqSlopeVar    = CHIRP_USRR_0_FREQ_SLOPE_VAL;
    }
    else if (chirpNum == 3U)
    {
        /* Populate the default configuration for chirp 3 
         *  - USRR Tx2. */
        ptrChirpCfg->profileId       = CHIRP_USRR_1_PROFILE_ID;
        ptrChirpCfg->adcStartTimeVar = CHIRP_USRR_1_ADC_START_TIME_VAL;
        ptrChirpCfg->chirpEndIdx     = CHIRP_USRR_1_END_INDEX;
        ptrChirpCfg->chirpStartIdx   = CHIRP_USRR_1_START_INDEX;
        ptrChirpCfg->idleTimeVar     = CHIRP_USRR_1_IDLE_TIME_VAL;
        ptrChirpCfg->txEnable        = CHIRP_USRR_1_TX_CHANNEL;
        ptrChirpCfg->startFreqVar    = CHIRP_USRR_1_START_FREQ_VAL;
        ptrChirpCfg->freqSlopeVar    = CHIRP_USRR_1_FREQ_SLOPE_VAL;
        
    }
    
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
void Cfg_LowPowerModeInitParams (rlLowPowerModeCfg_t* ptrLowPowerMode)
{
    /* Initialize the configuration: */
    memset ((void*)ptrLowPowerMode, 0, sizeof(rlLowPowerModeCfg_t));

    /* Populate the low power configuration: */
    //TODO use the 'low power mode' i.e. LP_ADC_MODE_LOW_POWER in 
    // version 1.2. 
    ptrLowPowerMode->lpAdcMode     = LP_ADC_MODE_REGULAR;
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
void Cfg_ChannelCfgInitParams (rlChanCfg_t* ptrChannelCfg)
{
    /* Initialize the configuration: */
    memset ((void*)ptrChannelCfg, 0, sizeof(rlChanCfg_t));

    /* Populate the default configuration: */
    ptrChannelCfg->rxChannelEn = RX_CHANNEL_1_2_3_4_ENABLE;
    ptrChannelCfg->txChannelEn = TX_CHANNEL_1_2_ENABLE;
    ptrChannelCfg->cascading   = 0; /* Single Chip (no cascading)*/
    
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
void Cfg_ADCOutCfgInitParams (rlAdcOutCfg_t* ptrADCOutCfg)
{
    /* Initialize the configuration: */
    memset ((void*)ptrADCOutCfg, 0, sizeof(rlAdcOutCfg_t));

    /* Populate the default configuration: */
    ptrADCOutCfg->fmt.b2AdcBits   = ADC_BITS_16;
    ptrADCOutCfg->fmt.b2AdcOutFmt = ADC_FORMAT_COMPLEX;
    ptrADCOutCfg->fmt.b8FullScaleReducFctr = 0;
    
    return;
}

/*
 *   @file  cfg.c
 *
 *   @brief
 *      The file configures the MRR TI Design and demonstrates the 
 *      use of the minimal mode. 
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

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/uart/UART.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/cli/cli.h>
#include "mrr_config_consts.h"
#include "mrr_config.h"
/**************************************************************************
  Look-Up Table for the TX phase shifter codes for steering angles from 
  -60 to +60 degrees .
  The index0, index1, index2, index3.. is for -60 degree, -59 degree, -58 degree, 
  -57 degree... 
 **************************************************************************/
MmwDemo_Tx_phaseshift_Cfg tx_phaseShift_CodeLUT[]={
{-60 ,0, 9,17},
{-59 ,0, 9,18},
{-58 ,0,10,19},
{-57 ,0,10,21},
{-56 ,0,11,22},
{-55 ,0,12,23},
{-54 ,0,12,24},
{-53 ,0,13,26},
{-52 ,0,14,27},
{-51 ,0,14,28},
{-50 ,0,15,30},
{-49 ,0,16,31},
{-48 ,0,16,33},
{-47 ,0,17,34},
{-46 ,0,18,36},
{-45 ,0,19,37},
{-44 ,0,20,39},
{-43 ,0,20,41},
{-42 ,0,21,42},
{-41 ,0,22,44},
{-40 ,0,23,46},
{-39 ,0,24,47},
{-38 ,0,25,49},
{-37 ,0,25,51},
{-36 ,0,26,53},
{-35 ,0,27,54},
{-34 ,0,28,56},
{-33 ,0,29,58},
{-32 ,0,30,60},
{-31 ,0,31,62},
{-30 ,0,32, 0},
{-29 ,0,33, 2},
{-28 ,0,34, 4},
{-27 ,0,35, 6},
{-26 ,0,36, 8},
{-25 ,0,37,10},
{-24 ,0,38,12},
{-23 ,0,39,14},
{-22 ,0,40,16},
{-21 ,0,41,18},
{-20 ,0,42,20},
{-19 ,0,43,22},
{-18 ,0,44,24},
{-17 ,0,45,26},
{-16 ,0,46,28},
{-15 ,0,47,31},
{-14 ,0,48,33},
{-13 ,0,50,35},
{-12 ,0,51,37},
{-11 ,0,52,39},
{-10 ,0,53,41},
{-9  ,0,54,44},
{-8  ,0,55,46},
{-7  ,0,56,48},
{-6  ,0,57,50},
{-5  ,0,58,53},
{-4  ,0,59,55},
{-3  ,0,61,57},
{-2  ,0,62,59},
{-1  ,0,63,61},
{0   ,0, 0, 0},
{1   ,0, 1, 2},
{2   ,0, 2, 4},
{3   ,0, 3, 6},
{4   ,0, 4, 9},
{5   ,0, 6,11},
{6   ,0, 7,13},
{7   ,0, 8,15},
{8   ,0, 9,18},
{9   ,0,10,20},
{10  ,0,11,22},
{11  ,0,12,24},
{12  ,0,13,26},
{13  ,0,14,28},
{14  ,0,15,31},
{15  ,0,17,33},
{16  ,0,18,35},
{17  ,0,19,37},
{18  ,0,20,39},
{19  ,0,21,41},
{20  ,0,22,43},
{21  ,0,23,46},
{22  ,0,24,48},
{23  ,0,25,50},
{24  ,0,26,52},
{25  ,0,27,54},
{26  ,0,28,56},
{27  ,0,29,58},
{28  ,0,30,60},
{29  ,0,31,62},
{30  ,0,32, 0},
{31  ,0,33, 2},
{32  ,0,34, 4},
{33  ,0,35, 5},
{34  ,0,36, 7},
{35  ,0,37, 9},
{36  ,0,38,11},
{37  ,0,38,13},
{38  ,0,39,15},
{39  ,0,40,16},
{40  ,0,41,18},
{41  ,0,42,20},
{42  ,0,43,21},
{43  ,0,44,23},
{44  ,0,44,25},
{45  ,0,45,26},
{46  ,0,46,28},
{47  ,0,47,29},
{48  ,0,48,31},
{49  ,0,48,32},
{50  ,0,49,34},
{51  ,0,50,35},
{52  ,0,50,37},
{53  ,0,51,38},
{54  ,0,52,39},
{55  ,0,52,41},
{56  ,0,53,42},
{57  ,0,54,43},
{58  ,0,54,44},
{59  ,0,55,46},
{60  ,0,55,47} 
};

/*Max Steering Angle supported in degrees*/
#define MAX_STEERING_ANGLE      60U




/**
 *  @b Description
 *  @n
 *      The function to get the phase shifter codes for all the TX based on the steering *      angle input. 
        
        Algorithm to calculate the index to be used for phase shifter codes.
        The phase shifter codes are in the array and start from 0-120 for steering angle from
        -60 to +60. 
        Eg: If the user inputs a steering angle of -20 to +20 the index is calculated as below:
         Steering angle  : +/- 20 degree
         Phase shifter Code Start Index = MAX_STEERING_ANGLE - STEERING_ANGLE   
         Phase shifter Code End Index   = MAX_STEERING_ANGLE +   STEERING_ANGLE
   *
 *  @param[out] 
 *      
 *
 *  @retval
 *      uint8_t return : scan_count
 */
void Get_PhaseTxShifterCodeValue(int8_t steering_angle , uint8_t *tx1PhShCode, uint8_t *tx2PhShCode, uint8_t *tx3PhShCode)
{
    uint8_t index;
    uint8_t calib_params[3]  = {0, 0, 19 };
    int8_t l_tx1phCode, l_tx2phCode, l_tx3phCode;
    
    for(index = 0; index < sizeof(tx_phaseShift_CodeLUT);index++)
    {   
        if(steering_angle == tx_phaseShift_CodeLUT[index].steering_angle)
        {
            l_tx1phCode = -(tx_phaseShift_CodeLUT[index].tx1_phsh_deg);
            l_tx2phCode = -(tx_phaseShift_CodeLUT[index].tx2_phsh_deg);
            l_tx3phCode = -(tx_phaseShift_CodeLUT[index].tx3_phsh_deg);
            
            if(l_tx1phCode < 0)
                l_tx1phCode = l_tx1phCode + 64 ;
            if(l_tx2phCode < 0)
                l_tx2phCode = l_tx2phCode + 64 ;
            if(l_tx3phCode < 0)
                l_tx3phCode = l_tx3phCode + 64 ;
            
            /*Use the calibration parameters to do the compensation*/
            l_tx1phCode = (l_tx1phCode  + calib_params[0])%64;
            l_tx2phCode = (l_tx2phCode  + calib_params[1])%64;
            l_tx3phCode = (l_tx3phCode  + calib_params[2])%64;
            
            *tx1PhShCode = (uint8_t)l_tx1phCode;
            *tx2PhShCode = (uint8_t)l_tx2phCode;
            *tx3PhShCode = (uint8_t)l_tx3phCode;
         
            break;
        }
    }
    
    return;
    
}



/**
 *  @b Description
 *  @n
 *      The function to calculate the index for the phase shifter codes based on the 
 *      steering angle configured by the user. 
        
        Algorithm to calculate the index to be used for phase shifter codes.
        The phase shifter codes are in the array and start from 0-120 for steering angle from
        -60 to +60. 
        Eg: If the user inputs a steering angle of -20 to +20 the index is calculated as below:
         Steering angle  : +/- 20 degree
         Phase shifter Code Start Index = MAX_STEERING_ANGLE - STEERING_ANGLE   
         Phase shifter Code End Index   = MAX_STEERING_ANGLE +   STEERING_ANGLE
   *
 *  @param[out] 
 *      
 *
 *  @retval
 *      uint8_t return : scan_count
 */
uint8_t Cfg_PhaseTxShifterCodeIndex(uint8_t steering_angle, uint8_t *ptrstartIndex, uint8_t *ptrendIndex)
{
    if(steering_angle != 0){
        /*Value of the start index of the phase shifter code*/
        *ptrstartIndex = MAX_STEERING_ANGLE - steering_angle ; 
        
        /*Value of the end index of the phase shifter code*/
        *ptrendIndex = MAX_STEERING_ANGLE + steering_angle ; 
    }        
    else{
        /*Value of the start index of the phase shifter code*/
        *ptrstartIndex = 0U ; 
        
        /*Value of the end index of the phase shifter code*/
        *ptrendIndex = 0U ; 
    }
        return (*ptrendIndex  - *ptrstartIndex);
    
}

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
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numLoops           = SUBFRAME_MRR_LOOP_COUNT;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfBurst         = 1;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfBurstLoops    = 1;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].chirpStartIdxOffset= 0;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfChirps        = SUBFRAME_MRR_CHIRP_END_IDX - SUBFRAME_MRR_CHIRP_START_IDX + 1;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].chirpStartIdx      = SUBFRAME_MRR_CHIRP_START_IDX;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].subFramePeriodicity= SUBFRAME_MRR_PERIODICITY_VAL;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].burstPeriodicity   = SUBFRAME_MRR_PERIODICITY_VAL;
    ptrAdvFrameCfg->frameData.subframeDataCfg[0].numAdcSamples = PROFILE_MRR_ADC_SAMPLE_VAL*2;
    ptrAdvFrameCfg->frameData.subframeDataCfg[0].totalChirps   = SUBFRAME_MRR_NUM_CHIRPS_TOTAL;
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
    #ifdef SUBFRAME_CONF_MRR
        /* The low resolution 80m subframe */
        ptrAdvFrameCfg->frameSeq.subFrameCfg[0].forceProfileIdx    = 0;
        ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numLoops           = SUBFRAME_MRR_LOOP_COUNT;
        ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfBurst         = 1;
        ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfBurstLoops    = 1;
        ptrAdvFrameCfg->frameSeq.subFrameCfg[0].chirpStartIdxOffset= 0;
        ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfChirps        = SUBFRAME_MRR_CHIRP_END_IDX - SUBFRAME_MRR_CHIRP_START_IDX + 1;
        ptrAdvFrameCfg->frameSeq.subFrameCfg[0].chirpStartIdx      = SUBFRAME_MRR_CHIRP_START_IDX;
        ptrAdvFrameCfg->frameSeq.subFrameCfg[0].subFramePeriodicity= SUBFRAME_MRR_PERIODICITY_VAL;
        ptrAdvFrameCfg->frameSeq.subFrameCfg[0].burstPeriodicity   = SUBFRAME_MRR_PERIODICITY_VAL;
        ptrAdvFrameCfg->frameData.subframeDataCfg[0].numAdcSamples = PROFILE_MRR_ADC_SAMPLE_VAL*2;
        ptrAdvFrameCfg->frameData.subframeDataCfg[0].totalChirps   = SUBFRAME_MRR_NUM_CHIRPS_TOTAL;
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
        ptrProfileCfg->profileId             = PROFILE_MRR_PROFILE_ID;
        ptrProfileCfg->startFreqConst        = PROFILE_MRR_START_FREQ_VAL;
        ptrProfileCfg->idleTimeConst         = PROFILE_MRR_IDLE_TIME_VAL;
        ptrProfileCfg->adcStartTimeConst     = PROFILE_MRR_ADC_START_TIME_VAL;
        ptrProfileCfg->rampEndTime           = PROFILE_MRR_RAMP_END_TIME_VAL;
        ptrProfileCfg->txOutPowerBackoffCode = PROFILE_MRR_TXOUT_POWER_BACKOFF;
        ptrProfileCfg->txPhaseShifter        = PROFILE_MRR_TXPHASESHIFTER_VAL;
        ptrProfileCfg->freqSlopeConst        = PROFILE_MRR_FREQ_SLOPE_VAL;
        ptrProfileCfg->txStartTime           = PROFILE_MRR_TX_START_TIME_VAL;
        ptrProfileCfg->numAdcSamples         = PROFILE_MRR_ADC_SAMPLE_VAL;
        ptrProfileCfg->digOutSampleRate      = PROFILE_MRR_DIGOUT_SAMPLERATE_VAL;
        ptrProfileCfg->hpfCornerFreq1        = PROFILE_MRR_HPFCORNER_FREQ1_VAL;
        ptrProfileCfg->hpfCornerFreq2        = PROFILE_MRR_HPFCORNER_FREQ2_VAL;
        ptrProfileCfg->rxGain                = PROFILE_MRR_RX_GAIN_VAL;
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
         *  - MRR Fast Chirp for max-velocity enhancement. */
        ptrChirpCfg->profileId       = CHIRP_MRR_0_PROFILE_ID;
        ptrChirpCfg->adcStartTimeVar = CHIRP_MRR_0_ADC_START_TIME_VAL;
        ptrChirpCfg->chirpEndIdx     = CHIRP_MRR_0_END_INDEX;
        ptrChirpCfg->chirpStartIdx   = CHIRP_MRR_0_START_INDEX;
        ptrChirpCfg->idleTimeVar     = CHIRP_MRR_0_IDLE_TIME_VAL;
        ptrChirpCfg->txEnable        = CHIRP_MRR_0_TX_CHANNEL;
        ptrChirpCfg->startFreqVar    = CHIRP_MRR_0_START_FREQ_VAL;
        ptrChirpCfg->freqSlopeVar    = CHIRP_MRR_0_FREQ_SLOPE_VAL;
    }
    else if (chirpNum == 1U)
    {
        /* Populate the default configuration for chirp 2 
         *  - MRR Slow Chirp for max-velocity enhancement. */
        ptrChirpCfg->profileId       = CHIRP_MRR_1_PROFILE_ID;
        ptrChirpCfg->adcStartTimeVar = CHIRP_MRR_1_ADC_START_TIME_VAL;
        ptrChirpCfg->chirpEndIdx     = CHIRP_MRR_1_END_INDEX;
        ptrChirpCfg->chirpStartIdx   = CHIRP_MRR_1_START_INDEX;
        ptrChirpCfg->idleTimeVar     = CHIRP_MRR_1_IDLE_TIME_VAL;
        ptrChirpCfg->txEnable        = CHIRP_MRR_1_TX_CHANNEL;
        ptrChirpCfg->startFreqVar    = CHIRP_MRR_1_START_FREQ_VAL;
        ptrChirpCfg->freqSlopeVar    = CHIRP_MRR_1_FREQ_SLOPE_VAL;
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
    else if (chirpNum == 4U)
    {
        /* Populate the default configuration for chirp 4 
         *  - USRR Tx3. */
        ptrChirpCfg->profileId       = CHIRP_USRR_2_PROFILE_ID;
        ptrChirpCfg->adcStartTimeVar = CHIRP_USRR_2_ADC_START_TIME_VAL;
        ptrChirpCfg->chirpEndIdx     = CHIRP_USRR_2_END_INDEX;
        ptrChirpCfg->chirpStartIdx   = CHIRP_USRR_2_START_INDEX;
        ptrChirpCfg->idleTimeVar     = CHIRP_USRR_2_IDLE_TIME_VAL;
        ptrChirpCfg->txEnable        = CHIRP_USRR_2_TX_CHANNEL;
        ptrChirpCfg->startFreqVar    = CHIRP_USRR_2_START_FREQ_VAL;
        ptrChirpCfg->freqSlopeVar    = CHIRP_USRR_2_FREQ_SLOPE_VAL;
    
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
    ptrLowPowerMode->lpAdcMode     = LP_ADC_MODE_LOW_POWER;
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
    ptrChannelCfg->txChannelEn = TX_CHANNEL_1_2_3_ENABLE;
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
/**
 *  @b Description
 *  @n
 *      The function initializes the chirp configuration with the default
 *      parameters.
 *
 
 *  @param[out] ptrChirpCfg
 *      Pointer to the chirp configuration
 *
 *  @retval
 *      Not applicable
 */
void Cfg_TxPhaseShiftInitParams ( rlRfPhaseShiftCfg_t* ptrtxPhaseShiftCfg, uint8_t tx0PhShCode, uint8_t tx1PhShCode, uint8_t tx2PhShCode)
{
    /* Initialize the configuration: */
    memset ((void*)ptrtxPhaseShiftCfg, 0, sizeof(rlRfPhaseShiftCfg_t));

    /* Populate the default configuration for chirp 0 and chirp 1 . 
     *  - MRR Fast Chirp for max-velocity enhancement. */
    ptrtxPhaseShiftCfg->chirpStartIdx = CHIRP_MRR_0_START_INDEX;
    ptrtxPhaseShiftCfg->chirpEndIdx   = CHIRP_MRR_1_END_INDEX;
    ptrtxPhaseShiftCfg->tx0PhaseShift = tx0PhShCode;
    ptrtxPhaseShiftCfg->tx1PhaseShift = tx1PhShCode;
    ptrtxPhaseShiftCfg->tx2PhaseShift = tx2PhShCode; 
    return;
}

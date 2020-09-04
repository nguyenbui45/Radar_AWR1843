/*
 *   @file  srr_config_consts.h
 *
 *   @brief
 *      This file holds constants related to the SRR chirp configuration. 
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


/*! @brief TODO JIT Need to specify range, velocity and other parameter which can be find with this Configuration */
#define PROFILE_IDLE_PROFILE_ID              (0U)
#define PROFILE_IDLE_HPFCORNER_FREQ1_VAL     RL_RX_HPF1_175_KHz
#define PROFILE_IDLE_HPFCORNER_FREQ2_VAL     RL_RX_HPF2_350_KHz
#define PROFILE_IDLE_RX_GAIN_VAL             (30U)
#define PROFILE_IDLE_DIGOUT_SAMPLERATE_VAL   (2000U)
#define PROFILE_IDLE_ADC_SAMPLE_VAL          (64U)
#define PROFILE_IDLE_IDLE_TIME_VAL           TIME_US_TO_10NS(240U)
#define PROFILE_IDLE_RAMP_END_TIME_VAL       TIME_US_TO_10NS(40U)
#define PROFILE_IDLE_START_FREQ_GHZ          (77.01f)
#define PROFILE_IDLE_START_FREQ_VAL          FREQ_GHZ_CONV(PROFILE_IDLE_START_FREQ_GHZ)
#define PROFILE_IDLE_TXOUT_POWER_BACKOFF     (0U)
#define PROFILE_IDLE_TXPHASESHIFTER_VAL      (0U)
#define PROFILE_IDLE_FREQ_SLOPE_MHZ_PER_US   (99.0f)
#define PROFILE_IDLE_FREQ_SLOPE_VAL          FREQ_MHZ_PER_MICRO_S_SLOPE_CONV(PROFILE_IDLE_FREQ_SLOPE_MHZ_PER_US)/*(CONV_SLOPE_MHZ_PER_US_TO_CODEWORD(PROFILE_IDLE_FREQ_SLOPE_MHZ_PER_US)) */
#define PROFILE_IDLE_TX_START_TIME_VAL       TIME_US_TO_10NS(1U)
#define PROFILE_IDLE_ADC_START_TIME_VAL      TIME_US_TO_10NS(6U)

#define CHIRP_IDLE_0_PROFILE_ID                (PROFILE_IDLE_PROFILE_ID)
#define CHIRP_IDLE_0_START_INDEX               (0U)
#define CHIRP_IDLE_0_END_INDEX                 (0U)
#define CHIRP_IDLE_0_START_FREQ_VAL            (0U)
#define CHIRP_IDLE_0_FREQ_SLOPE_VAL            (0U)
#define CHIRP_IDLE_0_IDLE_TIME_VAL             (0U)
#define CHIRP_IDLE_0_ADC_START_TIME_VAL        (0U)
#define CHIRP_IDLE_0_TX_CHANNEL                (TX_CHANNEL_1_ENABLE)

#define CHIRP_IDLE_1_PROFILE_ID                (PROFILE_IDLE_PROFILE_ID)
#define CHIRP_IDLE_1_START_INDEX               (1U)
#define CHIRP_IDLE_1_END_INDEX                 (1U)
#define CHIRP_IDLE_1_START_FREQ_VAL            (0U)
#define CHIRP_IDLE_1_FREQ_SLOPE_VAL            (0U)
#define CHIRP_IDLE_1_IDLE_TIME_VAL             (0U)
#define CHIRP_IDLE_1_ADC_START_TIME_VAL        (0U)
#define CHIRP_IDLE_1_TX_CHANNEL                (TX_CHANNEL_2_ENABLE)

/* FRAME Configuration */
#define FRAME_IDLE_CHIRP_START_IDX           (CHIRP_IDLE_0_START_INDEX)
#define FRAME_IDLE_CHIRP_END_IDX             (CHIRP_IDLE_0_END_INDEX)
#define FRAME_IDLE_COUNT                     (0)
#define FRAME_IDLE_LOOP_COUNT                (128U)
#define FRAME_IDLE_PERIODICITY_VAL           ((60 * 1000000) / 5) /* 60 msec */
#define FRAME_IDLE_TIGGER_SELECT             (1U)
#define FRAME_IDLE_TRIGGER_DELAY_VAL         (0U)
#define FRAME_IDLE_NUM_REAL_ADC_SAMPLES      (PROFILE_IDLE_ADC_SAMPLE_VAL * 2)
#define FRAME_IDLE_NUM_CMPLX_ADC_SAMPLES     (PROFILE_IDLE_ADC_SAMPLE_VAL)

#define FRAME_IDLE_NUM_CHIRPS_TOTAL          ((FRAME_IDLE_CHIRP_END_IDX - FRAME_IDLE_CHIRP_START_IDX + 1) * FRAME_IDLE_LOOP_COUNT)


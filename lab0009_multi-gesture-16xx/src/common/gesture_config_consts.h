/*
 *   @file  gesture_config_consts.h
 *
 *   @brief
 *      This file holds constants related to the Gesture chirp configuration. 
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


#include "gesture_defines.h"

#ifndef SRR_CONFIG_CONSTS_H
#define SRR_CONFIG_CONSTS_H
#include <ti/common/sys_common.h>

/*! @brief Active and idle profile configurations. */
#include "gesture_config_chirp_design_idleMode.h"
#include "gesture_config_chirp_design_activeMode.h"


/** \brief Macros for time calculation */
/* Some MACROS to simplify programming the device */
#define ROUND_TO_INT32(X)	((int32_t) (X))
#define CONV_FREQ_GHZ_TO_CODEWORD(X) ROUND_TO_INT32(X * (1.0e9/53.644))
#define CONV_SLOPE_MHZ_PER_US_TO_CODEWORD(X) (ROUND_TO_INT32(X * (1000.0/48.279)))

#define TIME_MS_TO_5NS(x) ((x * 1000000)/5)
#define TIME_US_TO_10NS(x) (x * 100)

#define TIME_5NS_TO_MS(x) (((x) * 5)/1000000)

#define FREQ_CONST (3.6 * 1000000000.0/67108864.0)

/** \brief Macro for Frequency Calculation. Input 'x' is in GHz */
#define FREQ_GHZ_CONV(x) ((rlUInt32_t)(((float)x * 1000000000.0 + FREQ_CONST)/FREQ_CONST))

#define FREQ_SLOPE_CONST (FREQ_CONST * 900.0/1000.0)

/** \brief Macro for frequency slope conversion. Input 'x' is in MHz/us */
#define FREQ_MHZ_PER_MICRO_S_SLOPE_CONV(x) ((rlInt16_t)((x * 1000.0 + FREQ_SLOPE_CONST)/FREQ_SLOPE_CONST))


/** Universal options for the GESTURE TI Design. 
 *  These are two configurations used, one for IDLE mode where mmWave device will chirp with
    very low duty cycle to save power and the time it detects any movement in the field of view,
    it will switch to ACTIVE configuration where device will chirp at higher duty cycle to detect
    Gesture properly.    */

#define NUM_RX_CHANNELS						(4U)
#define NUM_CHIRP_PROG						(4U) /* Two for ACTIVE, Two for IDLE */
#define NUM_PROFILES						(2U) /* one for ACTIVE, one for IDLE */

#define IDLE_PROFILE_CFG_SELECTED               0
#define ACTIVE_PROFILE_CFG_SELECTED             1


/*! @brief   Channel Selection */
#define GESTURE_RX_CHANNEL_SELECTION        RX_CHANNEL_1_2_3_4_ENABLE
#define GESTURE_TX_CHANNEL_SELECTION        TX_CHANNEL_1_2_ENABLE
/*! @brief ADC bit and format selection */
#define GESTURE_ADC_BIT                     ADC_BITS_16
#define GESTURE_ADC_FORMAT                  ADC_FORMAT_COMPLEX

/*! @brief Calibration DC Default configurations */
#define GESTURE_CALIB_DC_ENABLE             0
#define GESTURE_CALIB_DC_NEG_BIN_IDX        -5
#define GESTURE_CALIB_DC_POS_BIN_IDX        8
#define GESTURE_CALIB_DC_NEG_AVG_CHIRP      256

/*! @brief NearField Default configurations */
#define GESTURE_NEARFIELD_ENABLE            0
#define GESTURE_NEARFIELD_START_RANGE_IDX   0
#define GESTURE_NEARFIELD_END_RANGE_IDX     0

#define GESTURE_CLUTTER_REMOVAL_ENABLE      0

/*! @brief ADCBUF config */
#define GESTURE_ADCBUF_OUT_FORMAT           0
#define GESTURE_ADCBUF_IQ_SWAP              ADC_I_FIRST
#define GESTURE_ADCBUF_INTERLEAVED          ADC_NON_INTERLEAVED_MODE
/*! @brief Number of chirps to be collected in the ADC buffer, before the chirp available interrupt. */
#define GESUTURE_ADCBUFF_CHIRP_THRESHOLD   (1U)

#define FREQUENCY_LIMIT_LOW             760U
#define FREQUENCY_LIMIT_HIGH            810U


/** @}*/ /* end defgroup. */

#endif

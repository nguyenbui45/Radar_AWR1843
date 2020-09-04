/*
 *   @file  srr_config_consts.h
 *
 *   @brief
 *      This file holds constants related to the SRR chirp configuration. 
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


#include "srr_defines.h"

/* The chirp design for the SRR TI design is as follows.
 *
 * The Radar operates in two modes - essentially two subfram-
 * es per frame, with alternate subframes belonging to one of
 * the two modes.
 * 1. A low range-resolution (30cm), high max range (80m),
 *	  high max velocity(55kmph), poor angle resolution short
 *	  range radar (SRR) mode - called the SRR80 mode.
 * 2. A high range-resolution (4.5cm), low max range (20m)
 *    low max velocity (18kmph), high angle resolution ultra
 *    short range (USRR) mode - called the USRR20 mode.
 *
 * However the max velocity of 80m is not enough for the SRR
 * use case, and we need to facilitate algorithmic techniques
 * to improve it. We will create two kinds of chirp in the
 * SRR80 mode with have a max unambiguous velocity of 55kmph
 * and the other having a max unambiguous velocity of
 * (55/1.2) kmph, and then use the 'chinese remainder
 *  theorem' to estimate the true velocity.
 *
 * The SRR80 consists of one profile, two kinds of chirps,
 * with each chirp being repeated 64 times. These two sets of
 * chirps are processed seperately. The estimate of velocity
 * in one set, is compared with the estimate in the other set
 * to generate the true velocity. Only one tx is active in
 * this mode. i.e. No MIMO is used in SRR80.
 *
 * USRR20 consists of one profile (much higher BW SRR80), two
 * chirps (in a TDM-MIMO configuration), processed as a conventi-
 * nal two-tx four-rx capture. Some improvements to the max-
 * velocity is done so that we can still maintain a reasonable 
 * 36kmph max velocity.
 *
 * Since the chirp design of SRR80 and USRR20 are  different 
 * two different 'processing paths' are used to process them. 
 * 
 * USRR20 is processed by POINT_CLOUD_PROCESSING path and 
 * SRR80 is processed by the MAX_VEL_ENH_PROCESSING path.
 * 
 */

#ifndef SRR_CONFIG_CONSTS_H
#define SRR_CONFIG_CONSTS_H
#include <ti/common/sys_common.h>

/** Universal options for the SRR TI Design. 
 *  These are different compile time options for the SRR TI Design, that modify the base SRR design
 *  based on different requirements. */
/** @{*/ 
/*! @brief LVDS based object transfer. 
 * The following line (if uncommented) uses the LVDS lanes to transfer object detection data
 * to the PC. Needs an LVDS receiver. Hasn't been tested with the TSW1400. */
// #define USE_LVDS_INTERFACE_FOR_OBJECT_DATA_TX 
/*! @brief Reduced Thresholds. 
 * The following line (if uncommented) reduces thresholds for the USRR detection algorithm 
 * to enable the detection of weak targets.  */
// #define LOW_THRESHOLD_FOR_USRR 
/*! @brief 120m max range. 
 * The SRR was initially designed for 80m, but with a slight configuration change will work at 120m (boresight). 
 * Uncomment the following line for enabling that config. */                     
// #define SRR_RANGE_120m 
/** @}*/ 

#define NUM_RX_CHANNELS						(4U)
#define NUM_CHIRP_PROG						(4U) /* Two for SRR80, Two for USRR20 */
#define NUM_PROFILES						(2U) /* one for SRR80, one for USRR20 */

/**! @brief The multi-mode Radar mode of operation. */
#define SUBFRAME_CONF_SRR_USRR                   /* Two subframes, SRR80 followed by USRR20. */
/**! @brief The USRR only mode of operation. */
// #define SUBFRAME_CONF_USRR                   /* One subframe USRR20. */
/**! @brief The SRR only mode of operation. */
// #define SUBFRAME_CONF_SRR                   /* One subframe SRR80. */

#ifdef SUBFRAME_CONF_SRR_USRR
    #ifdef SUBFRAME_CONF_USRR
        #error "More than one SRR configuration is defined."
    #endif
    #ifdef SUBFRAME_CONF_USRR
        #error "More than one SRR configuration is defined."
    #endif
#endif

#ifdef SUBFRAME_CONF_SRR
    #ifdef SUBFRAME_CONF_USRR
        #error "More than one SRR configuration is defined."
    #endif
    #ifdef SUBFRAME_CONF_SRR_USRR
        #error "More than one SRR configuration is defined."
    #endif
#endif

#ifdef SUBFRAME_CONF_USRR
    #ifdef SUBFRAME_CONF_SRR
        #error "More than one SRR configuration is defined."
    #endif
    #ifdef SUBFRAME_CONF_SRR_USRR
        #error "More than one SRR configuration is defined."
    #endif
#endif

#ifdef SUBFRAME_CONF_SRR_USRR
    #define NUM_SUBFRAMES						(2U) /* one for SRR80, one for USRR20 */
#else
    #define NUM_SUBFRAMES						(1U) /* one for SRR80, one for USRR20 */
#endif
/*! @brief Number of chirps to be collected in the ADC buffer, before the chirp available interrupt. */
#define ADCBUFF_CHIRP_THRESHOLD             (1U)

/*! @brief Which subframe is used to do max-vel-enhancement. */
#define MAX_VEL_ENH_SUBFRAME_IDX (0U)

#ifdef SRR_RANGE_120m
    /*! @brief Add in the SRR 80 chirp design. */
    #include "srr_config_chirp_design_SRR120.h"
#else
    /*! @brief Add in the SRR 80 chirp design. */
    #include "srr_config_chirp_design_SRR80.h"
#endif
/*! @brief Add in the USRR 20 chirp design. */
#include "srr_config_chirp_design_USRR20.h"

#if NUM_SUBFRAMES == 2
    #ifdef SUBFRAME_CONF_SRR_USRR
        #define FRAME_PERIODICITY_VAL (SUBFRAME_SRR_PERIODICITY_VAL + SUBFRAME_USRR_PERIODICITY_VAL)
    #else
        #error "invalid configuration"
    #endif
#else
    #ifdef SUBFRAME_CONF_SRR
        #define FRAME_PERIODICITY_VAL (SUBFRAME_SRR_PERIODICITY_VAL)
    #else
	#ifdef SUBFRAME_CONF_USRR
        #define FRAME_PERIODICITY_VAL (SUBFRAME_USRR_PERIODICITY_VAL)
    #else
        #error "invalid configuration"
    #endif
	#endif
#endif

/*! @brief The total frame periodicity in seconds. */
#define FRAME_PERIODICITY_SEC  (FRAME_PERIODICITY_VAL*5e-9)

/*! @brief The number of SNR Thresholds - used to vary the SNR requirement as a function of range. */
#define MAX_NUM_RANGE_DEPENDANT_SNR_THRESHOLDS (3U)

/*! @brief There are two processing paths in the SRR Demo. */
#define MAX_VEL_ENH_PROCESSING (0U)
#define POINT_CLOUD_PROCESSING (1U)

/*! @brief The maximum number of clusters out of the dbscan algorithm (for the USRR subframe). */
#define MAX_NUM_CLUSTER_USRR (24U)

/*! @brief The maximum number of clusters out of the dbscan algorithm (for the SRR subframe). */
#define MAX_NUM_CLUSTER_SRR (32U)

/*! @brief The maximum number of tracked objects from the Kalman filter. */
#define MAX_TRK_OBJs (24U)

/*! @brief Fractional bit width for most of the report data (range, velocity, x, y, etc). */
#define REPORT_N_BIT_FRAC (7U)

/*! @brief Fractional bit width for Thresholds for CFAR data (rangeSNRdB, dopplerSNRdB, AzimSNR, etc). */
#define CFARTHRESHOLD_N_BIT_FRAC (8U) 

/*! @brief The radar's range estimate has a constant error due to the finite distance from the antenna to the LO. */ 
#define MIN_RANGE_OFFSET_METERS (0.075f)

/*! @brief Wait for MIN_TICK_FOR_TX before letting the tracker results out. */ 
#define MIN_TICK_FOR_TX (10U)

/*! @brief We discard objects at extreme angles (greater than 55 degrees) 
 * from the tracking procedure. */ 
#define SIN_55_DEGREES (0.8192f)

/*! @brief We discard objects with poor azimuth SNR from the tracking 
 * procedure. */ 
#define TRK_SIN_AZIM_THRESH (1.0f/256.0f)

/*! @brief In processing the max-velocity enhancement subframe we need to check for det matrix transfer only after
 *   the second set of chirps are processed. */ 
#define CHECK_FOR_DET_MATRIX_TX 1
#define DO_NOT_CHECK_FOR_DET_MATRIX_TX 0

/*! @brief  The maximum number of objects to be send out per frame. 
 *  This number is upper bounded by the transfer rate to the external device. */
#define SRR_MAX_OBJ_OUT 200


/*! @brief  The maximum number of objects detected in the 'Max velocity enhanced' processing path. 
 * Because of all the pruning, and higher thresholds, and lower resolution, 
 * fewer objects are detected in MAX_VEL_ENH_PROCESSING. */
#define MAX_DET_OBJECTS_RAW_MAX_VEL_ENH_PROCESSING 200      
/*! @brief  The maximum number of objects detected in the 'point cloud ' processing path. */
#define MAX_DET_OBJECTS_RAW_POINT_CLOUD_PROCESSING 1024  
/*! @brief  The two peaks (from the 'fast chirp' and the 'slow chirp' should be within 2 dB. */
#define MAX_VEL_IMPROVEMENT_ASSOCIATION_THRESH_DB (3U)
/*! @brief  Convert the threshold to a CFAR magnitude difference. */
#define MAX_VEL_IMPROVEMENT_ASSOCIATION_THRESH ( (( 1U << CFARTHRESHOLD_N_BIT_FRAC) * NUM_RX_CHANNELS * MAX_VEL_IMPROVEMENT_ASSOCIATION_THRESH_DB) / 6U)
/*! @brief  Search across 2 bins in the slow chirp. */
#define MAX_VEL_IMPROVEMENT_NUM_SPREAD  (2U)
/*! @brief  Max velocity improvement of 3x. */
#define MAX_VEL_ENH_NUM_NYQUIST (2U)
/*! @brief  Restrict the number of detected objects per range-gate to 3. */
#define MAX_NUM_DET_PER_RANGE_GATE (3U)

/*! @brief Unused #defines @{ */
#define FRAME_CHIRP_START_IDX           (0U)
#define FRAME_CHIRP_END_IDX             (1U)
#define FRAME_COUNT_VAL                 (0U)
#define FRAME_LOOP_COUNT                (64U)
#define FRAME_TRIGGER_DELAY_VAL         (0U)
#define FRAME_NUM_REAL_ADC_SAMPLES      (512U)
#define FRAME_NUM_CMPLX_ADC_SAMPLES     (256U)
/*! @} */

/** @defgroup SRR EDMA Allocation.
 *  There are three seperate EDMA allocations in the SRR algorithm. 
 *  the first two are for subbframe 1 and subframe 2, and the 
 *  third is for CBUFF (LVDS data transfer).
 @{ */
 
/*! @brief EDMA configuration table for FFT processing of subframe 0 */
/** @{*/
#define SRR_SF0_EDMA_CH_1D_IN_PING      EDMA_TPCC1_REQ_FREE_0
#define SRR_SF0_EDMA_CH_1D_IN_PONG      EDMA_TPCC1_REQ_FREE_1
#define SRR_SF0_EDMA_CH_1D_OUT_PING     EDMA_TPCC1_REQ_FREE_2
#define SRR_SF0_EDMA_CH_1D_OUT_PONG     EDMA_TPCC1_REQ_FREE_3
#define SRR_SF0_EDMA_CH_2D_IN_PING      EDMA_TPCC1_REQ_FREE_4
#define SRR_SF0_EDMA_CH_2D_IN_PONG      EDMA_TPCC1_REQ_FREE_5
#define SRR_SF0_EDMA_CH_DET_MATRIX      EDMA_TPCC1_REQ_FREE_6
#define SRR_SF0_EDMA_CH_DET_MATRIX2     EDMA_TPCC1_REQ_FREE_7
#define SRR_SF0_EDMA_CH_3D_IN_PING      EDMA_TPCC1_REQ_FREE_8
#define SRR_SF0_EDMA_CH_3D_IN_PONG      EDMA_TPCC1_REQ_FREE_9
/** @} */

/*! @brief EDMA configuration table for FFT processing of subframe 1. */
/** @{*/
#define SRR_SF1_EDMA_CH_1D_IN_PING      EDMA_TPCC1_REQ_FREE_10
#define SRR_SF1_EDMA_CH_1D_IN_PONG      EDMA_TPCC1_REQ_FREE_11
#define SRR_SF1_EDMA_CH_1D_OUT_PING     EDMA_TPCC1_REQ_FREE_12
#define SRR_SF1_EDMA_CH_1D_OUT_PONG     EDMA_TPCC1_REQ_FREE_13
#define SRR_SF1_EDMA_CH_2D_IN_PING      EDMA_TPCC1_REQ_FREE_14
#define SRR_SF1_EDMA_CH_2D_IN_PONG      EDMA_TPCC1_REQ_FREE_15
#define SRR_SF1_EDMA_CH_DET_MATRIX      EDMA_TPCC1_REQ_FREE_16
#define SRR_SF1_EDMA_CH_DET_MATRIX2     EDMA_TPCC1_REQ_FREE_17
#define SRR_SF1_EDMA_CH_3D_IN_PING      EDMA_TPCC1_REQ_FREE_18
#define SRR_SF1_EDMA_CH_3D_IN_PONG      EDMA_TPCC1_REQ_FREE_19
/** @} */
/*! @brief EDMA configuration table for CBUF. */
/** @{*/
#define SRR_CBUFF_EDMA_CH               EDMA_TPCC0_REQ_CBUFF_0 
#define SRR_CBUFF_EDMA_SHADOW_CH        (EDMA_NUM_DMA_CHANNELS + 0U)
#define CBUFF_EDMA_INSTANCE             (0U)
/** @}*/ 

#define SRR_EDMA_TRIGGER_ENABLE  1
#define SRR_EDMA_TRIGGER_DISABLE 0
/** @}*/ /* end defgroup. */

#endif

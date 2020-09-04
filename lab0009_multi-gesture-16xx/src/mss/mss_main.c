/**
 *   @file  mss_main.c
 *
 *   @brief
 *     MSS main implementation of the Gesture Demo
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

 /** @mainpage Millimeter Wave Gesture (mmw-gesture) Demo for XWR1642
  *
  *  @section intro_sec Introduction
  *
  *  The millimeter wave demo shows some of the gesture recognition
  *  capabilities of the XWR1642 SoC.
  *
  *  Following is a high level description of the features of this demo:
  *  - Detect a set of gestures performed directly in front of the radar.
  *    These gestures are
  *    -# right to left.
  *    -# left to right.
  *    -# up to down.
  *    -# down to up.
  *    -# clockwise swirl
  *    -# anti-clockwise swirl
  *  - Presence detection - detect the presence of people nearby (within 1-2 feet).
  *  - Power saving mode - If no movement, demo will use Idle-mode profile
  *    (gesture_config_chirp_design_idleMode.h) which is much relaxed chirp configuration
  *    effectively saves the power and doesn't heat up the device. And if device detects
  *    any movement in the range of 1-2 feet then it switches to Active-mode profile
  *    (gesture_config_chirp_design_activeMode.h) which is required for Gesture recognition
  *    and more intense/power consuming configuration. Further device will switch to Idle-mode
  *    profile if it doesn't see any movement for 2-3sec.
  *    @Note:
  *    - During profile switching moment it may not detect any gesture.
  *    - To disable this feature set DISABLE_POWER_SAVING_MODE to '1' in dss_main.c and rebuild.
  *  - Do 1D, 2D, CFAR and Azimuth and elevation processing and optionally stream out
  *    a series of feature vectors. These feature-vectors are internally passed through a
  *    small ANN (neural network) engine to 'infer' gestures.
  *
  *  @section Chirp Design.
  *
  *  In this application, unlike the mmw-demo, a hard-coded chirp design is
  *  used. [common\gesture_config_chirp_design_activeMode.h and gesture_config_chirp_design_idleMode.h)
  *  As such, the device on powering up immediately starts sensing without waiting
  *  for any communication from the host or get the least required CLI command from
  *  the external processor or Visualizer to trigger the Gesture event.
  *
  *  The parameters for this chirp design were selected to detect relatively
  *  slow moving objects (like gestures are expected to be). Hence, this
  *  design has fine range resolution (~5cm), low max-velocity (6m/sec), and
  *  high velocity resolution (0.1m/s).
  *
  *  The following table lists all the parameters of interest of the chirp
  *  design.
  *
  *  Parameter          | Value   -| Comment
  *  ------------------ |----------|--------
  *  Slope              | 99 Mhz/us|
  *  RF start frequency | 77 Ghz   | The 'knee of the ramp' of the chirp starts at 77.01 Ghz, however, ADC sampling only starts after 6us.
  *  RF stop frequency  | 74 Ghz   |
  *  Bandwidth          | 3.9 Ghz  |
  *  Number of Tx used  | 2        | Tx 1 and 2 are used in a TDM-MIMO fashion.
  *  Chirp repeat rate  | 105 us   | Since there are two Txs used, the actual chirp repeat rate is 210us.
  *  Number of chirps   | 256      | Since there are two Txs used, the number of chirps per tx is 128.
  *  Frame Rate         | 16 Hz    |
  *  Sampling rate      |  2 ksps  |
  *  Range resolution   |  5 cm    |
  *
  *  The most important parameter for gesture recognition is velocity resolution.
  *  Having a fine velocity resolution helps distinguish fine movements, and
  *  resolve slow moving objects in the velocity dimension. Once these points
  *  are resolved in velocity, estimating their range, azimuthal angle, and
  *  elevation angle becomes simpler. Hence, the chirp design optimized for
  *  velocity resolution.
  *
  *  @section walk_thru Walkthrough.
  *  The millimeter wave demo is an application with a lot of moving parts,
  *  most of which are common between the mmw-gesture application and the more
  *  general mmw-demo. They  include items like the drivers (for UART, EDMA,
  *  etc), the OS, mailbox control ,etc. Documentation regarding the drivers
  *  and APIs are available in the mmwave SDK and associated collateral.
  *
  *  The following walkthrough concerns itself only with the flow of the
  *  gesture recognition/presence detection application. There are three major
  *  'c' files that have been modified/added in this demo. The files, the
  *  processing core (either MSS or DSS) and the primary functionality achieved
  *  in said file are described in the table below.
  *
  *  SNo | File              | Core | Functionality
  *  ----|------------------ |------|--------------
  *  1.  |dss/dss_data_path.c|DSS   |Performs the basic FMCW processing and generates the doppler-range-antennas radarcube.
  *      |                   |      |Functions in the same file, generates the feature vectors from the radarcube.
  *  2.  |dss/link_config.c  |DSS   |mmWaveLink implementation at DSS to configure RadarSS.
  *  3.  |mss/mss_main.c     |MSS   |Runs the ANN with the feature vectors from the DSS and performs inference.
  *  4.  |ANN/Ann_params_77Ghz.h|MSS|Bias and weights to infer the Gesture event from the generated Feature vectors.
  *
  * Each row of the above table will be elaborated in later sections.
  *
  * @subsection imp_structs Important structures.
  *
  *  This section lists the important data structures used in the demo.
  *
  *  1. Features_t (used as gfeatures) -> contains limits on the maximum and
  *     minimum range, maximum and minimum dopplers (both positive and
  *     negative) that are used to generate feature vectors. It is defined
  *     in dss/Features.h.
  *
  *  2. ANN_struct_t (used as gANN_struct_t) -> contains the weights and biases
  *     as well as the outputs for the ANN. Defined in mss/mss_gesture.h.
  *
  *  3. GestDemo_DSS_MCB (used as gGestDssMCB) -> the main control structure for
  *     DSS. It holds the chirp configuration, and options related to FMCW
  *     processing.
  *
  *  4. GestDemo_MCB (used as gGestMssMCB) -> the main control structure for
  *     MSS.
  *
  *
  * @subsection fv_ext FMCW processing within the DSS (dss/dss_data_path.c).
  *
  *  The FMCW processing performed within the DSS core is a cut-down version of
  * the processing as done in the mmw-demo, with the primary differences being
  * the following three points.
  *
  *  1. In dss_data_path.c::GestDemo_interFrameProcessingPart1/2(), once the 2D-FFT is
  *     complete,  we do not compute the full detection matrix (i.e. the
  *     logarithm of the absolute value for each range and doppler summed over
  *     all of the 'virtual' antennas'). In fact, no detection is performed at
  *     all. Instead, a partial detection matrix (called predetMatrixGesture)
  *     using only one Rx antenna, and only for the range bins of interest (i.e
  *     between gfeatures.rangebin_start and gfeatures.rangebin_stop) is
  *     computed).
  *
  *     The reason why only a single Rx antenna is used is because there was no
  *     performance degradation with only a single Rx antenna.
  *
  *     The reason for the limited range is because gestures are expected to be
  *     performed close to the Radar.
  *
  *  2. Unlike the mmw-demo, the 2D-FFT output is scaled down from 32-bit to 16
  *     bit using the function mmwavelib_scale32to16(), and stored back in the
  *     same L3 memory (as the 2D-FFT input). This allows us to bypass
  *     re-computing the 2D-FFT when we have to do angle estimation.
  *
  *  3. Additionally, the energy in the non-zero velocity bins is collected by
  *     the variable energyInNonZeroVelBinsPerBucket. Note that since clutter
  *     removal (which removes the zero velocity bin) is enabled by default,
  *     we sum energy across all the doppler bins to get the non-zero-vel
  *     energy.
  *
  *  The output of the 2D-FFT and the predetMatrixGesture are then used by the
  * feature extraction functions (Computefeatures_RDIBased() & GestDemo_interFra-
  * meProcessingPart2()) to extract a series of features.
  *
  * @subsection fv_ext Feature vector extraction (in dss/dss_data_path.c).
  * [Computefeatures_RDIBased, Computefeatures_DOABased, Computefeatures_Hybrid]
  *
  *  A classical 'gesture recognition' application using an FMCW radar would
  * generate the radarcube, and then provide the radar-cube to a trained deep
  * neural network (or DNN). Such a DNN would be able to extract features
  * and perform inference. However,  The AWR1642 does not have the necessary
  * MIPS to run a DNN. In any case, for the gestures that we are interested in,
  * a simpler approach suffices.
  *
  *  Our approach extracts from the radarcube a number of features. Each
  * feature is a number corresponding to a certain measurement. These features
  * are provided to the MSS, where they are buffered. The MSS will then run a
  * small ANN on these buffers (called  feature-vectors) and provide an
  * inference as output.
  *
  *  The list of features is computed over a subset of the radarcube, as we are
  * only interested in objects that lie near the radar (between
  * gfeatures.rangebin_start, and gfeatures.rangebin_stop).
  *
  *  The list of features generated are given below.
  *
  *  1. Weighted range  -> the weighted average range (within the range of
  *     interest).
  *  2. Weighted doppler -> the average doppler weighted by the bin
  *     energy.
  *  3. InstEnergy -> Instantaneous energy.
  *  4. Weighted azimuth -> the weighted average azimuth of the strongest
  *     NUM_SORTED_VALUES points in the 2D-FFT output (within the range of
  *     interest).
  *  5. Weighted elevation -> like the weighted azimuth (but for elevation).
  *  6. Azimuth Doppler correlation.
  *
  * Once these features have been computed, they are provided to the MSS using
  * the function GestDemo_dssSendProcessOutputToMSS() in dss/dss_main.c
  *
  * The reason for selecting the 'weighted average' as the representative
  * feature of every dimension (doppler, range, elevation, azimuth) is that we
  * have seen (by examining the feature-vectors) that they are more than enough
  * to visually distinguish between different simple gestures - moving from
  * right to left, up to down, vice versa.).
  *
  * However, for more complicated gestures more features may be necessary -
  * for example, if there are two objects (say palm/hand) moving away and
  * towards the radar, then features like 'weighted doppler (pos)' will
  * capture the palm moving away from the radar, whereas 'weighted doppler
  * (neg)' will capture the palm moving away from the radar. In such a way,
  * based on the gestures that have to be differentiated, different features
  * will need to be generated.
  *
  *
  * @subsection gest_recog Gesture recognition.
  *
  * Gesture recognition is achieved by passing the feature-vectors (computed in
  * the DSS, collated and augmented in the MSS) through a two-layer ANN. The
  * implementation of the ANN is given in the function computeInference() from
  * mss/gesture_utils.c.
  *
  * The non-linearity/activation function between the first and second layer,
  * is relu. The output of the 2nd layer is passed through a soft-max function
  * to generate the inference output - i.e probabilities of each gesture.
  *
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
#include <math.h>


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
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/utils/cli/cli.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/HwiP.h>
#include <ti/drivers/canfd/canfd.h>
#include <ti/common/mmwave_sdk_version.h>

/* Demo Include Files */
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include "mss_gesture.h"
#include "../common/gesture_messages.h"

/* Setting this MACRO to '1', application will send only string gesture output */
#define STRING_OUTPUT_ONLY   1

int findGestureType(void);
void GestDemo_sendEvtMsg(uint32_t gestId);
extern void GestDemo_CLIInit (void);
extern void GestDemo_getMMWaveExtensionOpenConfig(MMWave_OpenCfg* ptrOpenCfg);
extern void GestDemo_getMMWaveExtensionConfig(MMWave_CtrlCfg* ptrCtrlCfg);

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
GestDemo_MCB    gGestMssMCB;


/* structure to send calculated Gesture Feature */
typedef struct GestDemo_gestIdMsg_t
{
    /*! @brief Header of the detection information message */
    GestDemo_output_message_header header;
    
    volatile uint32_t gestId;
    
    uint32_t reserved;
}GestDemo_gestIdMsg;

GestDemo_gestIdMsg gestureIdMsg;

/**************************************************************************
 *************************** Extern Definitions *******************************
 **************************************************************************/
extern void GestUtils_inference(float *input, ANN_struct_t* pANN_struct_t);
/**************************************************************************
 ************************* Millimeter Wave Demo Functions Prototype**************
 **************************************************************************/

/* MMW demo Task */
void GestDemo_mssInitTask(UArg arg0, UArg arg1);
void GestDemo_mssCtrlPathTask(UArg arg0, UArg arg1);

/* external sleep function when in idle (used in .cfg file) */
void GestDemo_sleep(void);

/* DSS to MSS exception signalling ISR */
static void GestDemo_installDss2MssExceptionSignallingISR(void);

/* global flag to signal transfer of ANN output through UART to external world*/
volatile int8_t send_ANN_OP_to_UART = 0; // when set to 1, the ANN output will be sent to UART

#if STRING_OUTPUT_ONLY
/* Defines for String only output */
#define CIRCLE_POST_FILT_PARAMS  8
#define SWIPE_POST_FILT_PARAMS   5
#define HISTORY_LEN              13

#define GESTURE_BACKGROUND       0
#define GESTURE_RIGHT_TO_LEFT    1
#define GESTURE_LEFT_TO_RIGHT    2
#define GESTURE_TOP_TO_DOWN      3
#define GESTURE_DOWN_TO_UP       4
#define GESTURE_WHORL_CLK        5
#define GESTURE_WHORL_ANTICLK    6


float gAnnProb[HISTORY_LEN][NUM_NODES_SECOND_LAYER];
int   gAnnProbIdx = 0U;
volatile uint32_t gestureID = 0U;
#endif

/*Creating buffers to copy features from DSS */
#define HSRAM_BASE_ADDR_DSS  (0x21080000)
#define HSRAM_BASE_ADDR_MSS  (0x52080000)
#define L3RAM_BASE_ADDR_MSS  (0x51000000)
#define L3RAM_BASE_ADDR_DSS  (0x20000000)

/*define start offset of different features stored in Featurevector array*/
#define WTRANGE_FEATURE_START_OFFSET    (2*GESTURE_FEATURE_LENGTH)
#define WTDOPPLER_FEATURE_START_OFFSET  (0*GESTURE_FEATURE_LENGTH)
#define INSTENERGY_FEATURE_START_OFFSET (1*GESTURE_FEATURE_LENGTH)
#define MAXAZFREQ_FEATURE_START_OFFSET  (4*GESTURE_FEATURE_LENGTH)
#define MAXELFREQ_FEATURE_START_OFFSET  (5*GESTURE_FEATURE_LENGTH)
#define AZDOPPCORR_FEATURE_START_OFFSET (3*GESTURE_FEATURE_LENGTH)

/* The array below holds the feature vectors sent by DSS */
float FeaturevectorDSS[NUM_OF_GESTURE_FEATURES * GESTURE_FEATURE_LENGTH] = {0};

/* The array below holds the feature vector after they are processed in MSS */
float FeaturevectorMSS[NUM_OF_GESTURE_FEATURES * GESTURE_FEATURE_LENGTH] = {0};

float detect_feature_threshold = 0; // this will be overwritten using cli config
int16_t rangebin_stop = 0; // this will be overwritten using cli config
int16_t posdopplerbin_stop = 0; // this will be overwritten using cli config
uint32_t gesture_pktno = 0;
uint32_t frameNumber=0;

#ifdef TRANSMIT_OVER_CAN
/* defines and variable for CAN trasmission instead of UART */
#define MMWDEMO_HEADER              0x7U
#define MMWDEMO_PADDING             0x8U
#define CAN_MESSAGE_MMWDEMO_MAX     0x8U  /*(messge type (6) + header (1) + padding(1))*/

#define CAN_MESSAGE_MMWDEMO_HEADER  0xC1
#define CAN_MESSAGE_MMWDEMO_PADDING 0xB1

#define CAN_MSGOBJ_HEADER   0x7U
#define CAN_MSGOBJ_PADDING  0x8U

volatile uint32_t       gTxDoneFlag = 0, gRxDoneFlag = 0, gParityErrFlag = 0;
volatile uint32_t       gTxPkts = 0, gRxPkts = 0, gErrStatusInt = 0;
volatile uint32_t                iterationCount = 0U;
CANFD_MCANFrameType     frameType = CANFD_MCANFrameType_FD;
//CANFD_MCANFrameType     frameType = CANFD_MCANFrameType_CLASSIC;
uint8_t  rxData[64U];
uint32_t  txDataLength, rxDataLength;


typedef enum mmwDemo_can_message_type_e
{
    /*! @brief   List of detected points */
    CAN_MESSAGE_MMWDEMO_DETECTED_POINTS = 0xD1,

    /*! @brief   Range profile */
    CAN_MESSAGE_MMWDEMO_RANGE_PROFILE,

    /*! @brief   Noise floor profile */
    CAN_MESSAGE_MMWDEMO_NOISE_PROFILE,

    /*! @brief   Samples to calculate static azimuth  heatmap */
    CAN_MESSAGE_MMWDEMO_AZIMUT_STATIC_HEAT_MAP,

    /*! @brief   Range/Doppler detection matrix */
    CAN_MESSAGE_MMWDEMO_RANGE_DOPPLER_HEAT_MAP,

    /*! @brief   Stats information */
    CAN_MESSAGE_MMWDEMO_STATS
} mmwDemo_can_message_type;


static void MCANAppInitParams(CANFD_MCANInitParams* mcanCfgParams);

CANFD_Handle                canHandle;
CANFD_MsgObjHandle          txMsgObjHandle;

CANFD_MCANMsgObjCfgParams   txMsgObjectParams;
#endif

ANN_struct_t gANN_struct_t = {
#include "common/ANN/ANN_params_77Ghz.h"
};

/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/
void GestDemo_preprocessANNInputs(float FeaturevectorDSS[], float FeaturevectorMSS[]);

void Dbg_write (const char* format, ...)
{
    va_list     arg;
    char        logMessage[256];
    int32_t     sizeMessage;

    /* Format the message: */
    va_start (arg, format);
    sizeMessage = vsnprintf (&logMessage[0], sizeof(logMessage), format, arg);
    va_end (arg);

    /* Log the message on the UART CLI console: */
    if (true)
    {
        /* Polled mode: */
        UART_writePolling (gGestMssMCB.loggingUartHandle, (uint8_t*)&logMessage[0], sizeMessage);
    }
    else
    {
        /* Blocking Mode: */
        UART_write (gGestMssMCB.loggingUartHandle, (uint8_t*)&logMessage[0], sizeMessage);
    }
}

/**
 *  @b Description
 *  @n
 *      Function to send a message to peer through Mailbox virtural channel 
 *
 *  @param[in]  message
 *      Pointer to the MMW demo message.  
 *
 *  @retval
 *      Success    - 0
 *      Fail       < -1 
 */
int32_t GestDemo_mboxWrite(GestDemo_message     * message)
{
    int32_t                  retVal = -1;
    
    retVal = Mailbox_write (gGestMssMCB.peerMailbox, (uint8_t*)message, sizeof(GestDemo_message));
    if (retVal == sizeof(GestDemo_message))
    {
        retVal = 0;
    }
    return retVal;
}

#ifdef TRANSMIT_OVER_CAN
/**
 *  @b Description
 *  @n
 *      Application implemented callback function to handle error and status interrupts.
 *
 *   @param[in] handle
 *      Handle to the CANFD Driver
 *  @param[in]  reason
 *      Cause of the interrupt which prompted the callback.
 *  @param[in]  errStatusResp
 *      Response structure populated with the value of the fields that caused the error or status interrupt.
 *      Processing of this structure is dependent on the callback reason.
 *
 *  @retval
 *      Not Applicable.
 */
static void MCANAppErrStatusCallback(CANFD_Handle handle, CANFD_Reason reason, CANFD_ErrStatusResp* errStatusResp)
{
    gErrStatusInt++;

    return;
}
/**
 *  @b Description
 *  @n
 *      Application implemented callback function to handle Tx complete and receive interrupts.
 *
 *   @param[in] handle
 *      Handle to the message object
 *   @param[in] reason
 *      Cause of the interrupt which prompted the callback.
 *
 *  @retval
 *      Not Applicable.
 */
static void MCANAppCallback(CANFD_MsgObjHandle handle, CANFD_Reason reason)
{
    int32_t                 errCode, retVal;
    uint32_t                id;
    CANFD_MCANFrameType     rxFrameType;
    CANFD_MCANXidType       rxIdType;

    if (reason == CANFD_Reason_TX_COMPLETION)
    {
        {
            gTxPkts++;
            gTxDoneFlag = 1;
            return;
        }
    }
    if (reason == CANFD_Reason_RX)
    {
        {
            /* Reset the receive buffer */
            memset(&rxData, 0, sizeof (rxData));

            retVal = CANFD_getData (handle, &id, &rxFrameType, &rxIdType, &rxDataLength, &rxData[0], &errCode);
            if (retVal < 0)
            {
                System_printf ("Error: CAN receive data for iteration %d failed [Error code %d]\n", iterationCount, errCode);
                return;
            }

            if (rxFrameType != frameType)
            {
                System_printf ("Error: CAN received incorrect frame type Sent %d Received %d for iteration %d failed\n", frameType, rxFrameType, iterationCount);
                return;
            }

            /* Validate the data */
            gRxPkts++;
            gRxDoneFlag = 1;
            return;
        }
    }
    if (reason == CANFD_Reason_TX_CANCELED)
    {
        {
            gTxPkts++;
            gTxDoneFlag = 1;
            gRxDoneFlag = 1;
            return;
        }
    }

}


static void MCANAppInitParams(CANFD_MCANInitParams* mcanCfgParams)
{
    /*Intialize MCAN Config Params*/
    memset (mcanCfgParams, sizeof (CANFD_MCANInitParams), 0);

    mcanCfgParams->fdMode          = 0x1U;
    mcanCfgParams->brsEnable       = 0x1U;
    mcanCfgParams->txpEnable       = 0x0U;
    mcanCfgParams->efbi            = 0x0U;
    mcanCfgParams->pxhddisable     = 0x0U;
    mcanCfgParams->darEnable       = 0x1U;
    mcanCfgParams->wkupReqEnable   = 0x1U;
    mcanCfgParams->autoWkupEnable  = 0x1U;
    mcanCfgParams->emulationEnable = 0x0U;
    mcanCfgParams->emulationFAck   = 0x0U;
    mcanCfgParams->clkStopFAck     = 0x0U;
    mcanCfgParams->wdcPreload      = 0x0U;
    mcanCfgParams->tdcEnable       = 0x1U;
    mcanCfgParams->tdcConfig.tdcf  = 0U;
    mcanCfgParams->tdcConfig.tdco  = 8U;
    mcanCfgParams->monEnable       = 0x0U;
    mcanCfgParams->asmEnable       = 0x0U;
    mcanCfgParams->tsPrescalar     = 0x0U;
    mcanCfgParams->tsSelect        = 0x0U;
    mcanCfgParams->timeoutSelect   = CANFD_MCANTimeOutSelect_CONT;
    mcanCfgParams->timeoutPreload  = 0x0U;
    mcanCfgParams->timeoutCntEnable= 0x0U;
    mcanCfgParams->filterConfig.rrfe        = 0x1U;
    mcanCfgParams->filterConfig.rrfs        = 0x1U;
    mcanCfgParams->filterConfig.anfe        = 0x1U;
    mcanCfgParams->filterConfig.anfs        = 0x1U;
    mcanCfgParams->msgRAMConfig.lss         = 127U;
    mcanCfgParams->msgRAMConfig.lse         = 64U;
    mcanCfgParams->msgRAMConfig.txBufNum    = 32U;
    mcanCfgParams->msgRAMConfig.txFIFOSize  = 0U;
    mcanCfgParams->msgRAMConfig.txBufMode   = 0U;
    mcanCfgParams->msgRAMConfig.txEventFIFOSize         = 0U;
    mcanCfgParams->msgRAMConfig.txEventFIFOWaterMark    = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO0size             = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO0OpMode           = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO0waterMark        = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO1size             = 64U;
    mcanCfgParams->msgRAMConfig.rxFIFO1waterMark        = 64U;
    mcanCfgParams->msgRAMConfig.rxFIFO1OpMode           = 64U;

    mcanCfgParams->eccConfig.enable         = 1;
    mcanCfgParams->eccConfig.enableChk      = 1;
    mcanCfgParams->eccConfig.enableRdModWr  = 1;

    mcanCfgParams->errInterruptEnable   = 1U;
    mcanCfgParams->dataInterruptEnable  = 1U;
    mcanCfgParams->appErrCallBack       = MCANAppErrStatusCallback;
    mcanCfgParams->appDataCallBack      = MCANAppCallback;
}

void Can_Initialize(void)
{

    int32_t                     errCode = 0;
    int32_t                     retVal = 0;
    CANFD_MCANInitParams        mcanCfgParams;
    CANFD_MCANBitTimingParams   mcanBitTimingParams;
    #if 0
    CANFD_MCANMsgObjectStats    msgObjStats;
    CANFD_MCANErrCntStatus      errCounter;
    CANFD_MCANProtocolStatus    protoStatus;
    #endif
    CANFD_MCANMsgObjCfgParams   rxMsgObjectParams;
    CANFD_MsgObjHandle          rxMsgObjHandle[2];

    gTxDoneFlag = 0;
    gRxDoneFlag = 0;

    /* If using reference board based on TIDA-02001 design, then user needs to define this MACRO.
     * In TIDA-02001 CAN interface is connected over H14 and F14 pin of AWR1642.
     */
#ifdef CAN_OVER_UART_LINE
    /* Setup the PINMUX to bring out the XWR16xx CAN pins */
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINH14_PADAK, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINH14_PADAK, SOC_XWR16XX_PINH14_PADAK_CANFD_TX);

    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINF14_PADAJ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINF14_PADAJ, SOC_XWR16XX_PINF14_PADAJ_CANFD_RX);
#else
    /* In case of AWR1642BOOST RevA, user needs to do the ECU (remove resisters) to enable CAN */
    /* Setup the PINMUX to bring out the XWR16xx CAN pins */
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINE14_PADAE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINE14_PADAE, SOC_XWR16XX_PINE14_PADAE_CANFD_TX);

    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PIND13_PADAD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR16XX_PIND13_PADAD, SOC_XWR16XX_PIND13_PADAD_CANFD_RX);
#endif
     /* Configure the divide value for MCAN source clock */
    SOC_setPeripheralClock(gGestMssMCB.socHandle, SOC_MODULE_MCAN, SOC_CLKSOURCE_VCLK, 4U, &errCode);

    /* Initialize peripheral memory */
    SOC_initPeripheralRam(gGestMssMCB.socHandle, SOC_MODULE_MCAN, &errCode);
    
    
    CSL_FINSR(0x43201450, 22, 22, 0x1U);
    CSL_FINSR(0x4320140C, 26, 16, 0x23U);
    
	
    MCANAppInitParams (&mcanCfgParams);

    /* Initialize the CANFD driver */
    canHandle = CANFD_init(&mcanCfgParams, &errCode);
    if (canHandle == NULL)
    {
        System_printf ("Error: CANFD Module Initialization failed [Error code %d]\n", errCode);
        return ;
    }

    /* Configuring 1Mbps and 5Mbps as nominal and data bit-rate respectively
        Prop seg: 8
        Ph seg 1: 6
        Ph Seg2 : 5
        Sync jump: 1
        BRP(Baud rate Prescaler): 2

        Nominal Bit rate = (40)/(((8+6+5)+1)*BRP) = 1Mhz

        Timing Params for Data Bit rate:
        Prop seg: 2
        Ph seg 1: 2
        Ph Seg2 : 3
        Sync jump: 1
        BRP(Baud rate Prescaler): 1

        Nominal Bit rate = (40)/(((2+2+3)+1)*BRP) = 5Mhz
    */
#if 1
    mcanBitTimingParams.nomBrp      = 0x2U;
    mcanBitTimingParams.nomPropSeg  = 0x8U;
    mcanBitTimingParams.nomPseg1    = 0x6U;
    mcanBitTimingParams.nomPseg2    = 0x5U;
    mcanBitTimingParams.nomSjw      = 0x1U;
#else
    /*500Kbps NomBitRate: (40)/(((6+5+4)+1)*5)*/
    mcanBitTimingParams.nomBrp      = 0x5U;
    mcanBitTimingParams.nomPropSeg  = 0x6U;
    mcanBitTimingParams.nomPseg1    = 0x5U;
    mcanBitTimingParams.nomPseg2    = 0x4U;
    mcanBitTimingParams.nomSjw      = 0x1U;
#endif

#if 1 //5 Mbps
    mcanBitTimingParams.dataBrp     = 0x1U;
    mcanBitTimingParams.dataPropSeg = 0x2U;
    mcanBitTimingParams.dataPseg1   = 0x2U;
    mcanBitTimingParams.dataPseg2   = 0x3U;
    mcanBitTimingParams.dataSjw     = 0x1U;
#else //2 Mbps
    mcanBitTimingParams.dataBrp     = 0x4U;
    mcanBitTimingParams.dataPropSeg = 01U;
    mcanBitTimingParams.dataPseg1   = 0x2U;
    mcanBitTimingParams.dataPseg2   = 0x1U;
    mcanBitTimingParams.dataSjw     = 0x1U;

#endif
    /* Configure the CAN driver */
    retVal = CANFD_configBitTime (canHandle, &mcanBitTimingParams, &errCode);
    if (retVal < 0)
    {
        System_printf ("Error: CANFD Module configure bit time failed [Error code %d]\n", errCode);
        return ;
    }

    /* Setup the transmit message object */
    txMsgObjectParams.direction = CANFD_Direction_TX;
    txMsgObjectParams.msgIdType = CANFD_MCANXidType_29_BIT;
    txMsgObjectParams.msgIdentifier = 0xD1;

    txMsgObjHandle = CANFD_createMsgObject (canHandle, &txMsgObjectParams, &errCode);
    if (txMsgObjHandle == NULL)
    {
        System_printf ("Error: CANFD create Tx message object failed [Error code %d]\n", errCode);
        return ;
    }

    /* Setup the receive message object */
    rxMsgObjectParams.direction = CANFD_Direction_RX;
    rxMsgObjectParams.msgIdType = CANFD_MCANXidType_29_BIT;
    rxMsgObjectParams.msgIdentifier = 0xA1;

    rxMsgObjHandle[0] = CANFD_createMsgObject (canHandle, &rxMsgObjectParams, &errCode);
    if (rxMsgObjHandle[0] == NULL)
    {
        System_printf ("Error: CANFD create Rx message object failed [Error code %d]\n", errCode);
        return ;
    }

    rxMsgObjectParams.direction = CANFD_Direction_RX;
    rxMsgObjectParams.msgIdType = CANFD_MCANXidType_29_BIT;
    rxMsgObjectParams.msgIdentifier = 0xA2;

    rxMsgObjHandle[1] = CANFD_createMsgObject (canHandle, &rxMsgObjectParams, &errCode);
    if (rxMsgObjHandle[1] == NULL)
    {
        System_printf ("Error: CANFD create Rx message object failed [Error code %d]\n", errCode);
        return ;
    }
}

int32_t Can_Transmit_Schedule( uint32_t msg_id,
            uint8_t *txmsg, uint32_t len)
{
    volatile uint32_t                   index = 0;
    int32_t                     retVal = 0;
    int32_t                     errCode = 0;

    //System_printf ("MEssage %x  len %d\n", msg_id, len);
    if(frameType == CANFD_MCANFrameType_FD)
    {
        Task_sleep(1);

        while(len > 64U)
        {
            retVal = CANFD_transmitData (txMsgObjHandle, msg_id, CANFD_MCANFrameType_FD, 64U, &txmsg[index], &errCode);
            index = index + 64U;
            len = len - 64U;

            Task_sleep(1);
        }
        retVal = CANFD_transmitData (txMsgObjHandle, msg_id, CANFD_MCANFrameType_FD, len, &txmsg[index], &errCode);
    }
    else
    {
        while(len > 8U)
        {
            retVal = CANFD_transmitData (txMsgObjHandle, msg_id, CANFD_MCANFrameType_CLASSIC, 8U, &txmsg[index], &errCode);
            if (retVal < 0)
            {
                continue;
            }
            index = index + 8U;
            len = len - 8U;

        }

        retVal = CANFD_transmitData (txMsgObjHandle, msg_id, CANFD_MCANFrameType_CLASSIC, len, &txmsg[index], &errCode);

        while(retVal < 0)
        {
            //System_printf("Debug: Error transmitting CAN data %x , Errcode %x\n", retVal, errCode);
            retVal = CANFD_transmitData (txMsgObjHandle, msg_id, CANFD_MCANFrameType_CLASSIC, len, &txmsg[index], &errCode);
        }
    }
    return retVal;
}

mmwDemo_can_message_type Get_CanMessageIdentifier(MmwDemo_output_message_type type)
{
    mmwDemo_can_message_type can_msgId;

    if(type == MMWDEMO_OUTPUT_MSG_DETECTED_POINTS)
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_DETECTED_POINTS;
    }
    else if(type == MMWDEMO_OUTPUT_MSG_RANGE_PROFILE)
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_RANGE_PROFILE;
    }
    else if(type == MMWDEMO_OUTPUT_MSG_NOISE_PROFILE)
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_NOISE_PROFILE;
    }
    else if(type == MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP)
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_AZIMUT_STATIC_HEAT_MAP;
    }
    else if(type == MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP)
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_RANGE_DOPPLER_HEAT_MAP;
    }
    else if (type == MMWDEMO_OUTPUT_MSG_STATS)
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_STATS;
    }
    else if (type == MMWDEMO_HEADER)
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_HEADER;
    }
    else if (type == MMWDEMO_PADDING)
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_PADDING;
    }
    else
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_MAX;
    }

    if(txMsgObjectParams.msgIdType == CANFD_MCANXidType_29_BIT)
    {
        can_msgId |= ~(0xFFFFFFFF);
    }
    return can_msgId;
}
#endif

/** @brief Weighted doppler Index */
#define DSS_FEATURE_IDX_WTDOPPLER       0
/** @brief Weighted instance energy Index */
#define DSS_FEATURE_IDX_INSTAENERGY     1
/** @brief Weighted range index. */
#define DSS_FEATURE_IDX_WTRANGE         2
/** @brief Azimuth-doppler correlation Index */
#define DSS_FEATURE_IDX_AZDOPP_CORR     3
/** @brief Max Azimuth Frequency Index */
#define DSS_FEATURE_IDX_MAXAZFREQ       4
/** @brief Max Elevation Frequency Index */
#define DSS_FEATURE_IDX_MAXELFREQ       5
/** @brief Number of cycles for gesture processing index. */
#define DSS_FEATURE_IDX_NUMCYCLES       6

#define GESTURE_PROB_IDX                7

#define GESTURE_PKT_NO_IDX              14

/**
 *  @b Description
 *  @n
 *     A function to copy the current frame's feature vectors into a log. This
 *     log is then passed on to the Inference engine.  The oldest set of
 *     feature vector is deleted from the log, simultaneously.
 *
 *  @retval
 *      Not Applicable.
 */
void logFeatures(float * ptrFeatures, float *featureVecLog, int32_t negateAngleMeasurements)
{
    FeaturevectorDSS[WTRANGE_FEATURE_START_OFFSET + GESTURE_FEATURE_LENGTH -1U]     = ptrFeatures[DSS_FEATURE_IDX_WTRANGE];
    FeaturevectorDSS[WTDOPPLER_FEATURE_START_OFFSET +  GESTURE_FEATURE_LENGTH -1U]  = ptrFeatures[DSS_FEATURE_IDX_WTDOPPLER];
    FeaturevectorDSS[INSTENERGY_FEATURE_START_OFFSET +  GESTURE_FEATURE_LENGTH -1U] = (float) (20U * log10((double) ptrFeatures[DSS_FEATURE_IDX_INSTAENERGY]));
    FeaturevectorDSS[MAXAZFREQ_FEATURE_START_OFFSET + GESTURE_FEATURE_LENGTH -1U]   = ptrFeatures[DSS_FEATURE_IDX_MAXAZFREQ];
    FeaturevectorDSS[MAXELFREQ_FEATURE_START_OFFSET + GESTURE_FEATURE_LENGTH -1U]   = ptrFeatures[DSS_FEATURE_IDX_MAXELFREQ];
    FeaturevectorDSS[AZDOPPCORR_FEATURE_START_OFFSET + GESTURE_FEATURE_LENGTH -1U]  = ptrFeatures[DSS_FEATURE_IDX_AZDOPP_CORR];
}

/**
 *  @b Description
 *  @n
 *      The Task is used to handle  the mmw demo messages received from 
 *      Mailbox virtual channel.  
 *
 *  @param[in]  arg0
 *      arg0 of the Task. Not used
 *  @param[in]  arg1
 *      arg1 of the Task. Not used
 *
 *  @retval
 *      Not Applicable.
 */
static void GestDemo_mboxReadTask(UArg arg0, UArg arg1)
{
    GestDemo_message      message;
    int32_t              retVal = 0;
#if !(STRING_OUTPUT_ONLY)
    uint32_t totalPacketLen;
    uint32_t numPaddingBytes;
#endif
    uint32_t itemIdx, ii=0;
    float * ptrFeatures;

    /* wait for new message and process all the messages received from the peer */
    while(1)
    {
        Semaphore_pend(gGestMssMCB.mboxSemHandle, BIOS_WAIT_FOREVER);
        
        /* Read the message from the peer mailbox: We are not trying to protect the read
         * from the peer mailbox because this is only being invoked from a single thread */
        retVal = Mailbox_read(gGestMssMCB.peerMailbox, (uint8_t*)&message, sizeof(GestDemo_message));
        if (retVal < 0)
        {
            /* Error: Unable to read the message. Setup the error code and return values */
            System_printf ("Error: Mailbox read failed [Error code %d]\n", retVal);
        }
        else if (retVal == 0)
        {
            /* We are done: There are no messages available from the peer execution domain. */
            continue;
        }
        else
        {
            /* Flush out the contents of the mailbox to indicate that we are done with the message. This will
             * allow us to receive another message in the mailbox while we process the received message. */
            Mailbox_readFlush (gGestMssMCB.peerMailbox);

            /* Process the received message: */
            switch (message.type)
            {
                case GESTDEMO_DSS2MSS_DETOBJ_READY:
#if !(STRING_OUTPUT_ONLY)
                    /* Got detected objects , shipped out through UART */
                    /* Send header */
                    totalPacketLen = sizeof(GestDemo_output_message_header);
                    
                    UART_writePolling (gGestMssMCB.loggingUartHandle,
                                       (uint8_t*)&message.body.detObj.header,
                                       sizeof(GestDemo_output_message_header));
#endif
                    frameNumber = message.body.detObj.header.frameNumber;
                    /* First get the pointer to source address */
                    for (itemIdx = 0;  itemIdx < message.body.detObj.header.numTLVs; itemIdx++)
                    {
                        if(message.body.detObj.tlv[itemIdx].type == GESTDEMO_OUTPUT_MSG_GESTURE_FEATURES)
                        {
                            /* get the corresponding packet number */
                            gesture_pktno = (uint32_t) (*(uint32_t *)(HSRAM_BASE_ADDR_MSS + ((HSRAM_BASE_ADDR_DSS & 0x0000FFFF)+ 4U * sizeof(float)  + 2U * sizeof(int32_t))));
                        }
                    }

                    /* Get the feature address stored in HSRAM */
                    ptrFeatures = (float*)  SOC_translateAddress(message.body.detObj.tlv[0].address,
                            SOC_TranslateAddr_Dir_FROM_OTHER_CPU, NULL);

                    /* first shift the samples */
                    for(itemIdx = 0; itemIdx < GESTURE_FEATURE_LENGTH-1; itemIdx++)
                    {
                        FeaturevectorDSS[WTDOPPLER_FEATURE_START_OFFSET + itemIdx]   = FeaturevectorDSS[WTDOPPLER_FEATURE_START_OFFSET + itemIdx + 1U];
                        FeaturevectorDSS[WTRANGE_FEATURE_START_OFFSET + itemIdx]     = FeaturevectorDSS[WTRANGE_FEATURE_START_OFFSET + itemIdx + 1U];
                        FeaturevectorDSS[INSTENERGY_FEATURE_START_OFFSET + itemIdx]  = FeaturevectorDSS[INSTENERGY_FEATURE_START_OFFSET + itemIdx + 1U];
                        FeaturevectorDSS[AZDOPPCORR_FEATURE_START_OFFSET + itemIdx]  = FeaturevectorDSS[AZDOPPCORR_FEATURE_START_OFFSET + itemIdx + 1U];
                        FeaturevectorDSS[MAXAZFREQ_FEATURE_START_OFFSET + itemIdx]   = FeaturevectorDSS[MAXAZFREQ_FEATURE_START_OFFSET + itemIdx + 1U];
                        FeaturevectorDSS[MAXELFREQ_FEATURE_START_OFFSET + itemIdx]   = FeaturevectorDSS[MAXELFREQ_FEATURE_START_OFFSET + itemIdx + 1U];

                    }

                    /* Store all the features sent by DSS to MSS */
                    logFeatures(ptrFeatures, 0, 0);

                    GestDemo_preprocessANNInputs(&FeaturevectorDSS[0], &FeaturevectorMSS[0]);
                    GestUtils_inference(&FeaturevectorMSS[0], &gANN_struct_t);

                    /* ------- Append to the data packet coming from DSS ------------ */
                    for(ii=0; ii <= NUM_NODES_SECOND_LAYER; ii++)
                    {
                        if(ii < NUM_NODES_SECOND_LAYER)
                        {
                            ptrFeatures[GESTURE_PROB_IDX+ii] = gANN_struct_t.prob[ii];
                        }
                        else
                        {
                            ptrFeatures[GESTURE_PROB_IDX+ii] = (float)gesture_pktno;
                        }
                    }
#if STRING_OUTPUT_ONLY
                    gestureIdMsg.header.frameNumber  = message.body.detObj.header.frameNumber;
                    gestureIdMsg.header.timeCpuCycles= message.body.detObj.header.timeCpuCycles;

                    /* Send short Gesture Messages */
                    GestDemo_sendEvtMsg(findGestureType());
#else
                    /* ------- Append to the data packet coming from DSS ------------ */
                    /* Send TLVs */
                    for (itemIdx = 0;  itemIdx < message.body.detObj.header.numTLVs; itemIdx++)
                    {
#ifdef TRANSMIT_OVER_CAN
                        txMsgObjectParams.msgIdentifier = Get_CanMessageIdentifier((MmwDemo_output_message_type)message.body.detObj.tlv[itemIdx].type);

                        Can_Transmit_Schedule(txMsgObjectParams.msgIdentifier,
                        (uint8_t*)&message.body.detObj.tlv[itemIdx],sizeof(MmwDemo_output_message_tl));


                        txMsgObjectParams.msgIdentifier = Get_CanMessageIdentifier((MmwDemo_output_message_type)message.body.detObj.tlv[itemIdx].type);

                        Can_Transmit_Schedule( txMsgObjectParams.msgIdentifier,
                            (uint8_t*)SOC_translateAddress(message.body.detObj.tlv[itemIdx].address,SOC_TranslateAddr_Dir_FROM_OTHER_CPU,NULL),
                                           message.body.detObj.tlv[itemIdx].length);
#else
                        UART_writePolling (gGestMssMCB.loggingUartHandle,
                                           (uint8_t*)&message.body.detObj.tlv[itemIdx],
                                           sizeof(MmwDemo_output_message_tl));
                        UART_writePolling (gGestMssMCB.loggingUartHandle,
                                           (uint8_t*)SOC_translateAddress(message.body.detObj.tlv[itemIdx].address,
                                                                          SOC_TranslateAddr_Dir_FROM_OTHER_CPU,NULL),
                                           message.body.detObj.tlv[itemIdx].length);
#endif
                        totalPacketLen += sizeof(MmwDemo_output_message_tl) + message.body.detObj.tlv[itemIdx].length;
                    }

                    /* Send padding to make total packet length multiple of GESTDEMO_OUTPUT_MSG_SEGMENT_LEN */
                    numPaddingBytes = GESTDEMO_OUTPUT_MSG_SEGMENT_LEN - (totalPacketLen & (GESTDEMO_OUTPUT_MSG_SEGMENT_LEN-1));
                    if (numPaddingBytes<GESTDEMO_OUTPUT_MSG_SEGMENT_LEN)
                    {
                        uint8_t padding[GESTDEMO_OUTPUT_MSG_SEGMENT_LEN];
                        memset(&padding, 0xf, GESTDEMO_OUTPUT_MSG_SEGMENT_LEN);
#ifdef TRANSMIT_OVER_CAN
                        txMsgObjectParams.msgIdentifier = Get_CanMessageIdentifier((MmwDemo_output_message_type)MMWDEMO_PADDING);

                        Can_Transmit_Schedule( txMsgObjectParams.msgIdentifier,
                                padding,numPaddingBytes);
#else
                        UART_writePolling (gGestMssMCB.loggingUartHandle,
                                            padding,
                                            numPaddingBytes);
#endif
                    }
#endif
                    /* Send a message to MSS to log the output data */
                    memset((void *)&message, 0, sizeof(GestDemo_message));

                    message.type = GESTDEMO_MSS2DSS_DETOBJ_SHIPPED;

                    if (GestDemo_mboxWrite(&message) != 0)
                    {
                        System_printf ("Error: Mailbox send message id=%d failed \n", message.type);
                    }

                    send_ANN_OP_to_UART = 1; //set this to 0 after sending ANN op through UART
                    break;
                    
                case GESTDEMO_MSS2DSS_ACTIVE_PROFILE_SELECT:
                    /* Turn on the LED */
                    GPIO_write (SOC_XWR16XX_GPIO_2, 1U);
                    break;
                case GESTDEMO_MSS2DSS_IDLE_PROFILE_SELECT:
                    /* Turn off the LED */
                    GPIO_write (SOC_XWR16XX_GPIO_2, 0U);
                    break;
                case GESTDEMO_DSS2MSS_STOPDONE:
                    /* Post event that stop is done */
                    Event_post(gGestMssMCB.eventHandleNotify, GESTDEMO_DSS_STOP_COMPLETED_EVT);
                break;
                case GESTDEMO_DSS2MSS_ASSERT_INFO:
                    /* Send the received DSS assert info through CLI */
                    Dbg_write ("DSS Exception: %s, line %d.\n", message.body.assertInfo.file,
                        message.body.assertInfo.line);
                break;
                case GESTDEMO_DSS2MSS_ISR_INFO_ADDRESS:
                    gGestMssMCB.dss2mssIsrInfoAddress = 
                        SOC_translateAddress(message.body.dss2mssISRinfoAddress,
                                             SOC_TranslateAddr_Dir_FROM_OTHER_CPU, NULL);
                    GestDemo_installDss2MssExceptionSignallingISR();
                break;
                case GESTDEMO_DSS2MSS_MEASUREMENT_INFO:
                    /* Send the received DSS calibration info through CLI */
                    Dbg_write ("compRangeBiasAndRxChanPhase");
                    Dbg_write (" %.7f", message.body.compRxChanCfg.rangeBias);
                    int32_t i;
                    for (i = 0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
                    {
                        Dbg_write (" %.5f", (float) message.body.compRxChanCfg.rxChPhaseComp[i].real/32768.);
                        Dbg_write (" %.5f", (float) message.body.compRxChanCfg.rxChPhaseComp[i].imag/32768.);
                    }
                    Dbg_write ("\n");
                break;

                case GESTDEMO_DSS2MSS_SENSORSTART_DONE:
                    /* In the same event DSS sends sensor-start status Pass */
                    if(message.body.sensorStartStatus.status == 0)
                    {
                        /* Post an event to main data path task. 
                           This function in only called when mmwave_start() is called on DSS */
                        gGestMssMCB.stats.datapathStartEvt ++;

                        /* Post event to start is done */
                        Event_post(gGestMssMCB.eventHandleNotify, GESTDEMO_DSS_START_COMPLETED_EVT);
                    }
                    else
                    {
                        /* Post event to start is failed */
                        Event_post(gGestMssMCB.eventHandleNotify, GESTDEMO_DSS_START_FAILED_EVT);
                    }
                break;
                default:
                {
                    /* Message not support */
                    System_printf ("Error: unsupport Mailbox message id=%d\n", message.type);
                    break;
                }
            }
        }
    }
}

#if STRING_OUTPUT_ONLY

/* Calculate the gesture type and return gestureID */
int findGestureType(void)
{
    uint8_t ii, jj;
    float annProbSum[NUM_NODES_SECOND_LAYER] = {0};

    memcpy(&gAnnProb[gAnnProbIdx], &gANN_struct_t.prob[0], sizeof(float)*NUM_NODES_SECOND_LAYER);
    gAnnProbIdx++;

    /* reset the index to make it circular buffer */
    if(gAnnProbIdx >= HISTORY_LEN)
    {
        gAnnProbIdx = 0U;
    }
    
    /* sum up all the gesture weightage */
    for(jj=0;jj<NUM_NODES_SECOND_LAYER;jj++)
    {
        for(ii=0; ii < HISTORY_LEN; ii++)
        {
            annProbSum[jj] += gAnnProb[ii][jj];
        }
    }
    
    /* reset the gesture ID */
    gestureID = 0;
    
    /* check for Right->Left gesture */
    if(annProbSum[GESTURE_RIGHT_TO_LEFT] >= SWIPE_POST_FILT_PARAMS)
    {
        gestureID = (GESTURE_RIGHT_TO_LEFT);
    }
    /* check for Left->Right gesture */
    else if(annProbSum[GESTURE_LEFT_TO_RIGHT] >= SWIPE_POST_FILT_PARAMS)
    {
       gestureID = (GESTURE_LEFT_TO_RIGHT);
    }
    /* check for Top->Down gesture */
    else if(annProbSum[GESTURE_TOP_TO_DOWN] >= SWIPE_POST_FILT_PARAMS)
    {
       gestureID = (GESTURE_TOP_TO_DOWN);
    }
    /* check for Down->Top gesture */
    else if(annProbSum[GESTURE_DOWN_TO_UP] >= SWIPE_POST_FILT_PARAMS)
    {
       gestureID = (GESTURE_DOWN_TO_UP);
    }
    /* check for Clockwise whorl gesture */
    else if(annProbSum[GESTURE_WHORL_CLK] >= SWIPE_POST_FILT_PARAMS)
    {
       gestureID = (GESTURE_WHORL_CLK);
    }
    /* check for Anti-Clockwise whorl gesture */
    else if(annProbSum[GESTURE_WHORL_ANTICLK] >= SWIPE_POST_FILT_PARAMS)
    {
       gestureID = (GESTURE_WHORL_ANTICLK);
    }
    else
    {
        gestureID = (GESTURE_BACKGROUND);
    }

    return gestureID;
}

#define GEST_MESSAGE_SIZE  15
volatile uint32_t lastGestId = 0;

void GestDemo_sendEvtMsg(uint32_t gestId)
{
    char gestureMsg[7][GEST_MESSAGE_SIZE] = { "background   \r",
                                              "Left         \r",
                                              "Right        \r",
                                              "Down         \r",
                                              "Up           \r",
                                              "Whirl        \r",
                                              "AntiWhirl    \r" };
    char errMsg[GEST_MESSAGE_SIZE+2] = "Wrong Gesture\r\n", newLine[2] = "\n";

#ifdef TRANSMIT_OVER_CAN
    txMsgObjectParams.msgIdentifier = Get_CanMessageIdentifier(MMWDEMO_OUTPUT_MSG_GESTURE_STR);
#endif

    if(lastGestId != gestId)
    {
#ifdef TRANSMIT_OVER_CAN
        Can_Transmit_Schedule(txMsgObjectParams.msgIdentifier, (uint8_t*)&newLine[0], 2);
#else
        /* Print new line whenever new gesture detected so new gesture will be printed on next line */
        UART_writePolling (gGestMssMCB.loggingUartHandle, (uint8_t*)&newLine[0], 2);
#endif
    }

    if(gestId <= GESTURE_WHORL_ANTICLK)
    {
#ifdef TRANSMIT_OVER_CAN
        Can_Transmit_Schedule(txMsgObjectParams.msgIdentifier, (uint8_t*)&gestureMsg[gestId][0], GEST_MESSAGE_SIZE);
#else
        /* send UART message */
        UART_writePolling (gGestMssMCB.loggingUartHandle,
                       (uint8_t*)&gestureMsg[gestId][0], GEST_MESSAGE_SIZE);
#endif
    }
    else
    {
#ifdef TRANSMIT_OVER_CAN
        Can_Transmit_Schedule(txMsgObjectParams.msgIdentifier, (uint8_t*)&errMsg[0], GEST_MESSAGE_SIZE+2);
#else
        /* send UART message */
        UART_writePolling (gGestMssMCB.loggingUartHandle,
                       (uint8_t*)&errMsg[0], GEST_MESSAGE_SIZE+2);
#endif
    }

    lastGestId = gestId;
}
#endif

/**
 *  @b Description
 *  @n
 *      This function is a callback function that invoked when a message is received from the peer.
 *
 *  @param[in]  handle
 *      Handle to the Mailbox on which data was received
 *  @param[in]  peer
 *      Peer from which data was received
 
 *  @retval
 *      Not Applicable.
 */
void GestDemo_mboxCallback(Mbox_Handle  handle,
                           Mailbox_Type    peer)
{
    /* Message has been received from the peer endpoint. Wakeup the mmWave thread to process
     * the received message. */
    Semaphore_post (gGestMssMCB.mboxSemHandle);
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be invoked from the CLI or
 *      the Switch press to start the sensor. This sends an event to the
 *      sensor management task where the actual *start* procedure is
 *      implemented.
 *
 *  @retval
 *      Not applicable
 */
int32_t GestDemo_notifySensorStart(bool doReconfig)
{
    GestDemo_message     message;
    GestDemo_sensorStartStopCfg cfg;

    /* If sensor start is already being called without sensorStop command 
       then return error 
     */
    if(gGestMssMCB.stats.cliSensorStartEvt > 0)
    {
        return -1;
    }

    memset((void*)&cfg, 0, sizeof(GestDemo_sensorStartStopCfg));
    memset((void *)&message, 0, sizeof(GestDemo_message));

    
    /* Trigger the Frame rlSensorStart */
    cfg.coreRunsLink = DSS_CORE_RUNS_LINK;

    /* set command type to sensor-start */
    cfg.cmdType = 0;
    gGestMssMCB.stats.cliSensorStartEvt ++;
    gGestMssMCB.stats.cliSensorStopEvt --;
    
    /* notify DSS for Sensor-start */
    message.type = GESTDEMO_MSS2DSS_SENSOR_START_CFG;
    memcpy((void *)&message.body.sensorStartStopCfg, (void *)&cfg, sizeof(GestDemo_sensorStartStopCfg));

    if (GestDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be invoked from the CLI or
 *      the Switch press to start the sensor. This sends an event to the
 *      sensor management task where the actual *start* procedure is
 *      implemented.
 *
 *  @retval
 *      Not applicable
 */
int32_t GestDemo_notifySensorStop(void)
{
    GestDemo_message     message;
    GestDemo_sensorStartStopCfg cfg;

    /* If sensor stop is already being called without sensorStart command 
       then return error.
     */
    if(gGestMssMCB.stats.cliSensorStopEvt > 0)
    {
        return -1;
    }

    memset((void*)&cfg, 0, sizeof(GestDemo_sensorStartStopCfg));
    memset((void *)&message, 0, sizeof(GestDemo_message));

    
    /* Trigger the Frame rlSensorStart */
    cfg.coreRunsLink = DSS_CORE_RUNS_LINK;

    /* set command type to sensor-stop */
    cfg.cmdType = 1;
    
    gGestMssMCB.stats.cliSensorStopEvt ++;

    gGestMssMCB.stats.cliSensorStartEvt--;
    
    
    /* notify DSS for Sensor-start */
    message.type = GESTDEMO_MSS2DSS_SENSOR_START_CFG;
    memcpy((void *)&message.body.sensorStartStopCfg, (void *)&cfg, sizeof(GestDemo_sensorStartStopCfg));

    if (GestDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be used by the CLI to 
 *      pend for start complete (after GestDemo_notifySensorStart is called)
 *
 *  @retval
 *      Not applicable
 */
int32_t GestDemo_waitSensorStartComplete(void)
{
    UInt          event;
    int32_t       retVal;
    /* Wait till DSS sends notification */
    event = Event_pend(gGestMssMCB.eventHandleNotify, 
                          Event_Id_NONE, 
                          GESTDEMO_DSS_START_COMPLETED_EVT | GESTDEMO_DSS_START_FAILED_EVT,
                          BIOS_WAIT_FOREVER);
    
    /************************************************************************
     * DSS event:: START notification
     ************************************************************************/
    if(event & GESTDEMO_DSS_START_COMPLETED_EVT)
    {
        /* Sensor has been started successfully */
        gGestMssMCB.isSensorStarted = true;
        retVal = 0;
    }
    else if(event & GESTDEMO_DSS_START_FAILED_EVT)
    {
        /* Sensor start failed */
        gGestMssMCB.isSensorStarted = false;
        retVal = -1;
    } 
    else 
    {
        /* we should block forever till we get the events. If the desired event 
           didn't happen, then throw an assert */
        retVal = -1;
        GestDemo_mssAssert(0);
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be used by the CLI to 
 *      pend for stop complete (after GestDemo_notifySensorStart is called)
 *
 *  @retval
 *      Not applicable
 */
void GestDemo_waitSensorStopComplete(void)
{
    UInt          event;
    /* Pend on the START NOTIFY event */
    event = Event_pend(gGestMssMCB.eventHandleNotify, 
                          Event_Id_NONE, 
                          GESTDEMO_DSS_STOP_COMPLETED_EVT,
                          BIOS_WAIT_FOREVER);

    /************************************************************************
     * DSS event:: STOP notification
     ************************************************************************/
    if(event & GESTDEMO_DSS_STOP_COMPLETED_EVT)
    {
        /* Sensor has been stopped successfully */
        gGestMssMCB.isSensorStarted = false;
        
        /* print for user */
        System_printf("Sensor has been stopped\n");
    }
    else {
        /* we should block forever till we get the event. If the desired event 
           didn't happen, then throw an assert */
        GestDemo_mssAssert(0);
    }
}


/**
 *  @b Description
 *  @n
 *      DSS to MSS ISR used for direct signalling of things like urgent exception
 *      events from DSS. Posts deadline miss events to @ref GestDemo_mssCtrlPathTask.
 *
 *  @retval
 *      Not Applicable.
 */
void GestDemo_Dss2MssISR(uintptr_t arg)
{  
    switch(*(uint8_t*)gGestMssMCB.dss2mssIsrInfoAddress)
    {
        case GESTDEMO_DSS2MSS_CHIRP_PROC_DEADLINE_MISS_EXCEPTION:
            GestDemo_mssAssert(0);
        break;
        
        case GESTDEMO_DSS2MSS_FRAME_PROC_DEADLINE_MISS_EXCEPTION:
            GestDemo_mssAssert(0);
        break;

        default:
            GestDemo_mssAssert(0);
        break;
    }
}

/**
 *  @b Description
 *  @n
 *      Installs DSS to MSS Software Interrupt ISR for exception signalling.
 *
 *  @retval
 *      Not Applicable.
 */
static void GestDemo_installDss2MssExceptionSignallingISR(void)
{
    HwiP_Params  hwiParams;
    volatile HwiP_Handle  hwiHandle;

    HwiP_Params_init(&hwiParams);
    hwiParams.name = "Dss2MssSwISR";
    hwiHandle = HwiP_create(GESTDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_MSS, 
                            GestDemo_Dss2MssISR, &hwiParams);
}

void GestDemo_commModeSetup()
{
    UART_Params         uartParams;
    /* Pinmux setting */
    /* Setup the PINMUX to bring out the UART-1 */
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINN5_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);    
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINN5_PADBE, SOC_XWR16XX_PINN5_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINN4_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);    
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINN4_PADBD, SOC_XWR16XX_PINN4_PADBD_MSS_UARTA_RX);

    /* Setup the PINMUX to bring out the DSS UART */
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINP8_PADBM, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINP8_PADBM, SOC_XWR16XX_PINP8_PADBM_DSS_UART_TX);

    /* Initialize the UART */
    UART_init();

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency  = gGestMssMCB.cfg.sysClockFrequency;
    uartParams.baudRate        = gGestMssMCB.cfg.commandBaudRate;
    uartParams.isPinMuxDone    = 1U;

    /**** GESTURE_ENABLE: Gesture feature data is sent to UART:0 , RS232-TX */
    /* Open the UART Instance */
    gGestMssMCB.commandUartHandle = UART_open(0, &uartParams);
    if (gGestMssMCB.commandUartHandle == NULL)
    {
        System_printf("Error: MMWDemoMSS Unable to open the Command UART Instance\n");
        return;
    }
#ifdef TRANSMIT_OVER_CAN
    Can_Initialize();
#else
    /* Setup the PINMUX to bring out the UART-3 */
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINF14_PADAJ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINF14_PADAJ, SOC_XWR16XX_PINF14_PADAJ_MSS_UARTB_TX);

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency = gGestMssMCB.cfg.sysClockFrequency;
    uartParams.baudRate       = gGestMssMCB.cfg.loggingBaudRate;
    uartParams.isPinMuxDone   = 1U;

    /* Open the Logging UART Instance: */
    gGestMssMCB.loggingUartHandle = UART_open(1, &uartParams);
    if (gGestMssMCB.loggingUartHandle == NULL)
    {
        System_printf("Error: MMWDemoMSS Unable to open the Logging UART Instance\n");
        return;
    }
#endif
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
void GestDemo_mssInitTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    Task_Params         taskParams;
    Semaphore_Params    semParams;
    Mailbox_Config      mboxCfg;
    Error_Block         eb;

    /* Debug Message: */
    System_printf("Debug: MMWDemoMSS Launched the Initialization Task\n");

    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/
    /* Initialize the GPIO */
    GPIO_init();

    /**********************************************************************
     * Setup the PINMUX:
     * - GPIO Output: Configure pin K13 as GPIO_2 output
     **********************************************************************/
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINK13_PADAZ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINK13_PADAZ, SOC_XWR16XX_PINK13_PADAZ_GPIO_2);

    /**********************************************************************
     * Setup the DS3 LED on the EVM connected to GPIO_2
     **********************************************************************/
    GPIO_setConfig (SOC_XWR16XX_GPIO_2, GPIO_CFG_OUTPUT);

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_MSS);

    /*****************************************************************************
     * Open & configure the drivers:
     *****************************************************************************/
    GestDemo_commModeSetup();
    
    /*****************************************************************************
     * Creating communication channel between MSS & DSS
     *****************************************************************************/

    /* Create a binary semaphore which is used to handle mailbox interrupt. */
    Semaphore_Params_init(&semParams);
    semParams.mode             = Semaphore_Mode_BINARY;
    gGestMssMCB.mboxSemHandle = Semaphore_create(0, &semParams, NULL);

    /* Setup the default mailbox configuration */
    Mailbox_Config_init(&mboxCfg);

    /* Setup the configuration: */
    mboxCfg.chType       = MAILBOX_CHTYPE_MULTI;
    mboxCfg.chId         = MAILBOX_CH_ID_0;
    mboxCfg.writeMode    = MAILBOX_MODE_BLOCKING;
    mboxCfg.readMode     = MAILBOX_MODE_CALLBACK;
    mboxCfg.readCallback = &GestDemo_mboxCallback;

    /* Initialization of Mailbox Virtual Channel  */
    gGestMssMCB.peerMailbox = Mailbox_open(MAILBOX_TYPE_DSS, &mboxCfg, &errCode);
    if (gGestMssMCB.peerMailbox == NULL)
    {
        /* Error: Unable to open the mailbox */
        System_printf("Error: Unable to open the Mailbox to the DSS [Error code %d]\n", errCode);
        return;
    }

    /* Remote Mailbox is operational: Set the DSS Link State */
    SOC_setMMWaveMSSLinkState (gGestMssMCB.socHandle, 1U, &errCode);

    /* Create task to handle mailbox messages */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    Task_create(GestDemo_mboxReadTask, &taskParams, NULL);

    /*****************************************************************************
     * Create Event to handle mmwave callback and system datapath events 
     *****************************************************************************/    
    Error_init(&eb);
    gGestMssMCB.eventHandleNotify = Event_create(NULL, &eb);
    if (gGestMssMCB.eventHandleNotify == NULL) 
    {
        GestDemo_mssAssert(0);
        return ;
    }

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequisite and always needs to be invoked. */
    while (1)
    {
        volatile int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = SOC_isMMWaveDSSOperational (gGestMssMCB.socHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the mmWave control module */
            printf ("Error: MMWDemoMSS mmWave Control Synchronization failed [Error code %d]\n", errCode);
            return;
        }
        if (syncStatus == 1)
        {
            /* Synchronization achieved: */
            break;
        }
        /* Sleep and poll again: */
        //MmwDemo_sleep();
    }  
    
    /* Start CLI to get command from user */
    GestDemo_CLIInit();

    /*****************************************************************************
     * Benchmarking Count init
     *****************************************************************************/
    /* Configure banchmark counter */
    Pmu_configureCounter(0, 0x11, FALSE);
    Pmu_startCounter(0);
   
    return;
}

/**
 *  @b Description
 *  @n
 *     Function to sleep the R4F using WFI (Wait For Interrupt) instruction. 
 *     When R4F has no work left to do,
 *     the BIOS will be in Idle thread and will call this function. The R4F will
 *     wake-up on any interrupt (e.g chirp interrupt).
 *
 *  @retval
 *      Not Applicable.
 */
void GestDemo_sleep(void)
{
    /* issue WFI (Wait For Interrupt) instruction */
    asm(" WFI ");
}

/**
 *  @b Description
 *  @n
 *      Send MSS assert information through CLI.
 */
void _GestDemo_mssAssert(int32_t expression, const char *file, int32_t line)
{
    if (!expression) {
        Dbg_write ("MSS Exception: %s, line %d.\n",file,line);
    }
}

/**
 *  @b Description
 *  @n
 *      Entry point into the Millimeter Wave Demo
 *
 *  @retval
 *      Not Applicable.
 */
int main (void)
{
    Task_Params     taskParams;
    int32_t         errCode;
    SOC_Cfg         socCfg;

#if 0
    Cycleprofiler_init(); // initializing
#endif
    /* Initialize the ESM: */
    ESM_init(0U); //don't clear errors as TI RTOS does it

    /* Initialize and populate the demo MCB */
    memset ((void*)&gGestMssMCB, 0, sizeof(GestDemo_MCB));

    /* Initialize the SOC configuration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    gGestMssMCB.socHandle   = SOC_init (&socCfg, &errCode);
    if (gGestMssMCB.socHandle  == NULL)
    {
        System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the DEMO configuration: */
    gGestMssMCB.cfg.sysClockFrequency = MSS_SYS_VCLK;
    gGestMssMCB.cfg.loggingBaudRate   = 921600;
    gGestMssMCB.cfg.commandBaudRate   = 115200;
#if !(STRING_OUTPUT_ONLY)
    /* Fill up gesture ID structure  to send over UART */
    memset(&gestureIdMsg, 0, sizeof(GestDemo_gestIdMsg));

    gestureIdMsg.header.magicWord[0] = 0x0102;
    gestureIdMsg.header.magicWord[1] = 0x0304;
    gestureIdMsg.header.magicWord[2] = 0x0506;
    gestureIdMsg.header.magicWord[3] = 0x0708;
    gestureIdMsg.header.version      = 0x0101;
    gestureIdMsg.header.totalPacketLen = sizeof(GestDemo_output_message_header)+ 4 +4;
    gestureIdMsg.header.platform     = 0x1642;
#endif

    /* Debug Message: */
    System_printf ("**********************************************\n");
    System_printf ("Debug: Launching the Millimeter Wave Demo\n");
    System_printf ("**********************************************\n");

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.priority = 3;
    Task_create(GestDemo_mssInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}



void GestDemo_preprocessANNInputs(float FeaturevectorDSS[], float FeaturevectorMSS[])
{
    int i=0;
    float maxval = 0.0, minval = 1000.0;
    uint16_t maxpos=0, minpos=0;

    for(i=0; i<GESTURE_FEATURE_LENGTH; i++)
    {
        if(FeaturevectorDSS[INSTENERGY_FEATURE_START_OFFSET + i] > detect_feature_threshold) // compare instantaneous energy with threshold
        {
            FeaturevectorMSS[INSTENERGY_FEATURE_START_OFFSET + i] = FeaturevectorDSS[INSTENERGY_FEATURE_START_OFFSET + i]/INST_ENERGY_NORMFACTOR_dB ;
            FeaturevectorMSS[WTRANGE_FEATURE_START_OFFSET + i]    = FeaturevectorDSS[WTRANGE_FEATURE_START_OFFSET + i]/rangebin_stop;
            FeaturevectorMSS[WTDOPPLER_FEATURE_START_OFFSET + i]  = FeaturevectorDSS[WTDOPPLER_FEATURE_START_OFFSET + i]/posdopplerbin_stop;
            FeaturevectorMSS[MAXAZFREQ_FEATURE_START_OFFSET + i]  = FeaturevectorDSS[MAXAZFREQ_FEATURE_START_OFFSET + i]/NUM_ANGLE_BINS_ONE_SIDED;
            FeaturevectorMSS[MAXELFREQ_FEATURE_START_OFFSET + i]  = FeaturevectorDSS[MAXELFREQ_FEATURE_START_OFFSET + i]/NUM_ANGLE_BINS_ONE_SIDED;

        }
        else
        {
            FeaturevectorMSS[INSTENERGY_FEATURE_START_OFFSET + i] = 0.0;
            FeaturevectorMSS[WTRANGE_FEATURE_START_OFFSET + i]    = 1.0;
            FeaturevectorMSS[WTDOPPLER_FEATURE_START_OFFSET + i]  = 0.0;
            FeaturevectorMSS[MAXAZFREQ_FEATURE_START_OFFSET + i]  = 0;
            FeaturevectorMSS[MAXELFREQ_FEATURE_START_OFFSET + i]  = 0;

        }
        // update maxpos and minpos in wtdoppler features
        /*if(FeaturevectorMSS[WTDOPPLER_FEATURE_START_OFFSET + i] > maxval)
        {
            maxval = FeaturevectorMSS[WTDOPPLER_FEATURE_START_OFFSET + i];
            maxpos = i;
        }
        if(FeaturevectorMSS[WTDOPPLER_FEATURE_START_OFFSET + i] < minval)
        {
            minval = FeaturevectorMSS[WTDOPPLER_FEATURE_START_OFFSET + i];
            minpos = i;
        }*/
        // copy the azdopp corr
        FeaturevectorMSS[AZDOPPCORR_FEATURE_START_OFFSET + i] = FeaturevectorDSS[AZDOPPCORR_FEATURE_START_OFFSET + i];
    }

    for(i=0; i<GESTURE_FEATURE_LENGTH; i++)
    {
        // update maxpos and minpos in wtdoppler features
        if(FeaturevectorMSS[WTDOPPLER_FEATURE_START_OFFSET + i] > maxval)
        {
            maxval = FeaturevectorMSS[WTDOPPLER_FEATURE_START_OFFSET + i];
            maxpos = i;
        }
        if(FeaturevectorMSS[WTDOPPLER_FEATURE_START_OFFSET + i] < minval)
        {
            minval = FeaturevectorMSS[WTDOPPLER_FEATURE_START_OFFSET + i];
            minpos = i;
        }
    }

    for (i = (maxpos + 1) % GESTURE_FEATURE_LENGTH; i != minpos; i = (i + 1) % GESTURE_FEATURE_LENGTH)
    {
        FeaturevectorMSS[MAXAZFREQ_FEATURE_START_OFFSET + i]  = 0.0;
        FeaturevectorMSS[MAXELFREQ_FEATURE_START_OFFSET + i]  = 0.0;

    }

}


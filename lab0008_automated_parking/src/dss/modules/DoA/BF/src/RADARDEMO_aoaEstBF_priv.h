/*!
 *  \file   RADARDEMO_aoaEstBF_priv.h
 *
 *  \brief   Header file for RADARDEMO_aoaEstBF_priv.c
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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
 *
*/
#ifndef RADARDEMO_AOAESTBF_PRIV_H
#define RADARDEMO_AOAESTBF_PRIV_H

//! \brief include file <swpform.h>
//!
#include <swpform.h>
#ifndef _TMS320C6600
#include <modules/utilities/radar_c674x.h>
#endif

#define RADARDEMO_AOAESTBF_MAX_NUM_PEAKS (10)    //!< Maximum number of detected peak per input vector internally during peak search.
#define RADARDEMO_AOAESTBF_VAREST_CONST 1        //!< A constant used in angle variance estimation

#define RADARDEMO_AOAESTBF_PIOVER180 (3.141592653589793/180.0)      //!< define the pi/180
#define RADARDEMO_AOAESTBF_PI        (3.141592653589793f)           //!< define pi


//!  \brief   Structure used in peak record
//!
typedef struct _RADARDEMO_aoAEstBF_peakRecord_t
{
    uint32_t peakLoc;          //!< peak location
    float  peakValue;          //!< peak value
    uint32_t peakWidth;        //!< peak gamma bandwidth
}RADARDEMO_aoAEstBF_peakRecord_t;


//!  \brief   Structure element of the list of descriptors for UL allocations.
//!
typedef struct _RADARDEMO_aoAEstBF_handle
{
    uint8_t       enableMultiPeakSearch;    /**< Flag to enable multiple peak search, if set to 1*/
	uint8_t      vmaxUnrollFlag;         /**<flag for angle correction for Vmax unrolling */
    uint32_t      nRxAnt;                   //!< number of receive antennas.
    float         estResolution;            //!< Estimation resolution in degree.
    float         estRange;                 //!< Range of the estimation, from -estRange to +estRange degrees
    uint32_t      steeringVecSize;          //!< size of -estRange:estResolution:estRange
    cplxf_t       * steeringVec;            //!< steering vector for angle: -estRange:estResolution:estRange, for nRxAnt antennas, must be aligned to 8-byte boundary
                                            //!< The steeringVec is arranged in ... degree0ant1 degree0ant2 degree0ant3 degree1ant1 degree1ant2 degree1ant3 ... fashion
                                            //!< Ant0's steeringVec is 1 for all angle possiblities, so we don't save them
    uint32_t      * scratchPad;             //!< Pointer to the scratch memory used in this function, must have length sizeof(uint32_t) * (5 * steeringVecSize), must be aligned to 8-byte boundary
    uint8_t       firstStageSearchStep;     //!< Search step for the first stage search, if 1, only 1 stage search (multi-stage search is disabled.
    float         maxOutputVar;             //!< Maximum variance (in degree) for the output estimation. Estimates with bigger variance range will not be reported.
    float         widthAdjust_3dB;          //!< adjust the gamme bandwidth to 3dB.
    float         gamma;                    //!< Peak survive threshold
    float         sidelobeLevel;            //!< the relatively peak power threshold used to skip the lower peak to avoid the sidelobe.
	float        angleEstBound;             /**< the boundary of the estimates. Angle estimates outside [-angleEstBound angleEstBound] will be filtered out.*/
} RADARDEMO_aoAEstBF_handle;

/*!
 *   \fn     RADARDEMO_aoaEstimationBFMultiPeak
 *
 *   \brief   Estimate the angle of arrival of each detected object using BF for 4 Rx antennas.
 *
 *   \param[in]    numAnt
 *               number of antenna
 *
 *   \param[in]    gamma
 *               threshold to detect a peak survives
 *
 *   \param[in]    sidelobeLevel_dB
 *               relative threshold to skip the lower peaks which potentially can be a sidelobe from the main peak
 *
 *   \param[in]    widthAdjust_3dB
 *               the peak search will find the gamma bandwidth of the peak, we adjust the gamma bandwidth
 *               to get the 3dB bandwidth of a detected peak
 *
 *   \param[in]    inputSNR
 *               the signal to noise ratio in linear format for the input signal samples
 *
 *   \param[in]    estResolution
 *               the angle resolution, used for variance estimation
 *
 *   \param[in]    sigIn
 *               input antenna samples.
 *
 *   \param[in]    steeringVec
 *               Pointer to steering vector.
 *
 *   \param[in]    steeringVecSize
 *               Size of the steering vector.
 *
 *   \param[in]    scratchPad
 *               Scratch memory.
 *
 *   \param[out]    estVar
 *               Output variance of the detected angles.
 *
 *   \param[out]    angleEst
 *               Output angle estimates.
 *
 *   \ret        Number of output angle estimates, it can be zero,
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
int32_t  RADARDEMO_aoaEstimationBFMultiPeak(
                            IN  uint8_t numAnt,
                            IN  float gamma,
                            IN  float sidelobeLevel,
                            IN  float widthAdjust_3dB,
                            IN  float inputSNR,
                            IN  float estResolution,
                            IN  cplxf_t * sigIn,
                            IN  cplxf_t * steeringVec,
                            IN  int32_t steeringVecSize,
                            IN  float   * scratchPad,
                            OUT int32_t   * peakInd,
                            OUT float   * peakVal,
                            OUT float     * estVar,
                            OUT int32_t   * angleEst);

/*!
 *   \fn     RADARDEMO_aoaEstimationBFSinglePeak
 *
 *   \brief   Estimate the angle of arrival of each detected object using BF for 4 Rx antennas.
 *
 *   \param[in]    numAnt
 *               number of antenna
 *
 *   \param[in]    noise
 *               Input noise of total numAnt.
 *
 *   \param[in]    estResolution
 *               the angle resolution, used for variance estimation
 *
 *   \param[in]    sigIn
 *               input antenna samples.
 *
 *   \param[in]    steeringVec
 *               Pointer to steering vector.
 *
 *   \param[in]    steeringVecSize
 *               Size of the steering vector.
 *
 *   \param[in]    scratchPad
 *               Scratch memory.
 *
 *   \param[out]    estVar
 *               Output variance of the detected angles.
 *
 *   \param[out]    angleEst
 *               Output angle estimates.
 *
 *   \ret        Number of output angle estimates, it can be zero,
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
int32_t  RADARDEMO_aoaEstimationBFSinglePeak(
                            IN  uint8_t numAnt,
                            IN  float noise,
                            IN  float estResolution,
                            IN  cplxf_t * sigIn,
                            IN  cplxf_t * steeringVec,
                            IN  int32_t steeringVecSize,
                            IN  float   * scratchPad,
                            OUT float   * peakVal,
                            OUT float     * estVar,
                            OUT int32_t   * angleEst);

// calculate one point of BF spectrum
void  RADARDEMO_aoaEstimationBFOnePoint(
                            IN  uint8_t numAnt,
                            IN  cplxf_t * sigIn,
                            IN  cplxf_t * steeringVec,
                            OUT cplxf_t * outValue);
#endif //RADARDEMO_AOAESTBF_PRIV_H


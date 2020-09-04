/*!
 *  \file   RADARDEMO_aoaEstimationBF.h
 *
 *  \brief   Header file for RADARDEMO_aoaEstimationBF module
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

#ifndef RADARDEMO_AOAESTBF_H
#define RADARDEMO_AOAESTBF_H


//! \brief include file <swpform.h>
//!
#include "swpform.h"
//! \brief include file <radarOsal_malloc.h>
//!
#include <modules/utilities/radarOsal_malloc.h>
#include <modules/DoA/common/api/RADARDEMO_aoaEst_commonDef.h>


//!  \brief   Error code for BF AoA estimation module.
//!
typedef enum
{
    RADARDEMO_AOABF_NO_ERROR = 0,                   /**< no error */
    RADARDEMO_AOABF_ANTSPACE_NOTSUPPORTED,          /**< normalized antenna spacing non supported */
    RADARDEMO_AOABF_NUMANT_NOTSUPPORTED,            /**< number of antennas not supported */
    RADARDEMO_AOABF_FAIL_ALLOCATE_HANDLE,           /**< RADARDEMO_aoAEstBF_create failed to allocate handle */
    RADARDEMO_AOABF_FAIL_ALLOCATE_LOCALINSTMEM, /**< RADARDEMO_aoAEstBF_create failed to allocate memory for buffers in local instance  */
    RADARDEMO_AOABF_INOUTPTR_NOTCORRECT,         /**< input and/or output buffer for RADARDEMO_aoAEstBF_run are either NULL, or not aligned properly  */
    RADARDEMO_AOABF_NUM_PEAK_EXCEED_MAX              /**< the number of detected angle exceed the max value */
} RADARDEMO_aoAEstBF_errorCode;


//!  \brief   Structure element of the list of descriptors for RADARDEMO_aoAEstBF configuration.
//!
typedef struct _RADARDEMO_aoAEstBF_config_
{
    uint8_t       enableMultiPeakSearch;    /**< Flag to enable multiple peak search, if set to 1*/
	uint8_t      vmaxUnrollFlag;         /**<flag for angle correction for Vmax unrolling */
    uint8_t       * antSpacing;             /**< antenna spacing in unit of lambda/2, size of nRxAnt, only support uniform linear array*/
    uint32_t      nRxAnt;                   /**< number of receive antennas, only support 4 and 8 antennas now!!!.*/
    float        estResolution;             /**< Estimation resolution in degree.*/
    float        estRange;                  /**< Range of the estimation, from -estRange to +estRange degrees*/
    float        maxOutputVar;              /**< Maximum variance range (in degree) for the output estimation. Estimates with bigger confidence range will not be reported.*/
    float        sidelobeLevel_dB;          /**< the relatively peak power threshold used to skip the lower peak to avoid the sidelobe.*/
	float        angleEstBound;             /**< the boundary of the estimates. Angle estimates outside [-angleEstBound angleEstBound] will be filtered out.*/
} RADARDEMO_aoAEstBF_config;



/*!
 *   \fn     RADARDEMO_aoaEstimationBF_create
 *
 *   \brief   Create and initialize RADARDEMO_aoaEstimationBF module.
 *
 *   \param[in]    moduleConfig
 *
 *   \param[out]    errorCode
 *
 *   \ret     void pointer to the module handle
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern void * RADARDEMO_aoaEstimationBF_create(
                            IN  RADARDEMO_aoAEstBF_config * moduleConfig,
                            OUT RADARDEMO_aoAEstBF_errorCode * errorCode);

/*!
 *   \fn     RADARDEMO_aoaEstimationBF_delete
 *
 *   \brief   Delete RADARDEMO_aoaEstimationBF module.
 *
 *   \param[in]    handle
 *               Module handle.
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern void RADARDEMO_aoaEstimationBF_delete(
                            IN  void * handle);


/*!
 *   \fn     RADARDEMO_aoaEstimationBF_run
 *
 *   \brief   Estimate the angle of arrival of each detected object using BF.
 *
 *   \param[in]    handle
 *               Module handle.
 *
 *   \param[in]    input
 *               Input antenna signal and noise power for the detected object.
 *
 *   \param[out]    estOutput
 *               Pointer to the estimation output.
 *   \ret  error code
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern RADARDEMO_aoAEstBF_errorCode RADARDEMO_aoaEstimationBF_run(
                            IN  void * handle,
                            IN  RADARDEMO_aoAEst_input * input,
                            OUT RADARDEMO_aoAEst_output   * estOutput);


#endif //RADARDEMO_AOAESTBF_H


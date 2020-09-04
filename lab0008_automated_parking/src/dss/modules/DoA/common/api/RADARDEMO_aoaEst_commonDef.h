/*!
 *  \file   RADARDEMO_aoaEst_commonDef.h
 *
 *  \brief   Header file for data structures used in common for different AOA methods
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

#ifndef RADARDEMO_AOAEST_COMMONDEF_H
#define RADARDEMO_AOAEST_COMMONDEF_H

#define MAX_NUM_DETANGLE (2)    /**< Maximum number of output angles per input vector, 2 for current implementation of the algorithm.*/

#define RADARDEMO_AOAESTBF_MAX_NUM_DETANGLE (4)     /**< Maximum number of output angles per input vector.*/

/**
 *  \struct   _RADARDEMO_aoAEst_input_
 *   {
 *  cplxf_t      * inputAntSamples;
 *  float         * inputNoisePow;
 *   }   RADARDEMO_aoAEst_input;
 *
 *  \brief   Structure element of the list of descriptors for RADARDEMO_aoAEstDML module output.
 *
 *
 */

typedef struct _RADARDEMO_aoAEst_input_
{
    cplxf_t      * inputAntSamples;             /**< input antenna samples, array of size number of antennas.*/
    float         inputNoisePow;            /**< Input noise power estimation, sum of all antennas.*/
} RADARDEMO_aoAEst_input;


/**
 *  \struct   _RADARDEMO_aoAEst_output_
 *   {
 *  uint32_t      numOutput;
 *  float         outputVar[MAX_NUM_DETANGLE];
 *  float         outputAngles[MAX_NUM_DETANGLE];
 *   }   RADARDEMO_aoAEst_output;
 *
 *  \brief   Structure element of the list of descriptors for RADARDEMO_aoAEstDML module output.
 *
 *
 */

typedef struct _RADARDEMO_aoAEst_output_
{
    uint32_t      numOutput;                /**< Number of output angles per input vector, maximum 2 for current implementation of the algorithm.*/
    float         outputVar[RADARDEMO_AOAESTBF_MAX_NUM_DETANGLE];           /**< Output confidence for the angle estimation, in degree.*/
    float         outputAngles[RADARDEMO_AOAESTBF_MAX_NUM_DETANGLE];        /**< Output angles.*/
    float         outputEAngles[RADARDEMO_AOAESTBF_MAX_NUM_DETANGLE];        /**< Output angles.*/
} RADARDEMO_aoAEst_output;

#endif

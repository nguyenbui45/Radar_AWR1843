/*!
 *  \file   RADARDEMO_aoaEstBF.c
 *  \brief   Estimate the angle of arrival using BF support multiple peaks.
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

#include <modules/DoA/BF/api/RADARDEMO_aoaEstBF.h>
#include "RADARDEMO_aoaEstBF_priv.h"
#include <math.h>

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif

uint16_t    aziAntennaMap[8] = {0, 1, 2, 3, 4, 5, 6, 7}; // TX1 and TX3
uint16_t    ele1AntennaMap[4] = {8, 9, 10, 11}; // TX2
uint16_t    ele0AntennaMap[4] = {2, 3, 4, 5};

//! \copydoc RADARDEMO_aoaEstimationBF_create
void    * RADARDEMO_aoaEstimationBF_create(
                            IN  RADARDEMO_aoAEstBF_config * moduleConfig,
                            OUT RADARDEMO_aoAEstBF_errorCode * errorCode)

{
    uint32_t    i, j;
    double       ftemp1, freal1, fimag1, frealJ, fimagJ;
    RADARDEMO_aoAEstBF_handle * handle;

    *errorCode  =   RADARDEMO_AOABF_NO_ERROR;

    /* Check antenna spacing, if it's not a uniform linear array, return with NULL */
    if (moduleConfig->antSpacing[0] != 0)
        *errorCode = RADARDEMO_AOABF_ANTSPACE_NOTSUPPORTED;
    for (i = 1; i < moduleConfig->nRxAnt; i++ )
    {
        if (moduleConfig->antSpacing[i] - moduleConfig->antSpacing[i-1] != 1)
            *errorCode = RADARDEMO_AOABF_ANTSPACE_NOTSUPPORTED;
    }
    /* Check number of antenna , only supporting up to 8 antenna at this point */
    if (moduleConfig->nRxAnt > 8)
        *errorCode = RADARDEMO_AOABF_NUMANT_NOTSUPPORTED;

    if (*errorCode > RADARDEMO_AOABF_NO_ERROR)
        return (NULL);

    handle              =   (RADARDEMO_aoAEstBF_handle *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_aoAEstBF_handle), 1);
    if (handle == NULL)
    {
        *errorCode = RADARDEMO_AOABF_FAIL_ALLOCATE_HANDLE;
        return (handle);
    }

    handle->enableMultiPeakSearch      =   moduleConfig->enableMultiPeakSearch;
    handle->vmaxUnrollFlag             =   moduleConfig->vmaxUnrollFlag;
    handle->nRxAnt      =   moduleConfig->nRxAnt;
   	handle->estRange    =   90;  // hard-coded the range to [-90:90]
    handle->gamma       =   (float)(pow(10, 0.2/10.0));  // hard-coded the gamme_dB = 0.2 dB
    handle->widthAdjust_3dB =  2.5;  // hard-coded the 3dB adjustment
    handle->estResolution   =   moduleConfig->estResolution;
    handle->maxOutputVar   =   moduleConfig->maxOutputVar;
    handle->sidelobeLevel   =   pow(10.f, -1*moduleConfig->sidelobeLevel_dB/10);
    handle->angleEstBound   =   moduleConfig->angleEstBound;

    handle->steeringVecSize =   (uint32_t) ((2.f * handle->estRange) / (handle->estResolution)) + 1;

    handle->steeringVec =   (cplxf_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->nRxAnt * handle->steeringVecSize * sizeof(cplxf_t), 8);
    if (handle->steeringVec == NULL)
    {
        *errorCode  =   RADARDEMO_AOABF_FAIL_ALLOCATE_LOCALINSTMEM;
        return (handle);
    }
    handle->scratchPad  =   (uint32_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 1, (5 * handle->steeringVecSize) * sizeof(uint32_t), 8);
    if (handle->scratchPad == NULL)
    {
        *errorCode  =   RADARDEMO_AOABF_FAIL_ALLOCATE_LOCALINSTMEM;
        return (handle);
    }

    // Ant0's steeringVec is 1 for all angle possiblities, so we don't save them
    for (i = 0; i < handle->steeringVecSize; i++)
    {
        ftemp1          =   (double) sin((-handle->estRange + (double) i * handle->estResolution) * (double)RADARDEMO_AOAESTBF_PIOVER180);
        freal1          =   (double) cos(-RADARDEMO_AOAESTBF_PI*ftemp1);
        fimag1          =   (double) sin(-RADARDEMO_AOAESTBF_PI*ftemp1);
        frealJ          =   freal1;
        fimagJ          =   fimag1;
        handle->steeringVec[(handle->nRxAnt - 1) * i + 0].real = (float)frealJ;
        handle->steeringVec[(handle->nRxAnt - 1) * i + 0].imag = (float)fimagJ;
        for (j = 2; j < handle->nRxAnt; j++)
        {
            ftemp1      =   frealJ;
            frealJ      =   frealJ * freal1 - fimagJ * fimag1;
            fimagJ      =   ftemp1 * fimag1 + fimagJ * freal1;
            handle->steeringVec[(handle->nRxAnt - 1) * i + j - 1].real = (float)frealJ;
            handle->steeringVec[(handle->nRxAnt - 1) * i + j - 1].imag = (float)fimagJ;
        }
    }

    //printf("handle->steeringVecSize = %d\n", handle->steeringVecSize);
    //fwrite(handle->steeringVec, 4, handle->steeringVecSize * 6, testSteerVec);
    //fclose(testSteerVec);

    return((void *)handle);

}

//! \copydoc RADARDEMO_aoaEstimationBF_delete
void    RADARDEMO_aoaEstimationBF_delete(
                            IN  void * handle)
{
    RADARDEMO_aoAEstBF_handle *aoaEstBFInst;

    aoaEstBFInst    =   (RADARDEMO_aoAEstBF_handle *) handle;

    radarOsal_memFree(aoaEstBFInst->steeringVec, aoaEstBFInst->steeringVecSize * aoaEstBFInst->nRxAnt * sizeof(cplxf_t));
    radarOsal_memFree(aoaEstBFInst->scratchPad, 5* aoaEstBFInst->steeringVecSize * sizeof(uint32_t));

    radarOsal_memFree(handle, sizeof(RADARDEMO_aoAEstBF_handle));
}


//! \copydoc RADARDEMO_aoaEstimationBF_run
RADARDEMO_aoAEstBF_errorCode    RADARDEMO_aoaEstimationBF_run(
                            IN  void * handle,
                            IN  RADARDEMO_aoAEst_input * input,
                            OUT RADARDEMO_aoAEst_output   * estOutput)

{
    uint32_t     i, j, antID;
    RADARDEMO_aoAEstBF_handle *aoaEstBFInst;
    cplxf_t     * Rn, * inputSignal;
    float       * localScratch;
    int32_t       angleEst[RADARDEMO_AOAESTBF_MAX_NUM_PEAKS];
    float         estVar[RADARDEMO_AOAESTBF_MAX_NUM_PEAKS];
    RADARDEMO_aoAEstBF_errorCode errorCode = RADARDEMO_AOABF_NO_ERROR;
    float       signalP, snr, tempAngle;
	float       peakVal;
	int32_t		peakInd;

    aoaEstBFInst    =   (RADARDEMO_aoAEstBF_handle *) handle;

    //inputSignal     =   input->inputAntSamples;
    //if ( inputSignal == NULL)
    //    errorCode   =   RADARDEMO_AOABF_INOUTPTR_NOTCORRECT;
    if (  estOutput == NULL)
        errorCode   =   RADARDEMO_AOABF_INOUTPTR_NOTCORRECT;
    if (  aoaEstBFInst->scratchPad == NULL)
        errorCode   =   RADARDEMO_AOABF_INOUTPTR_NOTCORRECT;
    if ( aoaEstBFInst->steeringVec == NULL)
        errorCode   =   RADARDEMO_AOABF_INOUTPTR_NOTCORRECT;
    if (errorCode > RADARDEMO_AOABF_NO_ERROR)
        return (errorCode);

    Rn              =   (cplxf_t *) &aoaEstBFInst->scratchPad[0];
    inputSignal     =   (cplxf_t *) &aoaEstBFInst->scratchPad[2*aoaEstBFInst->nRxAnt];
    localScratch    =   (float *) &aoaEstBFInst->scratchPad[aoaEstBFInst->nRxAnt * (aoaEstBFInst->nRxAnt + 1) * 2];

    signalP = 0.0;
    for (i = 0; i < (int32_t) aoaEstBFInst->nRxAnt; i++ )
    {
        antID = aziAntennaMap[i];
        inputSignal[i] = input->inputAntSamples[antID];
        _amem8_f2(&Rn[i]) = _complex_conjugate_mpysp(_amem8_f2(&inputSignal[i]), _amem8_f2(&inputSignal[i]));
        signalP = signalP + Rn[i].real;
    }
    snr         =   signalP / input->inputNoisePow;

	if ( aoaEstBFInst->enableMultiPeakSearch)
		estOutput->numOutput = RADARDEMO_aoaEstimationBFMultiPeak(
                            aoaEstBFInst->nRxAnt,
                            aoaEstBFInst->gamma,
                            aoaEstBFInst->sidelobeLevel,
                            aoaEstBFInst->widthAdjust_3dB,
                            snr,
                            aoaEstBFInst->estResolution,
                            inputSignal,
                            aoaEstBFInst->steeringVec,
                            aoaEstBFInst->steeringVecSize,
                            localScratch,
							&peakInd,
							&peakVal,
                            estVar,
                            angleEst);
	else
		estOutput->numOutput = RADARDEMO_aoaEstimationBFSinglePeak(
                            aoaEstBFInst->nRxAnt,
                            input->inputNoisePow,
                            aoaEstBFInst->estResolution,
                            inputSignal,
                            aoaEstBFInst->steeringVec,
                            aoaEstBFInst->steeringVecSize,
                            localScratch,
							&peakVal,
                            estVar,
                            angleEst);
	
	
	if ( aoaEstBFInst->vmaxUnrollFlag == 1)
	{
		float		peakVal2;
		int32_t		peakInd2;
		int32_t     angleEst2[RADARDEMO_AOAESTBF_MAX_NUM_PEAKS];
		float       estVar2[RADARDEMO_AOAESTBF_MAX_NUM_PEAKS];

		for(i = (aoaEstBFInst->nRxAnt >> 1); i < aoaEstBFInst->nRxAnt; i++)
		{
			inputSignal[i].real = -inputSignal[i].real;
			inputSignal[i].imag = -inputSignal[i].imag;
		}
		if ( aoaEstBFInst->enableMultiPeakSearch)
			estOutput->numOutput = RADARDEMO_aoaEstimationBFMultiPeak(
								aoaEstBFInst->nRxAnt,
								aoaEstBFInst->gamma,
								aoaEstBFInst->sidelobeLevel,
								aoaEstBFInst->widthAdjust_3dB,
								snr,
								aoaEstBFInst->estResolution,
								inputSignal,
								aoaEstBFInst->steeringVec,
								aoaEstBFInst->steeringVecSize,
								localScratch,
								&peakInd2,
								&peakVal2,
								estVar2,
								angleEst2);
		else
			estOutput->numOutput = RADARDEMO_aoaEstimationBFSinglePeak(
								aoaEstBFInst->nRxAnt,
								input->inputNoisePow,
								aoaEstBFInst->estResolution,
								inputSignal,
								aoaEstBFInst->steeringVec,
								aoaEstBFInst->steeringVecSize,
								localScratch,
								&peakVal2,
								estVar2,
								angleEst2);

		estOutput->numOutput	=	1;
		if ( aoaEstBFInst->enableMultiPeakSearch == 0) 
		{
			peakInd2	=	0;
			peakInd		=	0;
		}
		if( peakVal2 > peakVal)
		{
			angleEst[0]		=	angleEst2[peakInd2];
			estVar[0]		=	estVar2[peakInd2];
		}
		else
		{
			angleEst[0]		=	angleEst[peakInd];
			estVar[0]		=	estVar[peakInd];
		}
	}

    // generate the new input data
    for(i = 0; i < 4; i++)
    {
        inputSignal[i] = input->inputAntSamples[ele1AntennaMap[i]];
        inputSignal[i+4] = input->inputAntSamples[ele0AntennaMap[i]];
    }

    i = 0;
    for (j = 0; j < estOutput->numOutput; j++)
    {
        //add the elevation angle estimation.
        cplxf_t aziOut, eleOut;
        float Wz, aAngle, eAngle;

	    RADARDEMO_aoaEstimationBFOnePoint(
	                            4,
	                            &inputSignal[0],
	                            &aoaEstBFInst->steeringVec[(aoaEstBFInst->nRxAnt - 1) * angleEst[j]],
	                            &eleOut);

        RADARDEMO_aoaEstimationBFOnePoint(
                                4, //aoaEstBFInst->nAntForEl,
                                &inputSignal[4],
                                &aoaEstBFInst->steeringVec[(aoaEstBFInst->nRxAnt - 1) * angleEst[j]],
                                &aziOut);

        //Wz = sin(eAngle);
        Wz = (float)(atan2(aziOut.imag * eleOut.real - aziOut.real * eleOut.imag,
                   aziOut.real * eleOut.real + aziOut.imag * eleOut.imag)/RADARDEMO_AOAESTBF_PI);


        // check variance threshold
        tempAngle        =   -(aoaEstBFInst->estRange) + (float) angleEst[j] * aoaEstBFInst->estResolution;
        if((tempAngle <= aoaEstBFInst->angleEstBound) && (tempAngle >= -aoaEstBFInst->angleEstBound)
            && (estVar[j] < aoaEstBFInst->maxOutputVar) )
        {
            if (i >= RADARDEMO_AOAESTBF_MAX_NUM_DETANGLE)
            {
                errorCode = RADARDEMO_AOABF_NUM_PEAK_EXCEED_MAX;
                break;
            }

            eAngle = (float)asin(Wz);
            aAngle = -tempAngle*RADARDEMO_AOAESTBF_PIOVER180;
            estOutput->outputEAngles[i]  =  eAngle * 57.30;
            //estOutput->outputAngles[i]  =  -tempAngle;
            estOutput->outputAngles[i]  =   57.30 * asin((float)(sin(aAngle))/(float)(cos(eAngle)));
            estOutput->outputVar[i]  =   estVar[j];
            i++;
         }
     }
     estOutput->numOutput    =   i;

    return (errorCode);
}


/*!
 *   \file   RADARDEMO_aoaEstBF_priv.c
 *
 *   \brief   Estimate the angle of arrival using BF.
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

#include "RADARDEMO_aoaEstBF_priv.h"
#include <math.h>
//#include <stdio.h>

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif

//#define UNOPTIMIZED_CODE

#ifdef UNOPTIMIZED_CODE

//! \brief          complex conjugage multipler
//!                   out = x'*y
//! \param[in] x
//! \param[in] y
//! \ret       multiplication output
//!
cplxf_t complex_conjugate_mpysp(cplxf_t x, cplxf_t y)
{
    cplxf_t out;
    out.real = x.real * y.real + x.imag * y.imag;
    out.imag = x.real * y.imag - x.imag * y.real;
    return(out);
}

float complex_abs(cplxf_t x)
{
    float out;
    out = x.real * x.real + x.imag * x.imag;
    return(out);
}

//! \brief          complex addition
//!                   out = x + y
//! \param[in] x
//! \param[in] y
//! \ret       addition output
//!
cplxf_t daddsp(cplxf_t x, cplxf_t y)
{
    cplxf_t out;
    out.real = x.real + y.real;
    out.imag = y.imag + x.imag;
    return(out);
}
#endif

//! \copydoc RADARDEMO_aoaEstimationBFMultiPeak
int32_t RADARDEMO_aoaEstimationBFMultiPeak(
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
                            OUT float   * estVar,
                            OUT int32_t * angleEst)
 {
    int32_t ind_obj;
    float * RESTRICT doaSpectrum;
    RADARDEMO_aoAEstBF_peakRecord_t * RESTRICT maxData;
    //float   * RESTRICT tempMetricBuf;

    int32_t extendLoc, ind_loc, maxLoc, maxLoc_r, numMax, bwidth, initStage;
    float minVal = (float)(3.0e+30); //INFINITY;
    float currentVal;
#ifdef UNOPTIMIZED_CODE
    float absMaxVal, spec;
    int32_t ind_ang, ind_ant;
    cplxf_t temp, tempSum;
#endif
    float peakThreshold;
    float maxVal;
    float peakWidth;
	float gamma_inv = 1.f/gamma;
    float snr, totPower = 0;
    int32_t locateMax = 0;
    int32_t runningInd;

    /*
     Solve: spectrum = A'*Rn*A = |A'x|.^2;
     where:
             A = [steeringVec(theta)]  is a nRxAnt by numAngles matrix
             Rn is the covariance matrix of the antenna signal
             x is the input signal vector
    */

    doaSpectrum  =   (float *) &scratchPad[0];
    maxData      =   (RADARDEMO_aoAEstBF_peakRecord_t *) &scratchPad[steeringVecSize];
    //tempMetricBuf    =   (float *) &scratchPad[steeringVecSize] + RADARDEMO_AOAESTBF_MAX_NUM_PEAKS * size(RADARDEMO_aoAEstBF_peakRecord_t)];

    /* Compute the spectrum and record the max */
#ifdef UNOPTIMIZED_CODE
	if(1)
    {
		absMaxVal = 0.0;
		for (ind_ang = 0; ind_ang < steeringVecSize; ind_ang ++)
		{
			tempSum.real = sigIn[0].real;
			tempSum.imag = sigIn[0].imag;
			for (ind_ant = 1; ind_ant < numAnt; ind_ant ++)
			{
				temp = complex_conjugate_mpysp(steeringVec[(numAnt - 1)*ind_ang + ind_ant - 1], sigIn[ind_ant]);
				tempSum = daddsp(tempSum, temp);
			}
			spec = complex_abs(tempSum);
			doaSpectrum[ind_ang] = spec;
			if (spec > absMaxVal)
			{
				/* Record the overall maximum value */
				absMaxVal = spec;
			}
		}
		peakThreshold = absMaxVal * sidelobeLevel;
    }
	else
#endif
    {
        int32_t 	i;
        float 		tempPow, totPower, maxPow;
    	__float2_t 	input0;
    	__float2_t 	* RESTRICT tempSteerVecPtr;

        /* Compute the spectrum and record the max */
        totPower 	= 	0.f;
    	maxPow 		=	0.f;
    	if ( numAnt == 4 )
    	{
    		__float2_t input1, input2, input3, f2temp1;

    		input0 			=	_amem8_f2(&sigIn[0]);
    		input1 			=	_amem8_f2(&sigIn[1]);
    		input2 			=	_amem8_f2(&sigIn[2]);
    		input3 			=	_amem8_f2(&sigIn[3]);
    		tempSteerVecPtr =	(__float2_t *) steeringVec;

    		for (i = 0; i < steeringVecSize; i++ )
    		{
    			f2temp1		=	_daddsp(input0, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input1));
    			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input2));
    			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input3));
    			f2temp1		=	_dmpysp(f2temp1, f2temp1);
    			tempPow		=	_hif2(f2temp1) + _lof2(f2temp1);
    			doaSpectrum[i]	=	tempPow;
    			totPower	+=	tempPow;
    			if (tempPow > maxPow)
    			{
    				maxPow	=	tempPow;
    			}
    		}
    	}
    	else if ( numAnt == 8 )
    	{
    		__float2_t input1, input2, input3, input4, input5, input6, input7, f2temp1;

    		input0 			=	_amem8_f2(&sigIn[0]);
    		input1 			=	_amem8_f2(&sigIn[1]);
    		input2 			=	_amem8_f2(&sigIn[2]);
    		input3 			=	_amem8_f2(&sigIn[3]);
    		input4 			=	_amem8_f2(&sigIn[4]);
    		input5 			=	_amem8_f2(&sigIn[5]);
    		input6 			=	_amem8_f2(&sigIn[6]);
    		input7 			=	_amem8_f2(&sigIn[7]);
    		tempSteerVecPtr =	(__float2_t *) steeringVec;

    		for (i = 0; i < steeringVecSize; i++ )
    		{
    			f2temp1		=	_daddsp(input0, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input1));
    			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input2));
    			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input3));
    			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input4));
    			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input5));
    			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input6));
    			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input7));
    			f2temp1		=	_dmpysp(f2temp1, f2temp1);
    			tempPow		=	_hif2(f2temp1) + _lof2(f2temp1);
    			doaSpectrum[i]	=	tempPow;
    			totPower	+=	tempPow;
    			if (tempPow > maxPow)
    			{
    				maxPow	=	tempPow;
    			}
    		}
    	}
#if 0
    	else
    	{
    		int32_t j;
    		__float2_t input1, f2temp1;

    		input0 			=	_amem8_f2(&sigIn[0]);
    		tempSteerVecPtr =	(__float2_t *) steeringVec;

    		for (i = 0; i < steeringVecSize; i++ )
    		{
    			input1	=	_amem8_f2(&sigIn[1]);
    			f2temp1	=	_daddsp(input0, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr), input1));
    			tempSteerVecPtr++;
    			for (j = 2; j < numAnt; j++ )
    			{
    				input1	=	_amem8_f2(&sigIn[j]);
    				f2temp1	=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr), input1));
    				tempSteerVecPtr++;
    			}
    			f2temp1		=	_dmpysp(f2temp1, f2temp1);
    			tempPow		=	_hif2(f2temp1) + _lof2(f2temp1);
    			doaSpectrum[i]	=	tempPow;
    			totPower	+=	tempPow;
    			if (tempPow > maxPow)
    			{
    				maxPow	=	tempPow;
    			}
    		}

    	}
#endif
		peakThreshold = maxPow * sidelobeLevel;
		*peakVal = maxPow;
    }

    // Multiple Peak search
    runningInd = 0;
    numMax = 0;
    extendLoc = 0;
    initStage = 1;
    maxVal = 0.0;
    totPower = 0;
    while (runningInd < (steeringVecSize + extendLoc))
    {
        runningInd++;		
		if (runningInd >= steeringVecSize)
            ind_loc = runningInd - steeringVecSize;
		else
			ind_loc = runningInd;

        currentVal = doaSpectrum[ind_loc];

        /* Record the local maximum value and its location */
        if (currentVal > maxVal)
        {
            maxVal = currentVal;
            maxLoc = ind_loc;
            maxLoc_r = runningInd;
        }

        /* Record the local minimum value */
        if (currentVal < minVal)
        {
            minVal = currentVal;
        }

        if (locateMax == 1)
        {
            // peak search
            if (currentVal < maxVal*gamma_inv)
            {
                // ignore the lower peaks to avoid detect sidelobe of the main peaks.
                if (maxVal >= peakThreshold)
                {
                    // Assign maximum value only if the value has fallen below the max by
                    // gamma, thereby declaring that the max value was a peak
                    bwidth = runningInd - maxLoc_r;
                    maxData[numMax].peakLoc   = maxLoc;
                    maxData[numMax].peakValue = maxVal;
                    maxData[numMax].peakWidth = bwidth;
                    totPower += maxVal;
                    numMax++;
                }
                minVal = currentVal;
                locateMax = 0;
            }
        }
        else
        {
            // bottom search
            if (currentVal > minVal * gamma)
            {
                // Assign minimum value if the value has risen above the min by
                // gamma, thereby declaring that the min value was a valley
                locateMax = 1;
                maxVal = currentVal;
                if (initStage == 1)
                {
                    // all the samples before qualifing for first peak search is attached to
                    // the end of the spectrum array to solve the edge problem.  For this purpose
                    // the estRange is alway fixed to [-90:90]
                    extendLoc = runningInd + 1;
                    initStage = 0;
                }
            }
        }
    }

    for (ind_obj = 0; ind_obj < numMax; ind_obj ++)
    {
        // variance estimation
        //    estVar(ind) = 2*angleStep*peakWidth_gamma(ind)*widthAdjust_3dB(peakLoc(ind))/...
        //        (km * sqrt(2*10^(estSNR/10)*maxData(ind, 2)/totPower));

        peakWidth = 2*estResolution*maxData[ind_obj].peakWidth*widthAdjust_3dB;
        snr = inputSNR*maxData[ind_obj].peakValue / totPower;
        estVar[ind_obj] =  (float)(peakWidth / ((float)(RADARDEMO_AOAESTBF_VAREST_CONST) * sqrt(2*snr)));
        angleEst[ind_obj] = maxData[ind_obj].peakLoc;
		if (maxData[ind_obj].peakValue == *peakVal)
			*peakInd = ind_obj;
    }

    return(numMax);
}



int32_t  RADARDEMO_aoaEstimationBFSinglePeak(
                            IN  uint8_t numAnt,
                            IN  float   noise,
                            IN  float   estResolution,
                            IN  cplxf_t * sigIn,
                            IN  cplxf_t * steeringVec,
                            IN  int32_t steeringVecSize,
                            IN  float   * scratchPad,
                            OUT float   * peakVal,
                            OUT float   * estVar,
                            OUT int32_t * angleEst)
 {
    int32_t 	i, maxInd, leftInd, rightInd, temp3dBSpan;
    float 		* RESTRICT doaSpectrum;
    float 		tempPow, totPower, signalPower, maxPow, thre3dB, tempVar, inputPower, tempVarSqrInv, tempInterpol;
	__float2_t 	input0;
	__float2_t 	* RESTRICT tempSteerVecPtr;
	__float2_t 	* RESTRICT tempInPtr;
    /*
     Solve: spectrum = A'*Rn*A = |A'x|.^2;
     where:
             A = [steeringVec(theta)]  is a nRxAnt by numAngles matrix
             Rn is the covariance matrix of the antenna signal
             x is the input signal vector
    */

    doaSpectrum  	=   (float *) &scratchPad[0];
	
	tempInPtr		=	(__float2_t *) sigIn;
	inputPower		=	0.f;
	for (i = 0; i < numAnt; i++ )
	{
		input0		=	_amem8_f2(tempInPtr++);
		input0		=	_dmpysp(input0, input0);
		inputPower	+=	_hif2(input0) + _lof2(input0);
	}

    /* Compute the spectrum and record the max */
    totPower 	= 	0.f;
	maxPow 		=	0.f;
	maxInd		=	-1;
	if ( numAnt == 4 )
	{
		__float2_t input1, input2, input3, f2temp1;
		
		input0 			=	_amem8_f2(&sigIn[0]);
		input1 			=	_amem8_f2(&sigIn[1]);
		input2 			=	_amem8_f2(&sigIn[2]);
		input3 			=	_amem8_f2(&sigIn[3]);
		tempSteerVecPtr =	(__float2_t *) steeringVec;
		
		for (i = 0; i < steeringVecSize; i++ )
		{
			f2temp1		=	_daddsp(input0, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input1));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input2));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input3));
			f2temp1		=	_dmpysp(f2temp1, f2temp1);
			tempPow		=	_hif2(f2temp1) + _lof2(f2temp1);
			doaSpectrum[i]	=	tempPow;
			totPower	+=	tempPow;
			if (tempPow > maxPow)
			{
				maxPow	=	tempPow;
				maxInd 	=	i;
			}
		}
	}
	else if ( numAnt == 8 )
	{
		__float2_t input1, input2, input3, input4, input5, input6, input7, f2temp1;
		
		input0 			=	_amem8_f2(&sigIn[0]);
		input1 			=	_amem8_f2(&sigIn[1]);
		input2 			=	_amem8_f2(&sigIn[2]);
		input3 			=	_amem8_f2(&sigIn[3]);
		input4 			=	_amem8_f2(&sigIn[4]);
		input5 			=	_amem8_f2(&sigIn[5]);
		input6 			=	_amem8_f2(&sigIn[6]);
		input7 			=	_amem8_f2(&sigIn[7]);
		tempSteerVecPtr =	(__float2_t *) steeringVec;
		
		for (i = 0; i < steeringVecSize; i++ )
		{
			f2temp1		=	_daddsp(input0, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input1));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input2));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input3));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input4));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input5));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input6));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input7));
			f2temp1		=	_dmpysp(f2temp1, f2temp1);
			tempPow		=	_hif2(f2temp1) + _lof2(f2temp1);
			doaSpectrum[i]	=	tempPow;
			totPower	+=	tempPow;
			if (tempPow > maxPow)
			{
				maxPow	=	tempPow;
				maxInd 	=	i;
			}
		}
	}
#if 0
	else
	{
		int32_t j;
		__float2_t input1, f2temp1;
		
		input0 			=	_amem8_f2(&sigIn[0]);
		tempSteerVecPtr =	(__float2_t *) steeringVec;
		
		for (i = 0; i < steeringVecSize; i++ )
		{
			input1	=	_amem8_f2(&sigIn[1]);
			f2temp1	=	_daddsp(input0, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr), input1));
			tempSteerVecPtr++;
			for (j = 2; j < numAnt; j++ )
			{
				input1	=	_amem8_f2(&sigIn[j]);
				f2temp1	=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr), input1));
				tempSteerVecPtr++;
			}
			f2temp1		=	_dmpysp(f2temp1, f2temp1);
			tempPow		=	_hif2(f2temp1) + _lof2(f2temp1);
			doaSpectrum[i]	=	tempPow;
			totPower	+=	tempPow;
			if (tempPow > maxPow)
			{
				maxPow	=	tempPow;
				maxInd 	=	i;
			}
		}
		
	}
#endif
	*peakVal = maxPow;
	thre3dB		=	maxPow * 0.5f;
	signalPower	=	0.f;
	leftInd     =	maxInd - 1;
	while ((doaSpectrum[leftInd] >= thre3dB) && (leftInd != maxInd))
	{
		signalPower 	+= 	doaSpectrum[leftInd];
		leftInd--;
		if (leftInd < 0)
			leftInd		=	steeringVecSize - 1;
	}
	
	rightInd     =	maxInd + 1;
	while ((doaSpectrum[rightInd] >= thre3dB) && (rightInd != maxInd))
	{
		signalPower 	+= 	doaSpectrum[rightInd];
		rightInd++;
		if (rightInd == steeringVecSize)
			rightInd	=	0;
	}
    temp3dBSpan			=	rightInd - 1 - (leftInd + 1) + 1;
	if (temp3dBSpan <= 0)
		temp3dBSpan		+=  steeringVecSize;
	tempVarSqrInv		=	2.f * (float)RADARDEMO_AOAESTBF_VAREST_CONST * (float)RADARDEMO_AOAESTBF_VAREST_CONST * inputPower * (float) numAnt * signalPower;
	tempInterpol		=	_rcpsp(noise * totPower);
	tempInterpol		=	tempInterpol * (2.f - noise * totPower * tempInterpol);
	tempInterpol		=	tempInterpol * (2.f - noise * totPower * tempInterpol);

	tempVarSqrInv		*=	tempInterpol;	
	tempVar 			= 	estResolution * (float) temp3dBSpan;
	tempInterpol		=	_rsqrsp(tempVarSqrInv);
    tempInterpol		=   tempInterpol * (1.5f - (0.5f * tempVarSqrInv) * tempInterpol * tempInterpol);
    tempInterpol		=   tempInterpol * (1.5f - (0.5f * tempVarSqrInv) * tempInterpol * tempInterpol);
	tempVar				*=	tempInterpol;
	
	*estVar 			=  	tempVar;
	*angleEst 			= 	maxInd;
    return(1);
}


void  RADARDEMO_aoaEstimationBFOnePoint(
                            IN  uint8_t numAnt,
                            IN  cplxf_t * sigIn,
                            IN  cplxf_t * steeringVec,
                            OUT cplxf_t * outValue)
 {
    __float2_t  * RESTRICT tempSteerVecPtr;
    __float2_t doaZigangTemp;

    doaZigangTemp    =   _amem8_f2(&sigIn[0]);
    if ( numAnt == 4 )
    {
        tempSteerVecPtr =   (__float2_t *) steeringVec;
        doaZigangTemp    =   _daddsp(doaZigangTemp, _complex_mpysp(_amem8_f2(tempSteerVecPtr++), _amem8_f2(&sigIn[1])));
        doaZigangTemp    =   _daddsp(doaZigangTemp, _complex_mpysp(_amem8_f2(tempSteerVecPtr++), _amem8_f2(&sigIn[2])));
        doaZigangTemp    =   _daddsp(doaZigangTemp, _complex_mpysp(_amem8_f2(tempSteerVecPtr++), _amem8_f2(&sigIn[3])));
    }
    _amem8_f2(outValue)   =   doaZigangTemp;
 }

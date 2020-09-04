/*! 
 *  \file   RADARDEMO_detectionCFAR_priv.c
 *
 *  \brief   CFAR functions.
 *
 *  Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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


#include "RADARDEMO_detectionCFAR_priv.h"

#ifdef _TMS320C6X
#include "c6x.h"
#endif

#include "math.h"

/* Log table */
double ti_math_logtable[8] = {
   0.0000000000,             
  -0.1177830356,             
  -0.2231435513,             
  -0.3184537311,             
  -0.4054651081,             
  -0.4855078157,             
  -0.5596157879,             
  -0.6286086594              
};



/*! 
   \fn     RADARDEMO_detectionCFAR_CA
 
   \brief   Performs peak search and calculation of range and speed of detected object using CA-CFAR.
  
   \param[in]    InputPower
               Input power profile from integration.
 
   \param[in]    detectionCFARInst
               Pointer to input detection handle.
 
   \param[out]    rangeInd
               Pointer to the output range indices to detected objects. 

   \param[out]    dopplerInd
               Pointer to the output Doppler indices to detected objects. 
			   
   \param[out]    rangeEst
               Pointer to the output range estimation to detected objects. 
			   
   \param[out]    dopplerEst
               Pointer to the output Doppler estimation to detected objects. 
			   
   \param[out]    snrEst
               Pointer to the output linear SNR estimation to detected objects.
			   
   \param[out]    noise
               Pointer to the output noise estimation detected objects. 
			   
   \ret       number of objects detected.
   
   \pre       none
 
   \post      none
  
 
 */

int32_t	RADARDEMO_detectionCFAR_CA(
							IN float   **InputPower,
                            IN  RADARDEMO_detectionCFAR_handle *detectionCFARInst,
							OUT  uint16_t * rangeInd, 
							OUT  uint16_t * dopplerInd,
							OUT  float    * rangeEst,
							OUT  float    * dopplerEst,
							OUT  float    * snrEst,
							OUT  float    * noise)
{

	int32_t		i, j, k;
	float       * RESTRICT powerPtr;
	double		totalPower;
	float		scale, relativeThr, threshold;
	__float2_t  f2temp1;
	uint32_t    detected, sign, signmask = 0x80000000, tempDetected;
	int32_t     mod1Dleft, mod1Dright, mod2Dleft, mod2Dright;
	int16_t		*tempRangeIndex;
	float		*tempNoise;
	int32_t		indexLeftIn, indexLeftOut, indexRightIn, indexRightOut;
	uint32_t    indexMask;

	tempRangeIndex	=	(int16_t *)detectionCFARInst->scratchPad;
	tempNoise		=	(float *) &detectionCFARInst->scratchPad[detectionCFARInst->fft1DSize/2];
	relativeThr		=	detectionCFARInst->relThr;
	scale			=	1.f/(float)(2 * detectionCFARInst->searchWinSizeRange);
	detected		=	0;
	indexMask 		=	detectionCFARInst->fft1DSize - 1;

	/* peak search */
	if (0)
	//if (detectionCFARInst->searchWinSize == 16)
	{ // optimized for search window size 16 -- Todo
	}
	else
	{// not optimized for search window size other than 16
		for (i = 0; i < (int32_t) detectionCFARInst->fft2DSize; i++)
		{
			tempDetected	=	0;
			powerPtr	=	(float *) InputPower[i];
			j			=	0;
			totalPower  =	0.0;
			//calculate DC but don't search peak
			//leftside of DC
			for (k = (int32_t)(detectionCFARInst->fft1DSize - detectionCFARInst->guardSizeRange - detectionCFARInst->searchWinSizeRange); k < (int32_t) (detectionCFARInst->fft1DSize - detectionCFARInst->guardSizeRange); k+=2 )
			{
				f2temp1	=	_mem8_f2(&powerPtr[k]);
				totalPower	+= (double)_hif2(f2temp1);
				totalPower	+= (double)_lof2(f2temp1);
			}
			//right side of DC
			for (k = (int32_t)detectionCFARInst->guardSizeRange + 1; k < (int32_t) detectionCFARInst->searchWinSizeRange + detectionCFARInst->guardSizeRange + 1; k+=2 )
			{
				f2temp1	=	_mem8_f2(&powerPtr[k]);
				totalPower	+= (double)_hif2(f2temp1);
				totalPower	+= (double)_lof2(f2temp1);
			}

			indexLeftOut 	=	1-(int32_t)(detectionCFARInst->searchWinSizeRange + detectionCFARInst->guardSizeRange + 1);
			indexLeftIn    	=   1 - (int32_t)detectionCFARInst->guardSizeRange - 1;
			indexRightOut	=	1 + (int32_t)detectionCFARInst->guardSizeRange;
			indexRightIn	=	1 + (int32_t)(detectionCFARInst->searchWinSizeRange + detectionCFARInst->guardSizeRange);
			for (j = 1; j < (int32_t)detectionCFARInst->fft1DSize; j++ )
			{
				// slide the window
				totalPower	-=	(double)powerPtr[(uint32_t)indexLeftOut & indexMask];
				totalPower	+=	(double)powerPtr[(uint32_t)indexLeftIn & indexMask];
				totalPower	-=	(double)powerPtr[(uint32_t)indexRightOut & indexMask];
				totalPower	+=	(double)powerPtr[(uint32_t)indexRightIn & indexMask];

				indexLeftOut++;
				indexLeftIn++;
				indexRightOut++;
				indexRightIn++;
				threshold	=	(float) totalPower * relativeThr * scale;

				if (powerPtr[j] > threshold)
				{
					tempNoise[tempDetected]			=	(float) totalPower;
					tempRangeIndex[tempDetected++]	=	j;
				}
			}

			mod2Dleft	=	(i - 1) & (detectionCFARInst->fft2DSize - 1);
			mod2Dright	=	(i + 1) & (detectionCFARInst->fft2DSize - 1);
			for (j = 0; j < (int32_t)tempDetected; j++)
			{
				mod1Dleft	=	(tempRangeIndex[j] - 1) & (detectionCFARInst->fft1DSize - 1);
				mod1Dright	=	(tempRangeIndex[j] + 1) & (detectionCFARInst->fft1DSize - 1);
				sign		=	signmask & _ftoi(powerPtr[tempRangeIndex[j]] - powerPtr[mod1Dleft]);
				sign		|=	signmask & _ftoi(powerPtr[tempRangeIndex[j]] - powerPtr[mod1Dright]);
				sign		|=	signmask & _ftoi(powerPtr[tempRangeIndex[j]] - InputPower[mod2Dleft][tempRangeIndex[j]]);
				sign		|=	signmask & _ftoi(powerPtr[tempRangeIndex[j]] - InputPower[mod2Dright][tempRangeIndex[j]]);

				if ((sign == 0) && (detected < detectionCFARInst->maxNumDetObj))
				{
					noise[detected]			=	tempNoise[j] * scale;
					rangeInd[detected]		=	tempRangeIndex[j];
					dopplerInd[detected++]	=	i;
				}
			}
		}
	}

	//calculate range, speed, noise and SNR of the detected objects. 
	for (i = 0; i < (int32_t) detected; i++ )
	{
		//Dodo: quadraticInterp2D
		rangeEst[i]		=	(float)rangeInd[i] * detectionCFARInst->rangeRes;
		snrEst[i]		=	InputPower[dopplerInd[i]][rangeInd[i]]/noise[i];

		if ((uint32_t)dopplerInd[i] > (detectionCFARInst->fft2DSize >> 1))
			k			=	(int32_t)dopplerInd[i] - (int32_t)(detectionCFARInst->fft2DSize);
		else
			k			=	(int32_t)dopplerInd[i];

		dopplerEst[i] = (float)k * detectionCFARInst->dopplerRes;
	}
	
	return(detected);
}


#ifdef DEBUGDETECTION
uint32_t insertCnt = 0;
uint32_t removeCnt = 0;
uint32_t missingLastCnt = 0;
uint16_t tempIBuf[100], tempIndx;
float tempFBuf[100];
#endif


/*!
   \fn     RADARDEMO_detectionCFAR_OS

   \brief   Performs peak search and calculation of range and speed of detected object using CFAROS. All parameters hardcoded for cycle performance.
            search window length = 2 * 16 and K = 24. 

   \param[in]    InputPower
               Input power profile from integration.
 
   \param[in]    detectionCFARInst
               Pointer to input detection handle.
 
   \param[out]    rangeInd
               Pointer to the output range indices to detected objects. 

   \param[out]    dopplerInd
               Pointer to the output Doppler indices to detected objects. 
			   
   \param[out]    rangeEst
               Pointer to the output range estimation to detected objects. 
			   
   \param[out]    dopplerEst
               Pointer to the output Doppler estimation to detected objects. 
			   
   \param[out]    snrEst
               Pointer to the output linear SNR estimation to detected objects.
			   
   \param[out]    noise
               Pointer to the output noise estimation detected objects. 
			   
   \ret       number of objects detected.
   
   \pre       none
 
   \post      none
  
 
 */

int32_t	RADARDEMO_detectionCFAR_OS(
							IN float   **InputPower,
                            IN  RADARDEMO_detectionCFAR_handle *detectionCFARInst,
							OUT  uint16_t * rangeInd, 
							OUT  uint16_t * dopplerInd,
							OUT  float    * rangeEst,
							OUT  float    * dopplerEst,
							OUT  float    * snrEst,
							OUT  float    * noise)
{

	int32_t		i, j, k, n, NminusK, pingpong, totalItem, skip, insert;
	float       * RESTRICT powerPtr;
	float		relativeThr, threshold, ftemp;
	uint32_t    detected, sign, signmask = 0x80000000, tempDetected;
	int32_t     mod1Dleft, mod1Dright, mod2Dleft, mod2Dright;
	int16_t		* RESTRICT tempRangeIndex;
	float		* RESTRICT tempNoise;
	float		* RESTRICT largest8Ping;
	float		* RESTRICT largest8Pong;
	float		* largest8[2];
	float       * RESTRICT localBuf;
#ifdef DEBUGDETECTION
	float       * RESTRICT testlargest8;
#endif
	float       * RESTRICT tempPtr1, * RESTRICT tempPtr2;
	float		max;
	int32_t     localMaxIdx;
#ifdef _TMS320C6600 //C674x
	float ftemp1;
	int32_t itemp1;
	int64_t lltemp1, lltemp2, lltemp3;
#endif
	int32_t  tempIndex;
	uint32_t indexMask = detectionCFARInst->fft1DSize - 1;

	tempRangeIndex	=	(int16_t *)detectionCFARInst->scratchPad;
	tempNoise		=	(float *) &detectionCFARInst->scratchPad[detectionCFARInst->fft1DSize/2];
	localBuf		=	(float *) &detectionCFARInst->scratchPad[detectionCFARInst->fft1DSize*3/2];
	largest8Ping	=	(float *) &detectionCFARInst->scratchPad[detectionCFARInst->fft1DSize*3/2 + 66];
	largest8Pong	=	(float *) &detectionCFARInst->scratchPad[detectionCFARInst->fft1DSize*3/2 + 66 + 20];
#ifdef DEBUGDETECTION
	testlargest8	=	(float *) &scratchPad[detectionCFARInst->fft1DSize*3/2 + 66 + 40];
	tempIndx = 0;
#endif
	largest8[0]		=	largest8Ping;
	largest8[1]		=	largest8Pong;
	relativeThr		=	detectionCFARInst->relThr;
	detected		=	0;
	NminusK			=	8;


	/* peak search */
	for (i = 0; i < (int32_t) detectionCFARInst->fft2DSize; i++)
	{
		tempDetected	=	0;
		powerPtr	=	(float *) InputPower[i];
		j			=	0;
		//sort largest NminusK
		for (k = (int32_t)(detectionCFARInst->fft1DSize - detectionCFARInst->searchWinSizeRange); k < (int32_t) detectionCFARInst->fft1DSize; k+=2 )
		{
			_mem8_f2(&localBuf[j])	=	_mem8_f2(&powerPtr[k]);
			j += 2;
		}
		for (k = 0; k < (int32_t) detectionCFARInst->searchWinSizeRange; k+=2 )
		{
			_mem8_f2(&localBuf[j])	=	_mem8_f2(&powerPtr[k]);
			j += 2;
		}
		localBuf[j]	=	powerPtr[k];

		for (n = 0; n < 9; n++)
		{
			max	=	-1.f;
			//for (k = 0; k < (int32_t) detectionCFARInst->searchWinSize*2 + 1; k++ )
			for (k = 0; k < 33; k++ )
			{
				if (localBuf[k] > max)
				{
					max	=	localBuf[k];
					localMaxIdx	=	k;
				}
			}
			largest8Ping[n] = max;
			localBuf[localMaxIdx] = 0.f;
		}
		pingpong	=	0;
		for (j = 1; j < (int32_t) detectionCFARInst->fft1DSize; j++ )
		{
			totalItem	=	8;

			tempIndex	=	j-(int32_t)(detectionCFARInst->searchWinSizeRange + 1);
			ftemp		=	(float)powerPtr[((uint32_t) tempIndex) & indexMask];
			tempPtr2	=	largest8[pingpong];

			if (ftemp > tempPtr2[7])
			{
				skip		=	0;
				tempPtr1	=	largest8[pingpong^1];
				tempPtr2	=	largest8[pingpong];

				for (n = 0; n < totalItem - 1; n++)
				{
					if(ftemp >= tempPtr2[n]) skip = 1;
					tempPtr1[n] = tempPtr2[n + skip];
				}
				if(tempPtr1[n] < tempPtr2[n + skip])
					tempPtr1[n] = tempPtr2[n + skip];
				totalItem	-=	skip;
				pingpong	^=  1;
			}
			else if (ftemp == tempPtr2[7])
			{
				totalItem--;
			}

			tempPtr1	=	largest8[pingpong^1];
			tempPtr2	=	largest8[pingpong];
			tempIndex	=	j + (int32_t)(detectionCFARInst->searchWinSizeRange);
			ftemp		=	(float)powerPtr[((uint32_t) tempIndex) & indexMask];
			//ftemp		=	(float)powerPtr[j + (int32_t)(detectionCFARInst->searchWinSize)];
			if (ftemp > tempPtr2[totalItem-1])
			{
				insert		=	1;
				for (n = totalItem - 1; n >= 0; n--)
				{
					if(ftemp <= tempPtr2[n])
					{
						insert		=	0;
					}
					tempPtr1[n]			= ftemp;
					tempPtr1[n+insert]	= tempPtr2[n];
				}
				totalItem	+=	insert;
				pingpong	^=  1;
			}
			else if ((ftemp > tempPtr2[totalItem]) && (ftemp > tempPtr2[totalItem+1]))
			{
				tempPtr2[totalItem] = ftemp;
				totalItem	+=	1;
			}

			if (totalItem < NminusK)
			{
#ifdef DEBUGDETECTION
				missingLastCnt++;
#endif

				n = 0;
				max	=	-1.f;
				tempPtr1	=	largest8[pingpong];

				for (k = -(int32_t)(detectionCFARInst->searchWinSizeRange); k < (int32_t)(detectionCFARInst->searchWinSizeRange+1); k++ )
				{
					tempIndex =	j + k;
#ifndef _TMS320C6600 //C674x
					ftemp	=	powerPtr[((uint32_t) tempIndex) & indexMask];
					if(ftemp == tempPtr1[0]) ftemp = 0.f;
					if(ftemp == tempPtr1[1]) ftemp = 0.f;
					if(ftemp == tempPtr1[2]) ftemp = 0.f;
					if(ftemp == tempPtr1[3]) ftemp = 0.f;
					if(ftemp == tempPtr1[4]) ftemp = 0.f;
					if(ftemp == tempPtr1[5]) ftemp = 0.f;
					if(ftemp == tempPtr1[6]) ftemp = 0.f;

					if (ftemp > max)
					{
						max	=	ftemp;
					}
#else //C66x

					ftemp1	=	powerPtr[((uint32_t) tempIndex) & indexMask];
					lltemp1	=	_itoll(_ftoi(ftemp1), _ftoi(ftemp1));
					lltemp2 =	_itoll(_dcmpeq2(lltemp1, _mem8(&tempPtr1[0])), _dcmpeq2(lltemp1, _mem8(&tempPtr1[2])));
					lltemp3 =	_itoll(_dcmpeq2(lltemp1, _mem8(&tempPtr1[4])), _dcmpeq2(lltemp1, _itoll(0, _amem4(&tempPtr1[6]))));
					lltemp1 =	_dpackl2(lltemp2, lltemp3);
					itemp1	=	_packl4(_hill(lltemp1), _loll(lltemp1));
					itemp1	=	_deal(itemp1);
					itemp1	=	(itemp1 & _rotl(itemp1, 16));
 					if((itemp1 == 0) && (ftemp1 > max)) max = ftemp1;
#endif
				}
				tempPtr1[NminusK-1] = max;
				totalItem++;
			}
			tempPtr1	=	largest8[pingpong];
			threshold	=	(float) tempPtr1[NminusK-1] * relativeThr;
#ifdef DEBUGDETECTION
			if ((i == 24) && (j == 1440))
			{
				tempFBuf[tempIndx++]  = powerPtr[j];
				tempFBuf[tempIndx++]  = threshold;
				tempFBuf[tempIndx++]  = largest8[pingpong][NminusK-1];
				for (n = 0; n < 8; n++)
				{
					tempFBuf[tempIndx++]  = tempPtr1[n];
				}
			}
#endif
			if (powerPtr[j] > threshold)
			{
				tempNoise[tempDetected]			=	(float) tempPtr1[NminusK-1];
				tempRangeIndex[tempDetected++]	=	j;
			}

			if(tempPtr1[8] > tempPtr1[7]) tempPtr1[8] = 0.f;
			if(tempPtr1[9] > tempPtr1[8]) tempPtr1[9] = 0.f;

#ifdef DEBUGDETECTION

			{
				//sort largest NminusK
				n = 0;
				for (k = j-(int32_t)(detectionCFARInst->searchWinSize); k < j + (int32_t)(detectionCFARInst->searchWinSize + 1); k++ )
				{
					localBuf[n++]	=	powerPtr[k];
				}
				//localBuf[detectionCFARInst->searchWinSize] = 0;
				for (n = 0; n < NminusK; n++)
				{
					max	=	-1.f;
					for (k = 0; k < (int32_t) detectionCFARInst->searchWinSize*2 + 1; k++ )
					{
						if (localBuf[k] > max)
						{
							max	=	localBuf[k];
							localMaxIdx	=	k;
						}
					}
					testlargest8[n] = max;
					localBuf[localMaxIdx] = 0.f;
				}

				for (n = 0; n < NminusK; n++)
				{
					if(testlargest8[n] != largest8[pingpong][n])
					{
						printf("testlargest8[n] = %f, largest8[pingpong][n] = %f\n", testlargest8[n], largest8[pingpong][n]);
					}
				}
			}
#endif
		}

		mod2Dleft	=	(i - 1) & (detectionCFARInst->fft2DSize - 1);
		mod2Dright	=	(i + 1) & (detectionCFARInst->fft2DSize - 1);
		tempPtr1	=	InputPower[mod2Dleft];
		tempPtr2	=	InputPower[mod2Dright];

		for (j = 0; j < (int32_t)tempDetected; j++)
		{
			mod1Dleft	=	(tempRangeIndex[j] - 1) & (detectionCFARInst->fft1DSize - 1);
			mod1Dright	=	(tempRangeIndex[j] + 1) & (detectionCFARInst->fft1DSize - 1);
			sign		=	signmask & _ftoi(powerPtr[tempRangeIndex[j]] - powerPtr[mod1Dleft]);
			sign		|=	signmask & _ftoi(powerPtr[tempRangeIndex[j]] - powerPtr[mod1Dright]);
			sign		|=	signmask & _ftoi(powerPtr[tempRangeIndex[j]] - tempPtr1[tempRangeIndex[j]]);
			sign		|=	signmask & _ftoi(powerPtr[tempRangeIndex[j]] - tempPtr2[tempRangeIndex[j]]);

			if ((sign == 0) && (detected < detectionCFARInst->maxNumDetObj))
			{
				noise[detected]			=	tempNoise[j];
				rangeInd[detected]		=	tempRangeIndex[j];
				dopplerInd[detected++]	=	i;
			}
		}
	}

#ifdef DEBUGDETECTION
	printf("missing last counter = %d\n", missingLastCnt);
#endif

	//calculate range, speed, noise and SNR of the detected objects. 
	for (i = 0; i < (int32_t) detected; i++ )
	{
		//Dodo: quadraticInterp2D
		rangeEst[i] = (float)rangeInd[i] * detectionCFARInst->rangeRes;
		snrEst[i] = InputPower[dopplerInd[i]][rangeInd[i]]/noise[i];

		if ((uint32_t)dopplerInd[i] > (detectionCFARInst->fft2DSize >> 1))
			k			=	(int32_t)dopplerInd[i] - (int32_t)(detectionCFARInst->fft2DSize);
		else
			k			=	(int32_t)dopplerInd[i];

		dopplerEst[i] = (float)k * detectionCFARInst->dopplerRes;
	}
	
	return(detected);
}

/*! 
   \fn     RADARDEMO_detectionCFAR_CAAll
 
   \brief   Performs peak search and calculation of range and speed of detected object using CASO-CFAR.
  
   \param[in]    InputPower
               Input power profile from integration.
 
   \param[in]    detectionCFARInst
               Pointer to input detection handle.
 
   \param[out]    rangeInd
               Pointer to the output range indices to detected objects. 

   \param[out]    dopplerInd
               Pointer to the output Doppler indices to detected objects. 
			   
   \param[out]    rangeEst
               Pointer to the output range estimation to detected objects. 
			   
   \param[out]    dopplerEst
               Pointer to the output Doppler estimation to detected objects. 
			   
   \param[out]    rangeVar
               Pointer to the output variance for range estimation to detected objects. 
			   
   \param[out]    dopplerVar
               Pointer to the output variance for Doppler estimation to detected objects. 
			   
   \param[out]    snrEst
               Pointer to the output linear SNR estimation to detected objects.
			   
   \param[out]    noise
               Pointer to the output noise estimation detected objects. 
			   
   \ret       number of objects detected.
   
   \pre       none
 
   \post      none
  
 
 */

int32_t	RADARDEMO_detectionCFAR_CAAll(
							IN float   **InputPower,
                            IN  RADARDEMO_detectionCFAR_handle *detectionCFARInst,
							OUT  uint16_t * rangeInd, 
							OUT  uint16_t * dopplerInd,
							OUT  float    * rangeEst,
							OUT  float    * dopplerEst,
							OUT  float    * rangeVar,
							OUT  float    * dopplerVar,
							OUT  float    * snrEst,
							OUT  float    * noise)
{

	int32_t		i, j, k, i_doppler;
	float       * RESTRICT powerPtr, totalPower;
	double		leftWinPower, rightWinPower;
	float		leftscale, rightscale, relativeThr, threshold, dopplerScale;
	__float2_t  f2temp1, scale, power2f;
	uint32_t    detected, tempDetected;
	int16_t		*tempRangeIndex;
	float		*tempNoise;
	int32_t     totalWinSize, searchWinSize, guardSize;

	totalWinSize	=	(int32_t)detectionCFARInst->guardSizeRange + (int32_t)detectionCFARInst->searchWinSizeRange;
	searchWinSize	=	(int32_t)detectionCFARInst->searchWinSizeRange;
	guardSize		=	(int32_t)detectionCFARInst->guardSizeRange;
	tempRangeIndex	=	(int16_t *)detectionCFARInst->scratchPad;
	tempNoise		=	(float *) &detectionCFARInst->scratchPad[detectionCFARInst->fft1DSize/2];
	relativeThr		=	detectionCFARInst->relThr;
	detected		=	0;

#ifdef _WIN32	
	if(detectionCFARInst->log2MagFlag)
	{
		for (i = 0; i < detectionCFARInst->fft2DSize; i++)
		{
			for(j = 0; j < detectionCFARInst->fft1DSize; j++)
			{
				//InputPower[i][j]	=	0.5f*log2(InputPower[i][j]);
				InputPower[i][j]	=	0.5f*log(1.f + InputPower[i][j])/log(2.f);
			}
		}
	}
#else
	if(detectionCFARInst->log2MagFlag)
	{
		for (i = 0; i < detectionCFARInst->fft2DSize; i++)
		{
			for(j = 0; j < detectionCFARInst->fft1DSize; j++)
			{
				//InputPower[i][j]	=	0.5f*log2(InputPower[i][j]);
				InputPower[i][j]	=	0.5f*log2sp_i(1.f + InputPower[i][j]);
			}
		}
	}
#endif
	
	dopplerScale	=	detectionCFARInst->dopplerSearchRelThr * 0.5f/((float)detectionCFARInst->searchWinSizeDoppler);

	if (detectionCFARInst->caCfarType == RADARDEMO_DETECTIONCFAR_CFAR_CAVG)
	{
		leftscale	=	0.5f/(float)(detectionCFARInst->searchWinSizeRange);
		rightscale	=	0.5f/(float)(detectionCFARInst->searchWinSizeRange);
	}
	else if (detectionCFARInst->caCfarType == RADARDEMO_DETECTIONCFAR_CFAR_CACC)
	{
		leftscale	=	1.f;
		rightscale	=	1.f;
	}
	else if (detectionCFARInst->caCfarType == RADARDEMO_DETECTIONCFAR_CFAR_CASO)
	{
		leftscale	=	1.f/(float)(detectionCFARInst->searchWinSizeRange);
		rightscale	=	0.f;
	}
	else if (detectionCFARInst->caCfarType == RADARDEMO_DETECTIONCFAR_CFAR_CAGO)
	{
		leftscale	=	0.f;
		rightscale	=	1.f/(float)(detectionCFARInst->searchWinSizeRange);
	}
	scale		=	_ftof2(leftscale, rightscale);

	/* peak search */
	if (0)
	//if (detectionCFARInst->searchWinSize == 16)
	{ // optimized for search window size 16 -- Todo
	}
	else
	{// not optimized for search window size other than 16
		for (i = 0; i < (int32_t) detectionCFARInst->fft2DSize; i++)
		{
#if 0 //ndef CCS
			/* search from low doppler to high doppler, both direction*/
			if (i & 1)
			{
				i_doppler 	=	detectionCFARInst->fft2DSize - 1 - (i >> 1);
			}
			else
			{
				i_doppler 	=	i >> 1;
			}
#else
			i_doppler		=	i;
#endif
			tempDetected	=	0;
			powerPtr		=	(float *) InputPower[i_doppler];
			j				=	0;
			leftWinPower	=	0.0;
			rightWinPower	=	0.0;

			/* first detectionCFARInst->guardSizeRange + detectionCFARInst->searchWinSizeRange samples starting from detectionCFARInst->leftSkipSize*/
			for (k = (int32_t)detectionCFARInst->leftSkipSize + guardSize; k < (int32_t) detectionCFARInst->leftSkipSize + totalWinSize; k+=2 )
			{
				f2temp1			=	_mem8_f2(&powerPtr[k]);
				leftWinPower	+= (double)_hif2(f2temp1);
				leftWinPower	+= (double)_lof2(f2temp1);
			}
			if (searchWinSize & 1)
                 leftWinPower -=     (double)powerPtr[detectionCFARInst->leftSkipSize + totalWinSize];

			rightWinPower	=	leftWinPower - (double)powerPtr[(int32_t)detectionCFARInst->leftSkipSize + guardSize];
			rightWinPower	+=	(double)powerPtr[(int32_t) detectionCFARInst->leftSkipSize + totalWinSize];

			power2f				=	_ftof2((float)rightWinPower, (float)leftWinPower);
			if (rightWinPower > leftWinPower)
				power2f			=	_ftof2((float)leftWinPower, (float)rightWinPower);
			power2f				=	_dmpysp(power2f, scale);
			totalPower			=	_hif2(power2f) + _lof2(power2f);
			threshold			=	totalPower * relativeThr;
			if (powerPtr[detectionCFARInst->leftSkipSize] > threshold)
			{
				tempNoise[tempDetected]			=	(float) totalPower;
				tempRangeIndex[tempDetected++]	=	detectionCFARInst->leftSkipSize;
			}
			for (k = 1 + (int32_t)detectionCFARInst->leftSkipSize; k < (int32_t)detectionCFARInst->leftSkipSize + guardSize + 1; k++ )
			{
				leftWinPower		-=	(double)powerPtr[k + guardSize - 1];
				leftWinPower		+=	(double)powerPtr[k + totalWinSize - 1];

				rightWinPower		+=	(double)powerPtr[k + totalWinSize];
				rightWinPower		-=	(double)powerPtr[k + guardSize];
				power2f				=	_ftof2((float)rightWinPower, (float)leftWinPower);
				if (rightWinPower > leftWinPower)
					power2f			=	_ftof2((float)leftWinPower, (float)rightWinPower);
				power2f				=	_dmpysp(power2f, scale);
				totalPower			=	_hif2(power2f) + _lof2(power2f);
				threshold			=	totalPower * relativeThr;
				if (powerPtr[k] > threshold)
				{
					tempNoise[tempDetected]			=	(float) totalPower;
					tempRangeIndex[tempDetected++]	=	k;
				}
			}
			for (k = (int32_t)detectionCFARInst->leftSkipSize + guardSize + 1; k < (int32_t)detectionCFARInst->leftSkipSize + totalWinSize + 1; k++ )
			{
				leftWinPower		-=	(double)powerPtr[k + guardSize - 1];
				leftWinPower		+=	(double)powerPtr[k - guardSize - 1];

				rightWinPower		+=	(double)powerPtr[k + totalWinSize];
				rightWinPower		-=	(double)powerPtr[k + guardSize];
				power2f				=	_ftof2((float)rightWinPower, (float)leftWinPower);
				if (rightWinPower > leftWinPower)
					power2f			=	_ftof2((float)leftWinPower, (float)rightWinPower);
				power2f				=	_dmpysp(power2f, scale);
				totalPower			=	_hif2(power2f) + _lof2(power2f);
				threshold			=	totalPower * relativeThr;
				if (powerPtr[k] > threshold)
				{
					tempNoise[tempDetected]			=	(float) totalPower;
					tempRangeIndex[tempDetected++]	=	k;
				}
			}

			/* middle portion of samples */
			for (k = (int32_t) detectionCFARInst->leftSkipSize + totalWinSize + 1; k < (int32_t) detectionCFARInst->fft1DSize - (int32_t)detectionCFARInst->rightSkipSize - totalWinSize; k++ )
			{
				leftWinPower		-=	(double)powerPtr[k - totalWinSize - 1];
				leftWinPower		+=	(double)powerPtr[k - 1 -  guardSize];

				rightWinPower		+=	(double)powerPtr[k + totalWinSize];
				rightWinPower		-=	(double)powerPtr[k + guardSize];
				power2f				=	_ftof2((float)rightWinPower, (float)leftWinPower);
				if (rightWinPower > leftWinPower)
					power2f			=	_ftof2((float)leftWinPower, (float)rightWinPower);
				power2f				=	_dmpysp(power2f, scale);
				totalPower			=	_hif2(power2f) + _lof2(power2f);
				threshold			=	totalPower * relativeThr;
				if (powerPtr[k] > threshold)
				{
					tempNoise[tempDetected]			=	(float) totalPower;
					tempRangeIndex[tempDetected++]	=	k;
				}
			}

			/* last detectionCFARInst->guardSizeRange + detectionCFARInst->searchWinSizeRange samples before detectionCFARInst->fft1DSize - detectionCFARInst->rightSkipSize*/
			for (k = (int32_t) detectionCFARInst->fft1DSize - (int32_t)detectionCFARInst->rightSkipSize - totalWinSize; k < (int32_t) detectionCFARInst->fft1DSize - (int32_t)detectionCFARInst->rightSkipSize - guardSize; k++ )
			{
				leftWinPower		-=	(double)powerPtr[k - totalWinSize - 1];
				leftWinPower		+=	(double)powerPtr[k - guardSize - 1];

				rightWinPower		+=	(double)powerPtr[k - (int32_t)detectionCFARInst->rightSkipSize];
				rightWinPower		-=	(double)powerPtr[k + guardSize];

				power2f				=	_ftof2((float)rightWinPower, (float)leftWinPower);
				if (rightWinPower > leftWinPower)
					power2f			=	_ftof2((float)leftWinPower, (float)rightWinPower);
				power2f				=	_dmpysp(power2f, scale);
				totalPower			=	_hif2(power2f) + _lof2(power2f);
				threshold			=	totalPower * relativeThr;
				if (powerPtr[k] > threshold)
				{
					tempNoise[tempDetected]			=	(float) totalPower;
					tempRangeIndex[tempDetected++]	=	k;
				}
			}
			for (k = (int32_t) detectionCFARInst->fft1DSize - (int32_t)detectionCFARInst->rightSkipSize - guardSize; k < (int32_t) detectionCFARInst->fft1DSize - (int32_t)detectionCFARInst->rightSkipSize; k++ )
			{
				leftWinPower		-=	(double)powerPtr[k - totalWinSize - 1];
				leftWinPower		+=	(double)powerPtr[k - guardSize - 1];

				rightWinPower		+=	(double)powerPtr[k - (int32_t)detectionCFARInst->rightSkipSize];
				rightWinPower		-=	(double)powerPtr[k - (int32_t)detectionCFARInst->rightSkipSize - searchWinSize];

				power2f				=	_ftof2((float)rightWinPower, (float)leftWinPower);
				if (rightWinPower > leftWinPower)
					power2f			=	_ftof2((float)leftWinPower, (float)rightWinPower);
				power2f				=	_dmpysp(power2f, scale);
				totalPower			=	_hif2(power2f) + _lof2(power2f);
				threshold			=	totalPower * relativeThr;
				if (powerPtr[k] > threshold)
				{
					tempNoise[tempDetected]			=	(float) totalPower;
					tempRangeIndex[tempDetected++]	=	k;
				}
			}

			if (detectionCFARInst->enableSecondPassSearch)
			{
				#if 0
				float leftPower, rightPower;
				int32_t localRangeInd;
				if (i_doppler <  (int32_t)(detectionCFARInst->searchWinSizeDoppler + detectionCFARInst->guardSizeDoppler)) //left edge
				{
					for (j = 0; j < (int32_t)tempDetected; j++)
					{
						localRangeInd	=	tempRangeIndex[j];
						leftPower		=	0.f;
						for (k = 0; k < i_doppler - (int32_t)detectionCFARInst->guardSizeDoppler; k++ )
						{
							leftPower += InputPower[k][localRangeInd];
						}
						rightPower		=	0.f;
						for (k = i_doppler + (int32_t)detectionCFARInst->guardSizeDoppler + 1; k <=  i_doppler + (int32_t)detectionCFARInst->guardSizeDoppler + (int32_t)detectionCFARInst->searchWinSizeDoppler; k++ )
						{
							rightPower += InputPower[k][localRangeInd];
						}

						if (i_doppler - (int32_t)detectionCFARInst->guardSizeDoppler <= 0)
						{
							totalPower	=	2 * rightPower;
						}
						else
						{
							totalPower	=	rightPower + leftPower *(float)detectionCFARInst->searchWinSizeDoppler*_rcpsp((float)(i_doppler - (int32_t)detectionCFARInst->guardSizeDoppler));
						}
						
						threshold	=	totalPower * dopplerScale;
						if ((InputPower[i_doppler][localRangeInd] > threshold) && (detected < detectionCFARInst->maxNumDetObj))
						{
							noise[detected]			=	tempNoise[j];
							rangeInd[detected]		=	localRangeInd;
							dopplerInd[detected++]	=	i_doppler;
						}
					}
				}
				else if(i_doppler >= (int32_t)detectionCFARInst->fft2DSize - (int32_t)(detectionCFARInst->searchWinSizeDoppler + detectionCFARInst->guardSizeDoppler)) //right edge
				{
					for (j = 0; j < (int32_t)tempDetected; j++)
					{
						localRangeInd	=	tempRangeIndex[j];
						leftPower		=	0.f;
						for (k = i_doppler - (int32_t)(detectionCFARInst->searchWinSizeDoppler - detectionCFARInst->guardSizeDoppler); k < i_doppler -(int32_t)detectionCFARInst->guardSizeDoppler; k++ )
						{
							leftPower += InputPower[k][localRangeInd];
						}
						rightPower		=	0.f;
						for (k = i_doppler + (int32_t)detectionCFARInst->guardSizeDoppler + 1; k < (int32_t)detectionCFARInst->fft2DSize; k++ )
						{
							rightPower += InputPower[k][localRangeInd];
						}
						if (i_doppler + (int32_t)detectionCFARInst->guardSizeDoppler >= (int32_t)detectionCFARInst->fft2DSize)
						{
							totalPower	=	2 * leftPower;
						}
						else
						{
							totalPower	=	leftPower + rightPower *(float)detectionCFARInst->searchWinSizeDoppler * _rcpsp((float)((int32_t)detectionCFARInst->fft2DSize - i_doppler - (int32_t)detectionCFARInst->guardSizeDoppler));
						}
						threshold	=	totalPower * dopplerScale;
						if ((InputPower[i_doppler][localRangeInd] > threshold) && (detected < detectionCFARInst->maxNumDetObj))
						{
							noise[detected]			=	tempNoise[j];
							rangeInd[detected]		=	localRangeInd;
							dopplerInd[detected++]	=	i_doppler;
						}
					}
				}
				else  //none edge
				{
					for (j = 0; j < (int32_t)tempDetected; j++)
					{
						localRangeInd	=	tempRangeIndex[j];
						totalPower		=	0.f;
						for (k = i_doppler - (int32_t)detectionCFARInst->searchWinSizeDoppler - (int32_t)detectionCFARInst->guardSizeDoppler; k < i_doppler - (int32_t)detectionCFARInst->guardSizeDoppler; k++ )
						{
							totalPower += InputPower[k][localRangeInd];
						}
						for (k = i_doppler + 1 + (int32_t)detectionCFARInst->guardSizeDoppler; k <= i_doppler + (int32_t)detectionCFARInst->searchWinSizeDoppler + (int32_t)detectionCFARInst->guardSizeDoppler; k++ )
						{
							totalPower += InputPower[k][localRangeInd];
						}
						threshold	=	totalPower * dopplerScale;
						if ((InputPower[i_doppler][localRangeInd] > threshold) && (detected < detectionCFARInst->maxNumDetObj))
						{
							noise[detected]			=	tempNoise[j];
							rangeInd[detected]		=	localRangeInd;
							dopplerInd[detected++]	=	i_doppler;
						}
					}
				}
			
			
			
				#else
				int32_t leftrepeat, rightrepeat, localRangeInd;

				leftrepeat		=	(int32_t)detectionCFARInst->searchWinSizeDoppler - i_doppler;
				if (leftrepeat < 0)
					leftrepeat = 0;
				rightrepeat		=	(int32_t)detectionCFARInst->searchWinSizeDoppler - ((int32_t)detectionCFARInst->fft2DSize - 1 - i_doppler);
				if (rightrepeat < 0)
					rightrepeat = 0;


				if (i_doppler <  (int32_t)detectionCFARInst->searchWinSizeDoppler) //left edge
				{
					for (j = 0; j < (int32_t)tempDetected; j++)
					{
						localRangeInd	=	tempRangeIndex[j];
						totalPower		=	0.f;
						for (k = (int32_t)detectionCFARInst->fft2DSize - leftrepeat; k < (int32_t)detectionCFARInst->fft2DSize; k++ )
						{
							totalPower += InputPower[k][localRangeInd];
						}
						for (k = 0; k <  (int32_t)detectionCFARInst->searchWinSizeDoppler - leftrepeat ; k++ )
						{
							totalPower += InputPower[k][localRangeInd];
						}

						for (k = i_doppler + 1; k <= i_doppler + (int32_t)detectionCFARInst->searchWinSizeDoppler; k++ )
						{
							totalPower += InputPower[k][localRangeInd];
						}
						threshold	=	totalPower * dopplerScale;
						if ((InputPower[i_doppler][localRangeInd] > threshold) && (detected < detectionCFARInst->maxNumDetObj))
						{
							noise[detected]			=	tempNoise[j];
							rangeInd[detected]		=	localRangeInd;
							dopplerInd[detected++]	=	i_doppler;
						}
					}
				}
				else if(i_doppler >= (int32_t)detectionCFARInst->fft2DSize - (int32_t)detectionCFARInst->searchWinSizeDoppler) //right edge
				{
					for (j = 0; j < (int32_t)tempDetected; j++)
					{
						localRangeInd	=	tempRangeIndex[j];
						totalPower		=	0.f;
						for (k = i_doppler - (int32_t)detectionCFARInst->searchWinSizeDoppler; k < i_doppler ; k++ )
						{
							totalPower += InputPower[k][localRangeInd];
						}
						for (k = 0; k < rightrepeat; k++ )
						{
							totalPower += InputPower[k][localRangeInd];
						}
						for (k = i_doppler + 1; k < (int32_t)detectionCFARInst->fft2DSize ; k++ )
						{
							totalPower += InputPower[k][localRangeInd];
						}
						threshold	=	totalPower * dopplerScale;
						if ((InputPower[i_doppler][localRangeInd] > threshold) && (detected < detectionCFARInst->maxNumDetObj))
						{
							noise[detected]			=	tempNoise[j];
							rangeInd[detected]		=	localRangeInd;
							dopplerInd[detected++]	=	i_doppler;
						}
					}
				}
				else  //none edge
				{
					for (j = 0; j < (int32_t)tempDetected; j++)
					{
						localRangeInd	=	tempRangeIndex[j];
						totalPower		=	0.f;
						for (k = i_doppler - (int32_t)detectionCFARInst->searchWinSizeDoppler; k < i_doppler ; k++ )
						{
							totalPower += InputPower[k][localRangeInd];
						}
						for (k = i_doppler + 1; k <= i_doppler + (int32_t)detectionCFARInst->searchWinSizeDoppler; k++ )
						{
							totalPower += InputPower[k][localRangeInd];
						}
						threshold	=	totalPower * dopplerScale;
						if ((InputPower[i_doppler][localRangeInd] > threshold) && (detected < detectionCFARInst->maxNumDetObj))
						{
							noise[detected]			=	tempNoise[j];
							rangeInd[detected]		=	localRangeInd;
							dopplerInd[detected++]	=	i_doppler;
						}
					}
				}
				#endif
			}
			else
			{
				if ((detected + tempDetected) < detectionCFARInst->maxNumDetObj )
				{
					for (j = 0; j < (int32_t)tempDetected; j++)
					{
						noise[detected]			=	tempNoise[j];
						rangeInd[detected]			=	tempRangeIndex[j];
						dopplerInd[detected++]		=	i_doppler;
					}
				}
				else
				{
					k	=	(int32_t)(detectionCFARInst->maxNumDetObj - detected);
					for (j = 0; j < k; j++)
					{
						noise[detected]			=	tempNoise[j];
						rangeInd[detected]			=	tempRangeIndex[j];
						dopplerInd[detected++]		=	i_doppler;
					}
					break;
				}
			}

		}
	}

	//calculate range, speed, noise and SNR of the detected objects. 
	for (i = 0; i < (int32_t) detected; i++ )
	{
		//Dodo: quadraticInterp2D
		rangeEst[i]		=	(float)rangeInd[i] * detectionCFARInst->rangeRes;
		snrEst[i]		=	InputPower[dopplerInd[i]][rangeInd[i]]/noise[i];
		/*this is an approximation since noise is not log2(sum(noise)) */
		if(detectionCFARInst->log2MagFlag)
		{
			snrEst[i]		=	6.f*(InputPower[dopplerInd[i]][rangeInd[i]] - noise[i]);
		}

		if ((uint32_t)dopplerInd[i] >= (detectionCFARInst->fft2DSize >> 1))
			k			=	(int32_t)dopplerInd[i] - (int32_t)(detectionCFARInst->fft2DSize);
		else
			k			=	(int32_t)dopplerInd[i];

		dopplerEst[i] = (float)k * detectionCFARInst->dopplerRes;
	}
	
	return(detected);
}


/*! 
   \fn     RADARDEMO_detectionCFAR_raCAAll
 
   \brief   Performs peak search and calculation of range and azimuth of detected object using CASO-CFAR.
  
   \param[in]    InputPower
               Input power profile from integration.
 
   \param[in]    detectionCFARInst
               Pointer to input detection handle.
 
   \param[out]    rangeInd
               Pointer to the output range indices to detected objects. 

   \param[out]    azimuthInd
               Pointer to the output azimuth indices to detected objects. 
			   
   \param[out]    snrEst
               Pointer to the output linear SNR estimation to detected objects.
			   
   \param[out]    noise
               Pointer to the output noise estimation detected objects. 
			   
   \ret       number of objects detected.
   
   \pre       none
 
   \post      none
  
 
 */

int32_t	RADARDEMO_detectionCFAR_raCAAll(
							IN float   **InputPower,
                            IN  RADARDEMO_detectionCFAR_handle *detectionCFARInst,
							OUT  uint16_t * rangeInd, 
							OUT  uint16_t * azimuthInd,
							OUT  float    * snrEst,
							OUT  float    * noise)
{

	int32_t		i, j, k, i_2d;
	float       * RESTRICT powerPtr, totalPower;
	double		leftWinPower, rightWinPower;
	float		leftscale, rightscale, relativeThr, threshold, dopplerScale;
	__float2_t  f2temp1, scale, power2f;
	uint32_t    detected, tempDetected;
	int16_t		*tempRangeIndex;
	float		*tempNoise;
	int32_t     totalWinSize, searchWinSize, guardSize;

	totalWinSize	=	(int32_t)detectionCFARInst->guardSizeRange + (int32_t)detectionCFARInst->searchWinSizeRange;
	searchWinSize	=	(int32_t)detectionCFARInst->searchWinSizeRange;
	guardSize		=	(int32_t)detectionCFARInst->guardSizeRange;
	tempRangeIndex	=	(int16_t *)detectionCFARInst->scratchPad;
	tempNoise		=	(float *) &detectionCFARInst->scratchPad[detectionCFARInst->fft1DSize/2];
	relativeThr		=	detectionCFARInst->relThr;
	detected		=	0;

#ifdef _WIN32	
	if(detectionCFARInst->log2MagFlag)
	{
		for (i = 0; i < detectionCFARInst->fft2DSize; i++)
		{
			for(j = 0; j < detectionCFARInst->fft1DSize; j++)
			{
				//InputPower[i][j]	=	0.5f*log2(InputPower[i][j]);
				InputPower[i][j]	=	0.5f*log(1.f + InputPower[i][j])/log(2.f);
			}
		}
	}
#else
	if(detectionCFARInst->log2MagFlag)
	{
		for (i = 0; i < detectionCFARInst->fft2DSize; i++)
		{
			for(j = 0; j < detectionCFARInst->fft1DSize; j++)
			{
				//InputPower[i][j]	=	0.5f*log2(InputPower[i][j]);
				InputPower[i][j]	=	0.5f*log2sp_i(1.f + InputPower[i][j]);
			}
		}
	}
#endif
	
	dopplerScale	=	detectionCFARInst->dopplerSearchRelThr/((float)detectionCFARInst->searchWinSizeDoppler);

	if (detectionCFARInst->caCfarType == RADARDEMO_DETECTIONCFAR_CFAR_CAVG)
	{
		leftscale	=	0.5f/(float)(detectionCFARInst->searchWinSizeRange);
		rightscale	=	0.5f/(float)(detectionCFARInst->searchWinSizeRange);
	}
	else if (detectionCFARInst->caCfarType == RADARDEMO_DETECTIONCFAR_CFAR_CACC)
	{
		leftscale	=	1.f;
		rightscale	=	1.f;
	}
	else if ((detectionCFARInst->caCfarType == RADARDEMO_DETECTIONCFAR_CFAR_CASO) || (detectionCFARInst->caCfarType == RADARDEMO_DETECTIONCFAR_RA_CFAR_CASO))
	{
		leftscale	=	1.f/(float)(detectionCFARInst->searchWinSizeRange);
		rightscale	=	0.f;
	}
	else if (detectionCFARInst->caCfarType == RADARDEMO_DETECTIONCFAR_CFAR_CAGO)
	{
		leftscale	=	0.f;
		rightscale	=	1.f/(float)(detectionCFARInst->searchWinSizeRange);
	}
	scale		=	_ftof2(leftscale, rightscale);

	/* peak search */
	for (i = detectionCFARInst->leftSkipSizeAzimuth; i < (int32_t) detectionCFARInst->fft2DSize - detectionCFARInst->rightSkipSizeAzimuth; i++)
	{
#if 0 //ndef CCS
		/* search from low doppler to high doppler, both direction*/
		if (i & 1)
		{
			i_2d 	=	detectionCFARInst->fft2DSize - 1 - (i >> 1);
		}
		else
		{
			i_2d 	=	i >> 1;
		}
#else
		i_2d		=	i;
#endif
		tempDetected	=	0;
		powerPtr		=	(float *) InputPower[i_2d];
		j				=	0;
		leftWinPower	=	0.0;
		rightWinPower	=	0.0;

		/* first detectionCFARInst->guardSizeRange + detectionCFARInst->searchWinSizeRange samples starting from detectionCFARInst->leftSkipSize*/
		for (k = (int32_t)detectionCFARInst->leftSkipSize + guardSize; k < (int32_t) detectionCFARInst->leftSkipSize + totalWinSize; k+=2 )
		{
			f2temp1			=	_mem8_f2(&powerPtr[k]);
			leftWinPower	+= (double)_hif2(f2temp1);
			leftWinPower	+= (double)_lof2(f2temp1);
		}
		if (searchWinSize & 1)
                leftWinPower -=     (double)powerPtr[detectionCFARInst->leftSkipSize + totalWinSize];

		rightWinPower	=	leftWinPower - (double)powerPtr[(int32_t)detectionCFARInst->leftSkipSize + guardSize];
		rightWinPower	+=	(double)powerPtr[(int32_t) detectionCFARInst->leftSkipSize + totalWinSize];

		power2f				=	_ftof2((float)rightWinPower, (float)leftWinPower);
		if (rightWinPower > leftWinPower)
			power2f			=	_ftof2((float)leftWinPower, (float)rightWinPower);
		power2f				=	_dmpysp(power2f, scale);
		totalPower			=	_hif2(power2f) + _lof2(power2f);
		threshold			=	totalPower * relativeThr;
		if (powerPtr[detectionCFARInst->leftSkipSize] > threshold)
		{
			tempNoise[tempDetected]			=	(float) totalPower;
			tempRangeIndex[tempDetected++]	=	detectionCFARInst->leftSkipSize;
		}
		for (k = 1 + (int32_t)detectionCFARInst->leftSkipSize; k < (int32_t)detectionCFARInst->leftSkipSize + guardSize + 1; k++ )
		{
			leftWinPower		-=	(double)powerPtr[k + guardSize - 1];
			leftWinPower		+=	(double)powerPtr[k + totalWinSize - 1];

			rightWinPower		+=	(double)powerPtr[k + totalWinSize];
			rightWinPower		-=	(double)powerPtr[k + guardSize];
			power2f				=	_ftof2((float)rightWinPower, (float)leftWinPower);
			if (rightWinPower > leftWinPower)
				power2f			=	_ftof2((float)leftWinPower, (float)rightWinPower);
			power2f				=	_dmpysp(power2f, scale);
			totalPower			=	_hif2(power2f) + _lof2(power2f);
			threshold			=	totalPower * relativeThr;
			if (powerPtr[k] > threshold)
			{
				tempNoise[tempDetected]			=	(float) totalPower;
				tempRangeIndex[tempDetected++]	=	k;
			}
		}
		for (k = (int32_t)detectionCFARInst->leftSkipSize + guardSize + 1; k < (int32_t)detectionCFARInst->leftSkipSize + totalWinSize + 1; k++ )
		{
			leftWinPower		-=	(double)powerPtr[k + guardSize - 1];
			leftWinPower		+=	(double)powerPtr[k - guardSize - 1];

			rightWinPower		+=	(double)powerPtr[k + totalWinSize];
			rightWinPower		-=	(double)powerPtr[k + guardSize];
			power2f				=	_ftof2((float)rightWinPower, (float)leftWinPower);
			if (rightWinPower > leftWinPower)
				power2f			=	_ftof2((float)leftWinPower, (float)rightWinPower);
			power2f				=	_dmpysp(power2f, scale);
			totalPower			=	_hif2(power2f) + _lof2(power2f);
			threshold			=	totalPower * relativeThr;
			if (powerPtr[k] > threshold)
			{
				tempNoise[tempDetected]			=	(float) totalPower;
				tempRangeIndex[tempDetected++]	=	k;
			}
		}

		/* middle portion of samples */
		for (k = (int32_t) detectionCFARInst->leftSkipSize + totalWinSize + 1; k < (int32_t) detectionCFARInst->fft1DSize - (int32_t)detectionCFARInst->rightSkipSize - totalWinSize; k++ )
		{
			leftWinPower		-=	(double)powerPtr[k - totalWinSize - 1];
			leftWinPower		+=	(double)powerPtr[k - 1 -  guardSize];

			rightWinPower		+=	(double)powerPtr[k + totalWinSize];
			rightWinPower		-=	(double)powerPtr[k + guardSize];
			power2f				=	_ftof2((float)rightWinPower, (float)leftWinPower);
			if (rightWinPower > leftWinPower)
				power2f			=	_ftof2((float)leftWinPower, (float)rightWinPower);
			power2f				=	_dmpysp(power2f, scale);
			totalPower			=	_hif2(power2f) + _lof2(power2f);
			threshold			=	totalPower * relativeThr;
			if (powerPtr[k] > threshold)
			{
				tempNoise[tempDetected]			=	(float) totalPower;
				tempRangeIndex[tempDetected++]	=	k;
			}
		}

		/* last detectionCFARInst->guardSizeRange + detectionCFARInst->searchWinSizeRange samples before detectionCFARInst->fft1DSize - detectionCFARInst->rightSkipSize*/
		for (k = (int32_t) detectionCFARInst->fft1DSize - (int32_t)detectionCFARInst->rightSkipSize - totalWinSize; k < (int32_t) detectionCFARInst->fft1DSize - (int32_t)detectionCFARInst->rightSkipSize - guardSize; k++ )
		{
			leftWinPower		-=	(double)powerPtr[k - totalWinSize - 1];
			leftWinPower		+=	(double)powerPtr[k - guardSize - 1];

			rightWinPower		+=	(double)powerPtr[k - (int32_t)detectionCFARInst->rightSkipSize];
			rightWinPower		-=	(double)powerPtr[k + guardSize];

			power2f				=	_ftof2((float)rightWinPower, (float)leftWinPower);
			if (rightWinPower > leftWinPower)
				power2f			=	_ftof2((float)leftWinPower, (float)rightWinPower);
			power2f				=	_dmpysp(power2f, scale);
			totalPower			=	_hif2(power2f) + _lof2(power2f);
			threshold			=	totalPower * relativeThr;
			if (powerPtr[k] > threshold)
			{
				tempNoise[tempDetected]			=	(float) totalPower;
				tempRangeIndex[tempDetected++]	=	k;
			}
		}
		for (k = (int32_t) detectionCFARInst->fft1DSize - (int32_t)detectionCFARInst->rightSkipSize - guardSize; k < (int32_t) detectionCFARInst->fft1DSize - (int32_t)detectionCFARInst->rightSkipSize; k++ )
		{
			leftWinPower		-=	(double)powerPtr[k - totalWinSize - 1];
			leftWinPower		+=	(double)powerPtr[k - guardSize - 1];

			rightWinPower		+=	(double)powerPtr[k - (int32_t)detectionCFARInst->rightSkipSize];
			rightWinPower		-=	(double)powerPtr[k - (int32_t)detectionCFARInst->rightSkipSize - searchWinSize];

			power2f				=	_ftof2((float)rightWinPower, (float)leftWinPower);
			if (rightWinPower > leftWinPower)
				power2f			=	_ftof2((float)leftWinPower, (float)rightWinPower);
			power2f				=	_dmpysp(power2f, scale);
			totalPower			=	_hif2(power2f) + _lof2(power2f);
			threshold			=	totalPower * relativeThr;
			if (powerPtr[k] > threshold)
			{
				tempNoise[tempDetected]			=	(float) totalPower;
				tempRangeIndex[tempDetected++]	=	k;
			}
		}

		if (detectionCFARInst->enableSecondPassSearch)
		{
			int32_t leftrepeat, rightrepeat, localRangeInd;
			float   powerLeft, powerRight;

			leftrepeat		=	(int32_t)(detectionCFARInst->searchWinSizeDoppler + detectionCFARInst->guardSizeDoppler) - i_2d;
			if (leftrepeat < 0)
				leftrepeat = 0;
			rightrepeat		=	(int32_t)(detectionCFARInst->searchWinSizeDoppler + detectionCFARInst->guardSizeDoppler) - ((int32_t)detectionCFARInst->fft2DSize - 1 - i_2d - detectionCFARInst->guardSizeDoppler);
			if (rightrepeat < 0)
				rightrepeat = 0;


			if (i_2d <  (int32_t)detectionCFARInst->searchWinSizeDoppler + detectionCFARInst->guardSizeDoppler) //left edge
			{
				for (j = 0; j < (int32_t)tempDetected; j++)
				{
					localRangeInd	=	tempRangeIndex[j];
					powerLeft		=	0.f;
					for (k = (int32_t)detectionCFARInst->fft2DSize - leftrepeat; k < (int32_t)(detectionCFARInst->fft2DSize - detectionCFARInst->guardSizeDoppler + detectionCFARInst->leftSkipSizeAzimuth); k++ )
					{
						powerLeft += InputPower[k][localRangeInd];
					}
					for (k = 0; k <  (int32_t)detectionCFARInst->searchWinSizeDoppler - leftrepeat ; k++ )
					{
						powerLeft += InputPower[k][localRangeInd];
					}

					powerRight		=	0.f;
					for (k = i_2d + 1 + detectionCFARInst->guardSizeDoppler; k <= i_2d + detectionCFARInst->guardSizeDoppler + (int32_t)detectionCFARInst->searchWinSizeDoppler; k++ )
					{
						powerRight += InputPower[k][localRangeInd];
					}
					if(powerRight > powerLeft)
						powerRight = powerLeft;
					threshold	=	powerRight * dopplerScale;  
					if ((InputPower[i_2d][localRangeInd] > threshold) && (detected < detectionCFARInst->maxNumDetObj))
					{
						noise[detected]			=	tempNoise[j];
						rangeInd[detected]		=	localRangeInd;
						azimuthInd[detected++]	=	i_2d;
					}
				}
			}
			else if(i_2d >= (int32_t)detectionCFARInst->fft2DSize - (int32_t)(detectionCFARInst->searchWinSizeDoppler + detectionCFARInst->guardSizeDoppler)) //right edge
			{
				for (j = 0; j < (int32_t)tempDetected; j++)
				{
					localRangeInd	=	tempRangeIndex[j];
					powerLeft		=	0.f;
					for (k = i_2d - (int32_t)detectionCFARInst->searchWinSizeDoppler; k < i_2d ; k++ )
					{
						powerLeft += InputPower[k][localRangeInd];
					}

					powerRight		=	0.f;
					for (k = 0; k < rightrepeat; k++ )
					{
						powerRight += InputPower[k][localRangeInd];
					}
					for (k = i_2d + 1 + detectionCFARInst->guardSizeDoppler; k < (int32_t)detectionCFARInst->fft2DSize ; k++ )
					{
						powerRight += InputPower[k][localRangeInd];
					}
					if(powerRight > powerLeft)
						powerRight = powerLeft;
					threshold	=	powerRight * dopplerScale;
					if ((InputPower[i_2d][localRangeInd] > threshold) && (detected < detectionCFARInst->maxNumDetObj))
					{
						noise[detected]			=	tempNoise[j];
						rangeInd[detected]		=	localRangeInd;
						azimuthInd[detected++]	=	i_2d;
					}
				}
			}
			else  //none edge
			{
				for (j = 0; j < (int32_t)tempDetected; j++)
				{
					localRangeInd	=	tempRangeIndex[j];
					powerLeft		=	0.f;
					for (k = i_2d - (int32_t)(detectionCFARInst->searchWinSizeDoppler + detectionCFARInst->guardSizeDoppler); k < i_2d  - (int32_t)detectionCFARInst->guardSizeDoppler; k++ )
					{
						powerLeft += InputPower[k][localRangeInd];
					}
					powerRight		=	0.f;
					for (k = i_2d + 1 + detectionCFARInst->guardSizeDoppler; k <= i_2d + (int32_t)(detectionCFARInst->searchWinSizeDoppler + detectionCFARInst->guardSizeDoppler); k++ )
					{
						powerRight += InputPower[k][localRangeInd];
					}
					if(powerRight > powerLeft)
						powerRight = powerLeft;
					threshold	=	powerRight * dopplerScale;
					if ((InputPower[i_2d][localRangeInd] > threshold) && (detected < detectionCFARInst->maxNumDetObj))
					{
						noise[detected]			=	tempNoise[j];
						rangeInd[detected]		=	localRangeInd;
						azimuthInd[detected++]	=	i_2d;
					}
				}
			}
		}
		else
		{
			if ((detected + tempDetected) < detectionCFARInst->maxNumDetObj )
			{
				for (j = 0; j < (int32_t)tempDetected; j++)
				{
					noise[detected]			=	tempNoise[j];
					rangeInd[detected]			=	tempRangeIndex[j];
					azimuthInd[detected++]		=	i_2d;
				}
			}
			else
			{
				k	=	(int32_t)(detectionCFARInst->maxNumDetObj - detected);
				for (j = 0; j < k; j++)
				{
					noise[detected]			=	tempNoise[j];
					rangeInd[detected]			=	tempRangeIndex[j];
					azimuthInd[detected++]		=	i_2d;
				}
				break;
			}
		}

	}

	//calculate range, speed, noise and SNR of the detected objects. 
	for (i = 0; i < (int32_t) detected; i++ )
	{
		//Dodo: quadraticInterp2D
		snrEst[i]		=	InputPower[azimuthInd[i]][rangeInd[i]]/noise[i];
		/*this is an approximation since noise is not log2(sum(noise)) */
		if(detectionCFARInst->log2MagFlag)
		{
			snrEst[i]		=	6.f*(InputPower[azimuthInd[i]][rangeInd[i]] - noise[i]);
		}
	}
	
	return(detected);
}


#ifdef USE_TABLE_FOR_K0


float rltvThr_CFARCA[] = {
72.00f,	51.99f,	36.99f,	25.74f,	17.30f,	10.97f,	6.23f,
34.60f,	27.81f,	21.94f,	16.86f,	12.45f,	8.64f,	5.34f,
27.71f,	22.98f,	18.68f,	14.77f,	11.23f,	8.00f,	5.08f,
24.90f,	20.95f,	17.28f,	13.86f,	10.67f,	7.71f,	4.95f,
23.40f,	19.85f,	16.50f,	13.34f,	10.36f,	7.54f,	4.88f,
22.45f,	19.15f,	16.01f,	13.01f,	10.15f,	7.43f,	4.83f,
21.81f,	18.68f,	15.67f,	12.78f,	10.01f,	7.35f,	4.80f,
21.35f,	18.33f,	15.42f,	12.61f,	9.91f,	7.29f,	4.77f
};

float rltvThr_CFAROS_8[] = {
7475.8f,    2358.9f,     740.8f,    229.1f,      67.3f,
688.2f,    315.7f,    142.8f,    62.5f,     25.3f,
196.0f,    107.4f,     57.5f,    29.5f,     13.8f,
86.4f,    52.3f,    30.8f,   17.3f,     8.7f,
46.7f,    30.1f,    18.8f,   11.1f,     5.9f,
27.8f,    18.7f,    12.1f,    7.4f,     4.0f,
16.8f,    11.6f,     7.7f,    4.8f,     2.8f  
};

float rltvThr_CFAROS_16[] = {
15476.4f,    4883.5f,    1533.7f,     474.4f,     139.4f,
1482.8f,    680.2f,    307.7f,    134.8f,     54.5f,
 442.7f,    242.6f,    130.1f,     66.8f,     31.2f,
 206.8f,    125.3f,     73.9f,     41.5f,     21.0f,
 120.4f,     77.8f,     48.7f,     28.9f,     15.4f,
  79.5f,     53.6f,     34.9f,     21.5f,     11.9f,
  56.6f,     39.4f,     26.4f,     16.8f,      9.5f,
  42.5f,     30.2f,     20.7f,     13.4f,      7.8f,
  32.9f,     23.8f,     16.6f,     10.9f,      6.4f,
  26.1f,     19.1f,     13.6f,      9.0f,      5.4f,
  21.0f,     15.6f,     11.1f,      7.5f,      4.5f,
  17.0f,     12.7f,      9.1f,      6.2f,      3.8f,
  13.8f,     10.4f,      7.5f,      5.0f,      3.0f,
  10.9f,      8.3f,      6.0f,      4.0f,      2.6f,
   8.4f,      6.4f,      4.7f,      3.0f,      1.9f
   };

float rltvThr_CFAROS_24[] = {
23471.2f,    7406.2f,    2326.0f,    719.5f,    211.4f,
2275.6f,   1043.9f,    472.2f,   206.9f,    83.7f,
 688.1f,    377.1f,    202.2f,   103.9f,    48.6f,
 326.0f,    197.6f,    116.5f,    65.4f,    33.2f,
 192.8f,    124.5f,     78.0f,    46.3f,    24.7f,
 129.5f,     87.3f,     57.0f,    35.1f,    19.4f,
  94.1f,     65.4f,     44.0f,    27.9f,    15.8f,
  72.1f,     51.3f,     35.3f,    22.8f,    13.2f,
  57.3f,     41.6f,     29.0f,    19.1f,    11.2f,
  46.9f,     34.5f,     24.4f,    16.3f,     9.7f,
  39.1f,     29.1f,     20.8f,    14.0f,     8.5f,
  33.1f,     24.9f,     18.0f,    12.2f,     7.4f,
  28.4f,     21.5f,     15.7f,    10.7f,     6.6f,
  24.5f,     18.7f,     13.7f,     9.5f,     5.8f,
  21.3f,     16.4f,     12.0f,     8.4f,     5.1f,
  18.7f,     14.4f,     10.7f,     7.4f,     4.6f,
  16.4f,     12.7f,      9.4f,     6.6f,     4.0f,
  14.3f,     11.1f,      8.3f,     5.8f,     3.7f,
  12.6f,      9.8f,      7.3f,     5.1f,     3.2f,
  10.9f,      8.6f,      6.4f,     4.6f,     2.9f,
   9.5f,      7.4f,      5.6f,     3.9f,     2.6f,
   7.9f,      6.2f,      4.7f,     3.3f,     2.0f,
   6.5f,      5.0f,      3.8f,     2.8f,     1.8f 
};

float rltvThr_CFAROS_32[] = {
31464.5f,    9928.4f,    3118.1f,     964.5f,     283.5f,
 3067.9f,    1407.4f,     636.6f,     278.9f,     112.8f,
  933.3f,     511.5f,     274.3f,     140.9f,      65.9f,
  444.9f,     269.7f,     159.1f,      89.3f,      45.3f,
  265.0f,     171.2f,     107.2f,      63.7f,      34.0f,
  179.2f,     120.9f,      78.9f,      48.7f,      26.9f,
  131.3f,      91.3f,      61.4f,      38.9f,      22.1f,
  101.4f,      72.2f,      49.6f,      32.1f,      18.6f,
   81.4f,      59.0f,      41.3f,      27.2f,      15.9f,
   67.2f,      49.4f,      35.0f,      23.4f,      13.9f,
   56.7f,      42.2f,      30.2f,      20.4f,      12.2f,
   48.6f,      36.5f,      26.4f,      17.9f,      10.9f,
   42.2f,      31.9f,      23.3f,      15.9f,       9.8f,
   37.0f,      28.2f,      20.7f,      14.3f,       8.8f,
   32.8f,      25.1f,      18.6f,      12.9f,       7.9f,
   29.2f,      22.5f,      16.7f,      11.7f,       7.2f,
   26.1f,      20.3f,      15.1f,      10.6f,       6.6f,
   23.6f,      18.3f,      13.7f,       9.7f,       6.0f,
   21.3f,      16.6f,      12.5f,       8.8f,       5.6f,
   19.3f,      15.1f,      11.4f,       8.0f,       5.0f,
   17.5f,      13.8f,      10.4f,       7.4f,       4.7f,
   15.9f,      12.6f,       9.5f,       6.8f,       4.3f,
   14.5f,      11.4f,       8.7f,       6.1f,       3.9f,
   13.1f,      10.4f,       7.9f,       5.7f,       3.7f,
   11.9f,       9.5f,       7.2f,       5.1f,       3.3f,
   10.8f,       8.6f,       6.6f,       4.7f,       3.0f,
    9.8f,       7.8f,       5.9f,       4.2f,       2.8f,
    8.8f,       6.9f,       5.3f,       3.8f,       2.5f,
    7.8f,       6.0f,       4.7f,       3.4f,       2.1f,
    6.8f,       5.3f,       4.0f,       2.9f,       1.9f,
    5.7f,       4.5f,       3.4f,       2.5f,       1.7f
};

float rltvThr_CFAROS_40[] = {
39457.3f,   12450.5f,    3910.2f,    1209.5f,     355.5f,
 3860.1f,    1770.8f,     801.0f,     350.9f,     142.0f,
 1178.5f,     645.9f,     346.3f,     177.9f,      83.2f,
  563.8f,     341.7f,     201.6f,     113.2f,      57.4f,
  337.1f,     217.7f,     136.4f,      81.0f,      43.2f,
  228.9f,     154.4f,     100.7f,      62.1f,      34.4f,
  168.4f,     117.1f,      78.7f,      49.9f,      28.3f,
  130.7f,      93.1f,      63.9f,      41.4f,      24.0f,
  105.4f,      76.4f,      53.4f,      35.2f,      20.7f,
   87.4f,      64.3f,      45.6f,      30.4f,      18.1f,
   74.1f,      55.2f,      39.6f,      26.7f,      16.0f,
   63.9f,      48.0f,      34.8f,      23.6f,      14.3f,
   55.8f,      42.3f,      30.8f,      21.1f,      12.9f,
   49.3f,      37.6f,      27.6f,      19.0f,      11.7f,
   43.9f,      33.7f,      24.9f,      17.3f,      10.7f,
   39.5f,      30.5f,      22.6f,      15.8f,       9.8f,
   35.7f,      27.7f,      20.6f,      14.4f,       9.0f,
   32.4f,      25.2f,      18.9f,      13.3f,       8.3f,
   29.6f,      23.1f,      17.4f,      12.2f,       7.7f,
   27.0f,      21.2f,      16.0f,      11.3f,       7.1f,
   24.9f,      19.6f,      14.8f,      10.5f,       6.6f,
   22.9f,      18.0f,      13.7f,       9.7f,       6.1f,
   21.1f,      16.7f,      12.7f,       9.0f,       5.8f,
   19.6f,      15.5f,      11.8f,       8.5f,       5.4f,
   18.1f,      14.4f,      10.9f,       7.8f,       5.0f,
   16.8f,      13.4f,      10.2f,       7.3f,       4.7f,
   15.6f,      12.4f,       9.5f,       6.8f,       4.4f,
   14.5f,      11.6f,       8.8f,       6.4f,       4.0f,
   13.5f,      10.8f,       8.2f,       5.9f,       3.8f,
   12.5f,       9.9f,       7.7f,       5.6f,       3.6f,
   11.6f,       9.2f,       7.0f,       5.1f,       3.3f,
   10.7f,       8.6f,       6.6f,       4.8f,       3.0f,
    9.9f,       7.9f,       6.0f,       4.5f,       2.9f,
    9.0f,       7.3f,       5.7f,       4.0f,       2.7f,
    8.3f,       6.7f,       5.0f,       3.8f,       2.5f,
    7.6f,       6.0f,       4.7f,       3.5f,       2.2f,
    6.8f,       5.5f,       4.1f,       3.0f,       2.0f,
    5.9f,       4.8f,       3.8f,       2.8f,       1.9f,
    5.0f,       4.0f,       3.0f,       2.1f,       1.6f
};     

float rltvThr_CFAROS_48[] = {
47449.9f,   14972.5f,    4702.2f,    1454.5f,     427.5f,
 4652.3f,    2134.2f,     965.4f,     422.9f,     171.1f,
 1423.5f,     780.2f,     418.4f,     214.9f,     100.5f,
  682.7f,     413.8f,     244.1f,     137.0f,      69.5f,
  409.2f,     264.3f,     165.6f,      98.3f,      52.5f,
  278.5f,     187.9f,     122.6f,      75.6f,      41.8f,
  205.4f,     142.9f,      96.1f,      60.9f,      34.6f,
  159.9f,     113.9f,      78.3f,      50.7f,      29.3f,
  129.3f,      93.8f,      65.6f,      43.2f,      25.4f,
  107.6f,      79.2f,      56.1f,      37.4f,      22.3f,
   91.5f,      68.1f,      48.8f,      32.9f,      19.8f,
   79.1f,      59.5f,      43.0f,      29.3f,      17.8f,
   69.4f,      52.6f,      38.4f,      26.3f,      16.0f,
   61.5f,      46.9f,      34.5f,      23.8f,      14.6f,
   55.0f,      42.3f,      31.2f,      21.7f,      13.4f,
   49.6f,      38.3f,      28.5f,      19.8f,      12.3f,
   45.0f,      34.9f,      26.0f,      18.2f,      11.4f,
   41.1f,      32.0f,      23.9f,      16.8f,      10.6f,
   37.7f,      29.5f,      22.1f,      15.6f,       9.8f,
   34.7f,      27.2f,      20.5f,      14.5f,       9.1f,
   32.0f,      25.2f,      19.0f,      13.5f,       8.6f,
   29.7f,      23.5f,      17.8f,      12.6f,       8.0f,
   27.6f,      21.8f,      16.6f,      11.8f,       7.5f,
   25.7f,      20.4f,      15.5f,      11.1f,       7.0f,
   24.0f,      19.0f,      14.6f,      10.4f,       6.7f,
   22.5f,      17.9f,      13.7f,       9.8f,       6.3f,
   21.0f,      16.8f,      12.8f,       9.2f,       5.9f,
   19.7f,      15.8f,      12.0f,       8.7f,       5.6f,
   18.5f,      14.8f,      11.4f,       8.2f,       5.3f,
   17.4f,      13.9f,      10.7f,       7.7f,       4.9f,
   16.4f,      13.1f,      10.0f,       7.3f,       4.7f,
   15.4f,      12.4f,       9.5f,       6.9f,       4.5f,
   14.5f,      11.7f,       8.9f,       6.5f,       4.2f,
   13.7f,      10.9f,       8.5f,       6.1f,       3.9f,
   12.8f,      10.3f,       7.9f,       5.8f,       3.8f,
   12.0f,       9.7f,       7.5f,       5.5f,       3.6f,
   11.3f,       9.0f,       7.0f,       5.1f,       3.4f,
   10.7f,       8.6f,       6.7f,       4.8f,       3.1f,
    9.9f,       8.0f,       6.2f,       4.6f,       2.9f,
    9.3f,       7.6f,       5.8f,       4.2f,       2.8f,
    8.7f,       6.9f,       5.5f,       3.9f,       2.7f,
    8.0f,       6.6f,       5.0f,       3.8f,       2.5f,
    7.5f,       5.9f,       4.7f,       3.5f,       2.2f,
    6.8f,       5.6f,       4.3f,       3.0f,       2.0f,
    6.1f,       4.9f,       3.9f,       2.9f,       1.9f,
    5.6f,       4.5f,       3.5f,       2.6f,       1.8f,
    4.8f,       3.9f,       2.9f,       2.0f,       1.5f
};

float rltvThr_CFAROS_56[] = {
 5444.4f,   2497.6f,   1129.8f,    494.9f,    200.3f,
 1668.6f,     914.5f,     490.4f,     251.9f,     117.8f,
  801.6f,     485.8f,     286.6f,     160.9f,      81.6f,
  481.2f,     310.8f,     194.7f,     115.6f,      61.7f,
  328.2f,     221.3f,     144.4f,      89.1f,      49.3f,
  242.5f,     168.7f,     113.4f,      71.9f,      40.8f,
  189.1f,     134.7f,      92.5f,      59.9f,      34.7f,
  153.2f,     111.1f,      77.7f,      51.1f,      30.0f,
  127.8f,      94.0f,      66.6f,      44.4f,      26.4f,
  108.9f,      81.1f,      58.1f,      39.2f,      23.6f,
   94.4f,      70.9f,      51.3f,      34.9f,      21.2f,
   82.9f,      62.9f,      45.8f,      31.4f,      19.2f,
   73.7f,      56.3f,      41.3f,      28.5f,      17.5f,
   66.1f,      50.8f,      37.5f,      26.0f,      16.1f,
   59.7f,      46.1f,      34.2f,      23.9f,      14.8f,
   54.4f,      42.2f,      31.5f,      22.0f,      13.7f,
   49.8f,      38.8f,      29.0f,      20.4f,      12.8f,
   45.8f,      35.8f,      26.9f,      18.9f,      11.9f,
   42.3f,      33.1f,      25.0f,      17.7f,      11.1f,
   39.2f,      30.8f,      23.3f,      16.5f,      10.5f,
   36.5f,      28.7f,      21.8f,      15.5f,       9.8f,
   34.0f,      26.9f,      20.4f,      14.6f,       9.2f,
   31.8f,      25.2f,      19.2f,      13.7f,       8.7f,
   29.8f,      23.7f,      18.0f,      12.9f,       8.2f,
   28.0f,      22.3f,      17.0f,      12.2f,       7.8f,
   26.3f,      21.0f,      16.0f,      11.6f,       7.4f,
   24.8f,      19.8f,      15.2f,      10.9f,       7.0f,
   23.4f,      18.7f,      14.4f,      10.4f,       6.7f,
   22.1f,      17.7f,      13.6f,       9.8f,       6.3f,
   20.9f,      16.8f,      12.9f,       9.4f,       6.0f,
   19.8f,      15.9f,      12.3f,       8.9f,       5.7f,
   18.8f,      15.0f,      11.7f,       8.5f,       5.5f,
   17.8f,      14.3f,      11.0f,       8.0f,       5.2f,
   16.9f,      13.6f,      10.6f,       7.7f,       4.9f,
   16.0f,      12.9f,      10.0f,       7.3f,       4.7f,
   15.2f,      12.3f,       9.6f,       6.9f,       4.5f,
   14.5f,      11.7f,       9.0f,       6.6f,       4.3f,
   13.8f,      11.0f,       8.6f,       6.3f,       4.0f,
   13.0f,      10.6f,       8.2f,       5.9f,       3.9f,
   12.4f,      10.0f,       7.8f,       5.7f,       3.7f,
   11.8f,       9.5f,       7.4f,       5.4f,       3.6f,
   11.1f,       9.0f,       7.0f,       5.1f,       3.4f,
   10.6f,       8.6f,       6.7f,       4.9f,       3.1f,
    9.9f,       8.0f,       6.3f,       4.7f,       3.0f,
    9.5f,       7.7f,       5.9f,       4.4f,       2.9f,
    8.9f,       7.2f,       5.7f,       4.0f,       2.8f,
    8.4f,       6.8f,       5.3f,       3.9f,       2.7f,
    7.9f,       6.4f,       4.9f,       3.7f,       2.5f,
    7.4f,       5.9f,       4.7f,       3.5f,       2.2f,
    6.9f,       5.6f,       4.4f,       3.1f,       2.0f,
    6.3f,       5.0f,       3.9f,       2.9f,       1.9f,
    5.8f,       4.8f,       3.7f,       2.8f,       1.9f,
    5.0f,       4.0f,       3.2f,       2.5f,       1.8f,
    4.6f,       3.7f,       2.9f,       2.0f,       1.4f
};

float rltvThr_CFAROS_64[] = {
 6236.5f,    2861.0f,    1294.2f,     566.9f,     229.4f,
 1913.6f,    1048.7f,     562.4f,     288.9f,     135.1f,
  920.4f,     557.8f,     329.1f,     184.8f,      93.7f,
  553.3f,     357.3f,     223.9f,     132.9f,      71.0f,
  377.8f,     254.8f,     166.3f,     102.6f,      56.7f,
  279.5f,     194.5f,     130.7f,      82.9f,      47.0f,
  218.3f,     155.5f,     106.8f,      69.2f,      40.0f,
  177.1f,     128.5f,      89.8f,      59.1f,      34.7f,
  147.9f,     108.8f,      77.1f,      51.5f,      30.6f,
  126.2f,      94.0f,      67.4f,      45.4f,      27.3f,
  109.6f,      82.4f,      59.6f,      40.6f,      24.6f,
   96.4f,      73.1f,      53.3f,      36.6f,      22.3f,
   85.8f,      65.5f,      48.1f,      33.2f,      20.4f,
   77.1f,      59.2f,      43.8f,      30.4f,      18.8f,
   69.8f,      53.9f,      40.0f,      27.9f,      17.3f,
   63.7f,      49.4f,      36.8f,      25.8f,      16.1f,
   58.4f,      45.5f,      34.0f,      23.9f,      15.0f,
   53.8f,      42.0f,      31.6f,      22.3f,      14.0f,
   49.8f,      39.0f,      29.5f,      20.8f,      13.1f,
   46.3f,      36.4f,      27.5f,      19.5f,      12.3f,
   43.1f,      34.0f,      25.8f,      18.3f,      11.6f,
   40.3f,      31.9f,      24.2f,      17.3f,      10.9f,
   37.8f,      29.9f,      22.8f,      16.3f,      10.4f,
   35.5f,      28.2f,      21.5f,      15.4f,       9.8f,
   33.5f,      26.6f,      20.4f,      14.6f,       9.3f,
   31.6f,      25.1f,      19.3f,      13.8f,       8.8f,
   29.8f,      23.8f,      18.3f,      13.1f,       8.4f,
   28.2f,      22.6f,      17.4f,      12.5f,       8.0f,
   26.8f,      21.5f,      16.5f,      11.9f,       7.7f,
   25.4f,      20.4f,      15.7f,      11.4f,       7.3f,
   24.1f,      19.4f,      14.9f,      10.8f,       6.9f,
   23.0f,      18.5f,      14.3f,      10.3f,       6.7f,
   21.9f,      17.6f,      13.6f,       9.9f,       6.4f,
   20.8f,      16.8f,      13.0f,       9.5f,       6.0f,
   19.9f,      16.0f,      12.4f,       9.0f,       5.8f,
   18.9f,      15.3f,      11.9f,       8.7f,       5.6f,
   18.1f,      14.7f,      11.4f,       8.3f,       5.4f,
   17.3f,      13.9f,      10.8f,       7.9f,       5.1f,
   16.6f,      13.4f,      10.4f,       7.6f,       4.9f,
   15.8f,      12.8f,       9.9f,       7.3f,       4.7f,
   15.1f,      12.2f,       9.6f,       6.9f,       4.6f,
   14.5f,      11.7f,       9.1f,       6.7f,       4.4f,
   13.8f,      11.2f,       8.7f,       6.4f,       4.1f,
   13.2f,      10.7f,       8.4f,       6.0f,       3.9f,
   12.7f,      10.2f,       7.9f,       5.9f,       3.8f,
   12.0f,       9.8f,       7.7f,       5.7f,       3.7f,
   11.6f,       9.4f,       7.3f,       5.4f,       3.6f,
   11.0f,       8.9f,       6.9f,       5.0f,       3.4f,
   10.5f,       8.6f,       6.7f,       4.9f,       3.2f,
   10.0f,       8.1f,       6.4f,       4.7f,       3.0f,
    9.6f,       7.8f,       6.0f,       4.5f,       2.9f,
    9.0f,       7.4f,       5.8f,       4.2f,       2.8f,
    8.7f,       7.0f,       5.6f,       4.0f,       2.7f,
    8.1f,       6.7f,       5.2f,       3.9f,       2.6f,
    7.8f,       6.3f,       4.9f,       3.7f,       2.5f,
    7.3f,       5.9f,       4.7f,       3.5f,       2.3f,
    6.9f,       5.7f,       4.4f,       3.2f,       2.0f,
    6.4f,       5.1f,       4.0f,       3.0f,       2.0f,
    5.9f,       4.9f,       3.8f,       2.9f,       1.9f,
    5.5f,       4.5f,       3.6f,       2.7f,       1.8f,
    4.9f,       4.0f,       3.0f,       2.3f,       1.7f,
    4.2f,       3.6f,       2.8f,       2.0f,       1.4f
};

float * rltvThr_CFAROS[] = {rltvThr_CFAROS_8, rltvThr_CFAROS_16, rltvThr_CFAROS_24, rltvThr_CFAROS_32, 
							rltvThr_CFAROS_40,rltvThr_CFAROS_48, rltvThr_CFAROS_56, rltvThr_CFAROS_64};
#endif

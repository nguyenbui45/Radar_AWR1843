/*! 
 *  \file   dss_dopplerProc_priv.c
 *
 *  \brief   Windowing and integration functions for doppler processing.
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

#include "odsdemo_dopplerProc_priv.h"

#ifdef _TMS320C6X
#include "c6x.h"
#endif


/*!
   \fn     RADARDEMO_dopplerProcWin2DFxdpinFltOut

   \brief   Windowing function for 2D FFT and prepare for the input to 2D FFT, fixed-point input, floating-point output.

   \param[in]    fftSize2D
               Input 2D FFT size.

   \param[in]    inputPtr
               Pointer to 16-bit I/Q input. Must be aligned to 8-byte boundary.

   \param[in]    inWinFunc
               Input 2D windowing function in doppler domain.

   \param[out]    outputPtr
               Pointer to floating-point output buffer after 2D windowing, in the order of real[0] imag[0] real[1] imag[1]... in memory.
               Must be aligned to 8-byte boundary.

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_dopplerProcWin2DFxdpinFltOut(
                            IN  uint32_t fftSize2D,
							IN  cplx16_t * inputPtr,
							IN  int16_t * RESTRICT inWinFunc,
							OUT float * outputPtr)
{
	int32_t		j;
	int64_t		lltemp1;
	int64_t		lltemp3;
	int64_t		* RESTRICT input1Ant1;
	__float2_t   * RESTRICT output1Ant1;
	int32_t  	win2;
	int32_t		itemp;

	input1Ant1	=	(int64_t  *) inputPtr;
	output1Ant1 =	(__float2_t  *) outputPtr;

	/* windowing - no assumption of symmetric*/
	for (j = 0; j < (int32_t) fftSize2D/4; j++)
	{
		lltemp1					=	_amem8(input1Ant1++);
		win2					=	_amem4(&inWinFunc[4*j]);
		lltemp3					=	_dcmpyr1(lltemp1, _dpack2(win2, 0));
		itemp					=	_loll(lltemp3);
		_amem8_f2(output1Ant1++) =	_dinthsp(_packlh2(itemp, itemp));
		itemp					=	_hill(lltemp3);
		_amem8_f2(output1Ant1++) =	_dinthsp(_packlh2(itemp, itemp));

		lltemp1					=	_amem8(input1Ant1++);
		win2					=	_amem4(&inWinFunc[4*j + 2]);
		lltemp3					=	_dcmpyr1(lltemp1, _dpack2(win2, 0));
		itemp					=	_loll(lltemp3);
		_amem8_f2(output1Ant1++) =	_dinthsp(_packlh2(itemp, itemp));
		itemp					=	_hill(lltemp3);
		_amem8_f2(output1Ant1++) =	_dinthsp(_packlh2(itemp, itemp));
	}


#if 0
	int32_t		j;
	int64_t		lltemp1;
	int64_t		lltemp3;
	int64_t		* RESTRICT input1Ant1, * RESTRICT input2Ant1;
	__float2_t   * RESTRICT output1Ant1, * RESTRICT output2Ant1;
	int32_t  	win2;
	int32_t		itemp;

	input1Ant1	=	(int64_t  *) inputPtr;
	output1Ant1 =	(__float2_t  *) outputPtr;

	input2Ant1	=	(int64_t  *) &inputPtr[fftSize2D - 2];
	output2Ant1 =	(__float2_t  *) &outputPtr[2 * fftSize2D - 2];
	/* windowing - assuming symmetric*/
	for (j = 0; j < (int32_t) fftSize2D/4; j++)
	{
		lltemp1					=	_amem8(input1Ant1++);
		win2					=	_amem4(&inWinFunc[2*j]);
		lltemp3					=	_dcmpyr1(lltemp1, _dpack2(win2, 0));
		itemp					=	_loll(lltemp3);
		_amem8_f2(output1Ant1++) =	_dinthsp(_packlh2(itemp, itemp));
		itemp					=	_hill(lltemp3);
		_amem8_f2(output1Ant1++) =	_dinthsp(_packlh2(itemp, itemp));

		win2					=	_rotl(win2, 16);
		lltemp1					=	_amem8(input2Ant1--);
		lltemp3					=	_dcmpyr1(lltemp1, _dpack2(win2, 0));
		itemp					=	_hill(lltemp3);
		_amem8_f2(output2Ant1--) =	_dinthsp(_packlh2(itemp, itemp));
		itemp					=	_loll(lltemp3);
		_amem8_f2(output2Ant1--) =	_dinthsp(_packlh2(itemp, itemp));
	}
#endif
}

/*!
   \fn     RADARDEMO_dopplerProcCRDCremoval

   \brief   Clutter removal by subtracting DC value.

   \param[in]    fftSize2D
               Input 2D FFT size.

   \param[in, out]    sigBuf
               Pointer to floating point I/Q samples.

   \pre       LITTLE ENDIAN ONLY

   \post      none


 */

void	RADARDEMO_dopplerProcCRDCremoval(
                            IN  uint32_t inputSize,
							OUT float * sigBuf)
{
	int32_t		j, inputSizeby2;
	__float2_t	* RESTRICT input1, * RESTRICT input2, sum1, sum2, f2temp, avg;
	__float2_t   * RESTRICT output1, * RESTRICT output2;
	float		ftemp;

	inputSizeby2	=	inputSize >> 1;
	input1		=	(__float2_t  *) &sigBuf[0];
	input2		=	(__float2_t  *) &sigBuf[inputSizeby2 << 1];
	sum1		=	_ftof2(0.f, 0.f);
	sum2		=	_ftof2(0.f, 0.f);
	ftemp		=	_rcpsp((float)inputSize);
	ftemp		=   ftemp * (2.f - ftemp * (float)inputSize);
	ftemp		=   ftemp * (2.f - ftemp * (float)inputSize);
	f2temp		=	_ftof2(ftemp, ftemp);

	/* windowing - no assumption of symmetric*/
	for (j = 0; j < (int32_t) inputSizeby2; j++)
	{
		sum1		=	_daddsp(sum1, _amem8_f2(input1++));
		sum2		=	_daddsp(sum2, _amem8_f2(input2++));
	}
	if (inputSize & 1)
		sum2		=	_daddsp(sum2, _amem8_f2(input2++));
	sum1		=	_daddsp(sum1, sum2);
	avg			=	_dmpysp(sum1, f2temp);

	output1		=	(__float2_t  *) &sigBuf[0];
	output2		=	(__float2_t  *) &sigBuf[inputSizeby2 << 1];

	for (j = 0; j < (int32_t) inputSizeby2; j++)
	{
		f2temp					=	_dsubsp(_amem8_f2(output1), avg);
		_amem8_f2(output1++)	=	f2temp;
		f2temp					=	_dsubsp(_amem8_f2(output2), avg);
		_amem8_f2(output2++)	=	f2temp;
	}
	if (inputSize & 1)
		_amem8_f2(output2)		=	_dsubsp(_amem8_f2(output2), avg);
}


/*!
   \fn     RADARDEMO_dopplerProcIntegrationFltp

   \brief   Single precision floating point integration, accumulate per rx ant.

   \param[in]    fftSize2D
               Input 2D FFT size.

   \param[in]    antIdx
               Index to antenna. First call to this function within the antenna loop, antIdx must be set to 0.

   \param[in]    inputPtr
               Single precision floating-point I/Q input, size of 2xFFT2Dsize. Must be aligned to 8-byte boundary.

   \param[in, out]    outputPtr
               Single precision power output (non-coherent combining) . Must be aligned to 8-byte boundary. This buffer is read-modify-write for antennas other than the first one.

   \pre       none

   \post      none

 */
void	RADARDEMO_dopplerProcIntegrationFltp(
                            IN  uint32_t fftSize2D,
							IN  int32_t antIdx,
							IN  float * inputPtr,
							INOUT  float * outputPtr)
{
	int32_t		j;
	__float2_t  * RESTRICT input1;
	float  *RESTRICT output;
	__float2_t	lltemp1;
	__float2_t	results;

	if (antIdx == 0)
	{
		input1 = (__float2_t  *) inputPtr;
		output = (float *) 	outputPtr;
		for (j = 0; j < (int32_t) fftSize2D; j+=2)
		{
			lltemp1		=	_amem8_f2(input1++);
			results     =	_dmpysp(lltemp1,lltemp1);
			*output++ 	= 	_hif2(results) + _lof2(results);
			lltemp1		=	_amem8_f2(input1++);
			results     =	_dmpysp(lltemp1,lltemp1);
			*output++ 	= 	_hif2(results) + _lof2(results);
		}
	}
	else
	{
		input1 = (__float2_t  *) inputPtr;
		output = (float *) 	outputPtr;
		for (j = 0; j < (int32_t) fftSize2D; j+=2)
		{
			lltemp1		=	_amem8_f2(input1++);
			results     =	_dmpysp(lltemp1,lltemp1);
			*output++ 	+= 	_hif2(results) + _lof2(results);
			lltemp1		=	_amem8_f2(input1++);
			results     =	_dmpysp(lltemp1,lltemp1);
			*output++ 	+= 	_hif2(results) + _lof2(results);
		}
	}
}

/*!
   \fn     RADARDEMO_dopplerProcIntegrationFltpAllAnt

   \brief   Single precision floating point integration, accumulate all rx ant.

   \param[in]    fftSize2D
               Input 2D FFT size.

   \param[in]    numAnt
               Number of antenna.

   \param[in]    inputPtr
               Single precision floating-point I/Q input, size of 2xFFT2Dsize, store one antenna after another. Must be aligned to 8-byte boundary.

   \param[in, out]    outputPtr
               Single precision power output (non-coherent combining) . Must be aligned to 8-byte boundary. This buffer is read-modify-write for antennas other than the first one.

   \pre       none

   \post      none

 */
void	RADARDEMO_dopplerProcIntegrationFltpAllAnt(
                            IN  uint32_t fftSize2D,
							IN  int32_t numAnt,
							IN  float * inputPtr,
							INOUT  float * outputPtr)
{
	int32_t		j;
	__float2_t	lltemp1;
	__float2_t	results1, results2;

	if (numAnt == 4)
	{
		__float2_t  * RESTRICT input1;
		__float2_t  * RESTRICT input2;
		__float2_t  * RESTRICT input3;
		__float2_t  * RESTRICT input4;
		__float2_t  *RESTRICT output;

		input1 = (__float2_t  *) inputPtr;
		input2 = (__float2_t  *) &inputPtr[2 * fftSize2D];
		input3 = (__float2_t  *) &inputPtr[4 * fftSize2D];
		input4 = (__float2_t  *) &inputPtr[6 * fftSize2D];
		output = (__float2_t *) 	outputPtr;
		for (j = 0; j < (int32_t) fftSize2D; j+=2)
		{
			lltemp1		=	_amem8_f2(input1++);
			results1    =	_dmpysp(lltemp1,lltemp1);
			lltemp1		=	_amem8_f2(input2++);
			results1	=	_daddsp(results1, _dmpysp(lltemp1,lltemp1));
			lltemp1		=	_amem8_f2(input3++);
			results1	=	_daddsp(results1, _dmpysp(lltemp1,lltemp1));
			lltemp1		=	_amem8_f2(input4++);
			results1	=	_daddsp(results1, _dmpysp(lltemp1,lltemp1));

			lltemp1		=	_amem8_f2(input1++);
			results2    =	_dmpysp(lltemp1,lltemp1);
			lltemp1		=	_amem8_f2(input2++);
			results2	=	_daddsp(results2, _dmpysp(lltemp1,lltemp1));
			lltemp1		=	_amem8_f2(input3++);
			results2	=	_daddsp(results2, _dmpysp(lltemp1,lltemp1));
			lltemp1		=	_amem8_f2(input4++);
			results2	=	_daddsp(results2, _dmpysp(lltemp1,lltemp1));

			_amem8_f2(output++)	=	_ftof2(_hif2(results2) + _lof2(results2), _hif2(results1) + _lof2(results1));
		}
	}
	else if (numAnt == 8)
	{
		__float2_t  * RESTRICT input1;
		__float2_t  * RESTRICT input2;
		__float2_t  * RESTRICT input3;
		__float2_t  * RESTRICT input4;
		__float2_t  * RESTRICT input5;
		__float2_t  * RESTRICT input6;
		__float2_t  * RESTRICT input7;
		__float2_t  * RESTRICT input8;
		float 		* RESTRICT output;

		input1 = (__float2_t  *) inputPtr;
		input2 = (__float2_t  *) &inputPtr[2 * fftSize2D];
		input3 = (__float2_t  *) &inputPtr[4 * fftSize2D];
		input4 = (__float2_t  *) &inputPtr[6 * fftSize2D];
		input5 = (__float2_t  *) &inputPtr[8 * fftSize2D];
		input6 = (__float2_t  *) &inputPtr[10 * fftSize2D];
		input7 = (__float2_t  *) &inputPtr[12 * fftSize2D];
		input8 = (__float2_t  *) &inputPtr[14 * fftSize2D];
		output = (float *) 	outputPtr;
		for (j = 0; j < (int32_t) fftSize2D; j++)
		{
			lltemp1		=	_amem8_f2(input1++);
			results1    =	_dmpysp(lltemp1,lltemp1);
			lltemp1		=	_amem8_f2(input2++);
			results1	=	_daddsp(results1, _dmpysp(lltemp1,lltemp1));
			lltemp1		=	_amem8_f2(input3++);
			results1	=	_daddsp(results1, _dmpysp(lltemp1,lltemp1));
			lltemp1		=	_amem8_f2(input4++);
			results1	=	_daddsp(results1, _dmpysp(lltemp1,lltemp1));
			lltemp1		=	_amem8_f2(input5++);
			results1	=	_daddsp(results1, _dmpysp(lltemp1,lltemp1));
			lltemp1		=	_amem8_f2(input6++);
			results1	=	_daddsp(results1, _dmpysp(lltemp1,lltemp1));
			lltemp1		=	_amem8_f2(input7++);
			results1	=	_daddsp(results1, _dmpysp(lltemp1,lltemp1));
			lltemp1		=	_amem8_f2(input8++);
			results1	=	_daddsp(results1, _dmpysp(lltemp1,lltemp1));
			*output++	=	_hif2(results1) + _lof2(results1);
		}
	}
#if 0
	else
	{
		__float2_t  * RESTRICT input1;
		float 		* RESTRICT output;
		int32_t i;

		/* first antenna */
		input1 = (__float2_t  *) inputPtr;
		output = (float *) 	outputPtr;
		for (j = 0; j < (int32_t) fftSize2D; j+=2)
		{
			lltemp1		=	_amem8_f2(input1++);
			results1     =	_dmpysp(lltemp1,lltemp1);
			*output++ 	= 	_hif2(results1) + _lof2(results1);
			lltemp1		=	_amem8_f2(input1++);
			results1     =	_dmpysp(lltemp1,lltemp1);
			*output++ 	= 	_hif2(results1) + _lof2(results1);
		}

		for (i = 1; i < numAnt; i++ )
		{
			input1 = (__float2_t  *) &inputPtr[2 * fftSize2D * i];
			output = (float *) 	outputPtr;
			for (j = 0; j < (int32_t) fftSize2D; j+=2)
			{
				lltemp1		=	_amem8_f2(input1++);
				results1     =	_dmpysp(lltemp1,lltemp1);
				*output++ 	+= 	_hif2(results1) + _lof2(results1);
				lltemp1		=	_amem8_f2(input1++);
				results1     =	_dmpysp(lltemp1,lltemp1);
				*output++ 	+= 	_hif2(results1) + _lof2(results1);
			}
		}
	}
#endif
}

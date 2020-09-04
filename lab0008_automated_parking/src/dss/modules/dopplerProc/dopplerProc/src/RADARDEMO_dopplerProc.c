/*! 
 *  \file   RADARDEMO_dopplerProc.c
 *
 *  \brief   Range processing. 
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

#include <modules/dopplerProc/dopplerProc/api/RADARDEMO_dopplerProc.h>
#include "RADARDEMO_dopplerProc_priv.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DEBUG(_x) //_x

#define PI (3.141592654)

#ifdef _TMS320C6X
#include "c6x.h"
#endif


static unsigned char brev[64] = {
    0x0, 0x20, 0x10, 0x30, 0x8, 0x28, 0x18, 0x38,
    0x4, 0x24, 0x14, 0x34, 0xc, 0x2c, 0x1c, 0x3c,
    0x2, 0x22, 0x12, 0x32, 0xa, 0x2a, 0x1a, 0x3a,
    0x6, 0x26, 0x16, 0x36, 0xe, 0x2e, 0x1e, 0x3e,
    0x1, 0x21, 0x11, 0x31, 0x9, 0x29, 0x19, 0x39,
    0x5, 0x25, 0x15, 0x35, 0xd, 0x2d, 0x1d, 0x3d,
    0x3, 0x23, 0x13, 0x33, 0xb, 0x2b, 0x1b, 0x3b,
    0x7, 0x27, 0x17, 0x37, 0xf, 0x2f, 0x1f, 0x3f
};

/*! 
   \fn     RADARDEMO_dopplerProc_create
 
   \brief   Create and initialize RADARDEMO_dopplerProc module. 
  
   \param[in]    moduleConfig
               Pointer to input configurations structure for RADARDEMO_dopplerProc module.

   \param[out]    errorCode
               Output error code.

			   
   \ret     void pointer to the module handle. Return value of NULL indicates failed module creation.
			   
   \pre       none
 
   \post      none
  
 
 */

void	* RADARDEMO_dopplerProc_create(
                            IN  RADARDEMO_dopplerProc_config * moduleConfig, 
							OUT RADARDEMO_dopplerProc_errorCode * errorCode)
							
{
	int32_t		i;
	RADARDEMO_dopplerProc_handle * handle;
	
	*errorCode	=	RADARDEMO_DOPPLERPROC_NO_ERROR;

	/* Check error configurations */
	/* unsupported FFT type */
	if (moduleConfig->fft2DType >= RADARDEMO_DOPPLERPROC_2DFFT_NOT_SUPPORTED)
		*errorCode	=	RADARDEMO_DOPPLERPROC_2DFFTTYPE_NOTSUPPORTED;
		
	/* unsupported integration type */
	if (moduleConfig->intgrType >= RADARDEMO_DOPPLERPROC_INTGRTYPE_NOT_SUPPORTED)
		*errorCode	=	RADARDEMO_DOPPLERPROC_INTEGRTYPE_NOTSUPPORTED;
	
	/* mismatch format */
	if ((moduleConfig->fxdpInputFlag == 1) && (moduleConfig->include2DwinFlag == 0))
		*errorCode	=	RADARDEMO_DOPPLERPROC_INPUTANDFFTTYPE_MISMATCH;
	if ((moduleConfig->fxdpInputFlag == 0) && (moduleConfig->include2DwinFlag == 1))
		*errorCode	=	RADARDEMO_DOPPLERPROC_INPUTANDFFTTYPE_MISMATCH;

	i	=	30 - _norm(moduleConfig->fft2DSize);
	if ( (1 << i) != moduleConfig->fft2DSize)
		*errorCode	=	RADARDEMO_DOPPLERPROC_2DFFTSIZE_NOTVALID;	

	if (*errorCode > RADARDEMO_DOPPLERPROC_NO_ERROR)
		return (NULL);
		
	handle						=	(RADARDEMO_dopplerProc_handle *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_dopplerProc_handle), 1);
	if (handle == NULL)
	{
		*errorCode	=	RADARDEMO_DOPPLERPROC_FAIL_ALLOCATE_HANDLE;
		return (handle);
	}
	memset(handle, 0, sizeof(RADARDEMO_dopplerProc_handle));
	handle->nRxAnt				=	moduleConfig->nRxAnt;
	handle->nRxPhyAnt           =   moduleConfig->nRxPhyAnt;
	handle->fft2DSize			=	moduleConfig->fft2DSize;
	handle->numChirpsPerFrame	=	moduleConfig->numChirpsPerFrame;
	handle->fft2DType			=	(uint8_t) moduleConfig->fft2DType;
	handle->intgrType			=	(uint8_t) moduleConfig->intgrType;
	handle->include2DwinFlag	=	moduleConfig->include2DwinFlag;
	handle->fxdpInputFlag		=	moduleConfig->fxdpInputFlag;
	handle->DCRemovalFlag		=	moduleConfig->DCRemovalFlag;

	if (handle->include2DwinFlag == 1)
	{
		if(handle->fxdpInputFlag == 1)
		{
			int16_t *tempPtr;
			int32_t itemp;
			handle->win2D		=	(void *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->fft2DSize * sizeof(int16_t), 8);
			if (handle->win2D == NULL)
			{
				*errorCode	=	RADARDEMO_DOPPLERPROC_FAIL_ALLOCATE_LOCALINSTMEM;
				return (handle);
			}
			tempPtr				=	(int16_t *)handle->win2D;
			for (i = 0; i < moduleConfig->numChirpsPerFrame; i++)
			{
				tempPtr[i] 	=	32767;
			}
			for (i = moduleConfig->numChirpsPerFrame; i < moduleConfig->fft2DSize; i++)
			{
				tempPtr[i] 	=	0;
			}

			for (i = 0; i < (int32_t) moduleConfig->win2DSize; i++ )
			{
				itemp			=	(int32_t)(moduleConfig->win2D[i] * 32768.f + 0.5f);
				if (itemp >= 32768) itemp = 32767;
				tempPtr[i]		=	(int16_t)itemp;
				tempPtr[moduleConfig->numChirpsPerFrame - 1 - i]		=	(int16_t)itemp;
			}
		}
		else
		{
			float *tempPtr;
			handle->win2D		=	(void *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->fft2DSize * sizeof(float), 8);
			if (handle->win2D == NULL)
			{
				*errorCode	=	RADARDEMO_DOPPLERPROC_FAIL_ALLOCATE_LOCALINSTMEM;
				return (handle);
			}
			tempPtr				=	(float *)handle->win2D;
			for (i = 0; i < moduleConfig->numChirpsPerFrame; i++)
			{
				tempPtr[i] 	=	1.f;
			}
			for (i = moduleConfig->numChirpsPerFrame; i < moduleConfig->fft2DSize; i++)
			{
				tempPtr[i] 	=	0.f;
			}

			for (i = 0; i < (int32_t) moduleConfig->win2DSize; i++ )
			{
				tempPtr[i]		=	moduleConfig->win2D[i];
				tempPtr[moduleConfig->numChirpsPerFrame - 1 - i]		=	moduleConfig->win2D[i];
			}
		}
	}

	if (moduleConfig->fft2DType == RADARDEMO_DOPPLERPROC_2DFFT_SPxSP)
	{
		float * tempTPtr;
		float re, im;
		handle->twiddle			=	(void *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, 2 * handle->fft2DSize * sizeof(float), 8);
		if (handle->twiddle == NULL)
		{
			*errorCode	=	RADARDEMO_DOPPLERPROC_FAIL_ALLOCATE_LOCALINSTMEM;
			return (handle);
		}
		tempTPtr				=	(float *) handle->twiddle;
		tw_gen_float(tempTPtr, handle->fft2DSize);
		if (handle->include2DwinFlag == 1)
		{
			handle->scratchPadSize  =   (1 + handle->nRxAnt) * 2 * handle->fft2DSize * sizeof(float);
		}
		else
		{
			handle->scratchPadSize  =   2 * handle->nRxAnt * handle->fft2DSize * sizeof(float);
		}
		handle->scratchPad		=	(void *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 1, handle->scratchPadSize, 8);

		re						=	(float) cos(PI *_rcpsp((float)handle->fft2DSize));
		im						=	(float) sin(PI *_rcpsp((float)handle->fft2DSize));
		handle->ejPIover2DFFT  =	_ftof2(re, im);

		if (handle->scratchPad == NULL)
		{
			*errorCode	=	RADARDEMO_DOPPLERPROC_FAIL_ALLOCATE_LOCALINSTMEM;
			return (handle);
		}
	}
	
	return((void *)handle);
}

/*! 
   \fn     RADARDEMO_dopplerProc_delete
 
   \brief   Delete RADARDEMO_dopplerProc module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
  
 
 */

void	RADARDEMO_dopplerProc_delete(
                            IN  void * handle)
{
	RADARDEMO_dopplerProc_handle *dopplerProcInst;
	
	dopplerProcInst	=	(RADARDEMO_dopplerProc_handle *) handle;
	
	if ((RADARDEMO_dopplerProc_2DFFTType)dopplerProcInst->fft2DType == RADARDEMO_DOPPLERPROC_2DFFT_SPxSP)
	{
		radarOsal_memFree(dopplerProcInst->scratchPad, dopplerProcInst->scratchPadSize);
		radarOsal_memFree(dopplerProcInst->twiddle, dopplerProcInst->fft2DSize * 2 * sizeof(float));
	}		
	if (dopplerProcInst->include2DwinFlag == 1)
	{
		if(dopplerProcInst->fxdpInputFlag == 1)
		{
			radarOsal_memFree(dopplerProcInst->win2D, (dopplerProcInst->fft2DSize) * sizeof(int16_t));
		}
		else
		{
			radarOsal_memFree(dopplerProcInst->win2D, (dopplerProcInst->fft2DSize) * sizeof(float));
		}
	}

	radarOsal_memFree(handle, sizeof(RADARDEMO_dopplerProc_handle));
}



/*! 
   \fn     RADARDEMO_dopplerProc_run
 
   \brief   Range processing, always called per chirp per antenna.
  
   \param[in]    handle
               Module handle.
 
   \param[in]    dopplerProcInput
               Pointer to Doppler input structure.
 
   \param[out]    outputSignal
               Output signal from doppler processing.
			   If module is called for range-Doppler heatmap generation, then the output signal is power profile in floating-point.
			   If module is called for reconstructingt the antenna data for angle estimation after Doppler FFT, 
			        then the output signal is complex antenna sample in floating-point, imaginary first and real next in the memroy.
 
   \ret error code

   \pre       none
 
   \post      none
  
 
 */

RADARDEMO_dopplerProc_errorCode	RADARDEMO_dopplerProc_run(
                            IN  void * handle,
							IN RADARDEMO_dopplerProc_input * sDopplerProcInput,
							OUT float  * outputSignal)

{
	int32_t		j, nAnt, nPhyAnt, k;
	RADARDEMO_dopplerProc_handle *dopplerProcInst;
	int32_t rad2D;
	RADARDEMO_dopplerProc_errorCode errorCode = RADARDEMO_DOPPLERPROC_NO_ERROR;
	__float2_t rotation, rotation2x;
	//int32_t sign;
	__float2_t  tempIn;

	dopplerProcInst	=	(RADARDEMO_dopplerProc_handle *) handle;
	nAnt		=	(int32_t) dopplerProcInst->nRxAnt;
	nPhyAnt      =   (int32_t) dopplerProcInst->nRxPhyAnt;
	
#ifndef _WIN32
	if ( ( (uint32_t)outputSignal & 0x7) != 0 )
		errorCode	=	RADARDEMO_DOPPLERPROC_INOUTPTR_NOTCORRECT;
	if (((uint32_t)sDopplerProcInput->dopplerProcInput & 0x7) != 0)
		errorCode	=	RADARDEMO_DOPPLERPROC_INOUTPTR_NOTCORRECT;
#endif
	if (outputSignal == NULL)
		errorCode	=	RADARDEMO_DOPPLERPROC_INOUTPTR_NOTCORRECT;
	if (sDopplerProcInput->dopplerProcInput == NULL)
		errorCode	=	RADARDEMO_DOPPLERPROC_INOUTPTR_NOTCORRECT;
	if (dopplerProcInst->include2DwinFlag == 1)
		if (dopplerProcInst->win2D == NULL)
			errorCode	=	RADARDEMO_DOPPLERPROC_INOUTPTR_NOTCORRECT;
	if (dopplerProcInst->scratchPad == NULL)
		errorCode	=	RADARDEMO_DOPPLERPROC_INOUTPTR_NOTCORRECT;

	if ( errorCode > RADARDEMO_DOPPLERPROC_NO_ERROR)
		return (errorCode);

	j  = 30 - _norm(dopplerProcInst->fft2DSize);
	if ((j & 1) == 0)
		rad2D = 4;
	else
		rad2D = 2;

	if ((sDopplerProcInput->reGen2DFFTout4AoAFlag == 1) && (sDopplerProcInput->dopplerComp4TDMMimo == 1))
	{
		float re, im, phi;
		if ((int32_t)sDopplerProcInput->dopplerIndex >= (int32_t)(dopplerProcInst->fft2DSize >> 1))
		{
			k		=	(int32_t)sDopplerProcInput->dopplerIndex - ((int32_t)dopplerProcInst->fft2DSize >> 1);
		}
		else
		{
			k		=	(int32_t)sDopplerProcInput->dopplerIndex + ((int32_t)dopplerProcInst->fft2DSize >> 1);
		}
        phi			=	-2*PI*(float)(k - ((int32_t)dopplerProcInst->fft2DSize >> 1))/((float)dopplerProcInst->fft2DSize * (float)(3));

		re			=	(float) cos(phi);
		im			=	(float) sin(phi);
		rotation	=	_ftof2(re, im);
        re          =   (float) cos(phi*2.0);
        im          =   (float) sin(phi*2.0);
        rotation2x    =   _ftof2(re, im);

	}

	if ((RADARDEMO_dopplerProc_2DFFTType)dopplerProcInst->fft2DType == RADARDEMO_DOPPLERPROC_2DFFT_SPxSP)
	{
		if (dopplerProcInst->fxdpInputFlag == 1)
		{
			if (dopplerProcInst->include2DwinFlag == 1)
			{
				cplx16_t	** tempPtr;
				int16_t		* inWinFunc;
				float		* tempWin2DOutPtr;
				float		* tempFFT2DOutPtr, * tempFFT2DOutPtr2;

				tempPtr		=	(cplx16_t**)sDopplerProcInput->dopplerProcInput;
				inWinFunc		=	(int16_t *)dopplerProcInst->win2D;
				tempWin2DOutPtr	=	(float *) dopplerProcInst->scratchPad;
				tempFFT2DOutPtr	=	(float *) &tempWin2DOutPtr[2 * dopplerProcInst->fft2DSize];

				for (j = 0; j < nAnt; j++)
				{
					tempFFT2DOutPtr2	=	(float *) &tempFFT2DOutPtr[j * 2 * dopplerProcInst->fft2DSize];
					RADARDEMO_dopplerProcWin2DFxdpinFltOut(
                            dopplerProcInst->fft2DSize,
							tempPtr[j],
							inWinFunc,
							tempWin2DOutPtr);

					if(dopplerProcInst->DCRemovalFlag)
						RADARDEMO_dopplerProcCRDCremoval(
                            dopplerProcInst->numChirpsPerFrame,
							tempWin2DOutPtr);

					DSPF_sp_fftSPxSP (
							dopplerProcInst->fft2DSize,
							tempWin2DOutPtr,
							(float *)dopplerProcInst->twiddle, 
							tempFFT2DOutPtr2,
							brev,
							rad2D,
							0,
							dopplerProcInst->fft2DSize);

					if(sDopplerProcInput->reGen2DFFTout4AoAFlag == 1)
					{
						if ((sDopplerProcInput->dopplerComp4TDMMimo == 1) && j >= (nPhyAnt << 1) )
						{   //ant 8-11
							tempIn 		=	_ftof2(tempFFT2DOutPtr2[2 * sDopplerProcInput->dopplerIndex], tempFFT2DOutPtr2[2 * sDopplerProcInput->dopplerIndex + 1]);
							tempIn		=	_complex_mpysp(rotation2x, tempIn);
							_amem8_f2(&outputSignal[2 * j])		=	tempIn;
						}
						else if ((sDopplerProcInput->dopplerComp4TDMMimo == 1) && j >= (nPhyAnt) )
                        {   //ant 4-7
                            tempIn      =   _ftof2(tempFFT2DOutPtr2[2 * sDopplerProcInput->dopplerIndex], tempFFT2DOutPtr2[2 * sDopplerProcInput->dopplerIndex + 1]);
                            tempIn      =   _complex_mpysp(rotation, tempIn);
                            _amem8_f2(&outputSignal[2 * j])     =   tempIn;
                        }
						else
						{   //ant 0-3
							_amem8_f2(&outputSignal[2 * j]) = _ftof2(tempFFT2DOutPtr2[2 * sDopplerProcInput->dopplerIndex], tempFFT2DOutPtr2[2 * sDopplerProcInput->dopplerIndex + 1]);
						}

					}
				}
				if(sDopplerProcInput->reGen2DFFTout4AoAFlag == 0)
					RADARDEMO_dopplerProcIntegrationFltpAllAnt(
						dopplerProcInst->fft2DSize,
						nAnt,
						tempFFT2DOutPtr,
						outputSignal);
			}
		}
		else
		{
			if (dopplerProcInst->include2DwinFlag == 0)
			{
				float		** tempPtr;
				float		* tempFFT2DOutPtr, * tempFFT2DOutPtr2;
				int32_t     i;

				tempPtr		=	(float**)sDopplerProcInput->dopplerProcInput;
				tempFFT2DOutPtr	=	(float *) dopplerProcInst->scratchPad;

				for (j = 0; j < nAnt; j++)
				{
					tempFFT2DOutPtr2	=	(float *) &tempFFT2DOutPtr[j * 2 * dopplerProcInst->fft2DSize];

					if(dopplerProcInst->DCRemovalFlag)
						RADARDEMO_dopplerProcCRDCremoval(
                            dopplerProcInst->numChirpsPerFrame,
							tempPtr[j]);
					for (i = dopplerProcInst->numChirpsPerFrame; i < dopplerProcInst->fft2DSize; i++)
					{
						_amem8_f2(&tempPtr[j][2 * i]) 	=	_ftof2(0.f, 0.f);
					}
					DSPF_sp_fftSPxSP (
							dopplerProcInst->fft2DSize,
							tempPtr[j],
							(float *)dopplerProcInst->twiddle, 
							tempFFT2DOutPtr2,
							brev,
							rad2D,
							0,
							dopplerProcInst->fft2DSize);
					if(sDopplerProcInput->reGen2DFFTout4AoAFlag == 1)
					{
						if ((sDopplerProcInput->dopplerComp4TDMMimo == 1) && j >= (nAnt >> 1))
						{
							tempIn 			=	_amem8_f2(&tempFFT2DOutPtr2[2 * sDopplerProcInput->dopplerIndex]);
							tempIn			=	_ftof2(_lof2(tempIn), _hif2(tempIn));
							tempIn			=	_complex_mpysp(rotation, tempIn);
							_amem8_f2(&outputSignal[2 * j])		=	tempIn;
						}
						else
						{
							_amem8_f2(&outputSignal[2 * j]) = _amem8_f2(&tempFFT2DOutPtr2[2 * sDopplerProcInput->dopplerIndex]);
						}

					}
				}
				if(sDopplerProcInput->reGen2DFFTout4AoAFlag == 0)
					RADARDEMO_dopplerProcIntegrationFltpAllAnt(
						dopplerProcInst->fft2DSize,
						nAnt,
						tempFFT2DOutPtr,
						outputSignal);
			}
		}
	}
	return (errorCode);
}


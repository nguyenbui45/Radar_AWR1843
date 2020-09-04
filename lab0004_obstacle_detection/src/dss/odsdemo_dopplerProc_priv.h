/*! 
 *  \file   odsdemo_dopplerProc_priv.h
 *
 *  \brief   Header file for RADARDEMO_dopplerProc module's internal functions
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

#ifndef ODSDEMO_DOPPLERPROC_RPIV_H
#define ODSDEMO_DOPPLERPROC_RPIV_H

#include <swpform.h>
#include <radar_c674x.h>

#define WIN1DLEFTSHFTGAP (2) 


/*!
   \fn     RADARDEMO_dopplerProcWin2DFxdpinFltOut

   \brief   Windowing function for 2D FFT and prepare for the input to 2D FFT.

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

extern void	RADARDEMO_dopplerProcWin2DFxdpinFltOut(
                            IN  uint32_t fftSize2D,
							IN  cplx16_t * inputPtr,
							IN  int16_t * RESTRICT inWinFunc,
							OUT float * outputPtr);						

/*!
   \fn     RADARDEMO_dopplerProcIntegrationFltp

   \brief   Single precision floating point integration, accumulate all rx ant.

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

extern void	RADARDEMO_dopplerProcIntegrationFltp(
                            IN  uint32_t fftSize2D,
							IN  int32_t antIdx,
							IN  float * inputPtr,
							INOUT  float * outputPtr);



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
extern void	RADARDEMO_dopplerProcIntegrationFltpAllAnt(
                            IN  uint32_t fftSize2D,
							IN  int32_t numAnt,
							IN  float * inputPtr,
							INOUT  float * outputPtr);

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

extern void	RADARDEMO_dopplerProcCRDCremoval(
                            IN  uint32_t fftSize2D,
							OUT float * sigBuf);

extern void tw_gen_float (float *w, int n);

#ifdef _WIN32
extern void DSPF_sp_fftSPxSP (int N, float *ptr_x, float *ptr_w, float *ptr_y,
    unsigned char *brev, int n_min, int offset, int n_max);
#endif //_WIN32

#endif //ODSDEMO_DOPPLERPROC_RPIV_H


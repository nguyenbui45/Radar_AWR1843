/*!
 *  \file   RADARDEMO_aoaEstBF2D.c
 *  \brief   Estimate the angle of arrival using Capon BF.
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

#include <math.h>
#include <string.h>

/* C674x mathlib */
#include <ti/mathlib/mathlib.h>
#include "radar_c674x.h"
#include "DSPF_sp_fftSPxSP.h"
#include "./common/odsdemo_common.h"
#include "./common/mmw_config.h"
#include <float.h>
#include "dss_mmw.h"
#include "swpform.h"
#include "odsdemo_aoaEstBF2D.h"

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
//#include "c6x.h"
#endif

extern ODSDemo_DataPathObj odsdemo_dataPathObj;
extern MmwDemo_DSS_MCB     gMmwDssMCB;

ODSDEMO_aoAEstBF2D_output  odsdemo_doaOutput;
extern uint16_t antenna2DMAP_a[ODSDEMO_MAX_RX_ANT];
extern uint16_t antenna2DMAP_e[ODSDEMO_MAX_RX_ANT];

//! \copydoc ODSDEMO__aoaEstBF2D
ODSDEMO_aoaEstBF2D_errorCode    ODSDEMO__aoaEstBF2D(
                            IN  float range,
                            IN  cplxf_t * inputSignal,
                            OUT ODSDEMO_aoAEstBF2D_output * estOutput)
{
    uint16_t    antInd, eInd, aInd, col_idx, row_idx, offset, bufferSize;
    int16_t     fft_2D_peak_row_idx, fft_2D_peak_col_idx;
    float       el_freq, az_freq;
    float       theta, phi, temp;
    float       maxVal, mag_sqr;
    unsigned char *brev = NULL;
    cplxfReIm_t     * scratchPadPtr;
    cplxfReIm_t     * DOA_2D_storage;
    float       *fftIn, *fftOut;

    ODSDEMO_aoaEstBF2D_errorCode errorCode = ODSDEMO_AOABF2D_NO_ERROR;

    scratchPadPtr = (cplxfReIm_t *)&odsdemo_dataPathObj.scratchPad[0];
    fftIn = (float *)&scratchPadPtr[0];
    fftOut = (float *)&scratchPadPtr[ODSDEMO_DOA2D_FFTSIZE];
    DOA_2D_storage  =  &scratchPadPtr[(ODSDEMO_DOA2D_FFTSIZE << 1)];

    // set zero to all the entry in DOA_2D_storage
    bufferSize = ODSDEMO_DOA2D_FFTSIZE * (odsdemo_dataPathObj.maxIndElevationAntMapping + 1) * sizeof(cplxfReIm_t);
    memset(DOA_2D_storage, 0, bufferSize);

    /*Arrange the complex input across virtual antennas in 2D grid based on ODS antenna placement*/
    for (antInd = 0; antInd < gMmwDssMCB.dataPathObj[0].numVirtualAntennas; antInd ++)
    {
        aInd = antenna2DMAP_a[antInd];
        eInd = antenna2DMAP_e[antInd];
        DOA_2D_storage[eInd * ODSDEMO_DOA2D_FFTSIZE + aInd].real = inputSignal[antInd].real;
        DOA_2D_storage[eInd * ODSDEMO_DOA2D_FFTSIZE + aInd].imag = inputSignal[antInd].imag;
    }

    // first FFT
    offset = 0;
    for (eInd = 0; eInd <= odsdemo_dataPathObj.maxIndElevationAntMapping; eInd ++)
    {
        // FFT input take the real first
        DSPF_sp_fftSPxSP (
                ODSDEMO_DOA2D_FFTSIZE,
                (float *)&DOA_2D_storage[offset],
                (float *)&gMmwDssMCB.dataPathObj[0].twiddle_DOA[0],
                fftOut,
                brev,
                gMmwDssMCB.dataPathObj[0].DOA2D_rad2D,
                0,
                ODSDEMO_DOA2D_FFTSIZE);

        // FFT output give the real first
        for (col_idx = 0;col_idx < ODSDEMO_DOA2D_FFTSIZE; col_idx++)
        {
            DOA_2D_storage[offset + col_idx].real =   fftOut[col_idx * 2];
            DOA_2D_storage[offset + col_idx].imag =   fftOut[col_idx * 2 + 1];
        }
        offset += ODSDEMO_DOA2D_FFTSIZE;
    }

    // second FFT
    maxVal = 0;
    bufferSize = ODSDEMO_DOA2D_FFTSIZE * sizeof(cplxfReIm_t);
    for (col_idx = 0; col_idx < ODSDEMO_DOA2D_FFTSIZE; col_idx ++)
    {
        // FFT input take the real first
        offset = 0;
        memset(fftIn, 0, bufferSize);
        for (row_idx = 0; row_idx <= odsdemo_dataPathObj.maxIndElevationAntMapping; row_idx ++)
        {
            fftIn[row_idx * 2] = DOA_2D_storage[offset + col_idx].real;
            fftIn[row_idx * 2 + 1] = DOA_2D_storage[offset + col_idx].imag;
            offset += ODSDEMO_DOA2D_FFTSIZE;
        }

        DSPF_sp_fftSPxSP (
                ODSDEMO_DOA2D_FFTSIZE,
                fftIn,
                (float *)&gMmwDssMCB.dataPathObj[0].twiddle_DOA[0],
                fftOut,
                brev,
                gMmwDssMCB.dataPathObj[0].DOA2D_rad2D,
                0,
                ODSDEMO_DOA2D_FFTSIZE);

        // FFT output give real first.  Instead of storing the FFT output
        // only record the peak.
        offset = 0;
        for (row_idx = 0; row_idx < ODSDEMO_DOA2D_FFTSIZE; row_idx++)
        {
            mag_sqr = fftOut[offset] * fftOut[offset] +  fftOut[offset+1] * fftOut[offset+1];
            if (mag_sqr > maxVal)
            {
                fft_2D_peak_row_idx = row_idx;
                fft_2D_peak_col_idx = col_idx;
                maxVal = mag_sqr;
            }
            offset += 2;

        }
    }


    /* convert the peak indices b/w [-Fs/2, Fs/2]*/
    if (fft_2D_peak_row_idx > ODSDEMO_DOA2D_HALFFFTSIZE)
    {
        fft_2D_peak_row_idx -= ODSDEMO_DOA2D_FFTSIZE;
    }

    if (fft_2D_peak_col_idx > ODSDEMO_DOA2D_HALFFFTSIZE)
    {
        fft_2D_peak_col_idx -= ODSDEMO_DOA2D_FFTSIZE;
    }

    /* Based on detected peak indices,  compute the azimuth and elevation frequency corresponding to peak value */
    el_freq =  fft_2D_peak_row_idx * ODSDEMO_DOA2D_HALFFFTSIZEINV;
    az_freq =  fft_2D_peak_col_idx * ODSDEMO_DOA2D_HALFFFTSIZEINV;

    /*Compute the elevation angle */
    phi= (float)(asin(el_freq));
    temp = az_freq/((float)(cos(phi)));

    /*Check if azimuth angle can be computed or not */
    if ((temp <= 1.0f) && (temp >= -1.0f))
    {
        theta =  (float)(asin(temp));
    }
    else
    {
        errorCode = ODSDEMO_AOABF2D_FAILTOGIVEOUTPUT;
        return(errorCode);
    }

    /* Compute (x,y,z) cordinates of the detected object */
    estOutput->x = range*(float)(sin(theta))*(float)(cos(phi));
    estOutput->y = range*(float)(cos(theta))*(float)(cos(phi));
    estOutput->z = range*(float)(sin(phi));
    estOutput->aAngleEst = theta;
    estOutput->eAngleEst = phi;

    return(errorCode);
}


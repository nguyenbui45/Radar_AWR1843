
#include <math.h>

/* C674x mathlib */
#include <ti/mathlib/mathlib.h>
#include "odsdemo_dopplerProc_priv.h"

#include "radar_c674x.h"
#include "DSPF_sp_fftSPxSP.h"

#include <ti/sysbios/knl/Event.h>
#include "./common/odsdemo_common.h"
#include "./common/mmw_config.h"
#include <float.h>
#include "dss_mmw.h"
#include "odsdemo_doppler.h"
//#include "swpform.h"

#ifdef _TMS320C6X
#include "c6x.h"
#endif

extern ODSDemo_DataPathObj odsdemo_dataPathObj;
extern MmwDemo_DSS_MCB     gMmwDssMCB;
ODSDEMO_dopplerProc_output  odsdemo_dopplerOut;


#define PI (3.141592654)
//#define DopplerWinSize    4
//const float DopplerWin[DopplerWinSize] =  {0.1624f, 0.4703f, 0.8216f, 0.9976f};

ODSDEMO_detectionCFAR_output odsdemo_cfarOut;

unsigned char brev[64] = {
    0x0, 0x20, 0x10, 0x30, 0x8, 0x28, 0x18, 0x38,
    0x4, 0x24, 0x14, 0x34, 0xc, 0x2c, 0x1c, 0x3c,
    0x2, 0x22, 0x12, 0x32, 0xa, 0x2a, 0x1a, 0x3a,
    0x6, 0x26, 0x16, 0x36, 0xe, 0x2e, 0x1e, 0x3e,
    0x1, 0x21, 0x11, 0x31, 0x9, 0x29, 0x19, 0x39,
    0x5, 0x25, 0x15, 0x35, 0xd, 0x2d, 0x1d, 0x3d,
    0x3, 0x23, 0x13, 0x33, 0xb, 0x2b, 0x1b, 0x3b,
    0x7, 0x27, 0x17, 0x37, 0xf, 0x2f, 0x1f, 0x3f
};

// compute the heatmap first and then search the peak doppler bin 
// then calculate the Doppler Output for DOA module
void ODSDEMO_dopplerProc(uint16_t doppMimoFlag,
                         cplx16_t **dopplerProcInput,
                         ODSDEMO_dopplerProc_output  *dopplerProcOut)
{
    uint32_t    dIdx, antIdx, nAnt, k, fftSize, offset, maxIndex;
    __float2_t rotation;
    __float2_t  tempIn;
    float       * win2DOutPtr, * fftOutPtr, * curentFftOutPtr, *heatmapOutPtr;
    float       maxValue;
	
    nAnt        =   (uint32_t) gMmwDssMCB.dataPathObj[0].numVirtualAntennas;
	fftSize     =   (uint32_t) gMmwDssMCB.dataPathObj[0].numDopplerBins;

    win2DOutPtr =   (float *)&odsdemo_dataPathObj.scratchPad[0];
	heatmapOutPtr   =   (float *)&win2DOutPtr[2 * fftSize];
    fftOutPtr =   (float *)&win2DOutPtr[3 * fftSize];

	for (antIdx = 0; antIdx < nAnt; antIdx++)
	{
    	curentFftOutPtr    =   (float *) &fftOutPtr[antIdx * 2 * fftSize];
		// input format: imaginary first, real second
    	// output format: real first, imaginary second
    	RADARDEMO_dopplerProcWin2DFxdpinFltOut(
				fftSize,
				dopplerProcInput[antIdx],
				gMmwDssMCB.dataPathObj[0].window2D,
				win2DOutPtr);

		// input/output format: real first, imaginary second
		DSPF_sp_fftSPxSP (
				fftSize,
				win2DOutPtr,
				(float *)&gMmwDssMCB.dataPathObj[0].twiddle_2D[0],
				curentFftOutPtr,
				brev,
				gMmwDssMCB.dataPathObj[0].fft2D_rad2D,
				0,
				fftSize);                    
	}
	// generate the heatmap 
	RADARDEMO_dopplerProcIntegrationFltpAllAnt(
			fftSize,
			nAnt,
			fftOutPtr,
			heatmapOutPtr);
			
	// find the peak in the heatmap
	maxIndex = 0;
	maxValue = heatmapOutPtr[0];
	for (dIdx = 1; dIdx < fftSize; dIdx++)
	{
		if (maxValue < heatmapOutPtr[dIdx])
		{
		    maxValue = heatmapOutPtr[dIdx];
			maxIndex = dIdx;
		}
	}
    k = maxIndex;
    if (maxIndex >= (int32_t)(fftSize >> 1))
        k  =   maxIndex - fftSize;
    dopplerProcOut->dopplerPeakIndex = k;

    // generate the rotation
    if (doppMimoFlag == 1)
    {
        float re, im, phi;
        phi         =   -PI*(float)(k)/(float)(fftSize);
        re          =   (float) cos(phi);
        im          =   (float) sin(phi);
        rotation    =   _ftof2(re, im);
    }						

    offset = 0;
	// Apply rotation, output format: imaginary first, real second
	for (antIdx = 0; antIdx < nAnt; antIdx++)
	{
		if ((doppMimoFlag == 1) && antIdx >= (nAnt >> 1))
		{
			tempIn  =  _ftof2(fftOutPtr[offset + 2 * maxIndex], fftOutPtr[offset + 2 * maxIndex + 1]);
			tempIn  =  _complex_mpysp(rotation, tempIn);
			_amem8_f2(&dopplerProcOut->outBuffer[2 * antIdx])     =   tempIn;
		}
		else
		{
			_amem8_f2(&dopplerProcOut->outBuffer[2 * antIdx]) = _ftof2(fftOutPtr[offset + 2 * maxIndex], fftOutPtr[offset + 2 * maxIndex + 1]);
		}
		offset += (fftSize << 1);
	}					

}


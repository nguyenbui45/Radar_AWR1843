
#include <math.h>

/* C674x mathlib */
#include <ti/mathlib/mathlib.h>

#include "radar_c674x.h"

#include <ti/sysbios/knl/Event.h>
#include "./common/odsdemo_common.h"
#include "./common/mmw_config.h"
#include <float.h>
#include "dss_mmw.h"
#ifdef _TMS320C6X
#include "c6x.h"
#endif
#include "odsdemo_cfar.h"

extern ODSDemo_DataPathObj odsdemo_dataPathObj;
extern MmwDemo_DSS_MCB     gMmwDssMCB;

#pragma DATA_SECTION(cfar_window, ".l1data");
#pragma DATA_ALIGN(cfar_window, 16);
float   cfar_window[CFAR_WIN_SIZE];

ODSDEMO_detectionCFAR_output odsdemo_cfarOut;


/* Create a 1D array of the heatmap elements needed for CFAR on a single range bin.
 * The center point of this in the heatmap is:
 *   Row = range_bin
 *   Col = peak_idx
 *
 * So, if the extension size is 10, then after range_bin = 10, left extension is
 * no longer needed. Right extension is only needed when:
 *    range_bin > (num_range_bins - (extension_size + 1))
 */
void create_extended_heatmap_row(uint16_t  range_bin, //current range bin to process
                                 uint16_t  peak_idx,  //index of max on the range-row
                                 uint16_t  map_width, //range-row size of heatmap
                                 float    *heatmap,   //pointer to first cell of heatmap
                                 MmwDemo_CfarCfg *cfarCfg)
{
  int16_t  idx, curr_idx;
  int16_t  num_ext;
  float   *col_ptr;
  uint16_t cfarWinSize = ((cfarCfg->expensionSize << 1) + 1);

  col_ptr  = &heatmap[peak_idx]; //locate the peak's column on row 0
  curr_idx = 0;

  // Do left extension, if any
  if (range_bin < cfarCfg->expensionSize)
  {
    num_ext = cfarCfg->expensionSize - range_bin;

    for (idx = 0; idx < num_ext; idx ++)
      cfar_window[curr_idx++] = *col_ptr;

    for (idx = 0; idx < (cfarWinSize - num_ext); idx ++)
    {
      cfar_window[curr_idx++] = *col_ptr;
      col_ptr += map_width;
    }
  }

  // Do right extension, if any
  else
  if (range_bin > (cfarCfg->maxRangeBinForDetect - (cfarCfg->expensionSize + 1)))
  {
    num_ext  =  range_bin - (cfarCfg->maxRangeBinForDetect - (cfarCfg->expensionSize + 1));
    col_ptr += (range_bin - cfarCfg->expensionSize) * map_width;

    for (idx = (range_bin - cfarCfg->expensionSize); curr_idx < (cfarWinSize-num_ext); idx ++)
    {
      cfar_window[curr_idx++] = *col_ptr;
      col_ptr += map_width;
    }

    col_ptr -= map_width;

    for (idx = 0; idx < num_ext; idx ++)
      cfar_window[curr_idx++] = *col_ptr;
  }

  else //no extension
  {
    col_ptr += (range_bin - cfarCfg->expensionSize) * map_width;

    for (idx = (range_bin - cfarCfg->expensionSize); curr_idx < cfarWinSize; idx ++)
    {
      cfar_window[curr_idx++] = *col_ptr;
      col_ptr += map_width;
    }
  }
}


/*!
   \fn     RADARDEMO_detectionCFAR_run

   \brief   Range processing, always called per chirp per antenna.

   \param[in]    handle
               Module handle.

   \param[in]    detectionCFARInput
               Input signal, with dimension [fft2DSize][fft1DSize]. Must be aligned to 8-byte boundary.

   \param[out]    estOutput
               Estimation output from CFAR module.
 */

ODSDemo_cfarErrorCodes ODSDemo_detectionCFAR(float    *aHeatmap,
                              float    *eHeatmap,
                              float    *zeroDoppler,
                              uint16_t *aHeatmapMaxIndex,
                              uint16_t *eHeatmapMaxIndex,
                              ODSDEMO_detectionCFAR_output *cfarOut)

{
    uint16_t idx;
    float   *aSigPwr,   *eSigPwr;
    float   *aNoisePwr, *eNoisePwr;
    bool    *aPeakFlag, *ePeakFlag;
    bool     flag1, flag2, flag;
    float    snr1, snr2, K0;
    MmwDemo_CfarCfg *cfarCfg = &gMmwDssMCB.cliCfg[0].cfarCfg;
    float    rangeRes = gMmwDssMCB.dataPathObj[0].rangeResolution;

    /* Create temporary storage for 1D outputs */
    aSigPwr   = (float *)&odsdemo_dataPathObj.scratchPad[0];
    eSigPwr   = (float *)&odsdemo_dataPathObj.scratchPad[1*cfarCfg->maxRangeBinForDetect];
    aNoisePwr = (float *)&odsdemo_dataPathObj.scratchPad[2*cfarCfg->maxRangeBinForDetect];
    eNoisePwr = (float *)&odsdemo_dataPathObj.scratchPad[3*cfarCfg->maxRangeBinForDetect];
    aPeakFlag = (bool  *)&odsdemo_dataPathObj.scratchPad[4*cfarCfg->maxRangeBinForDetect];
    ePeakFlag = (bool  *)&odsdemo_dataPathObj.scratchPad[5*cfarCfg->maxRangeBinForDetect];

    cfarOut->numObjDetected = 0;
    ODSDemo_detectionCFAR_cfar1D(aHeatmap,
                                 ODSDEMO_AZIMUTH_BINS,
                                 aHeatmapMaxIndex,
                                 zeroDoppler,
                                 cfarCfg,
                                 aSigPwr,
                                 aNoisePwr,
                                 aPeakFlag);

    ODSDemo_detectionCFAR_cfar1D(eHeatmap,
                                 ODSDEMO_ELEVATION_BINS,
                                 eHeatmapMaxIndex,
                                 zeroDoppler,
                                 cfarCfg,
                                 eSigPwr,
                                 eNoisePwr,
                                 ePeakFlag);

    K0 = cfarCfg->rangeThresh;
    for (idx = cfarCfg->leftSkipBin; idx < cfarCfg->maxRangeBinForDetect; idx++)
    {
        flag1 = ((aSigPwr[idx] > (K0 * aNoisePwr[idx])) && (aPeakFlag[idx]));
        flag2 = ((eSigPwr[idx] > (K0 * eNoisePwr[idx])) && (ePeakFlag[idx]));

        if (cfarCfg->detectMethod == ODSDEMO_DETECTIONCFAR_UNION)
            flag = flag1 || flag2;
        else if (cfarCfg->detectMethod == ODSDEMO_DETECTIONCFAR_INTERSECTION)
            flag = flag1 && flag2;

        if (flag)
        {
            if (cfarOut->numObjDetected > ODSDEMO_MAX_OBJECTS)
              return CFAR_ERROR_MAX_DETECTION_EXCEEDED;
            snr1 = aSigPwr[idx] / aNoisePwr[idx];
            snr2 = eSigPwr[idx] / eNoisePwr[idx];
            cfarOut->snrEst[cfarOut->numObjDetected] = (snr1 > snr2? snr1: snr2);
            cfarOut->noise[cfarOut->numObjDetected]  = (snr1 > snr2? aNoisePwr[idx]: eNoisePwr[idx]);
            cfarOut->rangeInd[cfarOut->numObjDetected] = idx;
            cfarOut->rangeEst[cfarOut->numObjDetected] = idx * rangeRes;
            cfarOut->aInd[cfarOut->numObjDetected] = aHeatmapMaxIndex[idx];
            cfarOut->eInd[cfarOut->numObjDetected] = eHeatmapMaxIndex[idx];
            cfarOut->numObjDetected ++;
        }
    }
    return CFAR_OK;
}


void ODSDemo_detectionCFAR_cfar1D(float    *heatmap,
                                  uint16_t  heapmap_width,
                                  uint16_t *heatmapMaxIndex,
                                  float    *zeroDoppler,
                                  MmwDemo_CfarCfg *cfarCfg,
                                  float    *sigPwr,
                                  float    *noisePwr,
                                  bool     *peakFlag)
{
    uint16_t idx, idx2, maxIdx, currSigIdxInHeatmap;
    int16_t  peakAngleDiff;
    float    noiseLeft, noiseRight, temp, maxZeroDopplerNeighbor;

    for (idx = cfarCfg->leftSkipBin; idx < cfarCfg->maxRangeBinForDetect; idx++)
    {
        maxIdx = heatmapMaxIndex[idx];
        create_extended_heatmap_row(idx, maxIdx, heapmap_width, heatmap, cfarCfg);

        sigPwr[idx] = cfar_window[cfarCfg->expensionSize];
        noiseLeft  = 0;
        noiseRight = 0;
        for (idx2 = 0; idx2 < cfarCfg->searchWinSizeRange; idx2++)
        {
          noiseLeft  += cfar_window[cfarCfg->expensionSize - cfarCfg->guardSizeRange - 1 - idx2];
          noiseRight += cfar_window[cfarCfg->expensionSize + cfarCfg->guardSizeRange + 1 + idx2];
        }

        maxZeroDopplerNeighbor = 0;
        currSigIdxInHeatmap = idx + cfarCfg->expensionSize;

        for (idx2 = 0; idx2 < cfarCfg->searchWinSizeSpreading; idx2++)
        {
            temp = zeroDoppler[currSigIdxInHeatmap - cfarCfg->guardSizeSpreading - 1 - idx2];
            maxZeroDopplerNeighbor = (temp > maxZeroDopplerNeighbor? temp:maxZeroDopplerNeighbor);

            temp = zeroDoppler[currSigIdxInHeatmap + cfarCfg->guardSizeSpreading + 1 + idx2];
            maxZeroDopplerNeighbor = (temp > maxZeroDopplerNeighbor? temp:maxZeroDopplerNeighbor);
        }

        // noise Power is the total accumulated noise power within the neighboring CFAR window
        switch (cfarCfg->noiseCalcType)
        {
            case ODSDEMO_DETECTIONCFAR_NOISECALC_MEAN:
                noisePwr[idx] = noiseLeft + noiseRight;

            case ODSDEMO_DETECTIONCFAR_NOISECALC_LEFTORMEAN:
                noisePwr[idx] = (noiseRight > noiseLeft ? (noiseLeft * 2.0f): (noiseLeft + noiseRight));

            case ODSDEMO_DETECTIONCFAR_NOISECALC_MIN:
                noisePwr[idx] = (noiseRight > noiseLeft ? (noiseLeft * 2.0f): (noiseRight * 2.0f));
        }

        peakFlag[idx] = ((cfar_window[cfarCfg->expensionSize] > cfar_window[cfarCfg->expensionSize - 1]) &&
                         (cfar_window[cfarCfg->expensionSize] > cfar_window[cfarCfg->expensionSize + 1]));

        if ((idx < cfarCfg->closeInRangeBin) && (peakFlag[idx]))
        {
            peakFlag[idx] = 0;
            peakAngleDiff = (heatmapMaxIndex[idx] - heatmapMaxIndex[idx-2]);
            peakAngleDiff = (peakAngleDiff > 0 ? peakAngleDiff: -1 * peakAngleDiff);
            // also check for idx-2
            if (cfar_window[cfarCfg->expensionSize] > cfar_window[cfarCfg->expensionSize - 2])
                peakFlag[idx] = 1;
            else if (peakAngleDiff > cfarCfg->peakAngleDiffThresh)
                peakFlag[idx] = 1;
            // check for fft spreading effect
            temp = maxZeroDopplerNeighbor * cfarCfg->fftSpreadingThresh;
            if (zeroDoppler[currSigIdxInHeatmap] <= temp)
               peakFlag[idx] = 0;
        }
        if ((idx >= cfarCfg->closeInRangeBin) && (cfarCfg->localPeakEnable == 0))
            peakFlag[idx] = 1;

        // scale sigPwr to be ready for SNR calculation
        sigPwr[idx] = sigPwr[idx] * (cfarCfg->searchWinSizeRange << 1);
    }
}

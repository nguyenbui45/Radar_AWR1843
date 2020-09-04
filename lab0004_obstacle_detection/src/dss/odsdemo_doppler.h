/**
 *  \file   odsdemo_doppler.h
 *
 *  Copyright (c) Texas Instruments Incorporated 2018
 *
 *  Use of this software is controlled by the terms and conditions found in the
 *  license agreement under which this software has been supplied or provided.
 *
 */
#ifndef ODSDEMO_DOPPLER
#define ODSDEMO_DOPPLER

//#include "./common/odsdemo_common.h"
#include <ti/alg/mmwavelib/mmwavelib.h>

typedef struct {
    uint16_t dopplerPeakIndex;
    float *outBuffer;
}ODSDEMO_dopplerProc_output;

void ODSDEMO_dopplerProc(uint16_t doppMimoFlag,
                         cplx16_t **dopplerProcInput,
                         ODSDEMO_dopplerProc_output  *dopplerProcOut);

#endif //ODSDEMO_DOPPLER

/*! 
 *  \file   RADARDEMO_detectionCFAR.c
 *
 *  \brief   CFAR detection. 
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


#include <modules/detection/CFAR/api/RADARDEMO_detectionCFAR.h>
#include "RADARDEMO_detectionCFAR_priv.h"
#include <math.h>
#include <stdio.h>

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif


/*! 
   \fn     RADARDEMO_detectionCFAR_create
 
   \brief   Create and initialize RADARDEMO_detectionCFAR module. 
  
   \param[in]    moduleConfig
               Pointer to input configurations structure for RADARDEMO_detectionCFAR module.
			   
   \param[in]    errorCode
               Output error code.
			   
   \ret     void pointer to the module handle. Return value of NULL indicates failed module creation.
			   
   \pre       none
 
   \post      none
  
 
 */

void	* RADARDEMO_detectionCFAR_create(
                            IN  RADARDEMO_detectionCFAR_config * moduleConfig,
							OUT RADARDEMO_detectionCFAR_errorCode * errorCode)
							
{
	RADARDEMO_detectionCFAR_handle * handle;

	*errorCode	=	RADARDEMO_DETECTIONCFAR_NO_ERROR;
	
	/* Check error configurations */
	/* unsupported CFAR type */
	if (moduleConfig->cfarType >= RADARDEMO_DETECTIONCFAR_NOT_SUPPORTED)
		*errorCode	=	RADARDEMO_DETECTIONCFAR_CFARTYPE_NOTSUPPORTED;
		
	/* unsupported input type */
	if (moduleConfig->inputType >= RADARDEMO_DETECTIONCFAR_INPUTTYPE_NOT_SUPPORTED)
		*errorCode = RADARDEMO_DETECTIONCFAR_CFARINPUTTYPE_NOTSUPPORTED;
	
	/* Unsupported CFAR-OS window size */
	if ((moduleConfig->cfarType == RADARDEMO_DETECTIONCFAR_CFAROS) && (moduleConfig->searchWinSizeRange != 16))
		*errorCode = RADARDEMO_DETECTIONCFAR_CFAROSWINSIZE_NOTSUPPORTED;
	
	/* incorrect CFAR-CASO window setting */
	if ((moduleConfig->cfarType == RADARDEMO_DETECTIONCFAR_CASOCFAR) && (moduleConfig->searchWinSizeDoppler == 0))
		*errorCode = RADARDEMO_DETECTIONCFAR_CFARCASOWINSIZE_NOTSUPPORTED;

	if (*errorCode > RADARDEMO_DETECTIONCFAR_NO_ERROR)
		return (NULL);
		
	handle						=	(RADARDEMO_detectionCFAR_handle *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_detectionCFAR_handle), 1);
	if (handle == NULL)
	{
		*errorCode =  RADARDEMO_DETECTIONCFAR_FAIL_ALLOCATE_HANDLE;
		return (handle);
	}
	
	handle->fft1DSize			=	moduleConfig->fft1DSize;
	handle->fft2DSize			=	moduleConfig->fft2DSize;
	handle->cfarType			=	(uint8_t) moduleConfig->cfarType;
	handle->rangeRes			=	moduleConfig->rangeRes;
	handle->dopplerRes			=	moduleConfig->dopplerRes;
	handle->searchWinSizeRange	=	moduleConfig->searchWinSizeRange;
	handle->guardSizeRange		=	moduleConfig->guardSizeRange;
	handle->searchWinSizeDoppler=	moduleConfig->searchWinSizeDoppler;
	handle->guardSizeDoppler	=	moduleConfig->guardSizeDoppler;
	handle->maxNumDetObj		=	moduleConfig->maxNumDetObj;
	handle->leftSkipSize		=	moduleConfig->leftSkipSize;
	handle->rightSkipSize		=	moduleConfig->rightSkipSize;
	handle->enableSecondPassSearch		=	moduleConfig->enableSecondPassSearch;
	handle->dopplerSearchRelThr		=	moduleConfig->dopplerSearchRelThr;
	handle->log2MagFlag			=	moduleConfig->log2MagFlag;
	

	/* parse CA-CFAR types */
	if (moduleConfig->cfarType == RADARDEMO_DETECTIONCFAR_CAVGCFAR)
		handle->caCfarType	=	RADARDEMO_DETECTIONCFAR_CFAR_CAVG;
	if (moduleConfig->cfarType == RADARDEMO_DETECTIONCFAR_CASOCFAR)
		handle->caCfarType	=	RADARDEMO_DETECTIONCFAR_CFAR_CASO;
	if (moduleConfig->cfarType == RADARDEMO_DETECTIONCFAR_CACCCFAR)
		handle->caCfarType	=	RADARDEMO_DETECTIONCFAR_CFAR_CACC;
	if (moduleConfig->cfarType == RADARDEMO_DETECTIONCFAR_CAGOCFAR)
		handle->caCfarType	=	RADARDEMO_DETECTIONCFAR_CFAR_CAGO;

	if (moduleConfig->cfarType == RADARDEMO_DETECTIONCFAR_RA_CASOCFAR)
	{
		handle->caCfarType	=	RADARDEMO_DETECTIONCFAR_RA_CFAR_CASO;
		handle->leftSkipSizeAzimuth		=	moduleConfig->leftSkipSizeAzimuth;
		handle->rightSkipSizeAzimuth		=	moduleConfig->rightSkipSizeAzimuth;
	}

	handle->relThr		=	moduleConfig->K0;
#ifdef USE_TABLE_FOR_K0
	if (moduleConfig->K0 == 0.f)
	{
		int32_t i, j, k;;
		if (moduleConfig->cfarType == RADARDEMO_DETECTIONCFAR_CAVGCFAR)
		{
			j 				= 	(int32_t) floor(9.5f + log10(moduleConfig->pfa)) - 1;
			if (j < 0) j	=	0;
			i 				= 	(handle->searchWinSizeRange >> 2) - 1;
			handle->relThr 	= 	rltvThr_CFARCA[i * 7 + j]; 
		}
		else if (moduleConfig->cfarType == RADARDEMO_DETECTIONCFAR_CFAROS)
		{
			j 				= 	(int32_t) floor(7.5f + log10(moduleConfig->pfa)) - 1;
			i 				= 	(handle->searchWinSizeRange >> 2) - 1;
			k 				=	((3 * (2 * handle->searchWinSizeRange)) >> 2) - 2 ;
			handle->relThr 	= 	rltvThr_CFAROS[i][k * 5 + j]; 
		}
	}
#endif

	handle->scratchPad 	=	(int32_t *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 1, handle->fft1DSize*(sizeof(float) +sizeof(int16_t)) + 100*sizeof(float), 1);
	if (handle->scratchPad == NULL)
	{
		*errorCode =  RADARDEMO_DETECTIONCFAR_FAIL_ALLOCATE_LOCALINSTMEM;
	}
	return((void *)handle);
}

/*! 
   \fn     RADARDEMO_detectionCFAR_delete
 
   \brief   Delete RADARDEMO_detectionCFAR module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
  
 
 */

void	RADARDEMO_detectionCFAR_delete(
                            IN  void * handle)
{
	RADARDEMO_detectionCFAR_handle *detectionCFARInst;
	
	detectionCFARInst	=	(RADARDEMO_detectionCFAR_handle *) handle;

	radarOsal_memFree(detectionCFARInst->scratchPad, detectionCFARInst->fft1DSize*(sizeof(float) +sizeof(int16_t)) + 100*sizeof(float));
	radarOsal_memFree(detectionCFARInst, sizeof(RADARDEMO_detectionCFAR_handle));
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

   \ret error code
 
   \pre       none
 
   \post      none
  
 
 */

RADARDEMO_detectionCFAR_errorCode	RADARDEMO_detectionCFAR_run(
                            IN  void * handle,
							IN  float ** detectionCFARInput,
							OUT RADARDEMO_detectionCFAR_output   * estOutput)

{
	RADARDEMO_detectionCFAR_handle *detectionCFARInst;
	RADARDEMO_detectionCFAR_errorCode	errorCode = RADARDEMO_DETECTIONCFAR_NO_ERROR;

	detectionCFARInst	=	(RADARDEMO_detectionCFAR_handle *) handle;

	if( detectionCFARInput == NULL)
		errorCode	=	RADARDEMO_DETECTIONCFAR_INOUTPTR_NOTCORRECT;
	if( estOutput == NULL)
		errorCode	=	RADARDEMO_DETECTIONCFAR_INOUTPTR_NOTCORRECT;
	if( estOutput->rangeInd == NULL)
		errorCode	=	RADARDEMO_DETECTIONCFAR_INOUTPTR_NOTCORRECT;
	if( estOutput->dopplerInd == NULL)
		errorCode	=	RADARDEMO_DETECTIONCFAR_INOUTPTR_NOTCORRECT;
	if ((RADARDEMO_detectionCFAR_Type)detectionCFARInst->cfarType != RADARDEMO_DETECTIONCFAR_RA_CASOCFAR)
	{
		if( estOutput->rangeEst == NULL)
			errorCode	=	RADARDEMO_DETECTIONCFAR_INOUTPTR_NOTCORRECT;
		if( estOutput->dopplerEst == NULL)
			errorCode	=	RADARDEMO_DETECTIONCFAR_INOUTPTR_NOTCORRECT;
	}
	if( estOutput->snrEst == NULL)
		errorCode	=	RADARDEMO_DETECTIONCFAR_INOUTPTR_NOTCORRECT;
	if( estOutput->noise == NULL)
		errorCode	=	RADARDEMO_DETECTIONCFAR_INOUTPTR_NOTCORRECT;
	if (errorCode > RADARDEMO_DETECTIONCFAR_NO_ERROR)
		return(errorCode);
	
	if ((RADARDEMO_detectionCFAR_Type)detectionCFARInst->cfarType == RADARDEMO_DETECTIONCFAR_CFAROS)
	{
		estOutput->numObjDetected = RADARDEMO_detectionCFAR_OS(
							detectionCFARInput,
                            detectionCFARInst,
							estOutput->rangeInd, 
							estOutput->dopplerInd,
							estOutput->rangeEst,
							estOutput->dopplerEst,
							estOutput->snrEst,
							estOutput->noise);
	}	
	else if (((RADARDEMO_detectionCFAR_Type)detectionCFARInst->cfarType == RADARDEMO_DETECTIONCFAR_CASOCFAR)
		|| ((RADARDEMO_detectionCFAR_Type)detectionCFARInst->cfarType == RADARDEMO_DETECTIONCFAR_CACCCFAR)
		|| ((RADARDEMO_detectionCFAR_Type)detectionCFARInst->cfarType == RADARDEMO_DETECTIONCFAR_CAGOCFAR)
		|| ((RADARDEMO_detectionCFAR_Type)detectionCFARInst->cfarType == RADARDEMO_DETECTIONCFAR_CAVGCFAR)
		)
	{
		estOutput->numObjDetected = RADARDEMO_detectionCFAR_CAAll(
							detectionCFARInput,
                            detectionCFARInst,
							estOutput->rangeInd, 
							estOutput->dopplerInd,
							estOutput->rangeEst,
							estOutput->dopplerEst,
							estOutput->rangeVar,
							estOutput->dopplerVar,
							estOutput->snrEst,
							estOutput->noise);
	}	
	else if ((RADARDEMO_detectionCFAR_Type)detectionCFARInst->cfarType == RADARDEMO_DETECTIONCFAR_RA_CASOCFAR)
	{
		// reuse doppler index output to store azimuth index for now.
		estOutput->numObjDetected = RADARDEMO_detectionCFAR_raCAAll(
							detectionCFARInput,
                            detectionCFARInst,
							estOutput->rangeInd, 
							estOutput->dopplerInd,
							estOutput->snrEst,
							estOutput->noise);
	}
	return(errorCode);
}


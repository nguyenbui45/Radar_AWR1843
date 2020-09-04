/*! 
 *  \file   RADARDEMO_detectionCFAR.h
 *
 *  \brief   Header file for RADARDEMO_detectionCFAR module
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


#ifndef RADARDEMO_DETECTIONCFAR_H
#define RADARDEMO_DETECTIONCFAR_H

#include <swpform.h>
#include <modules/utilities/radarOsal_malloc.h>


/**
 *  \enum   
 *   {
 *	RADARDEMO_DETECTIONCFAR_NO_ERROR = 0,				
 *  RADARDEMO_DETECTIONCFAR_CFARTYPE_NOTSUPPORTED,		
 *	RADARDEMO_DETECTIONCFAR_CFARINPUTTYPE_NOTSUPPORTED,
 *  RADARDEMO_DETECTIONCFAR_CFAROSWINSIZE_NOTSUPPORTED,
 *  RADARDEMO_DETECTIONCFAR_FAIL_ALLOCATE_HANDLE,		
 *  RADARDEMO_DETECTIONCFAR_FAIL_ALLOCATE_LOCALINSTMEM,
 *  RADARDEMO_DETECTIONCFAR_INOUTPTR_NOTCORRECT,	   
 *   }   RADARDEMO_detectionCFAR_errorCode;
 *
 *  \brief   Error code for CFAR detection module.
 *
 *
 */

typedef enum
{
	RADARDEMO_DETECTIONCFAR_NO_ERROR = 0,				/**< no error */
	RADARDEMO_DETECTIONCFAR_CFARTYPE_NOTSUPPORTED,		/**< CFART non supported type */
	RADARDEMO_DETECTIONCFAR_CFARINPUTTYPE_NOTSUPPORTED,	/**< CFART input type not supported */
	RADARDEMO_DETECTIONCFAR_CFAROSWINSIZE_NOTSUPPORTED, /**< CFAR-OS search window size not supported */ 
	RADARDEMO_DETECTIONCFAR_CFARCASOWINSIZE_NOTSUPPORTED, /**< CFAR-CASO search window size not supported */ 
	RADARDEMO_DETECTIONCFAR_FAIL_ALLOCATE_HANDLE,		/**< RADARDEMO_detectionCFAR_create failed to allocate handle */ 
	RADARDEMO_DETECTIONCFAR_FAIL_ALLOCATE_LOCALINSTMEM,	/**< RADARDEMO_detectionCFAR_create failed to allocate memory for buffers in local instance  */ 
	RADARDEMO_DETECTIONCFAR_INOUTPTR_NOTCORRECT,	    /**< input and/or output buffer for RADARDEMO_detectionCFAR_run are either NULL, or not aligned properly  */
	RADARDEMO_DETECTIONCFAR_CORRUPTED_2DSIZE	    /**< input and/or output buffer for RADARDEMO_detectionCFAR_run are either NULL, or not aligned properly  */
} RADARDEMO_detectionCFAR_errorCode;


/**
 *  \enum   
 *   {
 *	RADARDEMO_DETECTIONCFAR_CAVGCFAR = 0,	
 *  RADARDEMO_DETECTIONCFAR_CFAROS,
 *  RADARDEMO_DETECTIONCFAR_CASOCFAR,
 *  RADARDEMO_DETECTIONCFAR_CACCCFAR,
 *  RADARDEMO_DETECTIONCFAR_CAGOCFAR,
 *	RADARDEMO_DETECTIONCFAR_NOT_SUPPORTED
 *   }   RADARDEMO_detectionCFAR_Type;
 *
 *  \brief   enum for CFAR types.
 *
 *
 */

typedef enum
{
	RADARDEMO_DETECTIONCFAR_CFAROS = 0,					/**< CFAR type: ordered statistics*/
	RADARDEMO_DETECTIONCFAR_CAVGCFAR,					/**< CFAR type: cell average CFAR*/
	RADARDEMO_DETECTIONCFAR_CASOCFAR,					/**< CFAR type: cell average CFAR, smaller of the 2 windows*/
	RADARDEMO_DETECTIONCFAR_CACCCFAR,					/**< CFAR type: cell accumulation CFAR*/
	RADARDEMO_DETECTIONCFAR_CAGOCFAR,					/**< CFAR type: cell average CFAR, greater of the 2 windows*/
	RADARDEMO_DETECTIONCFAR_RA_CASOCFAR,				/**< CFAR type: cell average CFAR, smaller of the 2 windows for both 2 passes, and for range-azimuth*/
	RADARDEMO_DETECTIONCFAR_NOT_SUPPORTED
} RADARDEMO_detectionCFAR_Type;

/**
 *  \enum   
 *   {
 *	RADARDEMO_DETECTIONCFAR_INPUTTYPE_SP = 0,	
 *	RADARDEMO_DETECTIONCFAR_INPUTTYPE_NOT_SUPPORTED
 *   }   RADARDEMO_detectionCFAR_inputType;
 *
 *  \brief   enum for CFAR input type.
 *
 *
 */

typedef enum
{
	RADARDEMO_DETECTIONCFAR_INPUTTYPE_SP = 0,			/**< input type: single precision floating point*/
	RADARDEMO_DETECTIONCFAR_INPUTTYPE_NOT_SUPPORTED
} RADARDEMO_detectionCFAR_inputType;


/**
 *  \struct   _RADARDEMO_detectionCFAR_config_
 *   {
 *   	uint32_t     fft1DSize;  					
 *		uint32_t     fft2DSize;  					
 *		RADARDEMO_detectionCFAR_Type cfarType;		
 *		RADARDEMO_detectionCFAR_inputType inputType;
 *      float        pfa;  	    					
 *      float        K0;  	    					
 *      float        rangeRes;  	    			
 *      float        dopplerRes;  
 *      uint8_t      enableSecondPassSearch;
 *      uint8_t      searchWinSizeRange;	    			
 *      uint8_t      guardSizeRange;	    			   
 *      uint8_t      searchWinSizeDoppler;	    			
 *      uint8_t      guardSizeDoppler;	    			   
 *      uint8_t      leftSkipSize;	    			
 *      uint8_t      rightSkipSize;	    			   
 *   }   RADARDEMO_detectionCFAR_config;
 *
 *  \brief   Structure element of the list of descriptors for RADARDEMO_detectionCFAR configuration.
 *
 *
 */

typedef struct _RADARDEMO_detectionCFAR_config_
{
	uint32_t     fft1DSize;  					/**< 1D FFT size*/
	uint32_t     fft2DSize;  					/**< 2D FFT size*/
	RADARDEMO_detectionCFAR_Type cfarType;		/**< Type of CFAR.*/
	RADARDEMO_detectionCFAR_inputType inputType;  /**< Type of integration.*/
	float        pfa;  	    					/**< Desired false detection ratio.*/
	float        K0;  	    					/**< Relative detection threshold. If K0 is non-zero value, pfa setting will be ignored.*/
	float        rangeRes;  	    			/**< Range resolution.*/
	float        dopplerRes;  	    			/**< Doppler resolution.*/
	float        dopplerSearchRelThr;  	        /**< Doppler search relative threshold.*/
	uint8_t      enableSecondPassSearch;    	/**< Flag for enabling second pass search, if set to 1. If set to 0, no second pass search*/
	uint8_t      searchWinSizeRange;	    	/**< Search window size for range domain search.*/
	uint8_t      guardSizeRange;	    		/**< Number of guard samples for range domain search.*/
	uint8_t      searchWinSizeDoppler;	    	/**< Search window size for Doppler domain search.*/
	uint8_t      guardSizeDoppler;	    		/**< Number of guard samples for Doppler domain search.*/
	uint8_t      maxNumDetObj;	    			/**< maximum number of detected obj.*/
	uint8_t      leftSkipSize;                  /**< number of samples to be skipped on the left side in range domain. */
	uint8_t      rightSkipSize;                 /**< number of samples to be skipped on the right side in range domain. */
	uint8_t      leftSkipSizeAzimuth;            /**< number of samples to be skipped on the left side in azimuth domain. */
	uint8_t      rightSkipSizeAzimuth;           /**< number of samples to be skipped on the right side in azimuth domain. */
	uint32_t    log2MagFlag; 					/**<use log2(mag) as input*/
	uint32_t    shortened1DInput; 				/**<Flag if set to 1, to indicate that the heatmap is already trimmed by skip left sample at 1D dimmension*/
} RADARDEMO_detectionCFAR_config;

/**
 *  \struct   _RADARDEMO_detectionCFAR_output_
 *   {
 *   	uint16_t     numObjDetected;  	
 *   	uint16_t     *rangeInd;  	
 *		uint16_t     *dopplerInd; 
  *     float        *rangeEst;  	
 *		float        *dopplerEst; 
 *		float        *sntEst;  	
 *      float        *noise;  	
 *		float        *rangeVar;  	
 *      float        *dopplerVar;  	
 *   }   RADARDEMO_detectionCFAR_output;
 *
 *  \brief   Structure element of the list of descriptors for RADARDEMO_detectionCFAR output.
 *
 *
 */

typedef struct _RADARDEMO_detectionCFAR_output_
{
	uint16_t     numObjDetected;		/**< number of objectes detected*/
	uint16_t     *rangeInd;  			/**< range index pointer*/
	uint16_t     *dopplerInd;  			/**< Doppler index pointer*/
	float        *rangeEst;  			/**< range estimation pointer*/
	float        *dopplerEst;  			/**< Doppler estimation pointer*/
	float        *snrEst;  				/**< linear snr estimation pointer*/
	float        *noise;  				/**< Total noise estimation*/
	float        *rangeVar;  			/**< Variance for range estimation*/
	float        *dopplerVar;  			/**< Variance for Doppler estimation*/
} RADARDEMO_detectionCFAR_output;


/*! 
   \fn     RADARDEMO_detectionCFAR_create
 
   \brief   Create and initialize RADARDEMO_detectionCFAR module. 
  
   \param[in]    moduleConfig
               Pointer to input configurations structure for RADARDEMO_detectionCFAR module.
			   
   \param[out]    errorCode
               Output error code.
			   
   \ret     void pointer to the module handle. Return value of NULL indicates failed module creation.
			   
   \pre       none
 
   \post      none
  
 
 */
extern void	* RADARDEMO_detectionCFAR_create(
                            IN  RADARDEMO_detectionCFAR_config * moduleConfig,
							OUT RADARDEMO_detectionCFAR_errorCode * errorCode);

/*! 
   \fn     RADARDEMO_detectionCFAR_delete
 
   \brief   Delete RADARDEMO_detectionCFAR module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
  
 
 */

extern void	RADARDEMO_detectionCFAR_delete(
                            IN  void * handle);


/*! 
   \fn     RADARDEMO_detectionCFAR_run
 
   \brief   Range processing, always called per chirp per antenna.
  
   \param[in]    handle
               Module handle.
 
   \param[in]    detectionCFARInput
               Input signal (from range processing). If handle->fxdpInputFlag = 1, input is cplx16_t type with size [handle->nRxAnt][handle->fft2DSize].
			   If If handle->fxdpInputFlag = 0, input is float type with size [handle->nRxAnt][2 * handle->fft2DSize] and stored in real imag order in memory..
 
   \param[out]    outputSignal
               Output signal from doppler processing. 
 
   \ret error code
 
   \pre       none
 
   \post      none
  
 
 */

extern RADARDEMO_detectionCFAR_errorCode	RADARDEMO_detectionCFAR_run(
                            IN  void * handle,
							IN  float ** cfarInput,
							OUT RADARDEMO_detectionCFAR_output  * cfarOutput);
#endif //RADARDEMO_DETECTIONCFAR_H


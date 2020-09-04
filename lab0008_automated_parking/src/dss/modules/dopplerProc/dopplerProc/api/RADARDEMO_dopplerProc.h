/*! 
 *  \file   RADARDEMO_dopplerProc.h
 *
 *  \brief   Header file for RADARDEMO_rangeProc module
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

#ifndef RADARDEMO_DOPPLERPROC_H
#define RADARDEMO_DOPPLERPROC_H

#include <swpform.h>
#include <modules/utilities/radarOsal_malloc.h>

/**
 *  \enum   
 *   {
 *	RADARDEMO_DOPPLERPROC_NO_ERROR = 0,					
 *	RADARDEMO_DOPPLERPROC_2DFFTTYPE_NOTSUPPORTED,		
 *  RADARDEMO_DOPPLERPROC_2DFFTSIZE_NOTVALID,			
 *  RADARDEMO_DOPPLERPROC_INTEGRTYPE_NOTSUPPORTED,		
 *  RADARDEMO_DOPPLERPROC_INPUTANDFFTTYPE_MISMATCH,		
 *  RADARDEMO_DOPPLERPROC_FAIL_ALLOCATE_HANDLE,			
 *  RADARDEMO_DOPPLERPROC_FAIL_ALLOCATE_LOCALINSTMEM,	
 *  RADARDEMO_DOPPLERPROC_INOUTPTR_NOTCORRECT,			
 *   }   RADARDEMO_dopplerProc_errorCode;
 *
 *  \brief   enum for Doppler processing error code.
 *
 *
 */

typedef enum
{
	RADARDEMO_DOPPLERPROC_NO_ERROR = 0,						/**< No error */
	RADARDEMO_DOPPLERPROC_2DFFTTYPE_NOTSUPPORTED,			/**< 2D FFT type not supported*/
	RADARDEMO_DOPPLERPROC_2DFFTSIZE_NOTVALID,				/**< 2D FFT non valid size */ 
	RADARDEMO_DOPPLERPROC_INTEGRTYPE_NOTSUPPORTED,			/**< unsupported integration type*/
	RADARDEMO_DOPPLERPROC_INPUTANDFFTTYPE_MISMATCH,			/**< Doppler processing output type and FFT type mismatch*/
	RADARDEMO_DOPPLERPROC_FAIL_ALLOCATE_HANDLE,				/**< RADARDEMO_dopplerProc_create failed to allocate handle */ 
	RADARDEMO_DOPPLERPROC_FAIL_ALLOCATE_LOCALINSTMEM,		/**< RADARDEMO_dopplerProc_create failed to allocate memory for buffers in local instance  */ 
	RADARDEMO_DOPPLERPROC_INOUTPTR_NOTCORRECT				/**< input and/or output buffer for RADARDEMO_dopplerProc_run are either NULL, or not aligned properly  */
} RADARDEMO_dopplerProc_errorCode;


/**
 *  \enum   
 *   {
 *	RADARDEMO_DOPPLERPROC_2DFFT_SPxSP = 0,	
 *	RADARDEMO_DOPPLERPROC_2DFFT_NOT_SUPPORTED
 *   }   RADARDEMO_dopplerProc_2DFFTType;
 *
 *  \brief   enum for 2D FFT types.
 *
 *
 */

typedef enum
{
	RADARDEMO_DOPPLERPROC_2DFFT_SPxSP = 0,			/**< 2D FFT type: spxsp single precision floating point*/
	RADARDEMO_DOPPLERPROC_2DFFT_NOT_SUPPORTED
} RADARDEMO_dopplerProc_2DFFTType;

/**
 *  \enum   
 *   {
 *	RADARDEMO_DOPPLERPROC_INTGRTYPE_SP = 0,	
 *	RADARDEMO_DOPPLERPROC_INTGRTYPE_NOT_SUPPORTED
 *   }   RADARDEMO_dopplerProc_intgrType;
 *
 *  \brief   enum for integration type.
 *
 *
 */

typedef enum
{
	RADARDEMO_DOPPLERPROC_INTGRTYPE_SP = 0,			/**< integration type: single precision floating point*/
	RADARDEMO_DOPPLERPROC_INTGRTYPE_NOT_SUPPORTED
} RADARDEMO_dopplerProc_intgrType;


/**
 *  \struct   _RADARDEMO_dopplerProc_input_
 *   {
 *   	void       **dopplerProcInput;  		
 *      uint8_t    reGen2DFFTout4AoAFlag;
 *		uint16_t   dopplerIndex;	
 *   }   RADARDEMO_dopplerProc_input;
 *
 *  \brief   Structure for input to RADARDEMO_rangeProc module.
 *
 *
 */

typedef struct _RADARDEMO_dopplerProc_input_
{
	void		**dopplerProcInput;  		/**< Input signal (from range processing). If handle->fxdpInputFlag = 1, 
											input is cplx16_t type with size [handle->nRxAnt][handle->fft2DSize].
											If handle->fxdpInputFlag = 0, input is float type with size [handle->nRxAnt][2 * handle->fft2DSize] 
											and stored in real imag order in memory. Must be aligned to 8-byte boundary*/
	uint8_t    reGen2DFFTout4AoAFlag;		/**< Input Flag to indicate module is called for regenerating 2DFFT output purpose only. 
											If set to 1, generate 2D FFT output for the corresponding input dopplerIndex. 
											In this case, integration function is disabled. nRxAnt number of antenna samples corresponding to 
											dopplerIndex will be stored in ouput.
											If set to 0, module is called for regualar doppler processing with integration, and integration output will be stored in output.
											*/
	uint8_t		dopplerComp4TDMMimo;        /**< Input flag to enable Doppler compensation for TDM MIMO if set to 1, and if reGen2DFFTout4AoAFlag is set to 1*/
	uint16_t   dopplerIndex;				/**< Input dopplerIndex, only used when reGen2DFFTout4AoAFlag is set to 1*/									
} RADARDEMO_dopplerProc_input;


/**
 *  \struct   _RADARDEMO_dopplerProc_config_
 *   {
 *   	uint32_t     fft2DSize;  					
 *		uint8_t      nRxAnt;  	
  *     uint8_t      fxdpInputFlag;
 *		RADARDEMO_dopplerProc_2DFFTType fft2DType;  	   
 *		uint8_t      include2DwinFlag;  	    	
 *      int16_t      * win2D;  	   
 *   }   RADARDEMO_dopplerProc_config;
 *
 *  \brief   Structure element of the list of descriptors for RADARDEMO_dopplerProc configuration.
 *
 *
 */

typedef struct _RADARDEMO_dopplerProc_config_
{
	uint32_t     fft2DSize;  					/**< 2D FFT size*/
	uint32_t     numChirpsPerFrame;  			/**< number of chirps per frame */
	uint8_t      nRxAnt;  	       				/**< number of receive antennas.*/
	uint8_t      nRxPhyAnt;                     /**< number of physical RX antennas.*/
	uint8_t      fxdpInputFlag;    				/**< Flag indicating fixedpoint input to Doppler proc module. 
												set to 1 for fixed point, set to 0 for floating point.
												If set to 1, we have to have include2DwinFlag set to 1 since we only support floating point 2D FFT in current version.
												If set to 0, we have to have include2DwinFlag set to 0 for best cycle performance.*/
	uint8_t      include2DwinFlag;  	    	/**< Flag indicating 2D windowing is done with doppler processing when set to 1. Otherwise, 2D windowing is done in range processing*/
	uint8_t      DCRemovalFlag;  	    		/**< Flag to enable DC removal for clutter mitigation*/
	RADARDEMO_dopplerProc_2DFFTType fft2DType;  /**< Type of 2D FFT.*/
	RADARDEMO_dopplerProc_intgrType intgrType;  /**< Type of integration.*/
	float        * win2D;  	    	            /**< pointer to 2D windowing function.*/
	uint32_t     win2DSize;  					/**< 2D window size*/
} RADARDEMO_dopplerProc_config;


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

extern void	* RADARDEMO_dopplerProc_create(
                            IN  RADARDEMO_dopplerProc_config * moduleConfig, 
							OUT RADARDEMO_dopplerProc_errorCode * errorCode);

/*! 
   \fn     RADARDEMO_dopplerProc_delete
 
   \brief   Delete RADARDEMO_dopplerProc module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
  
 
 */

extern void	RADARDEMO_dopplerProc_delete(
                            IN  void * handle);


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

extern RADARDEMO_dopplerProc_errorCode	RADARDEMO_dopplerProc_run(
                            IN  void * handle,
							IN RADARDEMO_dopplerProc_input * sDopplerProcInput,
							OUT float  * outputSignal); 
#endif //RADARDEMO_DOPPLERPROC_H


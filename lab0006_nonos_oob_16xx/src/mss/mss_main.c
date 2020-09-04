/**
 *   @file  mss_main.c
 *
 *   @brief
 *     MSS main implementation of the millimeter wave Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
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
 */
 
/** @mainpage Non-OS OOB (mmw) Demo
 *  This application provides a reference of mmW demo without any OS running. For any further
 *  documentation please refer mmw demo doxygen document which is part of mmWave SDK installation.
 *
 *  Feature & Limitation:
 *   - mmwave layer is removed from MSS & DSS whereas an instance of mmWaveLink runs on DSS. DSS 
 *     configures RadarSS invoking mmWaveLink APIs and triggers the frame.
 *   - Processing chain at DSS is same as default mmw demo available in mmWave-SDK 2.0. 
 *   - Doesn't support listening any CLI command when device is chirping. Due to having a single 
 *     thread in Non-OS environment while device is chirping MSS will be busy reading object data 
 *     from DSS and sending over UART. So it can't do other task(reading CLI command) in that time.
 *   - LVDS, CQ and analog features is disabled in this application. This application is providing 
 *     a reference for basic mmw demo without OS.
 *   - Fastcode feature (code loaded in L3SRAM but run from L1PSRAM) from dss_mmw_linker.cmd is not 
 *     available in this application. Full L1PSRAM is used as cache.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <ti/drivers/osal/SemaphoreP.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/HwiP.h>

/* Demo Include Files */
#include "mss_mmw.h"
#include "common/mmw_messages.h"
#include "osal_nonos/Event_nonos.h"

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwDemo_MCB    gMmwMssMCB;
uint8_t    mmwaveExec = 0;

/**************************************************************************
 *************************** Extern Definitions *******************************
 **************************************************************************/
/* CLI Init function */
extern void MmwDemo_CLIInit (void);
extern void CLI_getMMWaveExtensionOpenConfig(MMWave_OpenCfg* ptrOpenCfg);
extern void CLI_getMMWaveExtensionConfig(MMWave_CtrlCfg* ptrCtrlCfg);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions Prototype**************
 **************************************************************************/

extern void CLI_write (const char* format, ...);
extern void CLI_task(void);

/* Data path functions */
int32_t MmwDemo_mssDataPathConfig(void);
int32_t MmwDemo_mssDataPathStop(void);


/* MMW demo Task */
void MmwDemo_mssInitTask(unsigned int arg0, unsigned int arg1);
void MmwDemo_mmWaveCtrlTask(unsigned int arg0, unsigned int arg1);
void MmwDemo_mssCtrlPathTask(unsigned int event);

/* external sleep function when in idle (used in .cfg file) */
void MmwDemo_sleep(void);

/* DSS to MSS exception signalling ISR */
static void MmwDemo_installDss2MssExceptionSignallingISR(void);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/


/**
 *  @b Description
 *  @n
 *      Function to send a message to peer through Mailbox virtural channel 
 *
 *  @param[in]  message
 *      Pointer to the MMW demo message.  
 *
 *  @retval
 *      Success    - 0
 *      Fail       < -1 
 */
int32_t MmwDemo_mboxWrite(MmwDemo_message     * message)
{
    int32_t                  retVal = -1;
    
    retVal = Mailbox_write (gMmwMssMCB.peerMailbox, (uint8_t*)message, sizeof(MmwDemo_message));
    if (retVal == sizeof(MmwDemo_message))
    {
        retVal = 0;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The Task is used to handle  the mmw demo messages received from 
 *      Mailbox virtual channel.  
 *
 *  @param[in]  arg0
 *      arg0 of the Task. Not used
 *  @param[in]  arg1
 *      arg1 of the Task. Not used
 *
 *  @retval
 *      Not Applicable.
 */
rlVersion_t gDevVersion;

static void MmwDemo_mboxReadTask(unsigned int arg0, unsigned int arg1)
{
    MmwDemo_message      message;
    int32_t              retVal = 0;
    uint32_t totalPacketLen;
    uint32_t numPaddingBytes;
    uint32_t itemIdx;
    int32_t i;

    /* Read the message from the peer mailbox: We are not trying to protect the read
     * from the peer mailbox because this is only being invoked from a single thread */
    retVal = Mailbox_read(gMmwMssMCB.peerMailbox, (uint8_t*)&message, sizeof(MmwDemo_message));
    if (retVal < 0)
    {
        /* Error: Unable to read the message. Setup the error code and return values */
        printf ("Error: Mailbox read failed [Error code %d]\n", retVal);
    }
    else if (retVal == 0)
    {
        /* We are done: There are no messages available from the peer execution domain. */
        return;
    }
    else
    {
        /* Flush out the contents of the mailbox to indicate that we are done with the message. This will
         * allow us to receive another message in the mailbox while we process the received message. */
        Mailbox_readFlush (gMmwMssMCB.peerMailbox);

        /* Process the received message: */
        switch (message.type)
        {
            case MMWDEMO_DSS2MSS_VERSION_INFO:
                memcpy(&gDevVersion, &message.body.devVerArgs, sizeof(rlVersion_t));
                break;
            case MMWDEMO_DSS2MSS_DETOBJ_READY:
                /* Got detetced objectes , shipped out through UART */
                /* Send header */
                totalPacketLen = sizeof(MmwDemo_output_message_header);
                UART_writePolling (gMmwMssMCB.loggingUartHandle,
                                   (uint8_t*)&message.body.detObj.header,
                                   sizeof(MmwDemo_output_message_header));

                /* Send TLVs */
                for (itemIdx = 0;  itemIdx < message.body.detObj.header.numTLVs; itemIdx++)
                {
                    UART_writePolling (gMmwMssMCB.loggingUartHandle,
                                       (uint8_t*)&message.body.detObj.tlv[itemIdx],
                                       sizeof(MmwDemo_output_message_tl));
                    UART_writePolling (gMmwMssMCB.loggingUartHandle,
                                       (uint8_t*)SOC_translateAddress(message.body.detObj.tlv[itemIdx].address,
                                                                      SOC_TranslateAddr_Dir_FROM_OTHER_CPU,NULL),
                                       message.body.detObj.tlv[itemIdx].length);
                    totalPacketLen += sizeof(MmwDemo_output_message_tl) + message.body.detObj.tlv[itemIdx].length;
                }

                /* Send padding to make total packet length multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN */
                numPaddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - (totalPacketLen & (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1));
                if (numPaddingBytes<MMWDEMO_OUTPUT_MSG_SEGMENT_LEN)
                {
                    uint8_t padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];
                    /*DEBUG:*/ memset(&padding, 0xf, MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
                    UART_writePolling (gMmwMssMCB.loggingUartHandle,
                                        padding,
                                        numPaddingBytes);
                }

                /* Send a message to MSS to log the output data */
                memset((void *)&message, 0, sizeof(MmwDemo_message));

                message.type = MMWDEMO_MSS2DSS_DETOBJ_SHIPPED;

                if (MmwDemo_mboxWrite(&message) != 0)
                {
                    printf ("Error: Mailbox send message id=%d failed \n", message.type);
                }

                break;
            case MMWDEMO_DSS2MSS_STOPDONE:
                gMmwMssMCB.isReadyForCli = 1;
                /* Post event that stop is done */
                Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_STOP_COMPLETED_EVT);
            break;
            case MMWDEMO_DSS2MSS_ASSERT_INFO:
                /* Send the received DSS assert info through CLI */
                CLI_write ("DSS Exception: %s, line %d.\n", message.body.assertInfo.file, \
                    message.body.assertInfo.line);
            break;
            case MMWDEMO_DSS2MSS_STATUS_INFO:
                 if(message.body.dssStatusInfo.eventType == MMWDEMO_BSS_CALIBRATION_REP_EVT)
                 {
                     CLI_write("Rf-init status [0x%x]\n", message.body.dssStatusInfo.errVal);
                 }
                 else if(message.body.dssStatusInfo.eventType == MMWDEMO_DSS_MMWAVELINK_FAILURE_EVT)
                 {
                     CLI_write("Link API failed [%d]\n", message.body.dssStatusInfo.errVal);
                     gMmwMssMCB.isReadyForCli = 1;
                 }
                 else if(message.body.dssStatusInfo.eventType == MMWDEMO_DSS_INIT_CONFIG_DONE_EVT)
                 {
                     gMmwMssMCB.isReadyForCli = 1;
                 }
                 else if(message.body.dssStatusInfo.eventType == MMWDEMO_DSS_START_COMPLETED_EVT)
                 {
                    /* Post event that Sensor Start successfully done by DSS */
                    Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_START_COMPLETED_EVT);
                 }
                 else if(message.body.dssStatusInfo.eventType == MMWDEMO_DSS_START_FAILED_EVT)
                 {
                    /* Post event that Sensor Start failed by DSS */
                    Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_START_FAILED_EVT);
                 }
                 else
                 {
                    /* do nothing */
                 }


                break;
            case MMWDEMO_DSS2MSS_ISR_INFO_ADDRESS:
                gMmwMssMCB.dss2mssIsrInfoAddress = 
                    SOC_translateAddress(message.body.dss2mssISRinfoAddress,
                                         SOC_TranslateAddr_Dir_FROM_OTHER_CPU, NULL);
                MmwDemo_installDss2MssExceptionSignallingISR();
            break;
            case MMWDEMO_DSS2MSS_MEASUREMENT_INFO:
                /* Send the received DSS calibration info through CLI */
                CLI_write ("compRangeBiasAndRxChanPhase");
                CLI_write (" %.7f", message.body.compRxChanCfg.rangeBias);
                for (i = 0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
                {
                    CLI_write (" %.5f", (float) message.body.compRxChanCfg.rxChPhaseComp[i].real/32768.);
                    CLI_write (" %.5f", (float) message.body.compRxChanCfg.rxChPhaseComp[i].imag/32768.);
                }
                CLI_write ("\n");
            break;
            default:
            {
                /* Message not support */
                printf ("Error: unsupport Mailbox message id=%d\n", message.type);
                break;
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      This function is a callback funciton that invoked when a message is received from the peer.
 *
 *  @param[in]  handle
 *      Handle to the Mailbox on which data was received
 *  @param[in]  peer
 *      Peer from which data was received
 
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mboxCallback
(
    Mbox_Handle  handle,
    Mailbox_Type    peer
)
{
    /* Message has been received from the peer endpoint. Wakeup the mmWave thread to process
     * the received message. */
    SemaphoreP_post (gMmwMssMCB.mboxSemHandle);
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be invoked from the CLI or
 *      the Switch press to start the sensor. This sends an event to the
 *      sensor management task where the actual *start* procedure is
 *      implemented.
 *
 *  @retval
 *      Not applicable
 */
int32_t MmwDemo_notifySensorStart(bool doReconfig)
{
    MmwDemo_message     message;

    gMmwMssMCB.stats.cliSensorStartEvt ++;

    /* we need to wait for Sensor-start response from DSS, so disable to read next CLI  */
    gMmwMssMCB.isReadyForCli = 0;

    /* Get the open configuration from the CLI mmWave Extension */
    CLI_getMMWaveExtensionOpenConfig (&gMmwMssMCB.cfg.openCfg);

    gMmwMssMCB.cfg.openCfg.freqLimitLow  = 760U;
    gMmwMssMCB.cfg.openCfg.freqLimitHigh = 810U;
    gMmwMssMCB.cfg.openCfg.disableFrameStartAsyncEvent = true;
    gMmwMssMCB.cfg.openCfg.disableFrameStopAsyncEvent  = false;

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    /* send these configurations to DSS over Mailbox */
    message.type = MMWDEMO_MSS2DSS_OPEN_CFG;
    message.subFrameNum = -1;
    memcpy((void *)&message.body.openCfg, (void *)&gMmwMssMCB.cfg.openCfg, sizeof(MMWave_OpenCfg));

    if (MmwDemo_mboxWrite(&message) != 0)
        return -1;

    CLI_getMMWaveExtensionConfig (&gMmwMssMCB.cfg.ctrlCfg);

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_CONTROL_CFG;
    message.subFrameNum = -1;
    memcpy((void *)&message.body.ctrlCfg, (void *)&gMmwMssMCB.cfg.ctrlCfg, sizeof(MMWave_CtrlCfg));

    if (MmwDemo_mboxWrite(&message) != 0)
        return -1;

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_SENSOR_START_CMD;
    message.subFrameNum = -1;

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;

}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be invoked from the CLI or
 *      the Switch press to start the sensor. This sends an event to the
 *      sensor management task where the actual *start* procedure is
 *      implemented.
 *
 *  @retval
 *      Not applicable
 */
int32_t MmwDemo_notifySensorStop(void)
{
    MmwDemo_message message;

    gMmwMssMCB.stats.cliSensorStopEvt ++;

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_SENSOR_STOP_CMD;
    message.subFrameNum = -1;

    /* we need to wait for Sensor-stop response from DSS, so disable to read next CLI  */
    gMmwMssMCB.isReadyForCli = 0;

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be used by the CLI to 
 *      pend for start complete (after MmwDemo_notifySensorStart is called)
 *
 *  @retval
 *      0: sensorStart success
 *     -1: sensorStart failed
 *    0xF: no event
 */
int32_t MmwDemo_waitSensorStartComplete(void)
{
    uint32_t      event;
    int32_t       retVal;
    /* Pend on the START NOTIFY event */
    event = Event_pend(gMmwMssMCB.eventHandleNotify, 
                          Event_Id_NONE, 
                          MMWDEMO_DSS_START_COMPLETED_EVT | MMWDEMO_DSS_START_FAILED_EVT,
                          EventP_NO_WAIT);

    /************************************************************************
     * DSS event:: START notification
     ************************************************************************/
    if(event & MMWDEMO_DSS_START_COMPLETED_EVT)
    {
        /* Sensor has been started successfully */
        gMmwMssMCB.isSensorStarted = true;
        /* Turn on the LED */
        GPIO_write (SOC_XWR16XX_GPIO_2, 1U);
        retVal = 0;
            
        /* Enable back CLI for next command to read over UART if event received */
        gMmwMssMCB.isReadyForCli = 1;
    }
    else if(event & MMWDEMO_DSS_START_FAILED_EVT)
    {
        /* Sensor start failed */
        gMmwMssMCB.isSensorStarted = false;
        retVal = -1;
            
        /* Enable back CLI for next command to read over UART if event received */
        gMmwMssMCB.isReadyForCli = 1;
    } 
    else 
    {
        /* If above two events didn't occur then return with non-zero +ve value */
        retVal = 0xF;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be used by the CLI to 
 *      pend for stop complete (after MmwDemo_notifySensorStart is called)
 *
 *  @retval
 *      0: sensorStop done
 *    0xF: no event
 */
int32_t MmwDemo_waitSensorStopComplete(void)
{
    uint32_t      event;
    int32_t       retVal;
    /* Pend on the START NOTIFY event */
    event = Event_pend(gMmwMssMCB.eventHandleNotify, 
                          Event_Id_NONE, 
                          MMWDEMO_DSS_STOP_COMPLETED_EVT,
                          EventP_NO_WAIT);

    /************************************************************************
     * DSS event:: STOP notification
     ************************************************************************/
    if(event & MMWDEMO_DSS_STOP_COMPLETED_EVT)
    {
        /* Sensor has been stopped successfully */
        gMmwMssMCB.isSensorStarted = false;
        
        /* Turn off the LED */
        GPIO_write (SOC_XWR16XX_GPIO_2, 0U);
        
        /* print for user */
        printf("Sensor has been stopped\n");
        retVal = 0;
    }
    else
    {
        /* If above event didn't occur then return with non-zero +ve value */
        retVal = 0xF;
    }
    return retVal;
}



/**
 *  @b Description
 *  @n
 *      Callback function invoked when the GPIO switch is pressed.
 *      This is invoked from interrupt context.
 *
 *  @param[in]  index
 *      GPIO index configured as input
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_switchPressFxn(unsigned int index)
{
    /* Was the sensor started? */
    if (gMmwMssMCB.isSensorStarted == true)
    {
        /* YES: We need to stop the sensor now */
        MmwDemo_notifySensorStop();
    }
    else
    {
        /* NO: We need to start the sensor now. */
        MmwDemo_notifySensorStart(true);
    }
}

/**
 *  @b Description
 *  @n
 *      The task is used to process data path events
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mssCtrlPathTask(unsigned int event)
{

    /************************************************************************
     * BSS event:: CPU fault
     ************************************************************************/
    if(event & MMWDEMO_BSS_CPUFAULT_EVT)
    {
        CLI_write ("BSS is in CPU fault.\n");
        MmwDemo_mssAssert(0);
    }

    /************************************************************************
     * BSS event:: ESM fault
     ************************************************************************/
    if(event & MMWDEMO_BSS_ESMFAULT_EVT)
    {
        CLI_write ("BSS is in ESM fault.\n");
        MmwDemo_mssAssert(0);
    }

    if(event & MMWDEMO_DSS_CHIRP_PROC_DEADLINE_MISS_EVT)
    {
        CLI_write ("DSS Chirp Processing Deadline Miss Exception.\n");
        DebugP_assert(0);
    }

    if(event & MMWDEMO_DSS_FRAME_PROC_DEADLINE_MISS_EVT)
    {
        CLI_write ("DSS Frame Processing Deadline Miss Exception.\n");
        DebugP_assert(0);
    }

    printf("Debug: MMWDemoDSS Data path exit\n");
}

/**
 *  @b Description
 *  @n
 *      DSS to MSS ISR used for direct signalling of things like urgent exception
 *      events from DSS. Posts deadline miss events to @ref MmwDemo_mssCtrlPathTask.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_Dss2MssISR(uintptr_t arg)
{  
    switch(*(uint8_t*)gMmwMssMCB.dss2mssIsrInfoAddress)
    {
        case MMWDEMO_DSS2MSS_CHIRP_PROC_DEADLINE_MISS_EXCEPTION:
            Event_post(gMmwMssMCB.eventHandle, MMWDEMO_DSS_CHIRP_PROC_DEADLINE_MISS_EVT);
        break;
        
        case MMWDEMO_DSS2MSS_FRAME_PROC_DEADLINE_MISS_EXCEPTION:
            Event_post(gMmwMssMCB.eventHandle, MMWDEMO_DSS_FRAME_PROC_DEADLINE_MISS_EVT);
        break;

        case MMWDEMO_DSS2MSS_FRAME_TRIGGERED_EVENT:
            Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_START_COMPLETED_EVT);
        break;
        default:
            MmwDemo_mssAssert(0);
        break;
    }
}

/**
 *  @b Description
 *  @n
 *      Installs DSS to MSS Software Interrupt ISR for exception signalling.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_installDss2MssExceptionSignallingISR(void)
{
    HwiP_Params  hwiParams;
    volatile HwiP_Handle  hwiHandle;

    HwiP_Params_init(&hwiParams);
    hwiParams.name = "Dss2MssSwISR";
    hwiHandle = HwiP_create(MMWDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_MSS, 
                            MmwDemo_Dss2MssISR, &hwiParams);
}

/**
 *  @b Description
 *  @n
 *      Non OS loop function which checks for DSS Mailbox message, CLI command and any event
 *      from DSS for number of loop count provided in function argument.
 *
 *  @param[in]  number of loop count, 0: infinite, 1: single count
  *
 *  @retval NONE
 */
void MmwDemo_nonOsLoop(uint8_t count)
{
    unsigned int matchingEvents;

    /* DSS Does RadarSS configuration */
    do
    {
        if(gMmwMssMCB.isReadyForCli)
        {
            CLI_task();
        }

        if(SemaphoreP_pend(gMmwMssMCB.mboxSemHandle, SemaphoreP_NO_WAIT) == SemaphoreP_OK)
        {
            /* Create task to handle mailbox messges */
            MmwDemo_mboxReadTask(0,0);
        }

        /* Check if DSS/BSS is in fault or exception */
        matchingEvents = Event_pend(gMmwMssMCB.eventHandle, Event_Id_NONE, MMWDEMO_BSS_FAULT_EVENTS | MMWDEMO_DSS_EXCEPTION_EVENTS, EventP_NO_WAIT);
        
        if(matchingEvents != 0)
        {
            /*****************************************************************************
             * Create a data path management task to handle data Path events
             *****************************************************************************/
            MmwDemo_mssCtrlPathTask(matchingEvents);
        }
    } while(!count);
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
UART_Params         uartParams;
void MmwDemo_mssInitTask(unsigned int arg0, unsigned int arg1)
{
    int32_t             errCode;
    SemaphoreP_Params   semParams;
    Mailbox_Config      mboxCfg;

    /* Debug Message: */
    printf("Debug: MMWDemoMSS Launched the Initialization Task\n");

    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/
    /* Pinmux setting */

    /* Setup the PINMUX to bring out the UART-1 */
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINN5_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);    
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINN5_PADBE, SOC_XWR16XX_PINN5_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINN4_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);    
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINN4_PADBD, SOC_XWR16XX_PINN4_PADBD_MSS_UARTA_RX);

    /* Setup the PINMUX to bring out the UART-3 */
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINF14_PADAJ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINF14_PADAJ, SOC_XWR16XX_PINF14_PADAJ_MSS_UARTB_TX);

    /* Setup the PINMUX to bring out the DSS UART */
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINP8_PADBM, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINP8_PADBM, SOC_XWR16XX_PINP8_PADBM_DSS_UART_TX);

    /**********************************************************************
     * Setup the PINMUX:
     * - GPIO Input : Configure pin J13 as GPIO_1 input
     * - GPIO Output: Configure pin K13 as GPIO_2 output
     **********************************************************************/
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINJ13_PADAC, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINJ13_PADAC, SOC_XWR16XX_PINJ13_PADAC_GPIO_1);
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINK13_PADAZ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINK13_PADAZ, SOC_XWR16XX_PINK13_PADAZ_GPIO_2);

    /* Initialize the GPIO */
    GPIO_init();
    /**********************************************************************
     * Setup the SW1 switch on the EVM connected to GPIO_1
     * - This is used as an input
     * - Enable interrupt to be notified on a switch press
     **********************************************************************/
    GPIO_setConfig (SOC_XWR16XX_GPIO_1, GPIO_CFG_INPUT | GPIO_CFG_IN_INT_RISING | GPIO_CFG_IN_INT_LOW);
    GPIO_setCallback (SOC_XWR16XX_GPIO_1, MmwDemo_switchPressFxn);
    GPIO_enableInt (SOC_XWR16XX_GPIO_1);

    /**********************************************************************
     * Setup the DS3 LED on the EVM connected to GPIO_2
     **********************************************************************/
    GPIO_setConfig (SOC_XWR16XX_GPIO_2, GPIO_CFG_OUTPUT);

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_MSS);

    /* Initialize the UART */
    UART_init();

    /*****************************************************************************
     * Open & configure the drivers:
     *****************************************************************************/

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency  = gMmwMssMCB.cfg.sysClockFrequency;
    uartParams.baudRate        = gMmwMssMCB.cfg.commandBaudRate;
    uartParams.isPinMuxDone    = 1U;

    /* Open the UART Instance */
    gMmwMssMCB.commandUartHandle = UART_open(0, &uartParams);
    if (gMmwMssMCB.commandUartHandle == NULL)
    {
        printf("Error: MMWDemoMSS Unable to open the Command UART Instance\n");
        return;
    }

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency = gMmwMssMCB.cfg.sysClockFrequency;
    uartParams.baudRate       = gMmwMssMCB.cfg.loggingBaudRate;
    uartParams.isPinMuxDone   = 1U;

    /* Open the Logging UART Instance: */
    gMmwMssMCB.loggingUartHandle = UART_open(1, &uartParams);
    if (gMmwMssMCB.loggingUartHandle == NULL)
    {
        printf("Error: MMWDemoMSS Unable to open the Logging UART Instance\n");
        return;
    }

    /*****************************************************************************
     * Creating communication channel between MSS & DSS
     *****************************************************************************/

    /* Create a binary semaphore which is used to handle mailbox interrupt. */
    SemaphoreP_Params_init(&semParams);
    semParams.mode             = SemaphoreP_Mode_BINARY;
    gMmwMssMCB.mboxSemHandle = SemaphoreP_create(0, &semParams);

    /* Setup the default mailbox configuration */
    Mailbox_Config_init(&mboxCfg);

    /* Setup the configuration: */
    mboxCfg.chType       = MAILBOX_CHTYPE_MULTI;
    mboxCfg.chId         = MAILBOX_CH_ID_0;
    mboxCfg.writeMode    = MAILBOX_MODE_BLOCKING;
    mboxCfg.readMode     = MAILBOX_MODE_CALLBACK;
    mboxCfg.readCallback = &MmwDemo_mboxCallback;

    /* Initialization of Mailbox Virtual Channel  */
    gMmwMssMCB.peerMailbox = Mailbox_open(MAILBOX_TYPE_DSS, &mboxCfg, &errCode);
    if (gMmwMssMCB.peerMailbox == NULL)
    {
        /* Error: Unable to open the mailbox */
        printf("Error: Unable to open the Mailbox to the DSS [Error code %d]\n", errCode);
        return;
    }

    /* Remote Mailbox is operational: Set the DSS Link State */
    SOC_setMMWaveMSSLinkState (gMmwMssMCB.socHandle, 1U, &errCode);

    /*****************************************************************************
     * Create Event to handle mmwave callback and system datapath events 
     *****************************************************************************/
    /* Default instance configuration params */
    gMmwMssMCB.eventHandle = Event_create(NULL, 0);
    if (gMmwMssMCB.eventHandle == NULL) 
    {
        MmwDemo_mssAssert(0);
        return ;
    }
    
    gMmwMssMCB.eventHandleNotify = Event_create(NULL, 0);
    if (gMmwMssMCB.eventHandleNotify == NULL) 
    {
        MmwDemo_mssAssert(0);
        return ;
    }

    /*****************************************************************************
     * mmWave: Initialization of the high level module
     *****************************************************************************/

    /* Synchronization: This will synchronize the execution of the control module
     * between the MSS and DSS cores. This is a prerequisite and always needs to be invoked. */
    while (1)
    {
        volatile int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = SOC_isMMWaveDSSOperational (gMmwMssMCB.socHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the mmWave control module */
            printf ("Error: MMWDemoMSS mmWave Control Synchronization failed [Error code %d]\n", errCode);
            return;
        }
        if (syncStatus == 1)
        {
            /* Synchronization achieved: */
            break;
        }
        /* Sleep and poll again: */
        MmwDemo_sleep();
    }  

    /*****************************************************************************
     * At this point, MSS and DSS are both up and synced. Configuration is ready to be sent.
     * Start CLI to get configuration from user
     *****************************************************************************/
    MmwDemo_CLIInit();

    /* Non OS loop for infinite count where it will check for DSS mailbox event, CLI commands  */
    MmwDemo_nonOsLoop(0);
   
    return;
}

/**
 *  @b Description
 *  @n
 *     Function to sleep the R4F using WFI (Wait For Interrupt) instruction. 
 *     When R4F has no work left to do,
 *     the BIOS will be in Idle thread and will call this function. The R4F will
 *     wake-up on any interrupt (e.g chirp interrupt).
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_sleep(void)
{
    /* issue WFI (Wait For Interrupt) instruction */
    asm(" WFI ");
}

/**
 *  @b Description
 *  @n
 *      Send MSS assert information through CLI.
 */
void _MmwDemo_mssAssert(int32_t expression, const char *file, int32_t line)
{
    if (!expression) {
        CLI_write ("MSS Exception: %s, line %d.\n",file,line);
    }
}

/**
 *  @b Description
 *  @n
 *      Entry point into the Millimeter Wave Demo
 *
 *  @retval
 *      Not Applicable.
 */
int main (void)
{
    int32_t         errCode;
    SOC_Cfg         socCfg;

    /* Initialize and populate the demo MCB */
    memset ((void*)&gMmwMssMCB, 0, sizeof(MmwDemo_MCB));

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Initialize the ESM: */
    gMmwMssMCB.esmHandle = ESM_init(0U); //dont clear errors as TI RTOS does it

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    gMmwMssMCB.socHandle   = SOC_init (&socCfg, &errCode);
    if (gMmwMssMCB.socHandle  == NULL)
    {
        printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the DEMO configuration: */
    gMmwMssMCB.cfg.sysClockFrequency = MSS_SYS_VCLK;
    gMmwMssMCB.cfg.loggingBaudRate   = 921600;
    gMmwMssMCB.cfg.commandBaudRate   = 115200;

    /* Debug Message: */
    printf ("**********************************************\n");
    printf ("Debug: Launching the Millimeter Wave Demo\n");
    printf ("**********************************************\n");

    MmwDemo_mssInitTask(0,0);

    return 0;
}

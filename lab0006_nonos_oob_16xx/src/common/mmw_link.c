/*
 *   @file  mmw_link.c
 *
 *   @brief
 *      The file contains common functions of mmWave Link API
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include <ti/drivers/osal/SemaphoreP.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/osal/hwiP.h>
#include <ti/demo/io_interface/mmw_config.h>
#include "common/mmw_messages.h"
#include "dss_mmw.h"

/**************************************************************************
 *************************** Local Structures *****************************
 **************************************************************************/

/**
 * @brief
 *  Mmwave Link Master Control Block
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  Mmwave Link.
 */
typedef struct MmwaveLink_MCB
{
    /**
     * @brief   Handle to the BSS Mailbox
     */
    Mbox_Handle              bssMailbox;

    /**
     * @brief   Semaphore handle for the mmWave Link
     */
    SemaphoreP_Handle            linkSemaphore;

    /**
     * @brief   mmWave Link Spawning function
     */
    RL_P_OSI_SPAWN_ENTRY        spawnFxn;

    /**
     * @brief   Status of the BSS:
     */
    volatile uint32_t           bssStatus;

    uint32_t                   spwanCount;

    /**
     * @brief   Counter which tracks of the number of times the spawn function was
     * overrun.
     */
    uint32_t                    spawnOverrun;
    /**
     * @brief   Handle to the CRC Channel
     */
    CRC_Handle                  crcHandle;
}MmwaveLink_MCB;

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/* Global Variable for tracking information required by the mmWave Link */
MmwaveLink_MCB    gMmwaveLinkMCB;

RL_P_EVENT_HANDLER g_MailboxInterruptFunc;

uint32_t anaMonEn = 0;
/* Async Event Enable and Direction configuration */
rlRfDevCfg_t rfDevCfg = {0};

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
extern rlInt32_t Osal_mutexCreate(rlOsiMutexHdl_t* mutexHdl, rlInt8_t* name);
extern rlInt32_t Osal_mutexLock(rlOsiMutexHdl_t* mutexHdl, rlOsiTime_t timeout);
extern rlInt32_t Osal_mutexUnlock(rlOsiMutexHdl_t* mutexHdl);
extern rlInt32_t Osal_mutexDelete(rlOsiMutexHdl_t* mutexHdl);
extern rlInt32_t Osal_semCreate(rlOsiSemHdl_t* semHdl, rlInt8_t* name);
extern rlInt32_t Osal_semWait(rlOsiSemHdl_t* semHdl, rlOsiTime_t timeout);
extern rlInt32_t Osal_semSignal(rlOsiSemHdl_t* semHdl);
extern rlInt32_t Osal_semDelete(rlOsiSemHdl_t* semHdl);
extern void MmwDemo_dssStatusInfo(uint16_t eventType, int32_t errVal);

/**************************************************************************
 ************************* Link Unit Test Functions ***********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Mailbox registered function which is invoked on the reception of data
 *
 *  @retval
 *      Success - Communicate Interface Channel Handle
 *  @retval
 *      Error   - NULL
 */
static void MmwaveLink_mboxCallbackFxn (Mbox_Handle handle, Mailbox_Type remoteEndpoint)
{
    /* Indicate to the Radar Link that a message has been received. */
    g_MailboxInterruptFunc(0, NULL);
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to open the communication
 *      interface channel
 *
 *  @retval
 *      Success - Communicate Interface Channel Handle
 *  @retval
 *      Error   - NULL
 */
static rlComIfHdl_t MmwaveLink_mboxOpen(rlUInt8_t deviceIndex, uint32_t flags)
{
    Mailbox_Config  cfg;
    int32_t         errCode;

    /* Initialize the mailbox configuration: */
    if(Mailbox_Config_init(&cfg) < 0)
    {
        printf("Error: Unable to initialize mailbox configuration\n");
        return NULL;
    }

    cfg.writeMode    = MAILBOX_MODE_POLLING;
    cfg.readMode     = MAILBOX_MODE_CALLBACK;
    cfg.readCallback = MmwaveLink_mboxCallbackFxn;

    /* Open the Mailbox to the BSS */
    gMmwaveLinkMCB.bssMailbox = Mailbox_open(MAILBOX_TYPE_BSS, &cfg, &errCode);
    if (gMmwaveLinkMCB.bssMailbox == NULL)
    {
        printf("Error: Unable to open the Mailbox Instance [Error code %d]\n", errCode);
        return NULL;
    }
    printf("Debug: BSS Mailbox Handle %p\n", gMmwaveLinkMCB.bssMailbox);
    return gMmwaveLinkMCB.bssMailbox;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to close the communication
 *      interface channel
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t MmwaveLink_mboxClose(rlComIfHdl_t fd)
{
    int32_t errCode;

    /* Close the Mailbox */
    errCode = Mailbox_close ((Mbox_Handle)fd);
    if (errCode < 0)
        printf ("Error: Unable to close the BSS Mailbox [Error code %d]\n", errCode);

    return errCode;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to read data from the communication
 *      interface channel
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t MmwaveLink_mboxRead(rlComIfHdl_t fd, uint8_t* pBuff, uint16_t len)
{
    return Mailbox_read((Mbox_Handle)fd, pBuff, len);
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to write data to the communication
 *      interface channel
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t MmwaveLink_mboxWrite(rlComIfHdl_t fd, uint8_t* pBuff, uint16_t len)
{
    int32_t    status;
    /*
      Currently, the mmwavelink can not detect the error condition where it did not receive a mailbox layer ACK from BSS.

      For instance:
      - The mmwavelink may try to send a message before an ACK was received for the previous message.
      - The mmwavelink may try to resend a message that did not receive a mmwavelink layer ACK back from BSS. It is possible that the
      message did not receive a mailbox layer ACK as well from BSS.

      In either case, Mailbox_writeReset() has to be called before another message is sent to BSS.

      The mmwavelink has no hooks to call the Mailbox_writeReset().
      Therefore, a write reset is done if it is detected that a mailbox layer ACK was not received for the
      previous message (MAILBOX_ETXFULL).
     */

    status = Mailbox_write((Mbox_Handle)fd, pBuff, len);
    if(status == MAILBOX_ETXFULL)
    {
        Mailbox_writeReset((Mbox_Handle)fd);
        status = Mailbox_write((Mbox_Handle)fd, pBuff, len);
    }

    return status;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to power on the AR1XX Device
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static rlInt32_t MmwaveLink_enableDevice(rlUInt8_t deviceIndex)
{
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to power off the AR1XX Device
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static rlInt32_t MmwaveLink_disableDevice(rlUInt8_t deviceIndex)
{
    printf("Debug: Disabling the device\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to mask the interrupts.
 *      In the case of Mailbox communication interface the driver will
 *      handle the interrupt management. This function is a dummy stub
 *
 *  @retval
 *      Not applicable
 */
static void MmwaveLink_maskHostIRQ(rlComIfHdl_t fd)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to umask the interrupts.
 *      In the case of the mailbox driver we will flush out and close the
 *      read buffer.
 *
 *  @retval
 *      Not applicable
 */
void MmwaveLink_unmaskHostIRQ(rlComIfHdl_t fd)
{
    Mailbox_readFlush((Mbox_Handle)fd);
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to register the Interrupt Handler.
 *      In the case of the Mailbox the driver does the interrupt registeration and
 *      so this function is a dummy stub.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static rlInt32_t MmwaveLink_registerInterruptHandler(rlUInt8_t deviceIndex, RL_P_EVENT_HANDLER pHandler, void* pValue)
{
    g_MailboxInterruptFunc = pHandler;
    return 0;
}


/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to call the function in a different context
 *      This function is invoked from the Interrupt context.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static rlInt32_t MmwaveLink_spawn (RL_P_OSI_SPAWN_ENTRY pEntry, const void* pValue, uint32_t flags)
{
    /* Record the function which is to be spawned. */
    if (gMmwaveLinkMCB.spawnFxn != NULL)
        gMmwaveLinkMCB.spawnOverrun++;

    gMmwaveLinkMCB.spwanCount++;
    /* Record the entry to be spawned. */
    gMmwaveLinkMCB.spawnFxn = pEntry;

    /* Post the semaphore and wake up the link management task */
    SemaphoreP_post (gMmwaveLinkMCB.linkSemaphore);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Radar Link Registered Callback function to compute the CRC.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static rlInt32_t MmwaveLink_computeCRC(rlUInt8_t* data, rlUInt32_t dataLen, rlUInt8_t crcType, rlUInt8_t* crc)
{
    CRC_SigGenCfg   signGenCfg;
    int32_t         errCode;
    uint64_t        signature;
    uint32_t        index;
    uint8_t*        ptrSignature;
    uint8_t         crcLength;

    /* Initialize the signature generation configuration */
    memset ((void *)&signGenCfg, 0, sizeof(CRC_SigGenCfg));

    /* Allocate a unique transaction id: */
    if (CRC_getTransactionId (gMmwaveLinkMCB.crcHandle, &signGenCfg.transactionId, &errCode) < 0)
    {
        printf ("Error: CRC Driver Get transaction id failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Populate the signature generation configuration: */
    signGenCfg.ptrData = (uint8_t*)data;
    signGenCfg.dataLen = dataLen;

    /* Compute the signature for the specific data on Channel-1 */
    if (CRC_computeSignature (gMmwaveLinkMCB.crcHandle, &signGenCfg, &errCode) < 0)
    {
        printf ("Error: CRC Driver compute signature failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Get the Signature for Channel */
    if (CRC_getSignature (gMmwaveLinkMCB.crcHandle, signGenCfg.transactionId, &signature, &errCode) < 0)
    {
        printf ("Error: CRC Driver get signature failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Get the pointer to the CRC Signature: */
    ptrSignature = (uint8_t*)&signature;

    /* Determine the length of the CRC: */
    switch (crcType)
    {
        case RL_CRC_TYPE_16BIT_CCITT:
        {
            crcLength = 2;
            break;
        }
        case RL_CRC_TYPE_32BIT:
        {
            crcLength = 4;
            break;
        }
        case RL_CRC_TYPE_64BIT_ISO:
        {
            crcLength = 8;
            break;
        }
        default:
        {
            printf ("Error: Unknown CRC Type passed from mmWave Link: %d\n", crcType);
            return -1;
        }
    }

    /* Copy the CRC signature into CRC output array */
    for(index = 0U; index < crcLength; index++)
        *(crc + index) = *(ptrSignature + index);
    return 0;
}

uint32_t MmwaveLink_getSpawnCount(void)
{
    return gMmwaveLinkMCB.spwanCount;
}

/**
 *  @b Description
 *  @n
 *      This is the task which handles the mmWave Link communication
 *      messages between the BSS and MSS.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwaveLink_mmwaveLinkMgmtTask ()
{
    RL_P_OSI_SPAWN_ENTRY    spawnFxn;
    uintptr_t               key;

    if(gMmwaveLinkMCB.spwanCount > 0)
    {
        /* Critical Section: We record the spawn function which is to be executed */
        key = HwiP_disable();
        spawnFxn = gMmwaveLinkMCB.spawnFxn;
        gMmwaveLinkMCB.spawnFxn = NULL;
        HwiP_restore (key);

        if(spawnFxn != NULL)
        {
            /* Execute the spawn function: */
            spawnFxn (NULL);
            gMmwaveLinkMCB.spwanCount--;
        }
    }

}

/**
 *  @b Description
 *  @n
 *      The function is used to get and display the version information
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_getVersion (rlVersion_t *verArgs)
{
    int32_t     retVal;
	/* currently patch binaries are available for AWR16 ES2.0 & IWR16 ES2.0 only*/
	int8_t rfPatchBuildVer, rfPatchDebugVer;

    /* Get the version string: */
    retVal = rlDeviceGetVersion(RL_DEVICE_MAP_INTERNAL_BSS, verArgs);
    if (retVal != 0)
    {
        printf ("Error: Unable to get the device version from mmWave link [Error %d]\n", retVal);
        return -1;
    }

    /* Display the version information */
    printf ("RF H/W Version    : %02d.%02d\n",
                    verArgs->rf.hwMajor, verArgs->rf.hwMinor);
    printf ("RF F/W Version    : %02d.%02d.%02d.%02d.%02d.%02d.%02d\n",
                    verArgs->rf.fwMajor, verArgs->rf.fwMinor, verArgs->rf.fwBuild, verArgs->rf.fwDebug,
                    verArgs->rf.fwYear, verArgs->rf.fwMonth, verArgs->rf.fwDay);
	rfPatchDebugVer = ((verArgs->rf.patchBuildDebug) & 0x0F);
	rfPatchBuildVer = (((verArgs->rf.patchBuildDebug) & 0xF0) >> 4);
	
	printf ("RF F/W Patch Version : %02d.%02d.%02d.%02d.%02d.%02d.%02d\n",
                    verArgs->rf.patchMajor, verArgs->rf.patchMinor, rfPatchBuildVer, rfPatchDebugVer,
                    verArgs->rf.patchYear, verArgs->rf.patchMonth, verArgs->rf.patchDay);
    printf ("mmWaveLink Version: %02d.%02d.%02d.%02d\n",
                    verArgs->mmWaveLink.major, verArgs->mmWaveLink.minor,
                    verArgs->mmWaveLink.build, verArgs->mmWaveLink.debug);
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize and setup the mmWave link
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t MmwaveLink_initLink (rlUInt8_t deviceType, rlUInt8_t platform, void* asyncEventHandler)
{
    SemaphoreP_Params        semParams;
    CRC_Config          crcCfg;
    rlClientCbs_t       RlApp_ClientCtx;
    int32_t             errCode;

    /* Initialize and populate the Mmwave Link MCB */
    memset ((void*)&gMmwaveLinkMCB, 0, sizeof(MmwaveLink_MCB));

    /*****************************************************************************
     * Start CRC driver:
     *****************************************************************************/

    /* Setup the default configuration: */
    CRC_initConfigParams(&crcCfg);

#if 1
    /*******************************************************************************
     * This is the configuration for the 16bit CRC Type:
     *******************************************************************************/
    crcCfg.channel  = CRC_Channel_CH1;
    crcCfg.mode     = CRC_Operational_Mode_FULL_CPU;
    crcCfg.type     = CRC_Type_16BIT;
    crcCfg.bitSwap  = CRC_BitSwap_MSB;
    crcCfg.byteSwap = CRC_ByteSwap_ENABLED;
    crcCfg.dataLen  = CRC_DataLen_16_BIT;
#endif
#if 0
    /*******************************************************************************
     * This is the configuration for the 32bit CRC Type:
     *******************************************************************************/
    crcCfg.channel  = CRC_Channel_CH1;
    crcCfg.mode     = CRC_Operational_Mode_FULL_CPU;
    crcCfg.type     = CRC_Type_32BIT;
    crcCfg.bitSwap  = CRC_BitSwap_LSB;
    crcCfg.byteSwap = CRC_ByteSwap_DISABLED;
    crcCfg.dataLen  = CRC_DataLen_32_BIT;
#endif
#if 0
    /*******************************************************************************
     * This is the configuration for the 64bit CRC Type:
     *******************************************************************************/
    crcCfg.channel  = CRC_Channel_CH1;
    crcCfg.mode     = CRC_Operational_Mode_FULL_CPU;
    crcCfg.type     = CRC_Type_64BIT;
    crcCfg.bitSwap  = CRC_BitSwap_MSB;
    crcCfg.byteSwap = CRC_ByteSwap_ENABLED;
    crcCfg.dataLen  = CRC_DataLen_32_BIT;
#endif

    /* Open the CRC Driver */
    gMmwaveLinkMCB.crcHandle = CRC_open (&crcCfg, &errCode);
    if (gMmwaveLinkMCB.crcHandle == NULL)
    {
        printf ("Error: Unable to open the CRC Channel [Error Code %d]\n", errCode);
        return -1;
    }
    printf("Debug: CRC Channel %p has been opened successfully\n", gMmwaveLinkMCB.crcHandle);

    /*****************************************************************************
     * Launch the Mmwave Link Tasks:
     *****************************************************************************/


    /* Debug Message: */
    printf("Debug: Launched the mmwaveLink Management Task\n");

    /* Initialize the mmWave Link Semaphore: */
    SemaphoreP_Params_init(&semParams);
    semParams.mode  = SemaphoreP_Mode_BINARY;
    gMmwaveLinkMCB.linkSemaphore = SemaphoreP_create(0, &semParams);

    /*****************************************************************************
     * Initialize the mmWave Link: We need to have the link management task
     * operational to be able to process the SPAWN function.
     *****************************************************************************/

    /* Reset the client context: */
    memset ((void *)&RlApp_ClientCtx, 0, sizeof(rlClientCbs_t));

    RlApp_ClientCtx.ackTimeout  = 1000U;

    /* Setup the crc Type in the mmWave link and synchronize this with the
     * created CRC Channel. */
    if (crcCfg.type == CRC_Type_16BIT)
        RlApp_ClientCtx.crcType = RL_CRC_TYPE_16BIT_CCITT;
    else if (crcCfg.type == CRC_Type_32BIT)
        RlApp_ClientCtx.crcType = RL_CRC_TYPE_32BIT;
    else
        RlApp_ClientCtx.crcType = RL_CRC_TYPE_64BIT_ISO;

    /* Setup the platform on which the mmWave Link executes */
    RlApp_ClientCtx.platform  = platform;
    RlApp_ClientCtx.arDevType = deviceType;

    /* Initialize the Communication Interface API: */
    RlApp_ClientCtx.comIfCb.rlComIfOpen     = MmwaveLink_mboxOpen;
    RlApp_ClientCtx.comIfCb.rlComIfClose    = MmwaveLink_mboxClose;
    RlApp_ClientCtx.comIfCb.rlComIfRead     = MmwaveLink_mboxRead;
    RlApp_ClientCtx.comIfCb.rlComIfWrite    = MmwaveLink_mboxWrite;

    /* Initialize OSI Mutex Interface */
    RlApp_ClientCtx.osiCb.mutex.rlOsiMutexCreate = Osal_mutexCreate;
    RlApp_ClientCtx.osiCb.mutex.rlOsiMutexLock   = Osal_mutexLock;
    RlApp_ClientCtx.osiCb.mutex.rlOsiMutexUnLock = Osal_mutexUnlock;
    RlApp_ClientCtx.osiCb.mutex.rlOsiMutexDelete = Osal_mutexDelete;

    /* Initialize OSI Semaphore Interface */
    RlApp_ClientCtx.osiCb.sem.rlOsiSemCreate    = Osal_semCreate;
    RlApp_ClientCtx.osiCb.sem.rlOsiSemWait      = Osal_semWait;
    RlApp_ClientCtx.osiCb.sem.rlOsiSemSignal    = Osal_semSignal;
    RlApp_ClientCtx.osiCb.sem.rlOsiSemDelete    = Osal_semDelete;

    /* Initialize OSI Queue Interface */
    RlApp_ClientCtx.osiCb.queue.rlOsiSpawn      = MmwaveLink_spawn;

    /* Initialize OSI Timer Interface */
    RlApp_ClientCtx.timerCb.rlDelay             = NULL;

    /* Initialize the CRC Interface */
    RlApp_ClientCtx.crcCb.rlComputeCRC          = MmwaveLink_computeCRC;

    /* Initialize Device Control Interface */
    RlApp_ClientCtx.devCtrlCb.rlDeviceDisable            = MmwaveLink_disableDevice;
    RlApp_ClientCtx.devCtrlCb.rlDeviceEnable             = MmwaveLink_enableDevice;
    RlApp_ClientCtx.devCtrlCb.rlDeviceMaskHostIrq        = MmwaveLink_maskHostIRQ;
    RlApp_ClientCtx.devCtrlCb.rlDeviceUnMaskHostIrq      = MmwaveLink_unmaskHostIRQ;
    RlApp_ClientCtx.devCtrlCb.rlRegisterInterruptHandler = MmwaveLink_registerInterruptHandler;

    /* Initialize the Asynchronous Event Handler: */
    RlApp_ClientCtx.eventCb.rlAsyncEvent    = asyncEventHandler;

    /* Power on the Device: */
    if (rlDevicePowerOn(1U, RlApp_ClientCtx) != 0)
    {
        printf("Error: Power on request to the BSS failed\n");
        return -1;
    }
    printf("Debug: Power on request successfully passed to the BSS\n");

    return 0;
}

int32_t MmwaveLink_getRfBootupStatus (void)
{
    int32_t         retVal;
    rlRfBootStatusCfg_t statusCfg = {0};
    
    /* Set channel configuration */
    retVal = rlGetRfBootupStatus(RL_DEVICE_MAP_INTERNAL_BSS, &statusCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlGetRfBootupStatus retVal=%d\n", retVal);
        return -1;
    }
    
    printf("Debug: Finished get radarSS bootup status to BSS\n");

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set/send mmWave Link Channel Configuration to the BSS.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setChannelConfig (rlChanCfg_t *chCfg)
{
    int32_t         retVal;

    /* Set channel configuration */
    retVal = rlSetChannelConfig(RL_DEVICE_MAP_INTERNAL_BSS,(rlChanCfg_t*)chCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: setChannelConfig retVal=%d\n", retVal);
    }
    
    printf("Debug: Finished set channel configurations to BSS\n");

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function Sets the consolidated configuration of all analog monitoring excluding 
 *      CQ & CP monitoring.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfAnaMonConfig (void)
{
    int32_t  retVal;
    /* Each bit(0-31) represents different analog monitor enable configuration */
    rlMonAnaEnables_t data = { (1 << 0)   | \
                               (1 << 1)   | \
                               (1 << 2)   | \
                               (1 << 3)   | \
                               (1 << 4)   | \
                               (1 << 5)   | \
                               (0 << 6)   | \
                               (1 << 7)   | \
                               (1 << 8)   | \
                               (0 << 9)   | \
                               (1 << 10)  | \
                               (1 << 11)  | \
                               (1 << 12)  | \
                               (0 << 13)  | \
                               (1 << 14)  | \
                               (1 << 15)  | \
                               (1 << 16)  | \
                               (1 << 17)  | \
                               (0 << 18)  | \
                               (1 << 19)  | \
                               (1 << 20)  | \
                               (1 << 21)  | \
                               (1 << 22)  | \
                               (1 << 23)  | \
                               (0 << 24)  | \
                               (0 << 25)  | \
                               (1 << 26)  | \
                               (0 << 27)  | \
                               (0 << 28)  | \
                               (0 << 29)  | \
                               (0 << 30)  | \
                               (0 << 31)
                                ,0x0};
	anaMonEn = data.enMask;
    /* Set channel configuration */
    retVal = rlRfAnaMonConfig(RL_DEVICE_MAP_INTERNAL_BSS,&data);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfAnaMonConfig retVal=%d\n", retVal);
        return -1;
    }
    printf("Debug: Finished rlRfAnaMonConfig configurations to BSS\n");
    return 0;    
}


/**
 *  @b Description
 *  @n
 *      The function is used to set/send the Adc out configuration to the BSS.
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setAdcOutConfig (rlAdcOutCfg_t *adcOutCfgArgs)
{
    int32_t         retVal;

    retVal = rlSetAdcOutConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlAdcOutCfg_t*)adcOutCfgArgs);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: setAdcOutConfig retVal=%d\n", retVal);
    }
    
    printf("Debug: Finished setAdcOutConfig to BSS\n");
    
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set low power mode.
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setLowPowerModeConfig (rlLowPowerModeCfg_t *lowPowerModeCfg)
{
    int32_t         retVal;

    retVal = rlSetLowPowerModeConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlLowPowerModeCfg_t*)lowPowerModeCfg);

    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: setLowPowerMode retVal=%d\n", retVal);
    }
    
    printf("Debug: Finished setLowPowerMode\n");
    
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test Set HSI clock API.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setHsiClk (uint16_t hsiClk)
{
    int32_t     retVal;
    rlDevHsiClk_t                           hsiClkgs;

    memset ((void*)&hsiClkgs, 0, sizeof(rlDevHsiClk_t));
    /* Setup the HSI Clock as per the Radar Interface Document:
     * - This is set to 600Mhz DDR Mode */
    hsiClkgs.hsiClk = hsiClk;
    /* Set HSI clock */
    retVal = rlDeviceSetHsiClk(RL_DEVICE_MAP_INTERNAL_BSS, (rlDevHsiClk_t*)&hsiClkgs);
    if (retVal != 0)
    {
        printf ("Error: Unable to set HSI clock [Error %d]\n", retVal);
    }
    printf("Debug: Set HSI clock successfully\n");

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test RF Init/Calibration API.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_rfCalibration (void)
{
    int32_t     retVal;

    retVal = rlRfInit(RL_DEVICE_MAP_INTERNAL_BSS);
    if (retVal != 0)
    {
        printf ("Error: Unable to start RF [Error %d]\n", retVal);
    }
    printf("Debug: RF start successfully\n");

    return (int32_t)retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set Profile configuration.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setProfileConfig (rlProfileCfg_t *pProfileCfg, uint16_t cnt)
{
    int32_t         retVal;
   
    retVal = rlSetProfileConfig(RL_DEVICE_MAP_INTERNAL_BSS, cnt, (rlProfileCfg_t*)pProfileCfg);

    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
    printf("Error: rlSetProfileConfig retVal=%d\n", retVal);
        return -1;
    }
    
    printf("Debug: Finished rlSetProfileConfig\n");
    
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set Chirp configuration.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setChirpConfig (rlChirpCfg_t *pChirpCfg, uint16_t cnt)
{
    int32_t         retVal;

    retVal = rlSetChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, cnt, (rlChirpCfg_t*)pChirpCfg);
           
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlSetChirpConfig retVal=%d\n", retVal);
    }
    
    printf("Debug: Finished rlSetChirpConfig\n");
    
    return retVal;
}

rlChirpCfg_t getChirpCfg[10] = {0};

int32_t MmwaveLink_getChirpConfig (uint16_t startIdx, uint16_t endIdx)
{
    int32_t retVal;

    retVal = rlGetChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, startIdx, endIdx, &getChirpCfg[0]);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlGetChirpConfig retVal=%d\n", retVal);
        return -1;
    }
    
    printf("Debug: Finished rlGetChirpConfig\n");
    
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set Frame configuration.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setFrameConfig (rlFrameCfg_t *frameCfg)
{
    int32_t         retVal;
    
    retVal = rlSetFrameConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlFrameCfg_t*)frameCfg);
           
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlSetFrameConfig retVal=%d\n", retVal);
        return -1;
    }
    
    printf("Debug: Finished rlSetFrameConfig\n");
    
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to test start sensor API.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_sensorStart (void)
{
    int32_t     retVal;

    /* Trigger the frame */
    retVal = rlSensorStart(RL_DEVICE_MAP_INTERNAL_BSS);
    if (retVal != 0)
    {
        printf ("Error: Unable to start Sensor [Error %d]\n", retVal);
    }
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is used to test stop sensor API.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_sensorStop (void)
{
    int32_t     retVal;

    /* Stop the frame */
    retVal = rlSensorStop(RL_DEVICE_MAP_INTERNAL_BSS);
    if (retVal != 0)
    {
        if(retVal == RL_RET_CODE_FRAME_ALREADY_ENDED)
        {
            printf ("Debug: Frames are already stopped  [%d]\n", retVal);
            return 0;
        }
        printf ("Error: Unable to stop Sensor [Error %d]\n", retVal);
        return -1;
    }

    printf("Debug: Sensor stop successfully\n");

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set calib/monitoring configuration
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setCalMonConfig (uint16_t freqLimitLow, uint16_t freqLimitHigh)
{
    int32_t         retVal;
    rlRfCalMonTimeUntConf_t     timeCfg;
    rlRfCalMonFreqLimitConf_t   freqLimit;

    memset ((void *)&freqLimit,   0, sizeof(rlRfCalMonFreqLimitConf_t));
    memset ((void *)&timeCfg,     0, sizeof(rlRfCalMonTimeUntConf_t));
    /****************************************************************************************
     * Setup the RF Calibration Time unit:
     * - Periodicity is set to 1 Frame
     ****************************************************************************************/
    timeCfg.calibMonTimeUnit = 1U;

    retVal = rlRfSetCalMonTimeUnitConfig(RL_DEVICE_MAP_INTERNAL_BSS, 
                                     (rlRfCalMonTimeUntConf_t*)&timeCfg);
    if (retVal != 0)
    {
        printf ("Error: Unable to rlRfSetCalMonTimeUnitConfig [Error %d]\n", retVal);
        return retVal;
    }

    /****************************************************************************************
     * Setup the RF Calibration Frequency limit:
     ****************************************************************************************/
    freqLimit.freqLimitLow  = freqLimitLow;
    freqLimit.freqLimitHigh = freqLimitHigh;

    retVal = rlRfSetCalMonFreqLimitConfig(RL_DEVICE_MAP_INTERNAL_BSS, 
                                     (rlRfCalMonFreqLimitConf_t*)&freqLimit);

    if (retVal != 0)
    {
        printf ("Error: Unable to rlRfSetCalMonFreqLimitConfig[Error %d]\n", retVal);
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set Init time calibration.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setInitTimeCalibConfig (void)
{
    int32_t         retVal;
    rlRfInitCalConf_t           rfInitCalib;

    memset ((void *)&rfInitCalib, 0, sizeof(rlRfInitCalConf_t));
    /****************************************************************************************
     * Enable calibrations:
     ****************************************************************************************/
    rfInitCalib.calibEnMask = CSL_FMKR (4U, 4U, 1U)     |   /* LODIST calibration            */
                              CSL_FMKR (5U, 5U, 1U)     |   /* RX ADC DC offset calibration  */
                              CSL_FMKR (6U, 6U, 1U)     |   /* HPF cutoff calibration        */
                              CSL_FMKR (7U, 7U, 1U)     |   /* LPF cutoff calibration        */
                              CSL_FMKR (8U, 8U, 1U)     |   /* Peak detector calibration     */
                              CSL_FMKR (9U, 9U, 1U)     |   /* TX Power calibration          */
                              CSL_FMKR (10U, 10U, 1U);      /* RX gain calibration           */

    retVal = rlRfInitCalibConfig(RL_DEVICE_MAP_INTERNAL_BSS, 
                                (rlRfInitCalConf_t*)&rfInitCalib);

    if (retVal != 0)
    {
        printf ("Error: Unable to rlRfInitCalibConfig [Error %d]\n", retVal);
    }

    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is used to set run time calibration.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRunTimeCalibConfig (uint32_t calibPeriodicity)
{
    int32_t         retVal;
    rlRunTimeCalibConf_t    runTimeCalib;

    /* Initialize the runtime calibration configuration: */
    memset ((void *)&runTimeCalib, 0, sizeof(rlRunTimeCalibConf_t));

    /* Enable calibration: Always enable one time calibration and all calibrations are enabled */
    runTimeCalib.reportEn           = 0x0; //TODO JIT may need to enable this later
    runTimeCalib.oneTimeCalibEnMask = CSL_FMKR (4U,  4U,  1U) | /* LODIST calibration   */
                                      CSL_FMKR (9U,  9U,  1U) | /* TX Power calibration */
                                      CSL_FMKR (10U, 10U, 1U);  /* RX gain calibration  */

    runTimeCalib.periodicCalibEnMask = runTimeCalib.oneTimeCalibEnMask;
    runTimeCalib.calibPeriodicity    = calibPeriodicity;

    retVal = rlRfRunTimeCalibConfig(RL_DEVICE_MAP_INTERNAL_BSS, 
                                (rlRunTimeCalibConf_t*)&runTimeCalib);

    if (retVal != 0)
    {
        printf ("Error: Unable to rlRfRunTimeCalibConfig [Error %d]\n", retVal);
    }

//    while(gRunTimeCalibStatus == 0U)
//    {
//        /* Sleep and poll again: */
//        asm(" IDLE ");
//    }
//    gRunTimeCalibStatus = 0U;

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set advance Frame configuration.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setAdvFrameConfig (rlAdvFrameCfg_t *advFrameCfg)
{
    int32_t         retVal;
    
    retVal = rlSetAdvFrameConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlAdvFrameCfg_t*)advFrameCfg);

    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlSetAdvFrameConfig retVal=%d\n", retVal);
        return -1;
    }
    
    printf("Debug: Finished rlSetAdvFrameConfig\n");
    
    return 0;
}


/**
 *  @b Description
 *  @n
 *      The function is used to set Binary Phase Modulation Common configuration.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setBpmCommonConfig (void)
{
    int32_t         retVal;
    rlBpmCommonCfg_t        bpmCommonCfg;

    /* Select source of BPM pattern to be from BPM chirp cfg defined in bpmChirpCfg*/
    memset ((void *)&bpmCommonCfg, 0, sizeof(rlBpmCommonCfg_t));
    bpmCommonCfg.mode.b2SrcSel = 0U;
    
    retVal = rlSetBpmCommonConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlBpmCommonCfg_t*)&bpmCommonCfg);

    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
    printf("Error: rlSetBpmCommonConfig retVal=%d\n", retVal);
        return -1;
    }
    
    printf("Debug: Finished rlSetBpmCommonConfig\n");
    
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set Binary Phase Modulation Chirp configuration.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setBpmChirpConfig (rlBpmChirpCfg_t *pBpmChirpCfg)
{
    int32_t         retVal;
    
    retVal = rlSetBpmChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlBpmChirpCfg_t*)pBpmChirpCfg);

    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlSetBpmChirpConfig retVal=%d\n", retVal);
        return -1;
    }
    
    printf("Debug: Finished rlSetBpmChirpConfig\n");
    
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set Continous mode Configuration.
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setContModeConfig (rlContModeCfg_t *contModeCfg)
{
    int32_t         retVal;

    
    retVal = rlSetContModeConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlContModeCfg_t*)&contModeCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
    printf("Error: rlSetContModeConfig retVal=%d\n", retVal);
        return -1;
    }
    
    printf("Debug: Finished rlSetContModeConfig\n");
    
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to Enable/Disable Continous mode
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_enableContMode (uint8_t bEnable)
{
    int32_t         retVal;
    rlContModeEn_t contModeEnable;

    contModeEnable.contModeEn = bEnable;
    retVal = rlEnableContMode(RL_DEVICE_MAP_INTERNAL_BSS, (rlContModeEn_t*)&contModeEnable);

    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlEnableContMode retVal=%d\n", retVal);
        return -1;
    }
    
    printf("Debug: Finished rlEnableContMode\n");
    
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to Configure asynchronous event direction for device
 *      using the mmWave link API.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfDevCfg (rlUInt32_t dirData)
{
    int32_t         retVal;

    rfDevCfg.aeDirection = dirData; /* 0xa: BSS to DSS, 0: BSS to MSS */

    retVal = rlRfSetDeviceCfg(RL_DEVICE_MAP_INTERNAL_BSS, &rfDevCfg);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfSetDeviceCfg retVal=%d\n", retVal);
    }
    
    printf("Debug: Finished rlRfSetDeviceCfg\n");
    
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function Sets the consolidated configuration of CQ & CP monitoring If analog monitors
 *      already enabled then function enable analog monitors(those enabled), CQ & CP monitors .
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwaveLink_setRfAnaMonConfigCq (void)
{
    int32_t  retVal;
    /* Each bit(0-31) represents different analog monitor enable configuration */
    rlMonAnaEnables_t data = { (0 << 0)   | \
                               (0 << 1)   | \
                               (0 << 2)   | \
                               (0 << 3)   | \
                               (0 << 4)   | \
                               (0 << 5)   | \
                               (0 << 6)   | \
                               (0 << 7)   | \
                               (0 << 8)   | \
                               (0 << 9)   | \
                               (0 << 10)  | \
                               (0 << 11)  | \
                               (0 << 12)  | \
                               (0 << 13)  | \
                               (0 << 14)  | \
                               (0 << 15)  | \
                               (0 << 16)  | \
                               (0 << 17)  | \
                               (0 << 18)  | \
                               (0 << 19)  | \
                               (0 << 20)  | \
                               (0 << 21)  | \
                               (0 << 22)  | \
                               (0 << 23)  | \
                               (1 << 24)  | \
                               (1 << 25)  | \
                               (0 << 26)  | \
                               (0 << 27)  | \
                               (0 << 28)  | \
                               (0 << 29)  | \
                               (0 << 30)  | \
                               (0 << 31)
                                ,0x0};
	data.enMask |= anaMonEn;
	anaMonEn = data.enMask;
    /* Set channel configuration */
    retVal = rlRfAnaMonConfig(RL_DEVICE_MAP_INTERNAL_BSS,&data);
    /* Check for mmWaveLink API call status */
    if(retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlRfAnaMonConfig retVal=%d\n", retVal);
        return -1;
    }
    printf("Debug: Finished rlRfAnaMonConfig configurations to BSS\n");
    return 0;    
} 

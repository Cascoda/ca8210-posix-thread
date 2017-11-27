/*
 *  Copyright (c) 2016, Nest Labs, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the OpenThread platform abstraction for radio
 *   communication.
 *
 */

#include <types.h>
#include <openthread.h>
#include <thread_ftd.h>

#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <assert.h>
#include <common/code_utils.hpp>
#include <platform/radio.h>
#include <platform/random.h>
#include <platform/logging.h>
#include <ca821x_api.h>
#include <ca821x-posix.h>
#include <string.h>
#include <mac_messages.h>
#include "posix-platform.h"
#include <ieee_802_15_4.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/stat.h>

#include "selfpipe.h"

#define ARRAY_LENGTH(array) (sizeof((array))/sizeof((array)[0]))

//BARRIER
/*
 * The following functions create a thread safe system for allowing the worker
 * thread to access openthread functions safely. The main thread always has
 * priority and must explicitly give control to the worker thread while locking
 * itself. This has ONLY been designed to work with ONE worker and ONE main, in
 * a way that causes the worker thread to run one operation per poll cycle as
 * openthread was designed for.
 */
static pthread_mutex_t barrier_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t barrier_cond = PTHREAD_COND_INITIALIZER;

static enum barrier_waiting
{
	NOT_WAITING, WAITING, GREENLIGHT, DONE
} mbarrier_waiting;

static inline void barrier_main_letWorkerWork(void);
static inline void barrier_worker_waitForMain(void);
static inline void barrier_worker_endWork(void);
//END BARRIER

#define IEEEEUI_FILE "/usr/local/etc/.otEui"
static uint8_t sIeeeEui64[8];

//For cascoda API
static struct ca821x_dev s_pDeviceRef;
static struct ca821x_dev *pDeviceRef = NULL;

static int8_t sNoiseFloor = 127;

static otInstance *OT_INSTANCE = NULL;
static uint8_t sRadioInitialised = 0;

struct M_KeyDescriptor_thread
{
	struct M_KeyTableEntryFixed    Fixed;
	struct M_KeyIdLookupDesc       KeyIdLookupList[1];
	uint8_t                        flags[20]; //TODO: Provisional length
	//struct M_KeyDeviceDesc         KeyDeviceList[count];
	//struct M_KeyUsageDesc          KeyUsageList[2];
};

otError otPlatMlmeGet(otInstance *aInstance, otPibAttr aAttr, uint8_t aIndex, uint8_t *aLen, uint8_t *aBuf)
{
	uint8_t error;
	otError otErr;

	//Adaption for security table
	if(aAttr == OT_PIB_MAC_KEY_TABLE)
	{
		struct M_KeyDescriptor_thread caKeyDesc;
		otKeyTableEntry *otKeyDesc = (otKeyTableEntry*) aBuf;
		uint8_t flagOffset = 0;

		error = MLME_GET_request_sync(aAttr,
		                              aIndex,
		                              aLen,
		                              (uint8_t*)(&caKeyDesc),
		                              pDeviceRef);

		//Convert to ot format
		otKeyDesc->mKeyIdLookupListEntries = caKeyDesc.Fixed.KeyIdLookupListEntries;
		otKeyDesc->mKeyDeviceListEntries   = caKeyDesc.Fixed.KeyDeviceListEntries;
		otKeyDesc->mKeyUsageListEntries    = caKeyDesc.Fixed.KeyUsageListEntries;

		VerifyOrExit(otKeyDesc->mKeyIdLookupListEntries <= ARRAY_LENGTH(otKeyDesc->mKeyIdLookupDesc), otErr = OT_ERROR_GENERIC);
		VerifyOrExit(otKeyDesc->mKeyDeviceListEntries <= ARRAY_LENGTH(otKeyDesc->mKeyDeviceDesc), otErr = OT_ERROR_GENERIC);
		VerifyOrExit(otKeyDesc->mKeyUsageListEntries <= ARRAY_LENGTH(otKeyDesc->mKeyUsageDesc), otErr = OT_ERROR_GENERIC);

		memcpy(otKeyDesc->mKey, caKeyDesc.Fixed.Key, sizeof(otKeyDesc->mKey));
		otKeyDesc->mKeyIdLookupDesc[0] =
				*((struct otKeyIdLookupDesc*) &(caKeyDesc.KeyIdLookupList[0]));

		for(int i = 0; i < otKeyDesc->mKeyDeviceListEntries; i++, flagOffset++)
		{
			otKeyDesc->mKeyDeviceDesc[i].mDeviceDescriptorHandle =
					caKeyDesc.flags[flagOffset] & KDD_DeviceDescHandleMask;
			otKeyDesc->mKeyDeviceDesc[i].mUniqueDevice =
					!!(caKeyDesc.flags[flagOffset] & KDD_UniqueDeviceMask);
			otKeyDesc->mKeyDeviceDesc[i].mBlacklisted =
					!!(caKeyDesc.flags[flagOffset] & KDD_BlacklistedMask);
		}

		for(int i = 0; i < otKeyDesc->mKeyUsageListEntries; i++, flagOffset++)
		{
			uint8_t flag = caKeyDesc.flags[flagOffset];
			otKeyDesc->mKeyUsageDesc[i].mFrameType =
					flag & KUD_FrameTypeMask;
			otKeyDesc->mKeyUsageDesc[i].mCommandFrameId =
					(flag & KUD_CommandFrameIdentifierMask) >> KUD_CommandFrameIdentifierShift;
		}

		*aLen = sizeof(otKeyTableEntry);
	}
	else
	{
		error = MLME_GET_request_sync(aAttr,
		                              aIndex,
		                              aLen,
		                              aBuf,
		                              pDeviceRef);
	}

	switch ( error )
	{
	case MAC_SUCCESS:
		otErr = OT_ERROR_NONE;
		break;

	case MAC_UNSUPPORTED_ATTRIBUTE:
	case MAC_INVALID_INDEX:
		otErr = OT_ERROR_INVALID_ARGS;
		break;

	default:
		otErr = OT_ERROR_GENERIC;
	}

exit:
	return otErr;
}

otError otPlatMlmeSet(otInstance *aInstance, otPibAttr aAttr, uint8_t aIndex, uint8_t aLen, const uint8_t *aBuf){
	uint8_t error;
	otError otErr;

	//Adaption for security table
	if(aAttr == OT_PIB_MAC_KEY_TABLE)
	{
		struct M_KeyDescriptor_thread caKeyDesc;
		const otKeyTableEntry *otKeyDesc = (otKeyTableEntry*) aBuf;
		uint8_t flagOffset = 0;

		caKeyDesc.Fixed.KeyIdLookupListEntries = otKeyDesc->mKeyIdLookupListEntries;
		caKeyDesc.Fixed.KeyDeviceListEntries = otKeyDesc->mKeyDeviceListEntries;
		caKeyDesc.Fixed.KeyUsageListEntries = otKeyDesc->mKeyUsageListEntries;

		memcpy(caKeyDesc.Fixed.Key, otKeyDesc->mKey, sizeof(otKeyDesc->mKey));
		caKeyDesc.KeyIdLookupList[0] =
				*((struct M_KeyIdLookupDesc*) &(otKeyDesc->mKeyIdLookupDesc[0]));

		for(int i = 0; i < otKeyDesc->mKeyDeviceListEntries; i++, flagOffset++)
		{
			otKeyDeviceDesc *devDesc = &(otKeyDesc->mKeyDeviceDesc[i]);
			caKeyDesc.flags[flagOffset] =
					devDesc->mDeviceDescriptorHandle & KDD_DeviceDescHandleMask;
			caKeyDesc.flags[flagOffset] |=
					devDesc->mUniqueDevice ? KDD_UniqueDeviceMask : 0;
			caKeyDesc.flags[flagOffset] |=
					devDesc->mBlacklisted ? KDD_BlacklistedMask : 0;
		}

		for(int i = 0; i < otKeyDesc->mKeyUsageListEntries; i++, flagOffset++)
		{
			otKeyUsageDesc * useDesc = &(otKeyDesc->mKeyUsageDesc[i]);
			uint8_t flag;

			flag = useDesc->mFrameType & KUD_FrameTypeMask;
			flag |= ((useDesc->mCommandFrameId << KUD_CommandFrameIdentifierShift)
					& KUD_CommandFrameIdentifierMask );

			caKeyDesc.flags[flagOffset] = flag;
		}

		aLen = sizeof(caKeyDesc) + flagOffset - sizeof(caKeyDesc.flags);
		error = MLME_SET_request_sync(aAttr,
		                              aIndex,
		                              aLen,
		                              (uint8_t*)(&caKeyDesc),
		                              pDeviceRef);
	}
	else
	{
		error = MLME_SET_request_sync(aAttr,
		                              aIndex,
		                              aLen,
		                              aBuf,
		                              pDeviceRef);
	}

	switch ( error )
	{
	case MAC_SUCCESS:
		otErr = OT_ERROR_NONE;
		break;

	case MAC_READ_ONLY:
		otErr = OT_ERROR_NOT_CAPABLE;
		break;

	case MAC_INVALID_PARAMETER:
	case MAC_UNSUPPORTED_ATTRIBUTE:
	case MAC_INVALID_INDEX:
		otErr = OT_ERROR_INVALID_ARGS;
		break;

	default:
		otErr = OT_ERROR_GENERIC;
	}

	return otErr;
}

otError otPlatMlmeReset(otInstance *aInstance, bool setDefaultPib)
{
	uint8_t error;

	error = MLME_RESET_request_sync(setDefaultPib, pDeviceRef);

	if(setDefaultPib)
	{
		//Disable low LQI rejection @ MAC Layer
		uint8_t disable = 0;
		HWME_SET_request_sync(0x11, 1, &disable, pDeviceRef);

		//LQI values should be derived from receive energy
		uint8_t LQImode = HWME_LQIMODE_ED;
		HWME_SET_request_sync(HWME_LQIMODE, 1, &LQImode, pDeviceRef);
	}

	return error == MAC_SUCCESS ? OT_ERROR_NONE : OT_ERROR_FAILED;
}

otError otPlatMlmeStart(otInstance *aInstance, otStartRequest *aStartReq)
{
	uint8_t error;
	otError otErr;

	error = MLME_START_request_sync(aStartReq->mPanId,
	                                aStartReq->mLogicalChannel,
	                                aStartReq->mBeaconOrder,
	                                aStartReq->mSuperframeOrder,
	                                aStartReq->mPanCoordinator,
	                                aStartReq->mBatteryLifeExtension,
	                                aStartReq->mCoordRealignment,
	             (struct SecSpec*)  &(aStartReq->mCoordRealignSecurity),
	             (struct SecSpec*)  &(aStartReq->mBeaconSecurity),
	                                pDeviceRef);

	switch ( error )
	{
	case MAC_SUCCESS:
		otErr = OT_ERROR_NONE;
		break;

	case MAC_NO_SHORT_ADDRESS:
	case MAC_UNAVAILABLE_KEY:
		otErr = OT_ERROR_INVALID_STATE;
		break;

	case MAC_INVALID_PARAMETER:
	case MAC_FRAME_TOO_LONG:
		otErr = OT_ERROR_INVALID_ARGS;
		break;

	default:
		otErr = OT_ERROR_GENERIC;
	}

	return otErr;
}

otError otPlatMlmeScan(otInstance *aInstance, otScanRequest *aScanRequest)
{
	uint8_t error;

	error = MLME_SCAN_request(aScanRequest->mScanType,
	                          aScanRequest->mScanChannelMask,
	                          aScanRequest->mScanDuration,
	       (struct SecSpec*)  &(aScanRequest->mSecSpec),
	                          pDeviceRef);

	return error == MAC_SUCCESS ? OT_ERROR_NONE : OT_ERROR_FAILED;
}

otError otPlatMlmePollRequest(otInstance *aInstance, otPollRequest *aPollRequest)
{
	uint8_t error;
	uint8_t pollInterval[2] = {0};

	error = MLME_POLL_request_sync(
	         *((struct FullAddr*)  &(aPollRequest->mCoordAddress)),
	                               pollInterval,
	            (struct SecSpec*)  &(aPollRequest->mSecurity),
	                               pDeviceRef);

	return (error == MAC_SUCCESS || error == MAC_NO_DATA) ? OT_ERROR_NONE : OT_ERROR_NO_ACK;
}

otError otPlatMcpsDataRequest(otInstance *aInstance, otDataRequest *aDataRequest)
{
	uint8_t error;

	error = MCPS_DATA_request(aDataRequest->mSrcAddrMode,
	                          aDataRequest->mDst.mAddressMode,
	                          GETLE16(aDataRequest->mDst.mPanId),
	        (union MacAddr*)  aDataRequest->mDst.mAddress,
	                          aDataRequest->mMsduLength,
	                          aDataRequest->mMsdu,
	                          aDataRequest->mMsduHandle,
	                          aDataRequest->mTxOptions,
	       (struct SecSpec*)  &(aDataRequest->mSecurity),
	                          pDeviceRef);

	return (error == MAC_SUCCESS) ? OT_ERROR_NONE : OT_ERROR_INVALID_STATE;
}

otError otPlatMcpsPurge(otInstance *aInstance, uint8_t aMsduHandle)
{
	uint8_t error;

	error = MCPS_PURGE_request_sync(&aMsduHandle, pDeviceRef);

	return (error == MAC_SUCCESS) ? OT_ERROR_NONE : OT_ERROR_ALREADY;
}

static int handleDataIndication(struct MCPS_DATA_indication_pset *params, struct ca821x_dev *pDeviceRef)
{
	int16_t rssi;
	//TODO: Move this off the stack
	otDataIndication dataInd = {0};

	dataInd.mSrc = *((struct otFullAddr*) &(params->Src));
	dataInd.mDst = *((struct otFullAddr*) &(params->Dst));
	dataInd.mMsduLength = params->MsduLength;
	rssi = ((int16_t) params->MpduLinkQuality - 256)/2; //convert to rssi
	dataInd.mMpduLinkQuality = rssi;
	dataInd.mDSN = params->DSN;
	memcpy(dataInd.mMsdu, params->Msdu, dataInd.mMsduLength);
	memcpy(&(dataInd.mSecurity), params->Msdu + params->MsduLength, sizeof(dataInd.mSecurity));

	if(dataInd.mSecurity.mSecurityLevel == 0)
	{
		memset(&(dataInd.mSecurity), 0, sizeof(dataInd.mSecurity));
	}

	barrier_worker_waitForMain();
	otPlatMcpsDataIndication(OT_INSTANCE, &dataInd);
	barrier_worker_endWork();

	return 1;
}

static int handleDataConfirm(struct MCPS_DATA_confirm_pset *params, struct ca821x_dev *pDeviceRef)   //Async
{
	barrier_worker_waitForMain();
	otPlatMcpsDataConfirm(OT_INSTANCE, params->MsduHandle, params->Status);
	barrier_worker_endWork();

	return 1;
}

static int handleBeaconNotify(struct MLME_BEACON_NOTIFY_indication_pset *params, struct ca821x_dev *pDeviceRef) //Async
{
	//TODO: Move this off the stack
	otBeaconNotify beaconNotify = {0};
	uint8_t sduLenOffset;

	{
		uint8_t addrField = ((uint8_t *)params)[23];
		uint8_t shortaddrs  = addrField & 0x07;
		uint8_t extaddrs = (addrField >> 4) & 0x07;
		sduLenOffset = (24 + (2 * shortaddrs) + (8 * extaddrs));
	}

	beaconNotify.BSN = params->BSN;
	beaconNotify.mPanDescriptor = *((struct otPanDescriptor*) &(params->PanDescriptor));
	beaconNotify.mSduLength = ((uint8_t *)params)[sduLenOffset];
	memcpy(beaconNotify.mSdu, &(((uint8_t *)params)[sduLenOffset + 1]), beaconNotify.mSduLength);

	barrier_worker_waitForMain();
	otPlatMlmeBeaconNotifyIndication(OT_INSTANCE, &beaconNotify);
	barrier_worker_endWork();

	return 1;
}

static int handleScanConfirm(struct MLME_SCAN_confirm_pset *params, struct ca821x_dev *pDeviceRef)   //Async
{
	barrier_worker_waitForMain();
	otPlatMlmeScanConfirm(OT_INSTANCE, (otScanConfirm *)params);
	barrier_worker_endWork();

	return 1;
}

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
	memcpy(aIeeeEui64, sIeeeEui64, sizeof(sIeeeEui64));

	(void) aInstance;
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance){
	return -105;
}

static int driverErrorCallback(int error_number)
{
	otPlatLog(OT_LOG_LEVEL_CRIT, OT_LOG_REGION_HARDMAC, "DRIVER FAILED WITH ERROR %d\n\r", error_number);

	if(!sRadioInitialised)
		exit(EXIT_FAILURE);

	otPlatLog(OT_LOG_LEVEL_CRIT, OT_LOG_REGION_HARDMAC, "Attempting restart...\n\r", error_number);

	if(ca821x_util_reset(pDeviceRef) == 0){
		otThreadSetAutoStart(OT_INSTANCE, true);
		otInstanceReset(OT_INSTANCE);
	}

	abort();
	return 0;
}

void PlatformRadioStop(void)
{
	if(sRadioInitialised){
		//Reset the MAC to a default state
		otPlatLog(OT_LOG_LEVEL_INFO, OT_LOG_REGION_HARDMAC, "Resetting & Stopping Radio...\n\r");
		MLME_RESET_request_sync(1, pDeviceRef);
		ca821x_util_deinit(pDeviceRef);
		sRadioInitialised = false;
	}
}

void initIeeeEui64(){
	int file;
	uint8_t create = false;
	size_t fileNameLen = strlen(IEEEEUI_FILE) + 4; //"filename.00\0"
	char fileName[fileNameLen];

	snprintf(fileName, fileNameLen, "%s.%02u", IEEEEUI_FILE, NODE_ID);

	if (!access(fileName, R_OK))
	{
		uint8_t ret = 0;

		file = open(fileName, O_RDONLY);
		ret = read(file, sIeeeEui64, 8);
		if(ret != 8)
		{
			close(file);
			create = true;
		}
	}
	else
	{
		create = true;
	}

	if(create)
	{
		file = open(fileName, O_RDWR | O_CREAT, 0666);
		for (int i = 0; i < 4; i += 1)
		{
			uint16_t random = otPlatRandomGet();
			sIeeeEui64[2 * i] = random & 0xFF;
			sIeeeEui64[2 * i + 1] = (random >> 4) & 0xFF;
		}
		sIeeeEui64[0] &= ~1; //Unset Group bit
		sIeeeEui64[0] |= 2; //Set local bit
		write(file, sIeeeEui64, 8);
	}
	close(file);
}

int PlatformRadioInitWithDev(struct ca821x_dev *apDeviceRef)
{
	pDeviceRef = apDeviceRef;

	atexit(&PlatformRadioStop);
	selfpipe_init();

	struct ca821x_api_callbacks callbacks = {0};
	callbacks.MCPS_DATA_indication = &handleDataIndication;
	callbacks.MCPS_DATA_confirm = &handleDataConfirm;
	callbacks.MLME_BEACON_NOTIFY_indication = &handleBeaconNotify;
	callbacks.MLME_SCAN_confirm = &handleScanConfirm;
	ca821x_register_callbacks(&callbacks, pDeviceRef);

	//Reset the MAC to a default state
	otPlatMlmeReset(NULL, true);

	initIeeeEui64();
	sRadioInitialised = 1;

	return 0;
}

int PlatformRadioInit(void)
{
	int status;

	status = ca821x_util_init(&s_pDeviceRef, driverErrorCallback);
	if(status < 0)
		return status;

	return PlatformRadioInitWithDev(&s_pDeviceRef);
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
	return sNoiseFloor;
}

otError otPlatRadioEnable(otInstance *aInstance)
{
	otError error = OT_ERROR_NONE;
	OT_INSTANCE = aInstance;

	return error;
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
	return (OT_INSTANCE != NULL);
}

int PlatformRadioProcess(void)
{
	barrier_main_letWorkerWork();
	return 0;
}

//Lets the worker thread work synchronously if there is synchronous work to do
static inline void barrier_main_letWorkerWork()
{
	pthread_mutex_lock(&barrier_mutex);

	if (mbarrier_waiting == WAITING)
	{
		mbarrier_waiting = GREENLIGHT;
		pthread_cond_broadcast(&barrier_cond);

		while (mbarrier_waiting != DONE)
		{
			pthread_cond_wait(&barrier_cond, &barrier_mutex); //The worker thread does work now
		}
	}

	mbarrier_waiting = NOT_WAITING;
	pthread_cond_broadcast(&barrier_cond);
	pthread_mutex_unlock(&barrier_mutex);
}

static inline void barrier_worker_waitForMain()
{
	pthread_mutex_lock(&barrier_mutex);

	while (mbarrier_waiting != NOT_WAITING)
	{
		pthread_cond_wait(&barrier_cond, &barrier_mutex);
	}

	selfpipe_push();
	mbarrier_waiting = WAITING;
	pthread_cond_broadcast(&barrier_cond);

	while (mbarrier_waiting != GREENLIGHT)
	{
		pthread_cond_wait(&barrier_cond, &barrier_mutex); //wait for the main thread to signal worker to run
	}
}

static inline void barrier_worker_endWork()
{
	mbarrier_waiting = DONE;
	pthread_cond_broadcast(&barrier_cond);
	pthread_mutex_unlock(&barrier_mutex);
}

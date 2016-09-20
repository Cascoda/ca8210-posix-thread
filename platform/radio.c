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
 *   This file implements the OpenThread platform abstraction for radio communication.
 *
 */

#include <openthread-types.h>
#include <openthread.h>

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <assert.h>
#include <common/code_utils.hpp>
#include <platform/radio.h>
#include <platform/logging.h>
#include <cascoda_api.h>
#include <kernel_exchange.h>
#include <string.h>
#include <mac_messages.h>
#include "posix-platform.h"
#include <ieee_802_15_4.h>
#include <pthread.h>

#include "selfpipe.h"


//Mac tools
#define MAC_SC_SECURITYLEVEL(sc) (sc&0x07)
#define MAC_SC_KEYIDMODE(sc) ((sc>>3)&0x03)
#define MAC_KEYIDMODE_SC(keyidmode) ((keyidmode&0x03)<<3)
#define MAC_BASEHEADERLENGTH 3

enum
{
    IEEE802154_MIN_LENGTH = 5,
    IEEE802154_MAX_LENGTH = 127,
    IEEE802154_ACK_LENGTH = 5,
    IEEE802154_FRAME_TYPE_MASK = 0x7,
    IEEE802154_FRAME_TYPE_ACK = 0x2,
    IEEE802154_FRAME_PENDING = 1 << 4,
    IEEE802154_ACK_REQUEST = 1 << 5,
    IEEE802154_DSN_OFFSET = 2,
};

struct DeviceCache {
	uint8_t			isActive;
	otExtAddress 	mExtAddr;					//Extended address of the device
	uint8_t 		mFrameCounter[4];			//The most up to date frame counter from the device table
	uint8_t 		mTimeoutFrameCounter[4];	//The previously polled version of the frame counter
};

static struct DeviceCache sDeviceCache[DEVICE_TABLE_SIZE] = {0};

static uint8_t curDeviceTableSize = 0;

int PlatformRadioSignal(void);

int readFrame(struct MCPS_DATA_indication_pset *params);
int readConfirmFrame(struct MCPS_DATA_confirm_pset *params);

int beaconNotifyFrame(struct MLME_BEACON_NOTIFY_indication_pset *params);
int scanConfirmCheck(struct MLME_SCAN_confirm_pset *params);
int genericDispatchFrame(const uint8_t *buf, size_t len);

void keyChangedCallback(uint32_t aFlags, void *aContext);
void coordChangedCallback(uint32_t aFlags, void *aContext);

//DEVICE FRAME COUNTER CACHING
static void cacheDevices();
static struct DeviceCache * getCachedDevice(otExtAddress addr);

//BARRIER
static pthread_mutex_t barrierMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t barrierCondVar = PTHREAD_COND_INITIALIZER;
static enum barrier_waiting {NOT_WAITING, WAITING, GREENLIGHT, DONE} mbarrier_waiting;

static inline void barrier_main_letWorkerWork(void);
static inline void barrier_worker_waitForMain(void);
static inline void barrier_worker_endWork(void);

//INTRANSIT
static RadioPacket * getIntransitFrame(uint8_t handle);	//Return pointer to the relevant packet, or NULL
static int rmIntransitFrame(uint8_t handle);	//Release dynamically allocated memory -- returns 0 for success, or an error code
static int putIntransitFrame(uint8_t handle, const RadioPacket * in, uint8_t headerSize);	//returns 0 for success, or an error code
static int isHandleInUse(uint8_t handle); //returns 0 if the handle is free, or 1 if it is already in use

#define MAX_INTRANSITS 7 /*TODO:  5 indirect frames +2 (perhaps need more?)*/
uint8_t IntransitHandles[MAX_INTRANSITS] = {0};
RadioPacket IntransitPackets[MAX_INTRANSITS];
static pthread_mutex_t intransit_mutex = PTHREAD_MUTEX_INITIALIZER;

static RadioPacket sTransmitFrame;
static RadioPacket sReceiveFrame;
static ThreadError sTransmitError;
static ThreadError sReceiveError = kThreadError_None;

static otHandleActiveScanResult scanCallback;
static void * scanContext;
static uint8_t activeScanInProgress = 0;

static void* pDeviceRef = NULL;

static uint8_t sChannel = 0;

//TODO: make this an enum
enum tristateCache {tristate_disabled = 0, tristate_enabled = 1, tristate_uninit = 2};
static enum tristateCache promiscuousCache = tristate_uninit;

static uint8_t isCoord = 0;

static uint8_t sTransmitPsdu[IEEE802154_MAX_LENGTH];
static uint8_t sReceivePsdu[IEEE802154_MAX_LENGTH];

static uint8_t mBeaconPayload[32] = {3, 0x91};
static uint8_t payloadLength = 32;

static int8_t noiseFloor = 127;

pthread_mutex_t receiveFrame_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t receiveFrame_cond = PTHREAD_COND_INITIALIZER;

static PhyState sState;

void setChannel(uint8_t channel)
{
    if(sChannel != channel){
    	MLME_SET_request_sync(
    			phyCurrentChannel,
    	        0,
    	        sizeof(channel),
    	        &channel,
    	        pDeviceRef);
        sChannel = channel;
    }
}

ThreadError otActiveScan(uint32_t aScanChannels, uint16_t aScanDuration, otHandleActiveScanResult aCallback, void *aCallbackContext)
{

	/*
	 * This function causes an active scan to be run by the ca8210
	 */

	uint8_t ScanDuration = 5; //0 to 14
	struct SecSpec pSecurity = {0};
	if (aScanChannels == 0) aScanChannels = 0x07fff800; //11 to 26
	scanCallback = aCallback;
	scanContext = aCallbackContext;
	uint8_t scanRequest = MLME_SCAN_request(
			1,
			aScanChannels,
			ScanDuration,
			&pSecurity,
			pDeviceRef);

	if(scanRequest == MAC_SUCCESS) activeScanInProgress = 1;
	return scanRequest;
}

bool otActiveScanInProgress(void)
{
    return activeScanInProgress;
}

ThreadError otPlatSetNetworkName(const char *aNetworkName) {

	memcpy(mBeaconPayload + 2, aNetworkName, 16);
	if ((MLME_SET_request_sync(
			macBeaconPayload,
			0,
			payloadLength,
			mBeaconPayload,
			pDeviceRef) == MAC_SUCCESS) &&
		(MLME_SET_request_sync(
			macBeaconPayloadLength,
			0,
			1,
			&payloadLength,
			pDeviceRef) == MAC_SUCCESS)) {
        return kThreadError_None;
	}
	else return kThreadError_Failed;
}

ThreadError otPlatSetExtendedPanId(const uint8_t *aExtPanId) {

	memcpy(mBeaconPayload + 18, aExtPanId, 8);
	if ((MLME_SET_request_sync(
			macBeaconPayload,
			0,
			payloadLength,
			mBeaconPayload,
			pDeviceRef) == MAC_SUCCESS) &&
		(MLME_SET_request_sync(
			macBeaconPayloadLength,
			0,
			1,
			&payloadLength,
			pDeviceRef) == MAC_SUCCESS)) {
		return kThreadError_None;
	}

	else return kThreadError_Failed;
}

ThreadError otPlatRadioSetPanId(uint16_t panid)
{
	uint8_t LEarray[2];
	LEarray[0] = LS0_BYTE(panid);
	LEarray[1] = LS1_BYTE(panid);
    if ( MLME_SET_request_sync(
        macPANId,
        0,
        sizeof(panid),
        LEarray,
        pDeviceRef) == MAC_SUCCESS)
            return kThreadError_None;

    else return kThreadError_Failed;
}

ThreadError otPlatRadioSetExtendedAddress(uint8_t *address)
{
    if ( MLME_SET_request_sync(
        nsIEEEAddress,
        0,
        OT_EXT_ADDRESS_SIZE, 
        address,
        pDeviceRef) == MAC_SUCCESS){

        return kThreadError_None;
    }

    else return kThreadError_Failed;
}

ThreadError otPlatRadioSetShortAddress(uint16_t address)
{
	uint8_t LEarray[2];
		LEarray[0] = LS0_BYTE(address);
		LEarray[1] = LS1_BYTE(address);
    if ( MLME_SET_request_sync(
        macShortAddress,
        0,
        sizeof(address),
        LEarray,
        pDeviceRef) == MAC_SUCCESS)
            return kThreadError_None;

    else return kThreadError_Failed;
}

static int driverErrorCallback(int error_number){
	otPlatLog(kLogLevelCrit, kLogRegionHardMac, "DRIVER FAILED WITH ERROR %d\n\r", error_number);
	//TODO: Fail gracefully
	abort();
	return 0;
}

void PlatformRadioInit(void)
{
    sTransmitFrame.mLength = 0;
    sTransmitFrame.mPsdu = sTransmitPsdu;
    
    pthread_mutex_lock(&receiveFrame_mutex);
		sReceiveFrame.mLength = 0;
		sReceiveFrame.mPsdu = sReceivePsdu;
		pthread_cond_broadcast(&receiveFrame_cond);
	pthread_mutex_unlock(&receiveFrame_mutex);
    
	selfpipe_init();

    kernel_exchange_init_withhandler(driverErrorCallback);

    struct cascoda_api_callbacks callbacks = {0};
    callbacks.MCPS_DATA_indication = &readFrame;
    callbacks.MCPS_DATA_confirm = &readConfirmFrame;
    callbacks.MLME_BEACON_NOTIFY_indication = &beaconNotifyFrame;
    callbacks.MLME_SCAN_confirm = &scanConfirmCheck;
    callbacks.generic_dispatch = &genericDispatchFrame;	//UNCOMMENT TO ENABLE VIEWING UNHANDLED FRAMES
    cascoda_register_callbacks(&callbacks);
    
    uint8_t enable = 1;	//enable security
	MLME_SET_request_sync(
		macSecurityEnabled,
		0,
		sizeof(enable),
		&enable,
		pDeviceRef);

	//TODO: This should be 3 according to thread spec, but openthread still needs to sort out the higher level retries
	uint8_t retries = 7;	//Retry transmission 7 times if not acknowledged
	MLME_SET_request_sync(
		macMaxFrameRetries,
		0,
		sizeof(retries),
		&retries,
		pDeviceRef);

	retries = 5;	//max 5 CSMA backoffs
	MLME_SET_request_sync(
		macMaxCSMABackoffs,
		0,
		sizeof(retries),
		&retries,
		pDeviceRef);

	uint8_t maxBE = 4;	//max BackoffExponent 4
	MLME_SET_request_sync(
			macMaxBE,
			0,
			sizeof(maxBE),
			&maxBE,
			pDeviceRef);

	uint8_t defaultKeySource[8] = {0, 0, 0, 0, 0, 0, 0, 0xFF};	//set the defaultKeySource as defined in 7.2.2.1 of thread spec
	MLME_SET_request_sync(
		macDefaultKeySource,
		0,
		8,
		defaultKeySource,
		pDeviceRef);

	uint8_t LQImode = HWME_LQIMODE_ED;	//LQI values should be derived from receive energy
	HWME_SET_request_sync(
		HWME_LQIMODE,
		1,
		&LQImode,
		pDeviceRef);

}

void otHardMacStateChangeCallback(uint32_t aFlags, void *aContext){

	/*
	 * This function is called whenever there is an internal state change in thread
	 */

	keyChangedCallback(aFlags, aContext);
	coordChangedCallback(aFlags, aContext);
}

void coordChangedCallback(uint32_t aFlags, void *aContext) {

	/*
	 * This function sets the node as an 802.15.4 coordinator when it is set to act as a thread
	 * router, in order that is can respond to active scan beacon requests with beacons.
	 */

	if(aFlags & OT_NET_ROLE){
		struct SecSpec securityLevel = {0};
		if(otGetDeviceRole() == kDeviceRoleRouter || otGetDeviceRole() == kDeviceRoleLeader){
			if(!isCoord) {
				MLME_START_request_sync(
						otGetPanId(),
						sChannel,
						15,
						15,
						1,
						0,
						0,
						&securityLevel,
						&securityLevel,
						pDeviceRef);
				isCoord = 1;
			}
		} else if (isCoord) {
			MLME_START_request_sync(
					otGetPanId(),
					sChannel,
					15,
					15,
					0,
					0,
					0,
					&securityLevel,
					&securityLevel,
					pDeviceRef);
			isCoord = 0;
		}
	}
}

void keyChangedCallback(uint32_t aFlags, void *aContext){

	/*
	 * This Function updates the keytable and devicetable entries stored on the
	 * ca8210 whenever there is a new key or new device to communicate with.
	 */

	if((aFlags & (OT_NET_KEY_SEQUENCE | OT_THREAD_CHILD_ADDED | OT_THREAD_CHILD_REMOVED | OT_NET_ROLE | OT_THREAD_LINK_ACCEPT))){	//The thrKeySequenceCounter has changed or device descriptors need updating
		//Therefore update the keys stored in the macKeytable
		//TODO: (low priority) Utilise the device cache to speed this up
		otPlatLog(kLogLevelDebg, kLogRegionHardMac, "Updating keys\n\r");
		if(otGetKeySequenceCounter() == 0) otSetKeySequenceCounter(2);
		uint32_t tKeySeq = otGetKeySequenceCounter() - 1;

		uint8_t count = 0;	//Update device list
		if(otGetDeviceRole() != kDeviceRoleChild){
			for(uint8_t i = 0; i < 5; i++){
				otChildInfo tChildInfo;
				otGetChildInfoByIndex(i, &tChildInfo);

				//Do not register invalid devices
				uint8_t isValid = 0;
				for(int j = 0; j < 8; j++){
					if(tChildInfo.mExtAddress.m8[j] != 0){
						isValid = 1;
						break;
					}
				}
				if(!isValid) continue;

				struct M_DeviceDescriptor tDeviceDescriptor;

				PUTLE16(otGetPanId() ,tDeviceDescriptor.PANId);
				PUTLE16(tChildInfo.mRloc16, tDeviceDescriptor.ShortAddress);
				for(int j = 0; j < 8; j++) tDeviceDescriptor.ExtAddress[j] = tChildInfo.mExtAddress.m8[7-j];	//Flip endian
				tDeviceDescriptor.FrameCounter[0] = 0;	//TODO: Figure out how to do frame counter properly - this method is temporarily breaking replay protection as replays using previous key will still be successful
				tDeviceDescriptor.FrameCounter[1] = 0;
				tDeviceDescriptor.FrameCounter[2] = 0;
				tDeviceDescriptor.FrameCounter[3] = 0;
				tDeviceDescriptor.Exempt = 0;


				MLME_SET_request_sync(
						macDeviceTable,
						count++,
						sizeof(tDeviceDescriptor),
						&tDeviceDescriptor,
						pDeviceRef
						);

			}

			uint8_t maxRouters = 5 - count;
			otRouterInfo routers[maxRouters];
			uint8_t numRouters;
			otGetNeighborRouterInfo(routers, &numRouters, maxRouters);

			for(int i = 0; i < numRouters; i++){
				struct M_DeviceDescriptor tDeviceDescriptor;

				PUTLE16(otGetPanId() ,tDeviceDescriptor.PANId);
				PUTLE16(routers[i].mRloc16, tDeviceDescriptor.ShortAddress);
				for(int j = 0; j < 8; j++) tDeviceDescriptor.ExtAddress[j] = routers[i].mExtAddress.m8[7-j];	//Flip endian
				tDeviceDescriptor.FrameCounter[0] = 0;	//TODO: Figure out how to do frame counter properly - this method is temporarily breaking replay protection as replays using previous key will still be successful
				tDeviceDescriptor.FrameCounter[1] = 0;
				tDeviceDescriptor.FrameCounter[2] = 0;
				tDeviceDescriptor.FrameCounter[3] = 0;
				tDeviceDescriptor.Exempt = 0;

				MLME_SET_request_sync(
						macDeviceTable,
						count++,
						sizeof(tDeviceDescriptor),
						&tDeviceDescriptor,
						pDeviceRef
						);
			}

		}
		else{
			otRouterInfo tParentInfo;
			if(otGetParentInfo(&tParentInfo) == kThreadError_None){
				struct M_DeviceDescriptor tDeviceDescriptor;

				PUTLE16(otGetPanId(), tDeviceDescriptor.PANId);
				PUTLE16(tParentInfo.mRloc16, tDeviceDescriptor.ShortAddress);
				for(int j = 0; j < 8; j++) tDeviceDescriptor.ExtAddress[j] = tParentInfo.mExtAddress.m8[7-j];	//Flip endian
				tDeviceDescriptor.FrameCounter[0] = 0;	//TODO: Figure out how to do frame counter properly - this method is temporarily breaking replay protection as replays using previous key will still be successful
				tDeviceDescriptor.FrameCounter[1] = 0;
				tDeviceDescriptor.FrameCounter[2] = 0;
				tDeviceDescriptor.FrameCounter[3] = 0;
				tDeviceDescriptor.Exempt = 0;

				MLME_SET_request_sync(
						macDeviceTable,
						count++,
						sizeof(tDeviceDescriptor),
						&tDeviceDescriptor,
						pDeviceRef
						);
			}
			else{
				otPlatLog(kLogLevelWarn, kLogRegionHardMac, "Error retrieving parent!\n\r");
			}
		}

		MLME_SET_request_sync(
				macDeviceTableEntries,
				0,
				1,
				&count,
				pDeviceRef
				);

		struct M_KeyDescriptor_thread {
			struct M_KeyTableEntryFixed    Fixed;
			struct M_KeyIdLookupDesc       KeyIdLookupList[1];
			uint8_t                        flags[7];
			//struct M_KeyDeviceDesc         KeyDeviceList[count];
			//struct M_KeyUsageDesc          KeyUsageList[2];
		}tKeyDescriptor;


		//Table 7-5 in section 7.2.2.2.1 of thread spec
		tKeyDescriptor.Fixed.KeyIdLookupListEntries = 1;
		tKeyDescriptor.Fixed.KeyUsageListEntries = 2;
		tKeyDescriptor.Fixed.KeyDeviceListEntries = count;
		curDeviceTableSize = count;

		//Flags according to Cascoda API 5.3.1
		//tKeyDescriptor.KeyUsageList[0].Flags = (MAC_FC_FT_DATA & KUD_FrameTypeMask);	//data usage
		//tKeyDescriptor.KeyUsageList[1].Flags = (MAC_FC_FT_COMMAND & KUD_FrameTypeMask) | ((CMD_DATA_REQ << KUD_CommandFrameIdentifierShift) & KUD_CommandFrameIdentifierMask);	//Data req usage
		tKeyDescriptor.flags[count + 0] = (MAC_FC_FT_DATA & KUD_FrameTypeMask);	//data usage
		tKeyDescriptor.flags[count + 1] = (MAC_FC_FT_COMMAND & KUD_FrameTypeMask) | ((CMD_DATA_REQ << KUD_CommandFrameIdentifierShift) & KUD_CommandFrameIdentifierMask);	//Data req usage


		tKeyDescriptor.KeyIdLookupList[0].LookupDataSizeCode = 1; //1 means length 9
		//This sets the MSB of the lookUpData to equal defaultKeySource as is required by 7.5.8.2.2 of IEEE 15.4 spec
		for(int i = 0; i < 9; i++) tKeyDescriptor.KeyIdLookupList[0].LookupData[i] = 0;
		tKeyDescriptor.KeyIdLookupList[0].LookupData[8] = 0xFF;	//Set lookup data to the macDefaultKeySource to be right concatenated to the individual keyIndex param


		//Fill the deviceListEntries
		for(int i = 0; i < count; i++){
			//tKeyDescriptor.KeyDeviceList[i].Flags = i;
			tKeyDescriptor.flags[i] = i;
		}

		//Generate and store the keys for the current, previous, and next rotations
		uint8_t storeCount = 0;
		for(uint8_t i = 0; i < 3; i++){
			if((tKeySeq + i) > 0){	//0 is invalid key sequence
				memcpy(tKeyDescriptor.Fixed.Key, getMacKeyFromSequenceCounter(tKeySeq + i), 16);
				tKeyDescriptor.KeyIdLookupList[0].LookupData[0] = ((tKeySeq + i) & 0x7F) + 1;

				MLME_SET_request_sync(
					macKeyTable,
					storeCount++,
					sizeof(tKeyDescriptor) - (5-count),	/*dont send the unused bytes (those not counted by count)*/
					&tKeyDescriptor,
					pDeviceRef
					);
			}
		}
		MLME_SET_request_sync(
			macKeyTableEntries,
			0,
			1,
			&storeCount,
			pDeviceRef
			);

	}
}

ThreadError otPlatRadioEnable(void)    //TODO:(lowpriority) port 
{
	ThreadError error = kThreadError_Busy;

	if (sState == kStateSleep || sState == kStateDisabled)
	{
		error = kThreadError_None;
		sState = kStateSleep;

		#ifdef EXECUTE_MODE
			uint8_t HWMEAttVal[5] = {00, 00, 00, 00, 00};
			if (HWME_SET_request_sync (
				HWME_POWERCON,
				5,
				HWMEAttVal,
				pDeviceRef
				) == HWME_SUCCESS)
				return kThreadError_None;
			else return kThreadError_Failed;

		#endif
	}

    return error;
}

ThreadError otPlatRadioDisable(void)    //TODO:(lowpriority) port 
{
	ThreadError error = kThreadError_Busy;

	if (sState == kStateDisabled || sState == kStateSleep)
	{
		error = kThreadError_None;
		sState = kStateDisabled;

		//should sleep until restarted
		#ifdef EXECUTE_MODE
			uint8_t HWMEAttVal[5] = {0x0A, 00, 00, 00, 00};
			if (HWME_SET_request_sync (
				HWME_POWERCON,
				5,
				HWMEAttVal,
				pDeviceRef
				) == HWME_SUCCESS)
				return kThreadError_None;
			else return kThreadError_Failed;

		#endif
	}

    return error;
}

ThreadError otPlatRadioSleep(void)    //TODO:(lowpriority) port 
{
	return kThreadError_None;
	//This is handled by the hardmac and the state of rxOnWhenIdle
}

ThreadError otPlatRadioSetRxOnWhenIdle(uint8_t rxOnWhenIdle){
	ThreadError error = kThreadError_None;
	uint8_t ret = 0;
	rxOnWhenIdle = rxOnWhenIdle ? 1 : 0;

	ret = MLME_SET_request_sync(	//enable Rx when Idle
				macRxOnWhenIdle,
				0,
				sizeof(rxOnWhenIdle),
				&rxOnWhenIdle,
				pDeviceRef);

	error = ret ? kThreadError_Busy : kThreadError_None;

	return error;
}

ThreadError otPlatRadioReceive(uint8_t aChannel)
{
    ThreadError error = kThreadError_None;

    VerifyOrExit(sState != kStateDisabled, error = kThreadError_Busy);
    sState = kStateReceive;

    setChannel(aChannel);

	#ifdef EXECUTE_MODE
    	uint8_t HWMEAttVal[5] = {0x24, 00, 00, 00, 00};
		if (HWME_SET_request_sync (
			HWME_POWERCON,
			5,
			HWMEAttVal,
			pDeviceRef
			) == HWME_SUCCESS)
			return kThreadError_None;
		else return kThreadError_Failed;

	#endif

exit:
    return error;
}

RadioPacket *otPlatRadioGetTransmitBuffer(void)
{
    return &sTransmitFrame;
}

ThreadError otPlatRadioTransmit(void * transmitContext)
{
	/*
	 * This Function converts the openthread-provided PHY frame into a MAC primitive to
	 * communicate with the hardmac-enabled cascoda ca8210. Changes in the soft MAC layer
	 * prevent the frame from being encrypted until it reaches the hardmac.
	 */
    ThreadError error = kThreadError_None;
    static uint8_t handle = 0;
    handle++;

    while(isHandleInUse(handle)) handle++;  //make sure handle is available for use

    VerifyOrExit(sState != kStateDisabled, error = kThreadError_Busy);

    uint16_t frameControl = GETLE16(sTransmitFrame.mPsdu);
    VerifyOrExit(((frameControl & MAC_FC_FT_MASK) == MAC_FC_FT_DATA) || \
    	((frameControl & MAC_FC_FT_MASK) == MAC_FC_FT_COMMAND),         \
    	error = kThreadError_Abort;                                     \
    	otPlatLog(kLogLevelWarn, kLogRegionHardMac, "Unexpected frame type %#x\n\r", (frameControl & MAC_FC_FT_MASK)););

/*
    fputs("\r\nTransmit:",stderr);
    for(i = 0; i < sTransmitFrame.mLength; i++){
    	fprintf(stderr, " %#04x", sTransmitFrame.mPsdu[i]);
    }
    fputs("\r\n",stderr);
*/

    sState = kStateTransmit;
    sTransmitError = kThreadError_None;

    setChannel(sTransmitFrame.mChannel);

    //transmit
    struct MCPS_DATA_request_pset curPacket;
    struct SecSpec curSecSpec = {0};

    uint8_t headerLength = 0;
    uint8_t footerLength = 0;

    curPacket.SrcAddrMode = MAC_FC_SAM(frameControl);
    curPacket.Dst.AddressMode = MAC_FC_DAM(frameControl);
    curPacket.TxOptions = (frameControl & MAC_FC_ACK_REQ) ? 0x01 : 0x00;		//Set bit 0 for ack-requesting transmissions
    curPacket.TxOptions |= sTransmitFrame.mDirectTransmission ? 0x00 : 1<<2;	//Set bit 2 for indirect transmissions
    uint8_t isPanCompressed = frameControl & MAC_FC_PAN_COMP;

    uint8_t addressFieldLength = 0;

    if(curPacket.Dst.AddressMode == MAC_MODE_SHORT_ADDR){
    	memcpy(curPacket.Dst.Address, sTransmitFrame.mPsdu+5, 2);
    	memcpy(curPacket.Dst.PANId, sTransmitFrame.mPsdu+3, 2);
    	addressFieldLength +=4;
    }
    else if(curPacket.Dst.AddressMode == MAC_MODE_LONG_ADDR){
    	memcpy(curPacket.Dst.Address, sTransmitFrame.mPsdu+5, 8);
    	memcpy(curPacket.Dst.PANId, sTransmitFrame.mPsdu+3, 2);
    	addressFieldLength +=10;
    }

    if((frameControl & MAC_FC_FT_MASK) == MAC_FC_FT_DATA){
		sTransmitFrame.mTransmitContext = transmitContext;
		putIntransitFrame(handle, &sTransmitFrame, headerLength); //Don't need to store security information OR source address for intransits, just destination addressing info and fc (Max length 13 bytes)
	}

    if(curPacket.SrcAddrMode == MAC_MODE_SHORT_ADDR) addressFieldLength += 4;
    else if(curPacket.SrcAddrMode == MAC_MODE_LONG_ADDR) addressFieldLength += 10;
    if(curPacket.SrcAddrMode && isPanCompressed) addressFieldLength -= 2; //Remove size saved by not including the same PAN twice
    headerLength = addressFieldLength + MAC_BASEHEADERLENGTH;

    if(frameControl & MAC_FC_SEC_ENA){	//if security is required
    	uint8_t ASHloc = MAC_BASEHEADERLENGTH + addressFieldLength;
    	uint8_t securityControl = *(uint8_t*)(sTransmitFrame.mPsdu + ASHloc);
    	curSecSpec.SecurityLevel = MAC_SC_SECURITYLEVEL(securityControl);
    	curSecSpec.KeyIdMode = MAC_SC_KEYIDMODE(securityControl);

    	ASHloc += 5;//skip to key identifier
    	if(curSecSpec.KeyIdMode == 0x02){//Table 96
    		memcpy(curSecSpec.KeySource, sTransmitFrame.mPsdu + ASHloc, 4);
    		ASHloc += 4;
    	}
    	else if(curSecSpec.KeyIdMode == 0x03){//Table 96
			memcpy(curSecSpec.KeySource, sTransmitFrame.mPsdu + ASHloc, 8);
			ASHloc += 8;
		}
    	curSecSpec.KeyIndex = sTransmitFrame.mPsdu[ASHloc++];
    	headerLength = ASHloc;
    }

    if((frameControl & MAC_FC_FT_MASK) == MAC_FC_FT_DATA){
		//Table 95 to calculate auth tag length
		footerLength = 2 << (curSecSpec.SecurityLevel % 4);
		footerLength = (footerLength == 2) ? 0 : footerLength;

		footerLength += 2; //MFR length

		curPacket.MsduLength = sTransmitFrame.mLength - footerLength - headerLength;
		memcpy(curPacket.Msdu, sTransmitFrame.mPsdu + headerLength, curPacket.MsduLength);
		curPacket.MsduHandle = handle;

		MCPS_DATA_request(
			curPacket.SrcAddrMode,
			curPacket.Dst.AddressMode,
			GETLE16(curPacket.Dst.PANId),
			(union MacAddr*) curPacket.Dst.Address,
			curPacket.MsduLength,
			curPacket.Msdu,
			curPacket.MsduHandle,
			curPacket.TxOptions,
			&curSecSpec,
			pDeviceRef);
    }
    else if((frameControl & MAC_FC_FT_MASK) == MAC_FC_FT_COMMAND){
    	if(sTransmitFrame.mPsdu[headerLength] == 0x04){	//Data request command

    		uint8_t interval[2] = {0, 0};
    		uint8_t ret;
    		uint8_t count = 0;

    		do{
    		ret = MLME_POLL_request_sync(curPacket.Dst,
    		                       interval,
			                       &curSecSpec,
			                       pDeviceRef);
    		if(count > 0) otPlatLog(kLogLevelWarn, kLogRegionHardMac, "Poll Failed! Retry #%d", count);
    		} while(ret == 0xFF && (count++ < 10));

    		if(ret == MAC_SUCCESS){
    			otPlatRadioTransmitDone(true, error, &sTransmitFrame, transmitContext);
    		}
    		else if(ret == MAC_NO_DATA){
    			otPlatRadioTransmitDone(false, error, &sTransmitFrame, transmitContext);
    		}
    		else{
    			otPlatRadioTransmitDone(false, kThreadError_NoAck, &sTransmitFrame, transmitContext);
    		}
    	}
    	else{
    		assert(false);
    	}
    }

exit:
    return error;
}

int8_t otPlatRadioGetNoiseFloor(void)    //TODO:(lowpriority) port 
{
    return noiseFloor;
}

otRadioCaps otPlatRadioGetCaps(void)
{
    return kRadioCapsAckTimeout;
}

bool otPlatRadioGetPromiscuous(void)
{
	if(promiscuousCache != tristate_uninit) return promiscuousCache;

	uint8_t resultLen;
    uint8_t result;
    MLME_GET_request_sync(
        macPromiscuousMode,
        0,
        &resultLen,
        &result,
        pDeviceRef);

    result = result ? tristate_enabled : tristate_disabled;
    promiscuousCache = result;

    return (bool) result;
}

void otPlatRadioSetPromiscuous(bool aEnable)
{
	uint8_t enable = aEnable ? 1 : 0;	//Just to be sure we match spec
    MLME_SET_request_sync(
        macPromiscuousMode,
        0,
        sizeof(enable),
        &enable,
        pDeviceRef);
    promiscuousCache = aEnable;
}

int readFrame(struct MCPS_DATA_indication_pset *params)   //Async
{

	/*
	 * This Function converts the MAC primitive provided by the ca8210 into a PHY frame
	 * that can be processed by openthread. Changes in the soft MAC layer mean that the
	 * frame is not double decrypted, and the link security is accepted if the hardmac
	 * approves it.
	 */

	if(!otIsInterfaceUp()) return 1;

    pthread_mutex_lock(&receiveFrame_mutex);
	//wait until the main thread is free to process the frame
	while(sReceiveFrame.mLength != 0) {pthread_cond_wait(&receiveFrame_cond, &receiveFrame_mutex);}

	uint8_t headerLength = 0;
	uint8_t footerLength = 0;
	uint16_t frameControl = 0;
	uint8_t msduLength = params->MsduLength;
	struct SecSpec * curSecSpec = (struct SecSpec*)((unsigned int)params + (unsigned int)msduLength + 29); //Location defined in cascoda API docs

	frameControl |= (params->Src.AddressMode & 0x3) << 14;
	frameControl |= (params->Dst.AddressMode & 0x3) << 10;
	frameControl |= curSecSpec->SecurityLevel ? MAC_FC_SEC_ENA : 0;	//Security Enabled field
	frameControl |= MAC_FC_FT_DATA; //Frame type = data

	uint8_t addressFieldLength = 0;

	if(params->Dst.AddressMode == MAC_MODE_SHORT_ADDR){
		memcpy(sReceiveFrame.mPsdu+5, params->Dst.Address, 2);
		memcpy(sReceiveFrame.mPsdu+3, params->Dst.PANId, 2);
		addressFieldLength +=4;
	}
	else if(params->Dst.AddressMode == MAC_MODE_LONG_ADDR){
		memcpy(sReceiveFrame.mPsdu+5, params->Dst.Address, 8);
		memcpy(sReceiveFrame.mPsdu+3, params->Dst.PANId, 2);
		addressFieldLength +=10;
	}

	if(memcmp(params->Src.PANId, params->Dst.PANId, 2)){ //Different PANs
		if(params->Src.AddressMode == MAC_MODE_SHORT_ADDR){
			memcpy(sReceiveFrame.mPsdu + addressFieldLength + 5, params->Src.Address, 2);
			memcpy(sReceiveFrame.mPsdu + addressFieldLength + 3, params->Src.PANId, 2);
			addressFieldLength +=4;
		}
		else if(params->Src.AddressMode == MAC_MODE_LONG_ADDR){
			memcpy(sReceiveFrame.mPsdu + addressFieldLength + 5, params->Src.Address, 8);
			memcpy(sReceiveFrame.mPsdu + addressFieldLength + 3, params->Src.PANId, 2);
			addressFieldLength +=10;
		}
	}
	else{	//use PAN compression
		if(params->Src.AddressMode == MAC_MODE_SHORT_ADDR){
			memcpy(sReceiveFrame.mPsdu + addressFieldLength + 3, params->Src.Address, 2);
			addressFieldLength +=2;
		}
		else if(params->Src.AddressMode == MAC_MODE_LONG_ADDR){
			memcpy(sReceiveFrame.mPsdu + addressFieldLength + 3, params->Src.Address, 8);
			addressFieldLength +=8;
		}
		frameControl |= MAC_FC_PAN_COMP;
	}

	PUTLE16(frameControl, sReceiveFrame.mPsdu);	//Commit the frame control bytes to the frame

	headerLength = addressFieldLength + MAC_BASEHEADERLENGTH;

	if(frameControl & MAC_FC_SEC_ENA){	//if security is required
		uint8_t ASHloc = MAC_BASEHEADERLENGTH + addressFieldLength;

		uint8_t securityControl = 0;
		securityControl |= MAC_SC_SECURITYLEVEL(curSecSpec->SecurityLevel);
		securityControl |= MAC_KEYIDMODE_SC(curSecSpec->KeyIdMode);

		sReceiveFrame.mPsdu[ASHloc] = securityControl;

		ASHloc += 5;//skip to key identifier
		if(curSecSpec->KeyIdMode == 0x02){//Table 96
			memcpy(sReceiveFrame.mPsdu + ASHloc, curSecSpec->KeySource, 4);
			ASHloc += 4;
		}
		else if(curSecSpec->KeyIdMode == 0x03){//Table 96
			memcpy(sReceiveFrame.mPsdu + ASHloc, curSecSpec->KeySource, 8);
			ASHloc += 8;
		}
		sReceiveFrame.mPsdu[ASHloc++] = curSecSpec->KeyIndex;
		headerLength = ASHloc;
	}

	//Table 95 to calculate auth tag length
	footerLength = 2 << (curSecSpec->SecurityLevel % 4);
	footerLength = footerLength == 2 ? 0 : footerLength;

	footerLength += 2; //MFR length

	sReceiveFrame.mLength = params->MsduLength + footerLength + headerLength;

	if(sReceiveFrame.mLength > aMaxPHYPacketSize){
		otPlatLog(kLogLevelWarn, kLogRegionHardMac, "Invalid frame Length %d! Msdu: %d; Footer: %d; Header: %d;\n\r", sReceiveFrame.mLength, params->MsduLength, footerLength, headerLength);
		return 1;
	}

	memcpy(sReceiveFrame.mPsdu + headerLength, params->Msdu, params->MsduLength);
	sReceiveFrame.mLqi = params->MpduLinkQuality;
	sReceiveFrame.mChannel = sChannel;
	sReceiveFrame.mPower = (params->MpduLinkQuality - 256)/2;	//Formula from CA-821X API
	noiseFloor = sReceiveFrame.mPower;

/*
	fputs("\r\nReceived:",stderr);
	for(int i = 0; i < sReceiveFrame.mLength; i++){
		fprintf(stderr, " %#04x", sReceiveFrame.mPsdu[i]);
	}
	fputs("\r\n",stderr);
*/

    pthread_mutex_unlock(&receiveFrame_mutex);

    barrier_worker_waitForMain();
	sState = kStateReceive;
	otPlatRadioReceiveDone(&sReceiveFrame, sReceiveError);
	barrier_worker_endWork();

    PlatformRadioSignal();

    return 0;
}

int readConfirmFrame(struct MCPS_DATA_confirm_pset *params)   //Async
{

	/*
	 * This Function processes the MCPS_DATA_CONFIRM and passes the success or error
	 * to openthread as appropriate.
	 */
	if(!otIsInterfaceUp()) return 1;

	barrier_worker_waitForMain();

	RadioPacket * sentFrame = getIntransitFrame(params->MsduHandle);
	assert(sentFrame != NULL);

    if(params->Status == MAC_SUCCESS){
    	otPlatRadioTransmitDone(false, sTransmitError, sentFrame, sentFrame->mTransmitContext);
    	sState = kStateReceive;
    	sTransmitError = kThreadError_None;
    }
    else{
    	//TODO: Better handle the channel_access_failures
    	if(params->Status == MAC_CHANNEL_ACCESS_FAILURE) sTransmitError = kThreadError_ChannelAccessFailure;
    	else if(params->Status == MAC_NO_ACK) sTransmitError = kThreadError_NoAck;
    	else sTransmitError = kThreadError_Abort;
    	otPlatLog(kLogLevelWarn, kLogRegionHardMac, "MCPS_DATA_confirm error: %#x \r\n", params->Status);
    	otPlatRadioTransmitDone(false, sTransmitError, sentFrame, sentFrame->mTransmitContext);
    	sState = kStateReceive;
    	sTransmitError = kThreadError_None;
    }

    barrier_worker_endWork();

    rmIntransitFrame(params->MsduHandle);

    PlatformRadioSignal();

    return 0;
}

int beaconNotifyFrame(struct MLME_BEACON_NOTIFY_indication_pset *params) //Async
{

	/*
	 * This Function processes an incoming beacon from an activeScan, processing the payload
	 * and passing the relevant information to openthread in a struct.
	 */

	if(!otIsInterfaceUp()) return 1;

	otActiveScanResult resultStruct;

	uint8_t shortaddrs  = *((uint8_t*)params + 23) & 7;
	uint8_t extaddrs = (*((uint8_t*)params + 23) & 112) >> 4;

	if ((params->PanDescriptor.Coord.AddressMode) == 3) {
		memcpy(resultStruct.mExtAddress.m8, params->PanDescriptor.Coord.Address, 8);
	} else {
    	otPlatLog(kLogLevelWarn, kLogRegionHardMac, "Invalid beacon received!\r\n");
		return 1;
	}
	resultStruct.mPanId = GETLE16(params->PanDescriptor.Coord.PANId);
	resultStruct.mChannel = params->PanDescriptor.LogicalChannel;
	resultStruct.mRssi = (params->PanDescriptor.LinkQuality - 256)/2;
	noiseFloor = resultStruct.mRssi;
	resultStruct.mLqi = params->PanDescriptor.LinkQuality;
	VerifyOrExit(params->PanDescriptor.Security.SecurityLevel == 0,;);
	//Asset security = 0
	uint8_t *sduLength = (uint8_t*)params + (24 + (2 * shortaddrs) + (8 * extaddrs));
	if (*sduLength > 0) {
		uint8_t *Sdu = (uint8_t*)params + (25 + 2 * shortaddrs + 8 * extaddrs);
		uint8_t version = (*((uint8_t*)Sdu + 1) & 15);
		if(*Sdu == 3 && version == 1) {
			memcpy(&resultStruct.mNetworkName, ((char*)Sdu) + 2, sizeof(resultStruct.mNetworkName));
			memcpy(&resultStruct.mExtendedPanId, Sdu + 18, sizeof(resultStruct.mExtendedPanId));
			barrier_worker_waitForMain();
			scanCallback(&resultStruct, scanContext);
			barrier_worker_endWork();
		}
	}

exit:
    return 0;
}

int scanConfirmCheck(struct MLME_SCAN_confirm_pset *params) {

	if(!otIsInterfaceUp()) return 1;

	if (params->Status != MAC_SCAN_IN_PROGRESS) {
		barrier_worker_waitForMain();
		scanCallback(NULL, scanContext);
		activeScanInProgress = 1;
		barrier_worker_endWork();
		MLME_SET_request_sync(
		    			phyCurrentChannel,
		    	        0,
		    	        sizeof(sChannel),
		    	        &sChannel,
		    	        pDeviceRef);
	}

	return 0;
}

int genericDispatchFrame(const uint8_t *buf, size_t len) { //Async

	/*
	 * This is a debugging function for unhandled incoming MAC data
	 */

	fprintf(stderr, "\n\rUnhandled: ");
	for(int i = 0; i < len; i++) {
		fprintf(stderr, "%02x ", buf[i]);
	}
	fprintf(stderr, "\n\r");
	return 0;
}

int PlatformRadioSignal(void)
{
	pthread_mutex_lock(&receiveFrame_mutex);
    sReceiveFrame.mLength = 0;
    pthread_cond_broadcast(&receiveFrame_cond);
    pthread_mutex_unlock(&receiveFrame_mutex);

    return 0;
}

int PlatformRadioProcess(void){
	barrier_main_letWorkerWork();
	return 0;
}

static struct DeviceCache * getCachedDevice(otExtAddress addr){
	struct DeviceCache * toReturn = NULL;
	uint8_t foundEmpty = 0xFF;

	//Find existing cache
	for(int i = 0; i < DEVICE_TABLE_SIZE; i++){
		if(sDeviceCache[i].isActive){
			if(memcmp(addr.m8, sDeviceCache[i].mExtAddr.m8, sizeof(addr)) == 0){
				return &sDeviceCache[i];
			}
		}
		else if(foundEmpty == 0xFF){
			foundEmpty = i;
		}
	}

	toReturn = &sDeviceCache[foundEmpty];
	//Prepare and return a new cache
	memset(toReturn, 0, sizeof(*toReturn));
	toReturn->isActive = 1;
	toReturn->mExtAddr = addr;

	return toReturn;
}

static void cacheDevices(){
	for(uint8_t i = 0; i < curDeviceTableSize; i++){
		uint8_t length;
		struct M_DeviceDescriptor tDeviceDescriptor;

		MLME_GET_request_sync(	macDeviceTable,
								i,
								&length,
								&tDeviceDescriptor,
								pDeviceRef);

		otExtAddress cur;
		struct DeviceCache * curCache;
		memcpy(cur.m8, tDeviceDescriptor.ExtAddress, sizeof(cur));
		curCache = getCachedDevice(cur);
		memcpy(curCache->mFrameCounter, tDeviceDescriptor.FrameCounter, 4);
		curCache->isActive = 2;
	}

	for(uint8_t i = 0; i < DEVICE_TABLE_SIZE; i++){
		if(sDeviceCache[i].isActive == 1){
			//Device was not in device table, remove from cache
			memset(&sDeviceCache[i], 0, sizeof(sDeviceCache[i]));
		}
		else if(sDeviceCache[i].isActive == 2){
			sDeviceCache[i].isActive = 1;
		}
	}
}

uint8_t otPlatRadioIsDeviceActive(otExtAddress addr){
	//update cache
	//TODO: Don't recache all devices every time -> just update the frame counter for the relevant one and
	//recache all of the rest as the macDeviceTable is changed.
	cacheDevices();

	//Find existing cache
	for(int i = 0; i < DEVICE_TABLE_SIZE; i++){
		if(sDeviceCache[i].isActive && memcmp(addr.m8, sDeviceCache[i].mExtAddr.m8, sizeof(addr)) == 0){
			//Device match found
			if(memcmp(sDeviceCache[i].mFrameCounter, sDeviceCache[i].mTimeoutFrameCounter, 4) == 0){
				//Device has not sent any messages since last poll -> device is inactive
				//(This will promptly be removed by the higher level and will flush through)
				return 0;
			}
			else{
				//Update the timeout version, return 1 -> the device is active
				memcpy(sDeviceCache[i].mTimeoutFrameCounter, sDeviceCache[i].mFrameCounter, 4);
				return 1;
			}
		}
	}
	return 0;
}

/*
 * The following functions exist to create a queue for in-transit data packets. The complete frames are not
 * stored, only the header information required to process the completion of the transmission. This is because
 * it is not guaranteed for these frames to transmit in order, and the destination address is required for
 *  post-transmission processing.
 */

static int isHandleInUse(uint8_t handle){
	pthread_mutex_lock(&intransit_mutex);
	for(int i = 0; i < MAX_INTRANSITS; i++){
		if(IntransitHandles[i] == handle){
			pthread_mutex_unlock(&intransit_mutex);
			return 1;	//Handle is in use
		}
	}
	pthread_mutex_unlock(&intransit_mutex);
	return 0;
}

static int putIntransitFrame(uint8_t handle, const RadioPacket * in, uint8_t headerSize){
	int i = 0;
	uint8_t found = 0;

	pthread_mutex_lock(&intransit_mutex);
	while(!found){
		if(i >= MAX_INTRANSITS) return -1;	//No space for intransit frame storage! This should be impossible -> probably increase array size!

		if(IntransitHandles[i] == 0) break; //This index is empty

		i++;
	}

	IntransitHandles[i] = handle;

	memcpy(&IntransitPackets[i], in, sizeof(RadioPacket));
	IntransitPackets[i].mPsdu = malloc((size_t)headerSize);
	memcpy(IntransitPackets[i].mPsdu, in->mPsdu, headerSize);

	pthread_mutex_unlock(&intransit_mutex);

	return 0;
}

static int rmIntransitFrame(uint8_t handle){
	//TODO: Overchecks for debugging purposes -> remove this once confident
	uint8_t found = 0;

	pthread_mutex_lock(&intransit_mutex);

	for(int i = 0; i < MAX_INTRANSITS; i++){
		if(IntransitHandles[i] == handle){
			assert(found == 0); //Crash if there was more than one instance of the same handle
			found = 1;
			free(IntransitPackets[i].mPsdu);
			IntransitHandles[i] = 0;
		}
	}

	pthread_mutex_unlock(&intransit_mutex);

	return !found;	//0 if successfully removed
}

static RadioPacket * getIntransitFrame(uint8_t handle){
	pthread_mutex_lock(&intransit_mutex);

	for(int i = 0; i < MAX_INTRANSITS; i++){
		if(IntransitHandles[i] == handle){
			pthread_mutex_unlock(&intransit_mutex);
			return &IntransitPackets[i];
		}
	}
	pthread_mutex_unlock(&intransit_mutex);
	return NULL;
}

/*
 * The following functions create a thread safe system for allowing the worker thread to access openthread
 * functions safely. The main thread always has priority and must explicitly give control to the worker
 * thread while locking itself. This has ONLY been designed to work with ONE worker and ONE main, in a way
 * that causes the worker thread to run one operation per poll cycle as openthread was designed for.
 */

//Lets the worker thread work synchronously if there is synchronous work to do
static inline void barrier_main_letWorkerWork(){
	pthread_mutex_lock(&barrierMutex);
	if(mbarrier_waiting == WAITING){
		mbarrier_waiting = GREENLIGHT;
		pthread_cond_broadcast(&barrierCondVar);
		while(mbarrier_waiting != DONE) pthread_cond_wait(&barrierCondVar, &barrierMutex);	//The worker thread does work now
	}
	mbarrier_waiting = NOT_WAITING;
	pthread_cond_broadcast(&barrierCondVar);
	pthread_mutex_unlock(&barrierMutex);
}

static inline void barrier_worker_waitForMain(){
	pthread_mutex_lock(&barrierMutex);
	while(mbarrier_waiting != NOT_WAITING) pthread_cond_wait(&barrierCondVar, &barrierMutex);
	selfpipe_push();
	mbarrier_waiting = WAITING;
	pthread_cond_broadcast(&barrierCondVar);
	while(mbarrier_waiting != GREENLIGHT) pthread_cond_wait(&barrierCondVar, &barrierMutex); //wait for the main thread to signal worker to run
}

static inline void barrier_worker_endWork(){
	mbarrier_waiting = DONE;
	pthread_cond_broadcast(&barrierCondVar);
	pthread_mutex_unlock(&barrierMutex);
}

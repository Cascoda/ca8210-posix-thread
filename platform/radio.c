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
 * pp
 */

#include <openthread-types.h>
#include <openthread.h>

#include <math.h>
#include <stdio.h>
#include <assert.h>
#include <common/code_utils.hpp>
#include <platform/radio.h>
#include <cascoda_api.h>
#include <kernel_exchange.h>
#include <string.h>
#include <mac_messages.h>
#include "posix-platform.h"
#include <ieee_802_15_4.h>
#include <pthread.h>


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

void readFrame(struct MCPS_DATA_indication_pset *params);
void readConfirmFrame(struct MCPS_DATA_confirm_pset *params);

void beaconNotifyFrame(struct MLME_BEACON_NOTIFY_indication_pset *params);
int genericDispatchFrame(const uint8_t *buf, size_t len);
void keyChangedCallback(uint32_t aFlags, void *aContext);
void coordChangedCallback(uint32_t aFlags, void *aContext);


static struct MAC_Message response;
static RadioPacket sTransmitFrame;
static RadioPacket sReceiveFrame;
static ThreadError sTransmitError;
static ThreadError sReceiveError = kThreadError_None;

static otHandleActiveScanResult scanCallback;

static void* pDeviceRef = NULL;

static uint8_t sChannel = 0;

static isCoord = 0;

static uint8_t sTransmitPsdu[IEEE802154_MAX_LENGTH];
static uint8_t sReceivePsdu[IEEE802154_MAX_LENGTH];

static uint8_t mBeaconPayload[32] = {3, 0x98};

pthread_mutex_t receiveFrame_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t receiveFrame_cond = PTHREAD_COND_INITIALIZER;

typedef enum PhyState
{
    kStateDisabled = 0,
    kStateSleep,
    kStateIdle,
    kStateListen,
    kStateReceive,
    kStateTransmit,
} PhyState;

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
        fprintf(stderr, "\n\rChannel: %d\n\r", sChannel);
    }
}

void enableReceiver(void)
{
    // nothing
}

void disableReceiver(void)
{
    //nothing
}

ThreadError otActiveScan(uint32_t aScanChannels, uint16_t aScanDuration, otHandleActiveScanResult aCallback)
{
	fprintf(stderr, "\n\r I'm executing otActiveScan \n\r");
	//uint16_t aScanDuration = aBaseSuperframeDuration * (pow(2,ScanDuration) +1);
	//uint8_t ScanDuration = log2((aScanDuration/aBaseSuperframeDuration) -1);
	uint8_t ScanDuration = 8;
	struct SecSpec pSecurity = {0};

	scanCallback = aCallback;
	uint8_t scanRequest = MLME_SCAN_request(1, aScanChannels, ScanDuration, &pSecurity, pDeviceRef);
	fprintf(stderr, "\n %x \n", scanRequest);
	return scanRequest;
}

ThreadError otPlatSetNetworkName(const char *aNetworkName) {

	fprintf(stderr,"\n\r I'm setting the network name \n\r");
	uint8_t payloadLength = 32;
	memcpy(mBeaconPayload + 4, aNetworkName, 16);
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
			pDeviceRef) == MAC_SUCCESS))
        return kThreadError_None;

	else return kThreadError_Failed;
}

ThreadError otPlatSetExtendedPanId(const uint8_t *aExtPanId) {

	uint8_t payloadLength = 32;
	memcpy(mBeaconPayload + 20, aExtPanId, 8);
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
			pDeviceRef) == MAC_SUCCESS))
           return kThreadError_None;

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
        pDeviceRef) == MAC_SUCCESS)
            return kThreadError_None;

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

void PlatformRadioInit(void)
{
    sTransmitFrame.mLength = 0;
    sTransmitFrame.mPsdu = sTransmitPsdu;
    
    pthread_mutex_lock(&receiveFrame_mutex);
		sReceiveFrame.mLength = 0;
		sReceiveFrame.mPsdu = sReceivePsdu;
		pthread_cond_broadcast(&receiveFrame_cond);
	pthread_mutex_unlock(&receiveFrame_mutex);
    

    kernel_exchange_init();

    struct cascoda_api_callbacks callbacks = {0};
    callbacks.MCPS_DATA_indication = &readFrame;
    callbacks.MCPS_DATA_confirm = &readConfirmFrame;
    callbacks.MLME_BEACON_NOTIFY_indication = &beaconNotifyFrame;
    callbacks.generic_dispatch = &genericDispatchFrame;
    cascoda_register_callbacks(&callbacks);
    
    uint8_t enable = 1;	//enable security
	MLME_SET_request_sync(
		macSecurityEnabled,
		0,
		sizeof(enable),
		&enable,
		pDeviceRef);

	uint8_t defaultKeySource[8] = {0, 0, 0, 0, 0, 0, 0, 0xFF};	//set the defaultKeySource as defined in 7.2.2.1 of thread spec
	MLME_SET_request_sync(
		macDefaultKeySource,
		0,
		8,
		defaultKeySource,
		pDeviceRef);

}

void posixPlatformPostInit(void){
	otSetStateChangedCallback(&keyChangedCallback, NULL);
	otSetStateChangedCallback(&coordChangedCallback, NULL);
}

void coordChangedCallback(uint32_t aFlags, void *aContext) {
	fprintf(stderr,"\n\r Executing coordChangeCallback \n\r");
	if(aFlags & OT_NET_ROLE){
		fprintf(stderr,"\n\r got past role \n\r");
		struct SecSpec securityLevel = {0};
		if(otGetDeviceRole() == kDeviceRoleRouter || otGetDeviceRole() == kDeviceRoleLeader){
			fprintf(stderr,"\n\r I'm a leader or Router \n\r");
			if(!isCoord) {
				fprintf(stderr,"\n\r Changed Coord \n\r");
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
	if((aFlags & OT_NET_KEY_SEQUENCE) || (aFlags & OT_THREAD_CHILD_ADDED) || (aFlags & OT_THREAD_CHILD_REMOVED) || (aFlags & OT_NET_ROLE)){	//The thrKeySequenceCounter has changed
		//Therefore update the keys stored in the macKeytable
		fprintf(stderr, "\n\rUpdating keys\n\r");
		if(otGetKeySequenceCounter() == 0) otSetKeySequenceCounter(2);
		uint32_t tKeySeq = otGetKeySequenceCounter() - 1;
		fprintf(stderr, "-Key Sequence = %#x\n\r", tKeySeq);

		uint8_t count = 0;	//Update device list
		if(otGetDeviceRole() != kDeviceRoleChild){
			fprintf(stderr, "-Looking for children\n\r");
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
				fprintf(stderr, "-Found Valid Child\n\r");

				struct M_DeviceDescriptor tDeviceDescriptor;

				PUTLE16(otGetPanId() ,tDeviceDescriptor.PANId);
				PUTLE16(tChildInfo.mRloc16, tDeviceDescriptor.ShortAddress);
				memcpy(tDeviceDescriptor.ExtAddress, tChildInfo.mExtAddress.m8, 8);
				tDeviceDescriptor.FrameCounter[0] = 0;	//TODO: Figure out how to do frame counter properly - this method is temporarily breaking replay protection as replays using previous key will still be successful
				tDeviceDescriptor.FrameCounter[1] = 0;
				tDeviceDescriptor.FrameCounter[2] = 0;
				tDeviceDescriptor.FrameCounter[3] = 0;
				tDeviceDescriptor.Exempt = 0;

				fprintf(stderr, "-Device Descriptor: ");
				for(int j = 0; j < sizeof(tDeviceDescriptor); j++)fprintf(stderr, "%02x", ((uint8_t*)&tDeviceDescriptor)[j]);

				fprintf(stderr, "\n\r-Error: %#x", MLME_SET_request_sync(
						macDeviceTable,
						count++,
						sizeof(tDeviceDescriptor),
						&tDeviceDescriptor,
						pDeviceRef
						));

			}
		}
		else{
			otRouterInfo tParentInfo;
			fprintf(stderr, "-Looking for parent\n\r");
			if(otGetParent(&tParentInfo) == kThreadError_None){
				struct M_DeviceDescriptor tDeviceDescriptor;

				fprintf(stderr, "-Found Parent\n\r");

				PUTLE16(otGetPanId(), tDeviceDescriptor.PANId);
				PUTLE16(tParentInfo.mRloc16, tDeviceDescriptor.ShortAddress);
				memcpy(tDeviceDescriptor.ExtAddress, tParentInfo.mExtAddress.m8, 8);
				tDeviceDescriptor.FrameCounter[0] = 0;	//TODO: Figure out how to do frame counter properly - this method is temporarily breaking replay protection as replays using previous key will still be successful
				tDeviceDescriptor.FrameCounter[1] = 0;
				tDeviceDescriptor.FrameCounter[2] = 0;
				tDeviceDescriptor.FrameCounter[3] = 0;
				tDeviceDescriptor.Exempt = 0;

				fprintf(stderr, "-Device Descriptor: ");
				for(int j = 0; j < sizeof(tDeviceDescriptor); j++)fprintf(stderr, "%02x", ((uint8_t*)&tDeviceDescriptor)[j]);

				fprintf(stderr, "\n\r-Error: %#x\n\r", MLME_SET_request_sync(
						macDeviceTable,
						count++,
						sizeof(tDeviceDescriptor),
						&tDeviceDescriptor,
						pDeviceRef
						));
			}
			else fprintf(stderr, "\n\r-Error retrieving parent!\n\r");
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
			struct M_KeyDeviceDesc         KeyDeviceList[count];
			struct M_KeyUsageDesc          KeyUsageList[2];
		}tKeyDescriptor;


		//Table 7-5 in section 7.2.2.2.1 of thread spec
		tKeyDescriptor.Fixed.KeyIdLookupListEntries = 1;
		tKeyDescriptor.Fixed.KeyUsageListEntries = 2;
		tKeyDescriptor.Fixed.KeyDeviceListEntries = count;

		//Flags according to Cascoda API 5.3.1
		tKeyDescriptor.KeyUsageList[0].Flags = (MAC_FC_FT_DATA & KUD_FrameTypeMask);	//data usage
		tKeyDescriptor.KeyUsageList[1].Flags = (MAC_FC_FT_COMMAND & KUD_FrameTypeMask) | ((CMD_DATA_REQ << KUD_CommandFrameIdentifierShift) & KUD_CommandFrameIdentifierMask);	//Data req usage

		tKeyDescriptor.KeyIdLookupList[0].LookupDataSizeCode = 1; //1 means length 9
		//This sets the MSB of the lookUpData to equal defaultKeySource as is required by 7.5.8.2.2 of IEEE 15.4 spec
		for(int i = 0; i < 9; i++) tKeyDescriptor.KeyIdLookupList[0].LookupData[i] = 0;
		tKeyDescriptor.KeyIdLookupList[0].LookupData[8] = 0xFF;	//Set lookup data to the macDefaultKeySource to be right concatenated to the individual keyIndex param


		//Fill the deviceListEntries
		for(int i = 0; i < count; i++){
			tKeyDescriptor.KeyDeviceList[i].Flags = i;
		}

		//Generate and store the keys for the current, previous, and next rotations
		count = 0;
		for(uint8_t i = 0; i < 3; i++){
			if((tKeySeq + i) > 0){	//0 is invalid key sequence
				memcpy(tKeyDescriptor.Fixed.Key, getMacKeyFromSequenceCounter(tKeySeq + i), 16);
				tKeyDescriptor.KeyIdLookupList[0].LookupData[0] = ((tKeySeq + i) & 0x7F) + 1;

				fprintf(stderr, "-Key %d i s", tKeySeq + i);
				for(int j = 0; j < 16; j++)fprintf(stderr, "%02x ", tKeyDescriptor.Fixed.Key[j]);
				fprintf(stderr, "\n\r");

				fprintf(stderr, "-Lookup Data: ");
				for(int j = 0; j < 9; j++)fprintf(stderr, "%02x", tKeyDescriptor.KeyIdLookupList[0].LookupData[j]);
				fprintf(stderr, "\n\r");

				fprintf(stderr, "-Error: %#x\n\r", MLME_SET_request_sync(
					macKeyTable,
					count++,
					sizeof(tKeyDescriptor),
					&tKeyDescriptor,
					pDeviceRef
					));
			}
		}
		MLME_SET_request_sync(
			macKeyTableEntries,
			0,
			1,
			&count,
			pDeviceRef
			);

	}
}

ThreadError otPlatRadioEnable(void)    //TODO:(lowpriority) port 
{
    ThreadError error = kThreadError_None;

    VerifyOrExit(sState == kStateDisabled, error = kThreadError_Busy);
    sState = kStateSleep;

	#ifdef EXECUTE_MODE
    	uint8_t HWMEAttVal = 0; //0x00
    	if (HWME_SET_request_sync (
    		HWME_POWERCON,
    		1,
    		&HWMEAttVal,
    		pDeviceRef
    		) == HWME_SUCCESS)
    		return kThreadError_None;
        else return kThreadError_Failed;

	#endif


exit:
    return error;
}

ThreadError otPlatRadioDisable(void)    //TODO:(lowpriority) port 
{
    ThreadError error = kThreadError_None;

    VerifyOrExit(sState == kStateIdle, error = kThreadError_Busy);
    sState = kStateDisabled;

    //should sleep until restarted
	#ifdef EXECUTE_MODE
    	uint8_t HWMEAttVal = 10; //0x0A
    	if (HWME_SET_request_sync (
    		HWME_POWERCON,
    		1,
    		&HWMEAttVal,
    		pDeviceRef
    		) == HWME_SUCCESS)
	    	return kThreadError_None;
	    else return kThreadError_Failed;

	#endif

exit:
    return error;
}

ThreadError otPlatRadioSleep(void)    //TODO:(lowpriority) port 
{
    ThreadError error = kThreadError_None;

    VerifyOrExit(error = kStateIdle, error = kThreadError_Busy);
    sState = kStateSleep;

	#ifdef EXECUTE_MODE
    	uint8_t HWMEAttVal = 42; //0x2A
		if (HWME_SET_request_sync (
			HWME_POWERCON,
			1,
			&HWMEAttVal,
			pDeviceRef
			) == HWME_SUCCESS)
			return kThreadError_None;
	    else return kThreadError_Failed;

	#endif

exit:
    return error;
}

ThreadError otPlatRadioIdle(void)    //TODO:(lowpriority) port 
{
    ThreadError error = kThreadError_None;

    switch (sState)
    {
    case kStateSleep:
        sState = kStateIdle;
        break;

    case kStateIdle:
        break;

    case kStateListen:
    case kStateTransmit:
        disableReceiver();
        sState = kStateIdle;
        break;

    case kStateReceive:
    case kStateDisabled:
        ExitNow(error = kThreadError_Busy);
        break;
    }

	#ifdef EXECUTE_MODE
    	uint8_t HWMEAttVal = 36; //0x24
		if (HWME_GET_request_sync (
			HWME_POWERCON,
			1,
			&HWMEAttVal,
			pDeviceRef
			) == HWME_SUCCESS)
			return kThreadError_None;
		else return kThreadError_Failed;

	#endif

exit:
    return error;
}
ThreadError otPlatRadioReceive(uint8_t aChannel)
{
    ThreadError error = kThreadError_None;

    VerifyOrExit(sState == kStateIdle, error = kThreadError_Busy);
    sState = kStateListen;

    setChannel(aChannel);

    enableReceiver();

exit:
    return error;
}

RadioPacket *otPlatRadioGetTransmitBuffer(void)
{
    return &sTransmitFrame;
}

ThreadError otPlatRadioTransmit(void)
{
    ThreadError error = kThreadError_None;
    int i;
    static uint8_t handle = 0;
    handle++;

    VerifyOrExit(sState == kStateIdle, error = kThreadError_Busy);
    uint16_t frameControl = GETLE16(sTransmitFrame.mPsdu);
    VerifyOrExit((frameControl & MAC_FC_FT_MASK) == MAC_FC_FT_DATA, error = kThreadError_Abort);

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
    enableReceiver();

    //transmit
    struct MCPS_DATA_request_pset curPacket;
    struct SecSpec curSecSpec = {0};

    uint8_t headerLength = 0;
    uint8_t footerLength = 0;

    curPacket.SrcAddrMode = MAC_FC_SAM(frameControl);
    curPacket.Dst.AddressMode = MAC_FC_DAM(frameControl);
    curPacket.TxOptions = (frameControl & MAC_FC_ACK_REQ) ? 0x01 : 0x00;
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

    if(curPacket.SrcAddrMode == MAC_MODE_SHORT_ADDR) addressFieldLength += 4;
    else if(curPacket.SrcAddrMode == MAC_MODE_LONG_ADDR) addressFieldLength += 10;
    if(curPacket.SrcAddrMode && isPanCompressed) addressFieldLength -= 2; //Remove size saved by not including the same PAN twice
    headerLength = addressFieldLength + MAC_BASEHEADERLENGTH;

    if(frameControl & MAC_FC_SEC_ENA){	//if security is required
    	uint8_t ASHloc = MAC_BASEHEADERLENGTH + addressFieldLength;
    	uint8_t securityControl = *(uint8_t*)(sTransmitFrame.mPsdu + ASHloc);
    	fprintf(stderr, "\r\nSecurity Control: %#04x", securityControl);
    	curSecSpec.SecurityLevel = MAC_SC_SECURITYLEVEL(securityControl);
    	fprintf(stderr, "\r\nSecurity Level: %#04x", curSecSpec.SecurityLevel);
    	curSecSpec.KeyIdMode = MAC_SC_KEYIDMODE(securityControl);
    	fprintf(stderr, "\r\nKeyIDMode: %#04x\r\n", curSecSpec.KeyIdMode);

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
    	fprintf(stderr, "\r\nKeyIndex: %#04x\r\n", curSecSpec.KeyIndex);
    	headerLength = ASHloc;
    }

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
        curPacket.Dst.Address,
        curPacket.MsduLength,
        curPacket.Msdu,
        curPacket.MsduHandle,
        curPacket.TxOptions,
        &curSecSpec,
        pDeviceRef);

    //PlatformRadioProcess();

exit:

    if (sTransmitError != kThreadError_None || (sTransmitFrame.mPsdu[0] & IEEE802154_ACK_REQUEST) == 0)
    {
        disableReceiver();
    }

    return error;
}

int8_t otPlatRadioGetNoiseFloor(void)    //TODO:(lowpriority) port 
{
    return 0;
}

otRadioCaps otPlatRadioGetCaps(void)    //TODO:(lowpriority) port 
{
    return kRadioCapsNone;
}

bool otPlatRadioGetPromiscuous(void)
{
	uint8_t resultLen;
    uint8_t result;
    MLME_GET_request_sync(
        macPromiscuousMode,
        0,
        &resultLen,
        &result,
        pDeviceRef);
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

}

void readFrame(struct MCPS_DATA_indication_pset *params)   //Async
{


    //VerifyOrExit(sState == kStateListen, fputs("\r\nNot Listening!\r\n", stderr));

    pthread_mutex_lock(&receiveFrame_mutex);
	//wait until the main thread is free to process the frame
	while(sReceiveFrame.mLength != 0) {pthread_cond_wait(&receiveFrame_cond, &receiveFrame_mutex);}

	uint8_t headerLength = 0;
	uint8_t footerLength = 0;
	uint16_t frameControl = 0;
	uint8_t msduLength = params->MsduLength;
	struct SecSpec * curSecSpec = (struct SecSpec*)((unsigned int)params + (unsigned int)msduLength + 29); //Location defined in cascoda API docs

	//TODO: Move this
	#define CASCODA_DATAIND_SEC_LOC 29

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
	memcpy(sReceiveFrame.mPsdu + headerLength, params->Msdu, params->MsduLength);
	sReceiveFrame.mLqi = params->MpduLinkQuality;
	sReceiveFrame.mChannel = sChannel;
	sReceiveFrame.mPower = -20;

	/*
	fputs("\r\nReceived:",stderr);
	for(int i = 0; i < sReceiveFrame.mLength; i++){
		fprintf(stderr, " %#04x", sReceiveFrame.mPsdu[i]);
	}
	fputs("\r\n",stderr);
	*/

    pthread_mutex_unlock(&receiveFrame_mutex);

	sState = kStateIdle;
	otPlatRadioReceiveDone(&sReceiveFrame, sReceiveError);

    PlatformRadioProcess();

exit:
    return;
}

void readConfirmFrame(struct MCPS_DATA_confirm_pset *params)   //Async
{


    //VerifyOrExit(sState == kStateTransmit, ;);

    if(params->Status == MAC_SUCCESS){
    	otPlatRadioTransmitDone(false, sTransmitError);
    }
    else{
    	if(params->Status == MAC_CHANNEL_ACCESS_FAILURE) sTransmitError = kThreadError_ChannelAccessFailure;
    	else if(params->Status == MAC_NO_ACK) sTransmitError = kThreadError_NoAck;
    	else sTransmitError = kThreadError_Abort;
    	sState = kStateIdle;
    	fprintf(stderr, "\n\rMCPS_DATA_confirm error: %#x \r\n", params->Status);
    	otPlatRadioTransmitDone(false, sTransmitError);
    }

    sTransmitError = kThreadError_None;

    PlatformRadioProcess();

exit:
    return;
}

void beaconNotifyFrame(struct MLME_BEACON_NOTIFY_indication_pset *params)
{
	otActiveScanResult resultStruct;
	fprintf(stderr,"I'm getting a beaconNotifyFrame");
	uint8_t shortaddrs  = *((uint8_t*)params + 33) & 7;
	uint8_t extaddrs = (*((uint8_t*)params + 33) & 112) >> 4;

	if ((params->PanDescriptor.Coord.AddressMode) == 3) {
		memcpy(resultStruct.mExtAddress.m8, params->PanDescriptor.Coord.Address, 8);
	} else {
		assert(false);
	}
	resultStruct.mPanId = GETLE16(params->PanDescriptor.Coord.PANId);
	resultStruct.mChannel = params->PanDescriptor.LogicalChannel;
	resultStruct.mRssi = -20;
	resultStruct.mLqi = params->PanDescriptor.LinkQuality;
	uint8_t *sduLength = params + (34 + 2 * shortaddrs + 8 * extaddrs);
	if (*sduLength > 0) {
		uint8_t *Sdu = params + (35 + 2 * shortaddrs + 8 * extaddrs);
		if(*Sdu == 3 && (*(Sdu + 1) == 1)) {
			resultStruct.mNetworkName = Sdu + 4;
			resultStruct.mExtPanId = Sdu + 20;
			scanCallback(&resultStruct);
		}
	}

exit:
    return;
}

int genericDispatchFrame(const uint8_t *buf, size_t len) {
	fprintf(stderr, "\n\rGoing into a for loop now\n\r");
	for(int i = 0; i < len; i++) {
		fprintf(stderr, " %x ", buf[i]);
	}
}

int PlatformRadioProcess(void)    //TODO: port - This should be the callback in future for data receive

{
	pthread_mutex_lock(&receiveFrame_mutex);

    sReceiveFrame.mLength = 0;
    pthread_cond_broadcast(&receiveFrame_cond);
    pthread_mutex_unlock(&receiveFrame_mutex);

    if (sState == kStateIdle)
    {
        disableReceiver();
    }

    return 0;
}

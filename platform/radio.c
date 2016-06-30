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
 * ll
 */

#include <openthread-types.h>

#include <stdio.h>

#include <common/code_utils.hpp>
#include <platform/radio.h>
#include <cascoda_api.h>
#include <mac_messages.h>
#include "posix-platform.h"
#include <ieee_802_15_4.h>
#include <pthread.h>

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

static struct MAC_Message response;
static RadioPacket sTransmitFrame;
static RadioPacket sReceiveFrame;
static ThreadError sTransmitError;
static ThreadError sReceiveError;

static void* pDeviceRef = NULL;

static uint8_t sChannel = 0;

static uint8_t sTransmitPsdu[IEEE802154_MAX_LENGTH];
static uint8_t sReceivePsdu[IEEE802154_MAX_LENGTH];

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
        TDME_ChannelInit(channel, pDeviceRef);
        sChannel = channel;
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
    fputs("Initialising radio...", stderr);
    sTransmitFrame.mLength = 0;
    sTransmitFrame.mPsdu = sTransmitPsdu;
    
    fputs("attempting to aquire mutex...", stderr);
    pthread_mutex_lock(&receiveFrame_mutex);
		sReceiveFrame.mLength = 0;
		sReceiveFrame.mPsdu = sReceivePsdu;
		pthread_cond_broadcast(&receiveFrame_cond);
	pthread_mutex_unlock(&receiveFrame_mutex);
    
    fputs("Initiating kernal exchange...", stderr);

    kernel_exchange_init();

    fputs("Initialising callbacks", stderr);

    struct cascoda_api_callbacks callbacks;
    callbacks.MCPS_DATA_indication = &readFrame;
    callbacks.MCPS_DATA_confirm = &readConfirmFrame;
    cascoda_register_callbacks(&callbacks);
    
    fputs("Initialising Chip...", stderr);
    TDME_ChipInit(pDeviceRef);
    fputs("Radio Done!", stderr);
}

ThreadError otPlatRadioEnable(void)    //TODO:(lowpriority) port 
{
    ThreadError error = kThreadError_None;

    VerifyOrExit(sState == kStateDisabled, error = kThreadError_Busy);
    sState = kStateSleep;

exit:
    return error;
}

ThreadError otPlatRadioDisable(void)    //TODO:(lowpriority) port 
{
    ThreadError error = kThreadError_None;

    VerifyOrExit(sState == kStateIdle, error = kThreadError_Busy);
    sState = kStateDisabled;

exit:
    return error;
}

ThreadError otPlatRadioSleep(void)    //TODO:(lowpriority) port 
{
    ThreadError error = kThreadError_None;

    VerifyOrExit(error = kStateIdle, error = kThreadError_Busy);
    sState = kStateSleep;

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

exit:
    return error;
}

ThreadError otPlatRadioReceive(uint8_t aChannel)
{
    ThreadError error = kThreadError_None;

    VerifyOrExit(sState == kStateIdle, error = kThreadError_Busy);
    sState = kStateListen;

    setChannel(aChannel);

    pthread_mutex_lock(&receiveFrame_mutex);
    	sReceiveFrame.mLength = 0;
    	sReceiveFrame.mChannel = aChannel;
    	pthread_cond_broadcast(&receiveFrame_cond);
    pthread_mutex_unlock(&receiveFrame_mutex);

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

    VerifyOrExit(sState == kStateIdle, error = kThreadError_Busy);
    sState = kStateTransmit;
    sTransmitError = kThreadError_None;

    setChannel(sTransmitFrame.mChannel);
    enableReceiver();

    fprintf(stderr, "\n\n\nPacket Sending: ");
    for(int i = 0; i < sTransmitFrame.mLength; i++){
	fprintf(stderr, "%#04x ", sTransmitFrame.mPsdu[i]);
    
    }

    //TODO: Move these
	#define MAC_SC_SECURITYLEVEL(sc) (sc&0x07)
	#define MAC_SC_KEYIDMODE(sc) ((sc>>3)&0x03)

    //transmit
    struct MCPS_DATA_request_pset curPacket;
    struct SecSpec curSecSpec = {0};

    uint16_t frameControl = GETLE16(sTransmitFrame.mPsdu);
    curPacket.SrcAddrMode = MAC_FC_SAM(frameControl);
    curPacket.Dst.AddressMode = MAC_FC_DAM(frameControl);
    curPacket.TxOptions = frameControl & MAC_FC_ACK_REQ ? 0x01 : 0x00;

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

    if(curPacket.SrcAddrMode == MAC_MODE_SHORT_ADDR) addressFieldLength +=4;
    else if(curPacket.SrcAddrMode == MAC_MODE_LONG_ADDR) addressFieldLength +=10;

    if(frameControl & MAC_FC_SEC_ENA){	//if security is required
    	uint8_t ASHloc = 3 + addressFieldLength;
    	uint8_t securityControl = sTransmitFrame.mPsdu + ASHloc;
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
    	curSecSpec.KeyIndex = sTransmitFrame.mPsdu[ASHloc];
    }

    //Todo: MSDU + handle & Length

    MCPS_DATA_request(
        curPacket->SrcAddrMode,
        curPacket->Dst.AddressMode,
        curPacket->Dst.PANId,
        curPacket->Dst.Address,
        curPacket->MsduLength,
        curPacket->Msdu,
        curPacket->MsduHandle,
        curPacket->TxOptions,
        0,
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
    bool * result;
    MLME_GET_request_sync(
        macPromiscuousMode,
        0,
        NULL,
        result,
        pDeviceRef);
    return * result;
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


    VerifyOrExit(sState == kStateListen, ;);

    pthread_mutex_lock(&receiveFrame_mutex);
    	//wait until the main thread is free to process the frame
    	while(sReceiveFrame.mLength != 0) {pthread_cond_wait(&receiveFrame_cond, &receiveFrame_mutex);}

		sReceiveFrame.mPsdu = params;
		sReceiveFrame.mLength = params->MsduLength + 40;
		sReceiveFrame.mLqi = params->MpduLinkQuality;
		sReceiveFrame.mChannel = sChannel;
		//sReceiveFrame.mPower = -20;
    pthread_mutex_unlock(&receiveFrame_mutex);

    PlatformRadioProcess();

exit:
    return;
}

void readConfirmFrame(struct MCPS_DATA_confirm_pset *params)   //Async
{


    VerifyOrExit(sState == kStateTransmit, ;);

    pthread_mutex_lock(&receiveFrame_mutex);
    	//wait until the main thread is free to process the frame
    	while(sReceiveFrame.mLength != 0) {pthread_cond_wait(&receiveFrame_cond, &receiveFrame_mutex);}

		sReceiveFrame.mPsdu = params;
		sReceiveFrame.mLength = 6;
		sReceiveFrame.mChannel = sChannel;
		//sReceiveFrame.mPower = -20;
    pthread_mutex_unlock(&receiveFrame_mutex);

    PlatformRadioProcess();

exit:
    return;
}

int PlatformRadioProcess(void)    //TODO: port - This should be the callback in future for data receive
{
	fputs("Grabbing receiveFrame mutex...",	 stderr);
	pthread_mutex_lock(&receiveFrame_mutex);
	fputs("Got it!", stderr);
    switch (sState)
    {
    case kStateDisabled:
        break;

    case kStateSleep:
        break;

    case kStateIdle:
        break;

    case kStateListen:
    case kStateReceive:
    	fputs("(kStateListen)", stderr);
        if (sReceiveFrame.mLength > 0)
        {
            sState = kStateIdle;
            otPlatRadioReceiveDone(&sReceiveFrame, sReceiveError);
        }
        break;

    case kStateTransmit:
    	fputs("(kStateTransmit)", stderr);
        if (sTransmitError != kThreadError_None || (sTransmitFrame.mPsdu[0] & IEEE802154_ACK_REQUEST) == 0)
        {
            sState = kStateIdle;
            otPlatRadioTransmitDone(false, sTransmitError);
        }
        else if (sReceiveFrame.mLength == IEEE802154_ACK_LENGTH &&
                 (sReceiveFrame.mPsdu[0] & IEEE802154_FRAME_TYPE_MASK) == IEEE802154_FRAME_TYPE_ACK &&
                 (sReceiveFrame.mPsdu[IEEE802154_DSN_OFFSET] == sTransmitFrame.mPsdu[IEEE802154_DSN_OFFSET]))
        {
            sState = kStateIdle;
            otPlatRadioTransmitDone((sReceiveFrame.mPsdu[0] & IEEE802154_FRAME_PENDING) != 0, sTransmitError);
        }

        break;
    }

    sReceiveFrame.mLength = 0;

    fputs("Releasing mutex...", stderr);
    pthread_cond_broadcast(&receiveFrame_cond);
    pthread_mutex_unlock(&receiveFrame_mutex);
    fputs("Released!", stderr);

    if (sState == kStateIdle)
    {
        disableReceiver();
    }

    return 0;
}

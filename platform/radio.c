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
#include <kernel_exchange.h>
#include <string.h>
#include <mac_messages.h>
#include "posix-platform.h"
#include <ieee_802_15_4.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/stat.h>

#include "selfpipe.h"

//Mac tools
#define MAC_SC_SECURITYLEVEL(sc) (sc&0x07)
#define MAC_SC_KEYIDMODE(sc) ((sc>>3)&0x03)
#define MAC_KEYIDMODE_SC(keyidmode) ((keyidmode&0x03)<<3)
#define MAC_BASEHEADERLENGTH 3
#define MAX_DYNAMIC_DEVICES ( DEVICE_TABLE_SIZE - 1 )

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

static otInstance *OT_INSTANCE;

//CASCODA API CALLBACKS
static int handleDataIndication(struct MCPS_DATA_indication_pset *params);
static int handleDataConfirm(struct MCPS_DATA_confirm_pset *params);

static int handleBeaconNotify(struct MLME_BEACON_NOTIFY_indication_pset *params);
static int handleScanConfirm(struct MLME_SCAN_confirm_pset *params);

static int handleGenericDispatchFrame(const uint8_t *buf, size_t len);
//END CASCODA API CALLBACKS

//OPENTHREAD API CALLBACKS
static void keyChangeCallback(uint32_t aFlags, otInstance *aInstance);
static void coordChangeCallback(uint32_t aFlags, otInstance *aInstance);
//END OPENTHREAD API CALLBACKS

//DEVICE FRAME COUNTER CACHING
/*
 * The following functions and data create a system for monitoring the frame
 * counter values of connected devices from the device tree. This is used in
 * practice to check the activity of rx-off-when-idle children, who only need
 * to send polls in order to remain active and not be timed out. (If there is no
 * data for that child, there is no effect of these other than to increment the
 * frame counter)
 */
struct DeviceCache
{
	uint8_t			isActive;
	otExtAddress 	mExtAddr;
	uint8_t 		mFrameCounter[4];			//up to date frame counter
	uint8_t 		mTimeoutFrameCounter[4];	//Previous version
};

static struct DeviceCache sDeviceCache[DEVICE_TABLE_SIZE] = {{0}};

static uint8_t sCurDeviceTableSize = 0;
static void deviceCache_cacheDevices(void);
static struct DeviceCache *deviceCache_getCachedDevice(otExtAddress addr);
//END DEVICE FRAME COUNTER CACHING

//KEY MODE 2 DATA
static const uint8_t sMode2Key[] =
{
	0x78, 0x58, 0x16, 0x86, 0xfd, 0xb4, 0x58, 0x0f,
	0xb0, 0x92, 0x54, 0x6a, 0xec, 0xbd, 0x15, 0x66
};

static const otExtAddress sMode2ExtAddress =
{
	{ 0x35, 0x06, 0xfe, 0xb8, 0x23, 0xd4, 0x87, 0x12 },
};

static uint8_t sMode2DeviceIndex = 100; //location of Mode 2 device in PIB
//END KEY MODE 2 DATA

//COMMISSIONING & JOINING
static uint8_t sKekInUse = 0;
static uint8_t sKekDeviceIndex = 0;
static uint8_t sKekCounterpart[8];
static uint8_t sKekMessageHandle = 0;
//END COMMISSIONING & JOINING

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

//INTRANSIT
/*
 * The following functions exist to create a queue for in-transit data packets.
 * The complete frames are not stored, only the header information required to
 * process the completion of the transmission. This is because it is not
 * guaranteed for these frames to transmit in order, and the destination address
 * is required for post-transmission processing. Access to this system must be
 * protected by the barrier.
 */
static otRadioFrame *intransit_getFrame(uint8_t handle);
static int intransit_rmFrame(uint8_t handle);
static int intransit_purge(void * aSender);
static int intransit_putFrame(uint8_t handle, const otRadioFrame *in);
static int intransit_isHandleInUse(uint8_t handle);

#define MAX_INTRANSITS 7 /*TODO:  5 indirect frames +2 (perhaps need more?)*/
uint8_t IntransitHandles[MAX_INTRANSITS] = {0};
otRadioFrame IntransitPackets[MAX_INTRANSITS];
//END INTRANSIT

//FRAME DATA
static otRadioFrame sTransmitFrame;
static otError sTransmitError;
static uint8_t sTransmitPsdu[IEEE802154_MAX_LENGTH];
//END FRAME DATA

//SCAN DATA
static otHandleActiveScanResult sActiveScanCallback;
static void *sActiveScanContext;
static uint8_t sActiveScanInProgress = 0;

static otHandleEnergyScanResult sEnergyScanCallback;
static void *sEnergyScanContext;
static uint8_t sEnergyScanInProgress = 0;
static uint32_t sEnergyScanMask = 0;
//END SCAN DATA

//For cascoda API
static void *pDeviceRef = NULL;

static uint8_t sChannel = 0;

//Caching for values that may or may not be set on chip
enum tristateCache {
	tristate_disabled = 0, tristate_enabled = 1, tristate_uninit = 2
};

//Cached value for promiscuity
static enum tristateCache sPromiscuousCache = tristate_uninit;

//Cached current coordinator status
static uint8_t sIsCoordinator = 0;

#define IEEEEUI_FILE "/usr/local/etc/.otEui"
static uint8_t sIeeeEui64[8];

//BEACON DATA
#define BEACON_PAYLOAD_LENGTH 26
#define BEACON_JOIN_BIT (1 << 3)
static const uint8_t kBeaconPayloadLength = BEACON_PAYLOAD_LENGTH;
static uint8_t mBeaconPayload[BEACON_PAYLOAD_LENGTH] = {3, 0x20};
//END BEACON DATA

static int8_t noiseFloor = 127;

static otRadioState sState;

static void setChannel(uint8_t channel)
{
	if (sChannel != channel)
	{
		MLME_SET_request_sync(
		    phyCurrentChannel,
		    0,
		    sizeof(channel),
		    &channel,
		    pDeviceRef);
		sChannel = channel;
	}
}


void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
	(void) aInstance;
	(void) aEnable;
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
	(void) aInstance;
	(void) aShortAddress;
	false;
	return OT_ERROR_NONE;
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
	(void) aInstance;
	(void) aExtAddress;
	return OT_ERROR_NONE;
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
	(void) aInstance;
	(void) aShortAddress;
	return OT_ERROR_NONE;
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
	(void) aInstance;
	(void) aExtAddress;
	return OT_ERROR_NONE;
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
	(void) aInstance;
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
	(void) aInstance;
}

//Platform independent integer log2 implementation
//from http://stackoverflow.com/questions/11376288/fast-computing-of-log2-for-64-bit-integers
const uint8_t tab32[32] =
{
	0,  9,  1, 10, 13, 21,  2, 29,
	11, 14, 16, 18, 22, 25,  3, 30,
	8, 12, 20, 28, 15, 17, 24,  7,
	19, 27, 23,  6, 26,  5,  4, 31
};

uint8_t log2_32(uint32_t value)
{
	value |= value >> 1;
	value |= value >> 2;
	value |= value >> 4;
	value |= value >> 8;
	value |= value >> 16;
	return tab32[(uint32_t)(value * 0x07C4ACDD) >> 27];
}

otError otPlatRadioActiveScan(otInstance *aInstance, uint32_t aScanChannels,
		                          uint16_t aScanDuration, otHandleActiveScanResult aCallback,
		                          void *aCallbackContext)
{
	/*
	 * This function causes an active scan to be run by the ca8210
	 */
	if (sActiveScanInProgress || sEnergyScanInProgress)
	{
		return OT_ERROR_BUSY;
	}

	uint8_t ScanDuration;

	//This 'if' is to cope with the aScanDurations of 0 that get passed around
	if (aScanDuration >= 50)
	{
		//15 ~= (aBaseSuperframeDuration * aSymbolPeriod_us)/1000
		ScanDuration = log2_32(aScanDuration / 15); //5; //0 to 14

		if (ScanDuration > 14)
		{
			ScanDuration = 14;
		}
	}
	else
	{
		ScanDuration = 4;
	}

	otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_HARDMAC, "aScanDuration: %d, ScanDuration: %d\n\r", aScanDuration, ScanDuration);

	struct SecSpec pSecurity = {0};

	if (aScanChannels == 0)
	{
		aScanChannels = 0x07fff800;    //11 to 26
	}

	sActiveScanCallback = aCallback;
	sActiveScanContext = aCallbackContext;
	uint8_t scanRequest = MLME_SCAN_request(
	                          1, //Active Scan
	                          aScanChannels,
	                          ScanDuration,
	                          &pSecurity,
	                          pDeviceRef);

	if (scanRequest == MAC_SUCCESS)
	{
		sActiveScanInProgress = 1;
		return OT_ERROR_NONE;
	}
	else
	{
		return OT_ERROR_BUSY;
	}
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel,
		                          uint16_t aScanDuration)
{
	return OT_ERROR_NOT_IMPLEMENTED;	//We have a hardmac: Use the full one
}

otError otPlatRadioEnergyScanFull(otInstance *aInstance, uint32_t aScanChannels,
		                              uint16_t aScanDuration, otHandleEnergyScanResult aCallback,
		                              void *aCallbackContext)
{

	/*
	 * This function causes an energy scan to be run by the ca8210
	 */
	if (sActiveScanInProgress || sEnergyScanInProgress)
	{
		return OT_ERROR_BUSY;
	}

	uint8_t ScanDuration;

	sEnergyScanMask = aScanChannels;

	//This 'if' is to cope with the aScanDurations of 0 that get passed around
	if (aScanDuration >= 50)
	{
		//15 ~= (aBaseSuperframeDuration * aSymbolPeriod_us)/1000
		ScanDuration = log2_32(aScanDuration / 15); //6; //0 to 14

		if (ScanDuration > 14)
		{
			ScanDuration = 14;
		}
	}
	else
	{
		ScanDuration = 6;
	}

	otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_HARDMAC, "aScanDuration: %d, ScanDuration: %d\n\r", aScanDuration, ScanDuration);

	struct SecSpec pSecurity = {0};

	if (aScanChannels == 0)
	{
		aScanChannels = 0x07fff800;    //11 to 26
	}

	sEnergyScanCallback = aCallback;
	sEnergyScanContext = aCallbackContext;
	uint8_t scanRequest = MLME_SCAN_request(
	                          0, //Energy Scan
	                          aScanChannels,
	                          ScanDuration,
	                          &pSecurity,
	                          pDeviceRef);

	if (scanRequest == MAC_SUCCESS)
	{
		sEnergyScanInProgress = 1;
		return OT_ERROR_NONE;
	}
	else
	{
		return OT_ERROR_BUSY;
	}
}

bool otPlatRadioIsEnergyScanInProgress(otInstance *aInstance)
{
	return sEnergyScanInProgress;
}

bool otPlatRadioIsActiveScanInProgress(otInstance *aInstance)
{
	return sActiveScanInProgress;
}

otError updateBeacon(){
	if ((MLME_SET_request_sync(
			 macBeaconPayload,
			 0,
			 kBeaconPayloadLength,
			 mBeaconPayload,
			 pDeviceRef) == MAC_SUCCESS) &&
		(MLME_SET_request_sync(
			 macBeaconPayloadLength,
			 0,
			 1,
			 &kBeaconPayloadLength,
			 pDeviceRef) == MAC_SUCCESS))
	{
		return OT_ERROR_NONE;
	}
	else
	{
		return OT_ERROR_FAILED;
	}
}

otError otPlatRadioSetNetworkName(otInstance *aInstance, const char *aNetworkName)
{

	memcpy(mBeaconPayload + 2, aNetworkName, 16);

	return updateBeacon();
}

otError otPlatRadioSetJoiningEnabled(otInstance *aInstance, uint8_t isEnabled)
{

	if (isEnabled)
	{
		mBeaconPayload[1] |= BEACON_JOIN_BIT;
	}
	else
	{
		mBeaconPayload[1] &= ~BEACON_JOIN_BIT;
	}

	return updateBeacon();

}

otError otPlatRadioSetExtendedPanId(otInstance *aInstance, const uint8_t *aExtPanId)
{

	memcpy(mBeaconPayload + 18, aExtPanId, 8);

	return updateBeacon();
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t panid)
{
	uint8_t LEarray[2];
	LEarray[0] = LS0_BYTE(panid);
	LEarray[1] = LS1_BYTE(panid);
	MLME_SET_request_sync(
	    macPANId,
	    0,
	    sizeof(panid),
	    LEarray,
	    pDeviceRef);
}

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
	(void) aInstance;

	for (int i = 0; i < 8; i++)
	{
		aIeeeEui64[i] = sIeeeEui64[i];
	}
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, uint8_t *address)
{
	MLME_SET_request_sync(
	    nsIEEEAddress,
	    0,
	    OT_EXT_ADDRESS_SIZE,
	    address,
	    pDeviceRef);
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance){
	return -105;
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t address)
{
	uint8_t LEarray[2];
	LEarray[0] = LS0_BYTE(address);
	LEarray[1] = LS1_BYTE(address);
	MLME_SET_request_sync(
	    macShortAddress,
	    0,
	    sizeof(address),
	    LEarray,
	    pDeviceRef);
}

void otPlatRadioSetDefaultTxPower(otInstance *aInstance, int8_t aPower)
{
	// TODO: Create a proper implementation for this driver.
	(void)aInstance;
	(void)aPower;
}

static int driverErrorCallback(int error_number)
{
	otPlatLog(OT_LOG_LEVEL_CRIT, OT_LOG_REGION_HARDMAC, "DRIVER FAILED WITH ERROR %d\n\r", error_number);
	//TODO: Fail gracefully
	abort();
	return 0;
}

void PlatformRadioStop(void)
{
	//Reset the MAC to a default state
	otPlatLog(OT_LOG_LEVEL_INFO, OT_LOG_REGION_HARDMAC, "Resetting & Stopping Radio...\n\r");
	MLME_RESET_request_sync(1, pDeviceRef);
}

void otPlatRadioDumpPib(void){
	//can be used as a debug hook
	return;
}

void initIeeeEui64(){
	int file;
	uint8_t create = false;
	uint8_t ret = 0;

	if (!access(IEEEEUI_FILE, R_OK))
	{
		file = open(IEEEEUI_FILE, O_RDONLY);
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
		file = open(IEEEEUI_FILE, O_RDWR | O_CREAT, 0666);
		for (int i = 0; i < 4; i += 1)
		{
			uint16_t random = otPlatRandomGet();
			sIeeeEui64[2 * i] = random & 0xFF;
			sIeeeEui64[2 * i + 1] = (random >> 4) & 0xFF;
		}
		sIeeeEui64[0] |= 2; //Set local bit
		write(file, sIeeeEui64, 8);
	}

	close(file);
	otPlatRadioSetExtendedAddress(NULL, sIeeeEui64);
}

void PlatformRadioInit(void)
{
	sTransmitFrame.mLength = 0;
	sTransmitFrame.mPsdu = sTransmitPsdu;

	atexit(&PlatformRadioStop);

	selfpipe_init();

	kernel_exchange_init_withhandler(driverErrorCallback);

	struct ca821x_api_callbacks callbacks = {0};
	callbacks.MCPS_DATA_indication = &handleDataIndication;
	callbacks.MCPS_DATA_confirm = &handleDataConfirm;
	callbacks.MLME_BEACON_NOTIFY_indication = &handleBeaconNotify;
	callbacks.MLME_SCAN_confirm = &handleScanConfirm;
	callbacks.generic_dispatch = &handleGenericDispatchFrame;
	ca821x_register_callbacks(&callbacks);

	//Reset the MAC to a default state
	MLME_RESET_request_sync(1, pDeviceRef);

	uint8_t disable = 0; //Disable low LQI rejection @ MAC Layer
	HWME_SET_request_sync(0x11, 1, &disable, pDeviceRef);

	uint8_t enable = 1;	//enable security
	MLME_SET_request_sync(
	    macSecurityEnabled,
	    0,
	    sizeof(enable),
	    &enable,
	    pDeviceRef);

	uint8_t retries = 3;	//Retry transmission 3 times if not acknowledged
	MLME_SET_request_sync(
	    macMaxFrameRetries,
	    0,
	    sizeof(retries),
	    &retries,
	    pDeviceRef);

	retries = 4;	//max 4 CSMA backoffs
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

	//set the defaultKeySource as defined in 7.2.2.1 of thread spec
	uint8_t defaultKeySource[8] = {0, 0, 0, 0, 0, 0, 0, 0xFF};
	MLME_SET_request_sync(
	    macDefaultKeySource,
	    0,
	    8,
	    defaultKeySource,
	    pDeviceRef);

	//LQI values should be derived from receive energy
	uint8_t LQImode = HWME_LQIMODE_ED;
	HWME_SET_request_sync(
	    HWME_LQIMODE,
	    1,
	    &LQImode,
	    pDeviceRef);

	//Indirect transmissions should wait 90 seconds before timing out
	uint8_t persistanceTime[2];
	PUTLE16(0x16e3, persistanceTime); //=(90seconds * 10^6)/(aBaseSuperframeDuration * aSymbolPeriod_us)
	MLME_SET_request_sync(
	    macTransactionPersistenceTime,
	    0,
	    2,
	    persistanceTime,
	    pDeviceRef);

	initIeeeEui64();
}

void otHardMacStateChangeCallback(otInstance *aInstance, uint32_t aFlags, void *aContext)
{

	/*
	 * This function is called whenever there is an internal state change in thread
	 */

	keyChangeCallback(aFlags, aInstance);
	coordChangeCallback(aFlags, aInstance);
}

static void coordChangeCallback(uint32_t aFlags, otInstance *aInstance)
{

	/*
	 * This function sets the node as an 802.15.4 coordinator when it is set to
	 * act as a thread router, in order that is can respond to active scan
	 * beacon requests with beacons.
	 */
	struct SecSpec securityLevel = {0};
	otDeviceRole role;

	VerifyOrExit(aFlags & OT_NET_ROLE, );

	role = otThreadGetDeviceRole(aInstance);

	if (role == OT_DEVICE_ROLE_ROUTER || role == OT_DEVICE_ROLE_LEADER)
	{
		if (!sIsCoordinator)
		{
			MLME_START_request_sync(
				otLinkGetPanId(aInstance),
				sChannel,
				15,
				15,
				1,
				0,
				0,
				&securityLevel,
				&securityLevel,
				pDeviceRef);
			sIsCoordinator = 1;
		}
	}
	else if (sIsCoordinator)
	{
		MLME_RESET_request_sync(0, pDeviceRef);
		sIsCoordinator = 0;
	}

exit:
	return;
}


static void resetMode2Device()
{
	struct M_DeviceDescriptor tDeviceDescriptor;

	PUTLE16(0xFFFF, tDeviceDescriptor.PANId);
	PUTLE16(0xFFFF, tDeviceDescriptor.ShortAddress);

	for (int j = 0; j < 8; j++)
	{
		tDeviceDescriptor.ExtAddress[j] = sMode2ExtAddress.m8[7 - j]; //Flip endian
	}

	tDeviceDescriptor.FrameCounter[0] = 0;
	tDeviceDescriptor.FrameCounter[1] = 0;
	tDeviceDescriptor.FrameCounter[2] = 0;
	tDeviceDescriptor.FrameCounter[3] = 0;
	tDeviceDescriptor.Exempt = 0;

	MLME_SET_request_sync(
	    macDeviceTable,
	    sMode2DeviceIndex,
	    sizeof(tDeviceDescriptor),
	    &tDeviceDescriptor,
	    pDeviceRef
	);
}

static void putDeviceDescriptor(otInstance *aInstance, uint16_t shortAddr,
		                        uint8_t *extAddr, uint8_t count)
{
	struct M_DeviceDescriptor tDeviceDescriptor;

	PUTLE16(otLinkGetPanId(aInstance), tDeviceDescriptor.PANId);
	PUTLE16(shortAddr, tDeviceDescriptor.ShortAddress);

	for (int j = 0; j < 8; j++)
	{
		tDeviceDescriptor.ExtAddress[j] = extAddr[7 - j];    //Flip endian
	}

	otExtAddress tExtAddr;
	memcpy(&tExtAddr, tDeviceDescriptor.ExtAddress, 8);
	struct DeviceCache *curDeviceCache = deviceCache_getCachedDevice(tExtAddr);
	memcpy(tDeviceDescriptor.FrameCounter, curDeviceCache->mFrameCounter, 4);
	tDeviceDescriptor.Exempt = 0;

	MLME_SET_request_sync(
	    macDeviceTable,
	    count,
	    sizeof(tDeviceDescriptor),
	    &tDeviceDescriptor,
	    pDeviceRef
	);
}

static void putFinalKey(otInstance *aInstance)
{
	struct M_KeyDescriptor_thread
	{
		struct M_KeyTableEntryFixed    Fixed;
		struct M_KeyIdLookupDesc       KeyIdLookupList[1];
		uint8_t                        flags[2];
		//struct M_KeyDeviceDesc         KeyDeviceList[count];
		//struct M_KeyUsageDesc          KeyUsageList[2];
	} tKeyDescriptor;

	//Joiner router - replace mode2 key for a few milliseconds to send
	if (sKekInUse && !otPlatRadioIsJoining(aInstance))
	{
		otPlatRadioGetKek(aInstance, tKeyDescriptor.Fixed.Key);
		tKeyDescriptor.Fixed.KeyIdLookupListEntries = 1;
		tKeyDescriptor.Fixed.KeyUsageListEntries = 1;

		tKeyDescriptor.Fixed.KeyDeviceListEntries = 0;

		tKeyDescriptor.KeyIdLookupList[0].LookupDataSizeCode = 1; //1 indicates 9 octets length
		tKeyDescriptor.KeyIdLookupList[0].LookupData[0] = 0;

		for (int i = 0; i < 8; i++)
		{
			tKeyDescriptor.KeyIdLookupList[0].LookupData[i + 1] = sKekCounterpart[i];
		}

		tKeyDescriptor.flags[0] = (MAC_FC_FT_DATA & KUD_FrameTypeMask);	//data usage
		otPlatLog(OT_LOG_LEVEL_INFO, OT_LOG_REGION_HARDMAC, "JoinerRouter KEK added to table");
	}
	else
	{
		memcpy(tKeyDescriptor.Fixed.Key, sMode2Key, 16);
		memset(tKeyDescriptor.KeyIdLookupList[0].LookupData, 0xFF, 5);
		tKeyDescriptor.Fixed.KeyIdLookupListEntries = 1;
		tKeyDescriptor.Fixed.KeyUsageListEntries = 1;
		tKeyDescriptor.Fixed.KeyDeviceListEntries = 1;

		tKeyDescriptor.KeyIdLookupList[0].LookupDataSizeCode = 0; //0 indicates 5 octets length
		tKeyDescriptor.flags[0] = sMode2DeviceIndex;	//Device number for the mode2 device
		tKeyDescriptor.flags[1] = (MAC_FC_FT_DATA & KUD_FrameTypeMask);	//data usage
		otPlatLog(OT_LOG_LEVEL_INFO, OT_LOG_REGION_HARDMAC, "Mode2 Key added to table");
	}

	uint8_t unusedBytes = 1 - tKeyDescriptor.Fixed.KeyDeviceListEntries;

	MLME_SET_request_sync(
	    macKeyTable,
	    3,
	    sizeof(tKeyDescriptor) - unusedBytes,	/*dont send the unused bytes */
	    &tKeyDescriptor,
	    pDeviceRef
	);
}

void otPlatRadioSetKekCounterpart(otInstance *aInstance, uint8_t *otherAddress)
{
	memcpy(sKekCounterpart, otherAddress, 8);
	sKekInUse = true;
	otPlatLog(OT_LOG_LEVEL_INFO, OT_LOG_REGION_HARDMAC, "KEK counterpart set");
}

static void putJoinerKek(otInstance *aInstance)
{
	struct M_KeyDescriptor_thread
	{
		struct M_KeyTableEntryFixed    Fixed;
		struct M_KeyIdLookupDesc       KeyIdLookupList[1];
		uint8_t                        flags[2];
		//struct M_KeyDeviceDesc         KeyDeviceList[count];
		//struct M_KeyUsageDesc          KeyUsageList[2];
	} tKeyDescriptor;

	otPlatRadioGetKek(aInstance, tKeyDescriptor.Fixed.Key);
	tKeyDescriptor.Fixed.KeyIdLookupListEntries = 1;
	tKeyDescriptor.Fixed.KeyUsageListEntries = 1;

	tKeyDescriptor.Fixed.KeyDeviceListEntries = 1;

	tKeyDescriptor.KeyIdLookupList[0].LookupDataSizeCode = 1; //1 indicates 9 octets length
	tKeyDescriptor.KeyIdLookupList[0].LookupData[0] = 0;

	for (int i = 0; i < 8; i++)
	{
		tKeyDescriptor.KeyIdLookupList[0].LookupData[i + 1] = sKekCounterpart[7 - i];    //Flip endian
	}

	tKeyDescriptor.flags[0] = (sKekDeviceIndex);	//device
	tKeyDescriptor.flags[1] = (MAC_FC_FT_DATA & KUD_FrameTypeMask);	//data usage
	MLME_SET_request_sync(
	    macKeyTable,
	    0,
	    sizeof(tKeyDescriptor),
	    &tKeyDescriptor,
	    pDeviceRef
	);

	otPlatLog(OT_LOG_LEVEL_INFO, OT_LOG_REGION_HARDMAC, "Joiner KEK added to table");
}

void otPlatRadioDisableKek(otInstance *aInstance)
{
	memset(sKekCounterpart, 0, 8);
	sKekInUse = false;
	sKekMessageHandle = 0;
	otPlatLog(OT_LOG_LEVEL_INFO, OT_LOG_REGION_HARDMAC, "KEK disabled");
}

static void keyChangeCallback(uint32_t aFlags, otInstance *aInstance)
{

	/*
	 * This Function updates the keytable and devicetable entries stored on the
	 * ca8210 whenever there is a new key or new device to communicate with.
	 */

	if (!(aFlags & (
	          OT_NET_KEY_SEQUENCE_COUNTER |
	          OT_THREAD_CHILD_ADDED |
	          OT_THREAD_CHILD_REMOVED |
	          OT_NET_ROLE |
	          OT_THREAD_LINK_STATUS)))
	{
		//No relevant flag set
		return;
	}

	//Therefore update the keys stored in the macKeytable
	//TODO: (low priority) Utilise the device cache to reduce number of writes
	//Cache devices so the frame counters are correct
	deviceCache_cacheDevices();
	otPlatLog(OT_LOG_LEVEL_INFO, OT_LOG_REGION_HARDMAC, "Updating keys for flags: %x", aFlags);
	uint32_t tKeySeq = otThreadGetKeySequenceCounter(aInstance) - 1;

	uint8_t count = 0;	//Update device list

	if (otThreadGetDeviceRole(aInstance) != OT_DEVICE_ROLE_CHILD &&
		otThreadGetDeviceRole(aInstance) != OT_DEVICE_ROLE_DETACHED)
	{
		for (uint8_t i = 0; i < OPENTHREAD_CONFIG_MAX_CHILDREN; i++)
		{
			otChildInfo tChildInfo;

			if (otThreadGetChildInfoByIndex(aInstance, i, &tChildInfo) != OT_ERROR_NONE)
			{
				continue;
			}

			putDeviceDescriptor(aInstance, tChildInfo.mRloc16, tChildInfo.mExtAddress.m8, count++);

			if(count >= MAX_DYNAMIC_DEVICES){
				break;
			}
		}

		uint8_t maxRouters = MAX_DYNAMIC_DEVICES - count;
		otRouterInfo routers[maxRouters];
		uint8_t numRouters;
		otThreadGetNeighborRouterInfo(aInstance, routers, &numRouters, maxRouters);

		for (int i = 0; i < numRouters; i++)
		{
			putDeviceDescriptor(aInstance, routers[i].mRloc16, routers[i].mExtAddress.m8, count++);
		}

	}
	else
	{
		otRouterInfo tParentInfo;

		if (otThreadGetParentInfo(aInstance, &tParentInfo) == OT_ERROR_NONE)
		{
			putDeviceDescriptor(aInstance, tParentInfo.mRloc16, tParentInfo.mExtAddress.m8, count++);
		}
		else
		{
			otPlatLog(OT_LOG_LEVEL_WARN, OT_LOG_REGION_HARDMAC, "Error retrieving parent!\n\r");
		}
	}

	uint8_t activeDevices = count;

	if (sKekInUse && otPlatRadioIsJoining(aInstance))
	{
		sKekDeviceIndex = count;
		putDeviceDescriptor(aInstance, 0xFFFF, sKekCounterpart, count++);
	}

	sMode2DeviceIndex = count++;

	resetMode2Device();

	MLME_SET_request_sync(
	    macDeviceTableEntries,
	    0,
	    1,
	    &count,
	    pDeviceRef
	);

	struct M_KeyDescriptor_thread
	{
		struct M_KeyTableEntryFixed    Fixed;
		struct M_KeyIdLookupDesc       KeyIdLookupList[1];
		uint8_t                        flags[MAX_DYNAMIC_DEVICES + 2];
		//struct M_KeyDeviceDesc         KeyDeviceList[count];
		//struct M_KeyUsageDesc          KeyUsageList[2];
	} tKeyDescriptor;


	//Table 7-5 in section 7.2.2.2.1 of thread spec
	tKeyDescriptor.Fixed.KeyIdLookupListEntries = 1;
	tKeyDescriptor.Fixed.KeyUsageListEntries = 2;
	tKeyDescriptor.Fixed.KeyDeviceListEntries = activeDevices;
	sCurDeviceTableSize = activeDevices;

	//Flags according to Cascoda API 5.3.1
	tKeyDescriptor.flags[activeDevices + 0] = (MAC_FC_FT_DATA & KUD_FrameTypeMask);	//data usage
	tKeyDescriptor.flags[activeDevices + 1] = (MAC_FC_FT_COMMAND & KUD_FrameTypeMask) |
			((CMD_DATA_REQ << KUD_CommandFrameIdentifierShift) & KUD_CommandFrameIdentifierMask);	//Data req usage


	tKeyDescriptor.KeyIdLookupList[0].LookupDataSizeCode = 1; //1 means length 9

	//This sets the MSB of the lookUpData to equal defaultKeySource as is
	//required by 7.5.8.2.2 of IEEE 15.4 spec
	for (int i = 0; i < 9; i++)
	{
		tKeyDescriptor.KeyIdLookupList[0].LookupData[i] = 0;
	}

	//Set lookup data to the macDefaultKeySource to be right concatenated to the
	//individual keyIndex param
	tKeyDescriptor.KeyIdLookupList[0].LookupData[8] = 0xFF;


	//Fill the deviceListEntries
	for (int i = 0; i < activeDevices; i++)
	{
		tKeyDescriptor.flags[i] = i;
	}

	//Generate and store the keys for the current, previous, and next rotations
	uint8_t storeCount = 0;

	for (uint8_t i = 0; i < 3; i++)
	{
		if (i == 0 && sKekInUse && otPlatRadioIsJoining(aInstance))
		{
			//If joining, replace the first key (useless anyway) with the KEK
			continue;
		}

		memcpy(tKeyDescriptor.Fixed.Key, otPlatRadioGetMacKeyFromSequenceCounter(aInstance, tKeySeq + i), 16);
		tKeyDescriptor.KeyIdLookupList[0].LookupData[0] = ((tKeySeq + i) & 0x7F) + 1;

		MLME_SET_request_sync(
		    macKeyTable,
		    storeCount++,
		    sizeof(tKeyDescriptor) - (MAX_DYNAMIC_DEVICES - activeDevices),	/*dont send the unused bytes (those not counted by count)*/
		    &tKeyDescriptor,
		    pDeviceRef
		);
	}

	//Kek
	if (sKekInUse && otPlatRadioIsJoining(aInstance))
	{
		putJoinerKek(aInstance);
	}

	//Mode2/KEK
	storeCount++;
	putFinalKey(aInstance);

	MLME_SET_request_sync(
	    macKeyTableEntries,
	    0,
	    1,
	    &storeCount,
	    pDeviceRef
	);
}

otError otPlatRadioEnable(otInstance *aInstance)    //TODO:(lowpriority) port
{
	otError error = OT_ERROR_BUSY;

	if (OT_INSTANCE == NULL)
	{
		OT_INSTANCE = aInstance;
	}
	else
	{
		return error;
	}

	if (sState == OT_RADIO_STATE_SLEEP || sState == OT_RADIO_STATE_DISABLED)
	{
		error = OT_ERROR_NONE;
		sState = OT_RADIO_STATE_SLEEP;
	}

	return error;
}

otError otPlatRadioDisable(otInstance *aInstance)    //TODO:(lowpriority) port
{
	otError error = OT_ERROR_BUSY;

	if (sState == OT_RADIO_STATE_DISABLED || sState == OT_RADIO_STATE_SLEEP)
	{
		error = OT_ERROR_NONE;
		sState = OT_RADIO_STATE_DISABLED;

		//should sleep until restarted
#ifdef USE_LOWPOWER_MODES
		uint8_t HWMEAttVal[5] = {0x0A, 00, 00, 00, 00};

		if (HWME_SET_request_sync(
		        HWME_POWERCON,
		        5,
		        HWMEAttVal,
		        pDeviceRef
		    ) == HWME_SUCCESS)
		{
			return OT_ERROR_NONE;
		}
		else
		{
			return OT_ERROR_FAILED;
		}

#endif
	}

	return error;
}

otError otPlatRadioSleep(otInstance *aInstance)
{
	return OT_ERROR_NONE;
	//This is handled by the hardmac and the state of rxOnWhenIdle
}

otError otPlatRadioSetRxOnWhenIdle(otInstance *aInstance, uint8_t rxOnWhenIdle)
{
	otError error = OT_ERROR_NONE;
	uint8_t ret = 0;
	rxOnWhenIdle = rxOnWhenIdle ? 1 : 0;

	ret = MLME_SET_request_sync( //enable Rx when Idle
	          macRxOnWhenIdle,
	          0,
	          sizeof(rxOnWhenIdle),
	          &rxOnWhenIdle,
	          pDeviceRef);

	error = ret ? OT_ERROR_BUSY : OT_ERROR_NONE;

	return error;
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
	otError error = OT_ERROR_NONE;

	VerifyOrExit(sState != OT_RADIO_STATE_DISABLED, error = OT_ERROR_BUSY);
	sState = OT_RADIO_STATE_RECEIVE;

	setChannel(aChannel);

exit:
	return error;
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
	return &sTransmitFrame;
}

otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aPacket, void *transmitContext)
{
	/*
	 * This Function converts the openthread-provided PHY frame into a MAC
	 * primitive to communicate with the hardmac-enabled cascoda ca8210. Changes
	 * in the soft MAC layer prevent the frame from being encrypted until it
	 * reaches the hardmac.
	 */
	otError error = OT_ERROR_NONE;
	static uint8_t handle = 0;
	handle++;

	otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_HARDMAC, "otPlatRadioTransmit Called");

	while (intransit_isHandleInUse(handle))
	{
		handle++;    //make sure handle is available for use
	}

	VerifyOrExit(sState != OT_RADIO_STATE_DISABLED, error = OT_ERROR_BUSY);

	uint16_t frameControl = GETLE16(aPacket->mPsdu);
	VerifyOrExit(((frameControl & MAC_FC_FT_MASK) == MAC_FC_FT_DATA) || \
	             ((frameControl & MAC_FC_FT_MASK) == MAC_FC_FT_COMMAND),\
	             error = OT_ERROR_ABORT;                            \
	             otPlatLog(OT_LOG_LEVEL_WARN, OT_LOG_REGION_HARDMAC, "Unexpected frame type %#x\n\r", (frameControl & MAC_FC_FT_MASK)););

	sState = OT_RADIO_STATE_TRANSMIT;
	sTransmitError = OT_ERROR_NONE;

	setChannel(aPacket->mChannel);

	//transmit
	struct MCPS_DATA_request_pset curPacket;
	struct SecSpec curSecSpec = {0};

	uint8_t headerLength = 0;
	uint8_t footerLength = 0;

	curPacket.SrcAddrMode = MAC_FC_SAM(frameControl);
	curPacket.Dst.AddressMode = MAC_FC_DAM(frameControl);
	curPacket.TxOptions = (frameControl & MAC_FC_ACK_REQ) ? 0x01 : 0x00;	//ack-requested
	curPacket.TxOptions |= aPacket->mDirectTransmission ? 0x00 : 1 << 2;	//indirect
	uint8_t isPanCompressed = frameControl & MAC_FC_PAN_COMP;

	uint8_t addressFieldLength = 0;

	if (curPacket.Dst.AddressMode == MAC_MODE_SHORT_ADDR)
	{
		memcpy(curPacket.Dst.Address, aPacket->mPsdu + 5, 2);
		memcpy(curPacket.Dst.PANId, aPacket->mPsdu + 3, 2);
		addressFieldLength += 4;
	}
	else if (curPacket.Dst.AddressMode == MAC_MODE_LONG_ADDR)
	{
		memcpy(curPacket.Dst.Address, aPacket->mPsdu + 5, 8);
		memcpy(curPacket.Dst.PANId, aPacket->mPsdu + 3, 2);
		addressFieldLength += 10;
	}

	if (curPacket.SrcAddrMode == MAC_MODE_SHORT_ADDR)
	{
		addressFieldLength += 4;
	}
	else if (curPacket.SrcAddrMode == MAC_MODE_LONG_ADDR)
	{
		addressFieldLength += 10;
	}

	if (curPacket.SrcAddrMode && isPanCompressed)
	{
		//Remove size saved by not including the same PAN twice
		addressFieldLength -= 2;
	}

	headerLength = addressFieldLength + MAC_BASEHEADERLENGTH;

	if ((frameControl & MAC_FC_FT_MASK) == MAC_FC_FT_DATA)
	{
		aPacket->mTransmitContext = transmitContext;
		aPacket->mTransmitInstance = aInstance;
		intransit_putFrame(handle, aPacket);
	}

	if (frameControl & MAC_FC_SEC_ENA) 	//if security is required
	{
		uint8_t ASHloc = MAC_BASEHEADERLENGTH + addressFieldLength;
		uint8_t securityControl = *(uint8_t *)(aPacket->mPsdu + ASHloc);
		curSecSpec.SecurityLevel = MAC_SC_SECURITYLEVEL(securityControl);
		curSecSpec.KeyIdMode = MAC_SC_KEYIDMODE(securityControl);

		ASHloc += 5;//skip to key identifier

		if (curSecSpec.KeyIdMode == 0x02) //Table 96
		{
			memcpy(curSecSpec.KeySource, aPacket->mPsdu + ASHloc, 4);
			ASHloc += 4;
		}
		else if (curSecSpec.KeyIdMode == 0x03) //Table 96
		{
			memcpy(curSecSpec.KeySource, aPacket->mPsdu + ASHloc, 8);
			ASHloc += 8;
		}
		else if (curSecSpec.KeyIdMode == 0x00) //Table 96
		{
			//This should be using the KeK
			sKekMessageHandle = handle;
			otPlatRadioSetKekCounterpart(aInstance, curPacket.Dst.Address);
			putFinalKey(aInstance);
		}

		if (curSecSpec.KeyIdMode != 0x00)
		{
			curSecSpec.KeyIndex = aPacket->mPsdu[ASHloc++];
		}

		headerLength = ASHloc;
	}

	if ((frameControl & MAC_FC_FT_MASK) == MAC_FC_FT_DATA)
	{
		//Table 95 to calculate auth tag length
		footerLength = 2 << (curSecSpec.SecurityLevel % 4);
		footerLength = (footerLength == 2) ? 0 : footerLength;

		footerLength += 2; //MFR length

		curPacket.MsduLength = aPacket->mLength - footerLength - headerLength;
		memcpy(curPacket.Msdu, aPacket->mPsdu + headerLength, curPacket.MsduLength);
		curPacket.MsduHandle = handle;

		if (curSecSpec.KeyIdMode == 0x02)
		{
			//Don't use keyIdMode 2 for sending. It is optional for thread and requires changing ext address
			memset(&curSecSpec, 0, sizeof(curSecSpec));
		}

		otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_HARDMAC, "Data Packet Sent");
		MCPS_DATA_request(
		    curPacket.SrcAddrMode,
		    curPacket.Dst.AddressMode,
		    GETLE16(curPacket.Dst.PANId),
		    (union MacAddr *) curPacket.Dst.Address,
		    curPacket.MsduLength,
		    curPacket.Msdu,
		    curPacket.MsduHandle,
		    curPacket.TxOptions,
		    &curSecSpec,
		    pDeviceRef);
	}
	else if ((frameControl & MAC_FC_FT_MASK) == MAC_FC_FT_COMMAND)
	{
		if (aPacket->mPsdu[headerLength] == 0x04) 	//Data request command
		{

			uint8_t interval[2] = {0, 0};
			uint8_t ret;
			uint8_t count = 0;

			do
			{
				ret = MLME_POLL_request_sync(curPacket.Dst,
				                             interval,
				                             &curSecSpec,
				                             pDeviceRef);

				if (count > 0)
				{
					otPlatLog(OT_LOG_LEVEL_WARN, OT_LOG_REGION_HARDMAC, "Poll Failed! Retry #%d\n\r", count);
				}
			}
			while (ret == 0xFF && (count++ < 10));

			if (ret == MAC_SUCCESS)
			{
				otPlatRadioTransmitDone(aInstance, aPacket, true, transmitContext, error);
			}
			else if (ret == MAC_NO_DATA)
			{
				otPlatRadioTransmitDone(aInstance, aPacket, false, transmitContext, error);
			}
			else
			{
				otPlatRadioTransmitDone(aInstance, aPacket, false, transmitContext, OT_ERROR_NO_ACK);
			}
		}
		else
		{
			assert(false);
		}
	}

exit:
	return error;
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
	return noiseFloor;
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
	return OT_RADIO_CAPS_ACK_TIMEOUT;
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
	if (sPromiscuousCache != tristate_uninit)
	{
		return sPromiscuousCache;
	}

	uint8_t resultLen;
	uint8_t result;
	MLME_GET_request_sync(
	    macPromiscuousMode,
	    0,
	    &resultLen,
	    &result,
	    pDeviceRef);

	result = result ? tristate_enabled : tristate_disabled;
	sPromiscuousCache = result;

	return (bool) result;
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
	uint8_t enable = aEnable ? 1 : 0;	//Just to be sure we match spec
	MLME_SET_request_sync(
	    macPromiscuousMode,
	    0,
	    sizeof(enable),
	    &enable,
	    pDeviceRef);
	sPromiscuousCache = aEnable;
}

void otPlatRadioPurge(otInstance *aInstance, void * aSender){
	intransit_purge(aSender);
}

static int handleDataIndication(struct MCPS_DATA_indication_pset *params)   //Async
{
	static otRadioFrame sReceiveFrame;
	static uint8_t sReceivePsdu[IEEE802154_MAX_LENGTH];
	static otError sReceiveError = OT_ERROR_NONE;

	/*
	 * This Function converts the MAC primitive provided by the ca8210 into a
	 * PHY frame that can be processed by openthread. Changes in the soft MAC
	 * layer mean that the frame is not double decrypted, and the link security
	 * is accepted if the hardmac approves it.
	 */

	if (!otIp6IsEnabled(OT_INSTANCE))
	{
		return 1;
	}

	sReceiveFrame.mLength = 0;
	sReceiveFrame.mPsdu = sReceivePsdu;

	uint8_t headerLength = 0;
	uint8_t footerLength = 0;
	uint16_t frameControl = 0;
	uint8_t msduLength = params->MsduLength;
	struct SecSpec *curSecSpec = (struct SecSpec *)
			((unsigned int)params + (unsigned int)msduLength + 29); //Location defined in cascoda API docs

	frameControl |= (params->Src.AddressMode & 0x3) << 14;
	frameControl |= (params->Dst.AddressMode & 0x3) << 10;
	frameControl |= curSecSpec->SecurityLevel ? MAC_FC_SEC_ENA : 0;	//Security Enabled field
	frameControl |= MAC_FC_FT_DATA; //Frame type = data

	uint8_t addressFieldLength = 0;

	if (params->Dst.AddressMode == MAC_MODE_SHORT_ADDR)
	{
		memcpy(sReceiveFrame.mPsdu + 5, params->Dst.Address, 2);
		memcpy(sReceiveFrame.mPsdu + 3, params->Dst.PANId, 2);
		addressFieldLength += 4;
	}
	else if (params->Dst.AddressMode == MAC_MODE_LONG_ADDR)
	{
		memcpy(sReceiveFrame.mPsdu + 5, params->Dst.Address, 8);
		memcpy(sReceiveFrame.mPsdu + 3, params->Dst.PANId, 2);
		addressFieldLength += 10;
	}

	if (memcmp(params->Src.PANId, params->Dst.PANId, 2)) //Different PANs
	{
		if (params->Src.AddressMode == MAC_MODE_SHORT_ADDR)
		{
			memcpy(sReceiveFrame.mPsdu + addressFieldLength + 5, params->Src.Address, 2);
			memcpy(sReceiveFrame.mPsdu + addressFieldLength + 3, params->Src.PANId, 2);
			addressFieldLength += 4;
		}
		else if (params->Src.AddressMode == MAC_MODE_LONG_ADDR)
		{
			memcpy(sReceiveFrame.mPsdu + addressFieldLength + 5, params->Src.Address, 8);
			memcpy(sReceiveFrame.mPsdu + addressFieldLength + 3, params->Src.PANId, 2);
			addressFieldLength += 10;
		}
	}
	else 	//use PAN compression
	{
		if (params->Src.AddressMode == MAC_MODE_SHORT_ADDR)
		{
			memcpy(sReceiveFrame.mPsdu + addressFieldLength + 3, params->Src.Address, 2);
			addressFieldLength += 2;
		}
		else if (params->Src.AddressMode == MAC_MODE_LONG_ADDR)
		{
			memcpy(sReceiveFrame.mPsdu + addressFieldLength + 3, params->Src.Address, 8);
			addressFieldLength += 8;
		}

		frameControl |= MAC_FC_PAN_COMP;
	}

	//Commit the frame control bytes to the frame
	PUTLE16(frameControl, sReceiveFrame.mPsdu);

	headerLength = addressFieldLength + MAC_BASEHEADERLENGTH;

	if (frameControl & MAC_FC_SEC_ENA)
	{
		//security is required
		uint8_t ASHloc = MAC_BASEHEADERLENGTH + addressFieldLength;

		uint8_t securityControl = 0;
		securityControl |= MAC_SC_SECURITYLEVEL(curSecSpec->SecurityLevel);
		securityControl |= MAC_KEYIDMODE_SC(curSecSpec->KeyIdMode);

		sReceiveFrame.mPsdu[ASHloc] = securityControl;

		ASHloc += 5;//skip to key identifier

		if (curSecSpec->KeyIdMode == 0x02) //Table 96
		{
			resetMode2Device();
			memcpy(sReceiveFrame.mPsdu + ASHloc, curSecSpec->KeySource, 4);
			ASHloc += 4;
		}
		else if (curSecSpec->KeyIdMode == 0x03) //Table 96
		{
			memcpy(sReceiveFrame.mPsdu + ASHloc, curSecSpec->KeySource, 8);
			ASHloc += 8;
		}
		else if (curSecSpec->KeyIdMode == 0x00) //Table 96
		{
			otPlatLog(OT_LOG_LEVEL_INFO, OT_LOG_REGION_HARDMAC, "Key Mode 0 received");
		}

		if (curSecSpec->KeyIdMode != 0x00)
		{
			sReceiveFrame.mPsdu[ASHloc++] = curSecSpec->KeyIndex;
		}

		headerLength = ASHloc;
	}

	//Table 95 to calculate auth tag length
	footerLength = 2 << (curSecSpec->SecurityLevel % 4);
	footerLength = footerLength == 2 ? 0 : footerLength;

	footerLength += 2; //MFR length

	sReceiveFrame.mLength = params->MsduLength + footerLength + headerLength;

	if (sReceiveFrame.mLength > aMaxPHYPacketSize)
	{
		otPlatLog(OT_LOG_LEVEL_WARN, OT_LOG_REGION_HARDMAC, "Invalid frame Length %d! Msdu: %d; Footer: %d; Header: %d;\n\r", sReceiveFrame.mLength, params->MsduLength, footerLength, headerLength);
		return 1;
	}

	memcpy(sReceiveFrame.mPsdu + headerLength, params->Msdu, params->MsduLength);
	sReceiveFrame.mLqi = params->MpduLinkQuality;
	sReceiveFrame.mChannel = sChannel;
	sReceiveFrame.mPower = (params->MpduLinkQuality - 256) / 2;	//Formula from CA-821X API
	noiseFloor = sReceiveFrame.mPower;

	barrier_worker_waitForMain();
	sState = OT_RADIO_STATE_RECEIVE;
	otPlatRadioReceiveDone(OT_INSTANCE, &sReceiveFrame, sReceiveError);
	barrier_worker_endWork();

	sReceiveFrame.mLength = 0;

	return 0;
}

static int handleDataConfirm(struct MCPS_DATA_confirm_pset *params)   //Async
{

	/*
	 * This Function processes the MCPS_DATA_CONFIRM and passes the success or
	 * error to openthread as appropriate.
	 */
	barrier_worker_waitForMain();

	otRadioFrame *sentFrame = intransit_getFrame(params->MsduHandle);
	otInstance *aInstance = sentFrame->mTransmitInstance;

	if (aInstance == NULL || sentFrame == NULL || !otIp6IsEnabled(aInstance))
	{
		barrier_worker_endWork();
		return 1;
	}

	otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_HARDMAC, "Data confirm received!");

	assert(sentFrame != NULL);

	if (sKekInUse && params->MsduHandle == sKekMessageHandle)
	{
		sKekMessageHandle = 0;
		otPlatRadioDisableKek(aInstance);
		putFinalKey(aInstance);
	}

	switch (params->Status)
	{
	case MAC_SUCCESS:
		break;

	case MAC_CHANNEL_ACCESS_FAILURE:
	case MAC_TRANSACTION_OVERFLOW:
		sTransmitError = OT_ERROR_CHANNEL_ACCESS_FAILURE;
		break;

	default:
		sTransmitError = OT_ERROR_NO_ACK;
		break;
	}
	otPlatRadioTransmitDone(aInstance, sentFrame, false, sentFrame->mTransmitContext, sTransmitError);

	if(sTransmitError != OT_ERROR_NONE)
	{
		otPlatLog(OT_LOG_LEVEL_WARN, OT_LOG_REGION_HARDMAC, "MCPS_DATA_confirm error: %#x \r\n", params->Status);
	}

	sState = OT_RADIO_STATE_RECEIVE;
	sTransmitError = OT_ERROR_NONE;

	intransit_rmFrame(params->MsduHandle);

	barrier_worker_endWork();
	return 0;
}

static int handleBeaconNotify(struct MLME_BEACON_NOTIFY_indication_pset *params) //Async
{

	/*
	 * This Function processes an incoming beacon from an activeScan, processing
	 *  the payload and passing the relevant information to openthread in a struct.
	 */

	otActiveScanResult resultStruct;
	uint8_t *sduLength;
	uint8_t *Sdu;
	uint8_t version;

	uint8_t shortaddrs  = *((uint8_t *)params + 23) & 7;
	uint8_t extaddrs = (*((uint8_t *)params + 23) & 112) >> 4;

	if ((params->PanDescriptor.Coord.AddressMode) == 3)
	{
		memcpy(resultStruct.mExtAddress.m8, params->PanDescriptor.Coord.Address, 8);
	}
	else
	{
		otPlatLog(OT_LOG_LEVEL_WARN, OT_LOG_REGION_HARDMAC, "Invalid beacon received!");
		return 1;
	}

	resultStruct.mPanId = GETLE16(params->PanDescriptor.Coord.PANId);
	resultStruct.mChannel = params->PanDescriptor.LogicalChannel;
	resultStruct.mRssi = (params->PanDescriptor.LinkQuality - 256) / 2;
	noiseFloor = resultStruct.mRssi;
	resultStruct.mLqi = params->PanDescriptor.LinkQuality;
	VerifyOrExit(params->PanDescriptor.Security.SecurityLevel == 0,;);
	//Asset security = 0
	sduLength = (uint8_t *)params + (24 + (2 * shortaddrs) + (8 * extaddrs));

	VerifyOrExit(*sduLength >= 26, );

	Sdu = (uint8_t *)params + (25 + 2 * shortaddrs + 8 * extaddrs);
	version = (*((uint8_t *)Sdu + 1) & 0x0F);

	if (version != (mBeaconPayload[1] & 0x0F))
	{
		otPlatLog(OT_LOG_LEVEL_WARN, OT_LOG_REGION_HARDMAC, "Beacon received is from different Thread version");
		return 0;
	}

	if (*Sdu == 3)
	{
		memcpy(&resultStruct.mNetworkName, ((char *)Sdu) + 2, sizeof(resultStruct.mNetworkName));
		memcpy(&resultStruct.mExtendedPanId, Sdu + 18, sizeof(resultStruct.mExtendedPanId));
		barrier_worker_waitForMain();
		sActiveScanCallback(&resultStruct, sActiveScanContext);
		barrier_worker_endWork();
	}


exit:
	return 0;
}

static int handleScanConfirm(struct MLME_SCAN_confirm_pset *params)   //Async
{

	VerifyOrExit(params->Status != MAC_SCAN_IN_PROGRESS, );

	if (sActiveScanInProgress)
	{
		barrier_worker_waitForMain();
		sActiveScanCallback(NULL, sActiveScanContext);
		sActiveScanInProgress = 0;
		barrier_worker_endWork();
		MLME_SET_request_sync(
			phyCurrentChannel,
			0,
			sizeof(sChannel),
			&sChannel,
			pDeviceRef);
	}
	else if (sEnergyScanInProgress)
	{
		barrier_worker_waitForMain();
		uint8_t curMinChannel = 11;

		//Call the resultCallback for each result
		for (int i = 0; i < params->ResultListSize; i++)
		{
			otEnergyScanResult result;
			result.mMaxRssi = params->ResultList[i];
			result.mChannel = curMinChannel;
			sEnergyScanCallback(&result, sActiveScanContext);

			while (!(sEnergyScanMask & 1 << curMinChannel))
			{
				result.mChannel = curMinChannel++;

				// clear the bit for the channel just handled
				sEnergyScanMask &= sEnergyScanMask - 1;
			}
		}

		//Send completion callback
		sEnergyScanCallback(NULL, sActiveScanContext);
		sEnergyScanInProgress = 0;
		barrier_worker_endWork();
		MLME_SET_request_sync(
			phyCurrentChannel,
			0,
			sizeof(sChannel),
			&sChannel,
			pDeviceRef);
	}


exit:
	return 0;
}

static int handleGenericDispatchFrame(const uint8_t *buf, size_t len)   //Async
{

	/*
	 * This is a debugging function for unhandled incoming MAC data
	 */

	return 0;
}

int PlatformRadioProcess(void)
{
	barrier_main_letWorkerWork();
	return 0;
}

static struct DeviceCache *deviceCache_getCachedDevice(otExtAddress addr)
{
	struct DeviceCache *toReturn = NULL;
	uint8_t foundEmpty = 0xFF;

	//Find existing cache
	for (int i = 0; i < DEVICE_TABLE_SIZE; i++)
	{
		if (sDeviceCache[i].isActive)
		{
			if (memcmp(addr.m8, sDeviceCache[i].mExtAddr.m8, sizeof(addr)) == 0)
			{
				return &sDeviceCache[i];
			}
		}
		else if (foundEmpty == 0xFF)
		{
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

static void deviceCache_cacheDevices()
{
	for (uint8_t i = 0; i < sCurDeviceTableSize; i++)
	{
		uint8_t length;
		struct M_DeviceDescriptor tDeviceDescriptor;

		MLME_GET_request_sync(macDeviceTable,
		                      i,
		                      &length,
		                      &tDeviceDescriptor,
		                      pDeviceRef);

		otExtAddress cur;
		struct DeviceCache *curCache;
		memcpy(cur.m8, tDeviceDescriptor.ExtAddress, sizeof(cur));
		curCache = deviceCache_getCachedDevice(cur);
		memcpy(curCache->mFrameCounter, tDeviceDescriptor.FrameCounter, 4);
		curCache->isActive = 2;
	}

	for (uint8_t i = 0; i < DEVICE_TABLE_SIZE; i++)
	{
		if (sDeviceCache[i].isActive == 1)
		{
			//Device was not in device table, remove from cache
			memset(&sDeviceCache[i], 0, sizeof(sDeviceCache[i]));
		}
		else if (sDeviceCache[i].isActive == 2)
		{
			sDeviceCache[i].isActive = 1;
		}
	}
}

uint8_t otPlatRadioIsDeviceActive(otInstance *aInstance, otExtAddress addr)
{
	//update cache
	//TODO: Don't recache all devices every time -> just update the frame counter for the relevant one and recache all of the rest as the macDeviceTable is changed.
	deviceCache_cacheDevices();

	otExtAddress macAddr;

	for (int i = 0; i < sizeof(otExtAddress); i++)
	{
		macAddr.m8[i] = addr.m8[7 - i];
	}

	//Find existing cache
	for (int i = 0; i < DEVICE_TABLE_SIZE; i++)
	{
		if (sDeviceCache[i].isActive && memcmp(macAddr.m8, sDeviceCache[i].mExtAddr.m8, sizeof(macAddr)) == 0)
		{
			//Device match found
			if (memcmp(sDeviceCache[i].mFrameCounter, sDeviceCache[i].mTimeoutFrameCounter, 4) == 0)
			{
				//Device has not sent any messages since last poll -> device is inactive
				//(This will promptly be removed by the higher level and will flush through)
				return 0;
			}
			else
			{
				//Update the timeout version, return 1 -> the device is active
				memcpy(sDeviceCache[i].mTimeoutFrameCounter, sDeviceCache[i].mFrameCounter, 4);
				return 1;
			}
		}
	}

	return 0;
}

static int intransit_isHandleInUse(uint8_t handle)
{
	for (int i = 0; i < MAX_INTRANSITS; i++)
	{
		if (IntransitHandles[i] == handle)
		{
			return 1;	//Handle is in use
		}
	}

	return 0;
}

static int intransit_putFrame(uint8_t handle, const otRadioFrame *in)
{
	int i = 0;
	uint8_t found = 0;

	while (!found)
	{
		if (i >= MAX_INTRANSITS)
		{
			assert(false);
			return -1;
		}

		if (IntransitHandles[i] == 0)
		{
			break;    //This index is empty
		}

		i++;
	}

	IntransitHandles[i] = handle;
	//TODO: Only store what is needed
	memcpy(&IntransitPackets[i], in, sizeof(otRadioFrame));

	return 0;
}

static int intransit_purge(void * aSender){
	for (int i = 0; i < MAX_INTRANSITS; i++)
	{
		if (!memcmp(&IntransitPackets[i].mTransmitContext, &aSender, sizeof(aSender))){
			uint8_t handle = IntransitHandles[i];
			MCPS_PURGE_request_sync(&handle, pDeviceRef);
			intransit_rmFrame(IntransitHandles[i]);
			return 0;
		}
	}
	return 1;
}

static int intransit_rmFrame(uint8_t handle)
{
	uint8_t found = 0;

	for (int i = 0; i < MAX_INTRANSITS; i++)
	{
		if (IntransitHandles[i] == handle)
		{
			found = 1;
			IntransitHandles[i] = 0;
			break;
		}
	}

	return !found;	//0 if successfully removed
}

static otRadioFrame *intransit_getFrame(uint8_t handle)
{
	otRadioFrame *rval = NULL;

	for (int i = 0; i < MAX_INTRANSITS; i++)
	{
		if (IntransitHandles[i] == handle)
		{
			rval = &IntransitPackets[i];
			break;
		}
	}

	return rval;
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

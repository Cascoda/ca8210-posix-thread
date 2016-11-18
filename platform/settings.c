/*
 * settings.c
 *
 *  Created on: 18 Nov 2016
 *      Author: ciaran
 */

#include <platform/settings.h>

void otPlatSettingsInit(otInstance *aInstance){
	(void) aInstance;
}

ThreadError otPlatSettingsBeginChange(otInstance *aInstance){
	(void) aInstance;
	return kThreadError_None;
}

ThreadError otPlatSettingsCommitChange(otInstance *aInstance){
	(void) aInstance;
	return kThreadError_NotImplemented;
}

ThreadError otPlatSettingsAbandonChange(otInstance *aInstance){
	(void) aInstance;
	return kThreadError_NotImplemented;
}

ThreadError otPlatSettingsGet(otInstance *aInstance, uint16_t aKey, int aIndex, uint8_t *aValue,
                              uint16_t *aValueLength){
	(void) aInstance;
	return kThreadError_NotImplemented;
}

ThreadError otPlatSettingsSet(otInstance *aInstance, uint16_t aKey, const uint8_t *aValue, uint16_t aValueLength){
	(void) aInstance;
	return kThreadError_NotImplemented;
}

ThreadError otPlatSettingsAdd(otInstance *aInstance, uint16_t aKey, const uint8_t *aValue, uint16_t aValueLength){
	(void) aInstance;
	return kThreadError_NotImplemented;
}

ThreadError otPlatSettingsDelete(otInstance *aInstance, uint16_t aKey, int aIndex){
	(void) aInstance;
	return kThreadError_NotImplemented;
}

void otPlatSettingsWipe(otInstance *aInstance){
	(void) aInstance;
}

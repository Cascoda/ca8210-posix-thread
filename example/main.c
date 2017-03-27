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


#include <stdlib.h>
#include <stdio.h>

#include <openthread.h>
#include <cli.h>
#include <posix-platform.h>

#include <openthread-tasklet.h>

void otSignalTaskletPending(otInstance *aInstance)
{
	(void)aInstance;
}

int main(int argc, char *argv[])
{

    NODE_ID = 1;

    posixPlatformInit();
    OT_INSTANCE = otInstanceInit();
    otCliUartInit(OT_INSTANCE);

    /* Test harness specific config */
#ifdef TESTHARNESS
    otSetNetworkName(OT_INSTANCE, "GRL");
    otSetPanId(OT_INSTANCE, 0xface);
    uint8_t extPanId[] = {0x00, 0x0d, 0xb8, 0x00, 0x00, 0x00, 0x00, 0x00};
    otSetExtendedPanId(OT_INSTANCE, extPanId);
    otSetChannel(OT_INSTANCE, 20);
#else
    otSetPanId(OT_INSTANCE, 0x1234); //Convenience
#endif

    while (1)
    {
    	otProcessQueuedTasklets(OT_INSTANCE);
        posixPlatformProcessDrivers();
    }

    return 0;
}

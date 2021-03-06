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
 *   This file implements a pseudo-random number generator.
 *
 * @warning
 *   This implementation is not a true random number generator and does @em satisfy the Thread requirements.
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <assert.h>

#include "openthread/platform/random.h"
#include "ca821x-posix-thread/posix-platform.h"
#include "code_utils.h"
#include "ca821x_api.h"
#include "hwme_tdme.h"

void posixPlatformRandomInit(void)
{
    //nothing
}

uint32_t otPlatRandomGet(void)
{

	uint8_t length1 = 0;
	uint8_t length2 = 0;

	union randBytes{
		uint8_t randb[4];
		uint32_t rand32i;
	} randomBytes;

	/*
    HWME_GET_request_sync(
    		HWME_RANDOMNUM,
			&length1,
			randomBytes.randb,
			NULL);
    HWME_GET_request_sync(
			HWME_RANDOMNUM,
			&length2,
			randomBytes.randb + 2,
			NULL);
	*/

    if(length1 != 2 || length2 != 2){

    	uint32_t rnum = 0;
		int fd = open("/dev/urandom", O_RDONLY);
		if (fd != -1)
		{
			(void) read(fd, (void *)&rnum, sizeof(rnum));
			(void) close(fd);
		}
		else{
			assert(0); //All attempts at randomness have failed
		}

		return rnum;
    }

    return randomBytes.rand32i;

}

otError otPlatRandomGetTrue(uint8_t *aOutput, uint16_t aOutputLength){
	otError error = OT_ERROR_NONE;

	otEXPECT_ACTION(aOutput != NULL, error = OT_ERROR_INVALID_ARGS);

	int fd = open("/dev/random", O_RDONLY);
	if (fd != -1)
	{
		(void) read(fd, (void *)aOutput, aOutputLength);
		(void) close(fd);
	}
	else{
		error = OT_ERROR_FAILED;
	}

exit:
	return error;
}


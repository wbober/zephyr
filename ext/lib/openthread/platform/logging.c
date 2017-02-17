/*
 *  Copyright (c) 2016, The OpenThread Authors.
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

#include <stdarg.h>
#include <stdio.h>

#include "platform-zephyr.h"
#include <platform/logging.h>

#define SYS_LOG_DOMAIN "OPEN_THREAD"
#define SYS_LOG_LEVEL 4
#include <logging/sys_log.h>

#define LOG_PARSE_BUFFER_SIZE  128

void otPlatLog(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aFormat, ...)
{
    (void) aLogRegion;

    char logString[LOG_PARSE_BUFFER_SIZE + 1];
    uint16_t length = 0;

    // Parse user string.
    va_list paramList;
    va_start(paramList, aFormat);
    length += vsnprintf(&logString[length], (LOG_PARSE_BUFFER_SIZE - length),
                        aFormat, paramList);
    va_end(paramList);


	switch (aLogLevel)
	{
	case kLogLevelCrit:
		SYS_LOG_ERR("%s", logString);
		break;
	case kLogLevelWarn:
		SYS_LOG_WRN("%s", logString);
		break;
	case kLogLevelInfo:
		SYS_LOG_INF("%s", logString);
		break;
	case kLogLevelDebg:
		SYS_LOG_DBG("%s", logString);
		break;
	default:
		break;
	}
}


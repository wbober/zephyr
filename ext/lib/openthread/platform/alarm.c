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

#include <kernel.h>

#include <platform/alarm.h>
#include "platform-zephyr.h"

static bool isAlarmActive;
static uint32_t tAlarm;

void platformAlarmInit(void)
{
	isAlarmActive = false;
}

uint32_t otPlatAlarmGetNow(void)
{
	return k_uptime_get_32();
}

void otPlatAlarmStartAt(otInstance *aInstance, uint32_t t0, uint32_t dt)
{
    (void)aInstance;

    tAlarm = t0 + dt;
    isAlarmActive = true;
}

void otPlatAlarmStop(otInstance *aInstance)
{
	(void)aInstance;
	isAlarmActive = false;
}

void platformAlarmProcess(otInstance *aInstance)
{
	//TODO: This is for tests only: prone to overflow and doesn't allow
	//      the device to properly sleep. Possibly could be replaced with
	//      a timer.

	if (isAlarmActive && (k_uptime_get_32() >= tAlarm)) {
		isAlarmActive = false;
		otPlatAlarmFired(aInstance);
	}
}

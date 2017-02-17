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

#if 0
#ifdef OPENTHREAD_CONFIG_FILE
#include OPENTHREAD_CONFIG_FILE
#else
#include <openthread-config.h>
#endif
#endif

#include <kernel.h>
#include <openthread.h>
#include <openthread-diag.h>
#include <openthread-tasklet.h>
#include <cli/cli-uart.h>
#include <platform/platform.h>
#include <assert.h>

#define SYS_LOG_LEVEL 4
#include <logging/sys_log.h>

void _fini(void)
{

}

void otSignalTaskletPending(otInstance *aInstance)
{
    (void)aInstance;
}

void state_changed_callback(uint32_t flags, void * p_context)
{
	SYS_LOG_INF("State changed! Flags: 0x%08x Current role: %d", flags, otGetDeviceRole(p_context));
}

static otInstance * initialize_thread(void)
{
    otInstance *p_instance;

    p_instance = otInstanceInit();
    assert(p_instance);

    otCliUartInit(p_instance);

    SYS_LOG_INF("Thread version: %s", otGetVersionString());
    SYS_LOG_INF("Network name:   %s", otGetNetworkName(p_instance));

    otSetStateChangedCallback(p_instance, &state_changed_callback, p_instance);

//    otSetChannel(p_instance, 11);
//    otSetPanId(p_instance, 0xabcd);
//    otInterfaceUp(p_instance);
//    otThreadStart(p_instance);

    return p_instance;
}

int main(int argc, char *argv[])
{
    otInstance *sInstance;

    PlatformInit(argc, argv);

    sInstance = initialize_thread();

#if OPENTHREAD_ENABLE_DIAG
    diagInit(sInstance);
#endif

    while (1)
    {
        otProcessQueuedTasklets(sInstance);
        PlatformProcessDrivers(sInstance);

#if 0
        if (k_uptime_get_32() % 1000 == 0) {
        	k_call_stacks_analyze();
        }
#endif

    }

    return 0;
}

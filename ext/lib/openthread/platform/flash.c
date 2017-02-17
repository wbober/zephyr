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

#include <kernel.h>
#include <flash.h>

#include "platform-zephyr.h"

/* TODO: SoC page size */
#define FLASH_PAGE_SIZE     4096

/* Size of flash reserved for OpenThread */
#define FLASH_SIZE          (4*FLASH_PAGE_SIZE)

#define FLASH_START_ADDR (CONFIG_FLASH_BASE_ADDRESS + \
                          CONFIG_FLASH_SIZE*1024 - FLASH_SIZE);

struct device *flash_dev;

static inline uint32_t mapAddress(uint32_t aAddress)
{
	return aAddress + FLASH_START_ADDR;
}

ThreadError utilsFlashInit(void)
{
	ThreadError error = kThreadError_None;
	flash_dev = device_get_binding(
			CONFIG_OPENTHREAD_PLATFORM_FLASH_DEVICE_NAME);

	if (!flash_dev) {
		error = kThreadError_NotImplemented;
	}

	return error;
}

uint32_t utilsFlashGetSize(void)
{
	return FLASH_SIZE;
}

ThreadError utilsFlashErasePage(uint32_t aAddress)
{
	ThreadError error = kThreadError_None;

	if (flash_erase(flash_dev, mapAddress(aAddress), FLASH_PAGE_SIZE)) {
		error = kThreadError_Error;
	}

	return error;
}

ThreadError utilsFlashStatusWait(uint32_t aTimeout)
{
	(void) aTimeout;
	return kThreadError_None;
}

uint32_t utilsFlashWrite(uint32_t aAddress, uint8_t *aData, uint32_t aSize)
{
	uint32_t index = 0;

	flash_write_protection_set(flash_dev, false);
	if (!flash_write(flash_dev, mapAddress(aAddress), aData, aSize)) {
		index = aSize;
	}
	flash_write_protection_set(flash_dev, true);

	return index;
}

uint32_t utilsFlashRead(uint32_t aAddress, uint8_t *aData, uint32_t aSize)
{
	uint32_t index = 0;

	if (!flash_read(flash_dev, mapAddress(aAddress), aData, aSize)) {
		index = aSize;
	}

	return index;
}

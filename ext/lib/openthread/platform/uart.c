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

#include <assert.h>
#include <uart.h>
#include <openthread/platform/uart.h>
#include "platform-zephyr.h"

#define UART_RX_BUFFER_SIZE 256

static struct device *uart_dev;

/**
 *  UART TX buffer variables.
 */
static const uint8_t *sTransmitBuffer = NULL;
static uint16_t sTransmitLength = 0;
static bool sTransmitDone = false;

/**
 *  UART RX ring buffer variables.
 */
static uint8_t sReceiveBuffer[UART_RX_BUFFER_SIZE];
static uint16_t sReceiveHead = 0;
static uint16_t sReceiveTail = 0;

/**
 * Function for checking if RX buffer is full.
 *
 * @retval true  RX buffer is full.
 * @retval false RX buffer is not full.
 */
static inline bool isRxBufferFull()
{
	uint16_t next = (sReceiveHead + 1) % UART_RX_BUFFER_SIZE;
	return (next == sReceiveTail);
}

/**
 * Function for checking if RX buffer is empty.
 *
 * @retval true  RX buffer is empty.
 * @retval false RX buffer is not empty.
 */
static inline bool isRxBufferEmpty()
{
	return (sReceiveHead == sReceiveTail);
}

static void irq_handler(struct device *port)
{
	(void) port;

	if (uart_irq_update(uart_dev) && uart_irq_is_pending(uart_dev)) {
		/*
		 if (uart_api->irq_tx_ready(uart_dev))
		 {
		 if (sTransmitLength)
		 {
		 uart_api->fifo_fill(uart_dev, sTransmitBuffer, 1);
		 sTransmitBuffer++;
		 sTransmitLength--;
		 }
		 else
		 {
		 uart_api->irq_tx_disable(uart_dev);
		 sTransmitDone = true;
		 }
		 }
		 else
		 */
		if (uart_irq_rx_ready(uart_dev)) {
			if (!isRxBufferFull()) {
				uart_fifo_read(uart_dev,
				                sReceiveBuffer + sReceiveHead,
				                1);
				sReceiveHead = (sReceiveHead + 1)
				                % UART_RX_BUFFER_SIZE;
			}
		}
	}
}

void platformUartInit(void)
{
	//TODO: fix device name
	uart_dev = device_get_binding("UART_0");
	assert(uart_dev != NULL);

	uart_irq_rx_disable(uart_dev);
	uart_irq_tx_disable(uart_dev);

	uart_irq_callback_set(uart_dev, irq_handler);
}

ThreadError otPlatUartEnable(void)
{
	uart_irq_rx_enable(uart_dev);

	return kThreadError_None;
}

ThreadError otPlatUartDisable(void)
{
	uart_irq_rx_disable(uart_dev);

	return kThreadError_None;
}

ThreadError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
	ThreadError error = kThreadError_None;
	if (sTransmitBuffer != NULL) {
		return kThreadError_Busy;
	}

	while (aBufLength-- > 0) {
		uart_poll_out(uart_dev, *aBuf++);
	}

	otPlatUartSendDone();

	return error;
}

/**
 * Function for notifying application about new bytes received.
 */
static void processReceive(void)
{
	if (isRxBufferEmpty() == true) {
		return;
	}

	// Set head position to not be changed during read procedure.
	uint16_t head = sReceiveHead;

	// In case head roll back to the beginning of the buffer, notify about left
	// bytes from the end of the buffer.
	if (head < sReceiveTail) {
		otPlatUartReceived(&sReceiveBuffer[sReceiveTail],
		                (UART_RX_BUFFER_SIZE - sReceiveTail));
		sReceiveTail = 0;
	}

	// Notify about received bytes.
	if (head > sReceiveTail) {
		otPlatUartReceived(&sReceiveBuffer[sReceiveTail],
		                (head - sReceiveTail));
		sReceiveTail = head;
	}
}

/**
 * Function for notifying application about transmission being done.
 */
static void processTransmit(void)
{
	if ((sTransmitBuffer != NULL) && sTransmitDone) {
		// Clear Transmition transaction and notify application.
		sTransmitBuffer = NULL;
		sTransmitLength = 0;
		sTransmitDone = false;
		otPlatUartSendDone();
	}

	return;
}

void platformUartProcess(void)
{
	processReceive();
	processTransmit();
}

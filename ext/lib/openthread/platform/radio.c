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

/**
 * @file
 *   This file implements the OpenThread platform abstraction for radio communication.
 *
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define SYS_LOG_LEVEL CONFIG_OPENTHREAD_LOG_LEVEL
#define SYS_LOG_DOMAIN "otPlat/radio"
#include <logging/sys_log.h>

#include <kernel.h>
#include <device.h>
#include <net/ieee802154_radio.h>
#include <net/nbuf.h>

#include <platform/radio.h>
#include <platform/diag.h>

#include <openthread-types.h>

#define FCS_SIZE 2

static PhyState sState = kStateDisabled;

static RadioPacket sTransmitFrame;
static bool        sTransmitPendingBit;

static struct net_buf *tx_pkt;
static struct net_buf *tx_payload;

static struct device *radio_dev;
static struct ieee802154_radio_api *radio_api;
static struct k_fifo rx_queue;

static void dataInit(void)
{
	tx_pkt = net_nbuf_get_reserve_tx(0, K_NO_WAIT);
	assert(tx_pkt != NULL);

	tx_payload = net_nbuf_get_reserve_data(0, K_NO_WAIT);
	assert(tx_payload != NULL);

	net_buf_frag_insert(tx_pkt, tx_payload);

	//TODO: No API to get pending bit.
	sTransmitPendingBit = false;
	sTransmitFrame.mPsdu = tx_payload->data;
}

void ieee802154_init(struct net_if *iface)
{
	(void)iface;
	SYS_LOG_DBG("");
}

int net_recv_data(struct net_if *iface, struct net_buf *pkt)
{
	(void)iface;

	SYS_LOG_DBG("Got data, buf %p, len %d frags->len %d",
		    pkt, pkt->len, net_buf_frags_len(pkt));

	net_buf_put(&rx_queue, pkt);

	return 0;
}

extern enum net_verdict ieee802154_radio_handle_ack(struct net_if *iface,
						    struct net_buf *buf)
{
	(void)iface;
	(void)buf;
	SYS_LOG_DBG("");

	return NET_CONTINUE;
}

int ieee802154_radio_send(struct net_if *iface, struct net_buf *buf)
{
	(void)iface;
	(void)buf;
	SYS_LOG_DBG("");

	return -ENOTSUP;
}

void platformRadioInit(void)
{
    dataInit();

    // TODO: Don't use string as the device name.
    radio_dev = device_get_binding("IEEE802154_nrf5");
    assert(radio_dev != NULL);

    radio_api = (struct ieee802154_radio_api *)radio_dev->driver_api;

    k_fifo_init(&rx_queue);
}

void platformRadioProcess(otInstance *aInstance) {
	struct net_buf *pkt;

	while ((pkt = net_buf_get(&rx_queue, K_NO_WAIT)) != NULL)
	{
		RadioPacket recv_frame;
		recv_frame.mPsdu = net_buf_frag_last(pkt)->data;
		recv_frame.mLength = net_buf_frags_len(pkt); // Length inc. CRC.
		recv_frame.mChannel = 11; // TODO: get channel from packet
		recv_frame.mLqi = 0; // TODO: get LQI from the buffer
		recv_frame.mPower = 0; // TODO: get RSSI from packet

#if OPENTHREAD_ENABLE_DIAG
		if (otPlatDiagModeGet())
		{
			otPlatDiagRadioReceiveDone(aInstance, &recv_frame, kThreadError_None);
		}
		else
#endif
		{
			otPlatRadioReceiveDone(aInstance, &recv_frame,
					kThreadError_None);
		}

		net_buf_unref(pkt);
	}

	if (sState == kStateTransmit)
	{
		ThreadError result = kThreadError_None;

		// The payload is already in tx_payload->data, but we need to set
		// the length field according to sTransmitFrame.length. We subtract
		// the FCS size as radio drivers adds CRC and increases frame
		// length on its own.
		tx_payload->len = sTransmitFrame.mLength - FCS_SIZE;

		radio_api->set_channel(radio_dev, sTransmitFrame.mChannel);

		if (radio_api->tx(radio_dev, tx_pkt, tx_payload) == -EBUSY) {
			result = kThreadError_ChannelAccessFailure;
		}

		sState = kStateReceive;

#if OPENTHREAD_ENABLE_DIAG

		if (otPlatDiagModeGet())
		{
			otPlatDiagRadioTransmitDone(aInstance, &sTransmitFrame, sTransmitPendingBit, result);
		}
		else
#endif
		{
			otPlatRadioTransmitDone(aInstance, &sTransmitFrame,
					sTransmitPendingBit, result);
		}
	}
}

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    (void) aInstance;

    // TODO: No API in Zephyr to get the factory address.
    uint64_t factoryAddress = 0x1122334455667788;
    memcpy(aIeeeEui64, &factoryAddress, sizeof(factoryAddress));
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t aPanId)
{
    (void) aInstance;

    radio_api->set_pan_id(radio_dev, aPanId);
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, uint8_t *aExtendedAddress)
{
    (void) aInstance;
    radio_api->set_ieee_addr(radio_dev, aExtendedAddress);
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aShortAddress)
{
    (void) aInstance;

    radio_api->set_short_addr(radio_dev, aShortAddress);
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    (void)aInstance;

    return (sState != kStateDisabled) ? true : false;
}

ThreadError otPlatRadioEnable(otInstance *aInstance)
{
    if (!otPlatRadioIsEnabled(aInstance))
    {
        sState = kStateSleep;
    }

    return kThreadError_None;
}

ThreadError otPlatRadioDisable(otInstance *aInstance)
{
    if (otPlatRadioIsEnabled(aInstance))
    {
        sState = kStateDisabled;
    }

    return kThreadError_None;
}

ThreadError otPlatRadioSleep(otInstance *aInstance)
{
    ThreadError error = kThreadError_InvalidState;
    (void)aInstance;

    if (sState == kStateSleep || sState == kStateReceive)
    {
        error = kThreadError_None;
        sState = kStateSleep;
        radio_api->stop(radio_dev);
    }

    return error;
}

ThreadError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
    (void) aInstance;

    radio_api->set_channel(radio_dev, aChannel);
    radio_api->start(radio_dev);
    sState = kStateReceive;

    return kThreadError_None;
}

ThreadError otPlatRadioTransmit(otInstance *aInstance, RadioPacket *aPacket)
{
    ThreadError error = kThreadError_InvalidState;
    (void)aInstance;
    (void)aPacket;

    //TODO: AFIAK OT uses a single packet. If that's not the case
    //      then we need to allocate net_bufs here and place them into a
    //      TX queue.
    assert(aPacket == sTransmitFrame);

    if (sState == kStateReceive)
    {
        error = kThreadError_None;
        sState = kStateTransmit;
    }

    return error;
}

RadioPacket *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    (void) aInstance;

    return &sTransmitFrame;
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    (void) aInstance;

    // TODO: No API in Zephyr to get the RSSI.
    return 0;
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    (void) aInstance;

    return kRadioCapsNone;
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    (void) aInstance;
    // TODO: No API in Zephyr to get promiscuous mode.
    bool result = false;
    SYS_LOG_DBG("PromiscuousMode=%d", result ? 1 : 0);
    return result;
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    (void) aInstance;
    (void) aEnable;

    SYS_LOG_DBG("PromiscuousMode=%d", aEnable ? 1 : 0);
    // TODO: No API in Zephyr to get promiscuous mode.
}

ThreadError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    (void)aInstance;
    (void)aScanChannel;
    (void)aScanDuration;

    return kThreadError_NotImplemented;
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
    (void)aInstance;
    (void)aEnable;
}

ThreadError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    (void)aInstance;
    (void)aShortAddress;
    return kThreadError_None;
}

ThreadError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
    (void)aInstance;
    (void)aExtAddress;
    return kThreadError_None;
}

ThreadError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    (void)aInstance;
    (void)aShortAddress;
    return kThreadError_None;
}

ThreadError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
    (void)aInstance;
    (void)aExtAddress;
    return kThreadError_None;
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
    (void)aInstance;
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
    (void)aInstance;
}

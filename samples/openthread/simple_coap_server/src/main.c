/* Copyright (c) 2016 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup simple_coap_server_example_main main.c
 * @{
 * @ingroup simple_coap_server_example_example
 * @brief Simple CoAP Server Example Application main file.
 *
 * @details This example demonstrates a CoAP server application that provides resources to control BSP_LED_0
 *          via CoAP messages. It can be controlled by a board with related Simple CoAP Server application.
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <openthread.h>
#include <openthread-diag.h>
#include <openthread-tasklet.h>
#include <openthread-coap.h>
#include <openthread-message.h>
#include <cli/cli-uart.h>
#include <platform/platform.h>
#include <platform/alarm.h>

#define LED_INTERVAL             100

//APP_TIMER_DEF(m_provisioning_timer);
//APP_TIMER_DEF(m_led_timer);

static void light_request_handler(void                * p_context,
                                  otCoapHeader        * p_header,
                                  otMessage             p_message,
                                  const otMessageInfo * p_message_info);

static void provisioning_request_handler(void                * p_context,
                                         otCoapHeader        * p_header,
                                         otMessage             p_message,
                                         const otMessageInfo * p_message_info);

#define PROVISIONING_EXPIRY_TIME 5000

typedef enum
{
    DEVICE_TYPE_REMOTE_CONTROL,
    DEVICE_TYPE_LIGHT
} device_type_t;

typedef enum
{
    LED_OFF,
    LED_ON,
    LED_BLINK
} led_state_t;

typedef enum
{
    LIGHT_OFF = 0,
    LIGHT_ON,
    LIGHT_TOGGLE
} light_command_t;

typedef struct
{
    otInstance     * p_ot_instance;         /**< A pointer to the OpenThread instance. */
    bool             enable_provisioning;   /**< Information if provisioning is enabled. */
    uint32_t         provisioning_expiry;   /**< Provisioning timeout time. */
    otCoapResource   provisioning_resource; /**< CoAP provisioning resource. */
    otCoapResource   light_resource;        /**< CoAP light resource. */
    led_state_t      network_state_led;     /**< Network state LED. */
} application_t;

application_t m_app =
{
    .p_ot_instance         = NULL,
    .enable_provisioning   = false,
    .provisioning_expiry   = 0,
    .provisioning_resource = {"provisioning", provisioning_request_handler, NULL, NULL},
    .light_resource        = {"light", light_request_handler, NULL, NULL},
    .network_state_led     = LED_BLINK
};

void otSignalTaskletPending(otInstance *aInstance)
{
    (void)aInstance;
}

/***************************************************************************************************
 * @section CoAP
 **************************************************************************************************/

static void light_on(void)
{
//    LEDS_ON(BSP_LED_0_MASK);
}

static void light_off(void)
{
//    LEDS_OFF(BSP_LED_0_MASK);
}

static void light_toggle(void)
{
//    LEDS_INVERT(BSP_LED_0_MASK);
}

static void provisioning_disable(otInstance * p_instance)
{
    m_app.enable_provisioning = false;
    m_app.provisioning_expiry = 0;
    otCoapServerRemoveResource(p_instance, &m_app.provisioning_resource);
    app_timer_stop(m_provisioning_timer);
}

static void provisioning_enable(otInstance * p_instance)
{
    m_app.enable_provisioning = true;
    m_app.provisioning_expiry = otPlatAlarmGetNow() + PROVISIONING_EXPIRY_TIME;
    otCoapServerAddResource(p_instance, &m_app.provisioning_resource);
    app_timer_start(m_provisioning_timer,
                    APP_TIMER_TICKS(PROVISIONING_EXPIRY_TIME, APP_TIMER_PRESCALER),
                    p_instance);
}

static void light_response_send(void                * p_context,
                                otCoapHeader        * p_request_header,
                                const otMessageInfo * p_message_info)
{
    ThreadError  error = kThreadError_None;
    otCoapHeader header;
    otMessage    response;

    do
    {
        otCoapHeaderInit(&header, kCoapTypeAcknowledgment, kCoapResponseChanged);
        otCoapHeaderSetMessageId(&header, otCoapHeaderGetMessageId(p_request_header));
        otCoapHeaderSetToken(&header,
                             otCoapHeaderGetToken(p_request_header),
                             otCoapHeaderGetTokenLength(p_request_header));

        response = otCoapNewMessage(p_context, &header);
        if (response == NULL)
        {
            break;
        }

        error = otCoapSendResponse(p_context, response, p_message_info);

    } while (false);

    if (error != kThreadError_None && response != NULL)
    {
        otFreeMessage(response);
    }
}

static void light_request_handler(void                * p_context,
                                  otCoapHeader        * p_header,
                                  otMessage             p_message,
                                  const otMessageInfo * p_message_info)
{
    (void)p_message;
    uint8_t command;

    do
    {
        if (otCoapHeaderGetType(p_header) != kCoapTypeConfirmable &&
            otCoapHeaderGetType(p_header) != kCoapTypeNonConfirmable)
        {
            break;
        }

        if (otCoapHeaderGetCode(p_header) != kCoapRequestPut)
        {
            break;
        }

        if (otReadMessage(p_message, otGetMessageOffset(p_message), &command, 1) != 1)
        {
            NRF_LOG_INFO("light handler - missing command\r\n");
        }

        switch (command)
        {
            case LIGHT_ON:
                light_on();
                break;

            case LIGHT_OFF:
                light_off();
                break;

            case LIGHT_TOGGLE:
                light_toggle();
                break;

            default:
                break;
        }

        if (otCoapHeaderGetType(p_header) == kCoapTypeConfirmable)
        {
            light_response_send(p_context, p_header, p_message_info);
        }

    } while (false);
}

static ThreadError provisioning_response_send(void                * p_context,
                                              otCoapHeader        * p_request_header,
                                              uint8_t               device_type,
                                              const otMessageInfo * p_message_info)
{
    ThreadError  error = kThreadError_NoBufs;
    otCoapHeader header;
    otMessage    response;

    do
    {
        otCoapHeaderInit(&header, kCoapTypeNonConfirmable, kCoapResponseContent);
        otCoapHeaderSetToken(&header,
                             otCoapHeaderGetToken(p_request_header),
                             otCoapHeaderGetTokenLength(p_request_header));
        otCoapHeaderSetPayloadMarker(&header);

        response = otCoapNewMessage(p_context, &header);
        if (response == NULL)
        {
            break;
        }

        error = otAppendMessage(response, &device_type, 1);
        if (error != kThreadError_None)
        {
            break;
        }

        error = otAppendMessage(response, otGetMeshLocalEid(p_context), sizeof(otIp6Address));
        if (error != kThreadError_None)
        {
            break;
        }

        error = otCoapSendResponse(p_context, response, p_message_info);

    } while (false);

    if (error != kThreadError_None && response != NULL)
    {
        otFreeMessage(response);
    }

    return error;
}

static void provisioning_request_handler(void                * p_context,
                                         otCoapHeader        * p_header,
                                         otMessage             p_message,
                                         const otMessageInfo * p_message_info)
{
    (void)p_message;
    otMessageInfo message_info;

    if (otCoapHeaderGetType(p_header) == kCoapTypeNonConfirmable &&
        otCoapHeaderGetCode(p_header) == kCoapRequestGet)
    {
        message_info = *p_message_info;
        memset(&message_info.mSockAddr, 0, sizeof(message_info.mSockAddr));
        if (provisioning_response_send(p_context, p_header, DEVICE_TYPE_LIGHT, &message_info) ==
                kThreadError_None)
        {
            provisioning_disable(p_context);
        }
    }
}

/***************************************************************************************************
 * @section State
 **************************************************************************************************/

static void handle_role_change(void * p_context, otDeviceRole role)
{
    switch(role)
    {
        case kDeviceRoleChild:
        case kDeviceRoleRouter:
        case kDeviceRoleLeader:
            m_app.network_state_led = LED_ON;
            break;

        case kDeviceRoleOffline:
        case kDeviceRoleDisabled:
        case kDeviceRoleDetached:
        default:
            m_app.network_state_led = LED_BLINK;
            provisioning_disable(p_context);
            break;
    }
}

static void state_changed_callback(uint32_t flags, void * p_context)
{
    if (flags & OT_NET_ROLE)
    {
        handle_role_change(p_context, otGetDeviceRole(p_context));
    }

    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n", flags, otGetDeviceRole(p_context));
}

/***************************************************************************************************
 * @section Buttons
 **************************************************************************************************/

static void bsp_event_handler(bsp_event_t event)
{
#if 0
    switch (event)
    {
        case BSP_EVENT_KEY_0:
            break;

        case BSP_EVENT_KEY_1:
            break;

        case BSP_EVENT_KEY_2:
            break;

        case BSP_EVENT_KEY_3:
            provisioning_enable(m_app.p_ot_instance);
            break;

        default:
            return;
    }
#endif
}

/***************************************************************************************************
 * @section Timers
 **************************************************************************************************/

static void provisioning_timer_handler(void * p_context)
{
    provisioning_disable(p_context);
}

static void led_timer_handler(void * p_context)
{
    (void)p_context;
    static int state_led_counter = 0;

#if 0
    switch (m_app.network_state_led)
    {
        case LED_ON:
            LEDS_ON(BSP_LED_3_MASK);
            break;

        case LED_BLINK:
            state_led_counter = (state_led_counter + 1) % 2;
            if (!state_led_counter)
            {
                LEDS_INVERT(BSP_LED_3_MASK);
            }
            break;

        case LED_OFF:
        default:
            LEDS_OFF(BSP_LED_3_MASK);
            break;
    }

    if (m_app.enable_provisioning)
    {
        LEDS_INVERT(BSP_LED_2_MASK);
    }
    else
    {
        LEDS_OFF(BSP_LED_2_MASK);
    }
#endif
}

/***************************************************************************************************
 * @section Initialization
 **************************************************************************************************/

static otInstance * initialize_thread(void)
{
    otInstance * p_instance;

    p_instance = otInstanceInit();
    assert(p_instance);

    otCliUartInit(p_instance);

    NRF_LOG_INFO("Thread version: %s\r\n", (uint32_t)otGetVersionString());
    NRF_LOG_INFO("Network name:   %s\r\n", (uint32_t)otGetNetworkName(p_instance));

    assert(otSetStateChangedCallback(p_instance, &state_changed_callback, p_instance) == kThreadError_None);

    assert(otSetChannel(p_instance, CONFIG_OPENTHREAD_CHANNEL) == kThreadError_None);
    assert(otSetPanId(p_instance, CONFIG_OPENTHREAD_PANID) == kThreadError_None);
    assert(otInterfaceUp(p_instance) == kThreadError_None);
    assert(otThreadStart(p_instance) == kThreadError_None);

    return p_instance;
}

static void initialize_coap(otInstance * p_instance)
{
    m_app.light_resource.mContext = p_instance;
    m_app.provisioning_resource.mContext = p_instance;

    assert(otCoapServerStart(p_instance) == kThreadError_None);
    assert(otCoapServerAddResource(p_instance, &m_app.light_resource) == kThreadError_None);
}

static void initialize_timer(void)
{
#if 0
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    app_timer_create(&m_provisioning_timer, APP_TIMER_MODE_SINGLE_SHOT, provisioning_timer_handler);
    app_timer_create(&m_led_timer, APP_TIMER_MODE_REPEATED, led_timer_handler);
#endif
}

static void initialize_bsp(void)
{
#if 0
     uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                  APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                  bsp_event_handler);
    APP_ERROR_CHECK(err_code);
#endif
}

static void initialize_leds(void)
{
#if 0
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);

    app_timer_start(m_led_timer, APP_TIMER_TICKS(LED_INTERVAL, APP_TIMER_PRESCALER), NULL);
#endif
}

/***************************************************************************************************
 * @section Main
 **************************************************************************************************/

int main(int argc, char *argv[])
{
    NRF_LOG_INIT(NULL);

    PlatformInit(argc, argv);
    m_app.p_ot_instance = initialize_thread();
    initialize_coap(m_app.p_ot_instance);

    initialize_timer();
    initialize_bsp();
    initialize_leds();

    while (true)
    {
        otProcessQueuedTasklets(m_app.p_ot_instance);
        PlatformProcessDrivers(m_app.p_ot_instance);
    }

    return 0;
}

/**
 *@}
 **/

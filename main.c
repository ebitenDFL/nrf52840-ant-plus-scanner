#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "nrf.h"
#include "nrf_error.h"
#include "app_error.h"
#include "app_usbd.h"
#include "app_usbd_cdc_acm.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "ant_channel_config.h"
#include "ant_plus_key.h"

#define ANT_CHANNEL_NUMBER      0u
#define ANT_NETWORK_NUMBER      0u
#define ANTPLUS_RF_FREQUENCY   57u
#define USB_LINE_MAX           128u

#define CDC_ACM_COMM_INTERFACE 0
#define CDC_ACM_COMM_EPIN      NRF_DRV_USBD_EPIN2
#define CDC_ACM_DATA_INTERFACE 1
#define CDC_ACM_DATA_EPIN      NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT     NRF_DRV_USBD_EPOUT1

static bool m_cdc_open = false;
static char m_usb_line[USB_LINE_MAX];

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_NONE);

static void usb_write(char const * text)
{
    if (!m_cdc_open || text == NULL) return;
    size_t len = strlen(text);
    if (len == 0) return;
    (void)app_usbd_cdc_acm_write(&m_app_cdc_acm, text, len);
}

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    (void)p_inst;
    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
            m_cdc_open = true;
            usb_write("ANTSCANNER,READY\r\n");
            break;
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            m_cdc_open = false;
            break;
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    (void)event;
}

static void usb_init(void)
{
    ret_code_t err;
    static const app_usbd_config_t usbd_config = { .ev_state_proc = usbd_user_ev_handler };
    err = app_usbd_init(&usbd_config); APP_ERROR_CHECK(err);
    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    err = app_usbd_class_append(class_cdc_acm); APP_ERROR_CHECK(err);
    app_usbd_enable();
    app_usbd_start();
}

static void ant_print_rx(ant_evt_t * p_ant_evt)
{
    ANT_MESSAGE * msg = &p_ant_evt->message;
    uint8_t const * payload = msg->ANT_MESSAGE_aucPayload;
    uint8_t const * ext = msg->ANT_MESSAGE_aucExtData;
    uint8_t ext_bf = msg->ANT_MESSAGE_ucExtMesgBF;
    uint16_t dev_num = 0;
    uint8_t dev_type = 0;
    uint8_t trans_type = 0;
    int rssi_dbm = 127;
    uint8_t idx = 0;

    if ((ext_bf & ANT_EXT_MESG_BITFIELD_DEVICE_ID) != 0)
    {
        dev_num = (uint16_t)ext[idx] | ((uint16_t)ext[idx + 1] << 8);
        dev_type = ext[idx + 2] & 0x7Fu;
        trans_type = ext[idx + 3];
        idx += ANT_EXT_MESG_DEVICE_ID_FIELD_SIZE;
    }

    if ((ext_bf & ANT_EXT_MESG_BITFIELD_RSSI) != 0)
    {
        if (ext[idx] == RSSI_DBM_TYPE)
        {
            rssi_dbm = (int)((int8_t)ext[idx + 1]);
        }
    }

    int n = snprintf(m_usb_line, sizeof(m_usb_line),
                     "ANT,%u,%u,%u,%d,%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
                     (unsigned)dev_num,
                     (unsigned)dev_type,
                     (unsigned)trans_type,
                     rssi_dbm,
                     payload[0], payload[1], payload[2], payload[3],
                     payload[4], payload[5], payload[6], payload[7]);
    if (n > 0) usb_write(m_usb_line);
}

static void ant_evt_handler(ant_evt_t * p_ant_evt, void * p_context)
{
    (void)p_context;
    if (p_ant_evt == NULL) return;
    if (p_ant_evt->event == EVENT_RX) ant_print_rx(p_ant_evt);
}

NRF_SDH_ANT_OBSERVER(m_ant_observer, 1, ant_evt_handler, NULL);

static void ant_init(void)
{
    ret_code_t err;
    err = nrf_sdh_enable_request(); APP_ERROR_CHECK(err);
    err = nrf_sdh_ant_enable(); APP_ERROR_CHECK(err);
    err = sd_ant_network_address_set(ANT_NETWORK_NUMBER, ANT_PLUS_NETWORK_KEY); APP_ERROR_CHECK(err);
    err = sd_ant_lib_config_set(ANT_LIB_CONFIG_MESG_OUT_INC_DEVICE_ID |
                                ANT_LIB_CONFIG_MESG_OUT_INC_RSSI); APP_ERROR_CHECK(err);

    ant_channel_config_t channel_config =
    {
        .channel_number    = ANT_CHANNEL_NUMBER,
        .channel_type      = CHANNEL_TYPE_SLAVE_RX_ONLY,
        .ext_assign        = EXT_PARAM_ALWAYS_SEARCH,
        .rf_freq           = ANTPLUS_RF_FREQUENCY,
        .transmission_type = 0,
        .device_type       = 0,
        .device_number     = 0,
        .channel_period    = 0,
        .network_number    = ANT_NETWORK_NUMBER
    };

    err = ant_channel_init(&channel_config); APP_ERROR_CHECK(err);
    err = sd_ant_channel_rx_search_timeout_set(ANT_CHANNEL_NUMBER, 0); APP_ERROR_CHECK(err);
    err = sd_ant_channel_low_priority_rx_search_timeout_set(ANT_CHANNEL_NUMBER, 0xFF); APP_ERROR_CHECK(err);
    err = sd_ant_channel_open(ANT_CHANNEL_NUMBER); APP_ERROR_CHECK(err);
}

int main(void)
{
    usb_init();
    ant_init();
    for (;;)
    {
        while (app_usbd_event_queue_process()) {}
        __WFE();
    }
}

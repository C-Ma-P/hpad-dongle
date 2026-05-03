#include "radio_esb.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#if __has_include(<esb.h>)
#include <esb.h>

#define RADIO_ESB_PAYLOAD struct esb_payload
#define RADIO_ESB_EVENT struct esb_evt
#define RADIO_ESB_CONFIG struct esb_config
#define RADIO_ESB_DEFAULT_CONFIG ESB_DEFAULT_CONFIG
#define RADIO_ESB_EVENT_RX_RECEIVED ESB_EVENT_RX_RECEIVED
#define RADIO_ESB_EVENT_TX_SUCCESS ESB_EVENT_TX_SUCCESS
#define RADIO_ESB_EVENT_TX_FAILED ESB_EVENT_TX_FAILED
#define RADIO_ESB_PROTOCOL_DPL ESB_PROTOCOL_ESB_DPL
#define RADIO_ESB_BITRATE_2MBPS ESB_BITRATE_2MBPS
#define RADIO_ESB_MODE_PRX ESB_MODE_PRX
#define radio_esb_api_flush_rx esb_flush_rx
#define radio_esb_api_flush_tx esb_flush_tx
#define radio_esb_api_init esb_init
#define radio_esb_api_read_rx_payload esb_read_rx_payload
#define radio_esb_api_set_base_address_0 esb_set_base_address_0
#define radio_esb_api_set_base_address_1 esb_set_base_address_1
#define radio_esb_api_set_prefixes esb_set_prefixes
#define radio_esb_api_set_rf_channel esb_set_rf_channel
#define radio_esb_api_start_rx esb_start_rx
#define radio_esb_api_write_payload esb_write_payload
#elif __has_include(<nrf_esb.h>)
#include <nrf_esb.h>

#define RADIO_ESB_PAYLOAD nrf_esb_payload_t
#define RADIO_ESB_EVENT nrf_esb_evt_t
#define RADIO_ESB_CONFIG nrf_esb_config_t
#define RADIO_ESB_DEFAULT_CONFIG NRF_ESB_DEFAULT_CONFIG
#define RADIO_ESB_EVENT_RX_RECEIVED NRF_ESB_EVENT_RX_RECEIVED
#define RADIO_ESB_EVENT_TX_SUCCESS NRF_ESB_EVENT_TX_SUCCESS
#define RADIO_ESB_EVENT_TX_FAILED NRF_ESB_EVENT_TX_FAILED
#define RADIO_ESB_PROTOCOL_DPL NRF_ESB_PROTOCOL_ESB_DPL
#define RADIO_ESB_BITRATE_2MBPS NRF_ESB_BITRATE_2MBPS
#define RADIO_ESB_MODE_PRX NRF_ESB_MODE_PRX
#define radio_esb_api_flush_rx nrf_esb_flush_rx
#define radio_esb_api_flush_tx nrf_esb_flush_tx
#define radio_esb_api_init nrf_esb_init
#define radio_esb_api_read_rx_payload nrf_esb_read_rx_payload
#define radio_esb_api_set_base_address_0 nrf_esb_set_base_address_0
#define radio_esb_api_set_base_address_1 nrf_esb_set_base_address_1
#define radio_esb_api_set_prefixes nrf_esb_set_prefixes
#define radio_esb_api_set_rf_channel nrf_esb_set_rf_channel
#define radio_esb_api_start_rx nrf_esb_start_rx
#define radio_esb_api_write_payload nrf_esb_write_payload
#else
#error "Unable to find an ESB header for this SDK"
#endif

LOG_MODULE_REGISTER(radio_esb, LOG_LEVEL_INF);

#define RADIO_EVENT_RX BIT(0)

static struct k_work radio_work;
static struct k_mutex radio_tx_lock;
static atomic_t pending_events;
static radio_esb_report_handler_t macropad_report_handler;
static bool radio_initialized;

static void radio_esb_schedule_worker(void)
{
	(void)k_work_submit(&radio_work);
}

static void radio_esb_event_handler(const RADIO_ESB_EVENT *event)
{
	switch (event->evt_id) {
	case RADIO_ESB_EVENT_RX_RECEIVED:
		atomic_or(&pending_events, RADIO_EVENT_RX);
		break;
	default:
		break;
	}

	radio_esb_schedule_worker();
}

static void radio_esb_work_handler(struct k_work *work)
{
	RADIO_ESB_PAYLOAD payload;
	macropad_report_t report;
	int rc;
	atomic_val_t events;

	ARG_UNUSED(work);

	events = atomic_set(&pending_events, 0);
	if ((events & RADIO_EVENT_RX) == 0) {
		return;
	}

	memset(&payload, 0, sizeof(payload));
	while ((rc = radio_esb_api_read_rx_payload(&payload)) == 0) {
		if (payload.length != sizeof(report)) {
			LOG_WRN("Rejected payload on pipe %u len=%u: length mismatch",
				payload.pipe, payload.length);
			memset(&payload, 0, sizeof(payload));
			continue;
		}

		memcpy(&report, payload.data, sizeof(report));

		LOG_DBG("Macropad packet received: keys=0x%02x encoder_delta=%d encoder_pressed=%u",
			report.keys, report.encoder_delta, report.encoder_pressed);

		if (macropad_report_handler != NULL) {
			macropad_report_handler(&report);
		}

		memset(&payload, 0, sizeof(payload));
	}
}

int radio_esb_init(const struct esb_addr_config *addr_config, uint8_t rf_channel,
		   radio_esb_report_handler_t report_handler)
{
	RADIO_ESB_CONFIG config = RADIO_ESB_DEFAULT_CONFIG;
	int rc;

	if (addr_config == NULL) {
		return -EINVAL;
	}

	config.protocol = RADIO_ESB_PROTOCOL_DPL;
	config.bitrate = RADIO_ESB_BITRATE_2MBPS;
	config.mode = RADIO_ESB_MODE_PRX;
	config.event_handler = radio_esb_event_handler;
	config.selective_auto_ack = false;
	config.retransmit_delay = 600U;
	config.retransmit_count = 3U;
	config.payload_length = sizeof(macropad_report_t);

	k_work_init(&radio_work, radio_esb_work_handler);
	k_mutex_init(&radio_tx_lock);
	atomic_clear(&pending_events);
	macropad_report_handler = report_handler;

	rc = radio_esb_api_init(&config);
	if (rc != 0) {
		LOG_ERR("ESB init failed: %d", rc);
		return rc;
	}

	rc = radio_esb_api_set_base_address_0(addr_config->base_addr_0);
	if (rc != 0) {
		LOG_ERR("Failed to set base address 0: %d", rc);
		return rc;
	}

	rc = radio_esb_api_set_base_address_1(addr_config->base_addr_1);
	if (rc != 0) {
		LOG_ERR("Failed to set base address 1: %d", rc);
		return rc;
	}

	rc = radio_esb_api_set_prefixes(addr_config->prefixes, ARRAY_SIZE(addr_config->prefixes));
	if (rc != 0) {
		LOG_ERR("Failed to set prefixes: %d", rc);
		return rc;
	}

	rc = radio_esb_api_set_rf_channel(rf_channel);
	if (rc != 0) {
		LOG_ERR("Failed to set RF channel: %d", rc);
		return rc;
	}

	radio_initialized = true;
	LOG_INF("ESB initialized in PRX mode on channel %u", rf_channel);
	return 0;
}

int radio_esb_start(void)
{
	int rc;

	if (!radio_initialized) {
		return -EAGAIN;
	}

	radio_esb_api_flush_tx();
	radio_esb_api_flush_rx();
	rc = radio_esb_api_start_rx();
	if (rc != 0) {
		LOG_ERR("Failed to start ESB RX: %d", rc);
		return rc;
	}

	LOG_INF("ESB receiver started");
	return 0;
}

int radio_esb_queue_macropad_config(const macropad_config_t *config)
{
	RADIO_ESB_PAYLOAD payload;
	int rc;

	if (!radio_initialized) {
		return -EAGAIN;
	}
	if ((config == NULL) || (config->kind != HPAD_PROTOCOL_CONFIG_KIND_KEY_COLORS)) {
		return -EINVAL;
	}

	memset(&payload, 0, sizeof(payload));
	payload.pipe = 0U;
	payload.length = sizeof(*config);
	memcpy(payload.data, config, sizeof(*config));

	k_mutex_lock(&radio_tx_lock, K_FOREVER);
	radio_esb_api_flush_tx();
	rc = radio_esb_api_write_payload(&payload);
	k_mutex_unlock(&radio_tx_lock);
	if (rc != 0) {
		LOG_WRN("Failed to queue macropad config ACK payload: %d", rc);
		return rc;
	}

	LOG_INF("Queued macropad color config for ACK delivery");
	return 0;
}
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "consumer_action_engine.h"
#include "dongle_config.h"
#include "hid_vendor.h"
#include "radio_esb.h"
#include "radio_identity.h"
#include "usb_device.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define LED_NODE DT_ALIAS(led0)
#define RX_BLINK_MS 100
#define MACROPAD_KEY_MASK BIT_MASK(6)
#define MACROPAD_CONNECTION_TIMEOUT_MS 2500

static const struct gpio_dt_spec status_led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static struct k_work_delayable led_off_work;
static struct k_work_delayable macropad_disconnect_work;
static bool macropad_connected;

static void macropad_disconnect_work_handler(struct k_work *work);
static void handle_macropad_config(const macropad_config_t *config);

static void led_off_work_handler(struct k_work *work)
{
	int rc;

	ARG_UNUSED(work);

	rc = gpio_pin_set_dt(&status_led, 0);
	if (rc != 0) {
		LOG_ERR("Failed to turn LED off: %d", rc);
	}
}

static int status_led_init(void)
{
	int rc;

	if (!gpio_is_ready_dt(&status_led)) {
		return -ENODEV;
	}

	k_work_init_delayable(&led_off_work, led_off_work_handler);
	k_work_init_delayable(&macropad_disconnect_work, macropad_disconnect_work_handler);
	rc = gpio_pin_configure_dt(&status_led, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		LOG_ERR("Failed to configure status LED: %d", rc);
		return rc;
	}

	LOG_INF("Status LED initialized on port=%s pin=%u", status_led.port->name, status_led.pin);
	return 0;
}

static void blink_status_led_once(void)
{
	int rc;

	rc = gpio_pin_set_dt(&status_led, 1);
	if (rc != 0) {
		LOG_ERR("Failed to turn LED on: %d", rc);
		return;
	}

	(void)k_work_reschedule(&led_off_work, K_MSEC(RX_BLINK_MS));
}

static void forward_macropad_report(const macropad_report_t *report, bool connected)
{
	host_macropad_report_t host_report = {
		.connected = connected ? 1U : 0U,
		.keys = report->keys,
		.encoder_delta = report->encoder_delta,
		.encoder_pressed = report->encoder_pressed,
		.battery_mv = report->battery_mv,
		.usb_power_present = report->usb_power_present,
	};

	(void)hid_vendor_forward_macropad_report(&host_report);
}

static void macropad_disconnect_work_handler(struct k_work *work)
{
	const macropad_report_t report = {
		.keys = 0U,
		.encoder_delta = 0,
		.encoder_pressed = 0U,
		.battery_mv = 0U,
		.usb_power_present = 0U,
	};

	ARG_UNUSED(work);

	if (!macropad_connected) {
		return;
	}

	macropad_connected = false;
	consumer_action_engine_reset();
	forward_macropad_report(&report, false);
}

static void handle_macropad_report(const macropad_report_t *report)
{
	uint8_t unsupported_keys;
	bool input_activity;

	macropad_connected = true;
	(void)k_work_reschedule(&macropad_disconnect_work,
		K_MSEC(MACROPAD_CONNECTION_TIMEOUT_MS));
	forward_macropad_report(report, true);

	unsupported_keys = report->keys & (uint8_t)~MACROPAD_KEY_MASK;
	if (unsupported_keys != 0U) {
		LOG_WRN("Ignoring unsupported key bits: 0x%02x", unsupported_keys);
	}

	input_activity = consumer_action_engine_input_activity(report);
	if (input_activity) {
		blink_status_led_once();
	}

	consumer_action_engine_process_report(report);
}

static void handle_macropad_config(const macropad_config_t *config)
{
	int rc = radio_esb_queue_macropad_config(config);

	if (rc != 0) {
		LOG_WRN("Failed to queue macropad config: %d", rc);
	}
}

int main(void)
{
	struct dongle_identity identity;
	struct esb_addr_config addr_config;
	uint8_t rf_channel;
	bool generated;
	int rc;

	LOG_INF("Dongle boot start");

	LOG_INF("Initializing dongle config");
	rc = dongle_config_init();
	if (rc != 0) {
		LOG_ERR("dongle_config_init failed: %d", rc);
		return 0;
	}

	LOG_INF("Initializing USB composite HID + CDC ACM");
	hid_vendor_set_config_handler(handle_macropad_config);
	rc = usb_device_init();
	if (rc != 0) {
		LOG_ERR("usb_device_init failed: %d", rc);
		return 0;
	}

	LOG_INF("Initializing status LED");
	rc = status_led_init();
	if (rc != 0) {
		LOG_ERR("status_led_init failed: %d", rc);
		return 0;
	}
	blink_status_led_once();

	LOG_INF("Initializing persistent identity settings");
	rc = radio_identity_init();
	if (rc != 0) {
		LOG_ERR("radio_identity_init failed: %d", rc);
		return 0;
	}

	LOG_INF("Loading or creating dongle identity");
	rc = radio_identity_load_or_create(&identity, &generated);
	if (rc != 0) {
		LOG_ERR("radio_identity_load_or_create failed: %d", rc);
		return 0;
	}

	LOG_INF("Dongle identity synchronized");

	rc = radio_identity_derive_esb_config(&identity, &addr_config, &rf_channel);
	if (rc != 0) {
		LOG_ERR("radio_identity_derive_esb_config failed: %d", rc);
		return 0;
	}

	radio_identity_log_esb_config(&addr_config, rf_channel);

	LOG_INF("Initializing ESB receiver");
	rc = radio_esb_init(&addr_config, rf_channel, handle_macropad_report);
	if (rc != 0) {
		LOG_ERR("radio_esb_init failed: %d", rc);
		return 0;
	}

	LOG_INF("Starting ESB RX");
	rc = radio_esb_start();
	if (rc != 0) {
		LOG_ERR("radio_esb_start failed: %d", rc);
		return 0;
	}

	LOG_INF("Dongle ready and listening for macropad packets");

	while (true) {
		k_sleep(K_FOREVER);
	}

	return 0;
}

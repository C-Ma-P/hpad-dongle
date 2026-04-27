#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "dongle_config.h"
#include "radio_esb.h"
#include "radio_identity.h"
#include "usb_hid_consumer.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define LED_NODE DT_ALIAS(led0)
#define RX_BLINK_MS 100
#define MACROPAD_KEY_MASK BIT_MASK(6)

static const struct gpio_dt_spec status_led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static struct k_work_delayable led_off_work;
static uint8_t previous_keys;
static bool previous_encoder_pressed;

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

static void handle_macropad_report(const macropad_report_t *report)
{
	uint8_t current_keys;
	uint8_t unsupported_keys;
	bool input_activity;
	int rc;

	(void)usb_hid_consumer_forward_macropad_report(report);

	unsupported_keys = report->keys & (uint8_t)~MACROPAD_KEY_MASK;
	if (unsupported_keys != 0U) {
		LOG_WRN("Ignoring unsupported key bits: 0x%02x", unsupported_keys);
	}

	current_keys = report->keys & MACROPAD_KEY_MASK;
	input_activity = (current_keys != previous_keys) ||
		(report->encoder_delta != 0) ||
		(previous_encoder_pressed != (report->encoder_pressed != 0U));

	if (input_activity) {
		blink_status_led_once();
	}

	uint8_t newly_pressed = current_keys & ~previous_keys;

	for (uint8_t i = 0; i < KEY_COUNT; i++) {
		if (newly_pressed & BIT(i)) {
			uint16_t action = dongle_config_action_for_key(i);

			if (action != ACTION_NONE) {
				rc = usb_hid_consumer_trigger_action(action);

				if (rc != 0) {
					LOG_WRN("HID action key=%u action=%u failed: %d",
						i, action, rc);
				}
			}
		}
	}

	if (report->encoder_delta != 0) {
		uint16_t action = (report->encoder_delta > 0)
			? DONGLE_ACTION_VOLUME_UP
			: DONGLE_ACTION_VOLUME_DOWN;
		int steps = (report->encoder_delta > 0)
			? report->encoder_delta
			: -report->encoder_delta;

		for (int s = 0; s < steps; s++) {
			rc = usb_hid_consumer_trigger_action(action);

			if (rc != 0) {
				LOG_WRN("HID encoder volume action=%u failed: %d", action, rc);
			}
		}
	}

	bool encoder_pressed_now = (report->encoder_pressed != 0U);

	if (!previous_encoder_pressed && encoder_pressed_now) {
		rc = usb_hid_consumer_trigger_action(DONGLE_ACTION_MUTE);

		if (rc != 0) {
			LOG_WRN("HID encoder mute action failed: %d", rc);
		}
	}

	previous_keys = current_keys;
	previous_encoder_pressed = (report->encoder_pressed != 0U);
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
	rc = usb_hid_consumer_init();
	if (rc != 0) {
		LOG_ERR("usb_hid_consumer_init failed: %d", rc);
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

#include "consumer_action_engine.h"

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "dongle_config.h"
#include "hid_consumer.h"

LOG_MODULE_REGISTER(consumer_action_engine, LOG_LEVEL_INF);

#define MACROPAD_KEY_MASK BIT_MASK(KEY_COUNT)

static uint8_t previous_keys;
static bool previous_encoder_pressed;

void consumer_action_engine_reset(void)
{
	previous_keys = 0U;
	previous_encoder_pressed = false;
}

bool consumer_action_engine_input_activity(const macropad_report_t *report)
{
	uint8_t current_keys = report->keys & MACROPAD_KEY_MASK;

	return (current_keys != previous_keys) ||
		(report->encoder_delta != 0) ||
		(previous_encoder_pressed != (report->encoder_pressed != 0U));
}

void consumer_action_engine_process_report(const macropad_report_t *report)
{
	uint8_t current_keys = report->keys & MACROPAD_KEY_MASK;
	uint8_t newly_pressed = current_keys & ~previous_keys;
	bool encoder_pressed_now = (report->encoder_pressed != 0U);
	int rc;

	for (uint8_t i = 0; i < KEY_COUNT; i++) {
		if (newly_pressed & BIT(i)) {
			uint16_t action = dongle_config_action_for_key(i);

			if (action != ACTION_NONE) {
				rc = hid_consumer_trigger_action(action);

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
			rc = hid_consumer_trigger_action(action);

			if (rc != 0) {
				LOG_WRN("HID encoder volume action=%u failed: %d", action, rc);
			}
		}
	}

	if (!previous_encoder_pressed && encoder_pressed_now) {
		rc = hid_consumer_trigger_action(DONGLE_ACTION_MUTE);

		if (rc != 0) {
			LOG_WRN("HID encoder mute action failed: %d", rc);
		}
	}

	previous_keys = current_keys;
	previous_encoder_pressed = encoder_pressed_now;
}

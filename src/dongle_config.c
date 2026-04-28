#include "dongle_config.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(dongle_config, LOG_LEVEL_INF);

#define DONGLE_CONFIG_SETTINGS_ROOT "dongle_cfg"
#define DONGLE_CONFIG_SETTINGS_KEY DONGLE_CONFIG_SETTINGS_ROOT "/map"

static const dongle_config_t default_config = {
	.action_id = {
		ACTION_NONE,
		ACTION_NONE,
		ACTION_NONE,
		ACTION_NONE,
		ACTION_NONE,
		ACTION_NONE,
	},
};

static dongle_config_t stored_config;
static bool stored_config_loaded;

static int dongle_config_settings_set(const char *name, size_t len_rd,
				      settings_read_cb read_cb, void *cb_arg)
{
	int rc;

	if (!settings_name_steq(name, "map", NULL)) {
		return -ENOENT;
	}

	if (len_rd != sizeof(stored_config)) {
		LOG_WRN("Stored config length %zu does not match expected %zu",
			len_rd, sizeof(stored_config));
		stored_config_loaded = false;
		return 0;
	}

	rc = read_cb(cb_arg, &stored_config, sizeof(stored_config));
	if (rc < 0) {
		return rc;
	}

	if (rc != sizeof(stored_config)) {
		LOG_WRN("Settings read returned %d bytes for config", rc);
		stored_config_loaded = false;
		return 0;
	}

	stored_config_loaded = true;
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(dongle_config, DONGLE_CONFIG_SETTINGS_ROOT, NULL,
			      dongle_config_settings_set, NULL, NULL);

int dongle_config_init(void)
{
	int rc;

	stored_config = default_config;
	stored_config_loaded = false;

	rc = settings_load_subtree(DONGLE_CONFIG_SETTINGS_ROOT);
	if (rc != 0) {
		LOG_WRN("settings_load_subtree failed: %d, using defaults", rc);
		return 0;
	}

	if (stored_config_loaded) {
		LOG_INF("Loaded persisted key-action mapping");
		return 0;
	}

	rc = settings_save_one(DONGLE_CONFIG_SETTINGS_KEY,
			       &default_config, sizeof(default_config));
	if (rc != 0) {
		LOG_WRN("settings_save_one failed: %d, continuing with defaults", rc);
		return 0;
	}

	stored_config = default_config;
	stored_config_loaded = true;
	LOG_INF("Stored default key-action mapping");
	return 0;
}

const dongle_config_t *dongle_config_get(void)
{
	return &stored_config;
}

uint16_t dongle_config_action_for_key(uint8_t key_index)
{
	if (key_index >= KEY_COUNT) {
		return ACTION_NONE;
	}

	return stored_config.action_id[key_index];
}
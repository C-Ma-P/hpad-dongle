#ifndef HPADV2_DONGLE_CONFIG_H_
#define HPADV2_DONGLE_CONFIG_H_

#include <stdint.h>

#include <zephyr/toolchain.h>

#define KEY_COUNT 6U
#define ACTION_NONE 0U

enum dongle_action_id {
	DONGLE_ACTION_VOLUME_UP = 1,
	DONGLE_ACTION_VOLUME_DOWN = 2,
	DONGLE_ACTION_MUTE = 3,
	DONGLE_ACTION_NEXT_TRACK = 4,
	DONGLE_ACTION_PREVIOUS_TRACK = 5,
	DONGLE_ACTION_STOP = 6,
	DONGLE_ACTION_PLAY_PAUSE = 7,
};

typedef struct __packed {
	uint16_t action_id[KEY_COUNT];
} dongle_config_t;

int dongle_config_init(void);
const dongle_config_t *dongle_config_get(void);
uint16_t dongle_config_action_for_key(uint8_t key_index);

#endif
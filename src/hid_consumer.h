#ifndef HPADV2_HID_CONSUMER_H_
#define HPADV2_HID_CONSUMER_H_

#include <stdint.h>

int hid_consumer_init(void);
int hid_consumer_trigger_action(uint16_t action_id);

#endif
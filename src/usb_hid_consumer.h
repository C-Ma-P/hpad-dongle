#ifndef HPADV2_USB_HID_CONSUMER_H_
#define HPADV2_USB_HID_CONSUMER_H_

#include <stdint.h>

int usb_hid_consumer_init(void);
int usb_hid_consumer_trigger_action(uint16_t action_id);

#endif
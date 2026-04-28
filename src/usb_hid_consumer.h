#ifndef HPADV2_USB_HID_CONSUMER_H_
#define HPADV2_USB_HID_CONSUMER_H_

#include <stdint.h>

#include "wire_protocol.h"

int usb_hid_consumer_init(void);
int usb_hid_consumer_trigger_action(uint16_t action_id);
int usb_hid_consumer_forward_macropad_report(const host_macropad_report_t *report);

#endif
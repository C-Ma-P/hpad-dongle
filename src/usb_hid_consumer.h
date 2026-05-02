#ifndef HPADV2_USB_HID_CONSUMER_H_
#define HPADV2_USB_HID_CONSUMER_H_

#include <stdint.h>

#include "wire_protocol.h"

typedef void (*usb_hid_vendor_config_handler_t)(const macropad_config_t *config);

int usb_hid_consumer_init(void);
void usb_hid_consumer_set_config_handler(usb_hid_vendor_config_handler_t handler);
int usb_hid_consumer_trigger_action(uint16_t action_id);
int usb_hid_consumer_forward_macropad_report(const host_macropad_report_t *report);

#endif
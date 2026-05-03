#ifndef HPADV2_HID_VENDOR_H_
#define HPADV2_HID_VENDOR_H_

#include "wire_protocol.h"

typedef void (*hid_vendor_config_handler_t)(const macropad_config_t *config);

int hid_vendor_init(void);
void hid_vendor_set_config_handler(hid_vendor_config_handler_t handler);
int hid_vendor_forward_macropad_report(const host_macropad_report_t *report);

#endif
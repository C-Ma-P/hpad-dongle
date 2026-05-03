#ifndef HPADV2_USB_DEVICE_H_
#define HPADV2_USB_DEVICE_H_

#include <stdbool.h>

#include <zephyr/kernel.h>

int usb_device_init(void);
bool usb_device_is_initialized(void);
void usb_device_submit_work(struct k_work *work);

#endif
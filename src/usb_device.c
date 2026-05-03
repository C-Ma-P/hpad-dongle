#include "usb_device.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>

#include "hid_consumer.h"
#include "hid_vendor.h"

LOG_MODULE_REGISTER(usb_device, LOG_LEVEL_INF);

#define DONGLE_USB_VID 0xCAFE
#define DONGLE_USB_PID 0xB00B
#define USB_HID_WORKQ_STACK_SIZE 1536
#define USB_HID_WORKQ_PRIORITY 5

USBD_DEVICE_DEFINE(dongle_usbd,
		   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   DONGLE_USB_VID, DONGLE_USB_PID);

USBD_DESC_LANG_DEFINE(dongle_lang);
USBD_DESC_MANUFACTURER_DEFINE(dongle_mfr, "HPad");
USBD_DESC_PRODUCT_DEFINE(dongle_product, "Dongle");
USBD_DESC_CONFIG_DEFINE(dongle_fs_cfg_desc, "FS Configuration");
USBD_DESC_CONFIG_DEFINE(dongle_hs_cfg_desc, "HS Configuration");

USBD_CONFIGURATION_DEFINE(dongle_fs_config, 0, 100, &dongle_fs_cfg_desc);
USBD_CONFIGURATION_DEFINE(dongle_hs_config, 0, 100, &dongle_hs_cfg_desc);

static struct k_work_q usb_hid_workq;
K_KERNEL_STACK_DEFINE(usb_hid_workq_stack, USB_HID_WORKQ_STACK_SIZE);

static bool usb_initialized;
static bool usb_hid_workq_started;

static void usb_device_msg_cb(struct usbd_context *const ctx,
			      const struct usbd_msg *const msg)
{
	LOG_DBG("USBD message: %s", usbd_msg_type_string(msg->type));

	if (!usbd_can_detect_vbus(ctx)) {
		return;
	}

	if (msg->type == USBD_MSG_VBUS_READY) {
		if (usbd_enable(ctx) != 0) {
			LOG_ERR("Failed to enable USB device support");
		}
	} else if (msg->type == USBD_MSG_VBUS_REMOVED) {
		if (usbd_disable(ctx) != 0) {
			LOG_ERR("Failed to disable USB device support");
		}
	}
}

static int usb_device_register_speed(struct usbd_context *ctx, enum usbd_speed speed)
{
	struct usbd_config_node *cfg;
	int rc;

	cfg = (speed == USBD_SPEED_HS) ? &dongle_hs_config : &dongle_fs_config;

	rc = usbd_add_configuration(ctx, speed, cfg);
	if (rc != 0) {
		LOG_ERR("Failed to add USB configuration: %d", rc);
		return rc;
	}

	rc = usbd_register_class(ctx, "hid_0", speed, 1);
	if (rc != 0) {
		LOG_ERR("Failed to register HID class: %d", rc);
		return rc;
	}

	rc = usbd_register_class(ctx, "hid_1", speed, 1);
	if (rc != 0) {
		LOG_ERR("Failed to register vendor HID class: %d", rc);
		return rc;
	}

	rc = usbd_register_class(ctx, "cdc_acm_0", speed, 1);
	if (rc != 0) {
		LOG_ERR("Failed to register CDC ACM class: %d", rc);
		return rc;
	}

	return usbd_device_set_code_triple(ctx, speed,
					  USB_BCC_MISCELLANEOUS, 0x02, 0x01);
}

int usb_device_init(void)
{
	struct usbd_context *ctx = &dongle_usbd;
	int rc;

	if (usb_initialized) {
		return 0;
	}

	if (!usb_hid_workq_started) {
		k_work_queue_start(&usb_hid_workq,
				   usb_hid_workq_stack,
				   K_KERNEL_STACK_SIZEOF(usb_hid_workq_stack),
				   USB_HID_WORKQ_PRIORITY,
				   NULL);
		k_thread_name_set(&usb_hid_workq.thread, "usb_hid_workq");
		usb_hid_workq_started = true;
	}

	rc = hid_consumer_init();
	if (rc != 0) {
		return rc;
	}

	rc = hid_vendor_init();
	if (rc != 0) {
		return rc;
	}

	rc = usbd_add_descriptor(ctx, &dongle_lang);
	if (rc != 0) {
		LOG_ERR("Failed to add USB language descriptor: %d", rc);
		return rc;
	}

	rc = usbd_add_descriptor(ctx, &dongle_mfr);
	if (rc != 0) {
		LOG_ERR("Failed to add USB manufacturer descriptor: %d", rc);
		return rc;
	}

	rc = usbd_add_descriptor(ctx, &dongle_product);
	if (rc != 0) {
		LOG_ERR("Failed to add USB product descriptor: %d", rc);
		return rc;
	}

	rc = usbd_msg_register_cb(ctx, usb_device_msg_cb);
	if (rc != 0) {
		LOG_ERR("Failed to register USB message callback: %d", rc);
		return rc;
	}

	if (USBD_SUPPORTS_HIGH_SPEED && usbd_caps_speed(ctx) == USBD_SPEED_HS) {
		rc = usb_device_register_speed(ctx, USBD_SPEED_HS);
		if (rc != 0) {
			return rc;
		}
	}

	rc = usb_device_register_speed(ctx, USBD_SPEED_FS);
	if (rc != 0) {
		return rc;
	}

	rc = usbd_init(ctx);
	if (rc != 0) {
		LOG_ERR("usbd_init failed: %d", rc);
		return rc;
	}

	if (!usbd_can_detect_vbus(ctx)) {
		rc = usbd_enable(ctx);
		if (rc != 0) {
			LOG_ERR("usbd_enable failed: %d", rc);
			return rc;
		}
	}

	usb_initialized = true;
	LOG_INF("USB consumer control + vendor HID initialized");
	return 0;
}

bool usb_device_is_initialized(void)
{
	return usb_initialized;
}

void usb_device_submit_work(struct k_work *work)
{
	if (usb_hid_workq_started) {
		(void)k_work_submit_to_queue(&usb_hid_workq, work);
	}
}
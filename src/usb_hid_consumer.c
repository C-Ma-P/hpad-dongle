#include "usb_hid_consumer.h"

#include <errno.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/class/hid.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/usb/usbd.h>

#include "dongle_config.h"

LOG_MODULE_REGISTER(usb_hid_consumer, LOG_LEVEL_INF);

#define DONGLE_USB_VID 0xCAFE
#define DONGLE_USB_PID 0xB00B
#define USB_CONSUMER_USAGE_PAGE 0x0CU
#define USB_CONSUMER_USAGE_CONSUMER_CONTROL 0x01U
#define USB_CONSUMER_USAGE_MUTE 0x00E2U
#define USB_CONSUMER_USAGE_VOLUME_INCREMENT 0x00E9U
#define USB_CONSUMER_USAGE_VOLUME_DECREMENT 0x00EAU
#define USB_CONSUMER_USAGE_PLAY_PAUSE 0x00CDU
#define USB_CONSUMER_USAGE_SCAN_NEXT_TRACK 0x00B5U
#define USB_CONSUMER_USAGE_SCAN_PREVIOUS_TRACK 0x00B6U
#define USB_CONSUMER_USAGE_STOP 0x00B7U

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

static const struct device *const hid_dev = DEVICE_DT_GET_ONE(zephyr_hid_device);

static const uint8_t consumer_hid_report_desc[] = {
	HID_USAGE_PAGE(USB_CONSUMER_USAGE_PAGE),
	HID_USAGE(USB_CONSUMER_USAGE_CONSUMER_CONTROL),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
	HID_LOGICAL_MIN8(0x00),
	HID_LOGICAL_MAX16(0xFF, 0x03),
	HID_USAGE_MIN8(0x00),
	HID_USAGE_MAX16(0xFF, 0x03),
	HID_REPORT_SIZE(16),
	HID_REPORT_COUNT(1),
	HID_INPUT(0x00),
	HID_END_COLLECTION,
};

UDC_STATIC_BUF_DEFINE(consumer_press_report, sizeof(uint16_t));
UDC_STATIC_BUF_DEFINE(consumer_release_report, sizeof(uint16_t));

static struct usbd_context *dongle_usbd_ctx;
static bool hid_ready;
static bool usb_initialized;

static uint16_t usb_hid_consumer_usage_for_action(uint16_t action_id)
{
	switch (action_id) {
	case DONGLE_ACTION_VOLUME_UP:
		return USB_CONSUMER_USAGE_VOLUME_INCREMENT;
	case DONGLE_ACTION_VOLUME_DOWN:
		return USB_CONSUMER_USAGE_VOLUME_DECREMENT;
	case DONGLE_ACTION_MUTE:
		return USB_CONSUMER_USAGE_MUTE;
	case DONGLE_ACTION_NEXT_TRACK:
		return USB_CONSUMER_USAGE_SCAN_NEXT_TRACK;
	case DONGLE_ACTION_PREVIOUS_TRACK:
		return USB_CONSUMER_USAGE_SCAN_PREVIOUS_TRACK;
	case DONGLE_ACTION_STOP:
		return USB_CONSUMER_USAGE_STOP;
	case DONGLE_ACTION_PLAY_PAUSE:
		return USB_CONSUMER_USAGE_PLAY_PAUSE;
	default:
		return 0U;
	}
}

static void usb_hid_consumer_iface_ready(const struct device *dev, const bool ready)
{
	ARG_UNUSED(dev);

	hid_ready = ready;
	LOG_INF("Consumer control HID interface %s", ready ? "ready" : "not ready");
}

static int usb_hid_consumer_get_report(const struct device *dev,
					const uint8_t type, const uint8_t id,
					const uint16_t len, uint8_t *const buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(id);

	if (type != HID_REPORT_TYPE_INPUT) {
		return -ENOTSUP;
	}

	if (len < sizeof(uint16_t)) {
		return -EINVAL;
	}

	/* Return idle report: no key pressed */
	sys_put_le16(0U, buf);
	return sizeof(uint16_t);
}

static const struct hid_device_ops consumer_hid_ops = {
	.iface_ready = usb_hid_consumer_iface_ready,
	.get_report = usb_hid_consumer_get_report,
};

static void usb_hid_consumer_msg_cb(struct usbd_context *const ctx,
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

static int usb_hid_consumer_register_speed(struct usbd_context *ctx, enum usbd_speed speed)
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

	rc = usbd_register_class(ctx, "cdc_acm_0", speed, 1);
	if (rc != 0) {
		LOG_ERR("Failed to register CDC ACM class: %d", rc);
		return rc;
	}

	return usbd_device_set_code_triple(ctx, speed,
					  USB_BCC_MISCELLANEOUS, 0x02, 0x01);
}

int usb_hid_consumer_init(void)
{
	int rc;

	if (usb_initialized) {
		return 0;
	}

	if (!device_is_ready(hid_dev)) {
		return -ENODEV;
	}

	rc = hid_device_register(hid_dev, consumer_hid_report_desc,
				 sizeof(consumer_hid_report_desc), &consumer_hid_ops);
	if (rc != 0) {
		LOG_ERR("hid_device_register failed: %d", rc);
		return rc;
	}

	rc = usbd_add_descriptor(&dongle_usbd, &dongle_lang);
	if (rc != 0) {
		LOG_ERR("Failed to add USB language descriptor: %d", rc);
		return rc;
	}

	rc = usbd_add_descriptor(&dongle_usbd, &dongle_mfr);
	if (rc != 0) {
		LOG_ERR("Failed to add USB manufacturer descriptor: %d", rc);
		return rc;
	}

	rc = usbd_add_descriptor(&dongle_usbd, &dongle_product);
	if (rc != 0) {
		LOG_ERR("Failed to add USB product descriptor: %d", rc);
		return rc;
	}

	rc = usbd_msg_register_cb(&dongle_usbd, usb_hid_consumer_msg_cb);
	if (rc != 0) {
		LOG_ERR("Failed to register USB message callback: %d", rc);
		return rc;
	}

	if (USBD_SUPPORTS_HIGH_SPEED && usbd_caps_speed(&dongle_usbd) == USBD_SPEED_HS) {
		rc = usb_hid_consumer_register_speed(&dongle_usbd, USBD_SPEED_HS);
		if (rc != 0) {
			return rc;
		}
	}

	rc = usb_hid_consumer_register_speed(&dongle_usbd, USBD_SPEED_FS);
	if (rc != 0) {
		return rc;
	}

	rc = usbd_init(&dongle_usbd);
	if (rc != 0) {
		LOG_ERR("usbd_init failed: %d", rc);
		return rc;
	}

	dongle_usbd_ctx = &dongle_usbd;
	if (!usbd_can_detect_vbus(dongle_usbd_ctx)) {
		rc = usbd_enable(dongle_usbd_ctx);
		if (rc != 0) {
			LOG_ERR("usbd_enable failed: %d", rc);
			return rc;
		}
	}

	usb_initialized = true;
	LOG_INF("USB consumer control HID initialized");
	return 0;
}

int usb_hid_consumer_trigger_action(uint16_t action_id)
{
	uint16_t usage_id;
	int rc;

	if (!usb_initialized) {
		return -EAGAIN;
	}

	if (!hid_ready) {
		return -EAGAIN;
	}

	usage_id = usb_hid_consumer_usage_for_action(action_id);
	if (usage_id == 0U) {
		return -ENOTSUP;
	}

	sys_put_le16(usage_id, consumer_press_report);
	sys_put_le16(0U, consumer_release_report);

	rc = hid_device_submit_report(hid_dev, sizeof(uint16_t), consumer_press_report);
	if (rc != 0) {
		return rc;
	}

	return hid_device_submit_report(hid_dev, sizeof(uint16_t), consumer_release_report);
}
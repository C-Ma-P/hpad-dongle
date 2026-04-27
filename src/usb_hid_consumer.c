#include "usb_hid_consumer.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/class/hid.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/usb/usbd.h>

#include "dongle_config.h"
#include "wire_protocol.h"

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
#define USB_VENDOR_USAGE_PAGE 0xFF00U
#define USB_VENDOR_USAGE 0xFF01U
#define USB_HID_WORKQ_STACK_SIZE 1536
#define USB_HID_WORKQ_PRIORITY 5
#define USB_HID_CONSUMER_QUEUE_LEN 32
#define USB_HID_VENDOR_QUEUE_LEN 32

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

static const struct device *const consumer_hid_dev = DEVICE_DT_GET(DT_NODELABEL(hid_0));
static const struct device *const vendor_hid_dev = DEVICE_DT_GET(DT_NODELABEL(hid_1));

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

static const uint8_t vendor_hid_report_desc[] = {
	HID_ITEM(HID_ITEM_TAG_USAGE_PAGE, HID_ITEM_TYPE_GLOBAL, 2),
	(uint8_t)USB_VENDOR_USAGE_PAGE,
	(uint8_t)(USB_VENDOR_USAGE_PAGE >> 8),
	HID_USAGE16(USB_VENDOR_USAGE),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
	HID_LOGICAL_MIN8(0x00),
	HID_LOGICAL_MAX16(0xFF, 0x00),
	HID_REPORT_SIZE(8),
	HID_REPORT_COUNT(sizeof(macropad_report_t)),
	HID_INPUT(0x02),
	HID_END_COLLECTION,
};

UDC_STATIC_BUF_DEFINE(consumer_hid_buf, sizeof(uint16_t));
UDC_STATIC_BUF_DEFINE(vendor_hid_buf, sizeof(macropad_report_t));
K_MSGQ_DEFINE(consumer_report_queue, sizeof(uint16_t), USB_HID_CONSUMER_QUEUE_LEN, 4);
K_KERNEL_STACK_DEFINE(usb_hid_workq_stack, USB_HID_WORKQ_STACK_SIZE);

static struct usbd_context *dongle_usbd_ctx;
static struct k_work_q usb_hid_workq;
static struct k_work consumer_hid_work;
static struct k_work vendor_hid_work;
static bool consumer_hid_ready;
static bool vendor_hid_ready;
static bool usb_initialized;
static bool usb_hid_workq_started;
static atomic_t consumer_submit_pending;
static atomic_t vendor_submit_pending;

/* Vendor HID latest-report slot: new reports overwrite the slot so the queue
 * never fills up when no host reader has the hidraw device open. */
static struct k_spinlock vendor_slot_lock;
static macropad_report_t vendor_slot_report;
static bool vendor_slot_dirty;

static void usb_hid_consumer_work_handler(struct k_work *work);
static void usb_hid_vendor_work_handler(struct k_work *work);

static void usb_hid_submit_consumer_work(void)
{
	if (usb_hid_workq_started) {
		(void)k_work_submit_to_queue(&usb_hid_workq, &consumer_hid_work);
	}
}

static void usb_hid_submit_vendor_work(void)
{
	if (usb_hid_workq_started) {
		(void)k_work_submit_to_queue(&usb_hid_workq, &vendor_hid_work);
	}
}

static void usb_consumer_input_report_done(const struct device *dev,
					   const uint8_t *const buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(buf);
	atomic_set(&consumer_submit_pending, 0);
	usb_hid_submit_consumer_work();
}

static void usb_vendor_input_report_done(const struct device *dev,
					 const uint8_t *const buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(buf);
	/* Clear in-flight flag; if a newer report arrived while this one was
	 * in-flight, vendor_slot_dirty will be set and the work handler sends it. */
	atomic_set(&vendor_submit_pending, 0);
	usb_hid_submit_vendor_work();
}

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

	consumer_hid_ready = ready;
	LOG_INF("Consumer control HID interface %s", ready ? "ready" : "not ready");
}

static void usb_hid_vendor_iface_ready(const struct device *dev, const bool ready)
{
	ARG_UNUSED(dev);

	vendor_hid_ready = ready;
	LOG_INF("Vendor HID interface %s", ready ? "ready" : "not ready");
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

static int usb_hid_vendor_get_report(const struct device *dev,
				  const uint8_t type, const uint8_t id,
				  const uint16_t len, uint8_t *const buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(id);

	if (type != HID_REPORT_TYPE_INPUT) {
		return -ENOTSUP;
	}

	if (len < sizeof(macropad_report_t)) {
		return -EINVAL;
	}

	memset(buf, 0, sizeof(macropad_report_t));
	return sizeof(macropad_report_t);
}

static const struct hid_device_ops consumer_hid_ops = {
	.iface_ready = usb_hid_consumer_iface_ready,
	.get_report = usb_hid_consumer_get_report,
	.input_report_done = usb_consumer_input_report_done,
};

static const struct hid_device_ops vendor_hid_ops = {
	.iface_ready = usb_hid_vendor_iface_ready,
	.get_report = usb_hid_vendor_get_report,
	.input_report_done = usb_vendor_input_report_done,
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

static void usb_hid_consumer_work_handler(struct k_work *work)
{
	uint16_t report;
	int rc;

	ARG_UNUSED(work);

	if (!usb_initialized || !consumer_hid_ready) {
		return;
	}

	/* One in-flight submit at a time; completion callback re-triggers this work. */
	if (!atomic_cas(&consumer_submit_pending, 0, 1)) {
		return;
	}

	if (k_msgq_get(&consumer_report_queue, &report, K_NO_WAIT) != 0) {
		atomic_set(&consumer_submit_pending, 0);
		return;
	}

	sys_put_le16(report, consumer_hid_buf);
	rc = hid_device_submit_report(consumer_hid_dev, sizeof(uint16_t), consumer_hid_buf);
	if (rc != 0) {
		LOG_WRN("Consumer HID submit failed: %d", rc);
		atomic_set(&consumer_submit_pending, 0);
	}
}

static void usb_hid_vendor_work_handler(struct k_work *work)
{
	macropad_report_t report;
	k_spinlock_key_t key;
	bool has_data;
	int rc;

	ARG_UNUSED(work);

	if (!usb_initialized || !vendor_hid_ready) {
		return;
	}

	/* One in-flight submit at a time; input_report_done re-triggers this work. */
	if (!atomic_cas(&vendor_submit_pending, 0, 1)) {
		return;
	}

	key = k_spin_lock(&vendor_slot_lock);
	has_data = vendor_slot_dirty;
	if (has_data) {
		report = vendor_slot_report;
		vendor_slot_dirty = false;
	}
	k_spin_unlock(&vendor_slot_lock, key);

	if (!has_data) {
		atomic_set(&vendor_submit_pending, 0);
		return;
	}

	memcpy(vendor_hid_buf, &report, sizeof(report));
	rc = hid_device_submit_report(vendor_hid_dev, sizeof(report), vendor_hid_buf);
	if (rc != 0) {
		LOG_DBG("Vendor HID submit failed: %d", rc);
		atomic_set(&vendor_submit_pending, 0);
	}
}

int usb_hid_consumer_init(void)
{
	int rc;

	if (usb_initialized) {
		return 0;
	}

	if (!device_is_ready(consumer_hid_dev) || !device_is_ready(vendor_hid_dev)) {
		return -ENODEV;
	}

	if (!usb_hid_workq_started) {
		k_work_queue_start(&usb_hid_workq,
				   usb_hid_workq_stack,
				   K_KERNEL_STACK_SIZEOF(usb_hid_workq_stack),
				   USB_HID_WORKQ_PRIORITY,
				   NULL);
		k_work_init(&consumer_hid_work, usb_hid_consumer_work_handler);
		k_work_init(&vendor_hid_work, usb_hid_vendor_work_handler);
		k_thread_name_set(&usb_hid_workq.thread, "usb_hid_workq");
		usb_hid_workq_started = true;
	}

	rc = hid_device_register(consumer_hid_dev, consumer_hid_report_desc,
				 sizeof(consumer_hid_report_desc), &consumer_hid_ops);
	if (rc != 0) {
		LOG_ERR("consumer hid_device_register failed: %d", rc);
		return rc;
	}

	rc = hid_device_register(vendor_hid_dev, vendor_hid_report_desc,
				 sizeof(vendor_hid_report_desc), &vendor_hid_ops);
	if (rc != 0) {
		LOG_ERR("vendor hid_device_register failed: %d", rc);
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
	LOG_INF("USB consumer control + vendor HID initialized");
	return 0;
}

int usb_hid_consumer_trigger_action(uint16_t action_id)
{
	uint16_t usage_id;

	if (!usb_initialized) {
		return -EAGAIN;
	}

	if (!consumer_hid_ready) {
		return -EAGAIN;
	}

	usage_id = usb_hid_consumer_usage_for_action(action_id);
	if (usage_id == 0U) {
		return -ENOTSUP;
	}

	if (k_msgq_put(&consumer_report_queue, &usage_id, K_NO_WAIT) != 0) {
		return -ENOBUFS;
	}

	usage_id = 0U;
	if (k_msgq_put(&consumer_report_queue, &usage_id, K_NO_WAIT) != 0) {
		return -ENOBUFS;
	}

	usb_hid_submit_consumer_work();
	return 0;
}

int usb_hid_consumer_forward_macropad_report(const macropad_report_t *report)
{
	k_spinlock_key_t key;

	if (report == NULL) {
		return -EINVAL;
	}

	if (!usb_initialized || !vendor_hid_ready) {
		return 0;
	}

	/* Always overwrite the slot with the latest report. If a transfer is
	 * already in flight, input_report_done will pick this up when it fires.
	 * When no host reader has the device open, vendor_submit_pending stays 1
	 * and the slot is merely overwritten each call — no queue fill-up. */
	key = k_spin_lock(&vendor_slot_lock);
	vendor_slot_report = *report;
	vendor_slot_dirty = true;
	k_spin_unlock(&vendor_slot_lock, key);

	/* Only kick work if no transfer is already in flight. */
	if (atomic_cas(&vendor_submit_pending, 0, 1)) {
		usb_hid_submit_vendor_work();
	}

	return 0;
}
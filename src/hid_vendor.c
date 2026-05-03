#include "hid_vendor.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/usb/udc_buf.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>
#include <zephyr/usb/class/hid.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/usb/usbd.h>

#include "usb_device.h"

LOG_MODULE_REGISTER(hid_vendor, LOG_LEVEL_INF);

#define USB_VENDOR_USAGE_PAGE 0xFF00U
#define USB_VENDOR_USAGE 0xFF01U

static const struct device *const vendor_hid_dev = DEVICE_DT_GET(DT_NODELABEL(hid_1));

static const uint8_t vendor_hid_report_desc[] = {
	HID_ITEM(HID_ITEM_TAG_USAGE_PAGE, HID_ITEM_TYPE_GLOBAL, 2),
	(uint8_t)USB_VENDOR_USAGE_PAGE,
	(uint8_t)(USB_VENDOR_USAGE_PAGE >> 8),
	HID_USAGE16(USB_VENDOR_USAGE),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
	HID_LOGICAL_MIN8(0x00),
	HID_LOGICAL_MAX16(0xFF, 0x00),
	HID_REPORT_SIZE(8),
	HID_REPORT_COUNT(HPAD_USB_VENDOR_INPUT_REPORT_SIZE),
	HID_INPUT(0x02),
	HID_REPORT_COUNT(HPAD_USB_VENDOR_OUTPUT_REPORT_SIZE),
	HID_OUTPUT(0x02),
	HID_END_COLLECTION,
};

UDC_STATIC_BUF_DEFINE(vendor_hid_buf, HPAD_USB_VENDOR_INPUT_REPORT_SIZE);

static struct k_work vendor_hid_work;
static bool vendor_hid_ready;
static atomic_t vendor_submit_pending;
static hid_vendor_config_handler_t vendor_config_handler;

static struct k_spinlock vendor_slot_lock;
static host_macropad_report_t vendor_slot_report;
static bool vendor_slot_dirty;

static void hid_vendor_submit_work(void)
{
	usb_device_submit_work(&vendor_hid_work);
}

static void hid_vendor_input_report_done(const struct device *dev,
					 const uint8_t *const buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(buf);
	atomic_set(&vendor_submit_pending, 0);
	hid_vendor_submit_work();
}

static void hid_vendor_iface_ready(const struct device *dev, const bool ready)
{
	ARG_UNUSED(dev);

	vendor_hid_ready = ready;
	LOG_INF("Vendor HID interface %s", ready ? "ready" : "not ready");
	if (ready) {
		hid_vendor_submit_work();
	}
}

static int hid_vendor_get_report(const struct device *dev,
				 const uint8_t type, const uint8_t id,
				 const uint16_t len, uint8_t *const buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(id);

	if (type != HID_REPORT_TYPE_INPUT) {
		return -ENOTSUP;
	}

	if (len < HPAD_USB_VENDOR_INPUT_REPORT_SIZE) {
		return -EINVAL;
	}

	memset(buf, 0, HPAD_USB_VENDOR_INPUT_REPORT_SIZE);
	return HPAD_USB_VENDOR_INPUT_REPORT_SIZE;
}

static int hid_vendor_set_report(const struct device *dev,
				 const uint8_t type, const uint8_t id,
				 const uint16_t len, const uint8_t *const buf)
{
	const uint8_t *payload = buf;
	uint16_t payload_len = len;
	macropad_config_t config;

	ARG_UNUSED(dev);
	ARG_UNUSED(id);

	if (type != HID_REPORT_TYPE_OUTPUT) {
		return -ENOTSUP;
	}

	if ((payload_len == (HPAD_USB_VENDOR_OUTPUT_REPORT_SIZE + 1U)) &&
	    (payload[0] == 0U)) {
		payload++;
		payload_len--;
	}

	if (payload_len != sizeof(config)) {
		return -EINVAL;
	}

	memcpy(&config, payload, sizeof(config));
	if (config.kind != HPAD_PROTOCOL_CONFIG_KIND_KEY_COLORS) {
		return -EINVAL;
	}
	if (vendor_config_handler != NULL) {
		vendor_config_handler(&config);
	}

	return 0;
}

static const struct hid_device_ops vendor_hid_ops = {
	.iface_ready = hid_vendor_iface_ready,
	.get_report = hid_vendor_get_report,
	.set_report = hid_vendor_set_report,
	.input_report_done = hid_vendor_input_report_done,
};

static void hid_vendor_work_handler(struct k_work *work)
{
	host_macropad_report_t report;
	k_spinlock_key_t key;
	bool has_data;
	int rc;

	ARG_UNUSED(work);

	if (!usb_device_is_initialized() || !vendor_hid_ready) {
		return;
	}

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

int hid_vendor_init(void)
{
	int rc;

	if (!device_is_ready(vendor_hid_dev)) {
		return -ENODEV;
	}

	k_work_init(&vendor_hid_work, hid_vendor_work_handler);
	rc = hid_device_register(vendor_hid_dev, vendor_hid_report_desc,
				 sizeof(vendor_hid_report_desc), &vendor_hid_ops);
	if (rc != 0) {
		LOG_ERR("vendor hid_device_register failed: %d", rc);
	}

	return rc;
}

void hid_vendor_set_config_handler(hid_vendor_config_handler_t handler)
{
	vendor_config_handler = handler;
}

int hid_vendor_forward_macropad_report(const host_macropad_report_t *report)
{
	k_spinlock_key_t key;

	if (report == NULL) {
		return -EINVAL;
	}

	if (!usb_device_is_initialized()) {
		return 0;
	}

	key = k_spin_lock(&vendor_slot_lock);
	vendor_slot_report = *report;
	vendor_slot_dirty = true;
	k_spin_unlock(&vendor_slot_lock, key);

	hid_vendor_submit_work();
	return 0;
}
#include "hid_consumer.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/usb/udc_buf.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/class/hid.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/usb/usbd.h>

#include "dongle_config.h"
#include "usb_device.h"

LOG_MODULE_REGISTER(hid_consumer, LOG_LEVEL_INF);

#define USB_CONSUMER_USAGE_PAGE 0x0CU
#define USB_CONSUMER_USAGE_CONSUMER_CONTROL 0x01U
#define USB_CONSUMER_USAGE_MUTE 0x00E2U
#define USB_CONSUMER_USAGE_VOLUME_INCREMENT 0x00E9U
#define USB_CONSUMER_USAGE_VOLUME_DECREMENT 0x00EAU
#define USB_CONSUMER_USAGE_PLAY_PAUSE 0x00CDU
#define USB_CONSUMER_USAGE_SCAN_NEXT_TRACK 0x00B5U
#define USB_CONSUMER_USAGE_SCAN_PREVIOUS_TRACK 0x00B6U
#define USB_CONSUMER_USAGE_STOP 0x00B7U
#define USB_HID_CONSUMER_QUEUE_LEN 32

static const struct device *const consumer_hid_dev = DEVICE_DT_GET(DT_NODELABEL(hid_0));

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

UDC_STATIC_BUF_DEFINE(consumer_hid_buf, sizeof(uint16_t));
K_MSGQ_DEFINE(consumer_report_queue, sizeof(uint16_t), USB_HID_CONSUMER_QUEUE_LEN, 4);

static struct k_work consumer_hid_work;
static bool consumer_hid_ready;
static atomic_t consumer_submit_pending;

static void hid_consumer_work_handler(struct k_work *work)
{
	uint16_t report;
	int rc;

	ARG_UNUSED(work);

	if (!usb_device_is_initialized() || !consumer_hid_ready) {
		return;
	}

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

static void hid_consumer_submit_work(void)
{
	usb_device_submit_work(&consumer_hid_work);
}

static void hid_consumer_input_report_done(const struct device *dev,
					   const uint8_t *const buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(buf);
	atomic_set(&consumer_submit_pending, 0);
	hid_consumer_submit_work();
}

static uint16_t hid_consumer_usage_for_action(uint16_t action_id)
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

static void hid_consumer_iface_ready(const struct device *dev, const bool ready)
{
	ARG_UNUSED(dev);

	consumer_hid_ready = ready;
	LOG_INF("Consumer control HID interface %s", ready ? "ready" : "not ready");
}

static int hid_consumer_get_report(const struct device *dev,
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

	sys_put_le16(0U, buf);
	return sizeof(uint16_t);
}

static const struct hid_device_ops consumer_hid_ops = {
	.iface_ready = hid_consumer_iface_ready,
	.get_report = hid_consumer_get_report,
	.input_report_done = hid_consumer_input_report_done,
};

int hid_consumer_init(void)
{
	int rc;

	if (!device_is_ready(consumer_hid_dev)) {
		return -ENODEV;
	}

	k_work_init(&consumer_hid_work, hid_consumer_work_handler);
	rc = hid_device_register(consumer_hid_dev, consumer_hid_report_desc,
				 sizeof(consumer_hid_report_desc), &consumer_hid_ops);
	if (rc != 0) {
		LOG_ERR("consumer hid_device_register failed: %d", rc);
	}

	return rc;
}

int hid_consumer_trigger_action(uint16_t action_id)
{
	uint16_t usage_id;

	if (!usb_device_is_initialized()) {
		return -EAGAIN;
	}

	if (!consumer_hid_ready) {
		return -EAGAIN;
	}

	usage_id = hid_consumer_usage_for_action(action_id);
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

	hid_consumer_submit_work();
	return 0;
}
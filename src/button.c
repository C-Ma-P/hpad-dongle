#include "button.h"

#include <errno.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(button, LOG_LEVEL_INF);

#define BUTTON_NODE DT_ALIAS(sw0)
#define BUTTON_DEBOUNCE_MS 25

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static struct gpio_callback button_cb_data;
static struct k_work_delayable button_debounce_work;
static button_press_callback_t button_press_callback;
static bool button_latched;

static void button_debounce_work_handler(struct k_work *work)
{
	int state;

	ARG_UNUSED(work);

	state = gpio_pin_get_dt(&button);
	if (state < 0) {
		LOG_ERR("Failed to sample button state: %d", state);
		return;
	}

	if (state == 0) {
		if (!button_latched) {
			button_latched = true;
			LOG_INF("Button press detected on P0.09");
			if (button_press_callback != NULL) {
				button_press_callback();
			}
		}

		(void)k_work_reschedule(&button_debounce_work, K_MSEC(BUTTON_DEBOUNCE_MS));
		return;
	}

	button_latched = false;
}

static void button_gpio_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	(void)k_work_reschedule(&button_debounce_work, K_MSEC(BUTTON_DEBOUNCE_MS));
}

int button_init(button_press_callback_t callback)
{
	int rc;

	if (!gpio_is_ready_dt(&button)) {
		return -ENODEV;
	}

	button_press_callback = callback;
	button_latched = false;
	k_work_init_delayable(&button_debounce_work, button_debounce_work_handler);

	rc = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (rc != 0) {
		LOG_ERR("Failed to configure button GPIO: %d", rc);
		return rc;
	}

	rc = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (rc != 0) {
		LOG_ERR("Failed to configure button interrupt: %d", rc);
		return rc;
	}

	gpio_init_callback(&button_cb_data, button_gpio_isr, BIT(button.pin));
	rc = gpio_add_callback(button.port, &button_cb_data);
	if (rc != 0) {
		LOG_ERR("Failed to add button callback: %d", rc);
		return rc;
	}

	LOG_INF("Button initialized on port=%s pin=%u", button.port->name, button.pin);
	return 0;
}
#ifndef HPADV2_RADIO_ESB_H_
#define HPADV2_RADIO_ESB_H_

#include <stdint.h>

#include "radio_identity.h"
#include "wire_protocol.h"

typedef void (*radio_esb_report_handler_t)(const macropad_report_t *report);

int radio_esb_init(const struct esb_addr_config *addr_config, uint8_t rf_channel,
		   radio_esb_report_handler_t report_handler);
int radio_esb_start(void);
int radio_esb_queue_macropad_config(const macropad_config_t *config);

#endif
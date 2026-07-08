#ifndef HPADV2_CONSUMER_ACTION_ENGINE_H_
#define HPADV2_CONSUMER_ACTION_ENGINE_H_

#include <stdbool.h>

#include "wire_protocol.h"

void consumer_action_engine_reset(void);
bool consumer_action_engine_input_activity(const macropad_report_t *report);
void consumer_action_engine_process_report(const macropad_report_t *report);

#endif

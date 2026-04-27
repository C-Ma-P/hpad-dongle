#ifndef HPADV2_BUTTON_H_
#define HPADV2_BUTTON_H_

typedef void (*button_press_callback_t)(void);

int button_init(button_press_callback_t callback);

#endif
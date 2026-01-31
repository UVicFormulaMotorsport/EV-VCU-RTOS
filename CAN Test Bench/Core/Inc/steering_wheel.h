#ifndef STEERING_WHEEL_H
#define STEERING_WHEEL_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    BTN_1 = 0,
    BTN_2,
    BTN_3,
    BTN_4,
    BTN_5,
    BTN_6,
    BTN_7,
    BTN_8,
} button_t;

typedef enum {
    PRESS, 
    RELEASE
} event_t;

typedef struct {
    button_t button;
    event_t event;
    uint32_t time_ms;
} btn_event_t;

void steering_wheel_init(uint32_t id, uint32_t timeout_ms);
void steering_wheel_update(void);
void steering_wheel_can_frame(uint32_t id, const uint8_t *data, uint8_t len);
bool steering_wheel_get_event(btn_event_t *event);
bool steering_wheel_btn_pressed(button_t btn);
bool steering_wheel_timed_out(void);

#endif /* STEERING_WHEEL_H */
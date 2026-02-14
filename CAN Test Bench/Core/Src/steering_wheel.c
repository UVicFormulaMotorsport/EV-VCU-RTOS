#include "steering_wheel.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

#define MASK_BYTE 6 // this is from ecumaster docs
#define HEART_BYTE 7 

static uint32_t can_id;
static uint32_t timeout_ms;
static uint32_t btn_mask;
static uint32_t old_mask;
static uint32_t last_time;
static bool is_dead;

static btn_event_t events[16]; //queue
static uint8_t write_pos;
static uint8_t read_pos;


// call this at startup. the id is 0x334, timeout is 200ms. so steering_wheel_init(0x334, 200) 
void steering_wheel_init(uint32_t id, uint32_t to){
    can_id  = id;
    timeout_ms = to;
    btn_mask = old_mask = 0;
    last_time = 0;
    is_dead = true;
    write_pos = read_pos = 0;
}

// call in main to chekc timeout
void steering_wheel_update(void){
    uint32_t now = HAL_GetTick();

    if(last_time == 0){
        is_dead = true;
        return;
    }

    is_dead = (now - last_time > timeout_ms);
}

void steering_wheel_can_frame(uint32_t id, const uint8_t *data, uint8_t len){
    if (id != can_id || len < 8){
        return;
    }

    uint32_t now = HAL_GetTick();
    last_time = now;

    uint8_t new_mask = data[MASK_BYTE]; 
    uint8_t changed = new_mask ^ btn_mask; //flpped bits

    if ( changed){
        for(uint8_t i =0; i < 8; i++){
            uint8_t bit = 1U << i;
            if (changed & bit){
                bool pressed = (new_mask & bit) != 0;
                event_t type = pressed ? PRESS : RELEASE;

                uint8_t next = (write_pos + 1) % 16;
                if (next != read_pos){ //queue not full
                    events[write_pos].button = (button_t)i;
                    events[write_pos].event = type;
                    events[write_pos].time_ms = now;
                    write_pos = next;
                }
            }
        }
    }
    old_mask = btn_mask;
    btn_mask = new_mask;
}

bool steering_wheel_get_event(btn_event_t *event){
    if (read_pos == write_pos){
        return false; //queue empty
    }

    *event = events[read_pos];
    read_pos = (read_pos + 1) % 16;
    return true;
}

bool steering_wheel_btn_pressed(button_t btn){
    return (btn_mask & (1U << btn)) != 0;
}

bool steering_wheel_timed_out(void){
    return is_dead;
}
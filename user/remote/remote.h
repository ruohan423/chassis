//
// Created by mot on 2026/10/13.
//
#ifndef SENTRY_CHASSIS_REMOTE_H
#define SENTRY_CHASSIS_REMOTE_H

#include <stdbool.h>
#include <stdint.h>

#define RC_FRAME_LENGTH 18u
#define RC_STICK_OFFSET 1024

/* 摇杆数据 */
typedef struct {
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int8_t  s1;
    int8_t  s2;
} Remote;

/* 鼠标数据 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t last_press_l;
    uint8_t last_press_r;
    uint8_t press_l;
    uint8_t press_r;
} Mouse;

/* 键盘数据：value 整体赋值，bits 逐位读取 */
typedef union {
    uint16_t value;
    struct {
        uint16_t W: 1;
        uint16_t S: 1;
        uint16_t A: 1;
        uint16_t D: 1;
        uint16_t Shift: 1;
        uint16_t Ctrl: 1;
        uint16_t Q: 1;
        uint16_t E: 1;
        uint16_t R: 1;
        uint16_t F: 1;
        uint16_t G: 1;
        uint16_t Z: 1;
        uint16_t X: 1;
        uint16_t C: 1;
        uint16_t V: 1;
        uint16_t B: 1;
    } bits;
} Key_t;

/* 遥控器整体数据 */
typedef struct {
    Remote rc;
    Mouse  mouse;
    Key_t  key;
    Key_t  lastkey;
} RC_Ctl_t;

/* 输入模式 */
typedef enum {
    REMOTE_INPUT    = 1,
    KEY_MOUSE_INPUT = 3,
    STOP            = 2,
} InputMode_e;

extern RC_Ctl_t rc_data;

extern volatile uint32_t remote_uart_error_count;
extern volatile uint32_t remote_frame_gap_error;
extern volatile uint32_t remote_frame_invalid_count;
extern volatile uint32_t remote_dma_overrun_count;

/* 回调函数指针类型 */
typedef void (*RemoteControlCallback)(Remote *rc);
typedef void (*MouseKeyControlCallback)(Mouse *mouse, Key_t key, Key_t lastkey);
typedef void (*StopControlCallback)(void);

/* 注册三个回调；传 NULL 使用默认空实现 */
void Remote_RegisterCallbacks(RemoteControlCallback remoteCb,MouseKeyControlCallback mouseKeyCb,StopControlCallback stopCb);

bool remote_frame_is_valid(const uint8_t *frame);
void remote_rx(unsigned char *RxMsg);
void remote_clear(void);
#endif //SENTRY_CHASSIS_REMOTE_H

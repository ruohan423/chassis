#include "remote.h"

RC_Ctl_t rc_data = {.rc = {1024, 1024, 1024, 1024, 1, 1}};

#define REMOTE_CHANNEL_MIN 200U
#define REMOTE_CHANNEL_MAX 1800U

/* 默认空实现 */
static void remote_control_default(Remote *rc) {
    (void)rc;
}

static void mouse_key_control_default(Mouse *mouse, Key_t key, Key_t lastkey) {
    (void)mouse;
    (void)key;
    (void)lastkey;
}

static void stop_control_default(void) {
}

/* 回调指针 */
static RemoteControlCallback   remote_control_callback   = remote_control_default;
static MouseKeyControlCallback mouse_key_callback = mouse_key_control_default;
static StopControlCallback     stop_callback     = stop_control_default;

void Remote_RegisterCallbacks(RemoteControlCallback remoteCb,MouseKeyControlCallback mouseKeyCb,StopControlCallback stopCb) {
    remote_control_callback   = remoteCb   ? remoteCb   : remote_control_default;
    mouse_key_callback = mouseKeyCb ? mouseKeyCb : mouse_key_control_default;
    stop_callback     = stopCb     ? stopCb     : stop_control_default;
}

void remote_clear(void) {
    rc_data.rc.ch0 = RC_STICK_OFFSET;
    rc_data.rc.ch1 = RC_STICK_OFFSET;
    rc_data.rc.ch2 = RC_STICK_OFFSET;
    rc_data.rc.ch3 = RC_STICK_OFFSET;
    rc_data.rc.s1  = 2;
    rc_data.rc.s2  = 1;
    rc_data.dial.ch4 = RC_STICK_OFFSET;

    rc_data.mouse.x = rc_data.mouse.y = rc_data.mouse.z = 0;
    rc_data.mouse.press_l = rc_data.mouse.press_r = 0;
    rc_data.mouse.last_press_l = rc_data.mouse.last_press_r = 0;

    rc_data.key.value     = 0;
    rc_data.lastkey.value = 0;
}

bool remote_frame_is_valid(const uint8_t *RxMsg)
{
    uint16_t ch0 = (uint16_t)((RxMsg[0] | (RxMsg[1] << 8)) & 0x07FFU);
    uint16_t ch1 = (uint16_t)(((RxMsg[1] >> 3) | (RxMsg[2] << 5)) & 0x07FFU);
    uint16_t ch2 = (uint16_t)(((RxMsg[2] >> 6) | (RxMsg[3] << 2) | (RxMsg[4] << 10)) & 0x07FFU);
    uint16_t ch3 = (uint16_t)(((RxMsg[4] >> 1) | (RxMsg[5] << 7)) & 0x07FFU);

    return (ch0 >= REMOTE_CHANNEL_MIN) && (ch0 <= REMOTE_CHANNEL_MAX)
        && (ch1 >= REMOTE_CHANNEL_MIN) && (ch1 <= REMOTE_CHANNEL_MAX)
        && (ch2 >= REMOTE_CHANNEL_MIN) && (ch2 <= REMOTE_CHANNEL_MAX)
        && (ch3 >= REMOTE_CHANNEL_MIN) && (ch3 <= REMOTE_CHANNEL_MAX);
}

void remote_rx(unsigned char *RxMsg) {
    /* 摇杆 */
    rc_data.rc.ch0 = (int16_t)((RxMsg[0] | (RxMsg[1] << 8)) & 0x07FF);
    rc_data.rc.ch1 = (int16_t)(((RxMsg[1] >> 3) | (RxMsg[2] << 5)) & 0x07FF);
    rc_data.rc.ch2 = (int16_t)(((RxMsg[2] >> 6) | (RxMsg[3] << 2) | (RxMsg[4] << 10)) & 0x07FF);
    rc_data.rc.ch3 = (int16_t)(((RxMsg[4] >> 1) | (RxMsg[5] << 7)) & 0x07FF);

    /* 开关 */
    rc_data.rc.s1 = (int8_t)((RxMsg[5] >> 4 & 0x000C) >> 2);
    rc_data.rc.s2 = (int8_t)(RxMsg[5] >> 4 & 0x0003);

    /* 鼠标 */
    rc_data.mouse.x = (int16_t)(RxMsg[6]  | (RxMsg[7]  << 8));
    rc_data.mouse.y = (int16_t)(RxMsg[8]  | (RxMsg[9]  << 8));
    rc_data.mouse.z = (int16_t)(RxMsg[10] | (RxMsg[11] << 8));
    rc_data.mouse.press_l = RxMsg[12];
    rc_data.mouse.press_r = RxMsg[13];

    /* 键盘：整体赋值到 union.value */
    rc_data.key.value = (uint16_t)(RxMsg[14] | (RxMsg[15] << 8));

    /* 拨轮通道：固定帧末尾两个字节为 11 位原始通道值。 */
    rc_data.dial.ch4 = (int16_t)((RxMsg[16] | (RxMsg[17] << 8)) & 0x07FFU);

    /* 模式分发 */
    switch (rc_data.rc.s2) {
        case REMOTE_INPUT:
            remote_control_callback(&rc_data.rc);
            break;
        case KEY_MOUSE_INPUT:
            mouse_key_callback(&rc_data.mouse, rc_data.key, rc_data.lastkey);
            break;
        case STOP:
            stop_callback();
            break;
        default:
            break;
    }
    /* 保存上一帧状态 */
    rc_data.mouse.last_press_l = rc_data.mouse.press_l;
    rc_data.mouse.last_press_r = rc_data.mouse.press_r;
    rc_data.lastkey = rc_data.key;
}

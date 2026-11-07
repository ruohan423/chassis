#ifndef SENTRY_CHASSIS_WATCH_TASK_H
#define SENTRY_CHASSIS_WATCH_TASK_H

void Watchdog_Task(void *argument);
void swdg_feed_remote(void);
void swdg_feed_imu(void);
void swdg_feed_usb(void);
void swdg_feed_judge(void);
void swdg_feed_op(void);
void swdg_feed_chassis_1(void);
void swdg_feed_chassis_2(void);
void swdg_feed_chassis_3(void);
void swdg_feed_chassis_4(void);
void swdg_feed_chassis_track(void);
void swdg_feed_yaw(void);

#endif

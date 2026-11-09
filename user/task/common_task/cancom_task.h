#ifndef SENTRY_CHASSIS_CANCOM_TASK_H
#define SENTRY_CHASSIS_CANCOM_TASK_H

#define CANCOM_TASK_REMOTE_FRAME_FLAG  0x01U

void CanCom_Task(void *argument);
void CanCom_TaskNotifyRemoteFrame(void);

#endif //SENTRY_CHASSIS_CANCOM_TASK_H
